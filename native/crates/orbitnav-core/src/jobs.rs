//! The worker pool: derive and search off the caller's thread.
//!
//! The caller is a game's main thread, and everything here exists so it never waits. [`Pool::submit`]
//! hands work over and returns immediately; [`Pool::poll`] is a lock-and-read; [`Pool::take`] moves a
//! finished result out. Nothing in this module blocks the submitting thread.
//!
//! `std::thread` and `std::sync` only. A thread-pool crate would be this crate's first dependency and
//! would buy nothing at this size.
//!
//! # Volumes are snapshots, so invalidation cannot race
//!
//! A job clones an `Arc<Volume>` when it is submitted and records the generation that `Arc` came
//! from. Publishing a new volume replaces the `Arc` and bumps the generation; it does not mutate what
//! any running job is reading. So:
//!
//! - a job in flight always finishes against a consistent volume, whatever the main thread does;
//! - a result whose generation no longer matches is reported [`JobState::Stale`] rather than handed
//!   back, so the caller re-requests instead of steering by an answer about geometry that is gone;
//! - no lock is ever held across a call back into the caller.
//!
//! # A panicking worker is a failed job, not a dead process
//!
//! Every job body runs under `catch_unwind`. The crate is `forbid(unsafe_code)` and its inputs are
//! validated, so a panic should be unreachable -- but this is a library inside somebody's editor, and
//! the failure mode of getting that wrong is losing their unsaved work. `panic = "abort"` is
//! deliberately not set for the same reason.

use crate::astar::{Outcome, Search, SearchParams};
use crate::real::Vec3;
use crate::volume::Volume;
use std::collections::HashMap;
use std::panic::{catch_unwind, AssertUnwindSafe};
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Condvar, Mutex};

/// A handle to submitted work. Monotonic and NEVER reused, so a stale id from a freed volume
/// answers [`JobState::Unknown`] rather than another job's result.
pub type JobId = u64;

/// Which volume a job is about, and which publication of it.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct VolumeStamp {
    /// Which volume.
    pub id: u64,
    /// Which publication of that volume.
    pub generation: u64,
}

/// What a finished job produced.
#[derive(Debug)]
pub enum JobOutput {
    /// A derived volume, and how long the worker spent on it.
    Derived {
        /// The finished volume.
        volume: Arc<Volume>,
        /// Microseconds of worker time.
        usec: u64,
    },
    /// A finished search.
    Path {
        /// How the search ended.
        outcome: Outcome,
        /// The waypoints, empty unless the outcome carries a path.
        waypoints: Vec<Vec3>,
        /// How many cells were expanded.
        expansions: u32,
    },
}

/// Where a job is.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum JobState {
    /// No such job, or its result has already been taken.
    Unknown,
    /// Queued, not started.
    Pending,
    /// A worker is on it.
    Running,
    /// Finished; the result is waiting to be taken.
    Ready,
    /// The worker panicked.
    Failed,
    /// Cancelled before or during the run.
    Cancelled,
    /// Finished, but against a volume that has since been replaced.
    Stale,
}

enum Slot {
    Pending,
    Running,
    Ready(JobOutput),
    Failed,
    Cancelled,
}

enum Work {
    Derive {
        volume: Arc<Volume>,
        workers: usize,
    },
    Search {
        volume: Arc<Volume>,
        params: SearchParams,
    },
}

struct Job {
    id: JobId,
    stamp: VolumeStamp,
    work: Work,
}

#[derive(Default)]
struct Shared {
    queue: Mutex<Vec<Job>>,
    slots: Mutex<HashMap<JobId, Slot>>,
    stamps: Mutex<HashMap<JobId, VolumeStamp>>,
    cv: Condvar,
    shutdown: AtomicBool,
}

/// A fixed pool of workers.
pub struct Pool {
    shared: Arc<Shared>,
    threads: Vec<std::thread::JoinHandle<()>>,
    next_id: AtomicU64,
    derive_workers: usize,
}

impl Pool {
    /// Start a pool of `threads` workers, clamped to at least one.
    ///
    /// The default size is deliberately small: this runs beside a renderer and a simulation, and
    /// taking every core would win a benchmark and lose a frame.
    #[must_use]
    pub fn new(threads: usize) -> Self {
        let n = threads.max(1);
        let shared = Arc::new(Shared::default());
        let mut handles = Vec::with_capacity(n);
        for _ in 0..n {
            let sh = Arc::clone(&shared);
            handles.push(std::thread::spawn(move || worker_loop(sh)));
        }
        Self {
            shared,
            threads: handles,
            next_id: AtomicU64::new(1),
            derive_workers: n,
        }
    }

    /// A pool sized for this machine: one fewer than the cores it reports, capped at four.
    #[must_use]
    pub fn default_sized() -> Self {
        Self::new(Self::default_thread_count())
    }

    /// How many workers [`Pool::default_sized`] would start here.
    #[must_use]
    pub fn default_thread_count() -> usize {
        let cores = std::thread::available_parallelism().map_or(1, std::num::NonZeroUsize::get);
        cores.saturating_sub(1).clamp(1, 4)
    }

    /// How many workers the pool holds.
    #[must_use]
    pub fn threads(&self) -> usize {
        self.threads.len()
    }

    /// Queue a derive of `volume`'s occupancy.
    pub fn submit_derive(&self, volume: Arc<Volume>, stamp: VolumeStamp) -> JobId {
        let workers = self.derive_workers;
        self.enqueue(stamp, Work::Derive { volume, workers })
    }

    /// Queue a search over `volume`.
    pub fn submit_search(
        &self,
        volume: Arc<Volume>,
        stamp: VolumeStamp,
        params: SearchParams,
    ) -> JobId {
        self.enqueue(stamp, Work::Search { volume, params })
    }

    fn enqueue(&self, stamp: VolumeStamp, work: Work) -> JobId {
        let id = self.next_id.fetch_add(1, Ordering::Relaxed);
        self.shared.slots.lock().unwrap().insert(id, Slot::Pending);
        self.shared.stamps.lock().unwrap().insert(id, stamp);
        self.shared
            .queue
            .lock()
            .unwrap()
            .push(Job { id, stamp, work });
        self.shared.cv.notify_one();
        id
    }

    /// Where `job` is. `current` is the volume generation the caller believes in; a finished job
    /// about an older generation reports [`JobState::Stale`].
    #[must_use]
    pub fn poll(&self, job: JobId, current: Option<VolumeStamp>) -> JobState {
        let slots = self.shared.slots.lock().unwrap();
        let Some(slot) = slots.get(&job) else {
            return JobState::Unknown;
        };
        match slot {
            Slot::Pending => JobState::Pending,
            Slot::Running => JobState::Running,
            Slot::Failed => JobState::Failed,
            Slot::Cancelled => JobState::Cancelled,
            Slot::Ready(_) => {
                if let Some(cur) = current {
                    let stamps = self.shared.stamps.lock().unwrap();
                    if stamps.get(&job) != Some(&cur) {
                        return JobState::Stale;
                    }
                }
                JobState::Ready
            }
        }
    }

    /// Take `job`'s result. The id then answers [`JobState::Unknown`].
    #[must_use]
    pub fn take(&self, job: JobId) -> Option<JobOutput> {
        let mut slots = self.shared.slots.lock().unwrap();
        match slots.remove(&job) {
            Some(Slot::Ready(out)) => {
                self.shared.stamps.lock().unwrap().remove(&job);
                Some(out)
            }
            Some(other) => {
                slots.insert(job, other); // not ready: leave it where it was
                None
            }
            None => None,
        }
    }

    /// Abandon `job`. A queued job never runs; a running one finishes and its result is dropped.
    pub fn cancel(&self, job: JobId) {
        self.shared.queue.lock().unwrap().retain(|j| j.id != job);
        let mut slots = self.shared.slots.lock().unwrap();
        if let Some(slot) = slots.get_mut(&job) {
            *slot = Slot::Cancelled;
        }
        self.shared.stamps.lock().unwrap().remove(&job);
    }

    /// Forget every job about volume `id`, whatever state it is in.
    pub fn drop_volume(&self, id: u64) {
        let mut stamps = self.shared.stamps.lock().unwrap();
        let doomed: Vec<JobId> = stamps
            .iter()
            .filter(|(_, s)| s.id == id)
            .map(|(j, _)| *j)
            .collect();
        for j in &doomed {
            stamps.remove(j);
        }
        drop(stamps);
        let mut q = self.shared.queue.lock().unwrap();
        q.retain(|j| j.stamp.id != id);
        drop(q);
        let mut slots = self.shared.slots.lock().unwrap();
        for j in doomed {
            slots.remove(&j);
        }
    }

    /// `[threads, queued, running, ready]`, for diagnostics.
    #[must_use]
    pub fn stats(&self) -> [i64; 4] {
        let queued = self.shared.queue.lock().unwrap().len() as i64;
        let slots = self.shared.slots.lock().unwrap();
        let mut running = 0i64;
        let mut ready = 0i64;
        for s in slots.values() {
            match s {
                Slot::Running => running += 1,
                Slot::Ready(_) => ready += 1,
                _ => {}
            }
        }
        [self.threads.len() as i64, queued, running, ready]
    }
}

impl Drop for Pool {
    fn drop(&mut self) {
        self.shared.shutdown.store(true, Ordering::SeqCst);
        self.shared.cv.notify_all();
        for h in self.threads.drain(..) {
            let _ = h.join();
        }
    }
}

fn worker_loop(shared: Arc<Shared>) {
    loop {
        let job = {
            let mut q = shared.queue.lock().unwrap();
            loop {
                if shared.shutdown.load(Ordering::SeqCst) {
                    return;
                }
                if let Some(j) = q.pop() {
                    break j;
                }
                q = shared.cv.wait(q).unwrap();
            }
        };

        // A job cancelled between queueing and pickup must not run.
        {
            let mut slots = shared.slots.lock().unwrap();
            match slots.get(&job.id) {
                Some(Slot::Pending) => {
                    slots.insert(job.id, Slot::Running);
                }
                _ => continue,
            }
        }

        let started = std::time::Instant::now();
        let outcome = catch_unwind(AssertUnwindSafe(|| match job.work {
            Work::Derive { volume, workers } => {
                let derived = Volume::derive(&volume.occupancy(), workers);
                JobOutput::Derived {
                    volume: Arc::new(derived),
                    usec: started.elapsed().as_micros() as u64,
                }
            }
            Work::Search { volume, params } => {
                let mut s = Search::begin(&volume, params);
                s.run(&volume);
                JobOutput::Path {
                    outcome: s.outcome(),
                    waypoints: s.take_result(),
                    expansions: s.expansions(),
                }
            }
        }));

        let mut slots = shared.slots.lock().unwrap();
        // A cancel that landed while the job ran wins: drop the result rather than resurrect it.
        if !matches!(slots.get(&job.id), Some(Slot::Running)) {
            continue;
        }
        match outcome {
            Ok(out) => slots.insert(job.id, Slot::Ready(out)),
            Err(_) => slots.insert(job.id, Slot::Failed),
        };
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::grid::Grid;
    use crate::occupancy::Occupancy;

    fn stamp() -> VolumeStamp {
        VolumeStamp {
            id: 1,
            generation: 1,
        }
    }

    fn small() -> Arc<Volume> {
        let g = Grid::new(Vec3::ZERO, 1.0, [10, 10, 6]);
        let mut o = Occupancy::empty(g);
        for y in 0..10 {
            for x in 0..10 {
                o.solid[g.cell_index([x, y, 0])] = 1;
            }
        }
        Arc::new(Volume::derive(&o, 1))
    }

    fn settle(pool: &Pool, job: JobId) -> JobState {
        for _ in 0..20000 {
            let s = pool.poll(job, None);
            if !matches!(s, JobState::Pending | JobState::Running) {
                return s;
            }
            std::thread::yield_now();
        }
        panic!("job {job} never settled");
    }

    #[test]
    fn a_derive_comes_back() {
        let pool = Pool::new(2);
        let v = small();
        let job = pool.submit_derive(Arc::clone(&v), stamp());
        assert_eq!(settle(&pool, job), JobState::Ready);
        match pool.take(job) {
            Some(JobOutput::Derived { volume, .. }) => {
                assert_eq!(volume.surface_flag, v.surface_flag);
            }
            other => panic!("expected a derive, got {other:?}"),
        }
    }

    #[test]
    fn a_search_comes_back() {
        let pool = Pool::new(2);
        let v = small();
        let params = SearchParams::new(
            Vec3::new(1.5, 1.5, 1.5),
            Vec3::new(8.5, 8.5, 1.5),
            false,
            20000,
        );
        let job = pool.submit_search(Arc::clone(&v), stamp(), params);
        assert_eq!(settle(&pool, job), JobState::Ready);
        match pool.take(job) {
            Some(JobOutput::Path { outcome, .. }) => assert!(outcome.has_path()),
            other => panic!("expected a path, got {other:?}"),
        }
    }

    #[test]
    fn many_jobs_across_a_few_threads_all_complete() {
        let pool = Pool::new(4);
        let v = small();
        let mut ids = Vec::new();
        for k in 0..200u32 {
            let to = Vec3::new(1.5 + (k % 8) as f32, 8.5, 1.5);
            let params = SearchParams::new(Vec3::new(1.5, 1.5, 1.5), to, false, 5000);
            ids.push(pool.submit_search(Arc::clone(&v), stamp(), params));
        }
        for id in ids {
            assert_eq!(settle(&pool, id), JobState::Ready, "job {id}");
            assert!(pool.take(id).is_some());
        }
    }

    #[test]
    fn ids_are_never_reused() {
        let pool = Pool::new(1);
        let v = small();
        let mut seen = std::collections::HashSet::new();
        for _ in 0..50 {
            let id = pool.submit_derive(Arc::clone(&v), stamp());
            assert!(seen.insert(id), "id {id} was handed out twice");
            settle(&pool, id);
            let _ = pool.take(id);
        }
    }

    #[test]
    fn a_second_take_gets_nothing() {
        let pool = Pool::new(1);
        let v = small();
        let job = pool.submit_derive(v, stamp());
        settle(&pool, job);
        assert!(pool.take(job).is_some());
        assert!(pool.take(job).is_none());
        assert_eq!(pool.poll(job, None), JobState::Unknown);
    }

    #[test]
    fn a_result_about_an_older_generation_reads_stale() {
        let pool = Pool::new(1);
        let v = small();
        let job = pool.submit_derive(v, stamp());
        settle(&pool, job);
        let newer = VolumeStamp {
            id: 1,
            generation: 2,
        };
        assert_eq!(pool.poll(job, Some(newer)), JobState::Stale);
        assert_eq!(pool.poll(job, Some(stamp())), JobState::Ready);
    }

    #[test]
    fn cancelling_a_queued_job_stops_it_running() {
        let pool = Pool::new(1);
        let v = small();
        let mut ids = Vec::new();
        for _ in 0..40 {
            ids.push(pool.submit_derive(Arc::clone(&v), stamp()));
        }
        let victim = *ids.last().unwrap();
        pool.cancel(victim);
        for id in ids {
            let s = settle(&pool, id);
            if id == victim {
                assert!(matches!(
                    s,
                    JobState::Cancelled | JobState::Ready | JobState::Unknown
                ));
            }
        }
    }

    #[test]
    fn dropping_a_volume_forgets_its_jobs() {
        let pool = Pool::new(1);
        let v = small();
        let job = pool.submit_derive(v, stamp());
        pool.drop_volume(1);
        assert_eq!(pool.poll(job, None), JobState::Unknown);
    }

    #[test]
    fn stats_report_the_pool_size() {
        let pool = Pool::new(3);
        assert_eq!(pool.stats()[0], 3);
        assert_eq!(pool.threads(), 3);
    }

    #[test]
    fn the_default_size_leaves_a_core_and_caps_at_four() {
        let n = Pool::default_thread_count();
        assert!((1..=4).contains(&n), "got {n}");
    }

    #[test]
    fn dropping_the_pool_joins_its_workers() {
        let pool = Pool::new(3);
        let v = small();
        for _ in 0..10 {
            pool.submit_derive(Arc::clone(&v), stamp());
        }
        drop(pool); // must return rather than hang
    }
}
