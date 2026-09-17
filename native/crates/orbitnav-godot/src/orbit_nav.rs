//! The `OrbitNav` node: the whole class surface.
//!
//! One registered class. Volumes and jobs are `i64` handles this node owns, rather than classes of
//! their own -- an extra class would join the facade's grep boundary and add refcount traffic per
//! call, and an integer handle gives one lifetime owner and one crossing.
//!
//! The node registers no `_process`: polling is the caller's, on the caller's cadence.

use crate::convert::{
    affine_in, bytes_in, bytes_out, cell_in, cell_out, ints_out, points_in, points_out, v3_in,
    v3_out,
};
use godot::classes::Node;
use godot::prelude::*;
use orbitnav_core::astar::{Outcome, SearchParams};
use orbitnav_core::grid::{Grid, NO_CELL};
use orbitnav_core::jobs::{JobOutput, JobState, Pool, VolumeStamp, Voxelization};
use orbitnav_core::los::has_los;
use orbitnav_core::occupancy::Occupancy;
use orbitnav_core::smooth::smooth_path;
use orbitnav_core::snap::nearest_free_cell;
use orbitnav_core::volume::Volume;
use orbitnav_core::voxelize::{Region, Sides, Winding};
use std::collections::HashMap;
use std::sync::Arc;

/// One volume the caller holds a handle to.
struct Entry {
    /// The published volume. Replaced wholesale, never mutated, so a job reading an older `Arc`
    /// finishes against a consistent snapshot.
    volume: Arc<Volume>,
    /// Bumped on every publication, so a finished job about an older one reads as stale.
    generation: u64,
    /// Whether the derived arrays are real, or still the zeroes `create_volume` made.
    built: bool,
    /// What the last derive cost the worker, in microseconds.
    derive_usec: i64,
    /// What the last voxelize cost the worker, in microseconds.
    voxelize_usec: i64,
    /// Geometry staged since `begin_geometry`, waiting for `voxelize_async`.
    staged: Option<Voxelization>,
}

/// Off-thread voxel navigation: occupancy classification, connected components, line of sight and
/// pathfinding over a uniform grid.
#[derive(GodotClass)]
#[class(base = Node)]
pub struct OrbitNav {
    base: Base<Node>,
    pool: Option<Pool>,
    volumes: HashMap<i64, Entry>,
    next_volume: i64,
    /// How many workers the pool should hold. 0 asks for a size chosen from the machine.
    worker_threads: i64,
    /// How the most recently taken search ended. Read through [`Self::last_outcome`].
    last_outcome_code: i64,
    /// How many cells the most recently taken search expanded.
    last_expansions: i64,
}

#[godot_api]
impl INode for OrbitNav {
    fn init(base: Base<Node>) -> Self {
        Self {
            base,
            pool: None,
            volumes: HashMap::new(),
            next_volume: 1,
            worker_threads: 0,
            last_outcome_code: Outcome::Pending as i64,
            last_expansions: 0,
        }
    }
}

impl OrbitNav {
    /// The pool, started on first use so a project that never navigates pays for no threads.
    fn pool(&mut self) -> &Pool {
        if self.pool.is_none() {
            let n = if self.worker_threads > 0 {
                self.worker_threads as usize
            } else {
                Pool::default_thread_count()
            };
            self.pool = Some(Pool::new(n));
        }
        self.pool.as_ref().expect("pool was just created")
    }

    fn entry(&self, vol: i64) -> Option<&Entry> {
        self.volumes.get(&vol)
    }

    fn stamp(&self, vol: i64) -> Option<VolumeStamp> {
        self.entry(vol).map(|e| VolumeStamp {
            id: vol as u64,
            generation: e.generation,
        })
    }

    /// The geometry being staged for `vol`, or `None` when `begin_geometry` has not been called.
    fn staging(&mut self, vol: i64) -> Option<&mut Voxelization> {
        self.volumes.get_mut(&vol).and_then(|e| e.staged.as_mut())
    }

    /// A built volume, or `None`. Every query answers its own inert value when this is `None`.
    fn built(&self, vol: i64) -> Option<&Arc<Volume>> {
        self.entry(vol).filter(|e| e.built).map(|e| &e.volume)
    }
}

#[godot_api]
impl OrbitNav {
    // --- job states, as returned by `job_state` -----------------------------------------------

    /// No such job, or its result has already been taken.
    #[constant]
    const JOB_UNKNOWN: i64 = 0;
    /// Queued, not started.
    #[constant]
    const JOB_PENDING: i64 = 1;
    /// A worker is on it.
    #[constant]
    const JOB_RUNNING: i64 = 2;
    /// Finished; the result is waiting to be taken.
    #[constant]
    const JOB_READY: i64 = 3;
    /// The worker panicked.
    #[constant]
    const JOB_FAILED: i64 = 4;
    /// Cancelled.
    #[constant]
    const JOB_CANCELLED: i64 = 5;
    /// Finished, but the volume has been republished since. Re-request.
    #[constant]
    const JOB_STALE: i64 = 6;

    // --- search outcomes, as returned by `last_outcome` ---------------------------------------

    /// Still running.
    #[constant]
    const OUTCOME_PENDING: i64 = 0;
    /// A route was found.
    #[constant]
    const OUTCOME_OK: i64 = 1;
    /// The straight line was already clear.
    #[constant]
    const OUTCOME_DIRECT: i64 = 2;
    /// The two ends are in different components. No budget will help.
    #[constant]
    const OUTCOME_UNREACHABLE: i64 = 3;
    /// The expansion cap stopped a live frontier. More budget might help.
    #[constant]
    const OUTCOME_CAPPED: i64 = 4;
    /// An end had no usable cell within its snap radius.
    #[constant]
    const OUTCOME_UNSNAPPABLE: i64 = 5;
    /// The volume carries no derived data.
    #[constant]
    const OUTCOME_UNBUILT: i64 = 6;
    /// The volume was republished while the search was in flight.
    #[constant]
    const OUTCOME_STALE: i64 = 7;
    /// The search could not run.
    #[constant]
    const OUTCOME_FAILED: i64 = 8;

    // --- lifecycle ---------------------------------------------------------------------------

    /// Create an empty volume of `dims` cells of `cell_size` metres from world-space `origin`.
    ///
    /// Returns its handle, or 0 when the extents are not all positive.
    #[func]
    fn create_volume(&mut self, origin: Vector3, cell_size: f64, dims: Vector3i) -> i64 {
        let grid = Grid::new(v3_in(origin), cell_size, cell_in(dims));
        if grid.cell_count() == 0 {
            return 0;
        }
        let id = self.next_volume;
        self.next_volume += 1;
        let occ = Occupancy::empty(grid);
        self.volumes.insert(
            id,
            Entry {
                volume: Arc::new(Volume::derive(&occ, 1)),
                generation: 1,
                built: false,
                derive_usec: 0,
                voxelize_usec: 0,
                staged: None,
            },
        );
        id
    }

    /// Release a volume and forget every job about it. Freeing twice is not an error.
    #[func]
    fn free_volume(&mut self, vol: i64) {
        if self.volumes.remove(&vol).is_some() && self.pool.is_some() {
            self.pool().drop_volume(vol as u64);
        }
    }

    /// Which publication of this volume is current. A job's result is only valid against the
    /// generation it was submitted under.
    #[func]
    fn volume_generation(&self, vol: i64) -> i64 {
        self.entry(vol).map_or(0, |e| e.generation as i64)
    }

    /// Whether the volume exists and carries derived data.
    #[func]
    fn is_built(&self, vol: i64) -> bool {
        self.built(vol).is_some()
    }

    /// How many cells the volume holds.
    #[func]
    fn cell_count(&self, vol: i64) -> i64 {
        self.entry(vol).map_or(0, |e| e.volume.cell_count() as i64)
    }

    // --- derive ------------------------------------------------------------------------------

    /// Replace the volume's occupancy: one byte per cell, non-zero meaning solid.
    ///
    /// Refused unless the array is exactly one byte per cell. Publishing new occupancy bumps the
    /// generation, so any derive already in flight becomes stale rather than being applied on top.
    #[func]
    fn upload_occupancy(&mut self, vol: i64, solid: PackedByteArray) -> bool {
        let Some(entry) = self.volumes.get_mut(&vol) else {
            return false;
        };
        let grid = entry.volume.grid;
        let Some(occ) = Occupancy::new(grid, bytes_in(&solid)) else {
            return false;
        };
        let mut next = Volume::derive(&Occupancy::empty(grid), 1);
        next.solid = occ.solid;
        entry.volume = Arc::new(next);
        entry.generation += 1;
        entry.built = false;
        entry.derive_usec = 0;
        true
    }

    /// Queue the derive on a worker. Returns a job handle, or 0 when the volume is unknown.
    ///
    /// Poll it with [`Self::job_state`]; when it reads `JOB_READY`, [`Self::apply_derive`] installs
    /// the result.
    #[func]
    fn derive_async(&mut self, vol: i64) -> i64 {
        let Some(stamp) = self.stamp(vol) else {
            return 0;
        };
        let volume = Arc::clone(&self.volumes[&vol].volume);
        self.pool().submit_derive(volume, stamp) as i64
    }

    /// Install a finished derive. True when the job was ready and about the current generation.
    #[func]
    fn apply_derive(&mut self, vol: i64, job: i64) -> bool {
        let Some(stamp) = self.stamp(vol) else {
            return false;
        };
        if self.pool().poll(job as u64, Some(stamp)) != JobState::Ready {
            return false;
        }
        let Some(JobOutput::Derived {
            volume,
            usec,
            voxelize_usec,
        }) = self.pool().take(job as u64)
        else {
            return false;
        };
        let Some(entry) = self.volumes.get_mut(&vol) else {
            return false;
        };
        entry.volume = volume;
        entry.built = true;
        entry.derive_usec = usec as i64;
        entry.voxelize_usec = voxelize_usec as i64;
        true
    }

    /// Derive on the calling thread, for tests and tools. True on success.
    #[func]
    fn derive_blocking(&mut self, vol: i64) -> bool {
        let Some(entry) = self.volumes.get(&vol) else {
            return false;
        };
        let occ = entry.volume.occupancy();
        let started = std::time::Instant::now();
        let derived = Volume::derive(&occ, 1);
        let usec = started.elapsed().as_micros() as i64;
        let Some(entry) = self.volumes.get_mut(&vol) else {
            return false;
        };
        entry.volume = Arc::new(derived);
        entry.built = true;
        entry.derive_usec = usec;
        true
    }

    /// What the last derive cost the worker, in microseconds.
    #[func]
    fn derive_usec(&self, vol: i64) -> i64 {
        self.entry(vol).map_or(0, |e| e.derive_usec)
    }

    // --- voxelize ----------------------------------------------------------------------------

    /// Start staging geometry for [`Self::voxelize_async`], dropping anything staged before.
    ///
    /// `regions` are spheres (`xyz` centre, `w` radius) bounding where geometry may be: a cell whose centre
    /// is outside every one stays free. Empty places no restriction. `cell_rounding` rounds the cell box's
    /// edges and corners by that radius. False when the volume is unknown.
    #[func]
    fn begin_geometry(
        &mut self,
        vol: i64,
        regions: PackedVector4Array,
        cell_rounding: f64,
    ) -> bool {
        let Some(entry) = self.volumes.get_mut(&vol) else {
            return false;
        };
        let regions = regions
            .as_slice()
            .iter()
            .map(|r| Region {
                centre: orbitnav_core::real::Vec3::new(r.x, r.y, r.z),
                radius: r.w,
            })
            .collect();
        entry.staged = Some(Voxelization {
            regions,
            cell_rounding,
            ..Voxelization::default()
        });
        true
    }

    /// Stage a triangle mesh, in the order a `ConcavePolygonShape3D` stores its faces: three vertices per
    /// triangle, clockwise when seen from the front. `transform` places it in world space.
    ///
    /// A one-sided mesh claims only cells whose centre is in front of a triangle, which is what a physics
    /// engine that ignores back faces reports; `double_sided` claims both sides. Returns the triangle
    /// count staged, or -1 when nothing is being staged or `faces` is not whole triangles.
    #[func]
    fn add_faces(
        &mut self,
        vol: i64,
        faces: PackedVector3Array,
        transform: Transform3D,
        double_sided: bool,
    ) -> i64 {
        let Some(staged) = self.staging(vol) else {
            return -1;
        };
        let slice = faces.as_slice();
        if !slice.len().is_multiple_of(3) {
            return -1;
        }
        let sides = if double_sided {
            Sides::Both
        } else {
            Sides::Front(Winding::Clockwise)
        };
        staged
            .geometry
            .add_mesh(points_in(&faces), affine_in(transform), sides);
        (slice.len() / 3) as i64
    }

    /// Stage a box of full extents `size`, its edges and corners rounded by `rounding`. False when nothing
    /// is being staged.
    #[func]
    fn add_box(&mut self, vol: i64, size: Vector3, transform: Transform3D, rounding: f64) -> bool {
        let Some(staged) = self.staging(vol) else {
            return false;
        };
        let half = [
            f64::from(size.x) * 0.5,
            f64::from(size.y) * 0.5,
            f64::from(size.z) * 0.5,
        ];
        staged
            .geometry
            .add_box(half, rounding, affine_in(transform));
        true
    }

    /// Stage a sphere. False when nothing is being staged.
    #[func]
    fn add_sphere(&mut self, vol: i64, radius: f64, transform: Transform3D) -> bool {
        let Some(staged) = self.staging(vol) else {
            return false;
        };
        staged.geometry.add_sphere(radius, affine_in(transform));
        true
    }

    /// Stage a capsule along local y. `height` is the full height, caps included, as `CapsuleShape3D`
    /// states it. False when nothing is being staged.
    #[func]
    fn add_capsule(&mut self, vol: i64, radius: f64, height: f64, transform: Transform3D) -> bool {
        let Some(staged) = self.staging(vol) else {
            return false;
        };
        staged
            .geometry
            .add_capsule(radius, height * 0.5 - radius, affine_in(transform));
        true
    }

    /// Stage a cylinder along local y of full `height`, its rims rounded by `rounding`. False when nothing
    /// is being staged.
    #[func]
    fn add_cylinder(
        &mut self,
        vol: i64,
        radius: f64,
        height: f64,
        transform: Transform3D,
        rounding: f64,
    ) -> bool {
        let Some(staged) = self.staging(vol) else {
            return false;
        };
        staged
            .geometry
            .add_cylinder(radius, height * 0.5, rounding, affine_in(transform));
        true
    }

    /// Stage the convex hull of `points`. Not rounded. False when nothing is being staged.
    #[func]
    fn add_convex(&mut self, vol: i64, points: PackedVector3Array, transform: Transform3D) -> bool {
        let Some(staged) = self.staging(vol) else {
            return false;
        };
        staged
            .geometry
            .add_hull(&points_in(&points), affine_in(transform));
        true
    }

    /// Queue a voxelize of the staged geometry, followed by a derive, on a worker. Returns a job handle,
    /// or 0 when the volume is unknown or nothing was staged.
    ///
    /// Takes the staged geometry and publishes a new generation, exactly as an occupancy upload does, so a
    /// derive already in flight reads stale. Poll the job with [`Self::job_state`] and install it with
    /// [`Self::apply_derive`]; [`Self::solid`] then reads the occupancy it produced.
    #[func]
    fn voxelize_async(&mut self, vol: i64) -> i64 {
        let Some(entry) = self.volumes.get_mut(&vol) else {
            return 0;
        };
        let Some(input) = entry.staged.take() else {
            return 0;
        };
        let grid = entry.volume.grid;
        entry.volume = Arc::new(Volume::derive(&Occupancy::empty(grid), 1));
        entry.generation += 1;
        entry.built = false;
        entry.derive_usec = 0;
        entry.voxelize_usec = 0;
        let stamp = VolumeStamp {
            id: vol as u64,
            generation: entry.generation,
        };
        self.pool().submit_voxelize(grid, input, stamp) as i64
    }

    /// What the last voxelize cost the worker, in microseconds. Zero after a plain derive.
    #[func]
    fn voxelize_usec(&self, vol: i64) -> i64 {
        self.entry(vol).map_or(0, |e| e.voxelize_usec)
    }

    // --- bulk read-back ----------------------------------------------------------------------

    /// The occupancy array, one byte per cell.
    #[func]
    fn solid(&self, vol: i64) -> PackedByteArray {
        self.entry(vol)
            .map_or_else(PackedByteArray::new, |e| bytes_out(&e.volume.solid))
    }

    /// The per-cell mask of solid axis neighbours.
    #[func]
    fn surface_bits(&self, vol: i64) -> PackedByteArray {
        self.built(vol)
            .map_or_else(PackedByteArray::new, |v| bytes_out(&v.surface_bits))
    }

    /// The per-cell count of solid axis neighbours.
    #[func]
    fn solid_sides(&self, vol: i64) -> PackedByteArray {
        self.built(vol)
            .map_or_else(PackedByteArray::new, |v| bytes_out(&v.solid_sides))
    }

    /// Which free cells touch solid anywhere in their 26-neighbourhood.
    #[func]
    fn surface_flag(&self, vol: i64) -> PackedByteArray {
        self.built(vol)
            .map_or_else(PackedByteArray::new, |v| bytes_out(&v.surface_flag))
    }

    /// Free-graph component labels, -1 outside the graph.
    #[func]
    fn comp_free(&self, vol: i64) -> PackedInt32Array {
        self.built(vol)
            .map_or_else(PackedInt32Array::new, |v| ints_out(&v.comp_free))
    }

    /// Surface-graph component labels, -1 outside the graph.
    #[func]
    fn comp_surf(&self, vol: i64) -> PackedInt32Array {
        self.built(vol)
            .map_or_else(PackedInt32Array::new, |v| ints_out(&v.comp_surf))
    }

    // --- queries -----------------------------------------------------------------------------

    /// The usable cell nearest `p` within `max_r` cells, or `(-1, -1, -1)`.
    #[func]
    fn nearest_free_cell(&self, vol: i64, p: Vector3, surface_only: bool, max_r: i64) -> Vector3i {
        let Some(v) = self.built(vol) else {
            return cell_out(NO_CELL);
        };
        cell_out(nearest_free_cell(v, v3_in(p), surface_only, max_r as i32))
    }

    /// Whether the straight segment `a` -> `b` crosses only usable cells.
    #[func]
    fn has_los(&self, vol: i64, a: Vector3, b: Vector3, surface_only: bool) -> bool {
        let Some(v) = self.built(vol) else {
            return false;
        };
        has_los(v, v3_in(a), v3_in(b), surface_only)
    }

    /// Collapse `raw` into the fewest waypoints reachable in sequence from `from_p`.
    #[func]
    fn smooth_path(
        &self,
        vol: i64,
        from_p: Vector3,
        raw: PackedVector3Array,
        surface_only: bool,
    ) -> PackedVector3Array {
        let Some(v) = self.built(vol) else { return raw };
        points_out(&smooth_path(
            v,
            v3_in(from_p),
            &points_in(&raw),
            surface_only,
        ))
    }

    /// The component label at `p`, or -1.
    #[func]
    fn component_at(&self, vol: i64, p: Vector3, surface_only: bool) -> i64 {
        let Some(v) = self.built(vol) else { return -1 };
        i64::from(v.component_at(v3_in(p), surface_only))
    }

    /// A unit vector away from the solid a surface cell touches, or zero.
    #[func]
    fn surface_normal_hint(&self, vol: i64, c: Vector3i) -> Vector3 {
        let Some(v) = self.built(vol) else {
            return Vector3::ZERO;
        };
        v3_out(v.surface_normal_hint(cell_in(c)))
    }

    /// A cell of `comp`, selected by scanning forward from `draw` and wrapping.
    ///
    /// `draw` comes from the caller's own generator. This deliberately does not reproduce any
    /// particular generator: every member is reachable and the result is a pure function of `draw`.
    #[func]
    fn random_cell_in_component(
        &self,
        vol: i64,
        comp: i64,
        surface_only: bool,
        draw: i64,
    ) -> Vector3i {
        let Some(v) = self.built(vol) else {
            return cell_out(NO_CELL);
        };
        cell_out(v.random_cell_in_component_from(comp as i32, surface_only, draw as u64))
    }

    /// The point where the segment from `inside` toward `toward` leaves the volume, inset one cell.
    #[func]
    fn exit_point(&self, vol: i64, inside: Vector3, toward: Vector3) -> Vector3 {
        let Some(e) = self.entry(vol) else {
            return toward;
        };
        v3_out(e.volume.exit_point(v3_in(inside), v3_in(toward)))
    }

    /// Whether `p` lies inside the volume's box.
    #[func]
    fn contains_point(&self, vol: i64, p: Vector3) -> bool {
        self.entry(vol)
            .is_some_and(|e| e.volume.grid.contains_point(v3_in(p)))
    }

    // --- search jobs -------------------------------------------------------------------------

    /// Queue a search. Returns a job handle, or 0 when the volume is not built.
    #[func]
    fn request_path(
        &mut self,
        vol: i64,
        from_p: Vector3,
        to_p: Vector3,
        surface_only: bool,
        max_expansions: i64,
        start_snap_r: i64,
    ) -> i64 {
        if self.built(vol).is_none() {
            return 0;
        }
        let Some(stamp) = self.stamp(vol) else {
            return 0;
        };
        let volume = Arc::clone(&self.volumes[&vol].volume);
        let params = SearchParams {
            from_p: v3_in(from_p),
            to_p: v3_in(to_p),
            surface_only,
            max_expansions: max_expansions.max(0) as u32,
            start_snap_r: start_snap_r as i32,
        };
        self.pool().submit_search(volume, stamp, params) as i64
    }

    /// Where a job is: one of the `JOB_*` constants.
    #[func]
    fn job_state(&mut self, vol: i64, job: i64) -> i64 {
        let stamp = self.stamp(vol);
        let state = self.pool().poll(job as u64, stamp);
        job_state_code(state)
    }

    /// Take a finished search's waypoints. Empty unless the outcome carries a path.
    ///
    /// The job is consumed: polling it again reads `JOB_UNKNOWN`. Read [`Self::last_outcome`] after
    /// this to learn WHY an empty result is empty.
    #[func]
    fn take_path(&mut self, job: i64) -> PackedVector3Array {
        match self.pool().take(job as u64) {
            Some(JobOutput::Path {
                outcome,
                waypoints,
                expansions,
            }) => {
                self.last_outcome_code = outcome as i64;
                self.last_expansions = i64::from(expansions);
                points_out(&waypoints)
            }
            other => {
                if other.is_some() {
                    self.last_outcome_code = Outcome::Failed as i64;
                } else {
                    self.last_outcome_code = Outcome::Pending as i64;
                }
                self.last_expansions = 0;
                PackedVector3Array::new()
            }
        }
    }

    /// How the last taken search ended: one of the `OUTCOME_*` constants.
    ///
    /// This is the distinction the caller cannot otherwise make. An empty path may mean the route
    /// does not exist (`OUTCOME_UNREACHABLE`, never ask again), that the budget ran out
    /// (`OUTCOME_CAPPED`, ask again with more), or that an end had no usable cell nearby
    /// (`OUTCOME_UNSNAPPABLE`, move the request).
    #[func]
    fn last_outcome(&self) -> i64 {
        self.last_outcome_code
    }

    /// How many cells the last taken search expanded.
    #[func]
    fn last_expansions(&self) -> i64 {
        self.last_expansions
    }

    /// Abandon a job.
    #[func]
    fn cancel_job(&mut self, job: i64) {
        self.pool().cancel(job as u64);
    }

    // --- diagnostics -------------------------------------------------------------------------

    /// `[threads, queued, running, ready]`.
    ///
    /// A packed array rather than a dictionary: this is read on a diagnostic path that may run every
    /// frame, and a dictionary allocates per key.
    #[func]
    fn pool_stats(&mut self) -> PackedInt64Array {
        PackedInt64Array::from(&self.pool().stats())
    }

    /// How many workers the pool holds. Starts it if it has not started.
    #[func]
    fn worker_count(&mut self) -> i64 {
        self.pool().threads() as i64
    }

    /// Ask for a pool of `n` workers, or 0 to size it from the machine. Only read at pool start.
    #[func]
    fn set_worker_threads(&mut self, n: i64) {
        self.worker_threads = n.max(0);
    }
}

fn job_state_code(state: JobState) -> i64 {
    match state {
        JobState::Unknown => 0,
        JobState::Pending => 1,
        JobState::Running => 2,
        JobState::Ready => 3,
        JobState::Failed => 4,
        JobState::Cancelled => 5,
        JobState::Stale => 6,
    }
}
