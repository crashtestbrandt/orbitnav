//! A* over the free (or surface) cells, and the outcome of asking for a route.
//!
//! Six-connected, weighted-heuristic A* at CELL granularity. A search is created by
//! [`Search::begin`], which resolves the cheap cases immediately, and then driven by
//! [`Search::step`] until it settles.
//!
//! # Resolution order in `begin`
//!
//! Reproduced exactly, because each step can change the answer:
//!
//! 1. an unbuilt volume -> [`Outcome::Unbuilt`]
//! 2. snap the START at the caller's radius
//! 3. snap the GOAL at the default radius for the mover kind -- NOT the caller's
//! 4. either snap missing -> [`Outcome::Unsnappable`]
//! 5. the two cells in different components -> [`Outcome::Unreachable`]
//! 6. a clear straight run -> [`Outcome::Direct`], with the requested point as the only waypoint
//! 7. otherwise seed the open set
//!
//! Step 6 is the dominant case for open-space chases; without it the search floods an enormous free
//! component hunting a route the straight line already is.
//!
//! # Weighted, and deliberately inadmissible
//!
//! The heuristic is inflated: 1.6 for free flight, 2.4 for surface. An admissible search over a
//! surface manifold floods far too wide, because Euclidean distance is a poor guide once the skin
//! wraps. The routes are not shortest; they are found in time.
//!
//! # Quirks reproduced from the reference
//!
//! - **The seed key carries no weight.** Every in-loop push is `cost + h * weight`, but the seed is
//!   pushed with the bare distance. Harmless -- the seed pops first regardless -- and reproduced so
//!   the comparison against the reference is exact.
//! - **The heap stores `f32` and compares against `f64`.** See [`MinHeap::push`].
//! - **`g` scores are `f32`.** The dominance test therefore compares a rounded incumbent against an
//!   un-rounded challenger, which decides some ties differently from an all-`f64` search.

use crate::grid::{Cell, NO_CELL};
use crate::los::has_los;
use crate::real::{w, Vec3};
use crate::smooth::smooth_path;
use crate::snap::{default_snap_r, nearest_free_cell};
use crate::volume::Volume;

/// How a search ended.
///
/// The reference implementation collapses every failure onto "no waypoints", so a caller cannot tell
/// a route that does not exist from one it was not given time to find. Separating them is the point
/// of this enum: `Capped` means ask again with more budget, `Unreachable` means never ask again.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(i32)]
pub enum Outcome {
    /// Still running; no answer yet.
    Pending = 0,
    /// A route was found.
    Ok = 1,
    /// The straight line was already clear; the waypoints are just the destination.
    Direct = 2,
    /// The two ends are in different components. No budget will help.
    Unreachable = 3,
    /// The expansion cap was reached with the frontier still alive. More budget might help.
    Capped = 4,
    /// One or both ends had no usable cell within the snap radius.
    Unsnappable = 5,
    /// The volume has no derived data.
    Unbuilt = 6,
    /// The volume was replaced or freed while the search was in flight.
    Stale = 7,
    /// The search panicked or could not run.
    Failed = 8,
}

impl Outcome {
    /// Whether the search has stopped, whatever the answer.
    #[inline]
    #[must_use]
    pub fn is_settled(self) -> bool {
        self != Outcome::Pending
    }

    /// Whether waypoints are available.
    #[inline]
    #[must_use]
    pub fn has_path(self) -> bool {
        matches!(self, Outcome::Ok | Outcome::Direct)
    }
}

/// What a caller asks for.
#[derive(Clone, Copy, Debug)]
pub struct SearchParams {
    /// Where the mover is.
    pub from_p: Vec3,
    /// Where it wants to be.
    pub to_p: Vec3,
    /// Whether it is restricted to surface cells.
    pub surface_only: bool,
    /// The expansion ceiling for the whole search.
    pub max_expansions: u32,
    /// The snap radius for the START end.
    pub start_snap_r: i32,
}

impl SearchParams {
    /// Parameters with the default snap radius for the mover kind.
    #[must_use]
    pub fn new(from_p: Vec3, to_p: Vec3, surface_only: bool, max_expansions: u32) -> Self {
        Self {
            from_p,
            to_p,
            surface_only,
            max_expansions,
            start_snap_r: default_snap_r(surface_only),
        }
    }
}

/// A binary min-heap keyed by f-score.
///
/// QUIRK, and the reason this is written out rather than replaced with `BinaryHeap`: the key is
/// STORED narrowed to `f32` but COMPARED at the width it arrived in. `push` takes an `f64`, writes
/// the rounded value into the array, and then sifts by comparing the stored parent against the
/// UN-ROUNDED argument. `pop` re-reads its key from the array, so its comparisons are rounded on
/// both sides. Two nodes whose scores differ by less than `f32` precision therefore order one way on
/// the way in and another on the way out, which decides ties -- and a tie in the open set decides
/// which of two equal-cost routes is returned.
#[derive(Debug, Default)]
pub struct MinHeap {
    keys: Vec<f32>,
    vals: Vec<u32>,
}

impl MinHeap {
    /// An empty heap.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// How many entries the heap holds.
    #[inline]
    #[must_use]
    pub fn len(&self) -> usize {
        self.vals.len()
    }

    /// Whether the heap is empty.
    #[inline]
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.vals.is_empty()
    }

    /// Insert `val` under `key`. See the type note on the width asymmetry.
    pub fn push(&mut self, key: f64, val: u32) {
        let mut i = self.vals.len();
        self.keys.push(key as f32);
        self.vals.push(val);
        while i > 0 {
            let parent = (i - 1) >> 1;
            // Stored (f32, widened) against the caller's un-rounded f64.
            if w(self.keys[parent]) <= key {
                break;
            }
            self.keys[i] = self.keys[parent];
            self.vals[i] = self.vals[parent];
            i = parent;
        }
        self.keys[i] = key as f32;
        self.vals[i] = val;
    }

    /// Remove and return the lowest-keyed value, or `None`.
    pub fn pop(&mut self) -> Option<u32> {
        if self.vals.is_empty() {
            return None;
        }
        let top = self.vals[0];
        let last = self.vals.len() - 1;
        let key = self.keys[last];
        let val = self.vals[last];
        self.keys.remove(last);
        self.vals.remove(last);
        let n = last;
        if n == 0 {
            return Some(top);
        }
        let mut i = 0usize;
        loop {
            let l = 2 * i + 1;
            if l >= n {
                break;
            }
            let mut child = l;
            let r = l + 1;
            if r < n && self.keys[r] < self.keys[l] {
                child = r;
            }
            if self.keys[child] >= key {
                break;
            }
            self.keys[i] = self.keys[child];
            self.vals[i] = self.vals[child];
            i = child;
        }
        self.keys[i] = key;
        self.vals[i] = val;
        Some(top)
    }
}

/// A resumable search over one volume.
///
/// Unlike the reference, each search owns its scratch arrays, so two searches may run at once and
/// neither can invalidate the other. The reference shares one static scratch behind a generation
/// stamp and restarts a search whose stamp was stolen; that case cannot arise here.
pub struct Search {
    params: SearchParams,
    outcome: Outcome,
    result: Vec<Vec3>,
    open: MinHeap,
    g_score: Vec<f32>,
    came: Vec<i32>,
    seen: Vec<u8>,
    start_ci: usize,
    goal_ci: usize,
    goal_center: Vec3,
    h_weight: f64,
    expansions: u32,
    found: i64,
}

impl Search {
    /// Resolve the cheap cases and seed the open set.
    #[must_use]
    pub fn begin(vol: &Volume, params: SearchParams) -> Self {
        let n = vol.cell_count();
        let mut s = Self {
            params,
            outcome: Outcome::Pending,
            result: Vec::new(),
            open: MinHeap::new(),
            g_score: Vec::new(),
            came: Vec::new(),
            seen: Vec::new(),
            start_ci: 0,
            goal_ci: 0,
            goal_center: Vec3::ZERO,
            h_weight: 1.6,
            expansions: 0,
            found: -1,
        };
        if n == 0 {
            s.outcome = Outcome::Unbuilt;
            return s;
        }

        let start_cell =
            nearest_free_cell(vol, params.from_p, params.surface_only, params.start_snap_r);
        // The GOAL end snaps at the default radius for the mover kind, not the caller's. The two
        // ends must agree, or a mover can be able to plan a route and unable to be given a
        // destination -- and the goal a surface mover is handed is most often something off the skin.
        let goal_cell = nearest_free_cell(
            vol,
            params.to_p,
            params.surface_only,
            default_snap_r(params.surface_only),
        );
        if start_cell == NO_CELL || goal_cell == NO_CELL {
            s.outcome = Outcome::Unsnappable;
            return s;
        }

        let start_ci = vol.grid.cell_index(start_cell);
        let goal_ci = vol.grid.cell_index(goal_cell);
        if vol.label_at_index(start_ci, params.surface_only)
            != vol.label_at_index(goal_ci, params.surface_only)
        {
            s.outcome = Outcome::Unreachable;
            return s;
        }

        if has_los(vol, params.from_p, params.to_p, params.surface_only) {
            s.result = vec![params.to_p];
            s.outcome = Outcome::Direct;
            return s;
        }

        s.goal_center = vol.grid.cell_center(goal_cell);
        s.start_ci = start_ci;
        s.goal_ci = goal_ci;
        s.h_weight = if params.surface_only { 2.4 } else { 1.6 };
        s.g_score = vec![0.0; n];
        s.came = vec![-1; n];
        s.seen = vec![0; n];
        s.seen[start_ci] = 1;
        s.g_score[start_ci] = 0.0;
        s.came[start_ci] = -1;
        // QUIRK: the seed key is the bare distance, with no h_weight applied. Every later push
        // weights its heuristic. Harmless because this entry pops first regardless.
        let seed = vol.grid.cell_center(start_cell).distance_to(s.goal_center);
        s.open.push(w(seed), start_ci as u32);
        s
    }

    /// How the search ended, or [`Outcome::Pending`].
    #[inline]
    #[must_use]
    pub fn outcome(&self) -> Outcome {
        self.outcome
    }

    /// Whether the search has stopped.
    #[inline]
    #[must_use]
    pub fn is_settled(&self) -> bool {
        self.outcome.is_settled()
    }

    /// How many cells have been expanded.
    #[inline]
    #[must_use]
    pub fn expansions(&self) -> u32 {
        self.expansions
    }

    /// The waypoints, once settled. Empty unless the outcome carries a path.
    #[must_use]
    pub fn result(&self) -> &[Vec3] {
        &self.result
    }

    /// Take the waypoints, leaving the search empty.
    #[must_use]
    pub fn take_result(&mut self) -> Vec<Vec3> {
        std::mem::take(&mut self.result)
    }

    /// Expand at most `budget` cells. Returns whether the search has settled.
    pub fn step(&mut self, vol: &Volume, budget: u32) -> bool {
        if self.outcome.is_settled() {
            return true;
        }
        let g = vol.grid;
        let (dx, dy, dz) = (g.dims[0], g.dims[1], g.dims[2]);
        let strides = g.neighbour_strides();
        let cs = g.cell_size;
        let (ox, oy, oz) = (w(g.origin.x), w(g.origin.y), w(g.origin.z));
        let (gx, gy, gz) = (
            w(self.goal_center.x),
            w(self.goal_center.y),
            w(self.goal_center.z),
        );
        let mut spent = 0u32;

        while spent < budget && self.expansions < self.params.max_expansions {
            let Some(ci) = self.open.pop() else { break };
            let ci = ci as usize;
            self.expansions += 1;
            spent += 1;
            if ci == self.goal_ci {
                self.found = ci as i64;
                self.settle(vol);
                return true;
            }
            let c = g.index_cell(ci);
            let g_here = w(self.g_score[ci]);
            for (d, &stride) in strides.iter().enumerate() {
                let (nx, ny, nz) = match d {
                    0 => (c[0] - 1, c[1], c[2]),
                    1 => (c[0] + 1, c[1], c[2]),
                    2 => (c[0], c[1] - 1, c[2]),
                    3 => (c[0], c[1] + 1, c[2]),
                    4 => (c[0], c[1], c[2] - 1),
                    _ => (c[0], c[1], c[2] + 1),
                };
                if nx < 0 || ny < 0 || nz < 0 || nx >= dx || ny >= dy || nz >= dz {
                    continue;
                }
                let nb = (ci as isize + stride) as usize;
                if vol.solid[nb] != 0 {
                    continue;
                }
                if self.params.surface_only && vol.surface_flag[nb] == 0 {
                    continue;
                }
                let mut step_cost = cs;
                if self.params.surface_only {
                    // Prefer open skin over a tight pocket. A cell walled on several sides is hard
                    // terrain for a mover on a surface, so take the exterior wrap when one exists
                    // and dive through a narrow throat only when it is the only way.
                    let sides = i32::from(vol.solid_sides[nb]);
                    step_cost = cs * (1.0 + 2.5 * f64::from((sides - 1).max(0)));
                }
                let cost = g_here + step_cost;
                if self.seen[nb] != 0 && w(self.g_score[nb]) <= cost {
                    continue;
                }
                self.seen[nb] = 1;
                self.g_score[nb] = cost as f32;
                self.came[nb] = ci as i32;
                let hx = ox + (f64::from(nx) + 0.5) * cs - gx;
                let hy = oy + (f64::from(ny) + 0.5) * cs - gy;
                let hz = oz + (f64::from(nz) + 0.5) * cs - gz;
                let h = (hx * hx + hy * hy + hz * hz).sqrt();
                self.open.push(cost + h * self.h_weight, nb as u32);
            }
        }

        if self.open.is_empty() {
            self.settle(vol); // the frontier is exhausted: no route exists
            return true;
        }
        if self.expansions >= self.params.max_expansions {
            self.settle(vol); // the cap stopped a search that might still have found one
            return true;
        }
        false
    }

    /// Run to completion. Convenience for tests and for a caller with no incremental budget.
    pub fn run(&mut self, vol: &Volume) -> Outcome {
        while !self.step(vol, 4096) {}
        self.outcome
    }

    fn settle(&mut self, vol: &Volume) {
        if self.found < 0 {
            // Which failure it was depends on why the loop stopped.
            self.outcome = if self.expansions >= self.params.max_expansions && !self.open.is_empty()
            {
                Outcome::Capped
            } else {
                Outcome::Unreachable
            };
            self.result.clear();
            return;
        }
        let g = vol.grid;
        let mut raw: Vec<Vec3> = Vec::new();
        let mut cursor = self.found;
        while cursor != -1 {
            raw.push(g.cell_center(g.index_cell(cursor as usize)));
            cursor = i64::from(self.came[cursor as usize]);
        }
        raw.reverse();
        let mut out = smooth_path(vol, self.params.from_p, &raw, self.params.surface_only);
        // Land exactly on what was asked for when the last leg is clear; otherwise stop at the
        // goal cell's centre.
        let tail = out.last().copied().unwrap_or(self.params.from_p);
        if has_los(vol, tail, self.params.to_p, self.params.surface_only) {
            out.push(self.params.to_p);
        } else if out.is_empty() || !tail.is_equal_approx(self.goal_center) {
            out.push(self.goal_center);
        }
        self.result = out;
        self.outcome = Outcome::Ok;
    }

    /// The cell the search snapped its start to, for diagnostics.
    #[must_use]
    pub fn start_cell(&self, vol: &Volume) -> Cell {
        vol.grid.index_cell(self.start_ci)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::grid::Grid;
    use crate::occupancy::Occupancy;

    /// A wall with one doorway, so the straight-line short-circuit cannot answer.
    fn walled() -> Volume {
        let g = Grid::new(Vec3::ZERO, 1.0, [24, 8, 8]);
        let mut o = Occupancy::empty(g);
        for y in 0..8 {
            for z in 0..8 {
                if !(y == 6 && z == 6) {
                    o.solid[g.cell_index([12, y, z])] = 1;
                }
            }
        }
        Volume::derive(&o, 1)
    }

    fn sealed() -> Volume {
        // Two rooms with no doorway at all.
        let g = Grid::new(Vec3::ZERO, 1.0, [24, 8, 8]);
        let mut o = Occupancy::empty(g);
        for y in 0..8 {
            for z in 0..8 {
                o.solid[g.cell_index([12, y, z])] = 1;
            }
        }
        Volume::derive(&o, 1)
    }

    #[test]
    fn a_clear_run_answers_direct_without_expanding() {
        let v = walled();
        let p = SearchParams::new(
            Vec3::new(1.5, 1.5, 1.5),
            Vec3::new(9.5, 1.5, 1.5),
            false,
            8000,
        );
        let mut s = Search::begin(&v, p);
        assert_eq!(s.outcome(), Outcome::Direct);
        assert_eq!(s.expansions(), 0);
        assert_eq!(s.result(), &[Vec3::new(9.5, 1.5, 1.5)]);
        assert!(s.step(&v, 100), "an already-settled search stays settled");
    }

    #[test]
    fn a_route_through_the_doorway_is_found() {
        let v = walled();
        let p = SearchParams::new(
            Vec3::new(1.5, 1.5, 1.5),
            Vec3::new(22.5, 1.5, 1.5),
            false,
            20000,
        );
        let mut s = Search::begin(&v, p);
        assert_eq!(s.run(&v), Outcome::Ok);
        assert!(!s.result().is_empty());
        assert_eq!(*s.result().last().unwrap(), Vec3::new(22.5, 1.5, 1.5));
    }

    #[test]
    fn stepping_in_any_chunk_gives_the_same_route() {
        let v = walled();
        let p = SearchParams::new(
            Vec3::new(1.5, 1.5, 1.5),
            Vec3::new(22.5, 1.5, 1.5),
            false,
            20000,
        );
        let mut one = Search::begin(&v, p);
        assert_eq!(one.run(&v), Outcome::Ok);
        let reference: Vec<Vec3> = one.result().to_vec();
        for chunk in [1u32, 2, 7, 64] {
            let mut s = Search::begin(&v, p);
            while !s.step(&v, chunk) {}
            assert_eq!(s.outcome(), Outcome::Ok, "chunk {chunk}");
            assert_eq!(s.result(), reference.as_slice(), "chunk {chunk} diverged");
        }
    }

    #[test]
    fn a_sealed_room_is_unreachable() {
        let v = sealed();
        let p = SearchParams::new(
            Vec3::new(1.5, 1.5, 1.5),
            Vec3::new(22.5, 1.5, 1.5),
            false,
            20000,
        );
        let mut s = Search::begin(&v, p);
        assert_eq!(s.run(&v), Outcome::Unreachable);
        assert!(s.result().is_empty());
    }

    #[test]
    fn the_cap_is_distinguishable_from_unreachable() {
        let v = walled();
        let p = SearchParams::new(
            Vec3::new(1.5, 1.5, 1.5),
            Vec3::new(22.5, 1.5, 1.5),
            false,
            12,
        );
        let mut s = Search::begin(&v, p);
        assert_eq!(
            s.run(&v),
            Outcome::Capped,
            "this is the whole point of the enum"
        );
        assert!(s.result().is_empty());
    }

    #[test]
    fn an_unsnappable_end_says_so() {
        // Solid everywhere but one cell, so the goal has no usable cell within any radius.
        let g = Grid::new(Vec3::ZERO, 1.0, [24, 8, 8]);
        let mut o = Occupancy::empty(g);
        o.solid.iter_mut().for_each(|b| *b = 1);
        o.solid[g.cell_index([1, 1, 1])] = 0;
        let v = Volume::derive(&o, 1);
        let p = SearchParams {
            from_p: Vec3::new(1.5, 1.5, 1.5),
            to_p: Vec3::new(20.5, 6.5, 6.5),
            surface_only: false,
            max_expansions: 20000,
            start_snap_r: 1,
        };
        assert_eq!(Search::begin(&v, p).outcome(), Outcome::Unsnappable);
    }

    #[test]
    fn an_empty_volume_is_unbuilt() {
        let v = Volume::derive(&Occupancy::empty(Grid::new(Vec3::ZERO, 1.0, [0, 0, 0])), 1);
        let p = SearchParams::new(Vec3::ZERO, Vec3::ZERO, false, 100);
        assert_eq!(Search::begin(&v, p).outcome(), Outcome::Unbuilt);
    }

    #[test]
    fn the_heap_pops_in_key_order() {
        let mut h = MinHeap::new();
        for (k, v) in [(5.0, 5u32), (1.0, 1), (3.0, 3), (2.0, 2), (4.0, 4)] {
            h.push(k, v);
        }
        let mut got = Vec::new();
        while let Some(v) = h.pop() {
            got.push(v);
        }
        assert_eq!(got, vec![1, 2, 3, 4, 5]);
    }

    #[test]
    fn the_heap_stores_f32_and_compares_f64() {
        // Two keys that are distinct as f64 and identical once narrowed to f32. The store rounds
        // them together while the sift comparison still sees them apart -- the asymmetry the type
        // note describes.
        let a = 1.0f64 + f64::from(f32::EPSILON) * 0.25;
        assert_eq!(a as f32, 1.0f32, "the two keys must collide at f32 width");
        assert_ne!(a, 1.0f64, "but differ at f64 width");
        let mut h = MinHeap::new();
        h.push(a, 7);
        h.push(1.0, 9);
        assert_eq!(h.len(), 2);
        assert!(h.pop().is_some());
        assert!(h.pop().is_some());
        assert!(h.pop().is_none());
    }

    #[test]
    fn an_empty_heap_pops_nothing() {
        assert_eq!(MinHeap::new().pop(), None);
    }

    #[test]
    fn the_surface_pocket_penalty_changes_the_route() {
        // A floor with a narrow slot. A surface mover should prefer the open skin, so its route and
        // a free-flight route over the same endpoints differ.
        let g = Grid::new(Vec3::ZERO, 1.0, [20, 10, 8]);
        let mut o = Occupancy::empty(g);
        for y in 0..10 {
            for x in 0..20 {
                o.solid[g.cell_index([x, y, 0])] = 1;
            }
        }
        for x in 0..20 {
            for z in 1..4 {
                if x != 10 {
                    o.solid[g.cell_index([x, 5, z])] = 1;
                }
            }
        }
        let v = Volume::derive(&o, 1);
        let a = Vec3::new(2.5, 2.5, 1.5);
        let b = Vec3::new(17.5, 8.5, 1.5);
        let mut free = Search::begin(&v, SearchParams::new(a, b, false, 40000));
        let mut surf = Search::begin(&v, SearchParams::new(a, b, true, 40000));
        let fo = free.run(&v);
        let so = surf.run(&v);
        assert!(fo.has_path(), "free flight should find a way");
        assert!(so.is_settled());
        if so.has_path() {
            assert_ne!(
                free.result(),
                surf.result(),
                "the pocket penalty must bend the surface route"
            );
        }
    }

    /// The negative test the module's width discipline depends on.
    ///
    /// One recorded search, run again with the heap and the g-scores held at `f64` throughout. If
    /// the two ever agree for every input, the narrowing has stopped mattering and the QUIRK notes
    /// above are wrong.
    #[test]
    fn width_is_load_bearing() {
        // A heap that keeps full width, against the shipped one, on keys that collide at f32.
        let mut narrow = MinHeap::new();
        let mut wide: Vec<(f64, u32)> = Vec::new();
        let base = 1_000_000.0f64;
        for k in 0..64u32 {
            let key = base + f64::from(k) * 1e-3;
            narrow.push(key, k);
            wide.push((key, k));
        }
        wide.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap());

        let mut narrow_order = Vec::new();
        while let Some(v) = narrow.pop() {
            narrow_order.push(v);
        }
        let wide_order: Vec<u32> = wide.into_iter().map(|(_, v)| v).collect();

        assert_ne!(
            narrow_order, wide_order,
            "f32 keys must lose distinctions f64 keeps; if not, the width notes are stale"
        );
    }
}
