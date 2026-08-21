//! Path smoothing: collapse a cell-by-cell route into the fewest waypoints that still fit.
//!
//! Greedy furthest-visible. From the current anchor, take the LAST waypoint the anchor can see,
//! emit it, and make it the next anchor. A route that came out of the search as a staircase of
//! adjacent cell centres becomes a handful of corners.
//!
//! # Two ordering rules
//!
//! - **The candidate scan runs backwards**, from the far end toward the anchor, so the first leg it
//!   accepts is the longest one available.
//! - **The cheap test comes first.** A surface leg longer than [`SURFACE_LEG_MAX_M`] is rejected
//!   whatever line of sight says, and a rejected long leg is the most expensive DDA there is because
//!   it walks the whole span. Testing sight first spends most of the smoothing budget computing
//!   answers that are then discarded.
//!
//! The leg cap applies to surface movers only: a mover walking a hull must not be handed a long
//! straight leg that leaves the surface between its ends, even when nothing solid is in the way.

use crate::los::has_los;
use crate::real::Vec3;
use crate::volume::Volume;

/// The longest leg a surface-restricted route may contain, in metres.
pub const SURFACE_LEG_MAX_M: f32 = 3.0;

/// Collapse `raw` into the fewest waypoints reachable in sequence from `from_p`.
#[must_use]
pub fn smooth_path(vol: &Volume, from_p: Vec3, raw: &[Vec3], surface_only: bool) -> Vec<Vec3> {
    let mut out: Vec<Vec3> = Vec::new();
    let mut anchor = from_p;
    let mut i = 0usize;
    while i < raw.len() {
        let mut j = raw.len() - 1;
        while j > i {
            let leg_ok = !surface_only || anchor.distance_to(raw[j]) <= SURFACE_LEG_MAX_M;
            if leg_ok && has_los(vol, anchor, raw[j], surface_only) {
                break;
            }
            j -= 1;
        }
        out.push(raw[j]);
        anchor = raw[j];
        i = j + 1;
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::grid::Grid;
    use crate::occupancy::Occupancy;

    fn open() -> Volume {
        Volume::derive(&Occupancy::empty(Grid::new(Vec3::ZERO, 1.0, [24, 8, 8])), 1)
    }

    fn floor() -> Volume {
        let g = Grid::new(Vec3::ZERO, 1.0, [24, 8, 8]);
        let mut o = Occupancy::empty(g);
        for y in 0..8 {
            for x in 0..24 {
                o.solid[g.cell_index([x, y, 0])] = 1;
            }
        }
        Volume::derive(&o, 1)
    }

    fn straight_run(v: &Volume, z: i32, n: i32) -> Vec<Vec3> {
        (1..=n).map(|x| v.grid.cell_center([x, 4, z])).collect()
    }

    #[test]
    fn a_clear_run_collapses_to_its_far_end() {
        let v = open();
        let raw = straight_run(&v, 4, 20);
        let out = smooth_path(&v, Vec3::new(0.5, 4.5, 4.5), &raw, false);
        assert_eq!(out.len(), 1, "free flight has no leg cap");
        assert_eq!(out[0], raw[raw.len() - 1]);
    }

    #[test]
    fn the_surface_leg_cap_rejects_a_clear_long_leg() {
        let v = floor();
        let raw = straight_run(&v, 1, 20);
        let out = smooth_path(&v, v.grid.cell_center([0, 4, 1]), &raw, true);
        assert!(
            out.len() > 1,
            "the cap must break a long clear run into legs"
        );
        let mut anchor = v.grid.cell_center([0, 4, 1]);
        for p in &out {
            assert!(
                anchor.distance_to(*p) <= SURFACE_LEG_MAX_M + 0.0001,
                "leg of {} exceeds the cap",
                anchor.distance_to(*p)
            );
            anchor = *p;
        }
    }

    #[test]
    fn a_blocked_route_keeps_the_corner_it_needs() {
        // An L around a wall: smoothing must keep a waypoint at the corner.
        let g = Grid::new(Vec3::ZERO, 1.0, [12, 12, 4]);
        let mut o = Occupancy::empty(g);
        for y in 0..8 {
            o.solid[g.cell_index([6, y, 1])] = 1;
        }
        let v = Volume::derive(&o, 1);
        let mut raw: Vec<Vec3> = Vec::new();
        for y in 1..=9 {
            raw.push(v.grid.cell_center([3, y, 1]));
        }
        for x in 3..=10 {
            raw.push(v.grid.cell_center([x, 9, 1]));
        }
        let out = smooth_path(&v, v.grid.cell_center([3, 1, 1]), &raw, false);
        assert!(out.len() >= 2, "a corner cannot be smoothed away");
        assert_eq!(*out.last().unwrap(), *raw.last().unwrap());
    }

    #[test]
    fn every_input_waypoint_is_accounted_for() {
        let v = open();
        let raw = straight_run(&v, 4, 12);
        let out = smooth_path(&v, Vec3::new(0.5, 4.5, 4.5), &raw, false);
        assert!(!out.is_empty());
        assert_eq!(
            *out.last().unwrap(),
            *raw.last().unwrap(),
            "the destination survives"
        );
    }

    #[test]
    fn an_empty_route_smooths_to_nothing() {
        let v = open();
        assert!(smooth_path(&v, Vec3::new(0.5, 4.5, 4.5), &[], false).is_empty());
    }
}
