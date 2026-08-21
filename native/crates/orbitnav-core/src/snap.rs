//! Snapping an arbitrary point to a cell a mover can actually occupy.
//!
//! A query point is rarely in a usable cell: a mover hovering off a surface, a target inside
//! geometry, a destination picked in open space. [`nearest_free_cell`] walks outward in Chebyshev
//! shells and answers with the closest usable cell, or [`crate::grid::NO_CELL`].
//!
//! # Why the radius matters more than it looks
//!
//! This is the arming cost of a search: both endpoints snap before a single node is expanded, and a
//! MISS pays the full cost of every shell. The walk visits the whole `(2r+1)^3` cube at each radius
//! and rejects the interior inside the loop, so a miss at radius `r` costs the sum of those cubes --
//! 2 556 iterations at r = 5 and 19 900 at r = 9. Both ends are snapped before either is checked, so
//! a failing start still pays for the goal.
//!
//! # Order is load-bearing
//!
//! Shells are walked `dx` outermost, then `dy`, then `dz`, and a candidate replaces the incumbent
//! only on a STRICTLY smaller squared distance. Equidistant candidates therefore resolve to the
//! first in that order, which is what makes the answer reproducible rather than merely correct.

use crate::grid::{Cell, NO_CELL};
use crate::real::Vec3;
use crate::volume::Volume;

/// The snap radius a surface-restricted query uses.
///
/// Wider than the open-space default because a surface mover is routinely several cells off the
/// skin. The two ends of a search must agree on this: if a start can snap further than a goal, a
/// mover can be able to PLAN a route and unable to be given a destination.
pub const SURFACE_SNAP_R: i32 = 9;

/// The open-space default snap radius.
pub const FREE_SNAP_R: i32 = 5;

/// The radius a query of this kind should use when the caller has no reason to say otherwise.
#[inline]
#[must_use]
pub fn default_snap_r(surface_only: bool) -> i32 {
    if surface_only {
        SURFACE_SNAP_R
    } else {
        FREE_SNAP_R
    }
}

/// The usable cell nearest `p` within a Chebyshev radius of `max_r`, or [`NO_CELL`].
#[must_use]
pub fn nearest_free_cell(vol: &Volume, p: Vec3, surface_only: bool, max_r: i32) -> Cell {
    let c = vol.grid.world_to_cell(p);
    for r in 0..=max_r {
        let mut best = NO_CELL;
        let mut best_d = f32::INFINITY;
        for dx in -r..=r {
            for dy in -r..=r {
                for dz in -r..=r {
                    // Shell only: the interior radii were covered by earlier iterations. The test
                    // is inside the loop, which is what makes a miss cost the whole cube.
                    if dx.abs().max(dy.abs()).max(dz.abs()) != r {
                        continue;
                    }
                    let cand = [c[0] + dx, c[1] + dy, c[2] + dz];
                    if !vol.is_usable_cell(cand, surface_only) {
                        continue;
                    }
                    let d = vol.grid.cell_center(cand).distance_squared_to(p);
                    if d < best_d {
                        best_d = d;
                        best = cand;
                    }
                }
            }
        }
        if best != NO_CELL {
            return best;
        }
    }
    NO_CELL
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::grid::Grid;
    use crate::occupancy::Occupancy;

    fn floor() -> Volume {
        let g = Grid::new(Vec3::ZERO, 1.0, [12, 12, 10]);
        let mut o = Occupancy::empty(g);
        for y in 0..12 {
            for x in 0..12 {
                o.solid[g.cell_index([x, y, 0])] = 1;
            }
        }
        Volume::derive(&o, 1)
    }

    #[test]
    fn a_point_already_usable_snaps_to_its_own_cell() {
        let v = floor();
        assert_eq!(
            nearest_free_cell(&v, Vec3::new(4.5, 4.5, 4.5), false, 5),
            [4, 4, 4]
        );
    }

    #[test]
    fn a_point_in_solid_escapes_to_the_nearest_free_cell() {
        let v = floor();
        let c = nearest_free_cell(&v, Vec3::new(4.5, 4.5, 0.5), false, 5);
        assert_eq!(c, [4, 4, 1], "straight up out of the floor");
    }

    #[test]
    fn a_point_off_the_skin_needs_the_wider_radius() {
        let v = floor();
        let high = Vec3::new(5.5, 5.5, 7.5);
        assert_eq!(nearest_free_cell(&v, high, true, FREE_SNAP_R), NO_CELL);
        assert_eq!(nearest_free_cell(&v, high, true, SURFACE_SNAP_R), [5, 5, 1]);
    }

    #[test]
    fn nothing_in_range_answers_no_cell() {
        // A lattice that is entirely solid has no usable cell at any radius.
        let g = Grid::new(Vec3::ZERO, 1.0, [6, 6, 6]);
        let mut o = Occupancy::empty(g);
        o.solid.iter_mut().for_each(|b| *b = 1);
        let v = Volume::derive(&o, 1);
        assert_eq!(
            nearest_free_cell(&v, Vec3::new(3.5, 3.5, 3.5), false, 4),
            NO_CELL
        );
    }

    #[test]
    fn ties_resolve_to_the_first_in_shell_order() {
        // A point exactly on the corner between cells has several equidistant candidates. The walk
        // is dx outermost then dy then dz, and the incumbent is only replaced on a strictly smaller
        // distance, so the most negative dx wins.
        let g = Grid::new(Vec3::ZERO, 1.0, [8, 8, 8]);
        let mut o = Occupancy::empty(g);
        // Solid everywhere except a ring of equidistant cells around the query point.
        o.solid.iter_mut().for_each(|b| *b = 1);
        for c in [[3, 4, 4], [5, 4, 4], [4, 3, 4], [4, 5, 4]] {
            o.solid[g.cell_index(c)] = 0;
        }
        let v = Volume::derive(&o, 1);
        let got = nearest_free_cell(&v, v.grid.cell_center([4, 4, 4]), false, 3);
        assert_eq!(got, [3, 4, 4], "lowest dx among equidistant candidates");
    }

    #[test]
    fn default_radii_differ_by_mover_kind() {
        assert_eq!(default_snap_r(true), SURFACE_SNAP_R);
        assert_eq!(default_snap_r(false), FREE_SNAP_R);
    }
}
