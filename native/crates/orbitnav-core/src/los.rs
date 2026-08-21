//! Line of sight: whether a straight segment crosses only usable cells.
//!
//! A voxel DDA in the Amanatides & Woo form -- step to whichever axis boundary the ray reaches
//! first, test the cell it enters, repeat. Used twice: as the search's short-circuit when a route is
//! just a straight line, and as the test path smoothing asks about every candidate leg.
//!
//! Conservative in two ways, both deliberate:
//!
//! - **Leaving the lattice fails.** Nothing is known about geometry outside, so a segment that exits
//!   is refused rather than assumed clear.
//! - **Both endpoint cells are tested first**, so a segment that starts or ends inside solid is
//!   refused before any stepping.
//!
//! QUIRK: `t_max` and `t_delta` are held at `f32` width, because the reference keeps them in a
//! vector. Each step therefore rounds twice -- add at `f64`, store at `f32` -- rather than
//! accumulating at full width. On a long segment the two disagree about which axis steps next at a
//! near-tie, and that changes which cells are visited.

use crate::grid::Cell;
use crate::real::{n, w, Vec3};
use crate::volume::Volume;

/// The sign of a component, as a cell step: -1, 0 or +1.
#[inline]
#[must_use]
pub fn sign_step(v: f32) -> i32 {
    if v > 0.0 {
        1
    } else if v < 0.0 {
        -1
    } else {
        0
    }
}

/// Whether the straight segment `a` -> `b` crosses only cells usable by this kind of mover.
#[must_use]
pub fn has_los(vol: &Volume, a: Vec3, b: Vec3, surface_only: bool) -> bool {
    let g = vol.grid;
    let mut c: Cell = g.world_to_cell(a);
    let goal: Cell = g.world_to_cell(b);
    if !vol.is_usable_cell(c, surface_only) || !vol.is_usable_cell(goal, surface_only) {
        return false;
    }

    let delta = Vec3::new(b.x - a.x, b.y - a.y, b.z - a.z);
    let step = [sign_step(delta.x), sign_step(delta.y), sign_step(delta.z)];

    // t_max: how far along the segment the next boundary crossing on each axis lies.
    // t_delta: how far apart successive crossings on that axis are.
    let mut t_max = Vec3::ZERO;
    let mut t_delta = Vec3::ZERO;
    for axis in 0..3 {
        let d = delta.axis(axis);
        if d.abs() < 0.000001 {
            t_max.set_axis(axis, f32::INFINITY);
            t_delta.set_axis(axis, f32::INFINITY);
            continue;
        }
        let ahead = if step[axis] > 0 { 1.0 } else { 0.0 };
        let cell_edge = w(g.origin.axis(axis)) + (f64::from(c[axis]) + ahead) * g.cell_size;
        t_max.set_axis(axis, n((cell_edge - w(a.axis(axis))) / w(d)));
        t_delta.set_axis(axis, n(g.cell_size / w(d.abs())));
    }

    // A straight line crosses at most dx + dy + dz + 1 cells; the guard bounds the walk so a
    // degenerate ray cannot spin.
    let guard = g.dims[0] + g.dims[1] + g.dims[2] + 3;
    for _ in 0..guard {
        if c == goal {
            return true;
        }
        let mut axis_min = 0usize;
        if t_max.y < t_max.axis(axis_min) {
            axis_min = 1;
        }
        if t_max.z < t_max.axis(axis_min) {
            axis_min = 2;
        }
        c[axis_min] += step[axis_min];
        t_max.set_axis(axis_min, t_max.axis(axis_min) + t_delta.axis(axis_min));
        if !vol.is_usable_cell(c, surface_only) {
            return false;
        }
    }
    false
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::grid::Grid;
    use crate::occupancy::Occupancy;

    fn walled() -> Volume {
        // A 16x8x8 lattice with a full-height wall at x = 8 and a single doorway through it.
        let g = Grid::new(Vec3::ZERO, 1.0, [16, 8, 8]);
        let mut o = Occupancy::empty(g);
        for y in 0..8 {
            for z in 0..8 {
                if !(y == 4 && z == 4) {
                    o.solid[g.cell_index([8, y, z])] = 1;
                }
            }
        }
        Volume::derive(&o, 1)
    }

    #[test]
    fn a_clear_axis_run_sees() {
        let v = walled();
        assert!(has_los(
            &v,
            Vec3::new(1.5, 4.5, 4.5),
            Vec3::new(6.5, 4.5, 4.5),
            false
        ));
    }

    #[test]
    fn the_wall_blocks() {
        let v = walled();
        assert!(!has_los(
            &v,
            Vec3::new(1.5, 2.5, 2.5),
            Vec3::new(14.5, 2.5, 2.5),
            false
        ));
    }

    #[test]
    fn the_doorway_lets_a_straight_run_through() {
        let v = walled();
        assert!(has_los(
            &v,
            Vec3::new(1.5, 4.5, 4.5),
            Vec3::new(14.5, 4.5, 4.5),
            false
        ));
    }

    #[test]
    fn a_point_sees_itself() {
        let v = walled();
        let p = Vec3::new(3.5, 3.5, 3.5);
        assert!(has_los(&v, p, p, false));
    }

    #[test]
    fn leaving_the_lattice_fails() {
        let v = walled();
        assert!(!has_los(
            &v,
            Vec3::new(3.5, 3.5, 3.5),
            Vec3::new(300.0, 3.5, 3.5),
            false
        ));
    }

    #[test]
    fn a_segment_starting_in_solid_fails() {
        let v = walled();
        assert!(!has_los(
            &v,
            Vec3::new(8.5, 1.5, 1.5),
            Vec3::new(3.5, 1.5, 1.5),
            false
        ));
    }

    #[test]
    fn a_zero_component_never_steps_that_axis() {
        // delta.y is exactly zero, so its t_max is infinite and the axis is never chosen.
        let v = walled();
        assert!(has_los(
            &v,
            Vec3::new(1.5, 4.5, 4.5),
            Vec3::new(6.5, 4.5, 6.5),
            false
        ));
    }

    #[test]
    fn surface_only_refuses_a_shortcut_through_open_space() {
        // A floor world: a leg high above the floor is clear for free flight and refused for a
        // surface mover, because the cells it crosses are not skin.
        let g = Grid::new(Vec3::ZERO, 1.0, [12, 6, 8]);
        let mut o = Occupancy::empty(g);
        for y in 0..6 {
            for x in 0..12 {
                o.solid[g.cell_index([x, y, 0])] = 1;
            }
        }
        let v = Volume::derive(&o, 1);
        let a = Vec3::new(1.5, 2.5, 5.5);
        let b = Vec3::new(9.5, 2.5, 5.5);
        assert!(has_los(&v, a, b, false));
        assert!(!has_los(&v, a, b, true));
    }
}
