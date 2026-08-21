//! A finished navigation volume: occupancy plus everything derived from it.
//!
//! Built once and then read-only. Every query in this crate takes a `&Volume`, so a volume can be
//! shared across threads without synchronisation, and a caller that wants to change one publishes a
//! replacement rather than mutating in place (see [`crate::jobs`]).

use crate::components::{label_components, NO_COMPONENT};
use crate::grid::{Cell, Grid, DIRS, NO_CELL};
use crate::occupancy::Occupancy;
use crate::real::Vec3;
use crate::surface::{derive_surface, SurfaceArrays};

/// Occupancy plus the five derived arrays.
#[derive(Clone, Debug, PartialEq)]
pub struct Volume {
    /// The lattice.
    pub grid: Grid,
    /// One byte per cell; non-zero is solid.
    pub solid: Vec<u8>,
    /// Bit per solid axis neighbour, in [`DIRS`] order.
    pub surface_bits: Vec<u8>,
    /// Popcount of `surface_bits`.
    pub solid_sides: Vec<u8>,
    /// 1 when a free cell has solid in its 26-neighbourhood.
    pub surface_flag: Vec<u8>,
    /// Free-graph component label per cell, -1 outside the graph.
    pub comp_free: Vec<i32>,
    /// Surface-graph component label per cell, -1 outside the graph.
    pub comp_surf: Vec<i32>,
}

impl Volume {
    /// Derive everything from occupancy. `workers` of 0 or 1 derives serially.
    #[must_use]
    pub fn derive(occ: &Occupancy, workers: usize) -> Self {
        let surf = derive_surface(occ, workers);
        let (comp_free, comp_surf) = label_components(occ, &surf);
        Self {
            grid: occ.grid,
            solid: occ.solid.clone(),
            surface_bits: surf.surface_bits,
            solid_sides: surf.solid_sides,
            surface_flag: surf.surface_flag,
            comp_free,
            comp_surf,
        }
    }

    /// Assemble from arrays that were derived elsewhere. Used by the dump reader.
    #[must_use]
    pub fn from_parts(
        grid: Grid,
        solid: Vec<u8>,
        surf: SurfaceArrays,
        comp_free: Vec<i32>,
        comp_surf: Vec<i32>,
    ) -> Self {
        Self {
            grid,
            solid,
            surface_bits: surf.surface_bits,
            solid_sides: surf.solid_sides,
            surface_flag: surf.surface_flag,
            comp_free,
            comp_surf,
        }
    }

    /// The occupancy this volume was derived from.
    #[must_use]
    pub fn occupancy(&self) -> Occupancy {
        Occupancy {
            grid: self.grid,
            solid: self.solid.clone(),
        }
    }

    /// How many cells the volume holds.
    #[inline]
    #[must_use]
    pub fn cell_count(&self) -> usize {
        self.grid.cell_count()
    }

    /// Whether `c` is in bounds and not solid.
    #[inline]
    #[must_use]
    pub fn is_free_cell(&self, c: Cell) -> bool {
        self.grid.in_bounds(c) && self.solid[self.grid.cell_index(c)] == 0
    }

    /// Whether `c` is in bounds and carries the surface flag.
    #[inline]
    #[must_use]
    pub fn is_surface_cell(&self, c: Cell) -> bool {
        self.grid.in_bounds(c) && self.surface_flag[self.grid.cell_index(c)] != 0
    }

    /// Whether `c` may be stood in by a mover of this kind.
    #[inline]
    #[must_use]
    pub fn is_usable_cell(&self, c: Cell, surface_only: bool) -> bool {
        self.is_free_cell(c) && (!surface_only || self.is_surface_cell(c))
    }

    /// The component label at world point `p`, or -1 when off-lattice.
    #[must_use]
    pub fn component_at(&self, p: Vec3, surface_only: bool) -> i32 {
        let c = self.grid.world_to_cell(p);
        if !self.grid.in_bounds(c) {
            return NO_COMPONENT;
        }
        self.label_at_index(self.grid.cell_index(c), surface_only)
    }

    /// The component label of a cell by flat index.
    #[inline]
    #[must_use]
    pub fn label_at_index(&self, i: usize, surface_only: bool) -> i32 {
        if surface_only {
            self.comp_surf[i]
        } else {
            self.comp_free[i]
        }
    }

    /// A unit vector away from the solid a surface cell touches, or zero when it touches none.
    ///
    /// The sum of the outward directions of the solid face neighbours, normalised.
    #[must_use]
    pub fn surface_normal_hint(&self, c: Cell) -> Vec3 {
        if !self.grid.in_bounds(c) {
            return Vec3::ZERO;
        }
        let bits = self.surface_bits[self.grid.cell_index(c)];
        let mut acc = Vec3::ZERO;
        for (d, dir) in DIRS.iter().enumerate() {
            if bits & (1 << d) != 0 {
                acc.x -= dir[0] as f32;
                acc.y -= dir[1] as f32;
                acc.z -= dir[2] as f32;
            }
        }
        let len = (acc.x * acc.x + acc.y * acc.y + acc.z * acc.z).sqrt();
        if len <= 0.0 {
            Vec3::ZERO
        } else {
            Vec3::new(acc.x / len, acc.y / len, acc.z / len)
        }
    }

    /// The point where the segment from `inside` toward `toward` leaves the lattice, inset one cell
    /// so a snap around it stays in bounds. Returns `toward` when the segment never exits.
    #[must_use]
    pub fn exit_point(&self, inside: Vec3, toward: Vec3) -> Vec3 {
        let cs = self.grid.cell_size as f32;
        let lo = Vec3::new(
            self.grid.origin.x + cs,
            self.grid.origin.y + cs,
            self.grid.origin.z + cs,
        );
        let hi = Vec3::new(
            self.grid.origin.x + self.grid.dims[0] as f32 * cs - cs,
            self.grid.origin.y + self.grid.dims[1] as f32 * cs - cs,
            self.grid.origin.z + self.grid.dims[2] as f32 * cs - cs,
        );
        let d = Vec3::new(
            toward.x - inside.x,
            toward.y - inside.y,
            toward.z - inside.z,
        );
        let mut t_exit = 1.0f32;
        for a in 0..3 {
            let da = d.axis(a);
            if da.abs() < 0.000001 {
                continue;
            }
            let bound = if da > 0.0 { hi.axis(a) } else { lo.axis(a) };
            let t = (bound - inside.axis(a)) / da;
            if t >= 0.0 && t < t_exit {
                t_exit = t;
            }
        }
        let t = t_exit.clamp(0.0, 1.0);
        Vec3::new(inside.x + d.x * t, inside.y + d.y * t, inside.z + d.z * t)
    }

    /// A cell of component `comp`, chosen by scanning from `draw` and wrapping.
    ///
    /// **Deliberately NOT a reproduction of any other implementation.** The reference draws with the
    /// engine's own generator, and reproducing that generator to place a wander target is not worth
    /// the coupling. The caller supplies one integer from whatever generator it already has, and
    /// this walks forward from `draw % cell_count` to the first member. Every member is reachable,
    /// and the result is a pure function of `draw`, which is all a test needs.
    #[must_use]
    pub fn random_cell_in_component_from(&self, comp: i32, surface_only: bool, draw: u64) -> Cell {
        let n = self.cell_count();
        if n == 0 || comp < 0 {
            return NO_CELL;
        }
        let start = (draw % n as u64) as usize;
        for off in 0..n {
            let j = (start + off) % n;
            if self.label_at_index(j, surface_only) == comp {
                return self.grid.index_cell(j);
            }
        }
        NO_CELL
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn floor_world() -> Volume {
        // A solid floor plane at z = 0 under an 8x8x6 lattice.
        let g = Grid::new(Vec3::ZERO, 1.0, [8, 8, 6]);
        let mut o = Occupancy::empty(g);
        for y in 0..8 {
            for x in 0..8 {
                o.solid[g.cell_index([x, y, 0])] = 1;
            }
        }
        Volume::derive(&o, 1)
    }

    #[test]
    fn a_cell_above_the_floor_points_away_from_it() {
        let v = floor_world();
        let hint = v.surface_normal_hint([4, 4, 1]);
        assert_eq!(hint, Vec3::new(0.0, 0.0, 1.0));
    }

    #[test]
    fn a_cell_touching_nothing_has_no_hint() {
        let v = floor_world();
        assert_eq!(v.surface_normal_hint([4, 4, 5]), Vec3::ZERO);
    }

    #[test]
    fn free_and_surface_membership_differ() {
        let v = floor_world();
        assert!(
            v.is_usable_cell([4, 4, 1], true),
            "just above the floor is surface"
        );
        assert!(v.is_usable_cell([4, 4, 4], false), "high up is free");
        assert!(!v.is_usable_cell([4, 4, 4], true), "but not surface");
    }

    #[test]
    fn a_draw_always_lands_in_the_component() {
        let v = floor_world();
        let comp = v.component_at(Vec3::new(4.5, 4.5, 4.5), false);
        assert!(comp >= 0);
        for draw in [0u64, 1, 17, 999, u64::MAX] {
            let c = v.random_cell_in_component_from(comp, false, draw);
            assert_ne!(c, NO_CELL);
            assert_eq!(v.label_at_index(v.grid.cell_index(c), false), comp);
        }
    }

    #[test]
    fn an_empty_component_answers_no_cell() {
        let v = floor_world();
        assert_eq!(v.random_cell_in_component_from(99, false, 0), NO_CELL);
        assert_eq!(v.random_cell_in_component_from(-1, false, 0), NO_CELL);
    }

    #[test]
    fn exit_point_stops_inside_the_lattice() {
        let v = floor_world();
        let p = v.exit_point(Vec3::new(4.0, 4.0, 3.0), Vec3::new(400.0, 4.0, 3.0));
        assert!(p.x <= 7.0, "inset one cell from the far face, got {}", p.x);
        assert!(v.grid.contains_point(p));
    }

    #[test]
    fn exit_point_returns_a_target_that_never_leaves() {
        let v = floor_world();
        let target = Vec3::new(5.0, 5.0, 3.0);
        assert_eq!(v.exit_point(Vec3::new(4.0, 4.0, 3.0), target), target);
    }
}
