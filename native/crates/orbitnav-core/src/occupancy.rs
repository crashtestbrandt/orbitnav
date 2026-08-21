//! The occupancy grid: which cells hold solid material.
//!
//! This is the ONE input the derive phases read. Producing it is the caller's job, because deciding
//! whether a cell intersects geometry means asking a physics engine, and that is the part of the
//! bake this crate deliberately does not own.
//!
//! A cell nothing wrote is FREE. That is not an accident of initialisation: a caller may sample only
//! the neighbourhoods where geometry can possibly be, and leave the rest untouched. Everything here
//! treats a zero byte as "free" without asking how it got that way.

use crate::grid::{Cell, Grid};

/// A lattice plus one byte per cell: non-zero means solid.
#[derive(Clone, Debug)]
pub struct Occupancy {
    /// The lattice these bytes address.
    pub grid: Grid,
    /// One byte per cell, flat-indexed. Non-zero is solid.
    pub solid: Vec<u8>,
}

impl Occupancy {
    /// An all-free grid of the right size.
    #[must_use]
    pub fn empty(grid: Grid) -> Self {
        let n = grid.cell_count();
        Self {
            grid,
            solid: vec![0; n],
        }
    }

    /// Wrap caller-supplied occupancy. `None` if `solid` is not exactly one byte per cell.
    #[must_use]
    pub fn new(grid: Grid, solid: Vec<u8>) -> Option<Self> {
        if solid.len() != grid.cell_count() {
            return None;
        }
        Some(Self { grid, solid })
    }

    /// Whether the cell at flat index `i` is solid.
    #[inline]
    #[must_use]
    pub fn is_solid_index(&self, i: usize) -> bool {
        self.solid[i] != 0
    }

    /// The inclusive cell-space bounding box of the solid cells, or `None` when none are solid.
    ///
    /// The SURFACE pass uses this to skip open space: a free cell more than one cell outside the box
    /// has no solid neighbour by construction, so its expensive neighbourhood scan can be skipped.
    ///
    /// `None` means EVERY cell skips that scan, which is the whole point. The reference
    /// implementation reaches the same state by leaving an inverted sentinel box in place, so that
    /// every comparison against it fails; reading `None` as "unbounded" instead would invert the
    /// meaning and scan the entire grid.
    ///
    /// Recomputed here rather than accepted as a parameter. The box is exactly min/max over the
    /// cells this array marks solid, so deriving it from the array cannot disagree with the array --
    /// and a parameter could. It costs one linear pass, against a derive that walks the same cells
    /// with far more work per cell.
    #[must_use]
    pub fn solid_bbox(&self) -> Option<(Cell, Cell)> {
        let mut lo = [i32::MAX; 3];
        let mut hi = [i32::MIN; 3];
        let mut any = false;
        for (i, &b) in self.solid.iter().enumerate() {
            if b == 0 {
                continue;
            }
            any = true;
            let c = self.grid.index_cell(i);
            for a in 0..3 {
                if c[a] < lo[a] {
                    lo[a] = c[a];
                }
                if c[a] > hi[a] {
                    hi[a] = c[a];
                }
            }
        }
        if any {
            Some((lo, hi))
        } else {
            None
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::real::Vec3;

    fn grid() -> Grid {
        Grid::new(Vec3::ZERO, 1.0, [6, 5, 4])
    }

    #[test]
    fn an_all_free_grid_has_no_bbox() {
        assert_eq!(Occupancy::empty(grid()).solid_bbox(), None);
    }

    #[test]
    fn one_solid_cell_gives_a_degenerate_box() {
        let g = grid();
        let mut o = Occupancy::empty(g);
        o.solid[g.cell_index([2, 3, 1])] = 1;
        assert_eq!(o.solid_bbox(), Some(([2, 3, 1], [2, 3, 1])));
    }

    #[test]
    fn the_box_matches_a_brute_force_min_max() {
        let g = grid();
        let mut o = Occupancy::empty(g);
        for c in [[1, 1, 1], [4, 2, 0], [0, 4, 3]] {
            o.solid[g.cell_index(c)] = 1;
        }
        assert_eq!(o.solid_bbox(), Some(([0, 1, 0], [4, 4, 3])));
    }

    #[test]
    fn a_wrong_sized_array_is_refused() {
        assert!(Occupancy::new(grid(), vec![0; 3]).is_none());
        assert!(Occupancy::new(grid(), vec![0; grid().cell_count()]).is_some());
    }
}
