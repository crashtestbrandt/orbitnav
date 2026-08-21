//! The cell lattice: addressing, bounds and the world/cell mapping.
//!
//! A [`Grid`] is `dims` cubic cells of `cell_size` metres whose min corner sits at `origin`. Cells
//! are addressed by a FLAT index with x varying fastest:
//!
//! ```text
//! index = c.x + dims.x * (c.y + dims.y * c.z)
//! ```
//!
//! Every array in this crate -- occupancy, the three surface arrays, both component label arrays --
//! is indexed that way, and the derive and search loops walk them by stride arithmetic rather than
//! by calling back here. That layout is part of the format this crate must reproduce, so it is
//! stated once and never varied.

use crate::real::{n, w, Vec3};

/// A cell coordinate. Signed, because a query point may resolve outside the lattice.
pub type Cell = [i32; 3];

/// The sentinel a cell-returning query answers with when there is no such cell.
pub const NO_CELL: Cell = [-1, -1, -1];

/// The six axis neighbours, in the fixed order every bit mask and neighbour walk uses:
/// `-x, +x, -y, +y, -z, +z`, contributing bits `1, 2, 4, 8, 16, 32`.
pub const DIRS: [Cell; 6] = [
    [-1, 0, 0],
    [1, 0, 0],
    [0, -1, 0],
    [0, 1, 0],
    [0, 0, -1],
    [0, 0, 1],
];

/// `dims` cells of `cell_size` metres from a world-space min corner.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Grid {
    /// World position of the lattice's minimum corner.
    pub origin: Vec3,
    /// Edge length of one cubic cell, in metres.
    pub cell_size: f64,
    /// Cell counts along x, y and z.
    pub dims: Cell,
}

impl Grid {
    /// A lattice from its corner, cell size and extents.
    #[must_use]
    pub const fn new(origin: Vec3, cell_size: f64, dims: Cell) -> Self {
        Self {
            origin,
            cell_size,
            dims,
        }
    }

    /// How many cells the lattice holds. Zero if any extent is non-positive.
    #[inline]
    #[must_use]
    pub fn cell_count(&self) -> usize {
        if self.dims[0] <= 0 || self.dims[1] <= 0 || self.dims[2] <= 0 {
            return 0;
        }
        (self.dims[0] as usize) * (self.dims[1] as usize) * (self.dims[2] as usize)
    }

    /// Whether `c` addresses a cell of this lattice.
    #[inline]
    #[must_use]
    pub fn in_bounds(&self, c: Cell) -> bool {
        c[0] >= 0
            && c[1] >= 0
            && c[2] >= 0
            && c[0] < self.dims[0]
            && c[1] < self.dims[1]
            && c[2] < self.dims[2]
    }

    /// The flat index of `c`. Meaningless unless [`Grid::in_bounds`] holds.
    #[inline]
    #[must_use]
    pub fn cell_index(&self, c: Cell) -> usize {
        (c[0] + self.dims[0] * (c[1] + self.dims[1] * c[2])) as usize
    }

    /// The cell a flat index addresses. Inverse of [`Grid::cell_index`].
    #[inline]
    #[must_use]
    pub fn index_cell(&self, i: usize) -> Cell {
        let i = i as i32;
        let x = i % self.dims[0];
        let rest = i / self.dims[0];
        [x, rest % self.dims[1], rest / self.dims[1]]
    }

    /// The cell containing world point `p`.
    ///
    /// FLOORS rather than truncates. The two agree only for non-negative coordinates, and a lattice
    /// placed around a point of interest routinely has world coordinates on both sides of zero, so
    /// truncating here folds the two cells either side of an axis onto one another.
    #[inline]
    #[must_use]
    pub fn world_to_cell(&self, p: Vec3) -> Cell {
        [
            ((w(p.x) - w(self.origin.x)) / self.cell_size).floor() as i32,
            ((w(p.y) - w(self.origin.y)) / self.cell_size).floor() as i32,
            ((w(p.z) - w(self.origin.z)) / self.cell_size).floor() as i32,
        ]
    }

    /// The world position of the centre of cell `c`.
    #[inline]
    #[must_use]
    pub fn cell_center(&self, c: Cell) -> Vec3 {
        Vec3::new(
            n(w(self.origin.x) + (f64::from(c[0]) + 0.5) * self.cell_size),
            n(w(self.origin.y) + (f64::from(c[1]) + 0.5) * self.cell_size),
            n(w(self.origin.z) + (f64::from(c[2]) + 0.5) * self.cell_size),
        )
    }

    /// Whether world point `p` lies inside the lattice's box.
    #[inline]
    #[must_use]
    pub fn contains_point(&self, p: Vec3) -> bool {
        self.in_bounds(self.world_to_cell(p))
    }

    /// The flat-index steps for the six [`DIRS`], in the same order.
    #[inline]
    #[must_use]
    pub fn neighbour_strides(&self) -> [isize; 6] {
        let dx = self.dims[0] as isize;
        let plane = dx * self.dims[1] as isize;
        [-1, 1, -dx, dx, -plane, plane]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn g() -> Grid {
        Grid::new(Vec3::new(-4.0, 0.0, 2.0), 1.0, [5, 7, 3])
    }

    #[test]
    fn index_and_cell_round_trip_on_a_non_cubic_grid() {
        let g = g();
        assert_eq!(g.cell_count(), 5 * 7 * 3);
        for i in 0..g.cell_count() {
            assert_eq!(g.cell_index(g.index_cell(i)), i);
        }
    }

    #[test]
    fn x_varies_fastest() {
        let g = g();
        assert_eq!(g.cell_index([1, 0, 0]), 1);
        assert_eq!(g.cell_index([0, 1, 0]), 5);
        assert_eq!(g.cell_index([0, 0, 1]), 35);
    }

    #[test]
    fn strides_match_the_dirs_order() {
        let g = g();
        let s = g.neighbour_strides();
        for d in 0..6 {
            let from = [2, 3, 1];
            let to = [
                from[0] + DIRS[d][0],
                from[1] + DIRS[d][1],
                from[2] + DIRS[d][2],
            ];
            let expected = g.cell_index(to) as isize - g.cell_index(from) as isize;
            assert_eq!(s[d], expected, "stride {d}");
        }
    }

    #[test]
    fn world_to_cell_floors_across_the_origin() {
        let g = Grid::new(Vec3::ZERO, 1.0, [8, 8, 8]);
        // Truncation would fold -0.5 and +0.5 onto cell 0; flooring keeps them apart.
        assert_eq!(g.world_to_cell(Vec3::new(-0.5, 0.5, 0.5))[0], -1);
        assert_eq!(g.world_to_cell(Vec3::new(0.5, 0.5, 0.5))[0], 0);
    }

    #[test]
    fn cell_center_is_the_middle_of_the_cell() {
        let g = g();
        assert_eq!(g.cell_center([0, 0, 0]), Vec3::new(-3.5, 0.5, 2.5));
    }

    #[test]
    fn a_degenerate_extent_has_no_cells() {
        assert_eq!(Grid::new(Vec3::ZERO, 1.0, [0, 4, 4]).cell_count(), 0);
        assert_eq!(Grid::new(Vec3::ZERO, 1.0, [4, -1, 4]).cell_count(), 0);
    }
}
