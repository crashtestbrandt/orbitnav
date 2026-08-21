//! SURFACE classification: which free cells touch solid, and how enclosed each one is.
//!
//! Three arrays, all derived from occupancy alone:
//!
//! | array | meaning |
//! |---|---|
//! | `surface_bits` | one bit per axis neighbour that is solid, in [`DIRS`] order |
//! | `solid_sides` | the popcount of `surface_bits`, 0..=6 |
//! | `surface_flag` | 1 when a free cell has any solid cell in its 26-neighbourhood |
//!
//! `surface_bits` and `solid_sides` describe the SIX face neighbours; `surface_flag` describes all
//! TWENTY-SIX, so a cell touching solid only at a corner is flagged but has no bits. That asymmetry
//! is deliberate in the original -- the flag decides what a surface-restricted mover may stand on,
//! while the bits and the popcount price how enclosed a cell is -- and it is reproduced here.
//!
//! # Two rules that decide the output at the edges
//!
//! - **Out of bounds counts as OPEN, never solid.** A cell on the lattice boundary reads its missing
//!   neighbour as free, so it is not flagged as surface for that side. The opposite convention
//!   would wrap the whole outer shell of the lattice in a phantom surface.
//! - **A cell outside the grown solid bounding box is left entirely at zero**, without scanning. With
//!   no solid cells at all that skip applies to every cell (see [`Occupancy::solid_bbox`]).
//!
//! # Parallelism
//!
//! Every cell's three outputs depend only on the occupancy array and the box, and never on another
//! cell's output, so splitting the range across workers is bit-identical to walking it serially by
//! construction. `chunking_is_identical` pins that.

use crate::grid::DIRS;
use crate::occupancy::Occupancy;
use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::Arc;

/// The three arrays SURFACE derives, each one byte per cell.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct SurfaceArrays {
    /// Bit per solid axis neighbour, in [`DIRS`] order: `1, 2, 4, 8, 16, 32`.
    pub surface_bits: Vec<u8>,
    /// Popcount of `surface_bits`, 0..=6.
    pub solid_sides: Vec<u8>,
    /// 1 when a free cell has any solid cell among its 26 neighbours.
    pub surface_flag: Vec<u8>,
}

impl SurfaceArrays {
    /// All-zero arrays for a grid of `n` cells.
    #[must_use]
    pub fn zeroed(n: usize) -> Self {
        Self {
            surface_bits: vec![0; n],
            solid_sides: vec![0; n],
            surface_flag: vec![0; n],
        }
    }
}

/// Classify every cell. `workers` of 0 or 1 walks the range serially.
///
/// The result does not depend on `workers`; see the module note on parallelism.
#[must_use]
pub fn derive_surface(occ: &Occupancy, workers: usize) -> SurfaceArrays {
    let n = occ.grid.cell_count();
    let mut out = SurfaceArrays::zeroed(n);
    if n == 0 {
        return out;
    }
    // No solid cells: every cell is outside the box, so every cell stays zero.
    let Some((lo, hi)) = occ.solid_bbox() else {
        return out;
    };

    if workers <= 1 || n < 8192 {
        classify_range(
            occ,
            lo,
            hi,
            0,
            n,
            &mut out.surface_bits,
            &mut out.solid_sides,
            &mut out.surface_flag,
        );
        return out;
    }

    // Split into chunks and hand each worker a private slice of every output array. Rust's own
    // borrow rules give the disjointness, so no synchronisation is needed inside a chunk.
    let chunk = n.div_ceil(workers);
    let occ_ref = occ;
    let mut bits = std::mem::take(&mut out.surface_bits);
    let mut sides = std::mem::take(&mut out.solid_sides);
    let mut flag = std::mem::take(&mut out.surface_flag);
    {
        let bits_chunks: Vec<&mut [u8]> = bits.chunks_mut(chunk).collect();
        let sides_chunks: Vec<&mut [u8]> = sides.chunks_mut(chunk).collect();
        let flag_chunks: Vec<&mut [u8]> = flag.chunks_mut(chunk).collect();
        std::thread::scope(|s| {
            for (k, ((b, sd), f)) in bits_chunks
                .into_iter()
                .zip(sides_chunks)
                .zip(flag_chunks)
                .enumerate()
            {
                let start = k * chunk;
                let end = (start + b.len()).min(n);
                s.spawn(move || {
                    classify_range_slices(occ_ref, lo, hi, start, end, b, sd, f);
                });
            }
        });
    }
    out.surface_bits = bits;
    out.solid_sides = sides;
    out.surface_flag = flag;
    out
}

#[allow(clippy::too_many_arguments)]
fn classify_range(
    occ: &Occupancy,
    lo: [i32; 3],
    hi: [i32; 3],
    start: usize,
    end: usize,
    bits: &mut [u8],
    sides: &mut [u8],
    flag: &mut [u8],
) {
    classify_range_slices(
        occ,
        lo,
        hi,
        start,
        end,
        &mut bits[start..end],
        &mut sides[start..end],
        &mut flag[start..end],
    );
}

/// Classify flat indices `start..end`, writing into slices whose element 0 IS index `start`.
#[allow(clippy::too_many_arguments)]
fn classify_range_slices(
    occ: &Occupancy,
    lo: [i32; 3],
    hi: [i32; 3],
    start: usize,
    end: usize,
    bits: &mut [u8],
    sides: &mut [u8],
    flag: &mut [u8],
) {
    let g = occ.grid;
    let (dx, dy, dz) = (g.dims[0], g.dims[1], g.dims[2]);
    let solid = &occ.solid;

    for i in start..end {
        if solid[i] != 0 {
            continue; // a solid cell is not a surface cell; its three outputs stay zero
        }
        let c = g.index_cell(i);
        // Outside the grown solid box there can be no solid neighbour, so skip the whole scan.
        if c[0] < lo[0] - 1
            || c[1] < lo[1] - 1
            || c[2] < lo[2] - 1
            || c[0] > hi[0] + 1
            || c[1] > hi[1] + 1
            || c[2] > hi[2] + 1
        {
            continue;
        }

        // The six face neighbours, in DIRS order. Bounds are checked on the STEPPED axis only, and
        // a step that leaves the lattice reads as OPEN.
        let mut mask: u8 = 0;
        for (d, dir) in DIRS.iter().enumerate() {
            let nx = c[0] + dir[0];
            let ny = c[1] + dir[1];
            let nz = c[2] + dir[2];
            if nx < 0 || ny < 0 || nz < 0 || nx >= dx || ny >= dy || nz >= dz {
                continue;
            }
            if solid[g.cell_index([nx, ny, nz])] != 0 {
                mask |= 1 << d;
            }
        }

        let w = i - start;
        bits[w] = mask;
        sides[w] = mask.count_ones() as u8;

        if mask != 0 {
            // Any face neighbour solid already answers the 26-neighbourhood question.
            flag[w] = 1;
            continue;
        }

        // No face neighbour is solid, so look at the remaining edge and corner neighbours. The scan
        // is over the clamped 3x3x3 block and INCLUDES the centre, which is free here and so cannot
        // change the answer.
        let x0 = (c[0] - 1).max(0);
        let x1 = (c[0] + 1).min(dx - 1);
        let y0 = (c[1] - 1).max(0);
        let y1 = (c[1] + 1).min(dy - 1);
        let z0 = (c[2] - 1).max(0);
        let z1 = (c[2] + 1).min(dz - 1);
        let mut found = false;
        'scan: for z in z0..=z1 {
            for y in y0..=y1 {
                for x in x0..=x1 {
                    if solid[g.cell_index([x, y, z])] != 0 {
                        found = true;
                        break 'scan;
                    }
                }
            }
        }
        if found {
            flag[w] = 1;
        }
    }
}

/// How many worker threads a derive of `n` cells should use, given a pool of `available`.
///
/// Kept beside the classifier so the split rule and the work it splits stay together.
#[must_use]
pub fn workers_for(n: usize, available: usize) -> usize {
    if n < 8192 {
        1
    } else {
        available.max(1)
    }
}

/// Shared progress counter, so a caller can report a long derive without locking.
#[derive(Clone, Debug, Default)]
pub struct Progress(Arc<AtomicUsize>);

impl Progress {
    /// A counter at zero.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }
    /// Add `k` to the count.
    pub fn add(&self, k: usize) {
        self.0.fetch_add(k, Ordering::Relaxed);
    }
    /// The count so far.
    #[must_use]
    pub fn get(&self) -> usize {
        self.0.load(Ordering::Relaxed)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::grid::Grid;
    use crate::real::Vec3;

    fn grid(d: [i32; 3]) -> Grid {
        Grid::new(Vec3::ZERO, 1.0, d)
    }

    fn with_solid(d: [i32; 3], cells: &[[i32; 3]]) -> Occupancy {
        let g = grid(d);
        let mut o = Occupancy::empty(g);
        for &c in cells {
            o.solid[g.cell_index(c)] = 1;
        }
        o
    }

    #[test]
    fn an_all_free_volume_derives_to_all_zero() {
        let o = Occupancy::empty(grid([12, 12, 12]));
        let s = derive_surface(&o, 1);
        assert!(s.surface_bits.iter().all(|&b| b == 0));
        assert!(s.solid_sides.iter().all(|&b| b == 0));
        assert!(s.surface_flag.iter().all(|&b| b == 0));
    }

    #[test]
    fn each_direction_sets_its_own_bit() {
        let g = grid([5, 5, 5]);
        let centre = [2, 2, 2];
        for (d, dir) in DIRS.iter().enumerate() {
            let nb = [centre[0] + dir[0], centre[1] + dir[1], centre[2] + dir[2]];
            let o = with_solid([5, 5, 5], &[nb]);
            let s = derive_surface(&o, 1);
            let i = g.cell_index(centre);
            assert_eq!(s.surface_bits[i], 1 << d, "direction {d}");
            assert_eq!(s.solid_sides[i], 1);
            assert_eq!(s.surface_flag[i], 1);
        }
    }

    #[test]
    fn out_of_bounds_reads_as_open() {
        // A free cell in the lattice corner, with one solid cell placed so exactly one FACE
        // neighbour is solid. Its three missing neighbours must contribute nothing.
        let g = grid([4, 4, 4]);
        let o = with_solid([4, 4, 4], &[[1, 0, 0]]);
        let s = derive_surface(&o, 1);
        let i = g.cell_index([0, 0, 0]);
        assert_eq!(s.surface_bits[i], 1 << 1, "only +x is solid");
        assert_eq!(s.solid_sides[i], 1, "the three off-lattice sides are open");
    }

    #[test]
    fn a_corner_only_neighbour_flags_without_bits() {
        let g = grid([5, 5, 5]);
        let o = with_solid([5, 5, 5], &[[3, 3, 3]]);
        let s = derive_surface(&o, 1);
        let i = g.cell_index([2, 2, 2]);
        assert_eq!(s.surface_bits[i], 0, "a diagonal is not a face neighbour");
        assert_eq!(s.solid_sides[i], 0);
        assert_eq!(
            s.surface_flag[i], 1,
            "but it is inside the 26-neighbourhood"
        );
    }

    #[test]
    fn a_solid_cell_derives_nothing() {
        let g = grid([5, 5, 5]);
        let o = with_solid([5, 5, 5], &[[2, 2, 2], [3, 2, 2]]);
        let s = derive_surface(&o, 1);
        let i = g.cell_index([2, 2, 2]);
        assert_eq!(
            (s.surface_bits[i], s.solid_sides[i], s.surface_flag[i]),
            (0, 0, 0)
        );
    }

    #[test]
    fn a_cell_far_outside_the_box_stays_zero() {
        let g = grid([20, 5, 5]);
        let o = with_solid([20, 5, 5], &[[1, 2, 2]]);
        let s = derive_surface(&o, 1);
        let i = g.cell_index([18, 2, 2]);
        assert_eq!(s.surface_flag[i], 0);
    }

    #[test]
    fn chunking_is_identical() {
        // A world big enough to cross the parallel threshold, with structure in it.
        let d = [40, 40, 40];
        let g = grid(d);
        let mut o = Occupancy::empty(g);
        for z in 0..d[2] {
            for y in 0..d[1] {
                for x in 0..d[0] {
                    // A hollow shell plus a diagonal bar: face, edge and corner cases everywhere.
                    let shell = x == 5 || x == 34 || y == 5 || y == 34;
                    let bar = x == y && z % 3 == 0;
                    if shell || bar {
                        o.solid[g.cell_index([x, y, z])] = 1;
                    }
                }
            }
        }
        let serial = derive_surface(&o, 1);
        for w in [2usize, 3, 7, 16] {
            assert_eq!(derive_surface(&o, w), serial, "{w} workers diverged");
        }
    }
}
