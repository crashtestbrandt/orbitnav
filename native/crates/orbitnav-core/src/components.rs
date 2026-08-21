//! COMPONENTS: which free cells can reach each other, labelled twice.
//!
//! Two independent passes over one flood fill:
//!
//! | pass | array | membership |
//! |---|---|---|
//! | 0 | `comp_free` | free |
//! | 1 | `comp_surf` | free AND carrying the surface flag |
//!
//! Labels start at 0 in each pass, and a cell in no component keeps -1. A search uses them to reject
//! an unreachable pair before expanding anything, so the two arrays only ever have to agree with
//! themselves -- but the label NUMBERS are compared byte for byte against another implementation,
//! which makes the numbering itself part of the contract.
//!
//! # What fixes the numbering
//!
//! **The linear seed scan, and nothing else.** Labels are handed out in the order the scan finds the
//! next unlabelled member, walking flat index 0 upward, so component 0 is the one containing the
//! lowest-indexed member.
//!
//! Frontier discipline does NOT affect the output. A cell is labelled at the moment it is PUSHED,
//! not when it is popped, and every cell of a connected component receives the seed's label whatever
//! order the frontier is drained in. LIFO is used here because a `Vec` push/pop has better locality
//! than a queue, and `lifo_and_fifo_agree` pins the equivalence so a future change to the frontier
//! cannot be mistaken for a change to the format.

use crate::occupancy::Occupancy;
use crate::surface::SurfaceArrays;

/// The label a cell in no component carries.
pub const NO_COMPONENT: i32 = -1;

/// Label both graphs. Returns `(comp_free, comp_surf)`, one `i32` per cell.
#[must_use]
pub fn label_components(occ: &Occupancy, surf: &SurfaceArrays) -> (Vec<i32>, Vec<i32>) {
    let n = occ.grid.cell_count();
    let free = label_pass(occ, surf, n, false);
    let surface = label_pass(occ, surf, n, true);
    (free, surface)
}

/// One pass of the flood fill. `surface_pass` adds the surface-flag membership test.
fn label_pass(occ: &Occupancy, surf: &SurfaceArrays, n: usize, surface_pass: bool) -> Vec<i32> {
    let mut labels = vec![NO_COMPONENT; n];
    if n == 0 {
        return labels;
    }
    let g = occ.grid;
    let (dx, dy, dz) = (g.dims[0], g.dims[1], g.dims[2]);
    let strides = g.neighbour_strides();
    let solid = &occ.solid;
    let flag = &surf.surface_flag;

    let member = |i: usize| solid[i] == 0 && (!surface_pass || flag[i] != 0);

    let mut next_label: i32 = 0;
    let mut frontier: Vec<usize> = Vec::new();

    // The seed scan. Walking flat index 0 upward is what numbers the components.
    for seed in 0..n {
        if labels[seed] != NO_COMPONENT || !member(seed) {
            continue;
        }
        let label = next_label;
        next_label += 1;
        labels[seed] = label;
        frontier.push(seed);

        while let Some(i) = frontier.pop() {
            let c = g.index_cell(i);
            // The six face neighbours, in DIRS order, reached by stride. Bounds are checked on the
            // stepped axis alone, which is why the stride is safe to add.
            for (d, &stride) in strides.iter().enumerate() {
                let ok = match d {
                    0 => c[0] > 0,
                    1 => c[0] < dx - 1,
                    2 => c[1] > 0,
                    3 => c[1] < dy - 1,
                    4 => c[2] > 0,
                    _ => c[2] < dz - 1,
                };
                if !ok {
                    continue;
                }
                let j = (i as isize + stride) as usize;
                if labels[j] == NO_COMPONENT && member(j) {
                    // Labelled at PUSH, which is what makes the frontier order unobservable.
                    labels[j] = label;
                    frontier.push(j);
                }
            }
        }
    }
    labels
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::grid::Grid;
    use crate::real::Vec3;
    use crate::surface::derive_surface;

    fn world(d: [i32; 3], solid: &[[i32; 3]]) -> (Occupancy, SurfaceArrays) {
        let g = Grid::new(Vec3::ZERO, 1.0, d);
        let mut o = Occupancy::empty(g);
        for &c in solid {
            o.solid[g.cell_index(c)] = 1;
        }
        let s = derive_surface(&o, 1);
        (o, s)
    }

    #[test]
    fn an_empty_world_is_one_free_component_and_no_surface() {
        let (o, s) = world([6, 6, 6], &[]);
        let (free, surface) = label_components(&o, &s);
        assert!(free.iter().all(|&l| l == 0));
        assert!(surface.iter().all(|&l| l == NO_COMPONENT));
    }

    #[test]
    fn a_solid_cell_is_in_neither_graph() {
        let (o, s) = world([5, 5, 5], &[[2, 2, 2]]);
        let (free, surface) = label_components(&o, &s);
        let i = o.grid.cell_index([2, 2, 2]);
        assert_eq!(free[i], NO_COMPONENT);
        assert_eq!(surface[i], NO_COMPONENT);
    }

    #[test]
    fn a_sealed_room_is_its_own_component() {
        // A 1-cell cavity walled off inside a 7^3 lattice.
        let d = [7, 7, 7];
        let mut solid = Vec::new();
        for z in 2..=4 {
            for y in 2..=4 {
                for x in 2..=4 {
                    if [x, y, z] != [3, 3, 3] {
                        solid.push([x, y, z]);
                    }
                }
            }
        }
        let (o, s) = world(d, &solid);
        let (free, _) = label_components(&o, &s);
        let inside = free[o.grid.cell_index([3, 3, 3])];
        let outside = free[o.grid.cell_index([0, 0, 0])];
        assert_ne!(inside, outside, "the cavity must not join the outside");
        assert_eq!(free.iter().filter(|&&l| l == inside).count(), 1);
    }

    #[test]
    fn labels_are_numbered_by_the_linear_seed_scan() {
        // Two cavities at different depths: the one with the lower flat index must be labelled
        // first, whatever their positions look like geometrically.
        let d = [11, 5, 5];
        let mut solid = Vec::new();
        for x in 0..11 {
            for y in 0..5 {
                for z in 0..5 {
                    let wall = x == 5;
                    if wall {
                        solid.push([x, y, z]);
                    }
                }
            }
        }
        let (o, s) = world(d, &solid);
        let (free, _) = label_components(&o, &s);
        assert_eq!(
            free[o.grid.cell_index([0, 0, 0])],
            0,
            "lowest index seeds component 0"
        );
        assert_eq!(free[o.grid.cell_index([6, 0, 0])], 1);
    }

    #[test]
    fn the_surface_pass_restarts_numbering_at_zero() {
        let d = [9, 5, 5];
        let (o, s) = world(d, &[[2, 2, 2], [6, 2, 2]]);
        let (_, surface) = label_components(&o, &s);
        let mut seen: Vec<i32> = surface
            .iter()
            .copied()
            .filter(|&l| l != NO_COMPONENT)
            .collect();
        seen.sort_unstable();
        seen.dedup();
        assert_eq!(
            seen.first(),
            Some(&0),
            "the surface pass numbers from 0, not from the free pass's next label"
        );
    }

    /// A FIFO frontier over the same seed scan, to prove the discipline is unobservable.
    fn label_pass_fifo(occ: &Occupancy, surf: &SurfaceArrays, surface_pass: bool) -> Vec<i32> {
        let n = occ.grid.cell_count();
        let g = occ.grid;
        let (dx, dy, dz) = (g.dims[0], g.dims[1], g.dims[2]);
        let strides = g.neighbour_strides();
        let mut labels = vec![NO_COMPONENT; n];
        let member = |i: usize| occ.solid[i] == 0 && (!surface_pass || surf.surface_flag[i] != 0);
        let mut next_label = 0;
        let mut q: std::collections::VecDeque<usize> = std::collections::VecDeque::new();
        for seed in 0..n {
            if labels[seed] != NO_COMPONENT || !member(seed) {
                continue;
            }
            let label = next_label;
            next_label += 1;
            labels[seed] = label;
            q.push_back(seed);
            while let Some(i) = q.pop_front() {
                let c = g.index_cell(i);
                for (d, &stride) in strides.iter().enumerate() {
                    let ok = match d {
                        0 => c[0] > 0,
                        1 => c[0] < dx - 1,
                        2 => c[1] > 0,
                        3 => c[1] < dy - 1,
                        4 => c[2] > 0,
                        _ => c[2] < dz - 1,
                    };
                    if !ok {
                        continue;
                    }
                    let j = (i as isize + stride) as usize;
                    if labels[j] == NO_COMPONENT && member(j) {
                        labels[j] = label;
                        q.push_back(j);
                    }
                }
            }
        }
        labels
    }

    #[test]
    fn lifo_and_fifo_agree() {
        let d = [16, 12, 10];
        let mut solid = Vec::new();
        for x in 0..d[0] {
            for y in 0..d[1] {
                for z in 0..d[2] {
                    if (x * 7 + y * 13 + z * 5) % 11 == 0 {
                        solid.push([x, y, z]);
                    }
                }
            }
        }
        let (o, s) = world(d, &solid);
        let (free, surface) = label_components(&o, &s);
        assert_eq!(
            free,
            label_pass_fifo(&o, &s, false),
            "free graph depends on frontier order"
        );
        assert_eq!(
            surface,
            label_pass_fifo(&o, &s, true),
            "surface graph depends on frontier order"
        );
    }
}
