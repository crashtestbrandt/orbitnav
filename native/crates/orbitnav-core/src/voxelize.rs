//! Occupancy from geometry: the cells a cell-sized box query would find touching something.
//!
//! [`Occupancy`] is the one input the derive reads. A caller can fill it by asking its physics engine,
//! once per cell, whether a box the size of the cell overlaps anything. That is a pure function of the
//! geometry, so this module answers it from the geometry directly: the caller hands over triangles and
//! convex shapes once, and the whole grid is filled on a worker.
//!
//! # When a cell is solid
//!
//! A cell is solid when a box the size of the cell, centred on the cell's centre, overlaps a piece of
//! geometry. **Touching counts**, to within [`CONTACT_TOLERANCE`]: a triangle lying exactly on a cell's
//! face makes that cell solid.
//!
//! An engine working in `f32` decides an exact contact by its own rounding, so where geometry lies exactly
//! on cell faces -- axis-aligned plates on a whole-metre grid -- its per-cell answers can differ from plate
//! to plate. This module counts every such contact the same way. The tolerance absorbs this module's own
//! `f64` rounding and is smaller than the gap at which Jolt Physics stops reporting a face contact (a
//! hundredth of a millimetre, measured).
//!
//! Three rules reproduce what a physics engine's shape query reports, rather than the bare geometric
//! answer:
//!
//! - **The cell box may be rounded** (`cell_rounding`). Its faces stay where they are and its edges and
//!   corners are swept by that radius, which is how an engine that keeps a convex radius on every shape
//!   builds its query box. A convex piece may be rounded the same way. Two rounded shapes overlap when
//!   their unrounded cores are within the sum of the two radii.
//! - **A one-sided triangle claims only the cells whose centre is on its front side**, or exactly on its
//!   plane. An engine that ignores back faces in shape queries reports exactly those cells. Which side is
//!   the front comes from the mesh's [`Winding`]. A double-sided triangle claims cells on both sides.
//! - **A triangle with an edge shorter than [`WELD_DISTANCE`] is dropped**, as an engine that welds nearby
//!   vertices when it builds a mesh drops it.
//!
//! A triangle mesh is a SURFACE. A cell buried inside a closed mesh touches no triangle and stays free. A
//! convex piece is SOLID throughout, so every cell inside it is solid.
//!
//! # Regions
//!
//! A caller that knows where geometry can be may pass spheres ([`Region`]). A cell whose centre is outside
//! every sphere stays free, whatever it touches. The test is the one a cell-by-cell sampler over the same
//! spheres makes: the `f32` centre [`Grid::cell_center`] gives, the squared distance accumulated at `f32`,
//! compared with the radius squared at `f64`. The boundary cells therefore match such a sampler's.
//!
//! # Cost
//!
//! - A triangle visits only the cells its plane passes within half a cell of, found column by column along
//!   the normal's dominant axis, so a large tilted triangle costs its area and not its bounding box.
//! - The exact test per cell is a separating-axis test. With rounding, a triangle that reaches only into
//!   an edge or corner region of the cell box falls back to GJK.
//! - A convex piece tests every cell in its bounds with GJK. Convex pieces are expected to be few and small
//!   next to the meshes.
//!
//! The work runs on the thread that calls [`voxelize`]; [`crate::jobs::Pool::submit_voxelize`] puts it on
//! a worker.

use crate::grid::{Cell, Grid};
use crate::occupancy::Occupancy;
use crate::real::{w, Vec3};

/// A point or direction at `f64`.
pub type P3 = [f64; 3];

/// A triangle with an edge shorter than this, in world units, is dropped.
pub const WELD_DISTANCE: f64 = 1.0e-4;

/// Two shapes closer than this, in world units, touch.
pub const CONTACT_TOLERANCE: f64 = 1.0e-6;

/// GJK gives up after this many refinements and reports no overlap. The shapes here converge in well
/// under ten.
const GJK_MAX_ITERATIONS: usize = 64;

/// GJK stops when a refinement improves the squared distance by less than this fraction of it.
const GJK_RELATIVE_TOLERANCE: f64 = 1.0e-10;

#[inline]
fn add(a: P3, b: P3) -> P3 {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}

#[inline]
fn sub(a: P3, b: P3) -> P3 {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

#[inline]
fn scale(a: P3, s: f64) -> P3 {
    [a[0] * s, a[1] * s, a[2] * s]
}

#[inline]
fn neg(a: P3) -> P3 {
    [-a[0], -a[1], -a[2]]
}

#[inline]
fn dot(a: P3, b: P3) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

#[inline]
fn cross(a: P3, b: P3) -> P3 {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

/// `+1` for zero and above, `-1` below. A support point must be a vertex, so zero picks a side.
#[inline]
fn side(v: f64) -> f64 {
    if v < 0.0 {
        -1.0
    } else {
        1.0
    }
}

#[inline]
fn widen(v: Vec3) -> P3 {
    [w(v.x), w(v.y), w(v.z)]
}

/// An affine map: `world = rows * local + origin`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Affine {
    /// The linear part, one row per world axis.
    pub rows: [P3; 3],
    /// The translation.
    pub origin: P3,
}

impl Affine {
    /// The map that changes nothing.
    pub const IDENTITY: Affine = Affine {
        rows: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
        origin: [0.0, 0.0, 0.0],
    };

    /// A pure translation.
    #[must_use]
    pub const fn translation(origin: P3) -> Self {
        Self {
            rows: Self::IDENTITY.rows,
            origin,
        }
    }

    /// A point, mapped.
    #[inline]
    #[must_use]
    pub fn point(&self, p: P3) -> P3 {
        add(self.vector(p), self.origin)
    }

    /// A direction, mapped by the linear part only.
    #[inline]
    #[must_use]
    pub fn vector(&self, v: P3) -> P3 {
        [
            dot(self.rows[0], v),
            dot(self.rows[1], v),
            dot(self.rows[2], v),
        ]
    }

    /// The transpose of the linear part applied to `d`. A world-space support direction becomes a
    /// local one this way, for any linear part, scaled or sheared.
    #[inline]
    fn transpose_vector(&self, d: P3) -> P3 {
        [
            self.rows[0][0] * d[0] + self.rows[1][0] * d[1] + self.rows[2][0] * d[2],
            self.rows[0][1] * d[0] + self.rows[1][1] * d[1] + self.rows[2][1] * d[2],
            self.rows[0][2] * d[0] + self.rows[1][2] * d[1] + self.rows[2][2] * d[2],
        ]
    }

    /// The longest image of a local unit axis. A radius in local units is scaled by this, which is exact
    /// for a uniform scale and an over-estimate otherwise.
    fn largest_axis_scale(&self) -> f64 {
        (0..3)
            .map(|c| {
                let col = [self.rows[0][c], self.rows[1][c], self.rows[2][c]];
                dot(col, col).sqrt()
            })
            .fold(0.0, f64::max)
    }
}

/// Which way a mesh's vertices run when a triangle is viewed from its front.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Winding {
    /// Counter-clockwise from the front: the front is the side `(b - a) x (c - a)` points to.
    CounterClockwise,
    /// Clockwise from the front: the front is the side `(c - a) x (b - a)` points to.
    Clockwise,
}

/// Which sides of a mesh's triangles claim cells.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Sides {
    /// Only cells whose centre is on the front side, or on the plane.
    Front(Winding),
    /// Cells on either side.
    Both,
}

/// A sphere bounding where geometry may be. See the module notes on regions.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Region {
    /// The centre.
    pub centre: Vec3,
    /// The radius.
    pub radius: f32,
}

struct Mesh {
    vertices: Vec<Vec3>,
    map: Affine,
    sides: Sides,
}

/// A convex core. The piece it belongs to is this core swept by a radius.
#[derive(Clone, Debug)]
enum Core {
    Point(P3),
    Segment(P3, P3),
    Box {
        map: Affine,
        half: P3,
    },
    Cylinder {
        map: Affine,
        radius: f64,
        half_height: f64,
    },
    Hull(Vec<P3>),
}

struct Convex {
    core: Core,
    radius: f64,
}

/// Everything a volume is voxelized against: triangle meshes and convex pieces, each with the map that
/// places it in world space.
///
/// Adding a piece copies its data and does nothing else. Mapping, welding and culling all happen inside
/// [`voxelize`], so a caller filling this on a thread it cares about pays for the copy alone.
#[derive(Default)]
pub struct Geometry {
    meshes: Vec<Mesh>,
    convexes: Vec<Convex>,
}

impl std::fmt::Debug for Geometry {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Geometry")
            .field("meshes", &self.meshes.len())
            .field("triangles", &self.triangle_count())
            .field("convexes", &self.convexes.len())
            .finish()
    }
}

impl Geometry {
    /// No geometry.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// A triangle mesh: `vertices` in threes, placed by `map`. A trailing partial triangle is ignored.
    pub fn add_mesh(&mut self, vertices: Vec<Vec3>, map: Affine, sides: Sides) {
        if vertices.len() >= 3 {
            self.meshes.push(Mesh {
                vertices,
                map,
                sides,
            });
        }
    }

    /// A box of half extents `half`, its edges and corners rounded by `rounding`, placed by `map`.
    pub fn add_box(&mut self, half: P3, rounding: f64, map: Affine) {
        let r = rounding.clamp(0.0, half[0].min(half[1]).min(half[2]).max(0.0));
        self.convexes.push(Convex {
            core: Core::Box {
                map,
                half: [half[0] - r, half[1] - r, half[2] - r],
            },
            radius: r * map.largest_axis_scale(),
        });
    }

    /// A sphere of `radius` about the local origin, placed by `map`.
    pub fn add_sphere(&mut self, radius: f64, map: Affine) {
        self.convexes.push(Convex {
            core: Core::Point(map.point([0.0; 3])),
            radius: radius.max(0.0) * map.largest_axis_scale(),
        });
    }

    /// A capsule along the local y axis: a segment reaching `segment_half` either side of the origin,
    /// swept by `radius`, placed by `map`.
    pub fn add_capsule(&mut self, radius: f64, segment_half: f64, map: Affine) {
        let h = segment_half.max(0.0);
        self.convexes.push(Convex {
            core: Core::Segment(map.point([0.0, -h, 0.0]), map.point([0.0, h, 0.0])),
            radius: radius.max(0.0) * map.largest_axis_scale(),
        });
    }

    /// A cylinder along the local y axis, its rims rounded by `rounding`, placed by `map`.
    pub fn add_cylinder(&mut self, radius: f64, half_height: f64, rounding: f64, map: Affine) {
        let r = rounding.clamp(0.0, radius.min(half_height).max(0.0));
        self.convexes.push(Convex {
            core: Core::Cylinder {
                map,
                radius: (radius - r).max(0.0),
                half_height: (half_height - r).max(0.0),
            },
            radius: r * map.largest_axis_scale(),
        });
    }

    /// The convex hull of `points`, placed by `map`. Not rounded.
    pub fn add_hull(&mut self, points: &[Vec3], map: Affine) {
        if points.is_empty() {
            return;
        }
        let pts = points.iter().map(|p| map.point(widen(*p))).collect();
        self.convexes.push(Convex {
            core: Core::Hull(pts),
            radius: 0.0,
        });
    }

    /// How many triangles the meshes hold, before welding and culling.
    #[must_use]
    pub fn triangle_count(&self) -> usize {
        self.meshes.iter().map(|m| m.vertices.len() / 3).sum()
    }

    /// How many convex pieces have been added.
    #[must_use]
    pub fn convex_count(&self) -> usize {
        self.convexes.len()
    }

    /// Whether nothing has been added.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.meshes.is_empty() && self.convexes.is_empty()
    }
}

/// Fill a grid from `geometry`. See the module notes for what makes a cell solid.
///
/// `cell_rounding` is clamped to half a cell. An empty `regions` places no restriction.
#[must_use]
pub fn voxelize(
    grid: &Grid,
    geometry: &Geometry,
    regions: &[Region],
    cell_rounding: f64,
) -> Occupancy {
    let mut occ = Occupancy::empty(*grid);
    if occ.solid.is_empty() {
        return occ;
    }
    let mut v = Voxelizer::new(grid, regions, cell_rounding);
    for mesh in &geometry.meshes {
        for tri in mesh.vertices.chunks_exact(3) {
            if let Some(t) = Triangle::place(tri, &mesh.map, mesh.sides) {
                v.triangle(&t, &mut occ.solid);
            }
        }
    }
    for piece in &geometry.convexes {
        v.convex(piece, &mut occ.solid);
    }
    occ
}

/// A triangle in world space, in counter-clockwise order, with its bounds.
struct Triangle {
    a: P3,
    b: P3,
    c: P3,
    /// `(b - a) x (c - a)`, unnormalised: the front side for a one-sided triangle.
    normal: P3,
    one_sided: bool,
    lo: P3,
    hi: P3,
}

impl Triangle {
    /// Map three local vertices to world space, oriented so the front is the counter-clockwise side.
    /// `None` for a triangle welding would drop.
    fn place(tri: &[Vec3], map: &Affine, sides: Sides) -> Option<Self> {
        let a = map.point(widen(tri[0]));
        let mut b = map.point(widen(tri[1]));
        let mut c = map.point(widen(tri[2]));
        let weld2 = WELD_DISTANCE * WELD_DISTANCE;
        let ab = sub(b, a);
        let bc = sub(c, b);
        let ca = sub(a, c);
        if dot(ab, ab) < weld2 || dot(bc, bc) < weld2 || dot(ca, ca) < weld2 {
            return None;
        }
        let one_sided = match sides {
            Sides::Front(Winding::Clockwise) => {
                std::mem::swap(&mut b, &mut c);
                true
            }
            Sides::Front(Winding::CounterClockwise) => true,
            Sides::Both => false,
        };
        let mut lo = a;
        let mut hi = a;
        for p in [b, c] {
            for i in 0..3 {
                lo[i] = lo[i].min(p[i]);
                hi[i] = hi[i].max(p[i]);
            }
        }
        Some(Self {
            a,
            b,
            c,
            normal: cross(sub(b, a), sub(c, a)),
            one_sided,
            lo,
            hi,
        })
    }
}

struct Voxelizer<'a> {
    grid: &'a Grid,
    origin: P3,
    cell: f64,
    /// Half a cell: the unrounded box's half extent.
    half: f64,
    rounding: f64,
    /// The box's reach and its rounding, each grown by [`CONTACT_TOLERANCE`].
    half_reach: f64,
    rounding_reach: f64,
    regions: &'a [Region],
    /// Per cell, when regions apply: 0 not yet asked, 1 inside a region, 2 outside every region.
    region_state: Vec<u8>,
}

impl<'a> Voxelizer<'a> {
    fn new(grid: &'a Grid, regions: &'a [Region], cell_rounding: f64) -> Self {
        let half = grid.cell_size * 0.5;
        let rounding = cell_rounding.clamp(0.0, half);
        Self {
            grid,
            origin: widen(grid.origin),
            cell: grid.cell_size,
            half,
            rounding,
            half_reach: half + CONTACT_TOLERANCE,
            rounding_reach: rounding + CONTACT_TOLERANCE,
            regions,
            region_state: if regions.is_empty() {
                Vec::new()
            } else {
                vec![0; grid.cell_count()]
            },
        }
    }

    /// The inclusive cell range along `axis` whose boxes can touch `lo..=hi`, clamped to the lattice.
    /// Generous by up to a cell at each end; the exact test decides.
    fn range(&self, axis: usize, lo: f64, hi: f64) -> (i32, i32) {
        let o = self.origin[axis];
        let first = ((lo - o) / self.cell - 1.0).floor().max(0.0);
        let last = ((hi - o) / self.cell)
            .ceil()
            .min(f64::from(self.grid.dims[axis] - 1));
        if first.is_nan() || last.is_nan() {
            return (0, -1);
        }
        (first as i32, last as i32)
    }

    /// The cell centre the query box sits at: the `f32` centre the grid reports.
    fn centre(&self, c: Cell) -> P3 {
        widen(self.grid.cell_center(c))
    }

    fn in_region(&mut self, index: usize, c: Cell) -> bool {
        if self.regions.is_empty() {
            return true;
        }
        match self.region_state[index] {
            1 => true,
            2 => false,
            _ => {
                let cc = self.grid.cell_center(c);
                let inside = self
                    .regions
                    .iter()
                    .any(|r| w(cc.distance_squared_to(r.centre)) <= w(r.radius) * w(r.radius));
                self.region_state[index] = if inside { 1 } else { 2 };
                inside
            }
        }
    }

    fn triangle(&mut self, t: &Triangle, solid: &mut [u8]) {
        let rx = self.range(0, t.lo[0], t.hi[0]);
        let ry = self.range(1, t.lo[1], t.hi[1]);
        let rz = self.range(2, t.lo[2], t.hi[2]);
        if rx.0 > rx.1 || ry.0 > ry.1 || rz.0 > rz.1 {
            return;
        }
        let n = t.normal;
        let an = [n[0].abs(), n[1].abs(), n[2].abs()];
        let k = if an[0] >= an[1] && an[0] >= an[2] {
            0
        } else if an[1] >= an[2] {
            1
        } else {
            2
        };
        let ranges = [rx, ry, rz];
        if an[k] == 0.0 {
            // No plane to walk along; welding makes this rare. Test the whole box.
            for z in rz.0..=rz.1 {
                for y in ry.0..=ry.1 {
                    for x in rx.0..=rx.1 {
                        self.test_triangle(t, [x, y, z], solid);
                    }
                }
            }
            return;
        }
        let (i, j) = match k {
            0 => (1, 2),
            1 => (0, 2),
            _ => (0, 1),
        };
        // A cell box meets the plane when its centre is within this of it, measured along `n`.
        let reach = self.half_reach * (an[0] + an[1] + an[2]);
        let d = dot(n, t.a);
        let ok = self.origin[k];
        for ci in ranges[i].0..=ranges[i].1 {
            let pi = self.origin[i] + (f64::from(ci) + 0.5) * self.cell;
            for cj in ranges[j].0..=ranges[j].1 {
                let pj = self.origin[j] + (f64::from(cj) + 0.5) * self.cell;
                let rest = d - n[i] * pi - n[j] * pj;
                let s1 = (rest - reach) / n[k];
                let s2 = (rest + reach) / n[k];
                let (lo, hi) = if s1 <= s2 { (s1, s2) } else { (s2, s1) };
                let k0 = ((lo - ok) / self.cell - 0.5)
                    .floor()
                    .max(f64::from(ranges[k].0));
                let k1 = ((hi - ok) / self.cell - 0.5)
                    .ceil()
                    .min(f64::from(ranges[k].1));
                if k0.is_nan() || k1.is_nan() || k0 > k1 {
                    continue;
                }
                for ck in (k0 as i32)..=(k1 as i32) {
                    let mut c = [0; 3];
                    c[i] = ci;
                    c[j] = cj;
                    c[k] = ck;
                    self.test_triangle(t, c, solid);
                }
            }
        }
    }

    fn test_triangle(&mut self, t: &Triangle, c: Cell, solid: &mut [u8]) {
        let index = self.grid.cell_index(c);
        if solid[index] != 0 || !self.in_region(index, c) {
            return;
        }
        let centre = self.centre(c);
        if t.one_sided && dot(t.normal, sub(centre, t.a)) < 0.0 {
            return;
        }
        if self.triangle_touches_cell(sub(t.a, centre), sub(t.b, centre), sub(t.c, centre)) {
            solid[index] = 1;
        }
    }

    /// Whether a triangle, given relative to the cell centre, touches the (possibly rounded) cell box.
    ///
    /// The shape tested is the core box `half - rounding` swept by `rounding + CONTACT_TOLERANCE`. The box
    /// of half extent `half + CONTACT_TOLERANCE` contains it, and so does each of the three boxes grown
    /// that far along one axis only, so the separating-axis test settles every triangle that misses the
    /// first box or reaches one of the other three. GJK decides the rest.
    fn triangle_touches_cell(&self, a: P3, b: P3, c: P3) -> bool {
        let h = self.half_reach;
        if !triangle_overlaps_box(a, b, c, [h, h, h]) {
            return false;
        }
        let s = self.half - self.rounding;
        if triangle_overlaps_box(a, b, c, [h, s, s])
            || triangle_overlaps_box(a, b, c, [s, h, s])
            || triangle_overlaps_box(a, b, c, [s, s, h])
        {
            return true;
        }
        within(
            &BoxCore([s, s, s]),
            &TriangleCore(a, b, c),
            self.rounding_reach,
        )
    }

    fn convex(&mut self, piece: &Convex, solid: &mut [u8]) {
        let mut lo = [0.0; 3];
        let mut hi = [0.0; 3];
        for axis in 0..3 {
            let mut d = [0.0; 3];
            d[axis] = 1.0;
            hi[axis] = piece.core.support(d)[axis] + piece.radius;
            d[axis] = -1.0;
            lo[axis] = piece.core.support(d)[axis] - piece.radius;
        }
        let rx = self.range(0, lo[0], hi[0]);
        let ry = self.range(1, lo[1], hi[1]);
        let rz = self.range(2, lo[2], hi[2]);
        let core = BoxCore([self.half - self.rounding; 3]);
        let reach = self.rounding_reach + piece.radius;
        for z in rz.0..=rz.1 {
            for y in ry.0..=ry.1 {
                for x in rx.0..=rx.1 {
                    let c = [x, y, z];
                    let index = self.grid.cell_index(c);
                    if solid[index] != 0 || !self.in_region(index, c) {
                        continue;
                    }
                    let shifted = Shifted {
                        core: &piece.core,
                        by: self.centre(c),
                    };
                    if within(&core, &shifted, reach) {
                        solid[index] = 1;
                    }
                }
            }
        }
    }
}

/// Separating-axis overlap of a triangle and an origin-centred box of half extents `e`. Touching
/// overlaps. Thirteen axes: the box's three, the triangle's normal, and the nine edge cross products.
fn triangle_overlaps_box(v0: P3, v1: P3, v2: P3, e: P3) -> bool {
    for axis in 0..3 {
        let mn = v0[axis].min(v1[axis]).min(v2[axis]);
        let mx = v0[axis].max(v1[axis]).max(v2[axis]);
        if mn > e[axis] || mx < -e[axis] {
            return false;
        }
    }
    let edges = [sub(v1, v0), sub(v2, v1), sub(v0, v2)];
    for f in edges {
        for axis in 0..3 {
            let mut u = [0.0; 3];
            u[axis] = 1.0;
            let a = cross(u, f);
            let p0 = dot(a, v0);
            let p1 = dot(a, v1);
            let p2 = dot(a, v2);
            let rad = e[0] * a[0].abs() + e[1] * a[1].abs() + e[2] * a[2].abs();
            if p0.min(p1).min(p2) > rad || p0.max(p1).max(p2) < -rad {
                return false;
            }
        }
    }
    let n = cross(edges[0], edges[1]);
    let rad = e[0] * n[0].abs() + e[1] * n[1].abs() + e[2] * n[2].abs();
    dot(n, v0).abs() <= rad
}

/// The farthest point of a convex set in a direction.
trait Support {
    fn support(&self, d: P3) -> P3;
}

/// An origin-centred box.
struct BoxCore(P3);

impl Support for BoxCore {
    fn support(&self, d: P3) -> P3 {
        [
            side(d[0]) * self.0[0],
            side(d[1]) * self.0[1],
            side(d[2]) * self.0[2],
        ]
    }
}

struct TriangleCore(P3, P3, P3);

impl Support for TriangleCore {
    fn support(&self, d: P3) -> P3 {
        let (da, db, dc) = (dot(self.0, d), dot(self.1, d), dot(self.2, d));
        if da >= db && da >= dc {
            self.0
        } else if db >= dc {
            self.1
        } else {
            self.2
        }
    }
}

impl Support for Core {
    fn support(&self, d: P3) -> P3 {
        match self {
            Core::Point(p) => *p,
            Core::Segment(a, b) => {
                if dot(*a, d) >= dot(*b, d) {
                    *a
                } else {
                    *b
                }
            }
            Core::Box { map, half } => {
                let l = map.transpose_vector(d);
                map.point([
                    side(l[0]) * half[0],
                    side(l[1]) * half[1],
                    side(l[2]) * half[2],
                ])
            }
            Core::Cylinder {
                map,
                radius,
                half_height,
            } => {
                let l = map.transpose_vector(d);
                let across = (l[0] * l[0] + l[2] * l[2]).sqrt();
                let (x, z) = if across > 0.0 {
                    (radius * l[0] / across, radius * l[2] / across)
                } else {
                    (0.0, 0.0)
                };
                map.point([x, side(l[1]) * half_height, z])
            }
            Core::Hull(points) => {
                let mut best = points[0];
                let mut best_dot = dot(best, d);
                for p in &points[1..] {
                    let pd = dot(*p, d);
                    if pd > best_dot {
                        best = *p;
                        best_dot = pd;
                    }
                }
                best
            }
        }
    }
}

/// A core seen from a cell centre.
struct Shifted<'a> {
    core: &'a Core,
    by: P3,
}

impl Support for Shifted<'_> {
    fn support(&self, d: P3) -> P3 {
        sub(self.core.support(d), self.by)
    }
}

/// Whether two convex sets are within `radius` of each other: GJK on their Minkowski difference, stopping
/// as soon as the answer is known either way.
fn within<A: Support + ?Sized, B: Support + ?Sized>(a: &A, b: &B, radius: f64) -> bool {
    gjk_distance_sq(a, b, radius) <= radius * radius
}

/// The squared distance between two convex sets, or an early answer. Returns some value no greater than
/// `stop_within` squared as soon as the distance is known to be that small, and `f64::INFINITY` as soon as
/// it is known to be larger. Pass `0.0` for the plain distance.
fn gjk_distance_sq<A: Support + ?Sized, B: Support + ?Sized>(
    a: &A,
    b: &B,
    stop_within: f64,
) -> f64 {
    let stop_sq = stop_within * stop_within;
    let early_out = stop_within > 0.0;
    let seed = [1.0, 0.0, 0.0];
    let mut v = sub(a.support(seed), b.support(neg(seed)));
    // The starting point is a vertex of the simplex, so every reduction can only move closer.
    let mut simplex = Simplex::default();
    simplex.push(v);
    for _ in 0..GJK_MAX_ITERATIONS {
        let vv = dot(v, v);
        if vv <= stop_sq {
            return vv;
        }
        let wp = sub(a.support(neg(v)), b.support(v));
        let vw = dot(v, wp);
        // `vw / |v|` is a lower bound on the distance.
        if early_out && vw > 0.0 && vw * vw > stop_sq * vv {
            return f64::INFINITY;
        }
        if vv - vw <= GJK_RELATIVE_TOLERANCE * vv || !simplex.push(wp) {
            return if early_out { f64::INFINITY } else { vv };
        }
        match simplex.reduce() {
            None => return 0.0,
            Some(next) => {
                if dot(next, next) >= vv {
                    return if early_out { f64::INFINITY } else { vv };
                }
                v = next;
            }
        }
    }
    if early_out {
        f64::INFINITY
    } else {
        dot(v, v)
    }
}

#[derive(Default)]
struct Simplex {
    p: [P3; 4],
    n: usize,
}

impl Simplex {
    /// Add a vertex. False if it is already there, which means GJK has stopped making progress.
    fn push(&mut self, v: P3) -> bool {
        if self.p[..self.n].contains(&v) || self.n == 4 {
            return false;
        }
        self.p[self.n] = v;
        self.n += 1;
        true
    }

    /// Shrink to the smallest face holding the point nearest the origin and return that point. `None`
    /// when the origin is inside the tetrahedron.
    fn reduce(&mut self) -> Option<P3> {
        let p = self.p;
        let (point, keep) = match self.n {
            1 => (p[0], 0b0001),
            2 => closest_on_segment(p[0], p[1]),
            3 => closest_on_triangle(p[0], p[1], p[2]),
            _ => closest_on_tetrahedron(p[0], p[1], p[2], p[3])?,
        };
        let mut n = 0;
        for (i, vertex) in p.iter().enumerate().take(self.n) {
            if keep & (1 << i) != 0 {
                self.p[n] = *vertex;
                n += 1;
            }
        }
        self.n = n;
        Some(point)
    }
}

/// The point of segment `ab` nearest the origin, and which ends it depends on (bit 0 `a`, bit 1 `b`).
fn closest_on_segment(a: P3, b: P3) -> (P3, u8) {
    let ab = sub(b, a);
    let denom = dot(ab, ab);
    if denom <= 0.0 {
        return (a, 0b01);
    }
    let t = -dot(a, ab) / denom;
    if t <= 0.0 {
        (a, 0b01)
    } else if t >= 1.0 {
        (b, 0b10)
    } else {
        (add(a, scale(ab, t)), 0b11)
    }
}

/// The point of triangle `abc` nearest the origin, and which vertices it depends on. Voronoi regions in
/// the order Ericson gives them (Real-Time Collision Detection, 5.1.5).
fn closest_on_triangle(a: P3, b: P3, c: P3) -> (P3, u8) {
    let ab = sub(b, a);
    let ac = sub(c, a);
    let d1 = -dot(ab, a);
    let d2 = -dot(ac, a);
    if d1 <= 0.0 && d2 <= 0.0 {
        return (a, 0b001);
    }
    let d3 = -dot(ab, b);
    let d4 = -dot(ac, b);
    if d3 >= 0.0 && d4 <= d3 {
        return (b, 0b010);
    }
    let vc = d1 * d4 - d3 * d2;
    if vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0 && d1 - d3 > 0.0 {
        return (add(a, scale(ab, d1 / (d1 - d3))), 0b011);
    }
    let d5 = -dot(ab, c);
    let d6 = -dot(ac, c);
    if d6 >= 0.0 && d5 <= d6 {
        return (c, 0b100);
    }
    let vb = d5 * d2 - d1 * d6;
    if vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0 && d2 - d6 > 0.0 {
        return (add(a, scale(ac, d2 / (d2 - d6))), 0b101);
    }
    let va = d3 * d6 - d5 * d4;
    let e = d4 - d3;
    let f = d5 - d6;
    if va <= 0.0 && e >= 0.0 && f >= 0.0 && e + f > 0.0 {
        return (add(b, scale(sub(c, b), e / (e + f))), 0b110);
    }
    let sum = va + vb + vc;
    if sum <= 0.0 {
        // Collinear: the nearest point is on one of the edges.
        let mut best = closest_on_segment(a, b);
        for (cand, bits) in [
            (closest_on_segment(b, c), [0b010, 0b100]),
            (closest_on_segment(a, c), [0b001, 0b100]),
        ] {
            if dot(cand.0, cand.0) < dot(best.0, best.0) {
                let mut keep = 0;
                if cand.1 & 1 != 0 {
                    keep |= bits[0];
                }
                if cand.1 & 2 != 0 {
                    keep |= bits[1];
                }
                best = (cand.0, keep);
            }
        }
        return best;
    }
    let v = vb / sum;
    let wt = vc / sum;
    (add(a, add(scale(ab, v), scale(ac, wt))), 0b111)
}

/// The point of tetrahedron `abcd` nearest the origin and which vertices it depends on, or `None` when
/// the origin is inside.
fn closest_on_tetrahedron(a: P3, b: P3, c: P3, d: P3) -> Option<(P3, u8)> {
    let faces: [([P3; 4], [u8; 3]); 4] = [
        ([a, b, c, d], [0b0001, 0b0010, 0b0100]),
        ([a, c, d, b], [0b0001, 0b0100, 0b1000]),
        ([a, d, b, c], [0b0001, 0b1000, 0b0010]),
        ([b, d, c, a], [0b0010, 0b1000, 0b0100]),
    ];
    let mut best: Option<(P3, u8)> = None;
    for ([p, q, r, other], bits) in faces {
        if !origin_outside_face(p, q, r, other) {
            continue;
        }
        let (point, local) = closest_on_triangle(p, q, r);
        if best.is_none_or(|(bp, _)| dot(point, point) < dot(bp, bp)) {
            let mut keep = 0;
            for (i, bit) in bits.iter().enumerate() {
                if local & (1 << i) != 0 {
                    keep |= bit;
                }
            }
            best = Some((point, keep));
        }
    }
    best
}

/// Whether the origin and `other` are on opposite sides of plane `pqr`. A flat tetrahedron has no inside,
/// so a face `other` lies on counts as one the origin may be outside.
fn origin_outside_face(p: P3, q: P3, r: P3, other: P3) -> bool {
    let n = cross(sub(q, p), sub(r, p));
    let s_origin = -dot(p, n);
    let s_other = dot(sub(other, p), n);
    let tolerance = 1.0e-12 * dot(n, n).sqrt() * dot(sub(other, p), sub(other, p)).sqrt();
    if s_other.abs() <= tolerance {
        return true;
    }
    s_origin * s_other < 0.0
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A small deterministic generator, so a failure reproduces.
    struct Rng(u64);

    impl Rng {
        fn next(&mut self) -> f64 {
            self.0 ^= self.0 << 13;
            self.0 ^= self.0 >> 7;
            self.0 ^= self.0 << 17;
            (self.0 >> 11) as f64 / (1u64 << 53) as f64
        }

        fn range(&mut self, lo: f64, hi: f64) -> f64 {
            lo + (hi - lo) * self.next()
        }

        fn point(&mut self, lo: f64, hi: f64) -> P3 {
            [self.range(lo, hi), self.range(lo, hi), self.range(lo, hi)]
        }
    }

    fn v(p: P3) -> Vec3 {
        Vec3::new(p[0] as f32, p[1] as f32, p[2] as f32)
    }

    fn solid_at(occ: &Occupancy, c: Cell) -> bool {
        occ.solid[occ.grid.cell_index(c)] != 0
    }

    /// A square in the plane `z = height`, two triangles, counter-clockwise seen from +z.
    fn floor(height: f32, lo: f32, hi: f32) -> Vec<Vec3> {
        let (a, b, c, d) = (
            Vec3::new(lo, lo, height),
            Vec3::new(hi, lo, height),
            Vec3::new(hi, hi, height),
            Vec3::new(lo, hi, height),
        );
        vec![a, b, c, a, c, d]
    }

    fn box_distance_sq(lo_a: P3, hi_a: P3, lo_b: P3, hi_b: P3) -> f64 {
        (0..3)
            .map(|i| {
                let gap = (lo_b[i] - hi_a[i]).max(lo_a[i] - hi_b[i]).max(0.0);
                gap * gap
            })
            .sum()
    }

    #[test]
    fn gjk_measures_the_gap_between_two_boxes() {
        let mut rng = Rng(0x9e37_79b9_7f4a_7c15);
        for _ in 0..2000 {
            let half_a = rng.point(0.1, 2.0);
            let half_b = rng.point(0.1, 2.0);
            let centre_b = rng.point(-5.0, 5.0);
            let a = BoxCore(half_a);
            let b = Core::Box {
                map: Affine::translation(centre_b),
                half: half_b,
            };
            let expected = box_distance_sq(
                neg(half_a),
                half_a,
                sub(centre_b, half_b),
                add(centre_b, half_b),
            );
            let got = gjk_distance_sq(&a, &b, 0.0);
            assert!(
                (got.sqrt() - expected.sqrt()).abs() < 1.0e-6,
                "box gap: gjk {got} expected {expected}"
            );
        }
    }

    #[test]
    fn gjk_measures_a_point_against_a_triangle() {
        let mut rng = Rng(0x1234_5678_9abc_def1);
        for _ in 0..500 {
            let (a, b, c) = (
                rng.point(-2.0, 2.0),
                rng.point(-2.0, 2.0),
                rng.point(-2.0, 2.0),
            );
            let p = rng.point(-3.0, 3.0);
            let got = gjk_distance_sq(&Core::Point(p), &TriangleCore(a, b, c), 0.0).sqrt();
            // Dense barycentric sampling bounds the true distance from above, to within the sample spacing.
            let steps = 200;
            let mut best = f64::INFINITY;
            for i in 0..=steps {
                for j in 0..=(steps - i) {
                    let (u, t) = (
                        f64::from(i) / f64::from(steps),
                        f64::from(j) / f64::from(steps),
                    );
                    let q = add(a, add(scale(sub(b, a), u), scale(sub(c, a), t)));
                    let dq = sub(q, p);
                    best = best.min(dot(dq, dq).sqrt());
                }
            }
            let spacing = [sub(b, a), sub(c, a)]
                .iter()
                .map(|e| dot(*e, *e).sqrt())
                .fold(0.0, f64::max)
                / f64::from(steps);
            assert!(
                got <= best + 1.0e-9,
                "gjk {got} above a sampled point at {best}"
            );
            assert!(
                got >= best - spacing - 1.0e-9,
                "gjk {got} far below sampling {best}"
            );
        }
    }

    #[test]
    fn the_separating_axis_test_agrees_with_gjk() {
        let mut rng = Rng(0xdead_beef_cafe_f00d);
        let mut overlaps = 0;
        for _ in 0..20000 {
            let e = rng.point(0.2, 1.0);
            let (a, b, c) = (
                rng.point(-2.0, 2.0),
                rng.point(-2.0, 2.0),
                rng.point(-2.0, 2.0),
            );
            let sat = triangle_overlaps_box(a, b, c, e);
            let d = gjk_distance_sq(&BoxCore(e), &TriangleCore(a, b, c), 0.0).sqrt();
            if sat {
                overlaps += 1;
                assert!(d < 1.0e-7, "SAT overlaps but GJK measures {d}");
            } else {
                assert!(d > 0.0, "SAT separates but GJK measures zero");
            }
        }
        assert!(
            overlaps > 1000,
            "the sample exercised overlaps ({overlaps})"
        );
    }

    #[test]
    fn the_rounded_test_agrees_with_gjk_alone() {
        let mut rng = Rng(0x0bad_cafe_1234_5678);
        let grid = Grid::new(Vec3::ZERO, 1.0, [1, 1, 1]);
        let v = Voxelizer::new(&grid, &[], 0.2);
        for _ in 0..20000 {
            let (a, b, c) = (
                rng.point(-1.5, 1.5),
                rng.point(-1.5, 1.5),
                rng.point(-1.5, 1.5),
            );
            let fast = v.triangle_touches_cell(a, b, c);
            let slow = within(
                &BoxCore([0.3; 3]),
                &TriangleCore(a, b, c),
                0.2 + CONTACT_TOLERANCE,
            );
            assert_eq!(fast, slow, "triangle {a:?} {b:?} {c:?}");
        }
    }

    #[test]
    fn a_one_sided_floor_claims_only_the_cells_in_front_of_it() {
        let grid = Grid::new(Vec3::ZERO, 1.0, [8, 8, 10]);
        let mut up = Geometry::new();
        up.add_mesh(
            floor(5.3, 1.0, 7.0),
            Affine::IDENTITY,
            Sides::Front(Winding::CounterClockwise),
        );
        let occ = voxelize(&grid, &up, &[], 0.0);
        assert!(
            solid_at(&occ, [3, 3, 5]),
            "the cell the floor passes through, centre above it"
        );
        assert!(
            !solid_at(&occ, [3, 3, 4]),
            "the cell below does not reach the floor"
        );

        let mut down = Geometry::new();
        down.add_mesh(
            floor(5.3, 1.0, 7.0),
            Affine::IDENTITY,
            Sides::Front(Winding::Clockwise),
        );
        let occ = voxelize(&grid, &down, &[], 0.0);
        assert!(
            !solid_at(&occ, [3, 3, 5]),
            "facing down, a centre above the floor is behind it"
        );
        assert_eq!(occ.solid_bbox(), None, "and nothing else reaches it");

        let mut on_face = Geometry::new();
        on_face.add_mesh(
            floor(5.0, 1.0, 7.0),
            Affine::IDENTITY,
            Sides::Front(Winding::CounterClockwise),
        );
        let occ = voxelize(&grid, &on_face, &[], 0.0);
        assert!(
            solid_at(&occ, [3, 3, 5]),
            "a floor on a cell face touches the cell above"
        );
        assert!(
            !solid_at(&occ, [3, 3, 4]),
            "and the cell below it is behind"
        );

        let mut both = Geometry::new();
        both.add_mesh(floor(5.0, 1.0, 7.0), Affine::IDENTITY, Sides::Both);
        let occ = voxelize(&grid, &both, &[], 0.0);
        assert!(
            solid_at(&occ, [3, 3, 5]) && solid_at(&occ, [3, 3, 4]),
            "double-sided claims both"
        );
    }

    #[test]
    fn rounding_spares_a_cell_whose_corner_is_only_grazed() {
        // The plane x + y + z = 3h - depth cuts the sharp box's corner by `depth` along the axes. A rounded
        // box of radius r reaches it only when depth >= r * (3 - sqrt 3).
        let h = 0.5;
        let r = 0.04;
        for (depth, rounded_hit) in [(0.01, false), (0.1, true)] {
            let k = 3.0 * h - depth;
            let (a, b, c) = ([k, 0.0, 0.0], [0.0, k, 0.0], [0.0, 0.0, k]);
            let grid = Grid::new(Vec3::new(-0.5, -0.5, -0.5), 1.0, [1, 1, 1]);
            let sharp = Voxelizer::new(&grid, &[], 0.0);
            let rounded = Voxelizer::new(&grid, &[], r);
            assert!(
                sharp.triangle_touches_cell(a, b, c),
                "depth {depth}: the sharp box is cut"
            );
            assert_eq!(
                rounded.triangle_touches_cell(a, b, c),
                rounded_hit,
                "depth {depth}: rounded"
            );
        }
    }

    #[test]
    fn a_mesh_is_a_surface_and_a_convex_piece_is_solid() {
        let grid = Grid::new(Vec3::ZERO, 1.0, [20, 20, 20]);
        let centre = [10.0, 10.0, 10.0];
        let mut hull = Geometry::new();
        let corners: Vec<Vec3> = (0..8)
            .map(|i| {
                v([
                    if i & 1 != 0 { 15.0 } else { 5.0 },
                    if i & 2 != 0 { 15.0 } else { 5.0 },
                    if i & 4 != 0 { 15.0 } else { 5.0 },
                ])
            })
            .collect();
        hull.add_hull(&corners, Affine::IDENTITY);
        let occ = voxelize(&grid, &hull, &[], 0.0);
        assert!(
            solid_at(&occ, [10, 10, 10]),
            "a convex piece fills its inside"
        );

        let mut boxed = Geometry::new();
        boxed.add_box([5.0; 3], 0.0, Affine::translation(centre));
        assert_eq!(
            voxelize(&grid, &boxed, &[], 0.0).solid,
            occ.solid,
            "a box is its corner hull"
        );

        // The same cube as twelve outward-facing triangles.
        let c = |x: f32, y: f32, z: f32| Vec3::new(5.0 + 10.0 * x, 5.0 + 10.0 * y, 5.0 + 10.0 * z);
        let quads = [
            [c(0., 0., 0.), c(0., 1., 0.), c(1., 1., 0.), c(1., 0., 0.)],
            [c(0., 0., 1.), c(1., 0., 1.), c(1., 1., 1.), c(0., 1., 1.)],
            [c(0., 0., 0.), c(1., 0., 0.), c(1., 0., 1.), c(0., 0., 1.)],
            [c(0., 1., 0.), c(0., 1., 1.), c(1., 1., 1.), c(1., 1., 0.)],
            [c(0., 0., 0.), c(0., 0., 1.), c(0., 1., 1.), c(0., 1., 0.)],
            [c(1., 0., 0.), c(1., 1., 0.), c(1., 1., 1.), c(1., 0., 1.)],
        ];
        let mut tris = Vec::new();
        for [a, b, cc, d] in quads {
            tris.extend([a, b, cc, a, cc, d]);
        }
        let mut shell = Geometry::new();
        shell.add_mesh(
            tris,
            Affine::IDENTITY,
            Sides::Front(Winding::CounterClockwise),
        );
        let occ = voxelize(&grid, &shell, &[], 0.0);
        assert!(
            !solid_at(&occ, [10, 10, 10]),
            "a mesh leaves its inside free"
        );
        assert!(
            solid_at(&occ, [15, 10, 10]),
            "the cell outside a wall touches it"
        );
        assert!(
            !solid_at(&occ, [14, 10, 10]),
            "the cell inside a wall is behind it"
        );
    }

    #[test]
    fn a_rotated_box_is_its_rotated_corners() {
        let grid = Grid::new(Vec3::new(-8.0, -8.0, -8.0), 1.0, [16, 16, 16]);
        let (s, c) = (0.6_f64.sin(), 0.6_f64.cos());
        let map = Affine {
            rows: [[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]],
            origin: [0.3, -0.2, 0.1],
        };
        let half = [3.0, 1.5, 4.5];
        let mut boxed = Geometry::new();
        boxed.add_box(half, 0.0, map);
        let mut hull = Geometry::new();
        let corners: Vec<Vec3> = (0..8)
            .map(|i| {
                v([
                    if i & 1 != 0 { half[0] } else { -half[0] },
                    if i & 2 != 0 { half[1] } else { -half[1] },
                    if i & 4 != 0 { half[2] } else { -half[2] },
                ])
            })
            .collect();
        hull.add_hull(&corners, map);
        let a = voxelize(&grid, &boxed, &[], 0.04);
        let b = voxelize(&grid, &hull, &[], 0.04);
        assert!(a.solid_bbox().is_some());
        assert_eq!(a.solid, b.solid);
    }

    #[test]
    fn a_cell_outside_every_region_stays_free() {
        let grid = Grid::new(Vec3::ZERO, 1.0, [12, 12, 12]);
        let mut both = Geometry::new();
        both.add_mesh(floor(6.0, 0.0, 12.0), Affine::IDENTITY, Sides::Both);
        let region = Region {
            centre: Vec3::new(3.0, 3.0, 6.0),
            radius: 2.0,
        };
        let occ = voxelize(&grid, &both, &[region], 0.0);
        for (i, &s) in occ.solid.iter().enumerate() {
            let cc = grid.cell_center(grid.index_cell(i));
            if s != 0 {
                assert!(
                    w(cc.distance_squared_to(region.centre)) <= 4.0,
                    "solid outside the region"
                );
            }
        }
        assert!(
            solid_at(&occ, [3, 3, 6]),
            "inside the region the floor still counts"
        );
        assert!(!solid_at(&occ, [9, 9, 6]), "outside it the floor does not");
    }

    #[test]
    fn a_short_edge_welds_the_triangle_away() {
        let grid = Grid::new(Vec3::ZERO, 1.0, [4, 4, 4]);
        let mut sliver = Geometry::new();
        sliver.add_mesh(
            vec![
                Vec3::new(2.0, 2.0, 2.0),
                Vec3::new(2.00001, 2.0, 2.0),
                Vec3::new(2.0, 3.0, 2.0),
            ],
            Affine::IDENTITY,
            Sides::Both,
        );
        assert_eq!(voxelize(&grid, &sliver, &[], 0.0).solid_bbox(), None);
    }

    #[test]
    fn geometry_outside_the_lattice_marks_nothing() {
        let grid = Grid::new(Vec3::ZERO, 1.0, [4, 4, 4]);
        let mut far = Geometry::new();
        far.add_mesh(floor(50.0, -10.0, 10.0), Affine::IDENTITY, Sides::Both);
        far.add_box([1.0; 3], 0.0, Affine::translation([-20.0, 0.0, 0.0]));
        assert_eq!(voxelize(&grid, &far, &[], 0.0).solid_bbox(), None);
    }

    /// Every cell against every piece, with none of the range arithmetic.
    fn brute_force(grid: &Grid, geometry: &Geometry, regions: &[Region], rounding: f64) -> Vec<u8> {
        let mut v = Voxelizer::new(grid, regions, rounding);
        let mut solid = vec![0; grid.cell_count()];
        for i in 0..grid.cell_count() {
            let c = grid.index_cell(i);
            for mesh in &geometry.meshes {
                for tri in mesh.vertices.chunks_exact(3) {
                    if let Some(t) = Triangle::place(tri, &mesh.map, mesh.sides) {
                        v.test_triangle(&t, c, &mut solid);
                    }
                }
            }
            for piece in &geometry.convexes {
                if solid[i] != 0 || !v.in_region(i, c) {
                    continue;
                }
                let core = BoxCore([v.half - v.rounding; 3]);
                let shifted = Shifted {
                    core: &piece.core,
                    by: v.centre(c),
                };
                if within(&core, &shifted, v.rounding_reach + piece.radius) {
                    solid[i] = 1;
                }
            }
        }
        solid
    }

    #[test]
    fn the_walk_visits_every_cell_the_brute_force_finds() {
        let mut rng = Rng(0x5151_2323_7777_9999);
        for scene in 0..40 {
            let grid = Grid::new(
                Vec3::new(
                    rng.range(-3.0, 3.0) as f32,
                    rng.range(-3.0, 3.0) as f32,
                    0.25,
                ),
                [0.5, 1.0, 0.75][scene % 3],
                [10, 9, 11],
            );
            let span = 9.0;
            let mut geometry = Geometry::new();
            let mut tris = Vec::new();
            for _ in 0..12 {
                let base = rng.point(-4.0, span);
                for _ in 0..3 {
                    tris.push(v(add(base, rng.point(-4.0, 4.0))));
                }
            }
            let sides = match scene % 3 {
                0 => Sides::Both,
                1 => Sides::Front(Winding::CounterClockwise),
                _ => Sides::Front(Winding::Clockwise),
            };
            geometry.add_mesh(tris, Affine::IDENTITY, sides);
            geometry.add_sphere(
                rng.range(0.2, 2.0),
                Affine::translation(rng.point(-2.0, span)),
            );
            geometry.add_capsule(
                rng.range(0.2, 1.0),
                rng.range(0.0, 2.0),
                Affine::translation(rng.point(-2.0, span)),
            );
            geometry.add_cylinder(
                rng.range(0.5, 2.0),
                rng.range(0.5, 2.0),
                0.04,
                Affine::translation(rng.point(-2.0, span)),
            );
            let rounding = [0.0, 0.04, 0.2][scene % 3];
            let regions = if scene % 2 == 0 {
                Vec::new()
            } else {
                vec![Region {
                    centre: v(rng.point(0.0, span)),
                    radius: rng.range(2.0, 6.0) as f32,
                }]
            };
            let fast = voxelize(&grid, &geometry, &regions, rounding);
            let slow = brute_force(&grid, &geometry, &regions, rounding);
            assert_eq!(fast.solid, slow, "scene {scene}");
        }
    }

    #[test]
    fn a_cylinder_support_point_is_the_farthest_rim_point() {
        let map = Affine {
            rows: [[1.0, 0.2, 0.0], [0.0, 2.0, 0.0], [0.3, 0.0, 1.5]],
            origin: [1.0, 2.0, 3.0],
        };
        let core = Core::Cylinder {
            map,
            radius: 1.25,
            half_height: 0.75,
        };
        let mut rng = Rng(0x7777_1111_2222_3333);
        for _ in 0..200 {
            let d = rng.point(-1.0, 1.0);
            let s = core.support(d);
            let mut best = f64::NEG_INFINITY;
            for k in 0..720 {
                let a = f64::from(k) * std::f64::consts::TAU / 720.0;
                for y in [-0.75, 0.75] {
                    best = best.max(dot(map.point([1.25 * a.cos(), y, 1.25 * a.sin()]), d));
                }
            }
            assert!(dot(s, d) >= best - 1.0e-9, "support below a rim sample");
            assert!(dot(s, d) <= best + 1.0e-3, "support beyond the rim");
        }
    }
}
