//! Engine types in, plain data out.
//!
//! Every conversion is whole-array or whole-value. Nothing here is called per cell; see the crate
//! note on crossing cost.

use godot::prelude::*;
use orbitnav_core::real::Vec3 as CoreVec3;
use orbitnav_core::voxelize::Affine;

/// An engine vector as core's own.
#[inline]
#[must_use]
pub fn v3_in(v: Vector3) -> CoreVec3 {
    CoreVec3::new(v.x, v.y, v.z)
}

/// A core vector as the engine's.
#[inline]
#[must_use]
pub fn v3_out(v: CoreVec3) -> Vector3 {
    Vector3::new(v.x, v.y, v.z)
}

/// An engine cell coordinate as core's own.
#[inline]
#[must_use]
pub fn cell_in(v: Vector3i) -> [i32; 3] {
    [v.x, v.y, v.z]
}

/// A core cell coordinate as the engine's.
#[inline]
#[must_use]
pub fn cell_out(c: [i32; 3]) -> Vector3i {
    Vector3i::new(c[0], c[1], c[2])
}

/// A byte array as a plain vector, in one copy.
#[must_use]
pub fn bytes_in(a: &PackedByteArray) -> Vec<u8> {
    a.as_slice().to_vec()
}

/// A plain byte vector as an engine array, in one copy.
#[must_use]
pub fn bytes_out(v: &[u8]) -> PackedByteArray {
    PackedByteArray::from(v)
}

/// A plain `i32` vector as an engine array, in one copy.
#[must_use]
pub fn ints_out(v: &[i32]) -> PackedInt32Array {
    PackedInt32Array::from(v)
}

/// Waypoints as an engine array.
#[must_use]
pub fn points_out(v: &[CoreVec3]) -> PackedVector3Array {
    let mut out = PackedVector3Array::new();
    out.resize(v.len());
    for (i, p) in v.iter().enumerate() {
        out[i] = v3_out(*p);
    }
    out
}

/// Waypoints from an engine array.
#[must_use]
pub fn points_in(a: &PackedVector3Array) -> Vec<CoreVec3> {
    a.as_slice().iter().map(|v| v3_in(*v)).collect()
}

/// An engine transform as core's affine map, widened to `f64`.
#[must_use]
pub fn affine_in(t: Transform3D) -> Affine {
    let row = |v: Vector3| [f64::from(v.x), f64::from(v.y), f64::from(v.z)];
    Affine {
        rows: [
            row(t.basis.rows[0]),
            row(t.basis.rows[1]),
            row(t.basis.rows[2]),
        ],
        origin: row(t.origin),
    }
}
