//! The float widths the reference implementation uses, made explicit.
//!
//! Godot's `Vector3` holds three `f32` (`real_t` in a standard build), while a GDScript `float` is
//! `f64`. Arithmetic that flows through a `Vector3` therefore rounds to `f32` at every store, and
//! arithmetic that flows through script locals does not. Reproducing that implementation's output
//! exactly means reproducing where each rounding happens.
//!
//! Two mechanisms:
//!
//! - [`Vec3`] is three `f32`, so anywhere the original held a `Vector3` this type holds the same
//!   bits. Passing `[f64; 3]` instead would silently widen and the difference would surface as a
//!   diverging route on one map in ten.
//! - [`w`] and [`n`] name every widening and narrowing. They compile to nothing, and they exist so
//!   a reviewer can count the conversions in a function against the lines they came from.
//!
//! `sqrt` is not a portability risk on either width: IEEE-754 requires it to be correctly rounded,
//! Rust never enables fast-math, and Godot's `Math::sqrt` is the same operation.

/// Widen to the width a script local has. Names the conversion; compiles to nothing.
#[inline(always)]
#[must_use]
pub fn w(v: f32) -> f64 {
    f64::from(v)
}

/// Narrow to the width a vector component has. Names the conversion; compiles to nothing.
#[inline(always)]
#[must_use]
pub fn n(v: f64) -> f32 {
    v as f32
}

/// The tolerance `is_equal_approx` compares against, per component.
pub const CMP_EPSILON: f32 = 0.00001;

/// A point or direction in world space: three `f32`, like the engine's own vector.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Vec3 {
    /// X component.
    pub x: f32,
    /// Y component.
    pub y: f32,
    /// Z component.
    pub z: f32,
}

impl Vec3 {
    /// The origin.
    pub const ZERO: Vec3 = Vec3 {
        x: 0.0,
        y: 0.0,
        z: 0.0,
    };

    /// A point from its three components.
    #[inline]
    #[must_use]
    pub const fn new(x: f32, y: f32, z: f32) -> Self {
        Self { x, y, z }
    }

    /// Component by axis index, 0..=2. Panics outside that range.
    #[inline]
    #[must_use]
    pub fn axis(&self, i: usize) -> f32 {
        match i {
            0 => self.x,
            1 => self.y,
            2 => self.z,
            _ => panic!("axis index {i} out of range"),
        }
    }

    /// Set the component at axis index `i`, 0..=2. Panics outside that range.
    #[inline]
    pub fn set_axis(&mut self, i: usize, v: f32) {
        match i {
            0 => self.x = v,
            1 => self.y = v,
            2 => self.z = v,
            _ => panic!("axis index {i} out of range"),
        }
    }

    /// Squared distance, accumulated at `f32` width exactly as the engine does.
    #[inline]
    #[must_use]
    pub fn distance_squared_to(self, other: Vec3) -> f32 {
        let dx = other.x - self.x;
        let dy = other.y - self.y;
        let dz = other.z - self.z;
        dx * dx + dy * dy + dz * dz
    }

    /// Distance, accumulated and rooted at `f32` width exactly as the engine does.
    #[inline]
    #[must_use]
    pub fn distance_to(self, other: Vec3) -> f32 {
        self.distance_squared_to(other).sqrt()
    }

    /// Componentwise approximate equality against [`CMP_EPSILON`].
    #[inline]
    #[must_use]
    pub fn is_equal_approx(self, other: Vec3) -> bool {
        (self.x - other.x).abs() < CMP_EPSILON
            && (self.y - other.y).abs() < CMP_EPSILON
            && (self.z - other.z).abs() < CMP_EPSILON
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn distance_accumulates_at_f32_width() {
        let a = Vec3::new(0.0, 0.0, 0.0);
        let b = Vec3::new(3.0, 4.0, 0.0);
        assert_eq!(a.distance_to(b), 5.0);
        assert_eq!(a.distance_squared_to(b), 25.0);
    }

    /// The point of the module: for some inputs the two widths genuinely disagree, so holding a
    /// position in `f64` would not reproduce the reference implementation.
    #[test]
    fn the_two_widths_disagree_somewhere() {
        let a = Vec3::new(0.1, 0.2, 0.3);
        let b = Vec3::new(11.7, -3.35, 42.125);
        let narrow = w(a.distance_to(b));

        let dx = w(b.x) - w(a.x);
        let dy = w(b.y) - w(a.y);
        let dz = w(b.z) - w(a.z);
        let wide = (dx * dx + dy * dy + dz * dz).sqrt();

        assert_ne!(
            narrow, wide,
            "if these ever agree for every input, this module has stopped being load-bearing"
        );
    }

    #[test]
    fn is_equal_approx_is_per_component() {
        let a = Vec3::new(1.0, 1.0, 1.0);
        assert!(a.is_equal_approx(Vec3::new(1.000001, 1.0, 1.0)));
        assert!(!a.is_equal_approx(Vec3::new(1.001, 1.0, 1.0)));
    }

    #[test]
    fn axis_round_trips() {
        let mut v = Vec3::ZERO;
        v.set_axis(0, 1.5);
        v.set_axis(1, -2.5);
        v.set_axis(2, 7.0);
        assert_eq!((v.axis(0), v.axis(1), v.axis(2)), (1.5, -2.5, 7.0));
    }
}
