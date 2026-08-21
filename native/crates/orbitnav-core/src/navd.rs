//! The `NAVD` volume dump: a whole volume as bytes.
//!
//! One flat little-endian record, no padding and no per-section counts -- every array's length
//! follows from `dims`:
//!
//! | bytes | field |
//! |---|---|
//! | 4 | `"NAVD"` |
//! | 12 | `dims.x`, `dims.y`, `dims.z` as `i32` |
//! | 4 | `cell_size` as `f32` |
//! | 12 | `origin.x/y/z` as `f32` |
//! | N | `solid` |
//! | N | `surface_bits` |
//! | N | `solid_sides` |
//! | N | `surface_flag` |
//! | 4N | `comp_free` as `i32` |
//! | 4N | `comp_surf` as `i32` |
//! | | where `N = dims.x * dims.y * dims.z` |
//!
//! This exists so a dump produced by another implementation can be read here and re-derived from its
//! `solid` alone, and the five derived arrays compared byte for byte. That is the differential test
//! the port is gated on, and it runs as a plain `cargo test` with no engine anywhere.
//!
//! Every malformed input is an [`NavdError`], never a panic: the files come from outside this crate.

use crate::grid::Grid;
use crate::real::Vec3;
use crate::surface::SurfaceArrays;
use crate::volume::Volume;

/// The four magic bytes a dump starts with.
pub const MAGIC: &[u8; 4] = b"NAVD";

/// Why a dump could not be read.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum NavdError {
    /// The first four bytes are not [`MAGIC`].
    BadMagic,
    /// The buffer ends before the header does.
    ShortHeader,
    /// `dims` contains a non-positive extent.
    BadDims,
    /// The buffer is not the length `dims` implies.
    LengthMismatch {
        /// How many bytes the header implies.
        expected: usize,
        /// How many bytes there are.
        got: usize,
    },
}

impl std::fmt::Display for NavdError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            NavdError::BadMagic => write!(f, "not a NAVD dump (bad magic)"),
            NavdError::ShortHeader => write!(f, "truncated before the end of the header"),
            NavdError::BadDims => write!(f, "dims contains a non-positive extent"),
            NavdError::LengthMismatch { expected, got } => {
                write!(f, "dump is {got} bytes, but its header implies {expected}")
            }
        }
    }
}

impl std::error::Error for NavdError {}

/// How many bytes a dump of `n` cells occupies, header included.
#[must_use]
pub fn navd_len(n: usize) -> usize {
    32 + 4 * n + 8 * n
}

/// Read a dump into a volume.
pub fn read_navd(buf: &[u8]) -> Result<Volume, NavdError> {
    if buf.len() < 32 {
        return Err(NavdError::ShortHeader);
    }
    if &buf[0..4] != MAGIC {
        return Err(NavdError::BadMagic);
    }
    let i32_at = |o: usize| i32::from_le_bytes([buf[o], buf[o + 1], buf[o + 2], buf[o + 3]]);
    let f32_at = |o: usize| f32::from_le_bytes([buf[o], buf[o + 1], buf[o + 2], buf[o + 3]]);

    let dims = [i32_at(4), i32_at(8), i32_at(12)];
    if dims[0] <= 0 || dims[1] <= 0 || dims[2] <= 0 {
        return Err(NavdError::BadDims);
    }
    let cell_size = f32_at(16);
    let origin = Vec3::new(f32_at(20), f32_at(24), f32_at(28));
    let grid = Grid::new(origin, f64::from(cell_size), dims);
    let n = grid.cell_count();

    let expected = navd_len(n);
    if buf.len() != expected {
        return Err(NavdError::LengthMismatch {
            expected,
            got: buf.len(),
        });
    }

    let mut o = 32usize;
    let take = |o: &mut usize, n: usize| -> Vec<u8> {
        let v = buf[*o..*o + n].to_vec();
        *o += n;
        v
    };
    let solid = take(&mut o, n);
    let surface_bits = take(&mut o, n);
    let solid_sides = take(&mut o, n);
    let surface_flag = take(&mut o, n);

    let take_i32 = |o: &mut usize, n: usize| -> Vec<i32> {
        let mut v = Vec::with_capacity(n);
        for k in 0..n {
            let b = *o + k * 4;
            v.push(i32::from_le_bytes([
                buf[b],
                buf[b + 1],
                buf[b + 2],
                buf[b + 3],
            ]));
        }
        *o += n * 4;
        v
    };
    let comp_free = take_i32(&mut o, n);
    let comp_surf = take_i32(&mut o, n);

    Ok(Volume::from_parts(
        grid,
        solid,
        SurfaceArrays {
            surface_bits,
            solid_sides,
            surface_flag,
        },
        comp_free,
        comp_surf,
    ))
}

/// Write a volume as a dump.
#[must_use]
pub fn write_navd(vol: &Volume) -> Vec<u8> {
    let n = vol.cell_count();
    let mut out = Vec::with_capacity(navd_len(n));
    out.extend_from_slice(MAGIC);
    for a in 0..3 {
        out.extend_from_slice(&vol.grid.dims[a].to_le_bytes());
    }
    out.extend_from_slice(&(vol.grid.cell_size as f32).to_le_bytes());
    out.extend_from_slice(&vol.grid.origin.x.to_le_bytes());
    out.extend_from_slice(&vol.grid.origin.y.to_le_bytes());
    out.extend_from_slice(&vol.grid.origin.z.to_le_bytes());
    out.extend_from_slice(&vol.solid);
    out.extend_from_slice(&vol.surface_bits);
    out.extend_from_slice(&vol.solid_sides);
    out.extend_from_slice(&vol.surface_flag);
    for v in &vol.comp_free {
        out.extend_from_slice(&v.to_le_bytes());
    }
    for v in &vol.comp_surf {
        out.extend_from_slice(&v.to_le_bytes());
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::occupancy::Occupancy;

    fn sample() -> Volume {
        let g = Grid::new(Vec3::new(-2.0, 1.0, 0.5), 1.0, [7, 5, 4]);
        let mut o = Occupancy::empty(g);
        for x in 0..7 {
            o.solid[g.cell_index([x, 0, 0])] = 1;
        }
        Volume::derive(&o, 1)
    }

    #[test]
    fn a_volume_round_trips() {
        let v = sample();
        let bytes = write_navd(&v);
        assert_eq!(bytes.len(), navd_len(v.cell_count()));
        let back = read_navd(&bytes).expect("round trip");
        assert_eq!(back.grid, v.grid);
        assert_eq!(back.solid, v.solid);
        assert_eq!(back.surface_bits, v.surface_bits);
        assert_eq!(back.solid_sides, v.solid_sides);
        assert_eq!(back.surface_flag, v.surface_flag);
        assert_eq!(back.comp_free, v.comp_free);
        assert_eq!(back.comp_surf, v.comp_surf);
    }

    #[test]
    fn a_dump_re_derives_to_itself() {
        // The differential check, in miniature: read a dump, re-derive from `solid` alone, and the
        // five arrays must match what the dump carried.
        let v = sample();
        let back = read_navd(&write_navd(&v)).unwrap();
        let again = Volume::derive(&back.occupancy(), 1);
        assert_eq!(again.surface_bits, back.surface_bits);
        assert_eq!(again.solid_sides, back.solid_sides);
        assert_eq!(again.surface_flag, back.surface_flag);
        assert_eq!(again.comp_free, back.comp_free);
        assert_eq!(again.comp_surf, back.comp_surf);
    }

    #[test]
    fn bad_magic_is_an_error() {
        let mut b = write_navd(&sample());
        b[0] = b'X';
        assert_eq!(read_navd(&b), Err(NavdError::BadMagic));
    }

    #[test]
    fn a_truncated_dump_is_an_error() {
        let b = write_navd(&sample());
        let cut = &b[..b.len() - 9];
        assert!(matches!(
            read_navd(cut),
            Err(NavdError::LengthMismatch { .. })
        ));
        assert_eq!(read_navd(&b[..12]), Err(NavdError::ShortHeader));
    }

    #[test]
    fn dims_inconsistent_with_the_length_is_an_error() {
        let mut b = write_navd(&sample());
        b[4] = 99; // claim a far larger x extent
        assert!(matches!(
            read_navd(&b),
            Err(NavdError::LengthMismatch { .. })
        ));
    }

    #[test]
    fn a_non_positive_extent_is_an_error() {
        let mut b = write_navd(&sample());
        b[4..8].copy_from_slice(&0i32.to_le_bytes());
        assert_eq!(read_navd(&b), Err(NavdError::BadDims));
    }

    #[test]
    fn an_empty_buffer_is_an_error_not_a_panic() {
        assert_eq!(read_navd(&[]), Err(NavdError::ShortHeader));
    }
}
