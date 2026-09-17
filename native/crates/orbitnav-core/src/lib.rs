//! OrbitNav core — voxel navigation over a dense occupancy grid.
//!
//! # What this crate is
//!
//! A uniform grid of cubic cells, each free or solid, plus everything a mover needs to cross it:
//! which free cells touch solid (the SURFACE classification), which free cells reach each other
//! (connected COMPONENTS), whether a straight segment is clear (line of sight), the nearest usable
//! cell to an arbitrary point, and an A* route between two points. A worker pool runs all of it off
//! the caller's thread.
//!
//! # The one rule
//!
//! **This crate may not name a consumer's concepts.** No stations, modules, arenas, characters or
//! issue numbers -- and no `godot`. Everything here is a pure function of plain data. Decision code
//! (behaviour trees, target selection, steering) belongs to the consumer, which is why the crate is
//! named for navigation rather than for AI: a crate named for AI attracts the half that must not
//! move.
//!
//! # Reproducing another implementation exactly
//!
//! This crate exists to replace a GDScript implementation, and the changeover is gated on producing
//! IDENTICAL output -- all five derived arrays byte for byte, and identical waypoint sequences from
//! the search. That constraint is stronger than "correct", and it drives several choices that look
//! wrong in isolation:
//!
//! - **Float width is load-bearing.** Godot's `Vector3` is three `f32`, while its script `float` is
//!   `f64`, so the original mixes widths in ways a clean-room port would not. [`real::Vec3`] makes
//!   the `f32` half structural, and every widen/narrow is a named [`real::w`] / [`real::n`] call so
//!   a reviewer can count them against the source.
//! - **Iteration order is load-bearing.** Component labels are numbered by a linear scan for the
//!   next unlabelled cell, and snap ties resolve to the first candidate in shell order.
//! - **Some quirks are reproduced deliberately.** They are marked `QUIRK:` where they appear, with
//!   what the original does and why copying it is cheaper than diverging.
//!
//! Where a behaviour is deliberately NOT reproduced, the item's own documentation says so.
//! [`volume::Volume::random_cell_in_component_from`] is the only such case.

#![forbid(unsafe_code)]
#![warn(missing_docs)]

pub mod astar;
pub mod components;
pub mod grid;
pub mod jobs;
pub mod los;
pub mod navd;
pub mod occupancy;
pub mod real;
pub mod smooth;
pub mod snap;
pub mod surface;
pub mod volume;
pub mod voxelize;

pub use astar::{Outcome, Search, SearchParams};
pub use grid::Grid;
pub use jobs::{JobId, JobState, Pool};
pub use navd::{read_navd, write_navd, NavdError};
pub use occupancy::Occupancy;
pub use real::Vec3;
pub use snap::SURFACE_SNAP_R;
pub use surface::SurfaceArrays;
pub use volume::Volume;
pub use voxelize::{voxelize, Geometry};
