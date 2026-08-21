//! OrbitNav — the Godot 4 GDExtension binding.
//!
//! This crate is the boundary and nothing else. It registers one class, converts between engine
//! types and plain data, and owns the registries that give volumes and jobs their integer handles.
//! **Every algorithm lives in `orbitnav-core`, which does not depend on `godot`** and never sees a
//! `Variant`. A gdext upgrade therefore touches this crate and no logic.
//!
//! # One crossing per operation
//!
//! A call across this boundary costs roughly 0.185 microseconds -- negligible per operation, and
//! fatal per cell. A per-cell API over a two-million-cell volume would spend more time crossing than
//! the work it was crossing to do. So every function here takes or returns a WHOLE array, and there
//! is deliberately no per-element entry point to reach for.

use godot::prelude::*;

mod convert;
mod orbit_nav;

struct OrbitNavExtension;

#[gdextension]
unsafe impl ExtensionLibrary for OrbitNavExtension {}
