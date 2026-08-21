# Third-party licences

OrbitNav is MIT OR Apache-2.0. Its dependencies:

| Crate | Used by | Licence |
|---|---|---|
| [`godot`](https://github.com/godot-rust/gdext) (gdext) | `orbitnav-godot` | MPL-2.0 |

`godot` pulls in `godot-core`, `godot-ffi`, `godot-macros`, `godot-codegen`, `godot-cell`,
`godot-bindings`, `gdextension-api`, `glam`, `libc`, `nanoserde` and `venial` transitively; all are covered
by their own permissive or MPL-2.0 terms.

**`orbitnav-core` has no dependencies at all.** Its `[dependencies]` table is empty and stays that way: it
is the crate every algorithm lives in, and keeping it dependency-free is what makes it testable in
milliseconds and reusable outside Godot.

## MPL-2.0 and what it asks

gdext is MPL-2.0, which is file-level copyleft. Linking it into a larger work is fine and imposes nothing on
that work; modifying gdext's own source files means publishing those files' changes. OrbitNav does not
modify gdext.

A new dependency goes in this table in the same commit that adds it.
