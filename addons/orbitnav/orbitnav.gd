@tool
extends EditorPlugin
## OrbitNav plugin -- registers the `Nav` autoload (the navigation facade) while enabled. The autoload entry
## written into project.godot is what makes `Nav` available in exported builds; the EditorPlugin itself is
## editor-only, so enabling/disabling the plugin adds/removes it.
##
## Backend: the OrbitNav native Rust GDExtension -- sources in native/ (a cargo workspace at the repo root),
## binaries in the sibling addon addons/orbitnav_native/. The backend is reached ONLY through
## addons/orbitnav/nav.gd; the `just nav-check` grep gate (CI) fails if any other file references the backend
## class, so the navigation layer can be swapped without touching game code.
##
## BOTH addon directories must be installed together: this one is the GDScript surface, addons/orbitnav_native/
## carries the .gdextension and its binaries. `Nav` without the extension is a facade over nothing -- which is
## a supported state, not a crash: every call answers an inert value. See nav.gd.

const AUTOLOAD_NAME: String = "Nav"
const AUTOLOAD_PATH: String = "res://addons/orbitnav/nav.gd"

func _enter_tree() -> void:
	add_autoload_singleton(AUTOLOAD_NAME, AUTOLOAD_PATH)

func _exit_tree() -> void:
	remove_autoload_singleton(AUTOLOAD_NAME)
