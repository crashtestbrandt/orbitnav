#!/usr/bin/env bash
# OrbitNav native load smoke.
#
# Proves the whole Rust -> cdylib -> Godot chain end to end: the library builds, Godot loads the
# GDExtension, the Rust-defined classes register, their exported properties bind, their signals
# reach GDScript, and the tick clock actually advances.
#
# Deliberately runs against a THROWAWAY project in a temp directory, never against harness/ or a demo.
# That is the point: it proves the extension registers its classes OUTSIDE any project that has been
# set up for it -- no autoload, no plugin.cfg, no addon GDScript. If this passes and a project still
# fails, the fault is in the project's configuration, not in the library.
#
# Usage: tools/orbitnav-smoke.sh [--skip-build]
# Env:   GODOT (binary or wrapper to run; default: tools/godot-quiet.sh)
#
# Linux only: it reads ELF magic and asks `nm -D` for the entry symbol.
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
BIN="$ROOT/addons/orbitnav_native/bin"
GODOT="${GODOT:-$ROOT/tools/godot-quiet.sh}"

if [ "${1:-}" != "--skip-build" ]; then
	printf 'orbitnav-smoke: building both descriptor profiles\n'
	"$ROOT/tools/build-native.sh" build linux "$BIN"
fi

# Every name this platform's descriptor entries resolve to. Checking the SET rather than one file is
# what catches a profile that failed to stage: the throwaway project runs from source and loads only
# `template_debug`, so a missing `template_release` would pass here and fail one export later.
NAMES="$("$ROOT/tools/build-native.sh" names linux)"
missing=""
for name in $NAMES; do
	[ -s "$BIN/$name" ] || missing="$missing $name"
done
if [ -n "$missing" ]; then
	printf 'orbitnav-smoke FAILED: %s holds no library named:%s\n' "$BIN" "$missing" >&2
	printf 'No binary is committed to this repository. Run `just native-install`.\n' >&2
	exit 1
fi

for name in $NAMES; do
	lib="$BIN/$name"
	# A pointer file is a few hundred bytes of text that dlopen rejects with "invalid ELF header" behind
	# a confusing cascade. Name the real cause here rather than letting Godot guess at it.
	if head -c 64 "$lib" | grep -q 'git-lfs.github.com' 2>/dev/null; then
		printf 'orbitnav-smoke FAILED: %s is a Git LFS POINTER, not a library.\n' "$lib" >&2
		exit 1
	fi
	# The entry symbol the .gdextension names. If this is absent the library will load and then do
	# nothing, which presents as "class not found" far from the real cause.
	if command -v nm >/dev/null 2>&1; then
		if ! nm -D "$lib" | grep -q 'gdext_rust_init'; then
			printf 'orbitnav-smoke FAILED: %s exports no gdext_rust_init symbol.\n' "$lib" >&2
			exit 1
		fi
	fi
done

WORK="$(mktemp -d)"
trap 'rm -rf "$WORK"' EXIT
BIN_DIR="$WORK/addons/orbitnav_native/bin"
mkdir -p "$BIN_DIR"

cp "$ROOT/addons/orbitnav_native/orbitnav.gdextension" "$WORK/addons/orbitnav_native/orbitnav.gdextension"
# The whole set, under the shipped names, so the descriptor copied beside it resolves verbatim.
for name in $NAMES; do
	cp "$BIN/$name" "$BIN_DIR/$name"
done

cat > "$WORK/project.godot" <<'PROJECT'
config_version=5

[application]

config/name="orbitnav-smoke"
config/features=PackedStringArray("4.4")
PROJECT

cat > "$WORK/smoke.gd" <<'SMOKE'
extends SceneTree

## The throwaway-project smoke: does the LIBRARY load and work in a project that contains nothing else?
##
## No addon GDScript, no autoload, no plugin -- only the .gdextension and the staged binaries. That is the
## point: it isolates "the cdylib is loadable and its class works" from every other thing that can be wrong,
## so a failure here is the binary and a failure in the harness is the addon.
##
## This is the one place allowed to name the backend class directly; it is testing the class's existence.

var _volume: int = 0

func _initialize() -> void:
	if not ClassDB.class_exists("OrbitNav"):
		printerr("ORBIT-SMOKE FAIL: the OrbitNav class did not register")
		quit(1)
		return

	var obj: Object = ClassDB.instantiate("OrbitNav")
	var nav: Node = obj as Node
	if nav == null:
		printerr("ORBIT-SMOKE FAIL: OrbitNav did not instantiate as a Node")
		quit(1)
		return
	root.add_child(nav)

	# A volume, a floor, a derive: the whole bake path, on data built here rather than sampled.
	var dims := Vector3i(8, 8, 6)
	_volume = int(nav.call("create_volume", Vector3.ZERO, 1.0, dims))
	if _volume == 0:
		printerr("ORBIT-SMOKE FAIL: create_volume refused a valid grid")
		quit(1)
		return

	var cells: int = int(nav.call("cell_count", _volume))
	if cells != dims.x * dims.y * dims.z:
		printerr("ORBIT-SMOKE FAIL: cell_count is %d, expected %d" % [cells, dims.x * dims.y * dims.z])
		quit(1)
		return

	var solid := PackedByteArray()
	solid.resize(cells)
	for y in dims.y:
		for x in dims.x:
			solid[x + dims.x * (y + dims.y * 0)] = 1
	if not bool(nav.call("upload_occupancy", _volume, solid)):
		printerr("ORBIT-SMOKE FAIL: upload_occupancy refused a correctly sized array")
		quit(1)
		return
	if not bool(nav.call("derive_blocking", _volume)):
		printerr("ORBIT-SMOKE FAIL: derive_blocking failed")
		quit(1)
		return

	# The derived arrays come back whole, and say what they should about a floor.
	var flag: PackedByteArray = nav.call("surface_flag", _volume)
	if flag.size() != cells:
		printerr("ORBIT-SMOKE FAIL: surface_flag is %d bytes, expected %d" % [flag.size(), cells])
		quit(1)
		return
	if flag[4 + dims.x * (4 + dims.y * 1)] == 0:
		printerr("ORBIT-SMOKE FAIL: the cell above the floor is not flagged as surface")
		quit(1)
		return

	# A snap, and a search driven the way a consumer drives one.
	var snapped: Vector3i = nav.call("nearest_free_cell", _volume, Vector3(4.5, 4.5, 0.5), false, 5)
	if snapped != Vector3i(4, 4, 1):
		printerr("ORBIT-SMOKE FAIL: snap out of the floor gave %s" % str(snapped))
		quit(1)
		return

	var job: int = int(nav.call("request_path", _volume,
		Vector3(1.5, 1.5, 3.5), Vector3(6.5, 6.5, 3.5), false, 8000, 5))
	if job == 0:
		printerr("ORBIT-SMOKE FAIL: request_path refused a built volume")
		quit(1)
		return
	var spins: int = 0
	while spins < 200000:
		var state: int = int(nav.call("job_state", _volume, job))
		if state != 1 and state != 2:   # not PENDING, not RUNNING
			break
		spins += 1
	var path: PackedVector3Array = nav.call("take_path", job)
	if path.is_empty():
		printerr("ORBIT-SMOKE FAIL: no route across open space (outcome %d)"
			% int(nav.call("last_outcome")))
		quit(1)
		return

	var workers: int = int(nav.call("worker_count"))
	print("ORBIT-SMOKE ok  cells=%d workers=%d waypoints=%d outcome=%d"
		% [cells, workers, path.size(), int(nav.call("last_outcome"))])
	print("ORBIT-SMOKE PASS")
	quit(0)
SMOKE


# Godot discovers .gdextension files while scanning the project, and a fresh project has no scan
# cache. Without this pass the library is never loaded and the classes simply do not exist, which
# presents identically to a genuinely broken build.
"$GODOT" --headless --path "$WORK" --import >/dev/null 2>&1 || true

LOG="$WORK/smoke.log"
set +e
"$GODOT" --headless --path "$WORK" --script smoke.gd 2>&1 | tee "$LOG"
status="${PIPESTATUS[0]}"
set -e

if [ "$status" -ne 0 ]; then
	printf 'orbitnav-smoke FAILED: Godot exited %d\n' "$status" >&2
	exit 1
fi
if ! grep -q 'ORBIT-SMOKE PASS' "$LOG"; then
	printf 'orbitnav-smoke FAILED: no ORBIT-SMOKE PASS marker in the output\n' >&2
	exit 1
fi
if grep -q 'ORBIT-SMOKE FAIL' "$LOG"; then
	printf 'orbitnav-smoke FAILED: the smoke script reported a failure\n' >&2
	exit 1
fi

printf 'orbitnav-smoke passed: the extension loads and navigates in a project containing nothing else.\n'
