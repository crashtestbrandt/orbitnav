class_name NavVolumeHandle
extends RefCounted
## One navigation volume: a uniform grid of cells, plus everything derived from it.
##
## Obtained from [method Nav.make_volume], never constructed directly. The backend owns the data; this holds
## an integer handle to it, so passing one around costs nothing and there is exactly one owner of the
## lifetime.
##
## ## The shape of a bake
##
## [codeblock]
## var vol: NavVolumeHandle = Nav.make_volume(origin, 1.0, dims)
## vol.upload_occupancy(solid)          # one byte per cell; the consumer decides what is solid
## var job: NavPathJob = vol.derive()   # on a worker; the caller does not wait
## # ... later, once job.is_settled() ...
## vol.apply_derive(job)
## [endblock]
##
## Until the derive is applied the volume is not built, and every query answers its inert value.
##
## ## An INERT handle is not an error
##
## With no backend loaded, [method Nav.make_volume] still hands one of these back. [method is_active] reads
## false and each query answers "nothing known": [constant Nav.NO_CELL], `false`, `-1`, an unchanged path.
## Callers do not need a null check.

var _nav: Object = null
var _id: int = 0

func _init(backend: Object, id: int) -> void:
	_nav = backend
	_id = id

## Whether this handle refers to a live volume.
func is_active() -> bool:
	return _nav != null and _id != 0

## Whether the volume carries derived data.
func is_built() -> bool:
	return is_active() and _nav.call(&"is_built", _id)

## Which publication of this volume is current. A job's result is only valid against the generation it was
## submitted under; see [method NavPathJob.state].
func generation() -> int:
	if not is_active():
		return 0
	return _nav.call(&"volume_generation", _id)

## How many cells the volume holds.
func cell_count() -> int:
	if not is_active():
		return 0
	return _nav.call(&"cell_count", _id)

## Replace the occupancy: one byte per cell, non-zero meaning solid, flat-indexed with x varying fastest.
##
## Refused unless the array is exactly one byte per cell. Uploading bumps the generation, so a derive already
## in flight becomes stale rather than landing on top of the new occupancy.
func upload_occupancy(solid: PackedByteArray) -> bool:
	if not is_active():
		return false
	return _nav.call(&"upload_occupancy", _id, solid)

## Queue the derive on a worker. Poll the returned job; when it settles, hand it to [method apply_derive].
func derive() -> NavPathJob:
	if not is_active():
		return NavPathJob.new(null, 0, 0)
	var job: int = _nav.call(&"derive_async", _id)
	return NavPathJob.new(_nav, _id, job)

## Install a finished derive. True when the job was ready and about the current generation.
func apply_derive(job: NavPathJob) -> bool:
	if not is_active() or job == null:
		return false
	return _nav.call(&"apply_derive", _id, job.id())

## Derive on the CALLING thread. For tests and tools; a game should use [method derive].
func derive_blocking() -> bool:
	if not is_active():
		return false
	return _nav.call(&"derive_blocking", _id)

## What the last derive cost the worker, in microseconds.
func derive_usec() -> int:
	if not is_active():
		return 0
	return _nav.call(&"derive_usec", _id)

# --- the derived arrays ---------------------------------------------------------------------------------
# Read back in bulk, once per bake. Every one of these is a whole array in a single call: the boundary has
# no per-cell entry point, deliberately, because a per-cell crossing over a volume of this size costs more
# than the work it crosses to do.

## The occupancy array, one byte per cell.
func solid() -> PackedByteArray:
	if not is_active():
		return PackedByteArray()
	return _nav.call(&"solid", _id)

## Per-cell mask of solid axis neighbours, in the fixed order `-x, +x, -y, +y, -z, +z` (bits 1..32).
func surface_bits() -> PackedByteArray:
	if not is_active():
		return PackedByteArray()
	return _nav.call(&"surface_bits", _id)

## Per-cell count of solid axis neighbours, 0..6.
func solid_sides() -> PackedByteArray:
	if not is_active():
		return PackedByteArray()
	return _nav.call(&"solid_sides", _id)

## Which free cells have solid anywhere in their 26-neighbourhood.
func surface_flag() -> PackedByteArray:
	if not is_active():
		return PackedByteArray()
	return _nav.call(&"surface_flag", _id)

## Free-graph component label per cell, -1 outside the graph.
func comp_free() -> PackedInt32Array:
	if not is_active():
		return PackedInt32Array()
	return _nav.call(&"comp_free", _id)

## Surface-graph component label per cell, -1 outside the graph.
func comp_surf() -> PackedInt32Array:
	if not is_active():
		return PackedInt32Array()
	return _nav.call(&"comp_surf", _id)

# --- queries --------------------------------------------------------------------------------------------

## The usable cell nearest `p` within `max_r` cells, or [constant Nav.NO_CELL].
##
## This is the arming cost of a search: both ends snap before a single cell is expanded, and a MISS pays for
## every shell out to `max_r` -- the walk visits the whole cube at each radius and rejects the interior
## inside the loop. Passing a wider radius than the query needs is not free.
func nearest_free_cell(p: Vector3, surface_only: bool, max_r: int = -1) -> Vector3i:
	if not is_active():
		return Nav.NO_CELL
	var r: int = max_r
	if r < 0:
		r = Nav.SURFACE_SNAP_R if surface_only else Nav.FREE_SNAP_R
	return _nav.call(&"nearest_free_cell", _id, p, surface_only, r)

## Whether the straight segment `a` -> `b` crosses only cells this kind of mover can occupy.
##
## Conservative: a segment that leaves the volume fails, because nothing is known about geometry outside it.
func has_los(a: Vector3, b: Vector3, surface_only: bool) -> bool:
	if not is_active():
		return false
	return _nav.call(&"has_los", _id, a, b, surface_only)

## Collapse `raw` into the fewest waypoints reachable in sequence from `from_p`.
func smooth_path(from_p: Vector3, raw: PackedVector3Array, surface_only: bool) -> PackedVector3Array:
	if not is_active():
		return raw
	return _nav.call(&"smooth_path", _id, from_p, raw, surface_only)

## The component label at `p`, or -1 when off-grid. Two points with different labels cannot reach each other.
func component_at(p: Vector3, surface_only: bool) -> int:
	if not is_active():
		return -1
	return _nav.call(&"component_at", _id, p, surface_only)

## A unit vector away from the solid a surface cell touches, or zero when it touches none.
func surface_normal_hint(c: Vector3i) -> Vector3:
	if not is_active():
		return Vector3.ZERO
	return _nav.call(&"surface_normal_hint", _id, c)

## A cell of component `comp`, selected by scanning forward from `draw` and wrapping.
##
## `draw` is one integer from the caller's OWN generator. This deliberately does not reproduce any particular
## generator: every member is reachable and the result is a pure function of `draw`, which is what a
## reproducible test needs and all a wander target deserves.
func random_cell_in_component(comp: int, surface_only: bool, draw: int) -> Vector3i:
	if not is_active():
		return Nav.NO_CELL
	return _nav.call(&"random_cell_in_component", _id, comp, surface_only, draw)

## The point where the segment from `inside` toward `toward` leaves the volume, inset one cell so a snap
## around it stays in bounds. Returns `toward` when the segment never exits.
func exit_point(inside: Vector3, toward: Vector3) -> Vector3:
	if not is_active():
		return toward
	return _nav.call(&"exit_point", _id, inside, toward)

## Whether `p` lies inside the volume's box.
func contains_point(p: Vector3) -> bool:
	if not is_active():
		return false
	return _nav.call(&"contains_point", _id, p)

## Queue a search. Poll the returned job, then [method NavPathJob.take] its waypoints.
func request_path(from_p: Vector3, to_p: Vector3, surface_only: bool, max_expansions: int,
		start_snap_r: int = -1) -> NavPathJob:
	if not is_active():
		return NavPathJob.new(null, 0, 0)
	var r: int = start_snap_r
	if r < 0:
		r = Nav.SURFACE_SNAP_R if surface_only else Nav.FREE_SNAP_R
	var job: int = _nav.call(&"request_path", _id, from_p, to_p, surface_only, max_expansions, r)
	return NavPathJob.new(_nav, _id, job)

## Release the volume and forget every job about it. Freeing twice is not an error.
func free_volume() -> void:
	if not is_active():
		return
	_nav.call(&"free_volume", _id)
	_id = 0
	_nav = null
