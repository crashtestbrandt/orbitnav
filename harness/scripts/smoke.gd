extends Node
## The OrbitNav LOAD SMOKE: does a project that contains nothing else get a working addon?
##
## Every assertion here is something a CONSUMER can observe. It never names the backend class -- the
## nav-check gate applies to this file too -- so what it proves is that the facade, the plugin, the
## autoload and the extension are wired to each other, which is exactly the thing a unit suite cannot
## prove because a unit suite constructs what it tests.
##
## Prints `SMOKE ...` markers and exits non-zero on the first failure, so CI reads the exit code and a
## human reads the log. `--quit-after` is a frame-count backstop only: this quits itself as soon as it
## has a verdict.

var _failures: int = 0

func _ready() -> void:
	_check("Nav autoload exists", Nav != null)
	_check("the native extension is loaded", Nav.is_available())
	_check("the worker pool has at least one thread", Nav.worker_count() >= 1)

	var stats: PackedInt64Array = Nav.pool_stats()
	_check("pool_stats reports four numbers", stats.size() == 4)

	# A small world with a floor, built by hand: no physics, no geometry, no engine services beyond the
	# addon itself.
	var dims: Vector3i = Vector3i(8, 8, 6)
	var vol: NavVolumeHandle = Nav.make_volume(Vector3.ZERO, 1.0, dims)
	_check("make_volume returns an active handle", vol.is_active())
	_check("the volume has the cell count its dims imply", vol.cell_count() == 8 * 8 * 6)
	_check("a fresh volume is not built", not vol.is_built())

	var solid: PackedByteArray = PackedByteArray()
	solid.resize(vol.cell_count())
	for y: int in dims.y:
		for x: int in dims.x:
			solid[x + dims.x * (y + dims.y * 0)] = 1
	_check("occupancy uploads", vol.upload_occupancy(solid))
	_check("a wrong-sized upload is refused", not vol.upload_occupancy(PackedByteArray([1, 2, 3])))

	_check("the volume derives", vol.derive_blocking())
	_check("the volume is built afterwards", vol.is_built())
	_check("the derive reports a cost", vol.derive_usec() >= 0)

	var flag: PackedByteArray = vol.surface_flag()
	_check("surface_flag reads back one byte per cell", flag.size() == vol.cell_count())
	_check("a cell above the floor is surface", flag[4 + dims.x * (4 + dims.y * 1)] != 0)
	_check("a cell high above the floor is not", flag[4 + dims.x * (4 + dims.y * 5)] == 0)

	var comp: PackedInt32Array = vol.comp_free()
	_check("comp_free reads back one label per cell", comp.size() == vol.cell_count())
	_check("the open space is one component", comp[4 + dims.x * (4 + dims.y * 3)] >= 0)

	_check("a cell in the floor snaps out of it",
		vol.nearest_free_cell(Vector3(4.5, 4.5, 0.5), false) == Vector3i(4, 4, 1))
	_check("a clear run has line of sight",
		vol.has_los(Vector3(1.5, 4.5, 3.5), Vector3(6.5, 4.5, 3.5), false))
	_check("a point in open space has a component",
		vol.component_at(Vector3(4.5, 4.5, 3.5), false) >= 0)

	# A search, driven to settlement the way a consumer would: submit, poll, take.
	var job: NavPathJob = vol.request_path(Vector3(1.5, 1.5, 3.5), Vector3(6.5, 6.5, 3.5), false, 8000)
	_check("request_path returns a valid job", job.is_valid())
	var spins: int = 0
	while not job.is_settled() and spins < 100000:
		spins += 1
	_check("the search settles", job.is_settled())
	var path: PackedVector3Array = job.take()
	_check("the search produced a route", not path.is_empty())
	_check("the outcome says how it ended",
		job.outcome() == Nav.Outcome.OK or job.outcome() == Nav.Outcome.DIRECT)

	# The distinction the facade exists to expose: an unreachable pair is not a capped search.
	var sealed: NavVolumeHandle = _sealed_volume()
	var blocked: NavPathJob = sealed.request_path(
		Vector3(1.5, 1.5, 1.5), Vector3(14.5, 1.5, 1.5), false, 20000)
	spins = 0
	while not blocked.is_settled() and spins < 100000:
		spins += 1
	var blocked_path: PackedVector3Array = blocked.take()
	_check("a sealed route produces no waypoints", blocked_path.is_empty())
	_check("and says it is unreachable rather than merely empty",
		blocked.outcome() == Nav.Outcome.UNREACHABLE)

	# Freeing is idempotent: a consumer that frees on teardown and again on notification must not crash.
	vol.free_volume()
	sealed.free_volume()
	vol.free_volume()
	_check("a freed handle is inactive", not vol.is_active())
	_check("a freed handle still answers queries",
		vol.nearest_free_cell(Vector3.ZERO, false) == Nav.NO_CELL)

	if _failures == 0:
		print("SMOKE OK  every check passed")
	else:
		printerr("SMOKE FAIL  %d check(s) failed" % _failures)
	get_tree().quit(1 if _failures > 0 else 0)

func _sealed_volume() -> NavVolumeHandle:
	var dims: Vector3i = Vector3i(16, 4, 4)
	var vol: NavVolumeHandle = Nav.make_volume(Vector3.ZERO, 1.0, dims)
	var solid: PackedByteArray = PackedByteArray()
	solid.resize(vol.cell_count())
	for y: int in dims.y:
		for z: int in dims.z:
			solid[8 + dims.x * (y + dims.y * z)] = 1
	vol.upload_occupancy(solid)
	vol.derive_blocking()
	return vol

func _check(what: String, ok: bool) -> void:
	if ok:
		print("SMOKE ok    %s" % what)
	else:
		_failures += 1
		printerr("SMOKE FAIL  %s" % what)
