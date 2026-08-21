extends Node
## THE navigation facade, and the whole public surface of OrbitNav.
##
## Everything a consumer needs is reached through `Nav` or through the handles it hands out. **This is the
## only file in the project that may name the backend class**; `tools/nav-check.sh` greps for any other
## reference and fails the build. That boundary is what lets the backend be replaced, downgraded or absent
## without a single edit outside this file.
##
## ## What the layer does
##
## A [NavVolumeHandle] is a uniform grid of cubic cells over some region of the world. The consumer decides
## which cells hold solid material -- that means asking a physics engine, which is the one part of the bake
## this addon deliberately does not own -- and uploads the answer as one byte per cell. From that, the
## backend derives which free cells touch solid, how enclosed each one is, and which cells can reach each
## other; then it answers snap, line-of-sight and pathfinding queries against it.
##
## ## Work happens on a worker, and the caller never waits
##
## Deriving a volume and running a search both go to a pool. A [NavPathJob] is a handle to one of those:
## poll it, and take the result when it settles. Nothing in this file blocks.
##
## ## No backend is a supported state
##
## If the GDExtension is missing, older than this script, or was never installed, every call still answers.
## `is_available()` reads false, factories return INERT handles rather than null, and each query answers the
## value that means "nothing known": no cell, no sight, no component, no route. A consumer written against
## this facade runs -- badly, but without a crash and without a null check at every call site.

## How a search ended. The distinction the caller cannot otherwise make: an empty path may mean the route
## does not exist, that the budget ran out, or that an end had no usable cell nearby, and those want three
## different responses.
enum Outcome {
	PENDING = 0,       ## Still running.
	OK = 1,            ## A route was found.
	DIRECT = 2,        ## The straight line was already clear.
	UNREACHABLE = 3,   ## Different components. No budget will help; stop asking.
	CAPPED = 4,        ## The expansion cap stopped a live frontier. Ask again with more budget.
	UNSNAPPABLE = 5,   ## An end had no usable cell within its snap radius. Move the request.
	UNBUILT = 6,       ## The volume carries no derived data yet.
	STALE = 7,         ## The volume was republished mid-flight. Re-request.
	FAILED = 8,        ## The search could not run.
}

## Job states, as the backend reports them.
enum JobState {
	UNKNOWN = 0,
	PENDING = 1,
	RUNNING = 2,
	READY = 3,
	FAILED = 4,
	CANCELLED = 5,
	STALE = 6,
}

## The sentinel every cell-returning query answers with when there is no such cell.
const NO_CELL: Vector3i = Vector3i(-1, -1, -1)

## The snap radius a surface-restricted query uses. Wider than the open-space default because a mover on a
## surface routinely sits several cells off it. Both ends of a search must agree on this, or a mover can be
## able to PLAN a route and unable to be given a destination.
const SURFACE_SNAP_R: int = 9

## The open-space default snap radius.
const FREE_SNAP_R: int = 5

var _nav: Object = null
var _available: bool = false
var _method_cache: Dictionary[StringName, bool] = {}

func _init() -> void:
	# _init, not _ready: a unit-test runner in --script SceneTree mode calls into the facade before the tree
	# delivers _ready, and the no-backend contract has to hold there too. A child attached here enters the
	# tree with the autoload, so the backend's own lifecycle is unchanged.
	if not ClassDB.class_exists(&"OrbitNav"):
		push_warning("OrbitNav: the native extension is not loaded; navigation queries will answer inert values.")
		return
	_nav = ClassDB.instantiate(&"OrbitNav")
	if _nav == null:
		return
	var node: Node = _nav as Node
	if node == null:
		_nav = null
		return
	node.name = "OrbitNav"
	var threads: int = ProjectSettings.get_setting(&"orbitnav/worker_threads", 0)
	if _backend_has(&"set_worker_threads"):
		node.call(&"set_worker_threads", threads)
	add_child(node)
	_available = true

## Whether the backend loaded. False is a supported state; see the class documentation.
func is_available() -> bool:
	return _available

## How many workers the pool holds. 0 when there is no backend.
func worker_count() -> int:
	if not _available or not _backend_has(&"worker_count"):
		return 0
	return _nav.call(&"worker_count")

## `[threads, queued, running, ready]`. All zeroes when there is no backend.
func pool_stats() -> PackedInt64Array:
	if not _available or not _backend_has(&"pool_stats"):
		return PackedInt64Array([0, 0, 0, 0])
	return _nav.call(&"pool_stats")

## A grid of `dims` cells of `cell_size` metres from world-space min corner `origin`.
##
## Never returns null. With no backend the handle is INERT: `is_active()` reads false and every query answers
## its own "nothing known" value.
func make_volume(origin: Vector3, cell_size: float, dims: Vector3i) -> NavVolumeHandle:
	if not _available:
		return NavVolumeHandle.new(null, 0)
	var id: int = _nav.call(&"create_volume", origin, cell_size, dims)
	return NavVolumeHandle.new(_nav if id != 0 else null, id)

# --- backend access -------------------------------------------------------------------------------------
# A backend older than this script is a supported state, not an error. `Object.get` answers null for a
# property that does not exist rather than failing, `Object.set` is a silent no-op, and `has_method` is
# memoized here because the answer cannot change within a run and some of these are asked per frame.

func _backend_has(method: StringName) -> bool:
	if _nav == null:
		return false
	if _method_cache.has(method):
		return _method_cache[method]
	var present: bool = _nav.has_method(method)
	_method_cache[method] = present
	return present

func _backend_int(name: StringName, fallback: int) -> int:
	if _nav == null:
		return fallback
	var raw: Variant = _nav.get(name)
	if raw == null:
		return fallback
	var value: int = raw
	return value
