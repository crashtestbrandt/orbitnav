class_name NavPathJob
extends RefCounted
## A handle to work running on a worker: a volume derive, or one search.
##
## Obtained from [method NavVolumeHandle.derive] or [method NavVolumeHandle.request_path], never constructed
## directly. Polling costs a lock and a read; nothing here blocks the caller.
##
## [codeblock]
## var job: NavPathJob = vol.request_path(from_p, to_p, false, 6000)
## # ... on a later frame ...
## if job.is_settled():
##     var path: PackedVector3Array = job.take()
##     if path.is_empty():
##         match job.outcome():
##             Nav.Outcome.CAPPED:      pass  # ask again with more budget
##             Nav.Outcome.UNREACHABLE: pass  # never ask again
##             Nav.Outcome.UNSNAPPABLE: pass  # move the request
## [endblock]
##
## ## Staleness
##
## A job is submitted against one publication of a volume. If the volume is re-uploaded or re-derived while
## the job is in flight, the job still finishes -- against the snapshot it started with, which cannot tear --
## but [method state] then reads [constant Nav.JobState.STALE] and the result is not handed back. The caller
## re-requests. That is the whole invalidation story: no aborts, and a stale answer is visible rather than
## silently steering a mover by geometry that is gone.
##
## An INERT job (no backend) is settled immediately, takes an empty path, and reports
## [constant Nav.Outcome.UNBUILT].

var _nav: Object = null
var _vol: int = 0
var _id: int = 0
var _taken: bool = false
var _outcome: int = Nav.Outcome.PENDING

func _init(backend: Object, vol: int, id: int) -> void:
	_nav = backend
	_vol = vol
	_id = id
	if backend == null or id == 0:
		_outcome = Nav.Outcome.UNBUILT
		_taken = true

## The backend's own handle for this job. For the volume's derive plumbing.
func id() -> int:
	return _id

## Whether this job refers to live work.
func is_valid() -> bool:
	return _nav != null and _id != 0 and not _taken

## Where the job is: one of [enum Nav.JobState].
func state() -> int:
	if _nav == null or _id == 0 or _taken:
		return Nav.JobState.UNKNOWN
	return _nav.call(&"job_state", _vol, _id)

## Whether the job has stopped, whatever the answer. A stale or failed job is settled too.
func is_settled() -> bool:
	if _nav == null or _id == 0:
		return true
	if _taken:
		return true
	var s: int = state()
	return s != Nav.JobState.PENDING and s != Nav.JobState.RUNNING

## Take the waypoints. Empty when the search found no route, or when the job is not ready.
##
## The job is consumed: a second call answers empty. Read [method outcome] afterwards to learn why an empty
## result is empty.
func take() -> PackedVector3Array:
	if _nav == null or _id == 0 or _taken:
		return PackedVector3Array()
	var s: int = state()
	if s == Nav.JobState.STALE:
		_outcome = Nav.Outcome.STALE
		_taken = true
		return PackedVector3Array()
	if s != Nav.JobState.READY:
		if s == Nav.JobState.FAILED:
			_outcome = Nav.Outcome.FAILED
			_taken = true
		return PackedVector3Array()
	var path: PackedVector3Array = _nav.call(&"take_path", _id)
	_outcome = _nav.call(&"last_outcome")
	_taken = true
	return path

## How the search ended: one of [enum Nav.Outcome]. Meaningful once [method take] has been called.
func outcome() -> int:
	return _outcome

## How many cells the search expanded. Meaningful once [method take] has been called.
func expansions() -> int:
	if _nav == null:
		return 0
	return _nav.call(&"last_expansions")

## Abandon the job. A queued job never runs; a running one finishes and its result is dropped.
func cancel() -> void:
	if _nav == null or _id == 0 or _taken:
		return
	_nav.call(&"cancel_job", _id)
	_taken = true
