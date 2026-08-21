extends UnitTest
## The facade's own contract: handles, job lifecycle, and the outcomes a caller has to distinguish.
##
## The derive is covered by derive_parity_test.gd and by the Rust suites. This one covers the seam --
## what a consumer sees when it holds a handle, polls a job, or asks a question the volume cannot answer.

func test_the_backend_is_present() -> void:
	assert_true(Nav.is_available(), "the native extension is loaded")
	assert_true(Nav.worker_count() >= 1, "the pool has at least one worker")

func test_pool_stats_has_four_slots() -> void:
	var stats: PackedInt64Array = Nav.pool_stats()
	assert_eq(stats.size(), 4, "pool_stats is [threads, queued, running, ready]")
	assert_true(stats[0] >= 1, "the thread count is the pool's own")

func test_a_volume_reports_its_shape() -> void:
	var vol: NavVolumeHandle = Nav.make_volume(Vector3(-4.0, 0.0, 2.0), 1.0, Vector3i(5, 7, 3))
	assert_true(vol.is_active(), "the handle is active")
	assert_eq(vol.cell_count(), 5 * 7 * 3, "cell_count follows dims")
	assert_false(vol.is_built(), "a fresh volume carries no derived data")
	assert_true(vol.generation() >= 1, "a volume starts at generation 1 or later")
	vol.free_volume()

func test_a_degenerate_volume_is_refused() -> void:
	var vol: NavVolumeHandle = Nav.make_volume(Vector3.ZERO, 1.0, Vector3i(0, 8, 8))
	assert_false(vol.is_active(), "a zero extent has no cells, so there is no volume")

func test_uploading_bumps_the_generation() -> void:
	# The mechanism staleness rests on: a job carries the generation it was submitted under, so
	# republishing the volume must be visible.
	var vol: NavVolumeHandle = _floor(Vector3i(8, 8, 6))
	var before: int = vol.generation()
	var solid: PackedByteArray = PackedByteArray()
	solid.resize(vol.cell_count())
	assert_true(vol.upload_occupancy(solid), "re-upload succeeds")
	assert_true(vol.generation() > before, "the generation moved")
	vol.free_volume()

func test_an_unbuilt_volume_answers_inert_values() -> void:
	var vol: NavVolumeHandle = Nav.make_volume(Vector3.ZERO, 1.0, Vector3i(8, 8, 6))
	assert_eq(vol.nearest_free_cell(Vector3(4.5, 4.5, 4.5), false), Nav.NO_CELL, "no cell without a derive")
	assert_false(vol.has_los(Vector3.ZERO, Vector3.ONE, false), "no sight without a derive")
	assert_eq(vol.component_at(Vector3(4.5, 4.5, 4.5), false), -1, "no component without a derive")
	assert_true(vol.surface_flag().is_empty(), "no arrays without a derive")
	vol.free_volume()

func test_a_freed_handle_is_inert_rather_than_broken() -> void:
	var vol: NavVolumeHandle = _floor(Vector3i(8, 8, 6))
	vol.free_volume()
	vol.free_volume()   # freeing twice must not be an error
	assert_false(vol.is_active(), "the handle knows it is gone")
	assert_eq(vol.nearest_free_cell(Vector3.ZERO, false), Nav.NO_CELL, "and still answers")
	assert_eq(vol.cell_count(), 0, "with nothing")

func test_a_found_route_reports_ok_or_direct() -> void:
	var vol: NavVolumeHandle = _floor(Vector3i(10, 10, 6))
	var job: NavPathJob = vol.request_path(Vector3(1.5, 1.5, 3.5), Vector3(8.5, 8.5, 3.5), false, 8000)
	var path: PackedVector3Array = _settle(job)
	assert_false(path.is_empty(), "open space has a route")
	assert_true(job.outcome() == Nav.Outcome.OK or job.outcome() == Nav.Outcome.DIRECT,
		"a route reports OK or DIRECT, got %d" % job.outcome())
	vol.free_volume()

func test_a_sealed_pair_reports_unreachable() -> void:
	var vol: NavVolumeHandle = _sealed()
	var job: NavPathJob = vol.request_path(Vector3(1.5, 1.5, 1.5), Vector3(14.5, 1.5, 1.5), false, 20000)
	var path: PackedVector3Array = _settle(job)
	assert_true(path.is_empty(), "no route across a sealed wall")
	assert_eq(job.outcome(), Nav.Outcome.UNREACHABLE, "and the caller is told to stop asking")
	vol.free_volume()

func test_a_starved_search_reports_capped_rather_than_unreachable() -> void:
	# THE distinction the enum exists for. Both answers are an empty path; only one of them means
	# "ask again with more budget".
	var vol: NavVolumeHandle = _maze()
	var job: NavPathJob = vol.request_path(Vector3(1.5, 1.5, 1.5), Vector3(28.5, 8.5, 1.5), false, 6)
	var path: PackedVector3Array = _settle(job)
	assert_true(path.is_empty(), "six expansions cannot cross a maze")
	assert_eq(job.outcome(), Nav.Outcome.CAPPED, "and that is not the same as unreachable")
	vol.free_volume()

func test_taking_a_job_twice_yields_nothing_the_second_time() -> void:
	var vol: NavVolumeHandle = _floor(Vector3i(10, 10, 6))
	var job: NavPathJob = vol.request_path(Vector3(1.5, 1.5, 3.5), Vector3(8.5, 8.5, 3.5), false, 8000)
	var first: PackedVector3Array = _settle(job)
	assert_false(first.is_empty(), "the first take gets the route")
	assert_true(job.take().is_empty(), "the second gets nothing")
	vol.free_volume()

func test_a_cancelled_job_yields_nothing() -> void:
	var vol: NavVolumeHandle = _floor(Vector3i(10, 10, 6))
	var job: NavPathJob = vol.request_path(Vector3(1.5, 1.5, 3.5), Vector3(8.5, 8.5, 3.5), false, 8000)
	job.cancel()
	assert_true(job.take().is_empty(), "a cancelled job has no result to take")
	vol.free_volume()

func test_smoothing_without_a_backend_volume_returns_its_input() -> void:
	var vol: NavVolumeHandle = Nav.make_volume(Vector3.ZERO, 1.0, Vector3i(8, 8, 6))
	var raw: PackedVector3Array = PackedVector3Array([Vector3.ONE, Vector3(2, 2, 2)])
	assert_eq(vol.smooth_path(Vector3.ZERO, raw, false), raw, "an unbuilt volume smooths nothing away")
	vol.free_volume()

func test_the_surface_snap_radius_is_wider_than_the_free_one() -> void:
	# The two must differ, and a surface query must default to the wider one, or a mover can plan a route
	# it cannot be given a destination for.
	assert_true(Nav.SURFACE_SNAP_R > Nav.FREE_SNAP_R, "surface snaps wider than free flight")
	var vol: NavVolumeHandle = _floor(Vector3i(12, 12, 10))
	var high: Vector3 = Vector3(5.5, 5.5, 7.5)
	assert_eq(vol.nearest_free_cell(high, true, Nav.FREE_SNAP_R), Nav.NO_CELL, "too far at the free radius")
	assert_true(vol.nearest_free_cell(high, true) != Nav.NO_CELL, "found at the surface default")
	vol.free_volume()

# --- fixtures -------------------------------------------------------------------------------------------

func _floor(dims: Vector3i) -> NavVolumeHandle:
	var vol: NavVolumeHandle = Nav.make_volume(Vector3.ZERO, 1.0, dims)
	var solid: PackedByteArray = PackedByteArray()
	solid.resize(vol.cell_count())
	for y: int in dims.y:
		for x: int in dims.x:
			solid[x + dims.x * (y + dims.y * 0)] = 1
	vol.upload_occupancy(solid)
	vol.derive_blocking()
	return vol

func _sealed() -> NavVolumeHandle:
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

## A long corridor with staggered baffles, so no straight run exists and a small budget cannot finish.
func _maze() -> NavVolumeHandle:
	var dims: Vector3i = Vector3i(30, 10, 3)
	var vol: NavVolumeHandle = Nav.make_volume(Vector3.ZERO, 1.0, dims)
	var solid: PackedByteArray = PackedByteArray()
	solid.resize(vol.cell_count())
	for x: int in range(3, 28, 4):
		var gap_low: bool = (x / 4) % 2 == 0
		for y: int in dims.y:
			if gap_low and y < 2:
				continue
			if not gap_low and y > dims.y - 3:
				continue
			for z: int in dims.z:
				solid[x + dims.x * (y + dims.y * z)] = 1
	vol.upload_occupancy(solid)
	vol.derive_blocking()
	return vol

func _settle(job: NavPathJob) -> PackedVector3Array:
	var spins: int = 0
	while not job.is_settled() and spins < 200000:
		spins += 1
	return job.take()
