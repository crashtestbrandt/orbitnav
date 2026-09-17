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

# --- voxelize -------------------------------------------------------------------------------------------

func test_the_backend_can_voxelize() -> void:
	assert_true(Nav.can_voxelize(), "the native extension stages geometry")

func test_a_box_mesh_voxelizes_to_its_outside_skin() -> void:
	# A BoxMesh's triangles face outward in Godot's own winding, so this pins the face-order conversion: a
	# cell outside a wall is in front of it and touches it, the cell just inside is behind it.
	var vol: NavVolumeHandle = Nav.make_volume(Vector3.ZERO, 1.0, Vector3i(12, 12, 12))
	assert_true(vol.begin_geometry(), "staging opens")
	var mesh: BoxMesh = BoxMesh.new()
	mesh.size = Vector3(6.0, 6.0, 6.0)
	assert_true(vol.add_shape(mesh.create_trimesh_shape(), Transform3D(Basis.IDENTITY, Vector3(6.0, 6.0, 6.0))),
		"a concave shape stages")
	var solid: PackedByteArray = _voxelized(vol)
	assert_eq(solid[_index(vol, Vector3i(9, 6, 6))], 1, "the cell outside the +x wall touches it")
	assert_eq(solid[_index(vol, Vector3i(8, 6, 6))], 0, "the cell inside the +x wall is behind it")
	assert_eq(solid[_index(vol, Vector3i(2, 6, 6))], 1, "the cell outside the -x wall touches it")
	assert_eq(solid[_index(vol, Vector3i(6, 6, 6))], 0, "a mesh leaves its inside free")
	assert_true(vol.is_built(), "the derive landed with it")
	vol.free_volume()

func test_a_double_sided_mesh_claims_both_sides() -> void:
	var vol: NavVolumeHandle = Nav.make_volume(Vector3.ZERO, 1.0, Vector3i(12, 12, 12))
	vol.begin_geometry()
	var mesh: BoxMesh = BoxMesh.new()
	mesh.size = Vector3(6.0, 6.0, 6.0)
	var shape: ConcavePolygonShape3D = mesh.create_trimesh_shape()
	shape.backface_collision = true
	vol.add_shape(shape, Transform3D(Basis.IDENTITY, Vector3(6.0, 6.0, 6.0)))
	var solid: PackedByteArray = _voxelized(vol)
	assert_eq(solid[_index(vol, Vector3i(8, 6, 6))], 1, "the inside of the wall counts too")
	vol.free_volume()

func test_a_convex_shape_is_solid_throughout() -> void:
	var vol: NavVolumeHandle = Nav.make_volume(Vector3.ZERO, 1.0, Vector3i(12, 12, 12))
	vol.begin_geometry()
	var box: BoxShape3D = BoxShape3D.new()
	box.size = Vector3(6.0, 6.0, 6.0)
	var at: Transform3D = Transform3D(Basis.IDENTITY, Vector3(6.0, 6.0, 6.0))
	assert_true(vol.add_shape(box, at), "a box stages")
	var sphere: SphereShape3D = SphereShape3D.new()
	sphere.radius = 1.0
	assert_true(vol.add_shape(sphere, Transform3D(Basis.IDENTITY, Vector3(10.5, 10.5, 10.5))), "a sphere stages")
	assert_true(vol.add_shape(CapsuleShape3D.new(), at), "a capsule stages")
	assert_true(vol.add_shape(CylinderShape3D.new(), at), "a cylinder stages")
	var hull: ConvexPolygonShape3D = ConvexPolygonShape3D.new()
	hull.points = PackedVector3Array([Vector3(1, 1, 1), Vector3(2, 1, 1), Vector3(1, 2, 1), Vector3(1, 1, 2)])
	assert_true(vol.add_shape(hull, Transform3D.IDENTITY), "a convex hull stages")
	assert_false(vol.add_shape(WorldBoundaryShape3D.new(), at), "an unbounded shape is refused")
	var solid: PackedByteArray = _voxelized(vol)
	assert_eq(solid[_index(vol, Vector3i(6, 6, 6))], 1, "the middle of the box is solid")
	assert_eq(solid[_index(vol, Vector3i(10, 10, 10))], 1, "the sphere's cell is solid")
	assert_eq(solid[_index(vol, Vector3i(0, 11, 0))], 0, "a far corner is not")
	vol.free_volume()

func test_regions_leave_cells_outside_them_free() -> void:
	var vol: NavVolumeHandle = Nav.make_volume(Vector3.ZERO, 1.0, Vector3i(12, 12, 12))
	vol.begin_geometry(PackedVector4Array([Vector4(3.0, 6.0, 6.0, 2.0)]))
	var box: BoxShape3D = BoxShape3D.new()
	box.size = Vector3(10.0, 10.0, 10.0)
	vol.add_shape(box, Transform3D(Basis.IDENTITY, Vector3(6.0, 6.0, 6.0)))
	var solid: PackedByteArray = _voxelized(vol)
	assert_eq(solid[_index(vol, Vector3i(3, 6, 6))], 1, "inside the region the box counts")
	assert_eq(solid[_index(vol, Vector3i(8, 6, 6))], 0, "outside it the box does not")
	vol.free_volume()

func test_voxelizing_publishes_a_generation_and_reports_its_cost() -> void:
	var vol: NavVolumeHandle = Nav.make_volume(Vector3.ZERO, 1.0, Vector3i(8, 8, 8))
	var before: int = vol.generation()
	vol.begin_geometry()
	vol.add_shape(BoxShape3D.new(), Transform3D(Basis.IDENTITY, Vector3(4.0, 4.0, 4.0)))
	var job: NavPathJob = vol.voxelize()
	assert_true(vol.generation() > before, "a voxelize republishes the volume")
	_settle_job(job)
	assert_true(vol.apply_derive(job), "and its result installs against that generation")
	assert_true(vol.voxelize_usec() >= 0, "the worker's cost is readable")
	vol.free_volume()

func test_nothing_stages_without_begin_and_nothing_voxelizes_twice() -> void:
	var vol: NavVolumeHandle = Nav.make_volume(Vector3.ZERO, 1.0, Vector3i(8, 8, 8))
	assert_false(vol.add_shape(BoxShape3D.new(), Transform3D.IDENTITY), "no staging, no shape")
	assert_eq(vol.add_faces(PackedVector3Array(), Transform3D.IDENTITY, false), -1, "and no faces")
	assert_true(vol.voxelize().is_settled(), "a voxelize with nothing staged is an inert job")
	vol.begin_geometry()
	assert_eq(vol.add_faces(PackedVector3Array([Vector3.ZERO, Vector3.ONE]), Transform3D.IDENTITY, false), -1,
		"a partial triangle is refused")
	var job: NavPathJob = vol.voxelize()
	assert_true(vol.voxelize().is_settled(), "the staging was taken by the first voxelize")
	_settle_job(job)
	vol.free_volume()

func test_rounding_follows_the_physics_engine() -> void:
	# The harness runs whatever engine its project names; either answer must be one of the two the rule gives.
	var r: float = Nav.box_query_rounding(0.5)
	assert_true(is_equal_approx(r, 0.0) or is_equal_approx(r, 0.04), "0 or Jolt's 0.04 for a 1 m query box, got %f" % r)
	assert_true(Nav.convex_rounding(0.04, 0.01) <= 0.01, "never more than the fraction of a thin shape allows")

# --- fixtures -------------------------------------------------------------------------------------------

func _index(vol: NavVolumeHandle, c: Vector3i) -> int:
	return c.x + 12 * (c.y + 12 * c.z)

func _voxelized(vol: NavVolumeHandle) -> PackedByteArray:
	var job: NavPathJob = vol.voxelize()
	_settle_job(job)
	assert_true(vol.apply_derive(job), "the voxelize installs")
	return vol.solid()

func _settle_job(job: NavPathJob) -> void:
	var spins: int = 0
	while not job.is_settled() and spins < 2000000:
		spins += 1

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
