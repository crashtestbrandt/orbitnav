extends UnitTest
## The derive must agree with an INDEPENDENT implementation, not merely with itself.
##
## [RefDerive] is a second transcription of the same specification, written the long way in GDScript. This
## suite generates worlds, runs both, and demands identical arrays. A port that reproduces a wrong rule
## faithfully passes every self-comparison there is and fails here.
##
## Worlds are generated from a seed rather than hand-drawn, so the coverage is wide enough to hit the cases
## nobody thinks to write: cells on the lattice boundary, cells whose only solid neighbour is a corner,
## components that touch the edge, and volumes with no solid cells at all.

const _SEEDS: Array[int] = [1, 7, 4242]

func test_the_backend_is_present() -> void:
	# ASSERTED, never skipped. A suite that quietly passes when the extension is missing reports green for
	# a build that navigates nothing.
	assert_true(Nav.is_available(), "the native extension is loaded")

func test_an_empty_world_agrees() -> void:
	_compare(Vector3i(10, 8, 6), PackedByteArray(), "an all-free world")

func test_a_full_world_agrees() -> void:
	var dims: Vector3i = Vector3i(8, 8, 8)
	var solid: PackedByteArray = PackedByteArray()
	solid.resize(dims.x * dims.y * dims.z)
	solid.fill(1)
	_compare(dims, solid, "an all-solid world")

func test_a_floor_agrees() -> void:
	var dims: Vector3i = Vector3i(12, 10, 6)
	var solid: PackedByteArray = _blank(dims)
	for y: int in dims.y:
		for x: int in dims.x:
			solid[_index(dims, Vector3i(x, y, 0))] = 1
	_compare(dims, solid, "a floor plane")

func test_a_corner_only_contact_agrees() -> void:
	# The case where surface_bits is 0 and surface_flag is 1. An implementation that derives the flag from
	# the bits gets this wrong and nothing else catches it.
	var dims: Vector3i = Vector3i(6, 6, 6)
	var solid: PackedByteArray = _blank(dims)
	solid[_index(dims, Vector3i(3, 3, 3))] = 1
	_compare(dims, solid, "a single solid cell touched only diagonally")

func test_a_boundary_hugging_slab_agrees() -> void:
	# Solid pressed against the lattice edge, so every out-of-bounds neighbour is consulted.
	var dims: Vector3i = Vector3i(8, 6, 6)
	var solid: PackedByteArray = _blank(dims)
	for y: int in dims.y:
		for z: int in dims.z:
			solid[_index(dims, Vector3i(0, y, z))] = 1
	_compare(dims, solid, "a slab on the lattice boundary")

func test_two_sealed_rooms_agree() -> void:
	var dims: Vector3i = Vector3i(13, 6, 6)
	var solid: PackedByteArray = _blank(dims)
	for y: int in dims.y:
		for z: int in dims.z:
			solid[_index(dims, Vector3i(6, y, z))] = 1
	_compare(dims, solid, "two sealed rooms")

func test_seeded_noise_agrees() -> void:
	for s: int in _SEEDS:
		var dims: Vector3i = Vector3i(11, 9, 7)
		var solid: PackedByteArray = _blank(dims)
		var rng: RandomNumberGenerator = RandomNumberGenerator.new()
		rng.seed = s
		for i: int in solid.size():
			solid[i] = 1 if rng.randf() < 0.25 else 0
		_compare(dims, solid, "seeded noise %d" % s)

func test_seeded_noise_agrees_at_a_non_cubic_size() -> void:
	# A non-cubic grid catches an index formula that happens to work when the extents are equal.
	var dims: Vector3i = Vector3i(17, 5, 9)
	var solid: PackedByteArray = _blank(dims)
	var rng: RandomNumberGenerator = RandomNumberGenerator.new()
	rng.seed = 99
	for i: int in solid.size():
		solid[i] = 1 if rng.randf() < 0.18 else 0
	_compare(dims, solid, "non-cubic noise")

## Build the world through the backend and through [RefDerive], and demand every array match.
func _compare(dims: Vector3i, occupancy: PackedByteArray, label: String) -> void:
	var solid: PackedByteArray = occupancy
	if solid.is_empty():
		solid = _blank(dims)

	var vol: NavVolumeHandle = Nav.make_volume(Vector3.ZERO, 1.0, dims)
	if not vol.is_active():
		assert_true(false, "%s: could not create a volume" % label)
		return
	assert_true(vol.upload_occupancy(solid), "%s: occupancy uploads" % label)
	assert_true(vol.derive_blocking(), "%s: the volume derives" % label)

	var reference: RefDerive = RefDerive.new(dims, solid)
	var surf: Dictionary[String, PackedByteArray] = reference.surface_arrays()
	var comps: Dictionary[String, PackedInt32Array] = reference.component_arrays(surf["surface_flag"])

	_bytes_eq(vol.surface_bits(), surf["surface_bits"], "%s: surface_bits" % label)
	_bytes_eq(vol.solid_sides(), surf["solid_sides"], "%s: solid_sides" % label)
	_bytes_eq(vol.surface_flag(), surf["surface_flag"], "%s: surface_flag" % label)
	_ints_eq(vol.comp_free(), comps["comp_free"], "%s: comp_free" % label)
	_ints_eq(vol.comp_surf(), comps["comp_surf"], "%s: comp_surf" % label)

	vol.free_volume()

## Compare byte for byte, and report the FIRST disagreement with its index -- a count alone does not say
## where to look.
func _bytes_eq(actual: PackedByteArray, expected: PackedByteArray, label: String) -> void:
	if actual.size() != expected.size():
		assert_eq(actual.size(), expected.size(), "%s size" % label)
		return
	for i: int in expected.size():
		if actual[i] != expected[i]:
			assert_true(false, "%s differs at cell %d (expected %d, got %d)" % [label, i, expected[i], actual[i]])
			return
	assert_true(true, label)

func _ints_eq(actual: PackedInt32Array, expected: PackedInt32Array, label: String) -> void:
	if actual.size() != expected.size():
		assert_eq(actual.size(), expected.size(), "%s size" % label)
		return
	for i: int in expected.size():
		if actual[i] != expected[i]:
			assert_true(false, "%s differs at cell %d (expected %d, got %d)" % [label, i, expected[i], actual[i]])
			return
	assert_true(true, label)

func _blank(dims: Vector3i) -> PackedByteArray:
	var a: PackedByteArray = PackedByteArray()
	a.resize(dims.x * dims.y * dims.z)
	return a

func _index(dims: Vector3i, c: Vector3i) -> int:
	return c.x + dims.x * (c.y + dims.y * c.z)
