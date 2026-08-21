class_name RefDerive
extends RefCounted
## An INDEPENDENT GDScript transcription of the two derive phases, written from the specification rather
## than from the Rust.
##
## This exists to catch the one failure the dump-based tests cannot. A dump proves the port reproduces a
## recorded output; a re-derive of the port's own output proves it is self-consistent. Neither catches a port
## that faithfully reproduces a WRONG rule, because both compare the implementation against itself.
##
## So: a second implementation, in a different language, by a different route, and a suite that runs both over
## generated worlds and demands identical arrays. When they disagree, one of them is wrong and the
## disagreement says where.
##
## Deliberately slow and obvious -- no stride arithmetic, no bounding box, no early exits. Its only job is to
## be independently correct, and every shortcut is a chance to make the same mistake twice.

## Bit per axis neighbour, in the fixed order `-x, +x, -y, +y, -z, +z`.
const DIRS: Array[Vector3i] = [
	Vector3i(-1, 0, 0), Vector3i(1, 0, 0),
	Vector3i(0, -1, 0), Vector3i(0, 1, 0),
	Vector3i(0, 0, -1), Vector3i(0, 0, 1),
]

var dims: Vector3i = Vector3i.ZERO
var solid: PackedByteArray = PackedByteArray()

func _init(d: Vector3i, occupancy: PackedByteArray) -> void:
	dims = d
	solid = occupancy

func cell_count() -> int:
	return dims.x * dims.y * dims.z

func index_of(c: Vector3i) -> int:
	return c.x + dims.x * (c.y + dims.y * c.z)

func in_bounds(c: Vector3i) -> bool:
	return c.x >= 0 and c.y >= 0 and c.z >= 0 and c.x < dims.x and c.y < dims.y and c.z < dims.z

## True when `c` is inside the grid AND holds solid material. Out of bounds is OPEN, not solid.
func is_solid(c: Vector3i) -> bool:
	if not in_bounds(c):
		return false
	return solid[index_of(c)] != 0

## `surface_bits`, `solid_sides` and `surface_flag`, derived the long way.
func surface_arrays() -> Dictionary[String, PackedByteArray]:
	var n: int = cell_count()
	var bits: PackedByteArray = PackedByteArray()
	var sides: PackedByteArray = PackedByteArray()
	var flag: PackedByteArray = PackedByteArray()
	bits.resize(n)
	sides.resize(n)
	flag.resize(n)
	for z: int in dims.z:
		for y: int in dims.y:
			for x: int in dims.x:
				var c: Vector3i = Vector3i(x, y, z)
				var i: int = index_of(c)
				if is_solid(c):
					continue   # a solid cell classifies as nothing
				var mask: int = 0
				for d: int in DIRS.size():
					if is_solid(c + DIRS[d]):
						mask |= 1 << d
				bits[i] = mask
				var count: int = 0
				for d: int in DIRS.size():
					if mask & (1 << d) != 0:
						count += 1
				sides[i] = count
				# The 26-neighbourhood, walked in full. No short-circuit on `mask`: this must be able to
				# disagree with an implementation that takes one.
				var touching: bool = false
				for dz: int in [-1, 0, 1]:
					for dy: int in [-1, 0, 1]:
						for dx: int in [-1, 0, 1]:
							if is_solid(c + Vector3i(dx, dy, dz)):
								touching = true
				flag[i] = 1 if touching else 0
	return {"surface_bits": bits, "solid_sides": sides, "surface_flag": flag}

## `comp_free` and `comp_surf`, labelled by a plain flood fill seeded on a linear scan.
func component_arrays(surface_flag: PackedByteArray) -> Dictionary[String, PackedInt32Array]:
	return {
		"comp_free": _label(surface_flag, false),
		"comp_surf": _label(surface_flag, true),
	}

func _label(surface_flag: PackedByteArray, surface_pass: bool) -> PackedInt32Array:
	var n: int = cell_count()
	var labels: PackedInt32Array = PackedInt32Array()
	labels.resize(n)
	labels.fill(-1)
	var next_label: int = 0
	for seed: int in n:
		if labels[seed] != -1 or not _member(seed, surface_flag, surface_pass):
			continue
		labels[seed] = next_label
		var frontier: Array[int] = [seed]
		while not frontier.is_empty():
			var i: int = frontier.pop_back()
			var c: Vector3i = _cell_of(i)
			for d: int in DIRS.size():
				var nb: Vector3i = c + DIRS[d]
				if not in_bounds(nb):
					continue
				var j: int = index_of(nb)
				if labels[j] == -1 and _member(j, surface_flag, surface_pass):
					labels[j] = next_label
					frontier.push_back(j)
		next_label += 1
	return labels

func _member(i: int, surface_flag: PackedByteArray, surface_pass: bool) -> bool:
	if solid[i] != 0:
		return false
	return not surface_pass or surface_flag[i] != 0

func _cell_of(i: int) -> Vector3i:
	var x: int = i % dims.x
	var rest: int = i / dims.x
	return Vector3i(x, rest % dims.y, rest / dims.y)
