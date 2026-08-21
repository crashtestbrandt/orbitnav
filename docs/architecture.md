# Architecture

## The two crates

| Crate | Holds | Depends on |
|---|---|---|
| `orbitnav-core` | every algorithm | nothing |
| `orbitnav-godot` | registration, conversion, the handle registries | `orbitnav-core`, `godot` |

The split is what keeps a gdext upgrade from touching logic, and what lets the algorithms run under
`cargo test` in milliseconds with no engine anywhere.

## The modules

| Module | Responsibility |
|---|---|
| `real` | the float widths, made explicit: `Vec3` is `f32`, and `w()` / `n()` name every conversion |
| `grid` | addressing, bounds, and the world/cell mapping |
| `occupancy` | which cells hold solid material, plus the solid bounding box |
| `surface` | which free cells touch solid, and how enclosed each one is |
| `components` | which free cells reach each other, labelled twice |
| `los` | whether a straight segment crosses only usable cells |
| `snap` | the usable cell nearest an arbitrary point |
| `astar` | the search, and the outcome of asking for a route |
| `smooth` | collapsing a cell-by-cell route into waypoints |
| `volume` | the finished product: occupancy plus the five derived arrays |
| `jobs` | the worker pool |
| `navd` | the dump format, for differential testing |

## What crosses the boundary

Whole arrays, once per operation. A crossing costs about 0.185 µs — nothing per operation, and fatal per
cell: a per-cell API over a two-million-cell volume would spend more time crossing than working. So a bake
is about ten crossings and a search about five, and `convert.rs` offers no per-element entry point to reach
for.

Reading the five derived arrays back into the caller costs a copy and buys a consumer that can keep every
existing reader of those arrays unchanged.

## Concurrency

- **Volumes are immutable snapshots.** Publishing new occupancy or a new derive replaces an `Arc` and bumps
  a generation; it never mutates what a running job is reading.
- **A job records the generation it was submitted under.** A result about an older one reports as stale
  rather than being handed back, so a mover is never steered by geometry that is gone.
- **No lock is held across a call back into the caller**, and nothing blocks the submitting thread.
- **Workers run under `catch_unwind`**, so a panic is a failed job rather than a dead editor. `panic =
  "abort"` is deliberately unset for the same reason.
- **`Gd` is not `Send`**, so touching an engine object from a worker is a compile error rather than a race.

## Determinism

The bar is reproducibility for tests and differential comparison, not agreement between machines over a
wire. There is no protocol, no schema hash and no version negotiation.

Two consequences:

- **SURFACE is chunked across workers and is bit-identical to a serial run by construction**, because every
  cell's outputs depend only on the occupancy array and the solid bounding box. A test derives at one and
  seven workers and demands equality.
- **COMPONENTS is single-threaded.** Its label numbering is a function of the linear seed scan, and a
  parallel union-find would produce different — equally valid — labels, which is exactly what a byte-for-byte
  comparison against another implementation cannot absorb.

Searches run in parallel across jobs and serially within one, each owning private scratch.
