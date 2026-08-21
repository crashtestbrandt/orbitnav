# Contributing to OrbitNav

This file is the authority on this repository: layout, the boundary the gates enforce, the GDScript rules,
the addon-sync model, where coverage belongs and how decisions get recorded.

## Two commands

```sh
just native-install    # required once after cloning: nothing here ships a binary
just check             # the whole gate, fail-fastest first
```

`just check` runs in this order deliberately: the gates that need no toolchain (two greps and a file
comparison), then Rust, then the Godot suites. A broken boundary reports in a second rather than after a
Godot download.

## Layout

The repo root is **not** a Godot project. OrbitNav is configured through an `[orbitnav]` project-settings
block, and two projects cannot share one, so each is its own project with its own COPY of the addon.

```
addons/orbitnav/          canonical GDScript surface: nav.gd, the handles, plugin.cfg
addons/orbitnav_native/   the .gdextension; bin/ is gitignored and empty in a fresh clone
native/                   the cargo workspace, deliberately OUTSIDE the shipped addon
harness/                  a Godot project that exercises the addon on its own terms
tools/                    the gates, the build script, the canonical test harness
```

## The boundary, and why it is grep-enforced

**The backend class is named in exactly one file: `addons/orbitnav/nav.gd`.** `tools/nav-check.sh` fails the
build if any other file names it, and the gate scans `harness/` as well as the addon.

That is not tidiness. It is what lets a consuming project depend on `Nav` without ever knowing a native
extension exists, which in turn is what makes "no backend installed" a supported state rather than a crash.
A test that needs a backend symbol is a test proving the facade has a hole: **widen the facade, do not
exempt the caller.**

The one exception is `tools/orbitnav-smoke.sh`, whose whole job is to assert the class registers in a
project containing nothing else. It is not scanned.

## Rust

One rule: **`orbitnav-core` never sees a `Variant`, and never depends on `godot`.** Its `[dependencies]`
table is empty and stays empty — `std::thread` and `std::sync` are the whole of the concurrency toolkit.
`orbitnav-godot` is registration, conversion and the handle registries; every algorithm lives in core, so a
gdext upgrade touches one crate and no logic.

`#![forbid(unsafe_code)]` in core. `orbitnav-godot` cannot carry it — the `ExtensionLibrary` impl is unsafe
by construction — and has no other unsafe code.

`just native-test` is fmt, clippy at `-D warnings`, and the suites. **Do not commit binaries.**

### Reproducing another implementation exactly

Parts of this crate exist to replace an implementation elsewhere, and the changeover is gated on producing
identical output rather than merely correct output. That is a stronger constraint than it sounds, and it
drives choices that look wrong in isolation:

- float WIDTH is load-bearing, so `real::Vec3` is `f32` and every widen/narrow is a named `w()` / `n()`;
- iteration ORDER is load-bearing, so component labels follow a linear seed scan and snap ties resolve to
  the first candidate in shell order;
- some quirks are reproduced deliberately, marked `QUIRK:` with what the original does and why copying is
  cheaper than diverging.

If you change any of those, `derive_parity_test.gd` is what will tell you.

## GDScript

`harness/project.godot` promotes four warnings to errors, so these are compile failures:

- every `var`, parameter and return carries an explicit type;
- `Array` and `Dictionary` are always typed;
- do not `as`-cast a `Variant` or pass one to a typed constructor — assign it to a typed local first;
- `@warning_ignore` needs a comment explaining the false positive.

Also: `size()` not `len()`, `push_back()` not `append()`, `append_array()` not `+=` on arrays, `null` not
`None`, `%UniqueName` over `$Path/To/Node`, enums over magic values.

`exclude_addons=true` is set, because the facade's calls into an opaque backend cannot satisfy
`unsafe_method_access` by construction.

## The addon-sync model

`addons/orbitnav/` and `addons/orbitnav_native/` at the repo root are the single source of truth and the
AssetLib payload. Every project here gets a mirror COPY from `tools/sync-addons.sh`; the copies are
gitignored build artifacts.

- `just addon-drift` fails if a copy was edited instead of the canonical source. Local.
- `just addon-tracked` fails if a copy was COMMITTED, creating a second source of truth. CI.

Copies rather than symlinks because Git for Windows checks a symlink out as a text file unless both
`core.symlinks` and Developer Mode are on, which is a cryptic first-run failure for a public repo.

## Where coverage belongs

**Default to a unit test.**

- Pure logic → a Rust `#[cfg(test)]` module beside it. These run in milliseconds.
- Something a consumer observes through the facade → a `harness/tests/unit/*_test.gd` suite.
- Something only a real load can prove → the load smoke, or `tools/orbitnav-smoke.sh`.

An empty run is a failure, not a pass: the runner exits non-zero on zero suites and on zero test methods,
and `derive_parity_test.gd` asserts the extension is loaded rather than skipping when it is not. A suite
that quietly passes with no backend reports green for a build that navigates nothing.

## Recording decisions

**No ADRs.** A decision goes in the README, a page under `docs/`, or the header comment of the file it
governs. Header comments here are deliberately long and carry the reasoning rather than pointing at it;
when you change a decision, change its comment in the same commit.

## Writing for a reader who has only this repository

This is a public repository consumed by projects it knows nothing about.

- Do not name another project's classes, scenes, arenas, issue numbers or docs pages. A reader cannot
  resolve any of them.
- Where a concrete example earns its place, describe it in general terms.
- Concise, bulleted, aphorism-free. No metaphor, no epigram, no teaser clause joined by a colon. Record the
  rule and its consequence; skip the narrative of how it was discovered.

**A PR title is a release-notes line** — the release workflow quotes it verbatim. Form:
`<type>(<area>) <plain description>`, type one of `feat`, `fix`, `perf`, `refactor`, `docs`, `test`,
`build`, `chore`; area is the part of the addon the change lands in (`core`, `derive`, `astar`, `jobs`,
`facade`, `binding`, `harness`, `ci`, …).

## Licensing

MIT OR Apache-2.0, inbound equals outbound, no CLA. A new dependency goes into `THIRD_PARTY.md` in the same
commit that adds it.

## Pull requests

One change per PR, and say what it is *for*. CI runs on GitHub-hosted runners under `pull_request`, so fork
PRs need no approval and get no secrets.
