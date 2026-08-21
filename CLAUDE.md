# OrbitNav — Claude Code context

**CONTRIBUTING.md is the authority on this repository**: layout, the grep-enforced boundary, the GDScript
rules, the addon-sync model, where coverage belongs, and how decisions get recorded. Read it first and
follow it. This file adds what that document does not cover.

## Writing for humans

Everything written for a human reader — PR titles, commit messages, issue and review comments, release
notes, the README, every page under `docs/`, and the header comments CONTRIBUTING.md asks you to write at
length — is **concise, bulleted and aphorism-free**. State plainly what changed and what it now does.

- No metaphor, euphemism, or oblique stand-in for the thing you mean.
- No epigrams or rhetorical inversions ("A *X* is not a *Y*", "not *X*, but *Y*").
- No teaser clause joined by a colon to the real content.
- No emphatic capitalization, and no general truth standing in for the specific change.
- Short sections, bullets over paragraphs, tables for enumerations, identifiers in backticks. Record the
  rule and its consequence; skip the narrative of how it was discovered. Bold the term a rule is about.

A reader who has not seen the diff must learn what changed from the sentence alone.

**A PR title is a release-notes line** — `release.yml` quotes it verbatim on every tag, so it is read by
people who never open the diff. Form: `<type>(<area>) <plain description>`, type one of `feat`, `fix`,
`perf`, `refactor`, `docs`, `test`, `build`, `chore`; area is the part of the addon the change lands in
(`core`, `derive`, `astar`, `jobs`, `facade`, `binding`, `harness`, `ci`, …).

| Not this | This |
| --- | --- |
| A grid that forgets which cells it visited is not a grid | `fix(derive) treat an unvisited cell as free rather than re-sampling it` |
| The search could not say why it gave up | `feat(astar) report why a search ended instead of returning an empty path` |
| Threads are only free if nobody is watching | `perf(jobs) cap the default pool at four workers so a renderer keeps a core` |

## Write for a reader who has only this repository

This is a public repository consumed by projects it knows nothing about.

- Do not name another project's classes, scenes, arenas, issue numbers or docs pages. A reader cannot
  resolve any of them, and a header comment carries the reasoning rather than pointing at it.
- Where a concrete example earns its place, describe it in general terms: "a station-sized grid of half a
  million cells" rather than the name of the thing that happened to have them.
- The same applies to strings a consumer ships: default every configurable value to a neutral one, never to
  one title's real ones.

## The one rule the gates cannot express

**`orbitnav-core` may not name a consumer's concepts.** No stations, modules, arenas, characters, weapons
or issue numbers — in code, in tests, or in comments. The addon is named for navigation rather than for AI
because an addon named for AI attracts decision code, and decision code is the half that must stay in the
consuming game.

Work that is coupled to a consumer's own data model stays with the consumer, however spatial it looks.
Routing a mover through a building's own door graph is the consuming game's, because it needs the building.
