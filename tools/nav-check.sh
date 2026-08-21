#!/usr/bin/env bash
# OrbitNav facade boundary gate.
#
# The navigation backend (the OrbitNav native Rust GDExtension) must be reachable ONLY through
# addons/orbitnav/nav.gd. That seam is the whole reason the addon is extractable: it is what lets a
# consuming project depend on `Nav` without ever naming the backend class, and what makes replacing
# or removing the backend a one-file change. This fails if any file OTHER than nav.gd references it.
#
# It does NOT match the bare word "OrbitNav" in prose -- the addon is named that -- only real class
# and path usage. `Nav`, `NavVolumeHandle` and `NavPathJob` are the facade's own surface and are
# meant to be named everywhere.
#
# THE HARNESS IS THE POINT. It scans harness/ as well as the addon, so if the harness reaches past
# the facade the gate fails. A test that needs a backend symbol is a test proving the facade has a
# hole -- widen the facade, do not exempt the caller.
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"

# The backend class, the two ways to construct it, and the binary addon path. No \b word boundary --
# that is a GNU-only extension to ERE and behaves inconsistently on BSD/macOS grep -- so the tokens
# are spelled out in the forms that actually appear.
PATTERN='OrbitNav\.new|ClassDB\.instantiate\(&?"OrbitNav"|ClassDB\.class_exists\(&?"OrbitNav"|res://addons/orbitnav_native'

# The trees to police, and the one allowed file. Note what is NOT scanned: the SYNCED copy of the
# addon under harness/addons/. It is a byte-identical build artifact of the canonical source
# (tools/sync-addons.sh, gitignored), so scanning it would re-report every hit in nav.gd under a path
# the `^addons/orbitnav/nav.gd:` exemption does not cover -- the gate would fail on a correct tree
# the moment anyone ran `just sync-addons`. `just addon-drift` is what proves the copy matches; this
# gate only ever needs to read the canonical one.
FILES="$(find addons/orbitnav tools harness \
	-type d \( -name addons -o -name .godot -o -name target \) -prune -o \
	-type f \( -name '*.gd' -o -name '*.tscn' \) -print 2>/dev/null | sort || true)"

if [ -z "$FILES" ]; then
	printf 'nav-check FAILED: found no .gd/.tscn files to scan (wrong working directory?)\n' >&2
	exit 1
fi

hits="$(printf '%s\n' "$FILES" | tr '\n' '\0' | xargs -0 grep -nE "$PATTERN" 2>/dev/null \
	| grep -v '^addons/orbitnav/nav.gd:' || true)"

if [ -n "$hits" ]; then
	printf 'nav-check FAILED: navigation-backend symbols referenced outside addons/orbitnav/nav.gd:\n\n'
	printf '%s\n' "$hits"
	printf '\nRoute the access through addons/orbitnav/nav.gd (and its NavVolumeHandle / NavPathJob).\n'
	printf 'If the facade genuinely cannot express what you need, widen the facade -- that is a\n'
	printf 'library gap, not a reason to exempt a caller.\n'
	exit 1
fi

printf 'nav-check passed: the navigation backend is reached only through addons/orbitnav/nav.gd.\n'
