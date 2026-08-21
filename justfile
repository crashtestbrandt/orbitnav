set shell := ["bash", "-eu", "-o", "pipefail", "-c"]

# Godot, wrapped so a headless run's noise does not bury the thing you asked for.
godot := justfile_directory() / "tools" / "godot-quiet.sh"

default:
    @just --list

# ==================================================================================================
# Addon sync
#
# The canonical addon lives at the repo root; every Godot project here gets a mirror COPY, because
# the addon is configured through an [orbitnav] project-settings block and two projects cannot share
# one. The copies are gitignored build artifacts -- edit the canonical source, never a copy.
# ==================================================================================================

# Mirror the canonical addon into every project here.
sync-addons:
    tools/sync-addons.sh

# Fail if a synced copy has been edited instead of the canonical source. Local gate.
addon-drift:
    tools/sync-addons.sh --check

# Fail if a synced copy has been COMMITTED, creating a second source of truth. The CI variant.
addon-tracked:
    tools/sync-addons.sh --check-tracked

# ==================================================================================================
# Gates
# ==================================================================================================

# The facade boundary: the backend is reached only through addons/orbitnav/nav.gd.
nav-check:
    tools/nav-check.sh

# A descriptor entry naming a library nothing builds fails at dlopen on that platform and nowhere
# else. This compares the two lists instead.
descriptor-parity:
    tools/check-descriptor-parity.sh

# Headless project load with warnings promoted to errors.
lint:
    GODOT="{{godot}}" tools/lint-gdscript.sh harness

# The harness suites, including the derive parity suite that runs the Rust against an independent
# GDScript transcription of the same specification.
test:
    "{{godot}}" --headless --path harness --script tests/support/run_unit_tests.gd

# The addon end to end in a project that contains only the addon. --quit-after is a frame-count
# backstop only: smoke.gd quits itself as soon as it has a verdict.
harness-smoke:
    "{{godot}}" --headless --path harness --quit-after 600

# Everything, fail-fastest first: the two gates that need no toolchain, then Rust, then Godot.
check: addon-tracked addon-drift nav-check descriptor-parity native-test native-build lint test native-smoke

# ==================================================================================================
# Native
#
# Nothing in a consuming project builds this crate. `native-install` is what a contributor runs once
# after cloning, and what every CI job does before it loads anything.
# ==================================================================================================

# Format, lint and test the Rust workspace. No Godot needed.
native-test:
    cd native && cargo fmt --all --check
    cd native && cargo clippy --workspace --all-targets -- -D warnings
    cd native && cargo test --workspace

# Build this host's library, both descriptor profiles, into the addon.
native-build:
    tools/build-native.sh build "$(tools/build-native.sh host)" addons/orbitnav_native/bin

# Load the built library in a throwaway project containing nothing else.
native-smoke:
    GODOT="{{godot}}" tools/orbitnav-smoke.sh

# What a fresh clone runs once: build the library and mirror the addon into every project.
native-install: native-build
    tools/sync-addons.sh
    echo "orbitnav: built both descriptor profiles and re-synced every project"

# Build, smoke and test the native side without the Godot suites.
native-check: native-test native-build native-smoke

# ==================================================================================================
# Packaging
# ==================================================================================================

# The Asset Library payload: exactly the two addon directories, plus the licences.
assetlib-zip VERSION="dev":
    #!/usr/bin/env bash
    set -euo pipefail
    cp LICENSE LICENSE-MIT LICENSE-APACHE THIRD_PARTY.md addons/orbitnav/
    trap 'rm -f addons/orbitnav/LICENSE addons/orbitnav/LICENSE-MIT addons/orbitnav/LICENSE-APACHE addons/orbitnav/THIRD_PARTY.md' EXIT
    mkdir -p build
    rm -f "build/orbitnav-{{VERSION}}.zip"
    zip -r "build/orbitnav-{{VERSION}}.zip" addons/orbitnav addons/orbitnav_native -x '*/.*'
    echo "orbitnav: build/orbitnav-{{VERSION}}.zip"
