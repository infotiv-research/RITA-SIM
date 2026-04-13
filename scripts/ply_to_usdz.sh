#!/usr/bin/env bash
# scripts/ply_to_usdz.sh — convert a 3DGS splat .ply into a .usdz via 3dgrut.
#
# Usage:
#   scripts/ply_to_usdz.sh <input.ply> [<output.usdz>] [--with-collision]
#
# Both paths must live under the repo root — they are bind-mounted into the
# rita-3dgrut container at /workspace.
#
# With --with-collision, a convex-hull triangle mesh is generated from the
# splat positions and embedded into the USDZ via
# threedgrut.export.scripts.add_mesh_to_usdz as an invisible collider.
#
# The rita-3dgrut container is left running so repeat conversions reuse the
# warm conda env. Tear it down with:
#   docker compose -f setup/docker-compose.3dgrut.yaml down

set -euo pipefail

if [[ $# -lt 1 ]]; then
    echo "usage: $0 <input.ply> [<output.usdz>] [--with-collision]" >&2
    exit 1
fi

REPO_ROOT="$(git -C "$(dirname "$0")" rev-parse --show-toplevel)"
COMPOSE_FILE="${REPO_ROOT}/setup/docker-compose.3dgrut.yaml"
COMPOSE=(docker compose -f "${COMPOSE_FILE}")

INPUT_PLY="$1"
shift

OUTPUT_USDZ=""
WITH_COLLISION=0
for arg in "$@"; do
    case "${arg}" in
        --with-collision)
            WITH_COLLISION=1
            ;;
        *)
            if [[ -z "${OUTPUT_USDZ}" ]]; then
                OUTPUT_USDZ="${arg}"
            else
                echo "unexpected argument: ${arg}" >&2
                exit 1
            fi
            ;;
    esac
done

if [[ -z "${OUTPUT_USDZ}" ]]; then
    OUTPUT_USDZ="${INPUT_PLY%.ply}.usdz"
fi

if [[ ! -f "${INPUT_PLY}" ]]; then
    echo "input PLY not found: ${INPUT_PLY}" >&2
    exit 1
fi

# Translate a host path into the matching /workspace path inside the
# container. The path must resolve inside REPO_ROOT since that's the only
# thing bind-mounted by docker-compose.3dgrut.yaml.
to_container() {
    local host_path="$1"
    local abs
    abs="$(realpath -m "${host_path}")"
    case "${abs}" in
        "${REPO_ROOT}"/*)
            echo "/workspace/${abs#"${REPO_ROOT}/"}"
            ;;
        "${REPO_ROOT}")
            echo "/workspace"
            ;;
        *)
            echo "path must be inside repo root (${REPO_ROOT}): ${host_path}" >&2
            exit 1
            ;;
    esac
}

IN_C="$(to_container "${INPUT_PLY}")"
OUT_C="$(to_container "${OUTPUT_USDZ}")"

echo ">>> starting rita-3dgrut container"
"${COMPOSE[@]}" up -d

echo ">>> converting ${IN_C} -> ${OUT_C}"
"${COMPOSE[@]}" exec -T 3dgrut bash -lc "
    set -euo pipefail
    cd /opt/3dgrut
    conda run -n 3dgrut --no-capture-output \
        python -m threedgrut.export.scripts.ply_to_usd \
            '${IN_C}' --output_file '${OUT_C}'
"

if [[ "${WITH_COLLISION}" -eq 1 ]]; then
    HULL_PLY="${OUTPUT_USDZ%.usdz}_hull.ply"
    HULL_C="$(to_container "${HULL_PLY}")"

    # add_mesh_to_usdz unlinks the output path before reading the input,
    # so passing the same path for both aliases into a use-after-delete.
    # Write to a sibling temp path and mv it over the original on success.
    TMP_OUT_C="${OUT_C%.usdz}.withmesh.usdz"

    echo ">>> generating convex hull: ${HULL_C}"
    "${COMPOSE[@]}" exec -T 3dgrut bash -lc "
        set -euo pipefail
        cd /workspace
        conda run -n 3dgrut --no-capture-output \
            python scripts/gen_convex_hull_mesh.py '${IN_C}' '${HULL_C}'
    "

    echo ">>> embedding hull as collider in ${OUT_C}"
    "${COMPOSE[@]}" exec -T 3dgrut bash -lc "
        set -euo pipefail
        cd /opt/3dgrut
        conda run -n 3dgrut --no-capture-output \
            python -m threedgrut.export.scripts.add_mesh_to_usdz \
                --input_usdz '${OUT_C}' \
                --output_usdz '${TMP_OUT_C}' \
                --mesh_ply '${HULL_C}' \
                --set_collision \
                --set_invisible
        mv '${TMP_OUT_C}' '${OUT_C}'
    "
fi

echo ">>> done: ${OUTPUT_USDZ}"
