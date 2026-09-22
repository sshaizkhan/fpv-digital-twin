#!/usr/bin/env bash
# Build (if needed) and run Betaflight SITL in Docker.
#
# Native macOS builds of the 4.5.1 SITL target cannot link -- Apple's ld64 has
# no support for the GNU linker script the parameter-group registry needs.
# See docs/sitl_interface.md section 6 for the full evidence.
#
#   ./tools/run_sitl.sh              run it (builds the image on first use)
#   ./tools/run_sitl.sh --rebuild    force an image rebuild
#   ./tools/run_sitl.sh --shell      drop into the container instead
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IMAGE="fdt-betaflight-sitl:4.5.1"
CONTAINER="fdt-sitl"
# Must match config/quad.yaml:firmware and third_party/betaflight.
BF_COMMIT="77d01ba3b76a22909d5f09cb0628820141f95eaa"
STATE_DIR="${REPO_ROOT}/.sitl-state"

rebuild=0
shell=0
for arg in "$@"; do
    case "$arg" in
        --rebuild) rebuild=1 ;;
        --shell)   shell=1 ;;
        -h|--help) sed -n '2,12p' "$0"; exit 0 ;;
        *) echo "unknown argument: $arg" >&2; exit 2 ;;
    esac
done

if ! docker info >/dev/null 2>&1; then
    echo "Docker is not running. Start Docker Desktop and try again." >&2
    exit 1
fi

# Keep the image honest about which firmware it contains.
if [[ -d "${REPO_ROOT}/third_party/betaflight/.git" ]]; then
    actual="$(git -C "${REPO_ROOT}/third_party/betaflight" rev-parse HEAD)"
    if [[ "${actual}" != "${BF_COMMIT}" ]]; then
        echo "submodule is at ${actual}" >&2
        echo "but this script builds ${BF_COMMIT} -- update one of them" >&2
        exit 1
    fi
fi

if [[ "${rebuild}" == "1" ]] || ! docker image inspect "${IMAGE}" >/dev/null 2>&1; then
    echo "building ${IMAGE} (first build compiles Betaflight, expect a few minutes)"
    docker build \
        --build-arg "BF_COMMIT=${BF_COMMIT}" \
        -f "${REPO_ROOT}/docker/Dockerfile.sitl" \
        -t "${IMAGE}" \
        "${REPO_ROOT}/docker"
fi

mkdir -p "${STATE_DIR}"

# -it only works with a real terminal; without this the script cannot be used
# from a script, a CI job, or an agent shell.
TTY_FLAGS=()
if [[ -t 0 && -t 1 ]]; then
    TTY_FLAGS=(-it)
fi

docker rm -f "${CONTAINER}" >/dev/null 2>&1 || true

if [[ "${shell}" == "1" ]]; then
    exec docker run --rm "${TTY_FLAGS[@]}" --name "${CONTAINER}" \
        -v "${STATE_DIR}:/data" --entrypoint /bin/sh "${IMAGE}"
fi

cat <<'BANNER'
-------------------------------------------------------------------------
Betaflight SITL 4.5.1

  Configurator : TCP 127.0.0.1:5761   (Betaflight Configurator, "Manual"
                 connection, select TCP and that address)
  state  in    : UDP 9003   <- physics
  RC     in    : UDP 9004   <- physics
  motors out   : UDP 9002   -> physics

SITL has NO receiver until it gets its first RC packet on 9004, so it will
not arm until the physics side is sending. Config is saved to .sitl-state/
Ctrl-C to stop.
-------------------------------------------------------------------------
BANNER

exec docker run --rm "${TTY_FLAGS[@]}" --name "${CONTAINER}" \
    -p 5761:5761/tcp \
    -p 9001:9001/udp -p 9002:9002/udp -p 9003:9003/udp -p 9004:9004/udp \
    -v "${STATE_DIR}:/data" \
    "${IMAGE}"
