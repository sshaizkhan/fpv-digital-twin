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
# Must match config/quad.yaml:firmware and third_party/betaflight.
BF_COMMIT="77d01ba3b76a22909d5f09cb0628820141f95eaa"
# Tagged by COMMIT, not by version. The commit is what determines the image
# contents, so bumping it has to be a cache miss -- with a fixed :4.5.1 tag the
# `docker image inspect` below would still hit and silently run the old binary.
IMAGE="fdt-betaflight-sitl:${BF_COMMIT:0:12}"
CONTAINER="fdt-sitl"
STATE_DIR="${REPO_ROOT}/.sitl-state"

rebuild=0
shell=0
for arg in "$@"; do
    case "$arg" in
        --rebuild) rebuild=1 ;;
        --shell)   shell=1 ;;
        -h|--help) sed -n '2,10p' "$0"; exit 0 ;;
        *) echo "unknown argument: $arg" >&2; exit 2 ;;
    esac
done

if ! docker info >/dev/null 2>&1; then
    echo "Docker is not running. Start Docker Desktop and try again." >&2
    exit 1
fi

# Keep the image honest about which firmware it contains. Ask git rather than
# testing for a .git directory: after a recursive clone a submodule's .git is a
# gitlink FILE, so `[[ -d ]]` would skip this check silently on every machine
# but the one the submodule was first added on.
#
# `git -C <dir>` walks UP to find the enclosing repo, so on a clone without
# --recursive -- where third_party/betaflight exists but is EMPTY -- a bare
# `rev-parse HEAD` succeeds and hands back the SUPERPROJECT's HEAD. That reports
# a sha mismatch and points the user at BF_COMMIT when the real fix is
# `submodule update --init`. So require the repo git found to actually BE the
# submodule (-ef compares device+inode, which normalizes symlinks for free)
# before trusting its HEAD.
BF_DIR="${REPO_ROOT}/third_party/betaflight"
bf_toplevel="$(git -C "${BF_DIR}" rev-parse --show-toplevel 2>/dev/null || true)"
if [[ -n "${bf_toplevel}" ]] && [[ "${bf_toplevel}" -ef "${BF_DIR}" ]]; then
    actual="$(git -C "${BF_DIR}" rev-parse HEAD)"
    if [[ "${actual}" != "${BF_COMMIT}" ]]; then
        echo "submodule is at ${actual}" >&2
        echo "but this script builds ${BF_COMMIT} -- update one of them" >&2
        exit 1
    fi
else
    echo "third_party/betaflight is not checked out -- cannot confirm the firmware." >&2
    echo "run: git submodule update --init third_party/betaflight" >&2
    exit 1
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
# from a script, a CI job, or an agent shell. Every expansion below uses the
# ${x[@]+"${x[@]}"} form because bash 3.2 -- still /bin/bash on macOS -- treats
# expanding an EMPTY array under `set -u` as an unbound variable, which would
# abort in exactly the non-tty case this exists for.
TTY_FLAGS=()
if [[ -t 0 && -t 1 ]]; then
    TTY_FLAGS=(-it)
fi

# The entrypoint has no default-gateway fallback (deliberately -- see
# docker/sitl-entrypoint.sh), so SITL_TARGET_IP is the ONLY way to supply the
# host address when host.docker.internal does not resolve. `docker run` does not
# inherit the host environment, so without this the variable is silently dropped
# and the user is pushed into hand-rolling a `docker run` -- losing the loopback
# publishing and the /data mount that keeps the pasted config.
TARGET_ENV=()
if [[ -n "${SITL_TARGET_IP:-}" ]]; then
    TARGET_ENV=(-e "SITL_TARGET_IP=${SITL_TARGET_IP}")
fi

docker rm -f "${CONTAINER}" >/dev/null 2>&1 || true

if [[ "${shell}" == "1" ]]; then
    exec docker run --rm ${TTY_FLAGS[@]+"${TTY_FLAGS[@]}"} \
        ${TARGET_ENV[@]+"${TARGET_ENV[@]}"} --name "${CONTAINER}" \
        -v "${STATE_DIR}:/data" --entrypoint /bin/sh "${IMAGE}"
fi

cat <<'BANNER'
-------------------------------------------------------------------------
Betaflight SITL 4.5.1

  Configurator : TCP 127.0.0.1:5761   (Betaflight Configurator, "Manual"
                 connection, select TCP and that address)
  state  in    : UDP 9003   <- physics   (SITL binds; published to loopback)
  RC     in    : UDP 9004   <- physics   (SITL binds; published to loopback)
  motors out   : UDP 9002   -> physics   (SITL sends)
  motors raw   : UDP 9001   -> physics   (SITL sends)

9001/9002 are deliberately NOT published: the PHYSICS process binds them on
the host, and it must bind 0.0.0.0 -- these packets arrive from the Docker
VM's address, not from loopback.

SITL has NO receiver until it gets its first RC packet on 9004, so it will
not arm until the physics side is sending. Config is saved to .sitl-state/
Ctrl-C to stop.
-------------------------------------------------------------------------
BANNER

# Only 9003/9004 are published: SITL BINDS those (sitl.c:316,319) and we send to
# them. 9001/9002 are SITL's OUTBOUND direction (sitl.c:310,313, isServer=false);
# container-outbound UDP is NAT'd without any -p, and publishing them would make
# Docker own the host port so the physics receiver could not bind it.
#
# All three PUBLISHED ports are bound to 127.0.0.1: MSP on 5761 is
# unauthenticated full control of the FC, and 9003/9004 accept state and RC
# injection. None of those three needs to be reachable off-box. This does NOT
# extend to 9001/9002 -- see the banner above.
exec docker run --rm ${TTY_FLAGS[@]+"${TTY_FLAGS[@]}"} \
    ${TARGET_ENV[@]+"${TARGET_ENV[@]}"} --name "${CONTAINER}" \
    -p 127.0.0.1:5761:5761/tcp \
    -p 127.0.0.1:9003:9003/udp -p 127.0.0.1:9004:9004/udp \
    -v "${STATE_DIR}:/data" \
    "${IMAGE}"
