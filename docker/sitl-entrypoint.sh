#!/bin/sh
# SITL takes the simulator's address as argv[1] and passes it straight to
# inet_addr() (udplink.c:36) -- dotted IPv4 ONLY, no hostname resolution. So
# "host.docker.internal" cannot be handed over directly; it has to be resolved
# to a numeric address here first.
set -eu

TARGET_IP="${SITL_TARGET_IP:-}"

if [ -z "${TARGET_IP}" ]; then
    TARGET_IP="$(getent hosts host.docker.internal 2>/dev/null | awk '{print $1; exit}' || true)"
fi
if [ -z "${TARGET_IP}" ]; then
    # Fall back to the default gateway, which is the host on a bridge network.
    TARGET_IP="$(ip route 2>/dev/null | awk '/^default/ {print $3; exit}' || true)"
fi
if [ -z "${TARGET_IP}" ]; then
    echo "[entrypoint] could not determine the host address." >&2
    echo "[entrypoint] pass it explicitly: -e SITL_TARGET_IP=<your host IP>" >&2
    exit 1
fi

case "${TARGET_IP}" in
    *[!0-9.]*)
        echo "[entrypoint] '${TARGET_IP}' is not a dotted IPv4 address." >&2
        echo "[entrypoint] SITL calls inet_addr() and will not resolve names." >&2
        exit 1
        ;;
esac

echo "[entrypoint] motor packets -> ${TARGET_IP}:9002 (and :9001 raw)"
echo "[entrypoint] listening for state on :9003/udp and RC on :9004/udp"
echo "[entrypoint] Configurator: TCP :5761"
exec /usr/local/bin/betaflight_SITL "${TARGET_IP}" "$@"
