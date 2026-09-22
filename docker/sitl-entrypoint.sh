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
    # NO default-gateway fallback on purpose. On Docker Desktop (the only
    # platform this project targets) the default route's gateway is the bridge
    # gateway INSIDE the Linux VM, not the macOS host running the physics
    # process. It is a valid, routable, dotted-IPv4 address, so it passes every
    # check below, prints a confident banner, and sendto() succeeds -- while the
    # motor packets go nowhere. The symptom would be "physics never receives
    # motors", which reads as a bridge bug. Failing here is far cheaper.
    echo "[entrypoint] host.docker.internal did not resolve, so the host address" >&2
    echo "[entrypoint] is unknown. Pass it explicitly:" >&2
    echo "[entrypoint]     docker run -e SITL_TARGET_IP=<your host IP> ..." >&2
    echo "[entrypoint] (On Linux, the docker0 gateway is usually the right one.)" >&2
    exit 1
fi

# SITL calls inet_addr() (udplink.c:36) and will not resolve names, so this has
# to be four numeric octets. A digit-count regex like ^([0-9]{1,3}\.){3}...$ is
# NOT enough: it passes "192.168.1.400", and glibc's inet_addr() returns
# INADDR_NONE for that, which SITL stores as-is -- so every motor packet is
# sendto()'d to 255.255.255.255 while the banner below prints the typo'd address
# with confidence. Range-check the octets instead. Leading zeros are rejected
# too, because inet_addr() reads them as OCTAL ("010.1.1.1" is host 8), which is
# a wrong-but-valid address: the same silent failure by another route.
if ! echo "${TARGET_IP}" | awk '
    BEGIN { FS = "."; ok = 0 }
    {
        if (NR > 1) { ok = 0; exit }
        ok = 1
        if (NF != 4) { ok = 0; exit }
        for (i = 1; i <= 4; i++) {
            if ($i !~ /^[0-9]+$/ || $i + 0 > 255) { ok = 0; exit }
            if ($i != "0" && $i ~ /^0/) { ok = 0; exit }
        }
    }
    END { exit ok ? 0 : 1 }
'; then
    echo "[entrypoint] '${TARGET_IP}' is not a dotted IPv4 address." >&2
    echo "[entrypoint] SITL calls inet_addr() and will not resolve names;" >&2
    echo "[entrypoint] it needs four 0-255 octets with no leading zeros." >&2
    exit 1
fi

echo "[entrypoint] motor packets -> ${TARGET_IP}:9002 (and :9001 raw)"
echo "[entrypoint] listening for state on :9003/udp and RC on :9004/udp"
echo "[entrypoint] Configurator: TCP :5761"
exec /usr/local/bin/betaflight_SITL "${TARGET_IP}" "$@"
