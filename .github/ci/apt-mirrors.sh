#!/bin/sh
# Make apt in the CI container fall back to other Ubuntu mirrors.
#
# Run by industrial_ci as BEFORE_INIT, inside the container, before its
# first `apt-get update`. When archive.ubuntu.com / security.ubuntu.com
# cannot be reached, apt's mirror method (apt-transport-mirror(1),
# https://manpages.debian.org/testing/apt/apt-transport-mirror.1.en.html)
# tries the next mirror in priority order instead of failing the job.
# Every mirror carries the same signed InRelease files, so nothing is
# trusted that was not before.
#
# The runner is in Japan, so the Japanese mirrors come first: apt tries
# the first mirror for every file and only moves on after its timeout, so
# an unreachable first mirror costs that timeout on each file.
set -eu

if [ "$(id -u)" -ne 0 ] && command -v sudo >/dev/null; then
    SUDO=sudo
else
    SUDO=
fi

$SUDO tee /etc/apt/mirrors.txt >/dev/null <<'MIRRORS'
http://ftp.riken.jp/Linux/ubuntu/	priority:1
http://ftp.jaist.ac.jp/pub/Linux/ubuntu/	priority:2
http://archive.ubuntu.com/ubuntu/	priority:3
http://mirrors.kernel.org/ubuntu/	priority:4
MIRRORS

# Fail over within seconds rather than the default timeouts.
$SUDO tee /etc/apt/apt.conf.d/99-ci-mirrors >/dev/null <<'CONF'
Acquire::http::Timeout "10";
Acquire::Retries "3";
CONF

$SUDO sed -i \
    -e 's;http://archive\.ubuntu\.com/ubuntu/\?;mirror+file:/etc/apt/mirrors.txt;' \
    -e 's;http://security\.ubuntu\.com/ubuntu/\?;mirror+file:/etc/apt/mirrors.txt;' \
    /etc/apt/sources.list
grep -v '^#' /etc/apt/sources.list | grep -v '^$'
