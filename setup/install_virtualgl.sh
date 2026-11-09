#!/usr/bin/env bash
set -euo pipefail

root_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
install_dir="$root_dir/outputs/virtualgl"
if [[ -x "$install_dir/opt/VirtualGL/bin/vglrun" ]]; then
    exit 0
fi
if [[ "$(uname -sm)" != "Linux x86_64" ]]; then
    echo "This installer supports x86-64 Linux." >&2
    exit 2
fi

mkdir -p "$root_dir/outputs"
stage="$(mktemp -d "$root_dir/outputs/.virtualgl.XXXXXX")"
trap 'rm -rf -- "$stage"' EXIT
package="${1:-$stage/virtualgl.deb}"
if [[ $# -eq 0 ]]; then
    curl -fsSL --retry 2 --max-time 30 \
        https://github.com/VirtualGL/virtualgl/releases/download/3.1.5/virtualgl_3.1.5_amd64.deb \
        -o "$package"
fi
printf '%s  %s\n' df3f7788ce41b182a47c0d298e5cd6d2d63579522cb41825970b7726e825485e "$package" | sha256sum -c -
dpkg-deb -x "$package" "$stage/payload"
mv -T "$stage/payload" "$install_dir"
