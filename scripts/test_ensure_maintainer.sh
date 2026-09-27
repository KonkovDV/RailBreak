#!/usr/bin/env bash
# ensure_maintainer inserts the tag even when <license> is absent.
set -euo pipefail
ROOT=$(cd "$(dirname "$0")/.." && pwd)
# shellcheck disable=SC1091
source "${ROOT}/scripts/ensure_maintainer.sh"

tmp=$(mktemp -d)
trap 'rm -rf "${tmp}"' EXIT

cat > "${tmp}/with_license.xml" <<'EOF'
<package format="3">
  <description>Tram control messages</description>
  <license>MIT</license>
</package>
EOF
ensure_maintainer "${tmp}/with_license.xml"
grep -q '<maintainer email="organiser@example.invalid">organiser</maintainer>' "${tmp}/with_license.xml"
grep -q '<license>MIT</license>' "${tmp}/with_license.xml"
# description comes first, then the inserted tag
awk '/description/{d=NR} /maintainer/{m=NR} END{exit !(d && m && d < m)}' "${tmp}/with_license.xml"

cat > "${tmp}/no_license.xml" <<'EOF'
<package format="3">
  <name>tram_vehicle_msgs</name>
</package>
EOF
ensure_maintainer "${tmp}/no_license.xml"
grep -q "<maintainer" "${tmp}/no_license.xml"

cp "${tmp}/with_license.xml" "${tmp}/again.xml"
before=$(cksum "${tmp}/again.xml")
ensure_maintainer "${tmp}/again.xml"
after=$(cksum "${tmp}/again.xml")
if [ "${before}" != "${after}" ]; then
  echo "a second call rewrote a package that already had a maintainer" >&2
  exit 1
fi

echo "ensure_maintainer: license, no-license, idempotent"
