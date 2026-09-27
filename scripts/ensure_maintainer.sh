# Insert the <maintainer> tag catkin_pkg requires on Humble.
# The organiser package.xml has none. Replacing only <license> does nothing
# when that tag is absent, and colcon then rejects the package.
ensure_maintainer() {
  local pkg="$1"
  if [ ! -f "${pkg}" ]; then
    echo "package.xml not found: ${pkg}" >&2
    exit 1
  fi
  if grep -q "<maintainer" "${pkg}"; then
    return 0
  fi
  local tmp
  tmp=$(mktemp)
  awk '
    {
      lines[NR] = $0
    }
    END {
      done = 0
      for (i = 1; i <= NR; i++) {
        if (!done && lines[i] ~ /<\/description>/) {
          print lines[i]
          print "  <maintainer email=\"organiser@example.invalid\">organiser</maintainer>"
          done = 1
          continue
        }
        if (!done && (lines[i] ~ /<license>/ || lines[i] ~ /<\/package>/)) {
          print "  <maintainer email=\"organiser@example.invalid\">organiser</maintainer>"
          done = 1
        }
        print lines[i]
      }
      if (!done) exit 2
    }
  ' "${pkg}" > "${tmp}" || {
    rm -f "${tmp}"
    echo "could not insert <maintainer> into ${pkg}" >&2
    exit 1
  }
  mv "${tmp}" "${pkg}"
  if ! grep -q "<maintainer" "${pkg}"; then
    echo "could not insert <maintainer> into ${pkg}" >&2
    exit 1
  fi
  echo "inserted <maintainer> into ${pkg}" >&2
}
