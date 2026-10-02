# Writes railbreak_driver_cmd.hpp from the typesupport library that will be
# linked. A leftover driver_controller_command.hpp in install/ is not enough:
# the check-code package does not ship that message, and the node must not
# reference a symbol the library no longer exports.
if(NOT DEFINED TS OR NOT EXISTS "${TS}")
  file(WRITE "${OUT}" "/* typesupport library not found; __has_include decides */\n")
  return()
endif()
file(STRINGS "${TS}" _hit REGEX "DriverControllerCommand" LIMIT_COUNT 1)
if(_hit)
  set(_v 1)
else()
  set(_v 0)
endif()
file(WRITE "${OUT}" "#define RAILBREAK_HAS_DRIVER_CMD ${_v}\n")
message(STATUS "railbreak DriverControllerCommand in linked typesupport: ${_v}")
