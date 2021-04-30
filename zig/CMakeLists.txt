add_custom_command(
       OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/lib/libzigshell.a
       DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/build.zig ${CMAKE_CURRENT_SOURCE_DIR}/src/shell.zig
       COMMAND zig build --build-file ${CMAKE_CURRENT_SOURCE_DIR}/build.zig --prefix ${CMAKE_CURRENT_BINARY_DIR}
       VERBATIM)

add_custom_target(zigshell
	DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/lib/libzigshell.a)
