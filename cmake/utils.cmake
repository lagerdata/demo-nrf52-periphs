# Add custom command to print firmware size in Berkley format
function(firmware_size target)
    add_custom_command(TARGET ${target} POST_BUILD
        COMMAND ${CMAKE_SIZE_UTIL} -B
        "${CMAKE_CURRENT_BINARY_DIR}/${target}${CMAKE_EXECUTABLE_SUFFIX}"
    )
endfunction()

# Add a command to generate firmare in a provided format
function(generate_object target suffix type)
    add_custom_command(TARGET ${target} POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -O ${type}
        "${CMAKE_CURRENT_BINARY_DIR}/${target}${CMAKE_EXECUTABLE_SUFFIX}" "${CMAKE_CURRENT_BINARY_DIR}/${target}${suffix}"
    )
endfunction()


# Update a target LINK_DEPENDS property with a custom linker script.
# That allows to rebuild that target if the linker script gets changed
function(linker_script_target_dependency target path_to_script)
    get_target_property(_cur_link_deps ${target} LINK_DEPENDS)
    string(APPEND _cur_link_deps " ${path_to_script}")
    set_target_properties(${target} PROPERTIES LINK_DEPENDS ${_cur_link_deps})
endfunction()