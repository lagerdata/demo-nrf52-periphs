include(../cmake/utils.cmake)

add_executable(app "")
target_sources(app PRIVATE ${CMAKE_CURRENT_LIST_DIR}/main.c)



target_compile_options(app PRIVATE -mcpu=cortex-m4 -mthumb -mabi=aapcs -mfloat-abi=hard -mfpu=fpv4-sp-d16)
target_compile_options(app PRIVATE $<$<COMPILE_LANGUAGE:C>:-Wall -Werror -g -gstrict-dwarf -ffunction-sections -fdata-sections -fno-strict-aliasing -fno-builtin -fshort-enums>)
target_compile_options(app PRIVATE $<$<COMPILE_LANGUAGE:ASM>:-x assembler-with-cpp>)
target_compile_options(app PRIVATE "-ffile-prefix-map=${CMAKE_SOURCE_DIR}=.")

target_compile_definitions(app PRIVATE FLOAT_ABI_HARD NRF52840_XXAA)
target_compile_definitions(app PRIVATE $<$<COMPILE_LANGUAGE:C>:BOARD_PCA10056>)
target_compile_definitions(app PRIVATE $<$<COMPILE_LANGUAGE:ASM>:__HEAP_SIZE=8192 __STACK_SIZE=8192>)


target_link_libraries(app ledctrl)
target_link_libraries(app nrfx_sdk)
target_link_libraries(app shell)


target_link_directories(app PRIVATE "${PROJECT_SOURCE_DIR}/linker")
target_link_options(app PRIVATE -mcpu=cortex-m4 -mthumb -mabi=aapcs -mfloat-abi=hard -mfpu=fpv4-sp-d16 -Wl,--gc-sections -lc -lm -lnosys --specs=nano.specs -Tnrf52.ld)

firmware_size(app)
generate_object(app .bin binary)

if ("${CMAKE_C_COMPILER_ID}" STREQUAL "GNU")
    generate_object(app .hex ihex)
endif()
