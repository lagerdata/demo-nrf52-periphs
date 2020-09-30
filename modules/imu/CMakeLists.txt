
add_library (imu STATIC "")
target_sources(imu PRIVATE ${CMAKE_CURRENT_LIST_DIR}/imu.c)
target_include_directories(imu PUBLIC ${CMAKE_CURRENT_LIST_DIR}/)

target_link_libraries(imu nrfx_sdk)

target_compile_options(imu PRIVATE -mcpu=cortex-m4 -mthumb -mabi=aapcs -mfloat-abi=hard -mfpu=fpv4-sp-d16)
target_compile_options(imu PRIVATE $<$<COMPILE_LANGUAGE:C>:-Wall -Werror -g -gstrict-dwarf -ffunction-sections -fdata-sections -fno-strict-aliasing -fno-builtin -fshort-enums>)
target_compile_options(imu PRIVATE $<$<COMPILE_LANGUAGE:ASM>:-x assembler-with-cpp>)
target_compile_options(imu PUBLIC "-ffile-prefix-map=${CMAKE_SOURCE_DIR}=.")

target_compile_definitions(imu PRIVATE FLOAT_ABI_HARD NRF52840_XXAA)
target_compile_definitions(imu PRIVATE $<$<COMPILE_LANGUAGE:ASM>:__HEAP_SIZE=8192 __STACK_SIZE=8192>)
