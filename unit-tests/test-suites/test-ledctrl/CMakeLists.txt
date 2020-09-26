include(../../../cmake/utils.cmake)

add_executable(test-ledctrl "")
target_sources(test-ledctrl PRIVATE ${CMAKE_CURRENT_LIST_DIR}/main_ledctrl.c
  ${CMAKE_CURRENT_LIST_DIR}/test_ledctrl.c)
target_include_directories(test-ledctrl PRIVATE ${CMAKE_CURRENT_LIST_DIR})

set(UNITY_SRC_DIR ${CMAKE_CURRENT_LIST_DIR}/../../Unity/src)
target_sources(test-ledctrl PRIVATE ${UNITY_SRC_DIR}/unity.c)
target_include_directories(test-ledctrl PRIVATE ${UNITY_SRC_DIR})

set(UART_SRC_DIR ${CMAKE_CURRENT_LIST_DIR}/../../)
target_sources(test-ledctrl PRIVATE ${UART_SRC_DIR}/unity_config.c)
target_include_directories(test-ledctrl PRIVATE ${UART_SRC_DIR})

set(NRF52_SRC_DIR ${CMAKE_CURRENT_LIST_DIR}/../../../nRF5_SDK_17)
target_sources(test-ledctrl PRIVATE ${NRF52_SRC_DIR}/modules/nrfx/mdk/gcc_startup_nrf52840.S
  ${NRF52_SRC_DIR}/modules/nrfx/mdk/system_nrf52840.c)

target_include_directories(test-ledctrl PUBLIC ${NRF52_SRC_DIR}/modules/nrfx
  ${NRF52_SRC_DIR}/modules/nrfx/soc
  ${NRF52_SRC_DIR}/modules/nrfx/mdk
  ${NRF52_SRC_DIR}/modules/nrfx/hal
  ${NRF52_SRC_DIR}/modules/nrfx/drivers
  ${NRF52_SRC_DIR}/modules/nrfx/drivers/include
  ${NRF52_SRC_DIR}/modules/nrfx/drivers/src
  ${NRF52_SRC_DIR}/modules/nrfx/drivers/src/prs
  ${NRF52_SRC_DIR}/modules/nrfx/templates
  ${NRF52_SRC_DIR}/modules/nrfx/templates/nRF52840
  ${NRF52_SRC_DIR}/modules/nrfx/helpers
  ${NRF52_SRC_DIR}/components/libraries/delay)

target_include_directories(test-ledctrl PUBLIC ${NRF52_SRC_DIR}/components/toolchain/cmsis/include)

target_link_libraries(test-ledctrl ledctrl)
target_link_libraries(test-ledctrl ticks)

target_compile_options(test-ledctrl PRIVATE -mcpu=cortex-m4 -mthumb -mabi=aapcs -mfloat-abi=hard -mfpu=fpv4-sp-d16)
target_compile_options(test-ledctrl PRIVATE $<$<COMPILE_LANGUAGE:C>:-Wall -Werror -g3 -O3 -gstrict-dwarf -ffunction-sections -fdata-sections -fno-strict-aliasing -fno-builtin -fshort-enums>)
target_compile_options(test-ledctrl PRIVATE $<$<COMPILE_LANGUAGE:ASM>:-x assembler-with-cpp>)
target_compile_options(test-ledctrl PRIVATE "-ffile-prefix-map=${CMAKE_SOURCE_DIR}=.")

target_compile_definitions(test-ledctrl PRIVATE FLOAT_ABI_HARD NRF52840_XXAA)
target_compile_definitions(test-ledctrl PUBLIC $<$<COMPILE_LANGUAGE:C>:UNITY_INCLUDE_CONFIG_H BOARD_PCA10056>)
target_compile_definitions(test-ledctrl PRIVATE $<$<COMPILE_LANGUAGE:ASM>:__HEAP_SIZE=8192 __STACK_SIZE=8192 BOARD_PCA10056>)

target_link_directories(test-ledctrl PRIVATE ${CMAKE_CURRENT_LIST_DIR}/../../../linker)
target_link_options(test-ledctrl PRIVATE -mcpu=cortex-m4 -mthumb -mabi=aapcs -mfloat-abi=hard -mfpu=fpv4-sp-d16 -Wl,--gc-sections -lc -lm -lnosys --specs=nano.specs -Tnrf52.ld)


firmware_size(test-ledctrl)
generate_object(test-ledctrl .bin binary)

if ("${CMAKE_C_COMPILER_ID}" STREQUAL "GNU")
    generate_object(test-ledctrl .hex ihex)
endif()
