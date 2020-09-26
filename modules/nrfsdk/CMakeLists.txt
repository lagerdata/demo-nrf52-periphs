add_library(nrfx_sdk STATIC "")
set(NRF52_SRC_DIR ${PROJECT_SOURCE_DIR}/nRF5_SDK_17)
target_sources(nrfx_sdk PRIVATE ${NRF52_SRC_DIR}/modules/nrfx/mdk/gcc_startup_nrf52840.S
  ${NRF52_SRC_DIR}/modules/nrfx/mdk/system_nrf52840.c
  ${NRF52_SRC_DIR}/modules/nrfx/drivers/src/nrfx_timer.c
  ${NRF52_SRC_DIR}/modules/nrfx/drivers/src/nrfx_uart.c
  ${NRF52_SRC_DIR}/modules/nrfx/drivers/src/prs/nrfx_prs.c)
target_include_directories(nrfx_sdk PUBLIC ${NRF52_SRC_DIR}/modules/nrfx
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
  ${NRF52_SRC_DIR}/components/toolchain/cmsis/include)

target_compile_options(nrfx_sdk PRIVATE -mcpu=cortex-m4 -mthumb -mabi=aapcs -mfloat-abi=hard -mfpu=fpv4-sp-d16)
target_compile_options(nrfx_sdk PRIVATE $<$<COMPILE_LANGUAGE:C>:-Wall -Werror -g -gstrict-dwarf -ffunction-sections -fdata-sections -fno-strict-aliasing -fno-builtin -fshort-enums>)
target_compile_options(nrfx_sdk PRIVATE $<$<COMPILE_LANGUAGE:ASM>:-x assembler-with-cpp>)
target_compile_options(nrfx_sdk PUBLIC "-ffile-prefix-map=${CMAKE_SOURCE_DIR}=.")


target_compile_definitions(nrfx_sdk PRIVATE FLOAT_ABI_HARD NRF52840_XXAA)
target_compile_definitions(nrfx_sdk PRIVATE $<$<COMPILE_LANGUAGE:ASM>:__HEAP_SIZE=8192 __STACK_SIZE=8192>)
