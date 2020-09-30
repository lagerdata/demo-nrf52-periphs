add_library(nrfx_sdk STATIC "")
set(NRF52_SRC_DIR ${PROJECT_SOURCE_DIR}/nRF5_SDK_17)
target_sources(nrfx_sdk PRIVATE ${NRF52_SRC_DIR}/modules/nrfx/mdk/gcc_startup_nrf52840.S
  ${NRF52_SRC_DIR}/modules/nrfx/mdk/system_nrf52840.c
  ${NRF52_SRC_DIR}/modules/nrfx/drivers/src/nrfx_timer.c
  ${NRF52_SRC_DIR}/modules/nrfx/drivers/src/nrfx_uart.c
  ${NRF52_SRC_DIR}/modules/nrfx/drivers/src/prs/nrfx_prs.c
  ${NRF52_SRC_DIR}/components/libraries/util/app_error.c
  ${NRF52_SRC_DIR}/components/libraries/util/app_error_handler_gcc.c
  ${NRF52_SRC_DIR}/components/libraries/util/app_error_weak.c
  ${NRF52_SRC_DIR}/components/libraries/util/app_util_platform.c
  ${NRF52_SRC_DIR}/components/libraries/util/nrf_assert.c
  ${NRF52_SRC_DIR}/components/libraries/log/src/nrf_log_frontend.c
  ${NRF52_SRC_DIR}/components/libraries/log/src/nrf_log_str_formatter.c
  ${NRF52_SRC_DIR}/components/libraries/atomic/nrf_atomic.c
  ${NRF52_SRC_DIR}/components/libraries/balloc/nrf_balloc.c
  ${NRF52_SRC_DIR}/external/fprintf/nrf_fprintf.c
  ${NRF52_SRC_DIR}/external/fprintf/nrf_fprintf_format.c
  ${NRF52_SRC_DIR}/components/libraries/memobj/nrf_memobj.c
  ${NRF52_SRC_DIR}/components/libraries/ringbuf/nrf_ringbuf.c
  ${NRF52_SRC_DIR}/components/libraries/strerror/nrf_strerror.c
  ${NRF52_SRC_DIR}/modules/nrfx/soc/nrfx_atomic.c)
target_include_directories(nrfx_sdk PUBLIC ${NRF52_SRC_DIR}/modules/nrfx
  ${NRF52_SRC_DIR}/modules/nrfx/soc
  ${NRF52_SRC_DIR}/modules/nrfx/mdk
  ${NRF52_SRC_DIR}/modules/nrfx/hal
  ${NRF52_SRC_DIR}/modules/nrfx/drivers
  ${NRF52_SRC_DIR}/modules/nrfx/drivers/include
  ${NRF52_SRC_DIR}/modules/nrfx/drivers/src
  ${NRF52_SRC_DIR}/modules/nrfx/drivers/src/prs
  ${NRF52_SRC_DIR}/modules/nrfx/helpers
  ${NRF52_SRC_DIR}/integration/nrfx
  ${NRF52_SRC_DIR}/integration/nrfx/legacy
  ${NRF52_SRC_DIR}/components/
  ${NRF52_SRC_DIR}/components/toolchain/cmsis/include
  ${NRF52_SRC_DIR}/components/libraries/util
  ${NRF52_SRC_DIR}/components/drivers_nrf/nrf_soc_nosd
  ${NRF52_SRC_DIR}/components/libraries/bsp
  ${NRF52_SRC_DIR}/components/libraries/log
  ${NRF52_SRC_DIR}/components/libraries/log/src
  ${NRF52_SRC_DIR}/components/libraries/experimental_section_vars
  ${NRF52_SRC_DIR}/components/libraries/strerror
  ${NRF52_SRC_DIR}/components/libraries/balloc
  ${NRF52_SRC_DIR}/components/libraries/ringbuf
  ${NRF52_SRC_DIR}/components/libraries/atomic
  ${NRF52_SRC_DIR}/components/libraries/delay
  ${NRF52_SRC_DIR}/components/boards
  ${NRF52_SRC_DIR}/components/libraries/memobj
  ${NRF52_SRC_DIR}/components/libraries/log/src
  ${NRF52_SRC_DIR}/external/fprintf
  ${PROJECT_SOURCE_DIR}/modules/nrfsdk)

target_compile_options(nrfx_sdk PRIVATE -mcpu=cortex-m4 -mthumb -mabi=aapcs -mfloat-abi=hard -mfpu=fpv4-sp-d16)
target_compile_options(nrfx_sdk PRIVATE $<$<COMPILE_LANGUAGE:C>:-Wall -Werror -g3 -O3 -gstrict-dwarf -ffunction-sections -fdata-sections -fno-strict-aliasing -fno-builtin -fshort-enums>)
target_compile_options(nrfx_sdk PRIVATE $<$<COMPILE_LANGUAGE:ASM>:-x assembler-with-cpp>)
target_compile_options(nrfx_sdk PUBLIC "-ffile-prefix-map=${CMAKE_SOURCE_DIR}=.")


target_compile_definitions(nrfx_sdk PUBLIC FLOAT_ABI_HARD NRF52840_XXAA BSP_DEFINES_ONLY BOARD_PCA10056)
target_compile_definitions(nrfx_sdk PUBLIC $<$<COMPILE_LANGUAGE:ASM>:__HEAP_SIZE=8192 __STACK_SIZE=8192>)
