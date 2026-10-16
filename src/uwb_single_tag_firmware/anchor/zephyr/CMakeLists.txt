cmake_minimum_required(VERSION 3.13.1)
include($ENV{ZEPHYR_BASE}/cmake/app/boilerplate.cmake NO_POLICY_SCOPE)
project(nrf52_dw3000_anchor_package)

get_filename_component(PIO_ENV_NAME ${CMAKE_BINARY_DIR} NAME)
if(PIO_ENV_NAME MATCHES "^anchor_a([1-4])_single$")
  set(ANCHOR_PROFILE_VALUE ${CMAKE_MATCH_1})
else()
  message(FATAL_ERROR "Select anchor_a1_single through anchor_a4_single")
endif()

set(NRF52DK_REF_ROOT ${CMAKE_CURRENT_LIST_DIR}/../vendor/nrf52_dk_dwm3000_reference)
target_sources(app PRIVATE
  ../src/anchor_nrf52dk_ref_1to1.c
  ../src/nrf52dk_ref_vendor/wrap_deca_device.c
  ../src/nrf52dk_ref_vendor/wrap_port.c
  ../src/nrf52dk_ref_vendor/wrap_deca_sleep.c
  ../src/nrf52dk_ref_vendor/wrap_deca_spi.c
)
target_include_directories(app PRIVATE
  ${NRF52DK_REF_ROOT}
  ${NRF52DK_REF_ROOT}/decadriver
  ${NRF52DK_REF_ROOT}/platform
  ${NRF52DK_REF_ROOT}/compiler
)
target_compile_definitions(app PRIVATE
  ANCHOR_PROFILE=${ANCHOR_PROFILE_VALUE}
  UWB_FAST30_PROFILE=1
  CONFIG_DW3000=1
  CONFIG_DW3000_CHIP_DW3000=1
  CONFIG_DW3000_SPI_MAX_MHZ=8
  CONFIG_DW3000_LOG_LEVEL=3
)
