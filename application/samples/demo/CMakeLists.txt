#===============================================================================
# @brief    CMakeLists for SLE UART Bridge Demo
# Copyright (c) 2024. All rights reserved.
#===============================================================================

# Use PARENT_SCOPE pattern like sle_uart sample does
# This adds sources to the parent samples component

set(SOURCES_LIST
    ${CMAKE_CURRENT_SOURCE_DIR}/demo.c
    ${CMAKE_CURRENT_SOURCE_DIR}/demo_uart.c
    ${CMAKE_CURRENT_SOURCE_DIR}/demo_sle.c
    ${CMAKE_CURRENT_SOURCE_DIR}/demo_transport.c
)

set(HEADER_LIST
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# Propagate to parent scope
set(SOURCES "${SOURCES}" ${SOURCES_LIST} PARENT_SCOPE)
set(PUBLIC_HEADER "${PUBLIC_HEADER}" ${HEADER_LIST} PARENT_SCOPE)
