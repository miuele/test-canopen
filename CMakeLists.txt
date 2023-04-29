# Copyright (c) 2022 ARM Limited. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.19.0)


set(MBED_PATH ${CMAKE_CURRENT_SOURCE_DIR}/mbed-os CACHE INTERNAL "")
set(MBED_CONFIG_PATH ${CMAKE_CURRENT_BINARY_DIR} CACHE INTERNAL "")
set(MBED_OS_TARGET mbed-core)
set(APP_TARGET test-canopen)

include(${MBED_PATH}/tools/cmake/app.cmake)

project(${APP_TARGET})

add_executable(${APP_TARGET}
    main.cpp
	OD.c
	CO_driver_mbed.cpp
	CANopenNode/CANopen.c
	CANopenNode/301/CO_ODinterface.c
	CANopenNode/301/CO_Emergency.c
	CANopenNode/301/CO_HBconsumer.c
	CANopenNode/301/CO_NMT_Heartbeat.c
	CANopenNode/301/CO_PDO.c
	CANopenNode/301/CO_SDOclient.c
	CANopenNode/301/CO_SDOserver.c
	CANopenNode/301/CO_SYNC.c
	CANopenNode/301/CO_TIME.c
	CANopenNode/303/CO_LEDs.c
	CANopenNode/305/CO_LSSslave.c
)

target_include_directories(${APP_TARGET}
	PRIVATE CANopenNode/
	PRIVATE ./
)

set_property(TARGET ${APP_TARGET} PROPERTY CXX_STANDARD 14)

add_subdirectory(${MBED_PATH} ${CMAKE_CURRENT_BINARY_DIR}/mbed-os-binary)

target_link_libraries(${APP_TARGET} ${MBED_OS_TARGET})
target_link_libraries(${APP_TARGET} mbed-events)

set(CMAKE_CXX_STANDARD 14)
mbed_set_post_build(${APP_TARGET})

option(VERBOSE_BUILD "Have a verbose build process")
if(VERBOSE_BUILD)
    set(CMAKE_VERBOSE_MAKEFILE ON)
endif()

