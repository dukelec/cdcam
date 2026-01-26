# The following lines of boilerplate have to be in your project's CMakeLists
# in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

execute_process(
    COMMAND git describe --dirty --always --tags
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    OUTPUT_VARIABLE GIT_VERSION_FULL
    OUTPUT_STRIP_TRAILING_WHITESPACE
)

execute_process(
    COMMAND python3 ${CMAKE_SOURCE_DIR}/tools/git_version.py
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    OUTPUT_VARIABLE GIT_VERSION
    OUTPUT_STRIP_TRAILING_WHITESPACE
)

message(STATUS "GIT_VERSION_FULL=${GIT_VERSION_FULL}, GIT_VERSION=${GIT_VERSION}")
add_compile_definitions(SW_VER_FULL=\"${GIT_VERSION_FULL}\")
add_compile_definitions(SW_VER=\"${GIT_VERSION}\")


include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(cdcam)


set(ELF_FILE "${CMAKE_BINARY_DIR}/cdcam.elf")
set(BIN_FILE "${CMAKE_BINARY_DIR}/cdcam.bin")
set(HEX_FILE "${CMAKE_BINARY_DIR}/cdcam.hex")
set(PY_SCRIPT "${CMAKE_SOURCE_DIR}/tools/app_bin2hex.py")

add_custom_command(
    OUTPUT ${HEX_FILE}
    COMMAND ${CMAKE_COMMAND} -E echo "Converting BIN to IAP HEX..." 
    COMMAND python3 ${PY_SCRIPT} ${BIN_FILE}
    DEPENDS ${ELF_FILE}
    COMMENT "Auto converting ELF to IAP HEX"
    VERBATIM
)

add_custom_target(generate_iap_hex ALL
    DEPENDS ${HEX_FILE}
)

add_dependencies(generate_iap_hex app)
