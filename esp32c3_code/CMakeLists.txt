# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(tfg_mk3)

### NEW FOR ADA/SPARK INTEGRATION ###

# Build the Ada code using Alire
execute_process(COMMAND alr -n build
                WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}/ada_code/")

# Add the Ada code static library
add_library(ada_code STATIC IMPORTED GLOBAL)
set_property(TARGET ada_code PROPERTY IMPORTED_LOCATION "${CMAKE_SOURCE_DIR}/ada_code/lib/libAda_Code.a")
target_link_libraries(${CMAKE_PROJECT_NAME}.elf PUBLIC ada_code)

# Get path of Ada run-time library (libgnat.a)
execute_process(COMMAND bash -c "alr exec -- riscv64-elf-gnatls --RTS=zfp-rv32imac -v 2>&1 | grep adalib"
                WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}/ada_code/"
                RESULT_VARIABLE gnatls_result
                OUTPUT_VARIABLE gnatls_output)

string(STRIP "${gnatls_output}" ada_runtime_dir)

# Add the Ada run-time static library
message(STATUS "Link Ada run-time ${ada_runtime_dir}/libgnat.a")
add_library(libgnat STATIC IMPORTED GLOBAL)
set_property(TARGET libgnat PROPERTY IMPORTED_LOCATION "${ada_runtime_dir}/libgnat.a")
target_link_libraries(${CMAKE_PROJECT_NAME}.elf PUBLIC libgnat)

#####################################