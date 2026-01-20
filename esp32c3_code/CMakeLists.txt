# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(tfg_mk3)

###========NEW FOR ADA/SPARK INTEGRATION===========###

### Build the Ada code using Alire

# 1 y 2. CORREGIDO: Usamos add_custom_target directamente con el COMMAND.
# Al no tener un "OUTPUT", CMake considera que este target SIEMPRE está 
# "sucio" (out-of-date) y ejecutará 'alr build' en cada compilación.
# Dejamos que Alire decida si es necesario recompilar o no (build incremental).
add_custom_target(ada_compile ALL
    COMMAND alr -n build
    WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}/ada_code/"
    COMMENT "Verificando cambios y compilando Ada con Alire..."
    # BYPRODUCTS le dice a CMake que este target genera este archivo,
    # útil para dependencias de limpieza, aunque no estrictamente obligatorio.
    BYPRODUCTS "${CMAKE_SOURCE_DIR}/ada_code/lib/libAda_Code.a"
)

# 3. Importamos la librería estática (Igual que antes)
# Esto crea una referencia lógica al archivo .a para que CMake lo pueda enlazar.
add_library(ada_code STATIC IMPORTED GLOBAL)
set_property(TARGET ada_code PROPERTY IMPORTED_LOCATION "${CMAKE_SOURCE_DIR}/ada_code/lib/libAda_Code.a")

# 4. Forzamos el orden de compilación
# Esto asegura que 'ada_compile' (alr build) se ejecute SIEMPRE antes de que
# CMake intente enlazar 'ada_code' al ejecutable final.
add_dependencies(ada_code ada_compile)

# 5. Vinculamos al ejecutable (Igual que antes)
target_link_libraries(${CMAKE_PROJECT_NAME}.elf PUBLIC ada_code)

### Get path of Ada run-time library (libgnat.a)

# Buscar el ejecutable de Alire automáticamente
find_program(ALR_EXECUTABLE alr)

if(NOT ALR_EXECUTABLE)
    message(FATAL_ERROR "No se encontró el ejecutable 'alr'. Asegúrate de que esté en el PATH.")
endif()

# Definir el comando usando la ruta absoluta encontrada
set(ALR_CMD ${ALR_EXECUTABLE} exec -- riscv64-elf-gnatls --RTS=zfp-rv32imac -v)

# Ejecutar el proceso sin invocar bash explícitamente
execute_process(
    COMMAND ${ALR_CMD}
    WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}/ada_code/"
    RESULT_VARIABLE gnatls_result
    OUTPUT_VARIABLE gnatls_output
    ERROR_VARIABLE gnatls_error
)

if(gnatls_result EQUAL 0)
    # Buscamos la línea que contiene "adalib" de forma interna en CMake
    # Esto elimina la necesidad de usar 'grep'
    string(REGEX MATCH "[^\r\n]*adalib[^\r\n]*" ADALIB_LINE "${gnatls_output}")
    string(STRIP "${ADALIB_LINE}" ADALIB_PATH)
    
    # Aseguramos que la ruta use barras diagonales (compatibilidad Windows)
    file(TO_CMAKE_PATH "${ADALIB_PATH}" ADA_LIB_DIR)
    
    set(ADA_RUNTIME_LIB "${ADA_LIB_DIR}")
    message(STATUS "Ada runtime found at: ${ADA_RUNTIME_LIB}")
else()
    message(FATAL_ERROR "Error obteniendo ruta de Ada: ${gnatls_error}")
endif()

# Add the Ada run-time static library
message(STATUS "Link Ada run-time ${ADA_RUNTIME_LIB}/libgnat.a")
add_library(libgnat STATIC IMPORTED GLOBAL)
set_property(TARGET libgnat PROPERTY IMPORTED_LOCATION "${ADA_RUNTIME_LIB}/libgnat.a")
target_link_libraries(${CMAKE_PROJECT_NAME}.elf PUBLIC libgnat)

#####################################