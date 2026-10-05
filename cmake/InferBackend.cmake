# Resolve the compile-time inference backend for rmcs_auto_aim_v2.
#
# Every backend lives in its own module, `cmake/backend/<name>.cmake`, which is
# discovered automatically (no edits to the main CMakeLists.txt are required to
# add a backend). A module must set the following variables for its backend,
# with <NAME> the upper-cased backend name:
#
#   RMCS_AUTO_AIM_BACKEND_<NAME>_FOUND    SDK availability (ON/OFF)
#   RMCS_AUTO_AIM_BACKEND_<NAME>_SOURCES  sources to compile
#   RMCS_AUTO_AIM_BACKEND_<NAME>_LIBS     libraries/targets to link
#   RMCS_AUTO_AIM_BACKEND_<NAME>_INCLUDE  one `#include "..."` line for the config header
#   RMCS_AUTO_AIM_BACKEND_<NAME>_FACTORY  factory call expression, e.g. `rmcs::Foo::create(spec)`
#
# Inputs : RMCS_AUTO_AIM_ROOT, RMCS_AUTO_AIM_INFER_BACKEND
# Outputs (PARENT_SCOPE):
#   RMCS_AUTO_AIM_HAS_BACKEND
#   RMCS_AUTO_AIM_BACKEND_SELECTED_NAME
#   RMCS_AUTO_AIM_BACKEND_SELECTED_SOURCES
#   RMCS_AUTO_AIM_BACKEND_SELECTED_LIBS
#   RMCS_AUTO_AIM_BACKEND_GENERATED_INCLUDE_DIR
function(rmcs_auto_aim_configure_infer_backend)
    # Auto-include every backend module; a new file takes effect on reconfigure.
    file(GLOB _backend_modules CONFIGURE_DEPENDS "${RMCS_AUTO_AIM_ROOT}/cmake/backend/*.cmake")
    foreach(_module IN LISTS _backend_modules)
        include("${_module}")
    endforeach()

    set(_selected_name "${RMCS_AUTO_AIM_INFER_BACKEND}")
    set(_has_backend OFF)
    set(_selected_sources)
    set(_selected_libs)
    set(_selected_include "")
    set(_selected_body "")

    if(NOT _selected_name STREQUAL "none")
        string(TOUPPER "${_selected_name}" _selected_upper)
        if(RMCS_AUTO_AIM_BACKEND_${_selected_upper}_FOUND)
            set(_has_backend ON)
            set(_selected_sources ${RMCS_AUTO_AIM_BACKEND_${_selected_upper}_SOURCES})
            set(_selected_libs ${RMCS_AUTO_AIM_BACKEND_${_selected_upper}_LIBS})
            set(_selected_include "${RMCS_AUTO_AIM_BACKEND_${_selected_upper}_INCLUDE}")
            set(_selected_body "    return ${RMCS_AUTO_AIM_BACKEND_${_selected_upper}_FACTORY};")
            message(STATUS "rmcs_auto_aim_v2: inference backend '${_selected_name}' enabled")
        else()
            message(STATUS
                "rmcs_auto_aim_v2: inference backend '${_selected_name}' SDK not found, "
                "detector will be unavailable"
            )
            set(_selected_name "none")
        endif()
    else()
        message(STATUS "rmcs_auto_aim_v2: inference backend disabled")
    endif()

    if(NOT _has_backend)
        set(_selected_include "")
        set(_selected_body
            "    (void)spec;
    return std::unexpected { \"No inference backend is compiled in\" };"
        )
    endif()

    # Generate the config header that binds the selected backend into the factory.
    if(_has_backend)
        set(RMCS_AUTO_AIM_HAS_BACKEND 1)
    else()
        set(RMCS_AUTO_AIM_HAS_BACKEND 0)
    endif()
    set(RMCS_AUTO_AIM_BACKEND_SELECTED_NAME "${_selected_name}")
    set(RMCS_AUTO_AIM_BACKEND_SELECTED_INCLUDE "${_selected_include}")
    set(RMCS_AUTO_AIM_BACKEND_SELECTED_BODY "${_selected_body}")

    set(_generated_dir "${CMAKE_CURRENT_BINARY_DIR}/generated")
    file(MAKE_DIRECTORY "${_generated_dir}")
    configure_file(
        "${RMCS_AUTO_AIM_ROOT}/cmake/infer_backend_config.hpp.in"
        "${_generated_dir}/infer_backend_config.hpp"
        @ONLY
    )

    set(RMCS_AUTO_AIM_HAS_BACKEND ${_has_backend} PARENT_SCOPE)
    set(RMCS_AUTO_AIM_BACKEND_SELECTED_NAME "${_selected_name}" PARENT_SCOPE)
    set(RMCS_AUTO_AIM_BACKEND_SELECTED_SOURCES "${_selected_sources}" PARENT_SCOPE)
    set(RMCS_AUTO_AIM_BACKEND_SELECTED_LIBS "${_selected_libs}" PARENT_SCOPE)
    set(RMCS_AUTO_AIM_BACKEND_GENERATED_INCLUDE_DIR "${_generated_dir}" PARENT_SCOPE)
endfunction()
