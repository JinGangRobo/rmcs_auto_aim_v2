# Backend: rknn (Rockchip RKNPU2)
#
# The Rockchip runtime (headers + prebuilt aarch64 librknnrt.so) is not vendored
# in this package. It is downloaded on demand from a release mirror, or taken
# from a local path via RMCS_RKNN_RUNTIME_DIR. This module must stay lazy: every
# backend module is included unconditionally by cmake/InferBackend.cmake.
#
# Overrides:
#   RMCS_RKNN_RUNTIME_DIR  directory holding `librknn_api/`, or a runtime tarball
#   RMCS_RKNN_RUNTIME_URL  download URL of the runtime tarball

set(RMCS_AUTO_AIM_BACKEND_RKNN_FOUND OFF)

if(RMCS_AUTO_AIM_INFER_BACKEND STREQUAL "rknn")
    set(_rknn_default_url
        "https://github.com/mide233/rknn-toolkit2/releases/download/v2.3.0/rknn-runtime-linux-aarch64-2.3.0.tar.gz"
    )
    set(_rknn_default_sha256
        "b5ce6ad2d5d36819ddb884608707ab7eea8f7b5908c1db70be66733ad0b4709b"
    )

    set(RMCS_RKNN_RUNTIME_DIR "" CACHE PATH
        "Local RKNN runtime directory (with librknn_api/) or runtime tarball"
    )
    set(RMCS_RKNN_RUNTIME_URL "${_rknn_default_url}" CACHE STRING
        "Download URL of the RKNN linux-aarch64 runtime tarball"
    )

    set(_rknn_root "")

    # 1) A local runtime takes precedence, so offline builds work.
    if(RMCS_RKNN_RUNTIME_DIR)
        if(IS_DIRECTORY "${RMCS_RKNN_RUNTIME_DIR}")
            set(_rknn_root "${RMCS_RKNN_RUNTIME_DIR}")
        elseif(EXISTS "${RMCS_RKNN_RUNTIME_DIR}")
            set(_rknn_root "${CMAKE_CURRENT_BINARY_DIR}/_deps/rknn-runtime")
            if(NOT EXISTS "${_rknn_root}/librknn_api/aarch64/librknnrt.so")
                file(ARCHIVE_EXTRACT
                    INPUT "${RMCS_RKNN_RUNTIME_DIR}"
                    DESTINATION "${_rknn_root}"
                )
            endif()
        else()
            message(WARNING "RMCS_RKNN_RUNTIME_DIR does not exist: ${RMCS_RKNN_RUNTIME_DIR}")
        endif()
    endif()

    # 2) Otherwise download the runtime on aarch64 (cross-)builds.
    if(NOT _rknn_root AND CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
        set(_rknn_root "${CMAKE_CURRENT_BINARY_DIR}/_deps/rknn-runtime")

        if(NOT EXISTS "${_rknn_root}/librknn_api/aarch64/librknnrt.so")
            set(_rknn_archive "${CMAKE_CURRENT_BINARY_DIR}/_deps/rknn-runtime.tar.gz")

            set(_rknn_hash_args "")
            if("${RMCS_RKNN_RUNTIME_URL}" STREQUAL "${_rknn_default_url}")
                set(_rknn_hash_args EXPECTED_HASH "SHA256=${_rknn_default_sha256}")
            endif()

            message(STATUS "rmcs_auto_aim_v2: downloading RKNN runtime from "
                "${RMCS_RKNN_RUNTIME_URL}"
            )
            file(DOWNLOAD
                "${RMCS_RKNN_RUNTIME_URL}" "${_rknn_archive}"
                ${_rknn_hash_args}
                TLS_VERIFY ON
                STATUS _rknn_download_status
            )
            list(GET _rknn_download_status 0 _rknn_download_code)

            if(_rknn_download_code EQUAL 0)
                file(ARCHIVE_EXTRACT INPUT "${_rknn_archive}" DESTINATION "${_rknn_root}")
            else()
                message(WARNING "rmcs_auto_aim_v2: RKNN runtime download failed "
                    "(${_rknn_download_status})"
                )
            endif()
        endif()
    endif()

    # 3) Wire up the backend when the runtime is available.
    if(_rknn_root AND EXISTS "${_rknn_root}/librknn_api/aarch64/librknnrt.so")
        add_library(rknn::runtime SHARED IMPORTED)
        set_target_properties(rknn::runtime PROPERTIES
            IMPORTED_LOCATION "${_rknn_root}/librknn_api/aarch64/librknnrt.so"
            INTERFACE_INCLUDE_DIRECTORIES "${_rknn_root}/librknn_api/include"
        )

        set(RMCS_AUTO_AIM_BACKEND_RKNN_FOUND ON)
        set(RMCS_AUTO_AIM_BACKEND_RKNN_SOURCES
            "${RMCS_AUTO_AIM_ROOT}/src/utility/model/backend/rknn_backend.cpp"
        )
        set(RMCS_AUTO_AIM_BACKEND_RKNN_LIBS rknn::runtime)
        set(RMCS_AUTO_AIM_BACKEND_RKNN_INCLUDE
            "#include \"utility/model/backend/rknn_backend.hpp\""
        )
        set(RMCS_AUTO_AIM_BACKEND_RKNN_FACTORY "rmcs::RknnBackend::create(spec)")

        # Ship the runtime library with the package for board deployment.
        install(FILES "${_rknn_root}/librknn_api/aarch64/librknnrt.so" DESTINATION lib)
    else()
        message(WARNING
            "rmcs_auto_aim_v2: rknn backend requested but the runtime is unavailable "
            "(host '${CMAKE_SYSTEM_PROCESSOR}'); the detector will be unavailable"
        )
    endif()
endif()
