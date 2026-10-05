# Backend: openvino
find_package(OpenVINO QUIET)

set(RMCS_AUTO_AIM_BACKEND_OPENVINO_FOUND ${OpenVINO_FOUND})
set(RMCS_AUTO_AIM_BACKEND_OPENVINO_SOURCES
    "${RMCS_AUTO_AIM_ROOT}/src/utility/model/backend/openvino_backend.cpp"
)
set(RMCS_AUTO_AIM_BACKEND_OPENVINO_LIBS openvino::runtime)
set(RMCS_AUTO_AIM_BACKEND_OPENVINO_INCLUDE
    "#include \"utility/model/backend/openvino_backend.hpp\""
)
set(RMCS_AUTO_AIM_BACKEND_OPENVINO_FACTORY "rmcs::OpenVinoBackend::create(spec)")
