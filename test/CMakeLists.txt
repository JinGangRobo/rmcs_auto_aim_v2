cmake_minimum_required(VERSION 3.22)
project(rmcs_auto_aim_v2_tests)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(RMCS_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/..")
set(RMCS_SRC_DIR "${RMCS_SOURCE_DIR}/src")

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rmcs_msgs REQUIRED)

find_package(yaml-cpp REQUIRED)
find_package(OpenCV 4.5 REQUIRED)

# Resolve the inference backend; reuse the parent package's selection when the
# tests are built as part of the package, otherwise resolve it independently.
if(NOT DEFINED RMCS_AUTO_AIM_HAS_BACKEND)
    set(RMCS_AUTO_AIM_ROOT "${RMCS_SOURCE_DIR}")
    if(NOT DEFINED RMCS_AUTO_AIM_INFER_BACKEND)
        set(RMCS_AUTO_AIM_INFER_BACKEND "openvino")
    endif()
    include("${RMCS_AUTO_AIM_ROOT}/cmake/InferBackend.cmake")
    rmcs_auto_aim_configure_infer_backend()
endif()

include_directories(
    ${RMCS_SRC_DIR}
    ${RMCS_AUTO_AIM_BACKEND_GENERATED_INCLUDE_DIR}
    ${hikcamera_INCLUDE_DIRS}
    ${rclcpp_INCLUDE_DIRS}
    ${visualization_msgs_INCLUDE_DIRS}
    ${geometry_msgs_INCLUDE_DIRS}
    ${rmcs_msgs_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

set(TEST_DIR ${CMAKE_CURRENT_SOURCE_DIR})

# Detector and inference backend sources (compiled directly into tests; keep in sync with the package)
set(AUTO_AIM_DETECTOR_SOURCES
    ${RMCS_SRC_DIR}/module/detector/armor_detection.cpp
    ${RMCS_SRC_DIR}/utility/model/infer_backend.cpp
    ${RMCS_AUTO_AIM_BACKEND_SELECTED_SOURCES}
)

# Duck
ament_add_gtest(
    test_duck_type
    ${TEST_DIR}/duck_type.cpp
)

# Serializable
ament_add_gtest(
    test_serializable
    ${TEST_DIR}/serializable.cpp
)
target_link_libraries(
    test_serializable
    rclcpp::rclcpp
    yaml-cpp::yaml-cpp
)

# Interprocess
ament_add_gtest(
    test_interprocess
    ${TEST_DIR}/interprocess.cpp
)

# Model Infer
ament_add_gtest(
    test_model_infer
    ${TEST_DIR}/model_infer.cpp
    ${AUTO_AIM_DETECTOR_SOURCES}
)
target_link_libraries(
    test_model_infer
    yaml-cpp::yaml-cpp
    ${OpenCV_LIBRARIES}
)
if(RMCS_AUTO_AIM_BACKEND_SELECTED_LIBS)
    target_link_libraries(test_model_infer ${RMCS_AUTO_AIM_BACKEND_SELECTED_LIBS})
endif()

# Device Id
ament_add_gtest(
    test_device_id
    ${TEST_DIR}/device_id.cpp
)

# Solve pnp
ament_add_gtest(
    test_solve_pnp
    ${TEST_DIR}/solve_pnp.cpp
    ${RMCS_SRC_DIR}/utility/math/solve_pnp/pnp_solution.cpp
    ${RMCS_SRC_DIR}/utility/math/solve_pnp/yaw_optimizer.cpp
    ${RMCS_SRC_DIR}/utility/math/reprojection.cpp
    ${RMCS_SRC_DIR}/utility/math/camera.cpp
    ${AUTO_AIM_DETECTOR_SOURCES}
)
ament_target_dependencies(
    test_solve_pnp
    rclcpp
    visualization_msgs
    geometry_msgs
)
target_link_libraries(
    test_solve_pnp
    ${OpenCV_LIBRARIES}
    yaml-cpp::yaml-cpp
)
if(RMCS_AUTO_AIM_BACKEND_SELECTED_LIBS)
    target_link_libraries(test_solve_pnp ${RMCS_AUTO_AIM_BACKEND_SELECTED_LIBS})
endif()

# Static TF
ament_add_gtest(
    test_static_tf
    ${TEST_DIR}/static_tf.cpp
    ${RMCS_SRC_DIR}/utility/panic.cpp
)
target_link_libraries(
    test_static_tf
    yaml-cpp::yaml-cpp
)

# Transform Communication
ament_add_gtest(
    test_transform_communication
    ${TEST_DIR}/transform_communication.cpp
)

# Repeat
ament_add_gtest(
    test_repeat
    ${TEST_DIR}/repeat.cpp
)

# Rune
# ament_add_gtest(
#     test_rune
#     ${TEST_DIR}/rune.cpp
#     ${RMCS_SRC_DIR}/module/detector/rune.cpp
# )
# target_link_libraries(
#     test_rune
#     ${OpenCV_LIBRARIES}
# )
