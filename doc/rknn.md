# RKNN 后端

## 概述

推理后端在**编译期**选择，通过 CMake 变量 `RMCS_AUTO_AIM_INFER_BACKEND` 控制：

| 取值 | 说明 |
|---|---|
| `openvino` | 默认值，使用 OpenVINO 后端 |
| `rknn` | 使用瑞芯微 RKNPU（RK3588）后端 |
| `none`/其他 | 不编译任何后端，`has_infer_backend()` 返回 `false` |

RKNN 后端实现位于：

- `src/utility/model/backend/rknn_backend.hpp`
- `src/utility/model/backend/rknn_backend.cpp`
- `cmake/backend/rknn.cmake`

后端在 `/workspaces/RMCS/rmcs_ws/src/rmcs_auto_aim_v2` 仓库的 `dev/rk3588npu` 分支上。

## 模型文件

后端收到的 `ModelSpec::location` 是指向 `.onnx` 的**绝对路径**（由 `kernel/auto_aim.cpp` 用
`util::Parameters::share_location() / config["model_location"]` 拼出）。RKNN 后端会**自动把扩展名替换为
`.rknn`**，读取同目录同名的模型：

```
<share>/rmcs_auto_aim_v2/models/shenzhen-0526.onnx  ->  <share>/rmcs_auto_aim_v2/models/shenzhen-0526.rknn
```

因此：

- `config/config.yaml` 中的 `model_location` **不需要修改**，模型类型识别（按文件名判断
  `ShenZhen0526` / `ShenZhen0708` 等）也不受影响。
- 只需把对应的 `.rknn` 文件与 `.onnx` 放在 `models/` 下（会随 `install(DIRECTORY models/)` 一起安装）。

## 运行时依赖（librknnrt.so）

RKNN 后端需要 Rockchip 的 `librknnrt.so` 与头文件：

- 默认下载地址：
  `https://github.com/mide233/rknn-toolkit2/releases/download/v2.3.0/rknn-runtime-linux-aarch64-2.3.0.tar.gz`
- SHA256：`b5ce6ad2d5d36819ddb884608707ab7eea8f7b5908c1db70be66733ad0b4709b`

`cmake/backend/rknn.cmake` 的行为：

1. **仅在选择了 `rknn` 后端时才工作**（未选中时不下载、不联网，保证可切换）。
2. 若提供了 `RMCS_RKNN_RUNTIME_DIR`，优先使用本地运行时（目录或 `.tar.gz`）。
3. 否则，仅当交叉编译且 `CMAKE_SYSTEM_PROCESSOR` 匹配 `aarch64|arm64` 时，才从上面的默认地址下载并解压。
4. 在 x86_64 主机上选择 `rknn` 会**警告并降级为无后端**（不会编译失败），因为主机上没有 aarch64 运行时。

运行时被解压/定位到 `<root>/librknn_api/{include, aarch64/librknnrt.so}`，并：

- 以 IMPORTED 目标 `rknn::runtime` 链接到 `_utility`；
- `install(FILES .../librknnrt.so DESTINATION lib)`，即 **`librknnrt.so` 随包安装到 `lib/`**（板端部署需要）。

### 离线 / 内网覆盖

| 变量 | 默认 | 说明 |
|---|---|---|
| `RMCS_RKNN_RUNTIME_DIR` | 空 | 本地运行时目录（需含 `librknn_api/aarch64/librknnrt.so`），或 `rknn-runtime-*.tar.gz` 文件 |
| `RMCS_RKNN_RUNTIME_URL` | 上面的默认地址 | 自定义下载地址；**自定义 URL 时不做 SHA256 校验** |

## 输入 / 输出约定

- **零拷贝**：输入/输出张量内存各用 `rknn_create_mem` 分配一次，并通过 `rknn_set_io_mem` 绑定，
  每帧推理不再有暂存拷贝。输入用 `cv::cvtColor` 直接写入 NPU 输入内存；输出直接从 NPU 输出内存
  读取，`InferOutput::data` 指向该内存，生命周期保持到下一次 `infer()`（与 OpenVINO 后端一致）。
- **输入**：`u8`、`NHWC`、**RGB**、640×640，`pass_through=0`（由 RKNPU 运行时完成归一化），与官方
  `rknn_yolov5_demo` 一致。后端内部会把输入 `cv::Mat`（BGR）转成 RGB。
- **输出**：单个输出张量 `[1, 25200, 22]`（float），直接作为 `InferOutput` 交给上层
  `InferResultAdapter` 做 sigmoid / NMS / 坐标还原，无需额外解码。

## 如何选择后端

> **注意**：`./.script/build-rmcs-cross` 与 `./.script/build-rmcs` 都会在命令行末尾再追加一次
> `--cmake-args`，而 colcon 的 `--cmake-args` 是"覆盖"而非"追加"。因此通过脚本传入的
> `--cmake-args -DRMCS_AUTO_AIM_INFER_BACKEND=rknn` **不会生效**。

由于 `RMCS_AUTO_AIM_INFER_BACKEND` 是 CMake **缓存变量**，可以在构建目录已存在后，用一次性重配置来设置：

```sh
cd rmcs_ws

# 交叉编译（目标 arm64 / RK3588）：把该包的缓存变量设为 rknn
cmake -S src/rmcs_auto_aim_v2 -B build-cross-arm64/rmcs_auto_aim_v2 \
      -DRMCS_AUTO_AIM_INFER_BACKEND=rknn

# 之后按正常方式构建（无需再传 -D，缓存会保留）
./.script/build-rmcs-cross --target-arch arm64 --packages-select rmcs_auto_aim_v2
```

本机构建同理，构建目录为 `build/rmcs_auto_aim_v2`（x86_64 主机上会因无 aarch64 运行时降级为无后端）：

```sh
cmake -S src/rmcs_auto_aim_v2 -B build/rmcs_auto_aim_v2 -DRMCS_AUTO_AIM_INFER_BACKEND=rknn
```

切回默认：

```sh
cmake -S src/rmcs_auto_aim_v2 -B build-cross-arm64/rmcs_auto_aim_v2 \
      -DRMCS_AUTO_AIM_INFER_BACKEND=openvino
./.script/build-rmcs-cross --target-arch arm64 --packages-select rmcs_auto_aim_v2
```

如果构建目录不存在，先正常构建一次使其生成；也可用 `--cmake-clean-cache` 清理后再重配置。

### 验证选择结果

```sh
# 生成的头文件里应出现 rknn
grep BACKEND_NAME rmcs_ws/build-cross-arm64/rmcs_auto_aim_v2/generated/infer_backend_config.hpp
#   #define RMCS_AUTO_AIM_BACKEND_NAME "rknn"

# 动态依赖里应出现 librknnrt.so
readelf -d rmcs_ws/build-cross-arm64/rmcs_auto_aim_v2/librmcs_auto_aim_v2_utility.so | grep rknn
#   Shared library: [librknnrt.so]

# 运行时库应已安装
ls -l rmcs_ws/install-cross-arm64/lib/librknnrt.so
```
