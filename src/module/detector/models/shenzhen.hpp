#pragma once
#include "utility/model/armor_detection.hpp"
#include "utility/model/common_model.hpp"


namespace rmcs {

struct ShenZhenBasic {
    TensorLayout input_layout = TensorLayout::from<"NHWC">();
    TensorLayout model_layout = TensorLayout::from<"NCHW">();
    Dimensions dimensions     = Dimensions { .W = 640, .H = 640 };
    std::string infer_device  = "AUTO";

    struct ResultData {
        using precision_type = float;

        struct Corners {
            precision_type lt_x;
            precision_type lt_y;
            precision_type lb_x;
            precision_type lb_y;
            precision_type rb_x;
            precision_type rb_y;
            precision_type rt_x;
            precision_type rt_y;
        } corners;

        precision_type confidence;

        struct Color {
            precision_type blue;
            precision_type red;
            precision_type dark;
            precision_type mix;
        } color;

        struct Genre {
            precision_type sentry;
            precision_type hero;
            precision_type engineer;
            precision_type infantry_3;
            precision_type infantry_4;
            precision_type infantry_5;
            precision_type outpost;
            precision_type base_small;
            precision_type base_large;
        } genre;
    };

    using Result = InferResultAdapter<ResultData>;
};

}
