#include "utility/model/infer_backend.hpp"

#include "infer_backend_config.hpp"

#include <string>

namespace rmcs {

auto has_infer_backend() noexcept -> bool { return RMCS_AUTO_AIM_HAS_BACKEND != 0; }

auto InferBackend::create(const ModelSpec& spec)
    -> std::expected<std::unique_ptr<InferBackend>, std::string> {
    return detail::create_selected_backend(spec);
}

}
