#include "local_video.hpp"
using namespace rmcs::cap;

struct LocalVideo::Impl { };

auto LocalVideo::configure(const ConfigDetail& config) noexcept
    -> std::expected<void, std::string> {
    (void)config;
    return { };
}

auto LocalVideo::connect() noexcept -> std::expected<void, std::string> { return { }; }

auto LocalVideo::connected() const noexcept -> bool { return false; }

auto LocalVideo::wait_image() -> std::expected<std::unique_ptr<Image>, std::string> {
    return std::unexpected("Not implemented");
}

LocalVideo::LocalVideo() noexcept
    : pimpl { std::make_unique<Impl>() } { }

LocalVideo::~LocalVideo() noexcept = default;
