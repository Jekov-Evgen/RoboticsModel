#pragma once
// Crash-safe file writing (internal): write to a temporary file, then rename over the target.

#include <chrono>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>
#include <system_error>

namespace robotics::detail {

template <class Writer>
void writeFileAtomically(const std::string& path, Writer&& write) {
    auto stamp = std::chrono::steady_clock::now().time_since_epoch().count();
    std::filesystem::path temporary = path + "." + std::to_string(stamp) + ".robot.tmp";
    try {
        std::ofstream out(temporary);
        if (!out) {
            throw std::runtime_error("Cannot create scene file: " + path);
        }
        write(out);
        out.close();
        if (!out) {
            throw std::runtime_error("Failed to flush scene file");
        }
        std::filesystem::rename(temporary, path);
    } catch (...) {
        std::error_code ignored;
        std::filesystem::remove(temporary, ignored);
        throw;
    }
}

} // namespace robotics::detail
