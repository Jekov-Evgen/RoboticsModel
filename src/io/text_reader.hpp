#pragma once
// Whitespace-token reading helpers shared by text scene formats (internal).

#include "robotics/math.hpp"

#include <cstddef>
#include <istream>
#include <stdexcept>
#include <string>

namespace robotics::detail {

inline void readVec(std::istream& in, Vec3& v) {
    in >> v.x >> v.y >> v.z;
}

inline void readQuat(std::istream& in, Quat& q) {
    in >> q.w >> q.x >> q.y >> q.z;
}

// Consumes the next token and throws unless it equals `word`.
inline void expectToken(std::istream& in, const char* word) {
    std::string token;
    if (!(in >> token) || token != word) {
        throw std::runtime_error(std::string("Expected scene token: ") + word);
    }
}

// Reads a record count in [0, limit].
inline std::size_t readCount(std::istream& in, std::size_t limit) {
    long long count = 0;
    if (!(in >> count) || count < 0 || static_cast<unsigned long long>(count) > limit) {
        throw std::runtime_error("Invalid scene object count");
    }
    return static_cast<std::size_t>(count);
}

} // namespace robotics::detail
