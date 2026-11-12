#pragma once
// Minimal XML 1.0 reader for importers (internal to src/io).
//
// Supports elements, attributes (single or double quotes), character data, comments, CDATA sections,
// processing instructions and an internal DOCTYPE (both skipped), the five predefined entities and numeric
// character references. Namespace prefixes are kept as part of the name ("xacro:macro"). Input must be UTF-8.
// Errors throw XmlError with the line and column of the problem.

#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace robotics::detail {

struct XmlElement {
    std::string name;
    std::vector<std::pair<std::string, std::string>> attributes; // Document order, entities decoded.
    std::vector<XmlElement> children;
    std::string text; // Character data directly inside this element (children's text excluded).
    int line{};       // Line of the opening '<'.

    // nullptr when the attribute is absent.
    [[nodiscard]] const std::string* attribute(std::string_view key) const;

    // First child element with this name, or nullptr.
    [[nodiscard]] const XmlElement* child(std::string_view childName) const;

    [[nodiscard]] std::vector<const XmlElement*> childrenNamed(std::string_view childName) const;
};

class XmlError : public std::runtime_error {
public:
    XmlError(const std::string& source, int line, int column, const std::string& message);

    int line;
    int column;
};

// Parses a complete document and returns its root element. `source` (e.g. a file name) prefixes errors:
// "robot.urdf, line 12, column 5: ...".
[[nodiscard]] XmlElement parseXml(std::string_view text, const std::string& source = "XML");

} // namespace robotics::detail
