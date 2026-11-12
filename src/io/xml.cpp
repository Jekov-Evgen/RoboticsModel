#include "xml.hpp"

#include <cstdint>

namespace robotics::detail {
namespace {

constexpr int maxDepth = 256;

bool isSpace(char c) {
    return c == ' ' || c == '\t' || c == '\n' || c == '\r';
}

bool isNameStart(char c) {
    auto u = static_cast<unsigned char>(c);
    return (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') || c == '_' || c == ':' || u >= 0x80;
}

bool isNameChar(char c) {
    return isNameStart(c) || (c >= '0' && c <= '9') || c == '-' || c == '.';
}

void appendUtf8(std::string& out, std::uint32_t code) {
    if (code < 0x80) {
        out += static_cast<char>(code);
    } else if (code < 0x800) {
        out += static_cast<char>(0xC0 | (code >> 6));
        out += static_cast<char>(0x80 | (code & 0x3F));
    } else if (code < 0x10000) {
        out += static_cast<char>(0xE0 | (code >> 12));
        out += static_cast<char>(0x80 | ((code >> 6) & 0x3F));
        out += static_cast<char>(0x80 | (code & 0x3F));
    } else {
        out += static_cast<char>(0xF0 | (code >> 18));
        out += static_cast<char>(0x80 | ((code >> 12) & 0x3F));
        out += static_cast<char>(0x80 | ((code >> 6) & 0x3F));
        out += static_cast<char>(0x80 | (code & 0x3F));
    }
}

std::string describe(char c) {
    if (c == '\n' || c == '\r') {
        return "end of line";
    }
    return std::string("'") + c + "'";
}

class Parser {
public:
    Parser(std::string_view text, std::string source) : text_(text), source_(std::move(source)) {}

    XmlElement document() {
        if (text_.substr(0, 3) == "\xEF\xBB\xBF") {
            advance(3);
        }
        skipMisc();
        if (atEnd()) {
            fail("the document is empty (no root element)");
        }
        if (peek() != '<') {
            fail("expected '<' to start the root element, found " + describe(peek()));
        }
        XmlElement root = element(0);
        skipMisc();
        if (!atEnd()) {
            fail("unexpected content after the closing </" + root.name + "> tag of the root element");
        }
        return root;
    }

private:
    std::string_view text_;
    std::string source_;
    std::size_t pos_{};
    int line_{1};
    std::size_t lineStart_{};

    [[noreturn]] void fail(const std::string& message) const {
        throw XmlError(source_, line_, static_cast<int>(pos_ - lineStart_) + 1, message);
    }

    [[nodiscard]] bool atEnd() const { return pos_ >= text_.size(); }

    [[nodiscard]] char peek() const { return text_[pos_]; }

    [[nodiscard]] bool startsWith(std::string_view prefix) const {
        return text_.substr(pos_, prefix.size()) == prefix;
    }

    void advance(std::size_t count = 1) {
        for (std::size_t i = 0; i < count && pos_ < text_.size(); ++i) {
            if (text_[pos_] == '\n') {
                ++line_;
                lineStart_ = pos_ + 1;
            }
            ++pos_;
        }
    }

    void skipSpace() {
        while (!atEnd() && isSpace(peek())) {
            advance();
        }
    }

    // Skips everything up to and including `terminator`; `what` names the construct for errors.
    void skipPast(std::string_view terminator, const char* what) {
        int startLine = line_;
        while (!atEnd() && !startsWith(terminator)) {
            advance();
        }
        if (atEnd()) {
            fail(std::string(what) + " started on line " + std::to_string(startLine) + " is never closed");
        }
        advance(terminator.size());
    }

    void skipDoctype() {
        int startLine = line_;
        int depth = 0;
        advance(9); // "<!DOCTYPE"
        while (!atEnd()) {
            char c = peek();
            if (c == '[') {
                ++depth;
            } else if (c == ']') {
                --depth;
            } else if (c == '>' && depth <= 0) {
                advance();
                return;
            }
            advance();
        }
        line_ = startLine;
        fail("DOCTYPE started on line " + std::to_string(startLine) + " is never closed");
    }

    // Whitespace, comments, processing instructions and DOCTYPE outside the root element.
    void skipMisc() {
        while (true) {
            skipSpace();
            if (startsWith("<!--")) {
                skipPast("-->", "comment");
            } else if (startsWith("<?")) {
                skipPast("?>", "processing instruction");
            } else if (startsWith("<!DOCTYPE")) {
                skipDoctype();
            } else {
                return;
            }
        }
    }

    std::string name(const char* context) {
        if (atEnd() || !isNameStart(peek())) {
            fail(std::string("expected ") + context
                 + (atEnd() ? ", found end of file" : ", found " + describe(peek())));
        }
        std::size_t start = pos_;
        while (!atEnd() && isNameChar(peek())) {
            advance();
        }
        return std::string(text_.substr(start, pos_ - start));
    }

    void entity(std::string& out) {
        std::size_t end = text_.find(';', pos_);
        if (end == std::string_view::npos || end - pos_ > 12) {
            fail("'&' must start an entity such as &amp; (write &amp; for a literal ampersand)");
        }
        std::string_view ref = text_.substr(pos_ + 1, end - pos_ - 1);
        if (ref == "lt") {
            out += '<';
        } else if (ref == "gt") {
            out += '>';
        } else if (ref == "amp") {
            out += '&';
        } else if (ref == "quot") {
            out += '"';
        } else if (ref == "apos") {
            out += '\'';
        } else if (ref.size() > 1 && ref[0] == '#') {
            bool hex = ref[1] == 'x' || ref[1] == 'X';
            std::string_view digits = ref.substr(hex ? 2 : 1);
            std::uint32_t code = 0;
            bool valid = !digits.empty();
            for (char c : digits) {
                int digit = c >= '0' && c <= '9'          ? c - '0'
                            : hex && c >= 'a' && c <= 'f' ? c - 'a' + 10
                            : hex && c >= 'A' && c <= 'F' ? c - 'A' + 10
                                                          : -1;
                if (digit < 0 || code > 0x10FFFF) {
                    valid = false;
                    break;
                }
                code = code * (hex ? 16 : 10) + static_cast<std::uint32_t>(digit);
            }
            if (!valid || code == 0 || code > 0x10FFFF || (code >= 0xD800 && code <= 0xDFFF)) {
                fail("invalid character reference &" + std::string(ref) + ";");
            }
            appendUtf8(out, code);
        } else {
            fail("unknown entity &" + std::string(ref)
                 + "; (only &lt; &gt; &amp; &quot; &apos; are defined)");
        }
        advance(end - pos_ + 1);
    }

    std::string attributeValue(const std::string& attribute) {
        if (atEnd() || (peek() != '"' && peek() != '\'')) {
            fail("the value of attribute '" + attribute + "' must be in quotes");
        }
        char quote = peek();
        int startLine = line_;
        advance();
        std::string value;
        while (true) {
            if (atEnd()) {
                line_ = startLine;
                fail("the value of attribute '" + attribute + "' is missing its closing quote");
            }
            char c = peek();
            if (c == quote) {
                advance();
                return value;
            }
            if (c == '<') {
                fail("'<' is not allowed inside the value of attribute '" + attribute
                     + "' (missing closing quote?)");
            }
            if (c == '&') {
                entity(value);
                continue;
            }
            value += c;
            advance();
        }
    }

    XmlElement element(int depth) {
        if (depth > maxDepth) {
            fail("elements are nested too deeply");
        }
        XmlElement node;
        node.line = line_;
        advance(); // '<'
        node.name = name("an element name after '<'");
        while (true) {
            bool spaced = !atEnd() && isSpace(peek());
            skipSpace();
            if (atEnd()) {
                fail("the tag <" + node.name + "> is not closed with '>'");
            }
            if (startsWith("/>")) {
                advance(2);
                return node;
            }
            if (peek() == '>') {
                advance();
                break;
            }
            if (!spaced) {
                fail("expected whitespace, '>' or '/>' after <" + node.name + ", found " + describe(peek()));
            }
            std::string key = name("an attribute name or '>'");
            skipSpace();
            if (atEnd() || peek() != '=') {
                fail("expected '=' after attribute '" + key + "' of <" + node.name + ">");
            }
            advance();
            skipSpace();
            std::string value = attributeValue(key);
            if (node.attribute(key)) {
                fail("attribute '" + key + "' appears twice in <" + node.name + ">");
            }
            node.attributes.emplace_back(std::move(key), std::move(value));
        }
        content(node, depth);
        return node;
    }

    void content(XmlElement& node, int depth) {
        while (true) {
            if (atEnd()) {
                throw XmlError(source_, node.line, 1,
                               "<" + node.name + "> opened on line " + std::to_string(node.line)
                                   + " is never closed (missing </" + node.name + ">)");
            }
            if (startsWith("</")) {
                advance(2);
                std::string closing = name("an element name after '</'");
                if (closing != node.name) {
                    fail("closing tag </" + closing + "> does not match <" + node.name + "> opened on line "
                         + std::to_string(node.line));
                }
                skipSpace();
                if (atEnd() || peek() != '>') {
                    fail("expected '>' to finish </" + closing + ">");
                }
                advance();
                return;
            }
            if (startsWith("<!--")) {
                skipPast("-->", "comment");
            } else if (startsWith("<![CDATA[")) {
                int startLine = line_;
                advance(9);
                std::size_t end = text_.find("]]>", pos_);
                if (end == std::string_view::npos) {
                    line_ = startLine;
                    fail("CDATA section started on line " + std::to_string(startLine) + " is never closed");
                }
                node.text.append(text_.substr(pos_, end - pos_));
                advance(end - pos_ + 3);
            } else if (startsWith("<?")) {
                skipPast("?>", "processing instruction");
            } else if (peek() == '<') {
                node.children.push_back(element(depth + 1));
            } else if (peek() == '&') {
                entity(node.text);
            } else {
                node.text += peek();
                advance();
            }
        }
    }
};

} // namespace

const std::string* XmlElement::attribute(std::string_view key) const {
    for (const auto& [attributeName, value] : attributes) {
        if (attributeName == key) {
            return &value;
        }
    }
    return nullptr;
}

const XmlElement* XmlElement::child(std::string_view childName) const {
    for (const auto& element : children) {
        if (element.name == childName) {
            return &element;
        }
    }
    return nullptr;
}

std::vector<const XmlElement*> XmlElement::childrenNamed(std::string_view childName) const {
    std::vector<const XmlElement*> result;
    for (const auto& element : children) {
        if (element.name == childName) {
            result.push_back(&element);
        }
    }
    return result;
}

XmlError::XmlError(const std::string& source, int errorLine, int errorColumn, const std::string& message)
    : std::runtime_error(source + ", line " + std::to_string(errorLine) + ", column "
                         + std::to_string(errorColumn) + ": " + message),
      line(errorLine), column(errorColumn) {
}

XmlElement parseXml(std::string_view text, const std::string& source) {
    return Parser(text, source).document();
}

} // namespace robotics::detail
