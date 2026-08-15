#include "internal/json_value.hpp"

#include <cctype>
#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <string>
#include <utility>

namespace rozeta::internal {
namespace {

const std::string& emptyString() {
    static const std::string value;
    return value;
}

const JsonArray& emptyArray() {
    static const JsonArray value;
    return value;
}

const JsonObject& emptyObject() {
    static const JsonObject value;
    return value;
}

class Parser {
public:
    explicit Parser(const std::string& text) : text_(text) {}

    JsonParseResult run() {
        skipWhitespace();
        JsonValue value;
        Status status = parseValue(value, 0);
        if (!status.ok()) {
            return {{}, status};
        }
        skipWhitespace();
        if (cursor_ != text_.size()) {
            return {{}, error("trailing content after JSON document")};
        }
        return {std::move(value), Status::okStatus()};
    }

private:
    static constexpr int kMaxDepth = 64;

    Status error(const std::string& message) const {
        return Status::error(
            ErrorCode::ParseError, "JSON offset " + std::to_string(cursor_) + ": " + message);
    }

    bool atEnd() const { return cursor_ >= text_.size(); }
    char peek() const { return text_[cursor_]; }

    void skipWhitespace() {
        while (!atEnd()) {
            const char c = text_[cursor_];
            if (c == ' ' || c == '\t' || c == '\n' || c == '\r') {
                ++cursor_;
                continue;
            }
            break;
        }
    }

    bool consumeLiteral(const char* literal) {
        const std::string text(literal);
        if (text_.compare(cursor_, text.size(), text) != 0) {
            return false;
        }
        cursor_ += text.size();
        return true;
    }

    Status parseValue(JsonValue& out, int depth) {
        if (depth > kMaxDepth) {
            return error("nesting is too deep");
        }
        skipWhitespace();
        if (atEnd()) {
            return error("unexpected end of document");
        }

        switch (peek()) {
            case '{':
                return parseObject(out, depth);
            case '[':
                return parseArray(out, depth);
            case '"': {
                std::string value;
                Status status = parseString(value);
                if (!status.ok()) {
                    return status;
                }
                out = JsonValue(std::move(value));
                return Status::okStatus();
            }
            case 't':
                if (consumeLiteral("true")) {
                    out = JsonValue(true);
                    return Status::okStatus();
                }
                return error("invalid literal");
            case 'f':
                if (consumeLiteral("false")) {
                    out = JsonValue(false);
                    return Status::okStatus();
                }
                return error("invalid literal");
            case 'n':
                if (consumeLiteral("null")) {
                    out = JsonValue();
                    return Status::okStatus();
                }
                return error("invalid literal");
            default:
                return parseNumber(out);
        }
    }

    Status parseObject(JsonValue& out, int depth) {
        ++cursor_; // '{'
        JsonObject object;
        skipWhitespace();
        if (!atEnd() && peek() == '}') {
            ++cursor_;
            out = JsonValue(std::move(object));
            return Status::okStatus();
        }

        while (true) {
            skipWhitespace();
            if (atEnd() || peek() != '"') {
                return error("expected a member name");
            }
            std::string key;
            Status status = parseString(key);
            if (!status.ok()) {
                return status;
            }
            skipWhitespace();
            if (atEnd() || peek() != ':') {
                return error("expected ':' after a member name");
            }
            ++cursor_;

            JsonValue value;
            status = parseValue(value, depth + 1);
            if (!status.ok()) {
                return status;
            }
            object.emplace(std::move(key), std::move(value));

            skipWhitespace();
            if (atEnd()) {
                return error("unterminated object");
            }
            if (peek() == ',') {
                ++cursor_;
                continue;
            }
            if (peek() == '}') {
                ++cursor_;
                out = JsonValue(std::move(object));
                return Status::okStatus();
            }
            return error("expected ',' or '}' in object");
        }
    }

    Status parseArray(JsonValue& out, int depth) {
        ++cursor_; // '['
        JsonArray array;
        skipWhitespace();
        if (!atEnd() && peek() == ']') {
            ++cursor_;
            out = JsonValue(std::move(array));
            return Status::okStatus();
        }

        while (true) {
            JsonValue value;
            Status status = parseValue(value, depth + 1);
            if (!status.ok()) {
                return status;
            }
            array.push_back(std::move(value));

            skipWhitespace();
            if (atEnd()) {
                return error("unterminated array");
            }
            if (peek() == ',') {
                ++cursor_;
                continue;
            }
            if (peek() == ']') {
                ++cursor_;
                out = JsonValue(std::move(array));
                return Status::okStatus();
            }
            return error("expected ',' or ']' in array");
        }
    }

    static void appendUtf8(std::string& out, unsigned int code_point) {
        if (code_point < 0x80) {
            out.push_back(static_cast<char>(code_point));
        } else if (code_point < 0x800) {
            out.push_back(static_cast<char>(0xC0 | (code_point >> 6)));
            out.push_back(static_cast<char>(0x80 | (code_point & 0x3F)));
        } else if (code_point < 0x10000) {
            out.push_back(static_cast<char>(0xE0 | (code_point >> 12)));
            out.push_back(static_cast<char>(0x80 | ((code_point >> 6) & 0x3F)));
            out.push_back(static_cast<char>(0x80 | (code_point & 0x3F)));
        } else {
            out.push_back(static_cast<char>(0xF0 | (code_point >> 18)));
            out.push_back(static_cast<char>(0x80 | ((code_point >> 12) & 0x3F)));
            out.push_back(static_cast<char>(0x80 | ((code_point >> 6) & 0x3F)));
            out.push_back(static_cast<char>(0x80 | (code_point & 0x3F)));
        }
    }

    Status parseHex4(unsigned int& out) {
        if (cursor_ + 4 > text_.size()) {
            return error("truncated \\u escape");
        }
        unsigned int value = 0;
        for (int i = 0; i < 4; ++i) {
            const char c = text_[cursor_++];
            value <<= 4;
            if (c >= '0' && c <= '9') {
                value |= static_cast<unsigned int>(c - '0');
            } else if (c >= 'a' && c <= 'f') {
                value |= static_cast<unsigned int>(c - 'a' + 10);
            } else if (c >= 'A' && c <= 'F') {
                value |= static_cast<unsigned int>(c - 'A' + 10);
            } else {
                return error("invalid hex digit in \\u escape");
            }
        }
        out = value;
        return Status::okStatus();
    }

    Status parseString(std::string& out) {
        ++cursor_; // opening quote
        std::string value;
        while (true) {
            if (atEnd()) {
                return error("unterminated string");
            }
            const char c = text_[cursor_++];
            if (c == '"') {
                out = std::move(value);
                return Status::okStatus();
            }
            if (c != '\\') {
                if (static_cast<unsigned char>(c) < 0x20) {
                    return error("unescaped control character in string");
                }
                value.push_back(c);
                continue;
            }
            if (atEnd()) {
                return error("unterminated escape sequence");
            }
            const char escape = text_[cursor_++];
            switch (escape) {
                case '"': value.push_back('"'); break;
                case '\\': value.push_back('\\'); break;
                case '/': value.push_back('/'); break;
                case 'b': value.push_back('\b'); break;
                case 'f': value.push_back('\f'); break;
                case 'n': value.push_back('\n'); break;
                case 'r': value.push_back('\r'); break;
                case 't': value.push_back('\t'); break;
                case 'u': {
                    unsigned int code_point = 0;
                    Status status = parseHex4(code_point);
                    if (!status.ok()) {
                        return status;
                    }
                    if (code_point >= 0xD800 && code_point <= 0xDBFF && cursor_ + 1 < text_.size() &&
                        text_[cursor_] == '\\' && text_[cursor_ + 1] == 'u') {
                        cursor_ += 2;
                        unsigned int low = 0;
                        status = parseHex4(low);
                        if (!status.ok()) {
                            return status;
                        }
                        if (low >= 0xDC00 && low <= 0xDFFF) {
                            code_point = 0x10000 + ((code_point - 0xD800) << 10) + (low - 0xDC00);
                        } else {
                            appendUtf8(value, code_point);
                            code_point = low;
                        }
                    }
                    appendUtf8(value, code_point);
                    break;
                }
                default:
                    return error("unsupported escape sequence");
            }
        }
    }

    Status parseNumber(JsonValue& out) {
        const char* begin = text_.c_str() + cursor_;
        char* end = nullptr;
        errno = 0;
        const double value = std::strtod(begin, &end);
        if (end == begin) {
            return error("expected a value");
        }
        if (errno == ERANGE || !std::isfinite(value)) {
            return error("number is out of range");
        }
        cursor_ += static_cast<std::size_t>(end - begin);
        out = JsonValue(value);
        return Status::okStatus();
    }

    const std::string& text_;
    std::size_t cursor_{0};
};

} // namespace

JsonValue::JsonValue() = default;
JsonValue::JsonValue(bool value) : type_(Type::Boolean), boolean_(value) {}
JsonValue::JsonValue(double value) : type_(Type::Number), number_(value) {}
JsonValue::JsonValue(std::string value) : type_(Type::String), string_(std::move(value)) {}
JsonValue::JsonValue(JsonArray value)
    : type_(Type::Array), array_(std::make_shared<JsonArray>(std::move(value))) {}
JsonValue::JsonValue(JsonObject value)
    : type_(Type::Object), object_(std::make_shared<JsonObject>(std::move(value))) {}

bool JsonValue::asBoolean(bool fallback) const {
    return type_ == Type::Boolean ? boolean_ : fallback;
}

double JsonValue::asNumber(double fallback) const {
    return type_ == Type::Number ? number_ : fallback;
}

const std::string& JsonValue::asString() const {
    return type_ == Type::String ? string_ : emptyString();
}

const JsonArray& JsonValue::asArray() const {
    return type_ == Type::Array && array_ ? *array_ : emptyArray();
}

const JsonObject& JsonValue::asObject() const {
    return type_ == Type::Object && object_ ? *object_ : emptyObject();
}

const JsonValue* JsonValue::find(const std::string& key) const {
    if (type_ != Type::Object || !object_) {
        return nullptr;
    }
    const auto found = object_->find(key);
    return found == object_->end() ? nullptr : &found->second;
}

JsonParseResult parseJson(const std::string& text) {
    if (text.empty()) {
        return {{}, Status::error(ErrorCode::ParseError, "JSON document is empty")};
    }
    Parser parser(text);
    return parser.run();
}

} // namespace rozeta::internal
