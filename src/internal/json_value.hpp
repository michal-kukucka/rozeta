#pragma once

// Minimal dependency-free JSON reader.
//
// Rozeta ships no third-party JSON library, and the only JSON the library reads
// is its own map catalog: objects, arrays, strings, numbers, booleans and null.
// This parser covers exactly that subset (including \uXXXX escapes, encoded as
// UTF-8) and reports a message with the byte offset on failure. It is internal:
// no public header exposes JsonValue.

#include <rozeta/core.hpp>

#include <map>
#include <memory>
#include <string>
#include <vector>

namespace rozeta::internal {

class JsonValue;

using JsonObject = std::map<std::string, JsonValue>;
using JsonArray = std::vector<JsonValue>;

class JsonValue {
public:
    enum class Type {
        Null,
        Boolean,
        Number,
        String,
        Array,
        Object,
    };

    JsonValue();
    explicit JsonValue(bool value);
    explicit JsonValue(double value);
    explicit JsonValue(std::string value);
    explicit JsonValue(JsonArray value);
    explicit JsonValue(JsonObject value);

    Type type() const { return type_; }
    bool isNull() const { return type_ == Type::Null; }
    bool isBoolean() const { return type_ == Type::Boolean; }
    bool isNumber() const { return type_ == Type::Number; }
    bool isString() const { return type_ == Type::String; }
    bool isArray() const { return type_ == Type::Array; }
    bool isObject() const { return type_ == Type::Object; }

    bool asBoolean(bool fallback = false) const;
    double asNumber(double fallback = 0.0) const;
    const std::string& asString() const;
    const JsonArray& asArray() const;
    const JsonObject& asObject() const;

    /// Object member lookup; returns nullptr when absent or not an object.
    const JsonValue* find(const std::string& key) const;

private:
    Type type_{Type::Null};
    bool boolean_{false};
    double number_{0.0};
    std::string string_{};
    std::shared_ptr<JsonArray> array_{};
    std::shared_ptr<JsonObject> object_{};
};

struct JsonParseResult {
    JsonValue value{};
    Status status{Status::okStatus()};

    bool ok() const { return status.ok(); }
};

JsonParseResult parseJson(const std::string& text);

} // namespace rozeta::internal
