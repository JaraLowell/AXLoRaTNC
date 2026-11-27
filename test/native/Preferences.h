#pragma once
#include "Arduino.h"
#include <map>

enum PreferenceType { PT_INVALID, PT_U8, PT_U16, PT_U32, PT_FLOAT, PT_STR };

class Preferences {
 public:
  struct Value {
    double number = 0;
    std::string text;
    PreferenceType type = PT_INVALID;
  };
  inline static std::map<std::string, Value> values;
  inline static bool failBegin = false;
  std::string prefix;
  bool begin(const char* name, bool) {
    if (failBegin) return false;
    prefix = std::string(name) + ":";
    return true;
  }
  void end() {}
  bool isKey(const char* key) const { return values.count(prefix + key) != 0; }
  PreferenceType getType(const char* key) const {
    auto found = values.find(prefix + key);
    return found == values.end() ? PT_INVALID : found->second.type;
  }
  double getNumber(const char* key, double fallback) const {
    auto found = values.find(prefix + key);
    return found == values.end() ? fallback : found->second.number;
  }
  size_t putNumber(const char* key, double number, PreferenceType type, size_t size) {
    values[prefix + key] = {number, "", type}; return size;
  }
#define TEST_PREFERENCE(Name, Type, Kind) \
  Type get##Name(const char* key, Type fallback) const { \
    return static_cast<Type>(getNumber(key, fallback)); \
  } \
  size_t put##Name(const char* key, Type value) { \
    return putNumber(key, value, Kind, sizeof(Type)); \
  }
  TEST_PREFERENCE(Bool, bool, PT_U8)
  TEST_PREFERENCE(Char, int8_t, PT_U8)
  TEST_PREFERENCE(UChar, uint8_t, PT_U8)
  TEST_PREFERENCE(UShort, uint16_t, PT_U16)
  TEST_PREFERENCE(ULong, uint32_t, PT_U32)
  TEST_PREFERENCE(Float, float, PT_FLOAT)
#undef TEST_PREFERENCE
  size_t getString(const char* key, char* buffer, size_t capacity) const {
    auto found = values.find(prefix + key);
    if (found == values.end() || capacity == 0) return 0;
    strncpy(buffer, found->second.text.c_str(), capacity - 1);
    buffer[capacity - 1] = '\0';
    return strlen(buffer);
  }
  size_t putString(const char* key, const char* text) {
    values[prefix + key] = {0, text, PT_STR}; return strlen(text);
  }
};
