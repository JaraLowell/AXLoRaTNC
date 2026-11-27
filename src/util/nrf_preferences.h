#pragma once

#include <stddef.h>
#include <stdint.h>

enum PreferenceType { PT_INVALID, PT_U8, PT_U16, PT_U32, PT_FLOAT, PT_STR };

class Preferences {
 public:
  bool begin(const char* name, bool readOnly = false);
  void end();
  bool isKey(const char* key) const;
  PreferenceType getType(const char* key) const;
  bool getBool(const char* key, bool fallback) const;
  int8_t getChar(const char* key, int8_t fallback) const;
  uint8_t getUChar(const char* key, uint8_t fallback) const;
  uint16_t getUShort(const char* key, uint16_t fallback) const;
  uint32_t getULong(const char* key, uint32_t fallback) const;
  float getFloat(const char* key, float fallback) const;
  size_t getString(const char* key, char* buffer, size_t capacity) const;
  size_t putBool(const char* key, bool value);
  size_t putChar(const char* key, int8_t value);
  size_t putUChar(const char* key, uint8_t value);
  size_t putUShort(const char* key, uint16_t value);
  size_t putULong(const char* key, uint32_t value);
  size_t putFloat(const char* key, float value);
  size_t putString(const char* key, const char* value);

 private:
  bool pathFor(const char* key, char* path, size_t capacity) const;
  bool readValue(const char* key, PreferenceType type, void* value, size_t size) const;
  size_t writeValue(const char* key, PreferenceType type, const void* value, size_t size);
  char namespace_[16]{};
  bool open_ = false;
  bool readOnly_ = true;
};
