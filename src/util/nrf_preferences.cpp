#if defined(NRF52_PLATFORM)

#include "nrf_preferences.h"
#include <InternalFileSystem.h>
#include <stdio.h>
#include <string.h>
#include "util/log.h"

namespace {
using Adafruit_LittleFS_Namespace::FILE_O_READ;
using Adafruit_LittleFS_Namespace::FILE_O_WRITE;
using Adafruit_LittleFS_Namespace::File;

bool mountSettings() {
  static bool mounted = false;
  if (mounted) return true;
  mounted = InternalFS.Adafruit_LittleFS::begin();
  if (mounted) return true;

  // InternalFS.begin() reformats on any mount failure; only allow that for
  // an entirely erased settings partition.
  const volatile uint32_t* flash = reinterpret_cast<const volatile uint32_t*>(0xED000);
  for (size_t i = 0; i < (7 * 4096) / sizeof(uint32_t); ++i) {
    if (flash[i] != 0xFFFFFFFFu) {
      LOG_ERR("settings filesystem mount failed; refusing to erase existing data");
      return false;
    }
  }
  mounted = InternalFS.begin();
  if (!mounted) LOG_ERR("settings filesystem initialization failed");
  return mounted;
}
}

bool Preferences::begin(const char* name, bool readOnly) {
  if (name == nullptr || name[0] == '\0' || strlen(name) >= sizeof(namespace_) ||
      strchr(name, '/') != nullptr || !mountSettings()) return false;
  strcpy(namespace_, name);
  readOnly_ = readOnly;
  open_ = true;
  return true;
}

void Preferences::end() { open_ = false; }

bool Preferences::pathFor(const char* key, char* path, size_t capacity) const {
  if (!open_ || key == nullptr || key[0] == '\0' || strchr(key, '/') != nullptr) return false;
  const int count = snprintf(path, capacity, "/%s_%s", namespace_, key);
  return count > 0 && static_cast<size_t>(count) < capacity;
}

bool Preferences::isKey(const char* key) const {
  char path[40]{};
  return pathFor(key, path, sizeof(path)) && InternalFS.exists(path);
}

PreferenceType Preferences::getType(const char* key) const {
  char path[40]{};
  if (!pathFor(key, path, sizeof(path)) || !InternalFS.exists(path)) return PT_INVALID;
  File file(InternalFS);
  if (!file.open(path, FILE_O_READ)) {
    LOG_ERR("cannot open setting %s", key);
    return PT_INVALID;
  }
  const int type = file.read();
  file.close();
  if (type <= PT_INVALID || type > PT_STR) {
    LOG_ERR("invalid setting type for %s", key);
    return PT_INVALID;
  }
  return static_cast<PreferenceType>(type);
}

bool Preferences::readValue(const char* key, PreferenceType type, void* value, size_t size) const {
  char path[40]{};
  if (!pathFor(key, path, sizeof(path)) || !InternalFS.exists(path)) return false;
  File file(InternalFS);
  if (!file.open(path, FILE_O_READ)) {
    LOG_ERR("cannot open setting %s", key);
    return false;
  }
  const int savedType = file.read();
  const bool valid = savedType == type && file.size() == size + 1 &&
                     file.read(value, static_cast<uint16_t>(size)) == static_cast<int>(size);
  file.close();
  if (!valid) LOG_ERR("invalid saved setting %s", key);
  return valid;
}

size_t Preferences::writeValue(const char* key, PreferenceType type, const void* value, size_t size) {
  char path[40]{};
  if (readOnly_ || value == nullptr || !pathFor(key, path, sizeof(path))) {
    LOG_ERR("invalid setting write");
    return 0;
  }
  const char* temporary = "/axlora_tmp";
  if (InternalFS.exists(temporary) && !InternalFS.remove(temporary)) {
    LOG_ERR("cannot remove temporary setting");
    return 0;
  }
  File file(InternalFS);
  if (!file.open(temporary, FILE_O_WRITE)) {
    LOG_ERR("cannot create temporary setting");
    return 0;
  }
  const uint8_t typeByte = static_cast<uint8_t>(type);
  const bool written = file.write(&typeByte, 1) == 1 &&
                       file.write(static_cast<const uint8_t*>(value), size) == size;
  file.close();
  bool verified = written && file.open(temporary, FILE_O_READ);
  if (verified) {
    verified = file.size() == size + 1 && file.read() == typeByte;
    const uint8_t* expected = static_cast<const uint8_t*>(value);
    uint8_t buffer[32]{};
    for (size_t pos = 0; verified && pos < size;) {
      const size_t count = size - pos < sizeof(buffer) ? size - pos : sizeof(buffer);
      verified = file.read(buffer, static_cast<uint16_t>(count)) == static_cast<int>(count) &&
                 memcmp(buffer, expected + pos, count) == 0;
      pos += count;
    }
    file.close();
  }
  if (!verified || !InternalFS.rename(temporary, path)) {
    LOG_ERR("cannot save setting %s", key);
    return 0;
  }
  return size;
}

#define NRF_PREFERENCE(Name, Type, Kind) \
  Type Preferences::get##Name(const char* key, Type fallback) const { \
    Type value{}; return readValue(key, Kind, &value, sizeof(value)) ? value : fallback; \
  } \
  size_t Preferences::put##Name(const char* key, Type value) { \
    return writeValue(key, Kind, &value, sizeof(value)); \
  }
NRF_PREFERENCE(Bool, bool, PT_U8)
NRF_PREFERENCE(Char, int8_t, PT_U8)
NRF_PREFERENCE(UChar, uint8_t, PT_U8)
NRF_PREFERENCE(UShort, uint16_t, PT_U16)
NRF_PREFERENCE(ULong, uint32_t, PT_U32)
NRF_PREFERENCE(Float, float, PT_FLOAT)
#undef NRF_PREFERENCE

size_t Preferences::getString(const char* key, char* buffer, size_t capacity) const {
  char path[40]{};
  if (buffer == nullptr || capacity == 0 || !pathFor(key, path, sizeof(path)) ||
      !InternalFS.exists(path)) return 0;
  File file(InternalFS);
  if (!file.open(path, FILE_O_READ)) {
    LOG_ERR("cannot open setting %s", key);
    return 0;
  }
  const int type = file.read();
  const size_t length = file.size() > 0 ? file.size() - 1 : 0;
  if (type != PT_STR || length >= capacity ||
      file.read(buffer, static_cast<uint16_t>(length)) != static_cast<int>(length)) {
    file.close();
    LOG_ERR("invalid saved string %s", key);
    return 0;
  }
  file.close();
  buffer[length] = '\0';
  return length + 1;
}

size_t Preferences::putString(const char* key, const char* value) {
  return value == nullptr ? 0 : writeValue(key, PT_STR, value, strlen(value));
}

#endif
