#pragma once

#include <algorithm>
#include <map>
#include <string>
#include <vector>
#include <stdint.h>
#include <string.h>

class Adafruit_LittleFS {
 public:
  bool begin() { return true; }
};

class InternalFileSystem : public Adafruit_LittleFS {
 public:
  std::map<std::string, std::vector<uint8_t>> files;
  bool failRename = false;
  bool exists(const char* path) { return files.count(path) != 0; }
  bool remove(const char* path) { return files.erase(path) != 0; }
  bool rename(const char* from, const char* to) {
    if (failRename || !exists(from)) return false;
    files[to] = files[from];
    files.erase(from);
    return true;
  }
};

inline InternalFileSystem InternalFS;

namespace Adafruit_LittleFS_Namespace {
enum { FILE_O_READ, FILE_O_WRITE };
class File {
 public:
  explicit File(InternalFileSystem& fs) : fs_(fs) {}
  bool open(const char* path, int mode) {
    if (mode == FILE_O_READ && !fs_.exists(path)) return false;
    path_ = path;
    pos_ = mode == FILE_O_WRITE ? fs_.files[path_].size() : 0;
    return true;
  }
  int read() {
    auto& data = fs_.files[path_];
    return pos_ < data.size() ? data[pos_++] : -1;
  }
  int read(void* output, uint16_t count) {
    auto& data = fs_.files[path_];
    const size_t available = data.size() - pos_;
    const size_t n = std::min(static_cast<size_t>(count), available);
    memcpy(output, data.data() + pos_, n);
    pos_ += n;
    return static_cast<int>(n);
  }
  size_t write(const uint8_t* input, size_t count) {
    auto& data = fs_.files[path_];
    data.insert(data.end(), input, input + count);
    pos_ += count;
    return count;
  }
  size_t size() { return fs_.files[path_].size(); }
  void close() {}
 private:
  InternalFileSystem& fs_;
  std::string path_;
  size_t pos_ = 0;
};
}
