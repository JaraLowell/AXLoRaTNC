#include "tnc.h"
#include <Arduino.h>
#include <stdio.h>
#include <string.h>
#include "ded_protocol.h"

namespace axlora::tnc {
namespace {

uint8_t monthDays(uint8_t month, uint8_t year) {
  static constexpr uint8_t days[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
  return month == 2 && year % 4 == 0 ? 29 : days[month - 1];
}

}

bool Tnc::parseDedPath(const char* value, ax25::Address& destination,
                       ax25::Address* path, uint8_t& count, bool validate) const {
  char text[257]{};
  if (strlen(value) >= sizeof(text)) return false;
  strcpy(text, value);
  char* token = strtok(text, " ,");
  if (token == nullptr || !ded::parseCall(token, destination, validate)) return false;
  count = 0;
  while ((token = strtok(nullptr, " ,")) != nullptr) {
    if (strcasecmp(token, "via") == 0 && count == 0) continue;
    if (count == ax25::MAX_REPEATERS || !ded::parseCall(token, path[count], dedValidateCallsign_))
      return false;
    ++count;
  }
  return true;
}

bool Tnc::setDedMonitor(const char* value) {
  char text[257]{};
  if (strlen(value) >= sizeof(text)) return false;
  strcpy(text, value);
  for (char* p = text; *p != '\0'; ++p) *p = static_cast<char>(toupper(*p));
  unsigned flags = 0;
  bool none = false;
  char filter = dedMonitorCallFilter_;
  ax25::Address calls[ax25::MAX_REPEATERS]{};
  memcpy(calls, dedMonitorCalls_, sizeof(calls));
  uint8_t count = dedMonitorCallCount_;
  char* p = text;
  while (*p != '\0') {
    const char c = *p++;
    if (c == ' ' || c == ',') continue;
    if (c == '+' || c == '-') {
      filter = c;
      count = 0;
      char* token = strtok(p, " ,");
      while (token != nullptr) {
        if (count == ax25::MAX_REPEATERS || !ded::parseCall(token, calls[count], false))
          return false;
        ++count;
        token = strtok(nullptr, " ,");
      }
      if (count == 0) filter = '\0';
      break;
    }
    if (c == 'N') {
      if (flags != 0) return false;
      none = true;
      continue;
    }
    if (none) return false;
    switch (c) {
      case 'I': flags |= 1; break;
      case 'U': flags |= 2; break;
      case 'S': flags |= 4; break;
      case 'C': flags |= 8; break;
      default: return false;
    }
  }
  if (none || flags != 0) {
    size_t pos = 0;
    if (flags & 1) dedMonitorMode_[pos++] = 'I';
    if (flags & 2) dedMonitorMode_[pos++] = 'U';
    if (flags & 4) dedMonitorMode_[pos++] = 'S';
    if (flags & 8) dedMonitorMode_[pos++] = 'C';
    if ((flags & 7) == 0) dedMonitorMode_[pos++] = 'N';
    dedMonitorMode_[pos] = '\0';
    monitorEnabled_ = (flags & 7) != 0;
  }
  dedMonitorCallFilter_ = filter;
  dedMonitorCallCount_ = count;
  memcpy(dedMonitorCalls_, calls, sizeof(calls));
  return true;
}

bool Tnc::dedMonitorAllows(const ax25::Frame& frame) const {
  if (!monitorEnabled_) return false;
  if (!dedHostMode_ && dedSelectedChannel_ != 0 &&
      channels_[dedSelectedChannel_ - 1].link.state() != ax25::LinkState::Disconnected &&
      strchr(dedMonitorMode_, 'C') == nullptr) return false;
  const char type = ax25::kind(frame.control) == ax25::FrameKind::I ? 'I'
      : (ax25::kind(frame.control) == ax25::FrameKind::U &&
         ax25::uType(frame.control) == ax25::UFrameType::UI ? 'U' : 'S');
  if (strchr(dedMonitorMode_, type) == nullptr) return false;
  if (dedMonitorCallFilter_ == '\0') return true;
  bool match = false;
  for (uint8_t i = 0; i < dedMonitorCallCount_; ++i) {
    if (strcmp(dedMonitorCalls_[i].callsign, frame.source.callsign) == 0 ||
        strcmp(dedMonitorCalls_[i].callsign, frame.destination.callsign) == 0) match = true;
  }
  return dedMonitorCallFilter_ == '+' ? match : !match;
}

void Tnc::formatDedTimestamp(char* output, size_t capacity) {
  const uint32_t now = millis();
  const uint32_t elapsed = now - dedClockMs_;
  dedClockMs_ += (elapsed / 1000) * 1000;
  dedClockSeconds_ += elapsed / 1000;
  while (dedClockSeconds_ >= 86400) {
    dedClockSeconds_ -= 86400;
    if (++dedClockDay_ > monthDays(dedClockMonth_, dedClockYear_)) {
      dedClockDay_ = 1;
      if (++dedClockMonth_ > 12) {
        dedClockMonth_ = 1;
        dedClockYear_ = (dedClockYear_ + 1) % 100;
      }
    }
  }
  snprintf(output, capacity, "%02u%c%02u%c%02u %02lu:%02lu:%02lu",
           dedClockEuropean_ ? dedClockDay_ : dedClockMonth_,
           dedClockEuropean_ ? '.' : '/',
           dedClockEuropean_ ? dedClockMonth_ : dedClockDay_,
           dedClockEuropean_ ? '.' : '/', dedClockYear_,
           static_cast<unsigned long>(dedClockSeconds_ / 3600),
           static_cast<unsigned long>((dedClockSeconds_ / 60) % 60),
           static_cast<unsigned long>(dedClockSeconds_ % 60));
}

bool Tnc::setDedTimestamp(const char* value) {
  unsigned a = 0, b = 0, c = 0;
  char first = '\0', second = '\0', extra = '\0';
  if (sscanf(value, "%u%c%u%c%u%c", &a, &first, &b, &second, &c, &extra) != 5 ||
      first != second) return false;
  if (first == ':') {
    if (a > 23 || b > 59 || c > 59) return false;
    dedClockSeconds_ = a * 3600 + b * 60 + c;
    dedClockMs_ = millis();
    return true;
  }
  if (first != '.' && first != '/') return false;
  const unsigned day = first == '.' ? a : b;
  const unsigned month = first == '.' ? b : a;
  if (c > 99 || month < 1 || month > 12 || day < 1 ||
      day > monthDays(static_cast<uint8_t>(month), static_cast<uint8_t>(c))) return false;
  char previous[24]{};
  formatDedTimestamp(previous, sizeof(previous));
  dedClockDay_ = static_cast<uint8_t>(day);
  dedClockMonth_ = static_cast<uint8_t>(month);
  dedClockYear_ = static_cast<uint8_t>(c);
  dedClockEuropean_ = first == '.';
  return true;
}

}
