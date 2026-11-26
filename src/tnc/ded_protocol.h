#pragma once

#include <ctype.h>
#include <stdio.h>
#include <string.h>
#include "ax25/ax25_frame.h"

namespace axlora::tnc::ded {

inline bool parseNumber(const char* text, unsigned maximum, unsigned& value,
                        const char** remainder = nullptr) {
  while (*text == ' ') ++text;
  if (*text < '0' || *text > '9') return false;
  value = 0;
  do {
    const unsigned digit = static_cast<unsigned>(*text++ - '0');
    if (value > maximum / 10 || (value == maximum / 10 && digit > maximum % 10)) return false;
    value = value * 10 + digit;
  } while (*text >= '0' && *text <= '9');
  while (*text == ' ') ++text;
  if (remainder != nullptr) *remainder = text;
  else if (*text != '\0') return false;
  return true;
}

inline bool parseCall(const char* text, ax25::Address& out, bool validate) {
  const char* dash = strchr(text, '-');
  const size_t length = dash == nullptr ? strlen(text) : static_cast<size_t>(dash - text);
  if (length == 0 || length > ax25::CALLSIGN_LEN) return false;
  unsigned digits = 0;
  size_t lastDigit = 0;
  for (size_t i = 0; i < length; ++i) {
    if (!isalnum(static_cast<unsigned char>(text[i]))) return false;
    if (isdigit(static_cast<unsigned char>(text[i]))) { ++digits; lastDigit = i; }
  }
  if (validate && (length < 4 || digits == 0 || digits > 2 ||
                   isdigit(static_cast<unsigned char>(text[0])) || lastDigit + 1 == length))
    return false;
  if (dash != nullptr) {
    unsigned ssid = 0;
    if (!parseNumber(dash + 1, 15, ssid)) return false;
  }
  return ax25::parseAddress(text, out);
}

inline size_t monitorHeader(const ax25::Frame& frame, char* output, size_t capacity) {
  size_t length = 0;
  auto append = [&](const char* text) {
    while (*text != '\0' && length + 1 < capacity) output[length++] = *text++;
  };
  char text[32]{};
  append("fm ");
  ax25::formatAddress(frame.source, text, sizeof(text)); append(text);
  append(" to ");
  ax25::formatAddress(frame.destination, text, sizeof(text)); append(text);
  int lastRepeated = -1;
  for (uint8_t i = 0; i < frame.repeaterCount; ++i)
    if (frame.repeaters[i].repeated) lastRepeated = i;
  for (uint8_t i = 0; i < frame.repeaterCount; ++i) {
    append(i == 0 ? " via " : " ");
    ax25::formatAddress(frame.repeaters[i], text, sizeof(text)); append(text);
    if (i == lastRepeated) append("*");
  }
  append(" ctl ");
  const ax25::FrameKind kind = ax25::kind(frame.control);
  if (kind == ax25::FrameKind::I) {
    snprintf(text, sizeof(text), "I%u%u", ax25::nr(frame.control), ax25::ns(frame.control));
    append(text);
  } else if (kind == ax25::FrameKind::S) {
    const char* names[] = {"RR", "RNR", "REJ", "SREJ"};
    snprintf(text, sizeof(text), "%s%u", names[(frame.control >> 2) & 3], ax25::nr(frame.control));
    append(text);
  } else {
    const char* name = nullptr;
    switch (ax25::uType(frame.control)) {
      case ax25::UFrameType::UI: name = "UI"; break;
      case ax25::UFrameType::DM: name = "DM"; break;
      case ax25::UFrameType::SABM: name = "SABM"; break;
      case ax25::UFrameType::DISC: name = "DISC"; break;
      case ax25::UFrameType::UA: name = "UA"; break;
      case ax25::UFrameType::FRMR: name = "FRMR"; break;
      default: break;
    }
    if (name != nullptr) append(name);
    else { snprintf(text, sizeof(text), "?%02XH", frame.control); append(text); }
    if (ax25::uType(frame.control) == ax25::UFrameType::FRMR) {
      for (size_t i = 0; i < frame.infoLen; ++i) {
        snprintf(text, sizeof(text), "%02X", frame.info[i]); append(text);
      }
    }
  }
  if (frame.control & 0x10) append(frame.version2 ? (frame.command ? "+" : "-") : "!");
  else if (frame.version2) append(frame.command ? "^" : "v");
  if (kind == ax25::FrameKind::I || ax25::uType(frame.control) == ax25::UFrameType::UI) {
    snprintf(text, sizeof(text), " pid %02X", frame.pid); append(text);
  }
  if (frame.dama) append(" [DAMA]");
  if (capacity > 0) output[length] = '\0';
  return length;
}

}
