#pragma once

#if defined(AXLORA_VARIANT_STATION_G2)
#include <WiFi.h>
#include "ax25/ax25_kiss.h"

namespace axlora::tnc {

class WifiKiss {
 public:
  void begin();
  void loop(bool allowClient);
  void command(const char* args);
  bool active() const { return clientActive_; }
  bool readFrame(ax25::KissFrame& frame);
  void sendFrame(const uint8_t* data, size_t len);
  void closeClient();

 private:
  enum class Mode : uint8_t { Off, Station, AccessPoint };
  bool start();
  void stop();
  void status();
  bool saveString(const char* key, const char* value);
  bool saveMode(Mode mode);

  Mode mode_ = Mode::Off;
  char ssid_[33]{};
  char password_[64]{};
  WiFiServer server_{8100, 1};
  WiFiClient client_;
  bool clientActive_ = false;
  bool networkReady_ = false;
  bool started_ = false;
  uint32_t lastRetryMs_ = 0;
  ax25::KissDecoder decoder_;
  uint8_t pending_[4096]{};
  size_t pendingLen_ = 0;
  uint32_t lastProgressMs_ = 0;
};

}
#endif
