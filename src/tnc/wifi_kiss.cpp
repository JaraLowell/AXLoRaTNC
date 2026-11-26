#include "wifi_kiss.h"

#if defined(AXLORA_VARIANT_STATION_G2)
#include <Preferences.h>
#include <errno.h>
#include <lwip/sockets.h>
#include "util/log.h"

namespace axlora::tnc {
namespace {

bool enableKeepalive(WiFiClient& client) {
  const struct {
    int level;
    int option;
    int value;
  } options[] = {
      {SOL_SOCKET, SO_KEEPALIVE, 1},
      {IPPROTO_TCP, TCP_KEEPIDLE, 120},
      {IPPROTO_TCP, TCP_KEEPINTVL, 30},
      {IPPROTO_TCP, TCP_KEEPCNT, 5},
  };
  for (const auto& option : options) {
    if (::setsockopt(client.fd(), option.level, option.option,
                     &option.value, sizeof(option.value)) != 0) {
      LOG_ERR("KISS TCP keepalive setup failed: option=%d errno=%d", option.option, errno);
      return false;
    }
  }
  return true;
}

bool clientConnected(WiFiClient& client) {
  // Drain WiFiClient's buffered bytes before checking EOF on the underlying socket.
  if (client.available() > 0) return true;
  if (client.fd() < 0) return false;
  uint8_t byte = 0;
  const int received = ::recv(client.fd(), &byte, 1, MSG_PEEK | MSG_DONTWAIT);
  if (received > 0) return true;
  if (received == 0) return false;
  if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) return true;
  LOG_WARN("KISS TCP receive failed: errno=%d", errno);
  return false;
}

}

void WifiKiss::begin() {
  WiFi.persistent(false);
  Preferences prefs;
  if (!prefs.begin("axlora_wifi", false)) {
    LOG_ERR("Wi-Fi settings unavailable; Wi-Fi remains off");
    return;
  }
  const uint8_t savedMode = prefs.getUChar("mode", 0);
  prefs.getString("ssid", ssid_, sizeof(ssid_));
  prefs.getString("password", password_, sizeof(password_));
  prefs.end();
  if (savedMode > static_cast<uint8_t>(Mode::AccessPoint)) {
    LOG_ERR("invalid saved Wi-Fi mode; Wi-Fi remains off");
    return;
  }
  mode_ = static_cast<Mode>(savedMode);
  if (!start()) LOG_ERR("Wi-Fi startup failed; configure it over USB");
}

bool WifiKiss::start() {
  if (mode_ == Mode::Off) return true;
  if (ssid_[0] == '\0' ||
      (mode_ == Mode::AccessPoint && strlen(password_) < 8)) {
    LOG_ERR("Wi-Fi requires an SSID; AP mode also requires an 8-63 byte password");
    return false;
  }
  if (!WiFi.mode(mode_ == Mode::Station ? WIFI_STA : WIFI_AP)) {
    LOG_ERR("Wi-Fi mode setup failed");
    return false;
  }
  if (mode_ == Mode::AccessPoint) {
    if (!WiFi.softAP(ssid_, password_)) {
      LOG_ERR("Wi-Fi access point startup failed");
      return false;
    }
  } else {
    WiFi.setAutoReconnect(true);
    if (WiFi.begin(ssid_, password_) == WL_CONNECT_FAILED) {
      LOG_ERR("Wi-Fi station startup failed");
      return false;
    }
    lastRetryMs_ = millis();
    LOG_INFO("Wi-Fi station connecting");
  }
  started_ = true;
  return true;
}

void WifiKiss::closeClient() {
  client_.stop();
  if (clientActive_) LOG_INFO("KISS TCP client disconnected");
  clientActive_ = false;
  decoder_.reset();
  pendingLen_ = 0;
}

void WifiKiss::stop() {
  closeClient();
  server_.end();
  networkReady_ = false;
  started_ = false;
  if (!WiFi.mode(WIFI_OFF)) LOG_ERR("Wi-Fi shutdown failed");
}

void WifiKiss::loop(bool allowClient) {
  if (mode_ == Mode::Off || !started_) return;
  const bool ready = mode_ == Mode::AccessPoint
      ? (WiFi.getMode() == WIFI_AP && WiFi.softAPIP() != IPAddress(0, 0, 0, 0))
      : WiFi.status() == WL_CONNECTED;
  if (!ready) {
    if (networkReady_) {
      LOG_WARN("Wi-Fi link lost");
      closeClient();
      server_.end();
      networkReady_ = false;
    }
    if (mode_ == Mode::Station && millis() - lastRetryMs_ >= 30000) {
      lastRetryMs_ = millis();
      LOG_WARN("Wi-Fi disconnected; retrying");
      if (!WiFi.reconnect()) LOG_WARN("Wi-Fi reconnect request failed");
    }
    return;
  }
  if (!networkReady_) {
    if (millis() - lastRetryMs_ < 1000) return;
    lastRetryMs_ = millis();
    server_.begin();
    if (!server_) {
      LOG_ERR("KISS TCP listener startup failed");
      return;
    }
    networkReady_ = true;
    const IPAddress ip = mode_ == Mode::AccessPoint ? WiFi.softAPIP() : WiFi.localIP();
    LOG_INFO("KISS TCP listening on %s:8100", ip.toString().c_str());
  }
  if (clientActive_ && !clientConnected(client_)) closeClient();
  WiFiClient incoming = server_.available();
  if (incoming) {
    if (clientActive_ || !allowClient) {
      LOG_WARN("KISS TCP client rejected: TNC busy");
      incoming.stop();
    } else if (!enableKeepalive(incoming)) {
      incoming.stop();
    } else {
      client_ = incoming;
      client_.setNoDelay(true);
      clientActive_ = true;
      decoder_.reset();
      pendingLen_ = 0;
      LOG_INFO("KISS TCP client connected");
    }
  }
  if (!clientActive_ || pendingLen_ == 0) return;
  // WiFiClient::write retries internally; use a nonblocking send so a stalled
  // PC cannot delay radio service or WA8DED polling.
  const size_t chunk = pendingLen_ > 768 ? 768 : pendingLen_;
  const int sent = ::send(client_.fd(), pending_, chunk, MSG_DONTWAIT);
  if (sent > 0) {
    pendingLen_ -= static_cast<size_t>(sent);
    memmove(pending_, pending_ + sent, pendingLen_);
    lastProgressMs_ = millis();
  } else if (sent < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
    LOG_WARN("KISS TCP send failed: errno=%d", errno);
    closeClient();
  } else if (millis() - lastProgressMs_ >= 3000) {
    LOG_WARN("KISS TCP send stalled; closing client");
    closeClient();
  }
}

bool WifiKiss::readFrame(ax25::KissFrame& frame) {
  if (!clientActive_) return false;
  for (size_t count = 0; count < 192 && client_.available() > 0; ++count) {
    const int byte = client_.read();
    if (byte < 0) break;
    if (decoder_.feed(static_cast<uint8_t>(byte), frame)) return true;
  }
  return false;
}

void WifiKiss::sendFrame(const uint8_t* data, size_t len) {
  if (!clientActive_) return;
  if (len > sizeof(pending_) - pendingLen_) {
    LOG_WARN("KISS TCP output queue full; closing client");
    closeClient();
    return;
  }
  if (pendingLen_ == 0) lastProgressMs_ = millis();
  memcpy(pending_ + pendingLen_, data, len);
  pendingLen_ += len;
}

bool WifiKiss::saveString(const char* key, const char* value) {
  Preferences prefs;
  if (!prefs.begin("axlora_wifi", false)) {
    Serial.println("ERROR: cannot open Wi-Fi settings");
    return false;
  }
  const bool saved = *value == '\0'
      ? (!prefs.isKey(key) || prefs.remove(key))
      : prefs.putString(key, value) == strlen(value);
  prefs.end();
  if (!saved) Serial.println("ERROR: cannot save Wi-Fi setting");
  return saved;
}

bool WifiKiss::saveMode(Mode mode) {
  Preferences prefs;
  if (!prefs.begin("axlora_wifi", false)) {
    Serial.println("ERROR: cannot open Wi-Fi settings");
    return false;
  }
  const bool saved = prefs.putUChar("mode", static_cast<uint8_t>(mode)) == 1;
  prefs.end();
  if (!saved) Serial.println("ERROR: cannot save Wi-Fi mode");
  return saved;
}

void WifiKiss::status() {
  const char* mode = mode_ == Mode::Station ? "sta" : mode_ == Mode::AccessPoint ? "ap" : "off";
  const IPAddress ip = mode_ == Mode::AccessPoint ? WiFi.softAPIP() : WiFi.localIP();
  Serial.printf("wifi mode=%s ready=%u ip=%s port=8100 client=%u heap=%u psram=%u\r\n",
                mode, networkReady_ ? 1U : 0U, ip.toString().c_str(),
                clientActive_ ? 1U : 0U, ESP.getFreeHeap(), ESP.getFreePsram());
}

void WifiKiss::command(const char* args) {
  if (*args == '\0' || strcmp(args, "status") == 0) {
    status();
    return;
  }
  if (strncmp(args, "ssid ", 5) == 0 || strncmp(args, "password ", 9) == 0) {
    if (mode_ != Mode::Off) {
      Serial.println("ERROR: use wifi off before changing credentials");
      return;
    }
    const bool password = strncmp(args, "password ", 9) == 0;
    const char* value = args + (password ? 9 : 5);
    const size_t len = strlen(value);
    if ((!password && (len == 0 || len > 32)) ||
        (password && (len < 8 || len > 63) && strcmp(value, "clear") != 0)) {
      Serial.println("ERROR: SSID must be 1-32 bytes; password 8-63 bytes or clear");
      return;
    }
    if (password && strcmp(value, "clear") == 0) value = "";
    if (!saveString(password ? "password" : "ssid", value)) return;
    strcpy(password ? password_ : ssid_, value);
    Serial.println(password ? "Wi-Fi password saved (hidden)" : "Wi-Fi SSID saved");
    return;
  }
  Mode selected;
  if (strcmp(args, "off") == 0) selected = Mode::Off;
  else if (strcmp(args, "sta") == 0) selected = Mode::Station;
  else if (strcmp(args, "ap") == 0) selected = Mode::AccessPoint;
  else {
    Serial.println("usage: wifi [status|off|sta|ap|ssid <name>|password <value>|password clear]");
    return;
  }
  if (selected != Mode::Off &&
      (ssid_[0] == '\0' || (selected == Mode::AccessPoint && strlen(password_) < 8))) {
    Serial.println("ERROR: configure SSID and password first (AP requires a password)");
    return;
  }
  if (!saveMode(selected)) return;
  stop();
  mode_ = selected;
  if (!start()) {
    Serial.println("ERROR: Wi-Fi startup failed; mode is saved for retry after reboot");
    return;
  }
  status();
}

}
#endif
