#include "tnc.h"
#include <Arduino.h>
#if defined(NRF52_PLATFORM)
#include "util/nrf_preferences.h"
#else
#include <Preferences.h>
#include <esp_system.h>
#endif
#include <stdlib.h>
#include <string.h>
#include "aprs/aprs.h"
#include "ax25/ax25_fcs.h"
#include "ax25/ax25_frame.h"
#include "util/crc.h"
#include "util/log.h"
#include "util/timer.h"
#include "ded_protocol.h"
#if defined(AXLORA_VARIANT_SENSECAP_SOLAR)
#include "variant/sensecap_solar_power.h"
#endif

#ifndef AXLORATNC_TRACE_LINFBB
#define AXLORATNC_TRACE_LINFBB 0
#endif

#if AXLORATNC_TRACE_LINFBB
#define LINFBB_TRACE(...) LOG_INFO(__VA_ARGS__)
#else
#define LINFBB_TRACE(...) do {} while (0)
#endif

namespace axlora::tnc {
namespace {

#if defined(AXLORA_VARIANT_SENSECAP_SOLAR)
#if defined(NRF52_PLATFORM)
// millis() wraps when the 1024 Hz tick counter wraps, before reaching UINT32_MAX.
static constexpr uint32_t RELAY_TICKS_PER_SECOND = configTICK_RATE_HZ;
static uint32_t relayUptimeTick() { return xTaskGetTickCount(); }
#else
static constexpr uint32_t RELAY_TICKS_PER_SECOND = 1000;
static uint32_t relayUptimeTick() { return millis(); }
#endif
#endif

#if defined(AXLORA_VARIANT_STATION_G2)
static constexpr int FREQUENCY_DECIMALS = 4;
#else
static constexpr int FREQUENCY_DECIMALS = 3;
#endif

static void formatUptime(uint32_t ms, char* buf, size_t cap) {
  const uint32_t s = ms / 1000;
  snprintf(buf, cap, "%lu:%02lu:%02lu",
           static_cast<unsigned long>(s / 3600),
           static_cast<unsigned long>((s % 3600) / 60),
           static_cast<unsigned long>(s % 60));
}

void addressToText(const ax25::Address& address, char* out, size_t outCap) {
  ax25::formatAddress(address, out, outCap);
}

bool textToAddress(const char* text, ax25::Address& out) {
  return text != nullptr && text[0] != '\0' && ax25::parseAddress(text, out);
}

static uint8_t sanitizeNodeTextByte(uint8_t byte) {
  if (byte == '\r' || byte == '\n' || byte == '\t') return byte;
  if (byte >= 0x20 && byte <= 0x7E) return byte;
  return '?';
}

static bool isNodeInputTextByte(uint8_t byte) {
  return byte == '\t' || (byte >= 0x20 && byte <= 0x7E);
}

static void copySanitizedNodeText(char* dst, size_t dstCap, const char* src) {
  if (dst == nullptr || dstCap == 0) return;
  size_t pos = 0;
  if (src != nullptr) {
    while (*src != '\0' && pos + 1 < dstCap) {
      dst[pos++] = static_cast<char>(sanitizeNodeTextByte(static_cast<uint8_t>(*src++)));
    }
  }
  dst[pos] = '\0';
}

static bool serialWriteAll(const uint8_t* data, size_t len, uint32_t timeoutMs = 2000) {
  if (data == nullptr) return false;
  size_t pos = 0;
  uint32_t lastProgressMs = millis();
  while (pos < len) {
    const int writable = Serial.availableForWrite();
    if (writable > 0) {
      const size_t chunk = (len - pos) < static_cast<size_t>(writable)
          ? (len - pos)
          : static_cast<size_t>(writable);
      const size_t written = Serial.write(data + pos, chunk);
      if (written > 0) {
        pos += written;
        lastProgressMs = millis();
        continue;
      }
    }
    if (static_cast<uint32_t>(millis() - lastProgressMs) > timeoutMs) {
      return false;
    }
    delay(1);
  }
  return true;
}

bool isUiFrame(const ax25::Frame& frame) {
  return ax25::kind(frame.control) == ax25::FrameKind::U &&
         ax25::uType(frame.control) == ax25::UFrameType::UI;
}

static const char* skipSpaces(const char* s) {
  while (s != nullptr && *s == ' ') ++s;
  return s;
}

static bool parseByteValue(const char* s, uint8_t& out) {
  s = skipSpaces(s);
  if (s == nullptr || *s == '\0') return false;
  char* end = nullptr;
  const long value = strtol(s, &end, 10);
  if (end == s || value < 0 || value > 255) return false;
  out = static_cast<uint8_t>(value);
  return true;
}

static void copyFormattedAddress(const ax25::Address& address, char* out, size_t cap) {
  ax25::formatAddress(address, out, cap);
  if (cap > 0) out[cap - 1] = '\0';
}

static bool isValidBaud(uint32_t baud) {
  static constexpr uint32_t valid[] = {
    300, 1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600
  };
  for (auto v : valid) { if (baud == v) return true; }
  return false;
}


}  // namespace

// ---------------------------------------------------------------------------
// Lifecycle
// ---------------------------------------------------------------------------

void Tnc::begin(const char* callsign) {
#if defined(AXLORA_VARIANT_SENSECAP_SOLAR)
  relayLastUptimeTicks_ = relayUptimeTick();
  relayUptimeTicks_ = relayLastUptimeTicks_;
#endif
  loadSettings();
  applySerialBaud();
  mailbox_.begin();
  if (!rawTxQueue_.begin(RAW_TX_QUEUE_CAPACITY, RAW_TX_QUEUE_FALLBACK_CAPACITY)) {
    LOG_WARN("radio tx queue allocation failed");
  } else if (rawTxQueue_.capacity() < RAW_TX_QUEUE_CAPACITY) {
    LOG_WARN("radio tx queue fallback capacity=%u", static_cast<unsigned>(rawTxQueue_.capacity()));
  }
  if (!rawTxPriorityQueue_.begin(RAW_TX_PRIORITY_QUEUE_CAPACITY, RAW_TX_PRIORITY_QUEUE_FALLBACK_CAPACITY)) {
    LOG_WARN("radio priority tx queue allocation failed");
  } else if (rawTxPriorityQueue_.capacity() < RAW_TX_PRIORITY_QUEUE_CAPACITY) {
    LOG_WARN("radio priority tx queue fallback capacity=%u",
             static_cast<unsigned>(rawTxPriorityQueue_.capacity()));
  }
  // Prefer NVS callsign over compile-time default
  const char* cs = (savedCallsign_[0] != '\0') ? savedCallsign_ : callsign;
  if (!ax25::parseAddress(cs, local_)) {
    ax25::parseAddress("N0CALL", local_);
  }
  ax25::formatAddress(local_, dedLastCallsign_, sizeof(dedLastCallsign_));
  if (beacon_.destination.callsign[0] == '\0') {
    ax25::parseAddress("CQ", beacon_.destination);
  }
  if (dedUnprotoDestination_.callsign[0] == '\0') {
    ax25::parseAddress("CQ", dedUnprotoDestination_);
  }
  if (beacon_.text[0] == '\0') {
    strncpy(beacon_.text, "AXLoRaTNC LoRa AX.25", sizeof(beacon_.text) - 1);
  }
  if (netrom_.alias[0] == '\0') {
    memcpy(netrom_.alias, "AXLORA", 6);
    netrom_.alias[6] = '\0';
  }
  if (netrom_.ident[0] == '\0') {
    strncpy(netrom_.ident, "AXLoRaTNC NET/ROM node", sizeof(netrom_.ident) - 1);
  }
  for (uint8_t i = 0; i < CHANNEL_COUNT; ++i) {
    channelCtx_[i].tnc   = this;
    channelCtx_[i].chIdx = i;
    beginLink(i, local_);
  }
  dedClockMs_ = millis();
#if defined(AXLORA_VARIANT_STATION_G2)
  wifiKiss_.begin();
#endif
}

void Tnc::beginLink(uint8_t chIdx, const ax25::Address& local) {
  if (chIdx >= CHANNEL_COUNT) return;
  ax25::L2Config cfg{};
  cfg.local = local;
  cfg.t1Ms = static_cast<uint32_t>(dedFrack_) * 10UL;
  cfg.t2Ms = static_cast<uint32_t>(dedT2_) * 10UL;
  cfg.t3Ms = static_cast<uint32_t>(dedT3_) * 10UL;
  cfg.n2 = dedRetryLimit_;
  cfg.iPollLength = dedIPollFrameLength_;
  channels_[chIdx].link.begin(cfg, radioTxCallback, dataCallback, &channelCtx_[chIdx],
                             dataReadyCallback);
  channels_[chIdx].link.setMaxFrame(dedMaxFrame_);
  channels_[chIdx].initialRtt = dedFrack_;
  channels_[chIdx].retryLimit = dedRetryLimit_;
  channels_[chIdx].maxFrame = dedMaxFrame_;
}

void Tnc::resetLinksForLocal() {
  loopbackTxQueue_.clear();
  for (uint8_t i = 0; i < CHANNEL_COUNT; ++i) {
    channels_[i].loopbackPeer = CHANNEL_COUNT;
    beginLink(i, local_);
  }
}

bool Tnc::setChannelLocal(uint8_t chIdx, const ax25::Address& local) {
  if (chIdx >= CHANNEL_COUNT || channels_[chIdx].loopbackPeer < CHANNEL_COUNT) return false;
  return channels_[chIdx].link.setLocalAddress(local);
}

void Tnc::loop(bool radioReady) {
#if defined(AXLORA_VARIANT_SENSECAP_SOLAR)
  updateRelayUptime();
#endif
  if (!radioReady) radioConfigApplied_ = false;
  ++dedRoundsCount_;
  const uint32_t roundsElapsed = millis() - dedRoundsStartMs_;
  if (roundsElapsed >= 1000) {
    dedRoundsPerSecond_ = static_cast<uint32_t>(
        static_cast<uint64_t>(dedRoundsCount_) * 1000 / roundsElapsed);
    dedRoundsCount_ = 0;
    dedRoundsStartMs_ = millis();
  }
  serviceSerial();
#if defined(AXLORA_VARIANT_STATION_G2)
  bool allowTcpClient = serialMode_ == SerialMode::Console;
  for (const ChannelState& channel : channels_) {
    if (channel.link.state() != ax25::LinkState::Disconnected) allowTcpClient = false;
  }
  wifiKiss_.loop(allowTcpClient);
  for (uint8_t count = 0; count < 4 && wifiKiss_.active(); ++count) {
    if (rawTxQueue_.free() <= KISS_SERIAL_TXQ_HEADROOM &&
        rawTxPriorityQueue_.free() == 0) break;
    ax25::KissFrame frame{};
    if (!wifiKiss_.readFrame(frame)) break;
    handleKiss(frame, true);
  }
#endif
  serviceLoopbackTx();
  if (radioReady) {
    if (!radioConfigApplied_) {
      applyRadioConfig();
      radioConfigApplied_ = true;
    }
    serviceRadio();
  }
  for (uint8_t i = 0; i < CHANNEL_COUNT; ++i) {
    channels_[i].link.loop();
  }
  checkLinkStatusEvents();
#if defined(AXLORA_VARIANT_SENSECAP_SOLAR)
  if (serialMode_ == SerialMode::Console) {
    for (uint8_t i = 0; i < CHANNEL_COUNT; ++i) serviceRelayResponses(i);
  }
#endif
  if (serialMode_ == SerialMode::Wa8ded && !dedHostMode_) {
    serviceDedTerminalOutput();
  }
  if (radioReady) {
    serviceBeacon(radioReady);
    serviceNetrom(radioReady);
    serviceRawTx(radioReady);
  }
}

// ---------------------------------------------------------------------------
// Radio TX / RX
// ---------------------------------------------------------------------------

bool Tnc::isChannelBusy() const {
  if (lastRxMs_ == 0 || kissParams_.fullDuplex != 0) return false;
  const uint32_t windowMs = static_cast<uint32_t>(kissParams_.slotTime) * 10;
  return !axlora::util::elapsed(axlora::util::nowMs(), lastRxMs_, windowMs < 100 ? 100 : windowMs);
}

bool Tnc::transmitRaw(const uint8_t* data, size_t len, uint8_t chIdx) {
  if (chIdx < CHANNEL_COUNT && channels_[chIdx].loopbackPeer < CHANNEL_COUNT) {
    ax25::Frame frame{};
    if (data != nullptr && len <= MAX_PACKET_BYTES &&
        ax25::decodeFrame(data, len, frame, true) &&
        !isUiFrame(frame) &&
        ax25::addressEquals(frame.destination,
                            channels_[channels_[chIdx].loopbackPeer].link.local())) {
      LoopbackTx tx{};
      memcpy(tx.data, data, len);
      tx.len = len;
      tx.source = chIdx;
      tx.destination = channels_[chIdx].loopbackPeer;
      if (!loopbackTxQueue_.push(tx)) {
        LOG_WARN("loopback tx queue full");
        return false;
      }
      return true;
    }
  }
  if (!dedTxEnabled_) {
    return false;
  }
  const uint32_t delayMs = (kissParams_.fullDuplex == 0)
      ? static_cast<uint32_t>(kissParams_.txDelay) * 10
      : 0;
  return enqueueRawTx(data, len, delayMs, chIdx);
}

void Tnc::serviceLoopbackTx() {
  const size_t count = loopbackTxQueue_.size();
  for (size_t i = 0; i < count; ++i) {
    LoopbackTx tx{};
    if (!loopbackTxQueue_.pop(tx)) break;
    if (channels_[tx.source].loopbackPeer != tx.destination ||
        channels_[tx.destination].loopbackPeer != tx.source) continue;
    channels_[tx.source].link.notifyTxSent();
    channels_[tx.destination].link.receive(tx.data, tx.len);
  }
  if (!loopbackTxQueue_.empty()) return;
  for (uint8_t i = 0; i < CHANNEL_COUNT; ++i) {
    const uint8_t peer = channels_[i].loopbackPeer;
    if (peer < CHANNEL_COUNT &&
        channels_[i].link.state() == ax25::LinkState::Disconnected &&
        channels_[peer].link.state() == ax25::LinkState::Disconnected) {
      channels_[i].loopbackPeer = CHANNEL_COUNT;
      channels_[peer].loopbackPeer = CHANNEL_COUNT;
    }
  }
}

bool Tnc::enqueueRawTx(const uint8_t* data, size_t len, uint32_t delayMs, uint8_t chIdx) {
  if (data == nullptr || len == 0 || len > MAX_PACKET_BYTES) {
    ++rawTxQueueDrops_;
    return false;
  }

  PendingRawTx tx{};
  memcpy(tx.data, data, len);
  tx.len = len;
  tx.notBeforeMs = axlora::util::nowMs() + delayMs;
  tx.chIdx = chIdx;
  tx.id = rawTxNextId_++;
  tx.priority = rawFramePriority(data, len);

  RawTxQueue& queue = tx.priority ? rawTxPriorityQueue_ : rawTxQueue_;
  if (!queue.push(tx)) {
    ++rawTxQueueDrops_;
    LOG_WARN("radio tx %squeue full", tx.priority ? "priority " : "");
    LINFBB_TRACE("TXQ_DROP id=%lu len=%u prio=%u q=%u/%u pq=%u/%u",
                 static_cast<unsigned long>(tx.id), static_cast<unsigned>(tx.len),
                 tx.priority ? 1U : 0U,
                 static_cast<unsigned>(rawTxQueue_.size()),
                 static_cast<unsigned>(rawTxQueue_.capacity()),
                 static_cast<unsigned>(rawTxPriorityQueue_.size()),
                 static_cast<unsigned>(rawTxPriorityQueue_.capacity()));
    return false;
  }
  ++rawTxQueued_;
  const uint32_t depth = static_cast<uint32_t>(rawTxQueue_.size() + rawTxPriorityQueue_.size());
  if (depth > rawTxHighWater_) rawTxHighWater_ = depth;
  LINFBB_TRACE("TXQ_ENQ id=%lu len=%u prio=%u q=%u/%u pq=%u/%u",
               static_cast<unsigned long>(tx.id), static_cast<unsigned>(tx.len),
               tx.priority ? 1U : 0U,
               static_cast<unsigned>(rawTxQueue_.size()),
               static_cast<unsigned>(rawTxQueue_.capacity()),
               static_cast<unsigned>(rawTxPriorityQueue_.size()),
               static_cast<unsigned>(rawTxPriorityQueue_.capacity()));
  return true;
}

bool Tnc::rawFramePriority(const uint8_t* data, size_t len) const {
  ax25::Frame frame{};
  if (!ax25::decodeFrame(data, len, frame, true)) {
    return false;
  }
  const ax25::FrameKind kind = ax25::kind(frame.control);
  if (kind == ax25::FrameKind::S) {
    return true;
  }
  if (kind == ax25::FrameKind::U && ax25::uType(frame.control) != ax25::UFrameType::UI) {
    return true;
  }
  return false;
}

void Tnc::serviceRawTx(bool radioReady) {
  if (!radioReady) return;
  const uint32_t now = axlora::util::nowMs();
  if (static_cast<int32_t>(now - nextRawTxAttemptMs_) < 0) return;

  PendingRawTx tx{};
  RawTxQueue* queue = &rawTxPriorityQueue_;
  if (!queue->peek(tx)) {
    queue = &rawTxQueue_;
    if (!queue->peek(tx)) return;
  }
  if (static_cast<int32_t>(now - tx.notBeforeMs) < 0) {
    nextRawTxAttemptMs_ = tx.notBeforeMs;
    return;
  }

  if (kissParams_.fullDuplex == 0) {
    const uint32_t slotMs = static_cast<uint32_t>(kissParams_.slotTime) * 10;
    if (isChannelBusy() ||
        static_cast<uint8_t>(random(256)) > kissParams_.persistence) {
      ++rawTxDeferred_;
      nextRawTxAttemptMs_ = now + (slotMs < 10 ? 10 : slotMs);
      return;
    }
  }

  ++radioTxStarted_;
  LINFBB_TRACE("LORA_TX_START id=%lu len=%u prio=%u q=%u/%u pq=%u/%u",
               static_cast<unsigned long>(tx.id), static_cast<unsigned>(tx.len),
               tx.priority ? 1U : 0U,
               static_cast<unsigned>(rawTxQueue_.size()),
               static_cast<unsigned>(rawTxQueue_.capacity()),
               static_cast<unsigned>(rawTxPriorityQueue_.size()),
               static_cast<unsigned>(rawTxPriorityQueue_.capacity()));
  const radio::Result result = radio::driver().send(tx.data, tx.len);
  if (result == radio::Result::Ok) {
    queue->pop(tx);
    ++rawTx_;
    ++radioTxDone_;
    // Echo transmitted frames as KISS only on the console port. In WA8DED hostmode
    // the same serial stream belongs to the host protocol and raw KISS bytes would
    // force clients such as LinFBB to resync before seeing pending events.
#if !defined(AXLORA_VARIANT_SENSECAP_SOLAR)
    if (serialMode_ == SerialMode::Console && !kissTcpActive() && tx.len >= 2) {
      emitKissData(tx.data, tx.len - 2);
    }
#endif
    // Restart T1 from now — frame is on air. Corrects for CSMA + LoRa airtime delay.
    if (tx.chIdx < CHANNEL_COUNT) {
      channels_[tx.chIdx].link.notifyTxSent();
    }
    nextRawTxAttemptMs_ = axlora::util::nowMs();
    LINFBB_TRACE("LORA_TX_DONE id=%lu raw_tx=%lu", static_cast<unsigned long>(tx.id),
                 static_cast<unsigned long>(rawTx_));
    return;
  }
  if (result == radio::Result::Busy) {
    ++rawTxDeferred_;
    nextRawTxAttemptMs_ = axlora::util::nowMs() + 50;
    return;
  }
  LOG_WARN("radio tx failed: %s", radio::resultName(result));
  queue->pop(tx);
  ++rawTxQueueDrops_;
}

bool Tnc::radioTxCallback(const uint8_t* data, size_t len, void* ctx) {
  ChannelCtx* ch = static_cast<ChannelCtx*>(ctx);
  return ch->tnc->transmitRaw(data, len, ch->chIdx);
}

bool Tnc::dataReadyCallback(void* ctx) {
  const ChannelCtx* channel = static_cast<ChannelCtx*>(ctx);
  return channel->tnc->serialMode_ != SerialMode::Wa8ded ||
         channel->tnc->dedEvents_.free() > CHANNEL_COUNT;
}

bool Tnc::dataCallback(const uint8_t* data, size_t len, bool connected, void* ctx) {
  ChannelCtx* ch   = static_cast<ChannelCtx*>(ctx);
  Tnc*        self = ch->tnc;
  const uint8_t chIdx   = ch->chIdx;
  const uint8_t channel = chIdx + 1;   // 1-based for WA8DED

  if (self->serialMode_ == SerialMode::Wa8ded) {
    if (!self->dedHostMode_ && self->dedCtextMode_ == 2 && connected && len >= 3 &&
        data[0] == '/' && data[1] == '/' && data[2] == 'Q') {
      self->disconnect(chIdx);
      return true;
    }
    if (len == 0) return true;
    if (!dataReadyCallback(ctx)) return false;
    return self->enqueueDedEvent(connected ? channel : 0, connected ? 7 : 6, data, len);
  }
  if (connected) {
    self->processNodeInput(chIdx, data, len);
    return true;
  }
  if (axlora::util::logEnabled()) {
    Serial.printf("[AX25/UI ch%u] ", channel);
    for (size_t i = 0; i < len; ++i) Serial.write(data[i]);
    Serial.println();
  }
  return true;
}

int Tnc::findChannelForIncoming(const ax25::Frame& frame) const {
  // UI frames are handled at the TNC level, not routed to a connected LinkLayer
  if (ax25::kind(frame.control) == ax25::FrameKind::U &&
      ax25::uType(frame.control) == ax25::UFrameType::UI) {
    return -1;
  }
  // Find channel with matching peer in a non-Disconnected state
  for (int i = 0; i < static_cast<int>(CHANNEL_COUNT); ++i) {
    if (channels_[i].link.state() != ax25::LinkState::Disconnected &&
        ax25::addressEquals(channels_[i].link.peer(), frame.source) &&
        ax25::addressEquals(channels_[i].link.local(), frame.destination)) {
      return i;
    }
  }
  // New SABM: assign to first available (Disconnected) channel
  if (ax25::kind(frame.control) == ax25::FrameKind::U &&
      ax25::uType(frame.control) == ax25::UFrameType::SABM) {
    uint8_t active = 0;
    for (int i = 0; i < static_cast<int>(CHANNEL_COUNT); ++i) {
      if (channels_[i].link.state() != ax25::LinkState::Disconnected) ++active;
    }
    if (active >= dedMaxIncoming_) return -1;
    for (int i = 0; i < static_cast<int>(CHANNEL_COUNT); ++i) {
      if (channels_[i].link.state() == ax25::LinkState::Disconnected &&
          ax25::addressEquals(channels_[i].link.local(), frame.destination)) {
        return i;
      }
    }
  }
  return -1;
}

void Tnc::serviceRadio() {
  radio::RxPacket rx{};
  if (radio::driver().receive(rx) != radio::Result::Ok) return;
  lastRxMs_ = axlora::util::nowMs();
  lastRssi_ = rx.rssi;
  lastSnr_  = rx.snr;
  ++rawRx_;
  ++radioRxFrames_;
  if (rx.len < 2 || !ax25::checkFcs(rx.data, rx.len)) {
    ++radioCrcFail_;
    if (rx.len > 0) {
      LOG_RADIO("rx FCS error len=%u rssi=%.0f snr=%.0f",
                static_cast<unsigned>(rx.len),
                static_cast<double>(rx.rssi), static_cast<double>(rx.snr));
    }
    return;
  }
  {  // FCS OK
    LOG_RADIO("rx ok len=%u rssi=%.0f snr=%.0f", static_cast<unsigned>(rx.len),
              static_cast<double>(rx.rssi), static_cast<double>(rx.snr));
    ax25::Frame frame{};
    const bool decoded = ax25::decodeFrame(rx.data, rx.len, frame, true);
    if (decoded) {
      const ax25::FrameKind rkind = ax25::kind(frame.control);
      if (rkind == ax25::FrameKind::I) ++ax25IFramesOut_;
      if (rkind == ax25::FrameKind::S && ax25::sType(frame.control) == ax25::SFrameType::RR) ++ax25RrRx_;
      if (rkind == ax25::FrameKind::S && ax25::sType(frame.control) == ax25::SFrameType::REJ) ++ax25RejRx_;
      char src[12]{}, dst[12]{};
      ax25::formatAddress(frame.source, src, sizeof(src));
      ax25::formatAddress(frame.destination, dst, sizeof(dst));
      LINFBB_TRACE("RADIO_AX25_RX %s>%s kind=%u ns=%u nr=%u pf=%u pid=%02X info=%u",
                   src, dst, static_cast<unsigned>(rkind),
                   rkind == ax25::FrameKind::I ? ax25::ns(frame.control) : 0,
                   rkind == ax25::FrameKind::I || rkind == ax25::FrameKind::S ? ax25::nr(frame.control) : 0,
                   (frame.control & 0x10) ? 1U : 0U,
                   static_cast<unsigned>(frame.pid),
                   static_cast<unsigned>(frame.infoLen));
    }
    const bool tcpOwnsReceive = kissTcpActive();
    if (
#if !defined(AXLORA_VARIANT_SENSECAP_SOLAR)
        serialMode_ == SerialMode::Console ||
#endif
        serialMode_ == SerialMode::Kiss || tcpOwnsReceive) {
      emitKissData(rx.data, rx.len - 2);
    }
    if (serialMode_ == SerialMode::Kiss || tcpOwnsReceive) return;
    if (decoded) {
      // Detect APRS (UI, PID=0xF0) and log parsed content
      if (ax25::kind(frame.control) == ax25::FrameKind::U &&
          ax25::uType(frame.control) == ax25::UFrameType::UI &&
          frame.pid == ax25::PID_NO_LAYER3 && frame.infoLen > 0) {
        char aprsSum[80]{};
        if (aprs::parse(frame.info, frame.infoLen, aprsSum, sizeof(aprsSum))) {
          char src[12]{};
          ax25::formatAddress(frame.source, src, sizeof(src));
          LOG_INFO("APRS %s: %s", src, aprsSum);
        }
        // Auto-ACK APRS messages addressed to our callsign
        if (frame.infoLen > 0 && static_cast<char>(frame.info[0]) == ':') {
          aprs::AprsMessage msg{};
          if (aprs::parseMessage(frame.info, frame.infoLen, msg) &&
              !msg.isAck && !msg.isRej && msg.msgNum[0] != '\0') {
            char localStr[12]{};
            ax25::formatAddress(local_, localStr, sizeof(localStr));
            if (strncasecmp(msg.addressee, localStr, strlen(localStr)) == 0) {
              char ackInfo[32]{};
              char senderStr[12]{};
              ax25::formatAddress(frame.source, senderStr, sizeof(senderStr));
              if (aprs::encodeMessageAck(senderStr, msg.msgNum, ackInfo, sizeof(ackInfo))) {
                channels_[0].link.sendUi(frame.source,
                  reinterpret_cast<const uint8_t*>(ackInfo), strlen(ackInfo));
                LOG_INFO("APRS ACK sent to %s msgnum=%s", senderStr, msg.msgNum);
              }
            }
          }
        }
      }
      observeHeard(frame, rx.rssi, rx.snr);
      observeNetrom(frame);
      maybeDigipeat(frame);
      if (monitorEnabled_ && serialMode_ == SerialMode::Wa8ded) {
        if (dedMonitorAllows(frame)) {
          // Flood guard: reserve half the event queue for link-status/data events,
          // and limit monitor output to 8 events per second.
          constexpr size_t MONITOR_QUEUE_HEADROOM = 16;  // out of 32 slots
          constexpr uint32_t MONITOR_MIN_INTERVAL_MS = 125; // max 8/sec
          const uint32_t nowMs = axlora::util::nowMs();
          const bool queueOk = dedEvents_.size() <= (32 - MONITOR_QUEUE_HEADROOM);
          const bool rateOk  = axlora::util::elapsed(nowMs, monitorLastEnqueueMs_, MONITOR_MIN_INTERVAL_MS);
          if (queueOk && rateOk) {
            monitorLastEnqueueMs_ = nowMs;
            char header[160]{};
            size_t headerLen = ded::monitorHeader(frame, header, sizeof(header));
            if (dedTimestamp_ == 2) {
              char stamp[24]{};
              formatDedTimestamp(stamp, sizeof(stamp));
              snprintf(header + headerLen, sizeof(header) - headerLen, " - %s", stamp);
              headerLen = strlen(header);
            }
            const bool hasData = frame.infoLen > 0 &&
                (ax25::kind(frame.control) == ax25::FrameKind::I || isUiFrame(frame));
            if (hasData) {
              enqueueDedEvent(0, 5, reinterpret_cast<const uint8_t*>(header), headerLen);
              enqueueDedEvent(0, 6, frame.info, frame.infoLen);
            } else {
              enqueueDedEvent(0, 4, reinterpret_cast<const uint8_t*>(header), headerLen);
            }
          }
        }
      }
      const int chIdx = findChannelForIncoming(frame);
      if (chIdx >= 0) {
        channels_[chIdx].link.receive(rx.data, rx.len);
      } else if (serialMode_ == SerialMode::Wa8ded &&
                 ax25::kind(frame.control) == ax25::FrameKind::U &&
                 ax25::uType(frame.control) == ax25::UFrameType::SABM) {
        for (const ChannelState& channel : channels_) {
          if (!ax25::addressEquals(channel.link.local(), frame.destination)) continue;
          char source[12]{}, status[96]{};
          ax25::formatAddress(frame.source, source, sizeof(source));
          snprintf(status, sizeof(status), "CONNECT REQUEST fm %s", source);
          enqueueDedEvent(0, 3, reinterpret_cast<const uint8_t*>(status), strlen(status));
          ax25::Frame response{};
          response.destination = frame.source;
          response.source = frame.destination;
          response.command = false;
          response.control = ax25::makeU(ax25::UFrameType::DM, (frame.control & 0x10) != 0);
          uint8_t bytes[MAX_PACKET_BYTES]{};
          size_t size = 0;
          if (ax25::encodeFrame(response, bytes, sizeof(bytes), size, true))
            transmitRaw(bytes, size);
          break;
        }
      }
    }
  }
}

// ---------------------------------------------------------------------------
// Beacon / NET/ROM
// ---------------------------------------------------------------------------

void Tnc::serviceBeacon(bool radioReady) {
  if (!radioReady || !beacon_.enabled) return;
  const uint32_t now = axlora::util::nowMs();
  if (beacon_.lastTxMs != 0 && !axlora::util::elapsed(now, beacon_.lastTxMs, beacon_.intervalMs)) return;
  beacon_.lastTxMs = now;
  sendBeacon();
}

void Tnc::serviceNetrom(bool radioReady) {
  if (!radioReady || !netrom_.enabled) return;
  const uint32_t now = axlora::util::nowMs();
  // Expire routes not heard within 6 broadcast intervals (NET/ROM obsolescence)
  const uint32_t expiryMs = netrom_.broadcastIntervalMs * 6;
  for (NetromRoute& route : netromRoutes_) {
    if (route.active && axlora::util::elapsed(now, route.lastHeardMs, expiryMs)) {
      route.active = false;
    }
  }
  if (netrom_.lastBroadcastMs != 0 &&
      !axlora::util::elapsed(now, netrom_.lastBroadcastMs, netrom_.broadcastIntervalMs)) return;
  netrom_.lastBroadcastMs = now;
  sendNetromBroadcast();
}

// ---------------------------------------------------------------------------
// Serial
// ---------------------------------------------------------------------------

void Tnc::serviceSerial() {
  constexpr size_t  SERIAL_BYTE_BUDGET = 768;
  constexpr uint8_t KISS_FRAME_BUDGET  = 4;
  size_t bytesProcessed = 0;
  uint8_t kissFramesProcessed = 0;

  if (serialMode_ == SerialMode::Wa8ded && dedHostMode_ && dedHeaderPos_ != 0 &&
      Serial.available() == 0 && static_cast<uint32_t>(millis() - dedLastByteMs_) >= 1000) {
    dedHeaderPos_ = 0;
    dedDataPos_ = 0;
    dedDataLen_ = 0;
    ++dedRxTimeouts_;
    LOG_WARN("incomplete WA8DED frame discarded after serial inactivity");
  }

  while (Serial.available() > 0 && bytesProcessed < SERIAL_BYTE_BUDGET) {
    if (serialMode_ == SerialMode::Kiss &&
        rawTxQueue_.free() <= KISS_SERIAL_TXQ_HEADROOM &&
        rawTxPriorityQueue_.free() == 0) {
      break;
    }
    const uint8_t b = static_cast<uint8_t>(Serial.read());
    ++bytesProcessed;
    // A FEND where no WA8DED frame is in progress means a KISS host took over
    // the port (0xC0 is never a valid WA8DED channel). Enter full KISS mode so
    // the firmware stops answering AX.25 itself and keeps text off the wire.
    if (b == ax25::KISS_FEND && serialMode_ != SerialMode::Kiss &&
        (serialMode_ == SerialMode::Console || !dedHostMode_ || dedHeaderPos_ == 0)) {
      setSerialMode(SerialMode::Kiss);
    }
    if (serialMode_ == SerialMode::Wa8ded) {
      const bool wasHostMode = dedHostMode_;
      serviceWa8ded(b);
      if (!wasHostMode && !dedHostMode_) handleQuietEscape(b);
      continue;
    }
    if (serialMode_ == SerialMode::Kiss) {
      ax25::KissFrame frame{};
      if (kiss_.feed(b, frame)) {
        handleKiss(frame);
        if (++kissFramesProcessed >= KISS_FRAME_BUDGET || serialMode_ != SerialMode::Kiss) {
          break;
        }
      }
      continue;
    }
    if (b == 0x11) continue;
    if (b == 0x18) {
      linePos_ = 0;
      continue;
    }
    if (b == '\r' || b == '\n') {
      if (linePos_ == 0) continue;
      line_[linePos_] = '\0';
      if (b == '\r' && strcasecmp(line_, "\x1b" "JHOST1") == 0) {
        setSerialMode(SerialMode::Wa8ded);
        const char entry[] = "\x1b" "JHOST1\r";
        for (size_t i = 0; i < sizeof(entry) - 1; ++i) {
          serviceWa8dedTerminal(static_cast<uint8_t>(entry[i]));
        }
        continue;
      }
      Serial.print("\r\n");
      handleConsoleLine(line_);
      linePos_ = 0;
      continue;
    }
    if (b == 0x08 || b == 0x7F) {
      if (linePos_ > 0) {
        --linePos_;
        Serial.print("\b \b");
      }
      continue;
    }
    if (linePos_ < sizeof(line_) - 1) {
      line_[linePos_++] = static_cast<char>(b);
#if defined(AXLORA_VARIANT_STATION_G2)
      // Do not echo Wi-Fi credentials to the USB terminal.
      if (linePos_ < 5 || strncmp(line_, "wifi ", 5) != 0)
#endif
      Serial.write(b);
    } else {
      Serial.write(static_cast<uint8_t>(0x07));
    }
  }
}

void Tnc::handleKiss(const ax25::KissFrame& frame, bool fromTcp) {
  // TAPR KISS "Return" (C0 FF C0): leave KISS for the command console.
  if (frame.port == 0x0F && static_cast<uint8_t>(frame.command) == 0x0F) {
#if defined(AXLORA_VARIANT_STATION_G2)
    if (fromTcp) {
      wifiKiss_.closeClient();
      return;
    }
#else
    (void)fromTcp;
#endif
    setSerialMode(SerialMode::Console);
    return;
  }
  ++kissInFrames_;
  kissInBytes_ += static_cast<uint32_t>(frame.len + 1);
  LINFBB_TRACE("KISS_IN n=%lu port=%u cmd=%u len=%u first=%02X %02X %02X %02X",
               static_cast<unsigned long>(kissInFrames_),
               static_cast<unsigned>(frame.port),
               static_cast<unsigned>(frame.command),
               static_cast<unsigned>(frame.len),
               frame.len > 0 ? frame.data[0] : 0,
               frame.len > 1 ? frame.data[1] : 0,
               frame.len > 2 ? frame.data[2] : 0,
               frame.len > 3 ? frame.data[3] : 0);
  if (frame.command == ax25::KissCommand::Data) {
    ax25::Frame parsed{};
    if (ax25::decodeFrame(frame.data, frame.len, parsed, false)) {
      const ax25::FrameKind kind = ax25::kind(parsed.control);
      if (kind == ax25::FrameKind::I) ++ax25IFramesIn_;
      if (kind == ax25::FrameKind::U && ax25::uType(parsed.control) == ax25::UFrameType::UI) ++ax25UiFrames_;
      if (kind == ax25::FrameKind::S && ax25::sType(parsed.control) == ax25::SFrameType::RR) ++ax25RrTx_;
      if (kind == ax25::FrameKind::S && ax25::sType(parsed.control) == ax25::SFrameType::REJ) ++ax25RejTx_;
      char src[12]{}, dst[12]{};
      ax25::formatAddress(parsed.source, src, sizeof(src));
      ax25::formatAddress(parsed.destination, dst, sizeof(dst));
      LINFBB_TRACE("KISS_AX25_IN %s>%s kind=%u ns=%u nr=%u pf=%u pid=%02X info=%u",
                   src, dst, static_cast<unsigned>(kind),
                   kind == ax25::FrameKind::I ? ax25::ns(parsed.control) : 0,
                   kind == ax25::FrameKind::I || kind == ax25::FrameKind::S ? ax25::nr(parsed.control) : 0,
                   (parsed.control & 0x10) ? 1U : 0U,
                   static_cast<unsigned>(parsed.pid),
                   static_cast<unsigned>(parsed.infoLen));
      // Drop I-frame retransmits already pending in the TX queue.
      // LinFBB's T1 fires before LoRa round-trip completes; without dedup the
      // queue accumulates N2 copies of the same NS, each costing one full LoRa
      // airtime, so later I-frames arrive at the host only after duplicates
      // drain — causing the beginning of multi-frame responses to be "lost".
      if (kind == ax25::FrameKind::I) {
        const uint8_t txNs = ax25::ns(parsed.control);
        for (size_t qi = 0; qi < rawTxQueue_.size(); ++qi) {
          PendingRawTx queued{};
          if (!rawTxQueue_.peek(qi, queued)) break;
          ax25::Frame qf{};
          if (!ax25::decodeFrame(queued.data, queued.len, qf, true)) continue;
          if (ax25::kind(qf.control) != ax25::FrameKind::I) continue;
          if (ax25::ns(qf.control) == txNs &&
              ax25::addressEquals(qf.source, parsed.source) &&
              ax25::addressEquals(qf.destination, parsed.destination)) {
            ++kissIFrameDuplicates_;
            LINFBB_TRACE("KISS_IDUP_DROP ns=%u src=%s dst=%s q=%u",
                         static_cast<unsigned>(txNs), src, dst,
                         static_cast<unsigned>(rawTxQueue_.size()));
            return;
          }
        }
      }
    } else {
      ++kissDecodeErrors_;
      LINFBB_TRACE("KISS_AX25_IN_DECODE_FAIL len=%u", static_cast<unsigned>(frame.len));
    }
    uint8_t withFcs[MAX_PACKET_BYTES]{};
    size_t len = 0;
    if (ax25::appendFcs(frame.data, frame.len, withFcs, sizeof(withFcs), len)) {
      transmitRaw(withFcs, len);
    } else {
      ++kissDecodeErrors_;
    }
    return;
  }
  if (frame.len < 1) return;
  switch (frame.command) {
    case ax25::KissCommand::TxDelay:     kissParams_.txDelay     = frame.data[0]; break;
    case ax25::KissCommand::Persistence: kissParams_.persistence = frame.data[0]; break;
    case ax25::KissCommand::SlotTime:    kissParams_.slotTime    = frame.data[0]; break;
    case ax25::KissCommand::FullDuplex:  kissParams_.fullDuplex  = frame.data[0]; break;
    default: break;
  }
}

void Tnc::emitKissData(const uint8_t* frameNoFcs, size_t len) {
  ax25::KissFrame kissFrame{};
  kissFrame.command = ax25::KissCommand::Data;
  kissFrame.len = len > sizeof(kissFrame.data) ? sizeof(kissFrame.data) : len;
  memcpy(kissFrame.data, frameNoFcs, kissFrame.len);
  uint8_t encoded[(MAX_PACKET_BYTES * 2) + 4]{};
  size_t encodedLen = 0;
  if (ax25::encodeKiss(kissFrame, encoded, sizeof(encoded), encodedLen)) {
#if defined(AXLORA_VARIANT_STATION_G2)
    if (wifiKiss_.active()) {
      wifiKiss_.sendFrame(encoded, encodedLen);
      ++kissOutFrames_;
      return;
    }
#endif
    const uint32_t startMs = axlora::util::nowMs();
    if (!serialWriteAll(encoded, encodedLen)) {
      ++kissOutPartialWrites_;
      LOG_WARN("serial KISS write timeout len=%u", static_cast<unsigned>(encodedLen));
    }
    const uint32_t elapsedMs = axlora::util::nowMs() - startMs;
    if (elapsedMs > kissOutBlockMsMax_) kissOutBlockMsMax_ = elapsedMs;
    ++kissOutFrames_;
    LINFBB_TRACE("KISS_OUT n=%lu len=%u block_ms=%lu partial=%lu",
                 static_cast<unsigned long>(kissOutFrames_),
                 static_cast<unsigned>(encodedLen),
                 static_cast<unsigned long>(elapsedMs),
                 static_cast<unsigned long>(kissOutPartialWrites_));
  }
}

// ---------------------------------------------------------------------------
// Console command handler
// ---------------------------------------------------------------------------

void Tnc::handleConsoleLine(char* line) {
#if defined(AXLORA_VARIANT_STATION_G2)
  if (strcmp(line, "wifi") == 0 || strncmp(line, "wifi ", 5) == 0) {
    wifiKiss_.command(line + (line[4] == ' ' ? 5 : 4));
    memset(line, 0, sizeof(line_));
    return;
  }
#endif
  char* cmd = strtok(line, " ");
  if (cmd == nullptr) return;

  if (strcasecmp(cmd, "kiss") == 0) {
    // TAPR / Kenwood style "KISS ON" (as sent by KISSINIT-style KISS.INI files).
    char* arg = strtok(nullptr, " ");
    if (arg != nullptr && strcasecmp(arg, "on") == 0) {
      Serial.flush();
      setSerialMode(SerialMode::Kiss);
    } else {
      Serial.println("usage: KISS ON");
    }
    return;
  }
  if (strcasecmp(cmd, "restart") == 0) return;

  if (strcmp(cmd, "help") == 0) {
    Serial.println("Commands: help info callsign mode baud stats bbs");
    Serial.println("          radio [freq|corr|sf|bw|cr|power|reset] [val]");
    Serial.println("          duty [on|off|percent]");
    Serial.println("          profile [fast|normal]");
    Serial.println("          ax25 connect [ch] <CALL> disconnect [ch]");
    Serial.println("          send [ch] <msg> sendui <DEST> <msg>");
    Serial.println("          beacon [on|off|now|text|dest|interval|path]");
    Serial.println("          mheard [clear] digi [on|off|mode ui|all] digialias [CALL|off]");
#if defined(AXLORA_VARIANT_SENSECAP_SOLAR)
    Serial.println("          manager [name]");
#endif
    Serial.println("          node [on|off|alias|ident|interval|broadcast] nodes routes");
    Serial.println("          KISS ON (enter KISS; KISS frame C0 FF C0 returns here)");
#if defined(AXLORA_VARIANT_STATION_G2)
    Serial.println("          wifi [status|off|sta|ap|ssid <name>|password <value>|password clear]");
#endif

  } else if (strcmp(cmd, "info") == 0) {
    printInfo();

#if defined(AXLORA_VARIANT_SENSECAP_SOLAR)
  } else if (strcmp(cmd, "manager") == 0) {
    const char* value = skipSpaces(strtok(nullptr, ""));
    if (value == nullptr || *value == '\0') {
      Serial.printf("manager=%s\n", manager_);
    } else if (strlen(value) >= sizeof(manager_) || strpbrk(value, "\r\n") != nullptr) {
      Serial.println("invalid manager name");
    } else {
      Preferences prefs;
      if (!prefs.begin("axloratnc", false)) {
        Serial.println("manager: settings storage unavailable");
      } else {
        const size_t written = prefs.putString("manager", value);
        prefs.end();
        if (written != strlen(value)) {
          Serial.println("manager: settings save failed");
        } else {
          copySanitizedNodeText(manager_, sizeof(manager_), value);
          Serial.printf("manager=%s\n", manager_);
        }
      }
    }
#endif
  } else if (strcmp(cmd, "mode") == 0) {
    char* mode = strtok(nullptr, " ");
    if (mode == nullptr) {
      Serial.printf("mode=%s\n", serialModeName());
    } else if (strcmp(mode, "kiss") == 0) {
      Serial.println("mode=kiss"); Serial.flush();
      setSerialMode(SerialMode::Kiss); saveSerialMode(SerialMode::Kiss);
    } else if (strcmp(mode, "ded") == 0 || strcmp(mode, "wa8ded") == 0) {
      Serial.println("mode=ded"); Serial.flush();
      setSerialMode(SerialMode::Wa8ded); saveSerialMode(SerialMode::Wa8ded);
    } else if (strcmp(mode, "console") == 0) {
      setSerialMode(SerialMode::Console); saveSerialMode(SerialMode::Console);
      Serial.println("mode=console");
    } else {
      Serial.println("usage: mode [console|kiss|ded]");
    }

  } else if (strcmp(cmd, "baud") == 0) {
    char* modeArg = strtok(nullptr, " ");
    char* baudArg = strtok(nullptr, " ");
    if (modeArg == nullptr) {
      Serial.printf("baud console=%lu (fixed)\n", static_cast<unsigned long>(variant::SERIAL_BAUD));
      Serial.printf("baud kiss=%lu\n",    static_cast<unsigned long>(baudKiss_));
      Serial.printf("baud ded=%lu\n",     static_cast<unsigned long>(baudDed_));
    } else if (baudArg == nullptr) {
      Serial.println("usage: baud <kiss|ded> <rate>");
      Serial.println("valid: 300 1200 2400 4800 9600 19200 38400 57600 115200 230400 460800 921600");
    } else {
      const uint32_t rate = static_cast<uint32_t>(atol(baudArg));
      if (!isValidBaud(rate)) {
        Serial.println("invalid baud rate");
      } else if (strcmp(modeArg, "kiss") == 0) {
        baudKiss_ = rate;
        Preferences prefs;
        if (prefs.begin("axloratnc", false)) { prefs.putULong("baud_kiss", baudKiss_); prefs.end(); }
        Serial.printf("baud kiss=%lu\n", static_cast<unsigned long>(baudKiss_));
        if (serialMode_ == SerialMode::Kiss) applySerialBaud();
      } else if (strcmp(modeArg, "ded") == 0 || strcmp(modeArg, "wa8ded") == 0) {
        baudDed_ = rate;
        Preferences prefs;
        if (prefs.begin("axloratnc", false)) { prefs.putULong("baud_ded", baudDed_); prefs.end(); }
        Serial.printf("baud ded=%lu\n", static_cast<unsigned long>(baudDed_));
        if (serialMode_ == SerialMode::Wa8ded) applySerialBaud();
      } else {
        Serial.println("usage: baud <kiss|ded> <rate>");
      }
    }

  } else if (strcmp(cmd, "callsign") == 0) {
    char* cs = strtok(nullptr, " ");
    char addrStr[12]{};
    if (cs == nullptr) {
      ax25::formatAddress(local_, addrStr, sizeof(addrStr));
      Serial.printf("callsign=%s\n", addrStr);
    } else {
      ax25::Address newAddr{};
      if (!ax25::parseAddress(cs, newAddr)) {
        Serial.println("invalid callsign");
      } else {
        local_ = newAddr;
        resetLinksForLocal();
        ax25::formatAddress(local_, savedCallsign_, sizeof(savedCallsign_));
        Preferences prefs;
        if (prefs.begin("axloratnc", false)) {
          prefs.putString("callsign", savedCallsign_);
          prefs.end();
        }
        Serial.printf("callsign=%s\n", savedCallsign_);
      }
    }

  } else if (strcmp(cmd, "radio") == 0) {
    char* sub = strtok(nullptr, " ");
    if (sub == nullptr) {
      Serial.printf("freq=%.*f corr=%+.1f kHz bw=%.1f sf=%u cr=4/%u pwr=%d RSSI=%.1f SNR=%.1f\n",
                    FREQUENCY_DECIMALS, static_cast<double>(radioConfig_.frequencyMHz),
                    static_cast<double>(radioConfig_.frequencyCorrectionMHz * 1000.0f),
                    static_cast<double>(radioConfig_.bandwidthKhz),
                    radioConfig_.spreadingFactor, radioConfig_.codingRate,
                    radioConfig_.powerDbm,
                    static_cast<double>(radio::driver().getRSSI()),
                    static_cast<double>(radio::driver().getSNR()));
    } else if (strcmp(sub, "reset") == 0) {
      radioConfig_.frequencyMHz    = variant::DEFAULT_FREQUENCY_MHZ;
      radioConfig_.frequencyCorrectionMHz = variant::DEFAULT_FREQUENCY_CORRECTION_MHZ;
      radioConfig_.bandwidthKhz    = variant::DEFAULT_BANDWIDTH_KHZ;
      radioConfig_.spreadingFactor = variant::DEFAULT_SPREADING_FACTOR;
      radioConfig_.codingRate      = variant::DEFAULT_CODING_RATE;
      radioConfig_.powerDbm        = variant::DEFAULT_TX_POWER_DBM;
      applyRadioConfig();
      radioConfigApplied_ = true;
      saveRadioConfig();
      Serial.printf("radio reset freq=%.*f corr=%+.1f kHz bw=%.1f sf=%u cr=4/%u pwr=%d\n",
                    FREQUENCY_DECIMALS, static_cast<double>(radioConfig_.frequencyMHz),
                    static_cast<double>(radioConfig_.frequencyCorrectionMHz * 1000.0f),
                    static_cast<double>(radioConfig_.bandwidthKhz),
                    radioConfig_.spreadingFactor, radioConfig_.codingRate,
                    radioConfig_.powerDbm);
    } else if (strcmp(sub, "freq") == 0) {
      char* val = strtok(nullptr, " ");
      if (!val) { Serial.println("usage: radio freq <MHz>"); }
      else {
        radioConfig_.frequencyMHz = static_cast<float>(atof(val));
        radio::driver().setFrequency(radioConfig_.frequencyMHz);
        saveRadioConfig();
        Serial.printf("freq=%.*f MHz\n", FREQUENCY_DECIMALS, static_cast<double>(radioConfig_.frequencyMHz));
      }
    } else if (strcmp(sub, "corr") == 0 || strcmp(sub, "correction") == 0 || strcmp(sub, "offset") == 0) {
      char* val = strtok(nullptr, " ");
      if (!val) {
        Serial.printf("corr=%+.1f kHz\n", static_cast<double>(radioConfig_.frequencyCorrectionMHz * 1000.0f));
      } else if constexpr (variant::RADIO_TYPE != variant::RadioType::SX1276) {
        Serial.println("radio corr is only used by SX1276 variants");
      } else {
        const float correctionKhz = static_cast<float>(atof(val));
        if (correctionKhz < -250.0f || correctionKhz > 250.0f) {
          Serial.println("usage: radio corr <kHz>  (-250..250)");
        } else {
          radioConfig_.frequencyCorrectionMHz = correctionKhz / 1000.0f;
          radio::driver().setFrequencyCorrection(radioConfig_.frequencyCorrectionMHz);
          saveRadioConfig();
          Serial.printf("corr=%+.1f kHz tuned=%.3f MHz\n",
                        static_cast<double>(correctionKhz),
                        static_cast<double>(radioConfig_.frequencyMHz + radioConfig_.frequencyCorrectionMHz));
        }
      }
    } else if (strcmp(sub, "sf") == 0) {
      char* val = strtok(nullptr, " ");
      const long v = val ? atol(val) : 0;
      if (v < 6 || v > 12) { Serial.println("usage: radio sf <6-12>"); }
      else {
        radioConfig_.spreadingFactor = static_cast<uint8_t>(v);
        radio::driver().setSpreadingFactor(radioConfig_.spreadingFactor);
        saveRadioConfig();
        Serial.printf("sf=%u\n", radioConfig_.spreadingFactor);
      }
    } else if (strcmp(sub, "bw") == 0) {
      char* val = strtok(nullptr, " ");
      if (!val) { Serial.println("usage: radio bw <kHz>"); }
      else {
        radioConfig_.bandwidthKhz = static_cast<float>(atof(val));
        radio::driver().setBandwidth(radioConfig_.bandwidthKhz);
        saveRadioConfig();
        Serial.printf("bw=%.1f kHz\n", static_cast<double>(radioConfig_.bandwidthKhz));
      }
    } else if (strcmp(sub, "cr") == 0) {
      char* val = strtok(nullptr, " ");
      const long v = val ? atol(val) : 0;
      if (v < 5 || v > 8) { Serial.println("usage: radio cr <5-8>  (means 4/5..4/8)"); }
      else {
        radioConfig_.codingRate = static_cast<uint8_t>(v);
        radio::driver().setCodingRate(radioConfig_.codingRate);
        saveRadioConfig();
        Serial.printf("cr=4/%u\n", radioConfig_.codingRate);
      }
    } else if (strcmp(sub, "power") == 0 || strcmp(sub, "pwr") == 0) {
      char* val = strtok(nullptr, " ");
#if defined(AXLORA_VARIANT_STATION_G2)
      char* end = nullptr;
      const long power = val ? strtol(val, &end, 10) : 0;
      if (!val || end == val || *end != '\0' || power < -9 || power > variant::MAX_TX_POWER_DBM) {
        Serial.printf("usage: radio power <-9..%d> (SX1262 dBm)\n", variant::MAX_TX_POWER_DBM);
      } else if (radio::driver().setPower(static_cast<int8_t>(power)) != radio::Result::Ok) {
        Serial.println("radio power failed");
      } else {
        radioConfig_.powerDbm = static_cast<int8_t>(power);
        saveRadioConfig();
        Serial.printf("pwr=%d dBm (SX1262)\n", radioConfig_.powerDbm);
      }
#else
      if (!val) { Serial.println("usage: radio power <dBm>"); }
      else {
        radioConfig_.powerDbm = static_cast<int8_t>(atol(val));
        radio::driver().setPower(radioConfig_.powerDbm);
        saveRadioConfig();
        Serial.printf("pwr=%d dBm\n", radioConfig_.powerDbm);
      }
#endif
    } else {
      Serial.println("usage: radio [freq|corr|sf|bw|cr|power|reset] [value]");
    }

  } else if (strcmp(cmd, "duty") == 0) {
    char* arg = strtok(nullptr, " ");
    if (arg == nullptr) {
      Serial.printf("duty=%s %.3f%% drops=%lu\n",
                    dutyCycleEnabled_ ? "on" : "off",
                    static_cast<double>(dutyCyclePpm_) / 10000.0,
                    static_cast<unsigned long>(radio::stats().dutyDrops));
    } else if (strcmp(arg, "on") == 0) {
      dutyCycleEnabled_ = true;
      radio::driver().setDutyCycle(dutyCycleEnabled_, dutyCyclePpm_);
      saveRadioConfig();
      Serial.printf("duty=on %.3f%%\n", static_cast<double>(dutyCyclePpm_) / 10000.0);
    } else if (strcmp(arg, "off") == 0) {
      dutyCycleEnabled_ = false;
      radio::driver().setDutyCycle(false, dutyCyclePpm_);
      saveRadioConfig();
      Serial.println("WARNING: duty-cycle guard is OFF. Use only on dummy load/lab setups.");
    } else {
      const float percent = static_cast<float>(atof(arg));
      if (percent <= 0.0f || percent > 100.0f) {
        Serial.println("usage: duty [on|off|percent]");
      } else {
        dutyCyclePpm_ = static_cast<uint32_t>(percent * 10000.0f);
        dutyCycleEnabled_ = true;
        radio::driver().setDutyCycle(true, dutyCyclePpm_);
        saveRadioConfig();
        Serial.printf("duty=on %.3f%%\n", static_cast<double>(dutyCyclePpm_) / 10000.0);
      }
    }

  } else if (strcmp(cmd, "profile") == 0) {
    char* arg = strtok(nullptr, " ");
    if (arg == nullptr) {
      Serial.printf("profile custom duty=%s txdelay=%u p=%u slot=%u fulldup=%u\n",
                    dutyCycleEnabled_ ? "on" : "off",
                    kissParams_.txDelay, kissParams_.persistence,
                    kissParams_.slotTime, kissParams_.fullDuplex);
    } else if (strcmp(arg, "fast") == 0) {
      kissParams_.txDelay    = 0;
      kissParams_.persistence = 255;
      kissParams_.slotTime   = 1;
      kissParams_.fullDuplex = 0;
      dedFrack_ = 500;
      dedRetryLimit_ = 20;
      dedMaxFrame_ = 3;
      dedT2_ = 20;
      dedIPollFrameLength_ = 128;
      dutyCycleEnabled_ = false;
      radio::driver().setDutyCycle(false, dutyCyclePpm_);
      applyDedLinkConfig();
      saveRadioConfig();
      saveKissConfig();
      saveDedConfig();
      Serial.println("profile=fast txdelay=0 p=255 slot=1 fulldup=0 duty=off I=128 O=3 F=500 N=20 T2=20 @B=6/3");
      Serial.println("WARNING: fast profile disables duty-cycle guard. Use only on dummy load/lab setups.");
    } else if (strcmp(arg, "normal") == 0 || strcmp(arg, "default") == 0) {
      kissParams_.txDelay    = 30;
      kissParams_.persistence = 63;
      kissParams_.slotTime   = 10;
      kissParams_.fullDuplex = 0;
      dutyCycleEnabled_ = true;
      dutyCyclePpm_ = variant::DUTY_CYCLE_PPM;
      radio::driver().setDutyCycle(true, dutyCyclePpm_);
      saveRadioConfig();
      saveKissConfig();
      Serial.printf("profile=normal txdelay=30 p=63 slot=10 fulldup=0 duty=%.3f%%\n",
                    static_cast<double>(dutyCyclePpm_) / 10000.0);
    } else {
      Serial.println("usage: profile [fast|normal]");
    }

  } else if (strcmp(cmd, "ax25") == 0) {
    for (uint8_t i = 0; i < CHANNEL_COUNT; ++i) {
      if (channels_[i].link.state() != ax25::LinkState::Disconnected) {
        Serial.printf("ch%u: ", static_cast<unsigned>(i + 1));
        channels_[i].link.printStatus();
      }
    }

  } else if (strcmp(cmd, "node") == 0) {
    char* sub = strtok(nullptr, " ");
    if (sub == nullptr) {
      printNetrom();
    } else if (strcmp(sub, "on") == 0) {
      netrom_.enabled = true; netrom_.lastBroadcastMs = 0;
      saveSettings(); printNetrom();
    } else if (strcmp(sub, "off") == 0) {
      netrom_.enabled = false; saveSettings(); printNetrom();
    } else if (strcmp(sub, "alias") == 0) {
      char* alias = strtok(nullptr, " ");
      if (alias == nullptr || strlen(alias) > 6) {
        Serial.println("usage: node alias <1-6 chars>");
      } else {
        memset(netrom_.alias, 0, sizeof(netrom_.alias));
        strncpy(netrom_.alias, alias, sizeof(netrom_.alias) - 1);
        for (size_t i = 0; netrom_.alias[i] != '\0'; ++i) {
          if (netrom_.alias[i] >= 'a' && netrom_.alias[i] <= 'z')
            netrom_.alias[i] = static_cast<char>(netrom_.alias[i] - 32);
        }
        saveSettings(); printNetrom();
      }
    } else if (strcmp(sub, "ident") == 0) {
      char* ident = strtok(nullptr, "");
      if (ident == nullptr) {
        Serial.println("usage: node ident <text>");
      } else {
        copySanitizedNodeText(netrom_.ident, sizeof(netrom_.ident), ident);
        saveSettings(); printNetrom();
      }
    } else if (strcmp(sub, "interval") == 0) {
      char* secs = strtok(nullptr, " ");
      const long val = secs ? atol(secs) : 0;
      if (val < 60 || val > 86400) {
        Serial.println("usage: node interval <60-86400>");
      } else {
        netrom_.broadcastIntervalMs = static_cast<uint32_t>(val) * 1000UL;
        saveSettings(); printNetrom();
      }
    } else if (strcmp(sub, "broadcast") == 0) {
      Serial.println(sendNetromBroadcast() ? "node broadcast sent" : "node broadcast failed");
    } else {
      Serial.println("usage: node [on|off|alias|ident|interval|broadcast]");
    }

  } else if (strcmp(cmd, "nodes") == 0 || strcmp(cmd, "routes") == 0) {
    printNetromRoutes();

  } else if (strcmp(cmd, "beacon") == 0) {
    char* sub = strtok(nullptr, " ");
    if (sub == nullptr) {
      printBeacon();
    } else if (strcmp(sub, "on") == 0) {
      beacon_.enabled = true; beacon_.lastTxMs = 0; saveSettings(); printBeacon();
    } else if (strcmp(sub, "off") == 0) {
      beacon_.enabled = false; saveSettings(); printBeacon();
    } else if (strcmp(sub, "now") == 0) {
      Serial.println(sendBeacon() ? "beacon sent" : "beacon failed");
    } else if (strcmp(sub, "text") == 0) {
      char* text = strtok(nullptr, "");
      if (!text) { Serial.println("usage: beacon text <text>"); }
      else {
        copySanitizedNodeText(beacon_.text, sizeof(beacon_.text), text);
        saveSettings(); printBeacon();
      }
    } else if (strcmp(sub, "dest") == 0) {
      char* dest = strtok(nullptr, " ");
      if (!dest || !ax25::parseAddress(dest, beacon_.destination)) {
        Serial.println("usage: beacon dest <CALLSIGN-SSID>");
      } else { saveSettings(); printBeacon(); }
    } else if (strcmp(sub, "interval") == 0) {
      char* secs = strtok(nullptr, " ");
      const long val = secs ? atol(secs) : 0;
      if (val < 10 || val > 86400) { Serial.println("usage: beacon interval <10-86400>"); }
      else {
        beacon_.intervalMs = static_cast<uint32_t>(val) * 1000UL;
        saveSettings(); printBeacon();
      }
    } else if (strcmp(sub, "path") == 0) {
      char* path = strtok(nullptr, "");
      if (!setBeaconPath(path)) { Serial.println("usage: beacon path <CALL1,CALL2|off>"); }
      else { saveSettings(); printBeacon(); }
    } else if (strcmp(sub, "aprs") == 0) {
      // beacon aprs <lat> <lon> [symbol] [comment]
      // Formats beacon text as APRS position and sets dest to APRS
      char* latStr = strtok(nullptr, " ");
      char* lonStr = strtok(nullptr, " ");
      if (!latStr || !lonStr) {
        Serial.println("usage: beacon aprs <lat> <lon> [sym2] [comment]");
        Serial.println("  e.g. beacon aprs 52.0167 4.7000 /> LoRa TNC");
      } else {
        const double lat = atof(latStr);
        const double lon = atof(lonStr);
        char* sym  = strtok(nullptr, " ");
        char* cmt  = strtok(nullptr, "");
        char symBuf[3] = {'/', '>'};
        if (sym && strlen(sym) >= 2) { symBuf[0] = sym[0]; symBuf[1] = sym[1]; }
        char aprsInfo[96]{};
        if (aprs::encodePosition(lat, lon, symBuf, cmt ? cmt : "", aprsInfo, sizeof(aprsInfo))) {
          ax25::parseAddress("APRS", beacon_.destination);
          copySanitizedNodeText(beacon_.text, sizeof(beacon_.text), aprsInfo);
          beacon_.enabled = true;
          saveSettings();
          printBeacon();
        } else {
          Serial.println("aprs encode failed");
        }
      }
    } else {
      Serial.println("usage: beacon [on|off|now|text|dest|interval|path|aprs]");
    }

  } else if (strcmp(cmd, "mheard") == 0) {
    char* sub = strtok(nullptr, " ");
    if (sub && strcmp(sub, "clear") == 0) { clearMheard(); Serial.println("mheard cleared"); }
    else printMheard();

  } else if (strcmp(cmd, "digi") == 0) {
    char* mode = strtok(nullptr, " ");
    if (!mode) { printDigipeater(); }
    else if (strcmp(mode, "on") == 0)  { digi_.enabled = true;  saveSettings(); printDigipeater(); }
    else if (strcmp(mode, "off") == 0) { digi_.enabled = false; saveSettings(); printDigipeater(); }
    else if (strcmp(mode, "mode") == 0) {
      char* value = strtok(nullptr, " ");
      if (value && strcmp(value, "ui") == 0) {
        digi_.allFrames = false; saveSettings(); printDigipeater();
      } else if (value && strcmp(value, "all") == 0) {
        digi_.allFrames = true; saveSettings(); printDigipeater();
      } else {
        Serial.println("usage: digi mode <ui|all>");
      }
    }
    else Serial.println("usage: digi [on|off|mode ui|all]");

  } else if (strcmp(cmd, "digialias") == 0) {
    char* value = strtok(nullptr, " ");
    if (!value) { printDigipeater(); }
    else if (strcmp(value, "off") == 0) { digi_.hasAlias = false; saveSettings(); printDigipeater(); }
    else if (ax25::parseAddress(value, digi_.alias)) { digi_.hasAlias = true; saveSettings(); printDigipeater(); }
    else Serial.println("usage: digialias <CALLSIGN-SSID|off>");

  } else if (strcmp(cmd, "connect") == 0) {
    // connect [ch] <CALL>
    char* arg1 = strtok(nullptr, " ");
    char* arg2 = strtok(nullptr, " ");
    uint8_t chIdx = 0;
    const char* dest = arg1;
    if (arg2 != nullptr) {
      const long ch = atol(arg1);
      if (ch >= 1 && ch <= static_cast<long>(CHANNEL_COUNT)) {
        chIdx = static_cast<uint8_t>(ch - 1);
        dest  = arg2;
      }
    }
    Serial.println(connect(chIdx, dest) ? "connecting" : "connect failed");

  } else if (strcmp(cmd, "disconnect") == 0) {
    // disconnect [ch]
    char* arg = strtok(nullptr, " ");
    uint8_t chIdx = 0;
    if (arg != nullptr) {
      const long ch = atol(arg);
      if (ch >= 1 && ch <= static_cast<long>(CHANNEL_COUNT)) chIdx = static_cast<uint8_t>(ch - 1);
    }
    Serial.println(disconnect(chIdx) ? "disconnecting" : "disconnect failed");

  } else if (strcmp(cmd, "sendui") == 0) {
    char* dest = strtok(nullptr, " ");
    char* msg  = strtok(nullptr, "");
    Serial.println(sendUi(dest, msg) ? "ui queued" : "sendui failed");

  } else if (strcmp(cmd, "send") == 0) {
    // send [ch] <msg>
    char* arg1 = strtok(nullptr, " ");
    char* arg2 = strtok(nullptr, "");
    uint8_t chIdx = 0;
    const char* msg = arg1;
    if (arg2 != nullptr) {
      const long ch = atol(arg1);
      if (ch >= 1 && ch <= static_cast<long>(CHANNEL_COUNT)) {
        chIdx = static_cast<uint8_t>(ch - 1);
        msg   = arg2;
      }
    }
    if (sendConnected(chIdx, msg)) {
      Serial.printf("i queued ch%u depth=%u\n",
                    static_cast<unsigned>(chIdx + 1),
                    static_cast<unsigned>(channels_[chIdx].link.connectedQueueSize()));
    } else {
      Serial.println("send failed");
    }

  } else if (strcmp(cmd, "bbs") == 0) {
    // Local BBS list for sysop inspection
    char buf[512]{};
    mailbox_.list(buf, sizeof(buf));
    Serial.print(buf);

  } else if (strcmp(cmd, "stats") == 0) {
    printStats();

  } else {
    Serial.println("unknown command");
  }
}

// ---------------------------------------------------------------------------
// Info / Stats
// ---------------------------------------------------------------------------

void Tnc::printInfo() const {
  char local[12]{};
  ax25::formatAddress(local_, local, sizeof(local));
  Serial.printf("AXLoRaTNC local=%s variant=%s radio=%s\n",
                local, variant::NAME,
                variant::RADIO_TYPE == variant::RadioType::SX1262 ? "SX1262" : "SX1276");
  Serial.printf("  freq=%.*f MHz bw=%.1f kHz sf=%u cr=4/%u pwr=%d dBm\n",
                FREQUENCY_DECIMALS, static_cast<double>(radioConfig_.frequencyMHz),
                static_cast<double>(radioConfig_.bandwidthKhz),
                radioConfig_.spreadingFactor, radioConfig_.codingRate,
                radioConfig_.powerDbm);
  Serial.printf("  freq_corr=%+.1f kHz tuned=%.*f MHz\n",
                static_cast<double>(radioConfig_.frequencyCorrectionMHz * 1000.0f),
                FREQUENCY_DECIMALS, static_cast<double>(radioConfig_.frequencyMHz + radioConfig_.frequencyCorrectionMHz));
  Serial.printf("  duty=%s %.3f%%\n", dutyCycleEnabled_ ? "on" : "off",
                static_cast<double>(dutyCyclePpm_) / 10000.0);
  Serial.printf("serial mode=%s channels=%u baud_kiss=%lu baud_ded=%lu\n",
                serialModeName(), static_cast<unsigned>(CHANNEL_COUNT),
                static_cast<unsigned long>(baudKiss_), static_cast<unsigned long>(baudDed_));
  Serial.printf("ded host=%u selected_ch=%u last_I_ch=%u last_I_callsign=%s\n",
                dedHostMode_ ? 1U : 0U,
                static_cast<unsigned>(dedSelectedChannel_),
                static_cast<unsigned>(dedLastCallsignChannel_),
                dedLastCallsign_[0] != '\0' ? dedLastCallsign_ : "---");
  Serial.printf("ded last_C_ch=%u last_C_dest=%s\n",
                static_cast<unsigned>(dedLastConnectChannel_),
                dedLastConnect_[0] != '\0' ? dedLastConnect_ : "---");
}

void Tnc::printStats() const {
  const radio::Stats& rs = radio::stats();
  Serial.printf("tnc raw_tx=%lu raw_rx=%lu txq=%u/%u ptxq=%u/%u highwater=%lu queued=%lu deferred=%lu qdrops=%lu kiss_txdelay=%u p=%u slot=%u fulldup=%u\n",
                static_cast<unsigned long>(rawTx_), static_cast<unsigned long>(rawRx_),
                static_cast<unsigned>(rawTxQueue_.size()),
                static_cast<unsigned>(rawTxQueue_.capacity()),
                static_cast<unsigned>(rawTxPriorityQueue_.size()),
                static_cast<unsigned>(rawTxPriorityQueue_.capacity()),
                static_cast<unsigned long>(rawTxHighWater_),
                static_cast<unsigned long>(rawTxQueued_),
                static_cast<unsigned long>(rawTxDeferred_),
                static_cast<unsigned long>(rawTxQueueDrops_),
                kissParams_.txDelay, kissParams_.persistence, kissParams_.slotTime, kissParams_.fullDuplex);
  uint32_t ax25Retx = 0;
  uint32_t ax25T1Expired = 0;
  for (uint8_t i = 0; i < CHANNEL_COUNT; ++i) {
    const ax25::L2Stats& s = channels_[i].link.stats();
    ax25Retx += s.retx;
    ax25T1Expired += s.t1Expired;
  }
  Serial.printf("linfbb kiss_in_frames=%lu kiss_in_bytes=%lu kiss_decode_errors=%lu kiss_iframe_dupes=%lu ax25_iframes_in=%lu ax25_iframes_out=%lu ax25_ui_frames=%lu ax25_rr_rx=%lu ax25_rr_tx=%lu ax25_rej_rx=%lu ax25_rej_tx=%lu ax25_retx=%lu ax25_t1_expired=%lu\n",
                static_cast<unsigned long>(kissInFrames_),
                static_cast<unsigned long>(kissInBytes_),
                static_cast<unsigned long>(kissDecodeErrors_),
                static_cast<unsigned long>(kissIFrameDuplicates_),
                static_cast<unsigned long>(ax25IFramesIn_),
                static_cast<unsigned long>(ax25IFramesOut_),
                static_cast<unsigned long>(ax25UiFrames_),
                static_cast<unsigned long>(ax25RrRx_),
                static_cast<unsigned long>(ax25RrTx_),
                static_cast<unsigned long>(ax25RejRx_),
                static_cast<unsigned long>(ax25RejTx_),
                static_cast<unsigned long>(ax25Retx),
                static_cast<unsigned long>(ax25T1Expired));
  Serial.printf("linfbb radio_tx_started=%lu radio_tx_done=%lu radio_rx_frames=%lu radio_crc_fail=%lu txq_highwater=%lu qdrops=%lu kiss_out_frames=%lu kiss_out_partial_writes=%lu kiss_out_block_ms_max=%lu\n",
                static_cast<unsigned long>(radioTxStarted_),
                static_cast<unsigned long>(radioTxDone_),
                static_cast<unsigned long>(radioRxFrames_),
                static_cast<unsigned long>(radioCrcFail_),
                static_cast<unsigned long>(rawTxHighWater_),
                static_cast<unsigned long>(rawTxQueueDrops_),
                static_cast<unsigned long>(kissOutFrames_),
                static_cast<unsigned long>(kissOutPartialWrites_),
                static_cast<unsigned long>(kissOutBlockMsMax_));
  Serial.printf("digi enabled=%u mode=%s tx=%lu ui_tx=%lu conn_tx=%lu dupes=%lu drops=%lu\n",
                digi_.enabled, digi_.allFrames ? "all" : "ui",
                static_cast<unsigned long>(digiTx_),
                static_cast<unsigned long>(digiUiTx_),
                static_cast<unsigned long>(digiConnTx_),
                static_cast<unsigned long>(digiDupes_),
                static_cast<unsigned long>(digiDrops_));
  Serial.printf("beacon enabled=%u tx=%lu drops=%lu interval_s=%lu\n",
                beacon_.enabled,
                static_cast<unsigned long>(beaconTx_),
                static_cast<unsigned long>(beaconDrops_),
                static_cast<unsigned long>(beacon_.intervalMs / 1000));
  Serial.printf("netrom enabled=%u alias=%s broadcasts=%lu routes_heard=%lu\n",
                netrom_.enabled, netrom_.alias,
                static_cast<unsigned long>(netromBroadcasts_),
                static_cast<unsigned long>(netromRoutesHeard_));
  Serial.printf("radio tx_ok=%lu tx_fail=%lu rx_ok=%lu rx_fail=%lu duty_drops=%lu duty=%s %.3f%%\n",
                static_cast<unsigned long>(rs.txOk), static_cast<unsigned long>(rs.txFail),
                static_cast<unsigned long>(rs.rxOk), static_cast<unsigned long>(rs.rxFail),
                static_cast<unsigned long>(rs.dutyDrops),
                dutyCycleEnabled_ ? "on" : "off",
                static_cast<double>(dutyCyclePpm_) / 10000.0);
  for (uint8_t i = 0; i < CHANNEL_COUNT; ++i) {
    if (channels_[i].link.state() != ax25::LinkState::Disconnected) {
      Serial.printf("ch%u: ", static_cast<unsigned>(i + 1));
      channels_[i].link.printStats();
    }
  }
}

// ---------------------------------------------------------------------------
// Connect / disconnect / sendUi / sendConnected
// ---------------------------------------------------------------------------

bool Tnc::connect(uint8_t chIdx, const char* destination) {
  if (chIdx >= CHANNEL_COUNT || destination == nullptr) return false;
  ax25::Address dest{};
  return ax25::parseAddress(destination, dest) && connectChannel(chIdx, dest);
}

bool Tnc::connectChannel(uint8_t chIdx, const ax25::Address& destination) {
  if (kissTcpActive()) {
    LOG_WARN("local connection rejected: KISS TCP owns the radio");
    return false;
  }
  if (chIdx >= CHANNEL_COUNT ||
      channels_[chIdx].link.state() != ax25::LinkState::Disconnected ||
      channels_[chIdx].loopbackPeer < CHANNEL_COUNT) return false;
  bool localDestination = false;
  for (uint8_t i = 0; i < CHANNEL_COUNT; ++i) {
    if (!ax25::addressEquals(channels_[i].link.local(), destination)) continue;
    localDestination = true;
    if (i == chIdx || channels_[i].link.state() != ax25::LinkState::Disconnected ||
        channels_[i].loopbackPeer < CHANNEL_COUNT) continue;
    channels_[chIdx].loopbackPeer = i;
    channels_[i].loopbackPeer = chIdx;
    if (channels_[chIdx].link.connectTo(destination)) return true;
    channels_[chIdx].loopbackPeer = CHANNEL_COUNT;
    channels_[i].loopbackPeer = CHANNEL_COUNT;
    beginLink(chIdx, channels_[chIdx].link.local());
    return false;
  }
  // A local callsign must never fall through to an RF connection when busy.
  return !localDestination && channels_[chIdx].link.connectTo(destination);
}

bool Tnc::disconnect(uint8_t chIdx) {
  if (chIdx >= CHANNEL_COUNT) return false;
  return channels_[chIdx].link.disconnect();
}

bool Tnc::sendUi(const char* destination, const char* text) {
  ax25::Address dest{};
  if (!ax25::parseAddress(destination, dest) || text == nullptr) return false;
  return channels_[0].link.sendUi(dest, reinterpret_cast<const uint8_t*>(text), strlen(text));
}

bool Tnc::sendConnected(uint8_t chIdx, const char* text) {
  if (chIdx >= CHANNEL_COUNT || text == nullptr) return false;
  return sendConnectedChunked(chIdx, reinterpret_cast<const uint8_t*>(text), strlen(text));
}

// ---------------------------------------------------------------------------
// Link status event handling (all channels)
// ---------------------------------------------------------------------------

void Tnc::checkLinkStatusEvents() {
  for (uint8_t i = 0; i < CHANNEL_COUNT; ++i) {
    ChannelState& cs = channels_[i];
    const ax25::LinkState state = channels_[i].link.state();
    if (!dedUnattended_) cs.connectTextPending = false;
    if (cs.connectTextPending && state == ax25::LinkState::Connected) {
      char text[sizeof(dedUnattendedText_) + 1]{};
      snprintf(text, sizeof(text), "%s\r", dedUnattendedText_);
      if (sendConnectedChunked(i, reinterpret_cast<const uint8_t*>(text), strlen(text)))
        cs.connectTextPending = false;
    }
    const bool reset = cs.link.resetCount() != cs.lastResetCount;
    if (state == channels_[i].lastState && !reset) continue;
    const ax25::LinkState prevState = channels_[i].lastState;
    if (state == ax25::LinkState::Disconnected) {
      cs.connectTextPending = false;
      cs.initialRtt = dedFrack_;
      cs.retryLimit = dedRetryLimit_;
      cs.maxFrame = dedMaxFrame_;
      cs.link.setTimers(static_cast<uint32_t>(dedFrack_) * 10UL,
                        static_cast<uint32_t>(dedT2_) * 10UL,
                        static_cast<uint32_t>(dedT3_) * 10UL);
      cs.link.setRetryLimit(dedRetryLimit_);
      cs.link.setMaxFrame(dedMaxFrame_);
    }

    if (state == ax25::LinkState::Disconnected || state == ax25::LinkState::Recovery) {
      channels_[i].nodeGreetingSent = false;
      channels_[i].shellMode        = ShellMode::Node;
      channels_[i].nodeLinePos      = 0;
      channels_[i].composeBodyPos   = 0;
#if defined(AXLORA_VARIANT_SENSECAP_SOLAR)
      if (state == ax25::LinkState::Disconnected) {
        cs.relayGreetingPending = false;
        cs.relayInfoPending = false;
        cs.relayDisconnectPending = false;
        cs.relayMheardPending = false;
      }
#endif
    }

    const uint8_t channel = i + 1;
    if (serialMode_ == SerialMode::Console) {
      char peer[12]{};
      ax25::formatAddress(channels_[i].link.peer(), peer, sizeof(peer));
      switch (state) {
        case ax25::LinkState::Connecting:
          Serial.printf("*** ch%u Connecting to %s\n", channel, peer); break;
        case ax25::LinkState::Connected:
          Serial.printf("*** ch%u Connected to %s\n", channel, peer); break;
        case ax25::LinkState::Disconnecting:
          Serial.printf("*** ch%u Disconnecting from %s\n", channel, peer); break;
        case ax25::LinkState::Disconnected:
          Serial.printf("*** ch%u Disconnected from %s\n", channel, peer); break;
        case ax25::LinkState::Recovery:
          Serial.printf("*** ch%u Link failure with %s\n", channel, peer); break;
        default: break;
      }
    }

    {
      char peer[12]{};
      ax25::formatAddress(channels_[i].link.peer(), peer, sizeof(peer));
      const char* p = peer[0] != '\0' ? peer : "-";
      char text[160]{};
      switch (state) {
        case ax25::LinkState::Connecting:
          break;
        case ax25::LinkState::Connected:
          // Only fire "CONNECTED" for genuinely new connections, not Recovery→Connected.
          if (prevState == ax25::LinkState::Disconnected ||
              prevState == ax25::LinkState::Connecting) {
            snprintf(text, sizeof(text), "(%u) CONNECTED to %s", channel, p);
          } else if (reset) {
            snprintf(text, sizeof(text), "(%u) LINK RESET fm %s", channel, p);
          }
          break;
        case ax25::LinkState::Disconnecting:
          break;
        case ax25::LinkState::Disconnected:
          snprintf(text, sizeof(text), "(%u) %s %s", channel, cs.link.disconnectReason(), p);
          break;
        case ax25::LinkState::Recovery:
          break;
        default:
          break;
      }
      if (text[0] != '\0') {
        for (uint8_t n = 0; n < cs.link.pathCount(); ++n) {
          char call[12]{};
          ax25::formatAddress(cs.link.path()[n], call, sizeof(call));
          const size_t pos = strlen(text);
          snprintf(text + pos, sizeof(text) - pos, "%s%s", n == 0 ? " via " : " ", call);
        }
        if (state == ax25::LinkState::Disconnected &&
            strncmp(cs.link.disconnectReason(), "FRAME REJECT", 12) == 0) {
          const uint8_t* rejected = cs.link.rejectedFrame();
          const size_t pos = strlen(text);
          snprintf(text + pos, sizeof(text) - pos, " (%02X%02X%02X)",
                   rejected[0], rejected[1], rejected[2]);
        }
        if (dedTimestamp_ != 0) {
          char stamp[24]{};
          formatDedTimestamp(stamp, sizeof(stamp));
          const size_t pos = strlen(text);
          snprintf(text + pos, sizeof(text) - pos, " - %s", stamp);
        }
        if (
#if defined(AXLORA_VARIANT_SENSECAP_SOLAR)
            serialMode_ == SerialMode::Wa8ded &&
#endif
            !enqueueDedEvent(channel, 3, reinterpret_cast<const uint8_t*>(text), strlen(text)))
          continue;
      }
    }
    channels_[i].lastState = state;
    cs.lastResetCount = cs.link.resetCount();
#if defined(AXLORA_VARIANT_SENSECAP_SOLAR)
    if (serialMode_ == SerialMode::Console && state == ax25::LinkState::Connected &&
        prevState == ax25::LinkState::Disconnected) {
      cs.relayGreetingPending = true;
    }
#endif
    if (serialMode_ == SerialMode::Wa8ded && dedUnattended_ &&
        state == ax25::LinkState::Connected && prevState == ax25::LinkState::Disconnected &&
        dedUnattendedText_[0] != '\0') {
      cs.connectTextPending = true;
    }
  }
}

// ---------------------------------------------------------------------------
// Node / BBS shell
// ---------------------------------------------------------------------------

void Tnc::processNodeInput(uint8_t chIdx, const uint8_t* data, size_t len) {
  ChannelState& cs = channels_[chIdx];
#if !defined(AXLORA_VARIANT_SENSECAP_SOLAR)
  if (!cs.nodeGreetingSent) {
    sendNodeText(chIdx, "*** AXLoRaTNC\rType ? for commands or BBS for mailbox\r");
    cs.nodeGreetingSent = true;
  }
#endif
  for (size_t i = 0; i < len; ++i) {
    const uint8_t byte = data[i];
    const char c = static_cast<char>(byte);
    if (c == '\r' || c == '\n') {
      cs.nodeLine[cs.nodeLinePos] = '\0';
#if defined(AXLORA_VARIANT_SENSECAP_SOLAR)
      if (strcasecmp(cs.nodeLine, "//MH") == 0 ||
          strcasecmp(cs.nodeLine, "//MHEARD") == 0) {
        cs.relayMheardPending = true;
        cs.relayMheardIndex = 0;
      } else if (strcasecmp(cs.nodeLine, "//I") == 0) {
        cs.relayInfoPending = true;
      } else if (strcasecmp(cs.nodeLine, "//Q") == 0) {
        cs.relayDisconnectPending = true;
        cs.nodeLinePos = 0;
        return;
      }
#else
      switch (cs.shellMode) {
        case ShellMode::Node:       handleNodeLine(chIdx, cs.nodeLine);        break;
        case ShellMode::Bbs:        handleBbsLine(chIdx, cs.nodeLine);         break;
        case ShellMode::BbsCompose: handleBbsComposeLine(chIdx, cs.nodeLine);  break;
      }
#endif
      cs.nodeLinePos = 0;
      continue;
    }
    if (byte == 0x08 || byte == 0x7F) {
      if (cs.nodeLinePos > 0) --cs.nodeLinePos;
      continue;
    }
    if (!isNodeInputTextByte(byte)) continue;
    if (cs.nodeLinePos < sizeof(cs.nodeLine) - 1)
      cs.nodeLine[cs.nodeLinePos++] = c;
  }
}

#if defined(AXLORA_VARIANT_SENSECAP_SOLAR)
void Tnc::updateRelayUptime() {
  const uint32_t now = relayUptimeTick();
  relayUptimeTicks_ += static_cast<uint32_t>(now - relayLastUptimeTicks_);
  relayLastUptimeTicks_ = now;
}

void Tnc::serviceRelayResponses(uint8_t chIdx) {
  ChannelState& cs = channels_[chIdx];
  if (cs.link.state() != ax25::LinkState::Connected) return;
  if (cs.relayDisconnectPending) {
    if (!dedTxEnabled_) return;
    rawTxQueueDrops_ += static_cast<uint32_t>(rawTxQueue_.removeChannel(chIdx));
    rawTxQueueDrops_ += static_cast<uint32_t>(rawTxPriorityQueue_.removeChannel(chIdx));
    if (rawTxPriorityQueue_.free() == 0) return;
    if (!cs.link.disconnect()) {
      LOG_ERR("relay disconnect failed on channel %u", static_cast<unsigned>(chIdx + 1));
    }
    cs.relayDisconnectPending = false;
    return;
  }
  if (cs.relayGreetingPending) {
    char text[100]{};
    snprintf(text, sizeof(text), "This station is managed by %s.\r", manager_);
    if (!sendConnectedChunked(chIdx, reinterpret_cast<const uint8_t*>(text), strlen(text))) return;
    cs.relayGreetingPending = false;
  }
  if (cs.relayInfoPending) {
    if (dedConnectedAcceptFrameCapacity(chIdx) == 0) return;
    uint16_t millivolts = 0;
    char battery[24]{};
    if (variant::readBatteryMillivolts(millivolts)) {
      snprintf(battery, sizeof(battery), "%u.%03u V (%u%%)",
               static_cast<unsigned>(millivolts / 1000),
               static_cast<unsigned>(millivolts % 1000),
               static_cast<unsigned>(variant::batteryPercentageFromMillivolts(millivolts)));
    } else {
      snprintf(battery, sizeof(battery), "unavailable");
    }
    updateRelayUptime();
    const uint64_t totalMinutes = relayUptimeTicks_ / (RELAY_TICKS_PER_SECOND * 60UL);
    const unsigned long totalHours = static_cast<unsigned long>(totalMinutes / 60);
    char uptime[48]{};
    if (totalHours >= 24) {
      const unsigned long days = totalHours / 24;
      const unsigned long hours = totalHours % 24;
      snprintf(uptime, sizeof(uptime), "%lu day%s and %02lu hour%s",
               days, days == 1 ? "" : "s", hours, hours == 1 ? "" : "s");
    } else {
      const unsigned long minutes = static_cast<unsigned long>(totalMinutes % 60);
      snprintf(uptime, sizeof(uptime), "%02lu hour%s and %02lu minute%s",
               totalHours, totalHours == 1 ? "" : "s",
               minutes, minutes == 1 ? "" : "s");
    }
    const radio::Stats& stats = radio::stats();
    char text[128]{};
    snprintf(text, sizeof(text), "Battery: %s  TX: %lu  RX: %lu  Uptime: %s\r", battery,
             static_cast<unsigned long>(stats.txOk), static_cast<unsigned long>(stats.rxOk),
             uptime);
    if (!sendConnectedChunked(chIdx, reinterpret_cast<const uint8_t*>(text), strlen(text))) return;
    cs.relayInfoPending = false;
  }
  if (!cs.relayMheardPending) return;
  if (cs.relayMheardIndex == 0) {
    const char* header = "callsign   dest       first    last     fr  rssi  snr\r";
    if (!sendConnectedChunked(chIdx, reinterpret_cast<const uint8_t*>(header), strlen(header))) return;
    ++cs.relayMheardIndex;
  }
  while (cs.relayMheardIndex <= sizeof(mheard_) / sizeof(mheard_[0])) {
    const MheardEntry& entry = mheard_[cs.relayMheardIndex - 1];
    if (entry.active) {
      char src[12]{}, dst[12]{}, first[16]{}, last[16]{}, row[96]{};
      ax25::formatAddress(entry.source, src, sizeof(src));
      ax25::formatAddress(entry.destination, dst, sizeof(dst));
      formatUptime(entry.firstHeardMs, first, sizeof(first));
      formatUptime(entry.lastHeardMs, last, sizeof(last));
      snprintf(row, sizeof(row), "%-10s %-10s %s %s %3lu %5.1f %5.1f\r",
               src, dst, first, last, static_cast<unsigned long>(entry.frames),
               static_cast<double>(entry.lastRssi), static_cast<double>(entry.lastSnr));
      if (!sendConnectedChunked(chIdx, reinterpret_cast<const uint8_t*>(row), strlen(row))) return;
    }
    ++cs.relayMheardIndex;
  }
  cs.relayMheardPending = false;
}
#endif

void Tnc::handleNodeLine(uint8_t chIdx, const char* line) {
  // Build uppercase command token
  char cmd[16]{};
  size_t i = 0;
  while (line[i] && line[i] != ' ' && i < sizeof(cmd) - 1) {
    const char c = line[i];
    cmd[i] = (c >= 'a' && c <= 'z') ? static_cast<char>(c - 32) : c;
    ++i;
  }

  if (cmd[0] == '\0' || strcmp(cmd, "?") == 0) {
    sendNodeText(chIdx, "Commands: ?, INFO, NODES, ROUTES, MHEARD, BBS, BYE\r");
  } else if (strcmp(cmd, "INFO") == 0) {
    sendNodeText(chIdx, netrom_.ident);
    sendNodeText(chIdx, "\r");
  } else if (strcmp(cmd, "NODES") == 0 || strcmp(cmd, "ROUTES") == 0) {
    sendNodeText(chIdx, "Known nodes:\r");
    for (const NetromRoute& route : netromRoutes_) {
      if (!route.active) continue;
      char node[12]{}, via[12]{};
      ax25::formatAddress(route.node, node, sizeof(node));
      ax25::formatAddress(route.heardFrom, via, sizeof(via));
      char row[64]{};
      snprintf(row, sizeof(row), "%-6s %-10s via %-10s q=%u\r", route.alias, node, via, route.quality);
      sendNodeText(chIdx, row);
    }
  } else if (strcmp(cmd, "MHEARD") == 0) {
    sendNodeText(chIdx, "callsign   dest       first    last     fr  rssi  snr\r");
    for (const MheardEntry& entry : mheard_) {
      if (!entry.active) continue;
      char src[12]{}, dst[12]{}, first[10]{}, last[10]{};
      ax25::formatAddress(entry.source,      src, sizeof(src));
      ax25::formatAddress(entry.destination, dst, sizeof(dst));
      formatUptime(entry.firstHeardMs, first, sizeof(first));
      formatUptime(entry.lastHeardMs,  last,  sizeof(last));
      char row[80]{};
      snprintf(row, sizeof(row), "%-10s %-10s %s %s %3lu %5.1f %5.1f\r",
               src, dst, first, last,
               static_cast<unsigned long>(entry.frames),
               static_cast<double>(entry.lastRssi),
               static_cast<double>(entry.lastSnr));
      sendNodeText(chIdx, row);
    }
  } else if (strcmp(cmd, "BBS") == 0) {
    channels_[chIdx].shellMode = ShellMode::Bbs;
    sendNodeText(chIdx, "*** BBS - type ? for help\r");
  } else if (strcmp(cmd, "BYE") == 0 || strcmp(cmd, "B") == 0) {
    sendNodeText(chIdx, "73\r");
    disconnect(chIdx);
  } else {
    sendNodeText(chIdx, "Unknown command. Type ?\r");
  }
}

void Tnc::handleBbsLine(uint8_t chIdx, const char* line) {
  // Build uppercase command token
  char cmd[16]{};
  const char* args = line;
  size_t i = 0;
  while (args[i] && args[i] != ' ' && i < sizeof(cmd) - 1) {
    const char c = args[i];
    cmd[i] = (c >= 'a' && c <= 'z') ? static_cast<char>(c - 32) : c;
    ++i;
  }
  args = (args[i] == ' ') ? args + i + 1 : args + i;  // rest of line after command

  char peerCallsign[12]{};
  ax25::formatAddress(channels_[chIdx].link.peer(), peerCallsign, sizeof(peerCallsign));
  // strip SSID from peer for BBS from-field
  for (size_t j = 0; j < sizeof(peerCallsign); ++j) {
    if (peerCallsign[j] == '-') { peerCallsign[j] = '\0'; break; }
  }

  if (cmd[0] == '\0' || strcmp(cmd, "?") == 0 || strcmp(cmd, "H") == 0) {
    sendNodeText(chIdx, "BBS commands:\r L  - list all\r LT - list mine\r R n - read\r");
    sendNodeText(chIdx, " S CALL - send\r K n - kill\r X - exit BBS\r B - bye\r");

  } else if (strcmp(cmd, "L") == 0) {
    const char* filter = (args[0] == 'T' || args[0] == 't') ? peerCallsign : nullptr;
    char buf[512]{};
    mailbox_.list(buf, sizeof(buf), filter);
    sendNodeText(chIdx, buf);

  } else if (strcmp(cmd, "R") == 0) {
    const long idx = atol(args);
    const Mailbox::Message* m = mailbox_.get(static_cast<uint8_t>(idx - 1));
    if (m == nullptr) {
      sendNodeText(chIdx, "Message not found\r");
    } else {
      char hdr[64]{};
      snprintf(hdr, sizeof(hdr), "From: %-10s To: %s\r", m->from, m->to);
      sendNodeText(chIdx, hdr);
      sendNodeText(chIdx, m->body);
      sendNodeText(chIdx, "\r");
      mailbox_.markRead(static_cast<uint8_t>(idx - 1));
    }

  } else if (strcmp(cmd, "S") == 0) {
    if (args[0] == '\0') {
      sendNodeText(chIdx, "usage: S CALLSIGN\r");
    } else {
      ChannelState& cs = channels_[chIdx];
      strncpy(cs.composeTo, args, sizeof(cs.composeTo) - 1);
      cs.composeTo[sizeof(cs.composeTo) - 1] = '\0';
      cs.composeBodyPos = 0;
      cs.composeBody[0] = '\0';
      cs.shellMode = ShellMode::BbsCompose;
      sendNodeText(chIdx, "Enter message, empty line to send:\r");
    }

  } else if (strcmp(cmd, "K") == 0) {
    const long idx = atol(args);
    if (mailbox_.kill(static_cast<uint8_t>(idx - 1), peerCallsign)) {
      sendNodeText(chIdx, "Deleted\r");
    } else {
      sendNodeText(chIdx, "Cannot delete\r");
    }

  } else if (strcmp(cmd, "X") == 0 || strcmp(cmd, "EXIT") == 0) {
    channels_[chIdx].shellMode = ShellMode::Node;
    sendNodeText(chIdx, "Back to node shell. Type ?\r");

  } else if (strcmp(cmd, "B") == 0 || strcmp(cmd, "BYE") == 0) {
    sendNodeText(chIdx, "73\r");
    disconnect(chIdx);

  } else {
    sendNodeText(chIdx, "Unknown BBS command. Type ?\r");
  }
}

void Tnc::handleBbsComposeLine(uint8_t chIdx, const char* line) {
  ChannelState& cs = channels_[chIdx];
  if (line[0] == '\0') {
    // Empty line = end of message
    char peerCallsign[12]{};
    ax25::formatAddress(channels_[chIdx].link.peer(), peerCallsign, sizeof(peerCallsign));
    for (size_t j = 0; j < sizeof(peerCallsign); ++j) {
      if (peerCallsign[j] == '-') { peerCallsign[j] = '\0'; break; }
    }
    if (mailbox_.post(peerCallsign, cs.composeTo, cs.composeBody)) {
      sendNodeText(chIdx, "Message saved\r");
    } else {
      sendNodeText(chIdx, "Mailbox full\r");
    }
    cs.composeBodyPos = 0;
    cs.composeBody[0] = '\0';
    cs.shellMode = ShellMode::Bbs;
    return;
  }
  // Append line to body
  const size_t lineLen = strlen(line);
  const size_t remaining = Mailbox::MAX_BODY - cs.composeBodyPos;
  if (remaining < 2) {
    sendNodeText(chIdx, "Message too long, saved\r");
    // Force save
    char peerCallsign[12]{};
    ax25::formatAddress(channels_[chIdx].link.peer(), peerCallsign, sizeof(peerCallsign));
    for (size_t j = 0; j < sizeof(peerCallsign); ++j) {
      if (peerCallsign[j] == '-') { peerCallsign[j] = '\0'; break; }
    }
    mailbox_.post(peerCallsign, cs.composeTo, cs.composeBody);
    cs.composeBodyPos = 0;
    cs.composeBody[0] = '\0';
    cs.shellMode = ShellMode::Bbs;
    return;
  }
  const size_t copy = lineLen < remaining - 1 ? lineLen : remaining - 1;
  memcpy(cs.composeBody + cs.composeBodyPos, line, copy);
  cs.composeBodyPos += copy;
  if (cs.composeBodyPos < Mailbox::MAX_BODY) {
    cs.composeBody[cs.composeBodyPos++] = '\n';
  }
  cs.composeBody[cs.composeBodyPos] = '\0';
}

void Tnc::sendNodeText(uint8_t chIdx, const char* text) {
  if (text == nullptr) return;
  uint8_t chunk[64]{};
  size_t pos = 0;
  for (const char* p = text; *p != '\0'; ++p) {
    chunk[pos++] = sanitizeNodeTextByte(static_cast<uint8_t>(*p));
    if (pos == sizeof(chunk)) {
      sendConnectedChunked(chIdx, chunk, pos);
      pos = 0;
    }
  }
  if (pos > 0) {
    sendConnectedChunked(chIdx, chunk, pos);
  }
}

bool Tnc::sendConnectedChunked(uint8_t chIdx, const uint8_t* data, size_t len) {
  if (chIdx >= CHANNEL_COUNT || data == nullptr || len == 0) return false;
  const size_t paclen = ax25::MAX_INFO_LEN;
  const size_t neededFrames = (len + paclen - 1) / paclen;
  if (neededFrames > dedConnectedAcceptFrameCapacity(chIdx)) {
    return false;
  }
  size_t offset = 0;
  bool allQueued = true;
  while (offset < len) {
    const size_t chunk = (len - offset) > paclen ? paclen : (len - offset);
    if (!channels_[chIdx].link.sendConnected(data + offset, chunk)) {
      allQueued = false;
      break;
    }
    offset += chunk;
  }
  return allQueued;
}

// ---------------------------------------------------------------------------
// WA8DED hostmode
// ---------------------------------------------------------------------------

void Tnc::serviceWa8ded(uint8_t byte) {
  if (!dedHostMode_) {
    serviceWa8dedTerminal(byte);
    return;
  }
  serviceWa8dedHost(byte);
}

void Tnc::serviceWa8dedHost(uint8_t byte) {
  dedLastByteMs_ = millis();
  // CAN/ESC resynchronization is only safe where a channel byte is expected.
  if (dedHeaderPos_ == 0) {
    if (byte == 0x18) return;
    if (byte == 0x1B) {
      dedHostMode_ = false;
      dedDataPos_ = 0;
      dedDataLen_ = 0;
      linePos_ = 0;
      dedTerminalCommand_ = false;
      dedOutputPaused_ = false;
      serviceWa8dedTerminal(byte);
      return;
    }
  }
  // XON/XOFF only safe to intercept at frame boundary (channel bytes 0x11/0x13 are out of range)
  if (dedXonXoff_ && dedHeaderPos_ == 0) {
    if (byte == 0x13) { dedOutputPaused_ = true;  return; }
    if (byte == 0x11) { dedOutputPaused_ = false; return; }
  }
  if (dedHeaderPos_ < sizeof(dedHeader_)) {
    dedHeader_[dedHeaderPos_++] = byte;
    if (dedHeaderPos_ == sizeof(dedHeader_)) {
      dedDataLen_ = static_cast<size_t>(dedHeader_[2]) + 1;
      dedDataPos_ = 0;
    }
    return;
  }
  if (dedDataPos_ < sizeof(dedData_)) dedData_[dedDataPos_++] = byte;
  if (dedDataPos_ >= dedDataLen_) {
    handleDedHostFrame(dedHeader_[0], dedHeader_[1], dedData_, dedDataLen_);
    dedHeaderPos_ = 0; dedDataPos_ = 0; dedDataLen_ = 0;
  }
}

void Tnc::serviceWa8dedTerminal(uint8_t byte) {
  if (dedXonXoff_ && byte == 0x13) { dedOutputPaused_ = true; return; }
  if (dedXonXoff_ && byte == 0x11) { dedOutputPaused_ = false; return; }

  if (byte == 0x1B && linePos_ == 0) {
    dedTerminalCommand_ = true;
    if (dedEcho_) Serial.print("* ");
    return;
  }
  if (byte == '\n') return;
  if (byte == '\r') {
    line_[linePos_] = '\0';
    if (dedEcho_) Serial.print(dedAutoLf_ ? "\r\n" : "\r");
    handleDedTerminalLine(line_, dedTerminalCommand_);
    linePos_ = 0;
    dedTerminalCommand_ = false;
    return;
  }
  if (byte == 0x08 || byte == 0x7F) {
    if (linePos_ > 0) {
      --linePos_;
      if (dedEcho_) Serial.print("\b \b");
    }
    return;
  }
  if (byte == 0x15 || byte == 0x18) {
    while (linePos_ > 0) {
      --linePos_;
      if (dedEcho_) Serial.print("\b \b");
    }
    return;
  }
  if (linePos_ >= sizeof(line_) - 1) {
    Serial.write(static_cast<uint8_t>(0x07));
    return;
  }
  line_[linePos_++] = static_cast<char>(byte);
  if (dedEcho_) Serial.write(byte);
}

void Tnc::handleDedTerminalLine(const char* line, bool command) {
  if (line == nullptr) return;
  if (!command) {
    char text[sizeof(line_) + 1]{};
    snprintf(text, sizeof(text), "%s\r", line);
    const size_t len = strlen(text);
    if (dedSelectedChannel_ == 0) {
      if (!channels_[0].link.sendUi(dedUnprotoDestination_,
                                    reinterpret_cast<const uint8_t*>(text), len,
                                    dedUnprotoPath_, dedUnprotoPathCount_, dedUiPoll_, &local_)) {
        Serial.print("TNC BUSY - LINE IGNORED\r\n");
      }
      return;
    }
    const uint8_t chIdx = dedSelectedChannel_ - 1;
    bool ok = false;
    if (channels_[chIdx].link.state() == ax25::LinkState::Connected) {
      ok = sendConnectedChunked(chIdx, reinterpret_cast<const uint8_t*>(text), len);
    } else {
      ok = channels_[chIdx].link.sendUi(dedUnprotoDestination_,
                                        reinterpret_cast<const uint8_t*>(text), len,
                                        dedUnprotoPath_, dedUnprotoPathCount_, dedUiPoll_);
    }
    if (!ok) Serial.print("TNC BUSY - LINE IGNORED\r\n");
    return;
  }

  char cmd[256]{};
  strncpy(cmd, line, sizeof(cmd) - 1);
  if (strcasecmp(cmd, "CONSOLE") == 0) {
    Serial.print("mode=console\r\n");
    Serial.flush();
    setSerialMode(SerialMode::Console);
    saveSerialMode(SerialMode::Console);
    return;
  }
  if (strncasecmp(cmd, "MYCALL", 6) == 0) {
    const char* cs = skipSpaces(cmd + 6);
    char query[256]{};
    snprintf(query, sizeof(query), "I %s", cs);
    handleDedCommand(dedSelectedChannel_, query, strlen(query));
    return;
  }
  if (strncasecmp(cmd, "CONNECT", 7) == 0 && (cmd[7] == '\0' || cmd[7] == ' ')) {
    char query[256]{};
    snprintf(query, sizeof(query), "C %s", skipSpaces(cmd + 7));
    handleDedCommand(dedSelectedChannel_, query, strlen(query));
    return;
  }
  if (strcasecmp(cmd, "DISCONNECT") == 0) {
    handleDedCommand(dedSelectedChannel_, "D", 1);
    return;
  }

  // UNPROTO [dest [VIA path]]  — set unproto destination + optional via path for UI frames
  if (strncasecmp(cmd, "UNPROTO", 7) == 0) {
    const char* rest = skipSpaces(cmd + 7);
    char query[256]{};
    snprintf(query, sizeof(query), "C %s", rest);
    handleDedCommand(0, query, strlen(query));
    return;
  }

  // BTEXT [text]  — get/set beacon text
  if (strncasecmp(cmd, "BTEXT", 5) == 0) {
    const char* rest = skipSpaces(cmd + 5);
    if (*rest == '\0') {
      Serial.printf("BTEXT %s\r\n", beacon_.text);
      return;
    }
    copySanitizedNodeText(beacon_.text, sizeof(beacon_.text), rest);
    saveSettings();
    return;
  }

  handleDedCommand(dedSelectedChannel_, cmd, strlen(cmd));
}

void Tnc::serviceDedTerminalOutput() {
  if (dedOutputPaused_) return;
  size_t i = 0;
  while (i < dedEvents_.size() && !dedOutputPaused_) {
    DedEvent event{};
    if (!dedEvents_.peek(i, event)) return;
    if (event.channel != dedSelectedChannel_ && event.channel != 0) { ++i; continue; }
    if (!dedEvents_.remove(i, event)) return;
    if (event.code == 3) {
      event.data[event.len < sizeof(event.data) ? event.len : sizeof(event.data) - 1] = '\0';
      Serial.printf("* %s *\a\r\n", reinterpret_cast<const char*>(event.data));
    } else if (event.code == 4 || event.code == 5) {
      event.data[event.len < sizeof(event.data) ? event.len : sizeof(event.data) - 1] = '\0';
      Serial.print(reinterpret_cast<const char*>(event.data));
      Serial.print("\r\n");
    } else if (event.code == 6 || event.code == 7) {
      if (event.code == 6) dedMonitorPayloadPending_ = false;
      Serial.write(event.data, event.len);
      Serial.print("\r\n");
    }
  }
}

void Tnc::applyDedLinkConfig() {
  const uint32_t t2Ms = static_cast<uint32_t>(dedT2_) * 10UL;
  const uint32_t t3Ms = static_cast<uint32_t>(dedT3_) * 10UL;
  for (uint8_t i = 0; i < CHANNEL_COUNT; ++i) {
    channels_[i].link.setTimers(static_cast<uint32_t>(channels_[i].initialRtt) * 10UL, t2Ms, t3Ms);
    channels_[i].link.setRetryLimit(channels_[i].retryLimit);
    channels_[i].link.setIPollLength(dedIPollFrameLength_);
    channels_[i].link.setMaxFrame(channels_[i].maxFrame);
  }
}

void Tnc::printDedTerminalStatus(int channel) const {
  const int first = channel < 0 ? 0 : channel;
  const int last = channel < 0 ? CHANNEL_COUNT : channel;
  if (first < 0 || last > CHANNEL_COUNT) {
    Serial.print("? INVALID CHANNEL\r\n");
    return;
  }
  for (int ch = first; ch <= last; ++ch) {
    const char marker = ch == dedSelectedChannel_ ? '+' : ' ';
    if (ch == 0) {
      char dest[12]{};
      ax25::formatAddress(dedUnprotoDestination_, dest, sizeof(dest));
      Serial.printf("%c0 %s 0 0 0 0\r\n", marker, dest);
      continue;
    }
    const uint8_t chIdx = static_cast<uint8_t>(ch - 1);
    char peer[12]{};
    ax25::formatAddress(channels_[chIdx].link.peer(), peer, sizeof(peer));
    const bool connected = channels_[chIdx].link.state() == ax25::LinkState::Connected;
    Serial.printf("%c%d %s %u %u %u %u\r\n",
                  marker, ch, connected ? peer : "-",
                  connected ? 1U : 0U,
                  static_cast<unsigned>(channels_[chIdx].link.connectedQueueSize()),
                  0U, 0U);
  }
}

void Tnc::handleDedHostFrame(uint8_t channel, uint8_t infoCmd,
                              const uint8_t* data, size_t len) {
  if (channel > CHANNEL_COUNT &&
      !(channel == 255 && infoCmd == 1 && len == 1 && toupper(data[0]) == 'G')) {
    sendDedText(channel, 2, "INVALID CHANNEL NUMBER");
    return;
  }
  if (infoCmd == 0) {
    // Data frame
    if (channel == 0) {
      if (channels_[0].link.sendUi(dedUnprotoDestination_, data, len,
                                   dedUnprotoPath_, dedUnprotoPathCount_, dedUiPoll_, &local_))
        sendDedShort(channel, 0);
      else sendDedText(channel, 2, "TNC BUSY - LINE IGNORED");
      return;
    }
    if (channel >= 1 && channel <= CHANNEL_COUNT) {
      const uint8_t chIdx = channel - 1;
      if (channels_[chIdx].link.state() == ax25::LinkState::Connected ||
          channels_[chIdx].link.state() == ax25::LinkState::Recovery) {
        if (sendConnectedChunked(chIdx, data, len)) sendDedShort(channel, 0);
        else sendDedText(channel, 2, "TNC BUSY - LINE IGNORED");
      } else {
        // TF accepts and discards host data on a disconnected channel.
        sendDedShort(channel, 0);
      }
      return;
    }
    sendDedText(channel, 2, "INVALID CHANNEL");
    return;
  }
  if (infoCmd == 1) {
    handleDedCommand(channel, reinterpret_cast<const char*>(data), len);
    return;
  }
  sendDedText(channel, 2, "INVALID COMMAND");
}

void Tnc::handleDedCommand(uint8_t channel, const char* command, size_t len) {
  char cmd[257]{};
  if (len >= sizeof(cmd)) {
    sendDedText(channel, 2, "INVALID PARAMETER");
    return;
  }
  const size_t copyLen = len < sizeof(cmd) ? len : sizeof(cmd) - 1;
  memcpy(cmd, command, copyLen);
  size_t cmdLen = strlen(cmd);
  while (cmdLen > 0 && (cmd[cmdLen - 1] == '\r' || cmd[cmdLen - 1] == '\n'))
    cmd[--cmdLen] = '\0';
  const char* first = skipSpaces(cmd);
  if (first != cmd) memmove(cmd, first, strlen(first) + 1);
  cmd[0] = static_cast<char>(toupper(static_cast<unsigned char>(cmd[0])));
  if (cmd[0] == 'J' || cmd[0] == '@') {
    for (size_t i = 1; cmd[i] != '\0' && cmd[i] != ' '; ++i)
      cmd[i] = static_cast<char>(toupper(static_cast<unsigned char>(cmd[i])));
  }
  auto numberReply = [&](unsigned value) {
    char text[16]{};
    snprintf(text, sizeof(text), "%u", value);
    sendDedText(channel, 1, text);
  };
  auto invalidCommand = [&](bool extended = false) {
    char text[48]{};
    snprintf(text, sizeof(text), extended ? "INVALID EXTENDED COMMAND: %c" : "INVALID COMMAND: %c",
             extended ? cmd[1] : cmd[0]);
    sendDedText(channel, 2, text);
  };
  auto parseValue = [&](const char* text, unsigned maximum, unsigned& value,
                        unsigned minimum = 0) {
    if (!ded::parseNumber(text, UINT32_MAX, value)) {
      sendDedText(channel, 2, "INVALID PARAMETER");
      return false;
    }
    if (value < minimum || value > maximum) {
      char error[48]{};
      snprintf(error, sizeof(error), "INVALID VALUE: %u", value);
      sendDedText(channel, 2, error);
      return false;
    }
    return true;
  };
  if (cmd[0] == '\0') { sendDedShort(channel, 0); return; }

  if (cmd[0] == 'G') {
    if (!dedHostMode_) { invalidCommand(); return; }
    unsigned selector = 2;
    const char* arg = skipSpaces(cmd + 1);
    if (*arg != '\0' && !parseValue(arg, 1, selector)) return;
    if (channel == 255) {
      char pending[CHANNEL_COUNT + 2]{};
      size_t count = 0;
      for (uint8_t ch = 0; ch <= CHANNEL_COUNT; ++ch) {
        if (pendingDedEvents(ch, 0) != 0) pending[count++] = static_cast<char>(ch + 1);
      }
      sendDedText(channel, 1, pending);
      return;
    }
    DedEvent event{};
    uint8_t wanted = selector == 1 ? 3 : 0;
    if (selector == 0 && channel != 0) wanted = 7;
    if (channel == 0 && selector != 1) {
      if (dedMonitorPayloadPending_) wanted = 6;
      else if (selector == 0) wanted = 4;
      else if (pendingDedEvents(0, 3) != 0) wanted = 3;
      else wanted = 4;
    }
    if (popDedEvent(channel, wanted, event)) {
      if (event.code == 5) dedMonitorPayloadPending_ = true;
      if (event.code == 6) dedMonitorPayloadPending_ = false;
      if (event.code == 6 || event.code == 7) {
        sendDedCounted(event.channel, event.code, event.data, event.len);
      } else if (event.code >= 1 && event.code <= 5) {
        event.data[event.len < sizeof(event.data) ? event.len : sizeof(event.data) - 1] = '\0';
        sendDedText(event.channel, event.code, reinterpret_cast<const char*>(event.data));
      } else {
        sendDedShort(channel, event.code);
      }
    } else {
      sendDedShort(channel, 0);
    }
    return;
  }

  if (strncmp(cmd, "JHOST", 5) == 0) {
    const char* arg = skipSpaces(cmd + 5);
    if (*arg == '\0') { numberReply(dedHostMode_ ? 1U : 0U); return; }
    unsigned value = 0;
    if (!parseValue(arg, 1, value)) return;
    sendDedShort(channel, 0);
    if (value == 0) {
      dedHostMode_ = false;
      dedHeaderPos_ = 0; dedDataPos_ = 0; dedDataLen_ = 0;
      return;
    }
    if (value == 1) {
      dedHostMode_ = true;
      saveSerialMode(SerialMode::Wa8ded);
      return;
    }
    sendDedText(channel, 2, "INVALID JHOST");
    return;
  }

  if (cmd[0] == 'C') {
    const char* dest = skipSpaces(cmd + 1);
    if (*dest == '\0') {
      char text[128]{}, address[12]{};
      if (channel != 0 && channels_[channel - 1].link.state() == ax25::LinkState::Disconnected) {
        sendDedText(channel, 1, "CHANNEL NOT CONNECTED");
        return;
      }
      const ax25::LinkLayer& link = channels_[channel == 0 ? 0 : channel - 1].link;
      ax25::formatAddress(channel == 0 ? dedUnprotoDestination_ : link.peer(), text, sizeof(text));
      const uint8_t count = channel == 0 ? dedUnprotoPathCount_ : link.pathCount();
      const ax25::Address* path = channel == 0 ? dedUnprotoPath_ : link.path();
      for (uint8_t i = 0; i < count; ++i) {
        ax25::formatAddress(path[i], address, sizeof(address));
        const size_t pos = strlen(text);
        snprintf(text + pos, sizeof(text) - pos, "%s%s", i == 0 ? " via " : " ", address);
      }
      sendDedText(channel, 1, text);
      return;
    }
    ax25::Address path[ax25::MAX_REPEATERS]{}, connectDest{};
    uint8_t pathCount = 0;
    if (!parseDedPath(dest, connectDest, path, pathCount, channel != 0 && dedValidateCallsign_)) {
      sendDedText(channel, 2, "INVALID CALLSIGN");
      return;
    }
    if (channel == 0) {
        dedUnprotoDestination_ = connectDest;
        memcpy(dedUnprotoPath_, path, sizeof(path));
        dedUnprotoPathCount_ = pathCount;
        copyFormattedAddress(connectDest, dedLastConnect_, sizeof(dedLastConnect_));
        dedLastConnectChannel_ = channel;
        saveDedConfig();
        sendDedShort(channel, 0);
      return;
    }
    if (channel >= 1 && channel <= CHANNEL_COUNT) {
      ax25::LinkLayer& link = channels_[channel - 1].link;
      if (link.state() != ax25::LinkState::Disconnected) {
        if (ax25::addressEquals(link.peer(), connectDest)) {
          link.setPath(path, pathCount);
          sendDedShort(channel, 0);
        } else sendDedText(channel, 2, "CHANNEL ALREADY CONNECTED");
        return;
      }
      for (uint8_t i = 0; i < CHANNEL_COUNT; ++i) {
        if (i == channel - 1 || channels_[i].link.state() == ax25::LinkState::Disconnected) continue;
        if (ax25::addressEquals(channels_[i].link.peer(), connectDest) &&
            ax25::addressEquals(channels_[i].link.local(), link.local())) {
          sendDedText(channel, 2, "STATION ALREADY CONNECTED");
          return;
        }
      }
      link.setPath(path, pathCount);
      if (connectChannel(channel - 1, connectDest)) {
        copyFormattedAddress(connectDest, dedLastConnect_, sizeof(dedLastConnect_));
        dedLastConnectChannel_ = channel;
        sendDedShort(channel, 0);
      } else {
        sendDedText(channel, 2, "CONNECT FAILED");
      }
    } else {
      sendDedText(channel, 2, "CONNECT FAILED");
    }
    return;
  }

  if (cmd[0] == 'D') {
    if (channel == 0) { invalidCommand(); return; }
    if (channels_[channel - 1].link.state() == ax25::LinkState::Disconnected) {
      sendDedText(channel, 1, "CHANNEL NOT CONNECTED");
      return;
    }
    if (channel >= 1 && channel <= CHANNEL_COUNT && disconnect(channel - 1)) {
      sendDedShort(channel, 0);
    } else {
      sendDedText(channel, 2, "DISCONNECT FAILED");
    }
    return;
  }

  if (cmd[0] == 'I') {
    const char* arg = skipSpaces(cmd + 1);
    if (*arg == '\0') {
      char callsign[12]{};
      const ax25::Address& current =
          (channel >= 1 && channel <= CHANNEL_COUNT) ? channels_[channel - 1].link.local() : local_;
      ax25::formatAddress(current, callsign, sizeof(callsign));
      sendDedText(channel, 1, callsign);
      return;
    }
    ax25::Address newAddr{};
    if (!ded::parseCall(arg, newAddr, dedValidateCallsign_)) {
      sendDedText(channel, 2, "INVALID CALLSIGN");
      return;
    }
    char formatted[12]{};
    ax25::formatAddress(newAddr, formatted, sizeof(formatted));
    strncpy(dedLastCallsign_, formatted, sizeof(dedLastCallsign_) - 1);
    dedLastCallsign_[sizeof(dedLastCallsign_) - 1] = '\0';
    dedLastCallsignChannel_ = channel;
    if (channel == 0) {
      local_ = newAddr;
      strncpy(savedCallsign_, formatted, sizeof(savedCallsign_) - 1);
      savedCallsign_[sizeof(savedCallsign_) - 1] = '\0';
      for (uint8_t i = 0; i < CHANNEL_COUNT; ++i)
        if (channels_[i].link.state() == ax25::LinkState::Disconnected) beginLink(i, local_);
    } else if (channel <= CHANNEL_COUNT && !setChannelLocal(channel - 1, newAddr)) {
      sendDedText(channel, 2, "NOT WHILE CONNECTED");
      return;
    } else if (channel > CHANNEL_COUNT) {
      sendDedText(channel, 2, "INVALID CHANNEL");
      return;
    }
    Preferences prefs;
    if (channel == 0 && prefs.begin("axloratnc", false)) {
      prefs.putString("callsign", savedCallsign_);
      prefs.end();
    }
    sendDedShort(channel, 0);
    return;
  }

  if (cmd[0] == 'L') {
    if (!dedHostMode_) {
      unsigned selected = 0;
      const char* arg = skipSpaces(cmd + 1);
      if (*arg != '\0' && !ded::parseNumber(arg, CHANNEL_COUNT, selected))
        sendDedText(channel, 2, "INVALID PARAMETER");
      else printDedTerminalStatus(*arg == '\0' ? -1 : static_cast<int>(selected));
      return;
    }
    if (channel == 0) {
      char status[32]{};
      snprintf(status, sizeof(status), "%u %u",
               static_cast<unsigned>(pendingDedEvents(0, 3)),
               static_cast<unsigned>(pendingDedEvents(0, 4) + pendingDedEvents(0, 5)));
      sendDedText(0, 1, status);
      return;
    }
    const uint8_t chIdx = channel - 1;
    if (chIdx >= CHANNEL_COUNT) {
      sendDedText(channel, 2, "INVALID CHANNEL");
      return;
    }
    char status[80]{};
    // WA8DED: status pending, RX pending, TX queued, unacknowledged, retries, connected.
    snprintf(status, sizeof(status), "%u %u %u %u %u %u",
             static_cast<unsigned>(pendingDedEvents(channel, 3)),
             static_cast<unsigned>(pendingDedEvents(channel, 7)),
             static_cast<unsigned>(channels_[chIdx].link.connectedQueueSize()),
             static_cast<unsigned>(channels_[chIdx].link.outstandingFrameCount()),
             static_cast<unsigned>(channels_[chIdx].link.hostRetryCount()),
             static_cast<unsigned>(channels_[chIdx].link.hostState()));
    sendDedText(channel, 1, status);
    return;
  }

  if (cmd[0] == '@') {
    const char* token = cmd + 1;
    if (token[0] == 'K' && token[1] == '\0') {
      if (dedHostMode_) invalidCommand(true);
      else { Serial.flush(); setSerialMode(SerialMode::Kiss); }
      return;
    }
    if (token[0] == 'S' && token[1] == '\0') {
      if (channel == 0) invalidCommand(true);
      else numberReply(channels_[channel - 1].link.hostState());
      return;
    }
    if ((token[0] == 'D' && strncmp(token, "DIAG", 4) != 0) ||
        token[0] == 'F' || token[0] == 'H' || token[0] == 'I' ||
        token[0] == 'N' || token[0] == 'O' || token[0] == 'U') {
      const char* arg = skipSpaces(token + 1);
      unsigned value = 0;
      unsigned maximum = 1;
      switch (token[0]) {
        case 'D': value = kissParams_.fullDuplex; break;
        case 'F': value = dedFlags_ ? 1U : 0U; break;
        case 'H': value = dedModemMode_; maximum = 3; break;
        case 'I': value = dedIPollFrameLength_; maximum = 256; break;
        case 'N': value = dedHdlcBaud_; maximum = 9600; break;
        case 'O': value = dedTerminalBaud_; maximum = 19200; break;
        case 'U': value = dedUiPoll_ ? 1U : 0U; break;
      }
      if (*arg == '\0') { numberReply(value); return; }
      if (!parseValue(arg, maximum, value)) return;
      if ((token[0] == 'N' || token[0] == 'O') &&
           value != 300 && value != 600 && value != 1200 && value != 2400 &&
           value != 4800 && value != 9600 && (token[0] != 'O' || value != 19200)) {
        char error[48]{};
        snprintf(error, sizeof(error), "INVALID VALUE: %u", value);
        sendDedText(channel, 2, error);
        return;
      }
      switch (token[0]) {
        case 'D': kissParams_.fullDuplex = static_cast<uint8_t>(value); break;
        case 'F': dedFlags_ = value != 0; break;
        case 'H': dedModemMode_ = static_cast<uint8_t>(value); break;
        case 'I': dedIPollFrameLength_ = static_cast<uint16_t>(value); break;
        case 'N': dedHdlcBaud_ = static_cast<uint16_t>(value); break;
        case 'O': dedTerminalBaud_ = static_cast<uint16_t>(value); break;
        case 'U': dedUiPoll_ = value != 0; break;
      }
      saveDedConfig();
      if (token[0] == 'I') applyDedLinkConfig();
      sendDedShort(channel, 0);
      return;
    }
    if (token[0] == 'V') {
      const char* arg = skipSpaces(token + 1);
      if (*arg == '\0') {
        char text[8]{};
        snprintf(text, sizeof(text), "%u", dedValidateCallsign_ ? 1U : 0U);
        sendDedText(channel, 1, text);
      } else {
        char* end = nullptr;
        const long value = strtol(arg, &end, 10);
        if (end == arg || *end != '\0' || value < 0 || value > 1) {
          sendDedText(channel, 2, "INVALID PARAMETER");
          return;
        }
        dedValidateCallsign_ = value != 0;
        saveDedConfig();
        sendDedShort(channel, 0);
      }
      return;
    }
    if (token[0] == 'T' && (token[1] == '2' || token[1] == '3')) {
      const char* arg = skipSpaces(token + 2);
      uint16_t& value = token[1] == '2' ? dedT2_ : dedT3_;
      if (*arg == '\0') {
        char text[16]{};
        snprintf(text, sizeof(text), "%u", static_cast<unsigned>(value));
        sendDedText(channel, 1, text);
      } else {
        char* end = nullptr;
        const long parsed = strtol(arg, &end, 10);
        if (end == arg || *end != '\0' || parsed < 0 || parsed > 65535) {
          sendDedText(channel, 2, "INVALID PARAMETER");
          return;
        }
        value = static_cast<uint16_t>(parsed);
        applyDedLinkConfig();
        saveDedConfig();
        sendDedShort(channel, 0);
      }
      return;
    }
    if (token[0] == 'B' && (token[1] == '\0' || token[1] == ' ')) {
      char text[8]{};
      const unsigned freeBytes = dedFreeBufferUnits(channel);
      snprintf(text, sizeof(text), "%u", freeBytes);
#if AXLORATNC_TRACE_LINFBB
      if (channel >= 1 && channel <= CHANNEL_COUNT) {
        const uint8_t chIdx = channel - 1;
        const ax25::LinkLayer& link = channels_[chIdx].link;
        LINFBB_TRACE("DED_FREEBUF ch=%u state=%u q=%u/%u win_free=%u out=%u adv=%u accept=%u",
                     static_cast<unsigned>(channel),
                     static_cast<unsigned>(link.state()),
                     static_cast<unsigned>(link.connectedQueueSize()),
                     static_cast<unsigned>(link.connectedQueueSize() + link.connectedQueueFree()),
                     static_cast<unsigned>(link.connectedWindowFree()),
                     static_cast<unsigned>(link.outstandingFrameCount()),
                     freeBytes,
                     static_cast<unsigned>(dedConnectedAcceptFrameCapacity(chIdx) *
                                           (dedIPollFrameLength_ > 0 ? dedIPollFrameLength_ : 64u)));
      } else {
        LINFBB_TRACE("DED_FREEBUF ch=%u adv=%u", static_cast<unsigned>(channel), freeBytes);
      }
#endif
      sendDedText(channel, 1, text);
      return;
    }
    // Keep diagnostics outside the standard @D (full-duplex) command.
    if (strncmp(token, "DIAG", 4) == 0 && (token[4] == '\0' || token[4] == ' ')) {
      char buf[256]{};
      const auto& rs = radio::stats();
      size_t pos = snprintf(buf, sizeof(buf),
                            "radio rxOk=%lu rxFail=%lu txOk=%lu txFail=%lu rssi=%.0f snr=%.0f\n"
                            "uptime_ms=%lu reset=%u outFail=%lu rxTimeout=%lu block_ms=%lu\n",
                            static_cast<unsigned long>(rs.rxOk),
                            static_cast<unsigned long>(rs.rxFail),
                            static_cast<unsigned long>(rs.txOk),
                            static_cast<unsigned long>(rs.txFail),
                            static_cast<double>(lastRssi_),
                            static_cast<double>(lastSnr_),
                            static_cast<unsigned long>(millis()),
#if defined(NRF52_PLATFORM)
                            static_cast<unsigned>(NRF_POWER->RESETREAS),
#else
                            static_cast<unsigned>(esp_reset_reason()),
#endif
                            static_cast<unsigned long>(dedOutPartialWrites_),
                            static_cast<unsigned long>(dedRxTimeouts_),
                            static_cast<unsigned long>(dedOutBlockMsMax_));
      if (pos >= sizeof(buf)) pos = sizeof(buf) - 1;
      for (uint8_t i = 0; i < CHANNEL_COUNT; ++i) {
        const ax25::LinkLayer& lnk = channels_[i].link;
        if (lnk.state() == ax25::LinkState::Disconnected) continue;
        char peer[12]{};
        ax25::formatAddress(lnk.peer(), peer, sizeof(peer));
        const char* st = lnk.state() == ax25::LinkState::Connected    ? "CON" :
                         lnk.state() == ax25::LinkState::Connecting    ? "CONN" :
                         lnk.state() == ax25::LinkState::Recovery      ? "REC" :
                         lnk.state() == ax25::LinkState::Disconnecting ? "DISC" : "?";
        const size_t remaining = sizeof(buf) - pos;
        const int written = snprintf(buf + pos, remaining, "ch%u %s %s ret=%u\n",
                                     static_cast<unsigned>(i + 1), st, peer,
                                     static_cast<unsigned>(lnk.retryCount()));
        if (written < 0) {
          LOG_ERR("WA8DED diagnostic formatting failed");
          break;
        }
        if (static_cast<size_t>(written) >= remaining) break;
        pos += static_cast<size_t>(written);
      }
      sendDedText(channel, 1, buf);
      return;
    }
    invalidCommand(true);
    return;
  }

  if (cmd[0] == 'M') {
    const char* arg = skipSpaces(cmd + 1);
    if (*arg == '\0') {
      char mtext[128]{};
      snprintf(mtext, sizeof(mtext), "%s", monitorEnabled_ ? dedMonitorMode_ : "N");
      if (dedMonitorCallFilter_ != '\0') {
        size_t pos = strlen(mtext);
        snprintf(mtext + pos, sizeof(mtext) - pos, " %c", dedMonitorCallFilter_);
        for (uint8_t i = 0; i < dedMonitorCallCount_; ++i) {
          char call[12]{};
          ax25::formatAddress(dedMonitorCalls_[i], call, sizeof(call));
          pos = strlen(mtext);
          snprintf(mtext + pos, sizeof(mtext) - pos, " %s", call);
        }
      }
      sendDedText(channel, 1, mtext);
      return;
    }
    if (!setDedMonitor(arg)) {
      sendDedText(channel, 2, "INVALID PARAMETER");
      return;
    }
    saveDedConfig();
    sendDedShort(channel, 0);
    return;
  }

  if (cmd[0] == 'H') {
    invalidCommand();
    return;
  }

  if (cmd[0] == 'V') {
    sendDedText(channel, 1, "AXLoRaTNC WA8DED");
    return;
  }

  if (cmd[0] == 'B') { numberReply(dedRoundsPerSecond_); return; }

  if (cmd[0] == 'K') {
    const char* arg = skipSpaces(cmd + 1);
    if (*arg == '\0') {
      char stamp[24]{}, text[32]{};
      formatDedTimestamp(stamp, sizeof(stamp));
      snprintf(text, sizeof(text), "%u %s", dedTimestamp_, stamp);
      sendDedText(channel, 1, text);
      return;
    }
    unsigned value = 0;
    if (ded::parseNumber(arg, 2, value)) dedTimestamp_ = static_cast<uint8_t>(value);
    else if (!setDedTimestamp(arg)) {
      sendDedText(channel, 2, "INVALID PARAMETER");
      return;
    }
    saveDedConfig();
    sendDedShort(channel, 0);
    return;
  }

  if (cmd[0] == 'Y') {
    const char* arg = skipSpaces(cmd + 1);
    if (*arg == '\0') {
      unsigned connected = 0;
      for (const ChannelState& state : channels_)
        if (state.link.state() != ax25::LinkState::Disconnected) ++connected;
      char text[24]{};
      snprintf(text, sizeof(text), "%u (%u)", dedMaxIncoming_, connected);
      sendDedText(channel, 1, text);
    } else {
      unsigned value = 0;
      if (!parseValue(arg, CHANNEL_COUNT, value)) return;
      dedMaxIncoming_ = static_cast<uint8_t>(value);
      saveDedConfig();
      sendDedShort(channel, 0);
    }
    return;
  }

  if (cmd[0] == 'F' || cmd[0] == 'N' || cmd[0] == 'O') {
    const char* arg = skipSpaces(cmd + 1);
    const uint8_t index = channel == 0 ? 0 : channel - 1;
    unsigned value = cmd[0] == 'F' ? (channel == 0 ? dedFrack_ : channels_[index].initialRtt)
        : cmd[0] == 'N' ? (channel == 0 ? dedRetryLimit_ : channels_[index].retryLimit)
        : (channel == 0 ? dedMaxFrame_ : channels_[index].maxFrame);
    if (*arg == '\0') { numberReply(value); return; }
    if (!parseValue(arg, cmd[0] == 'F' ? 65535 : cmd[0] == 'N' ? 127 : 7, value,
                    cmd[0] == 'N' ? 0 : 1)) return;
    if (cmd[0] == 'F' && value < 16) value = value * 100 / dedSrttA3_;
    if (channel == 0) {
      if (cmd[0] == 'F') dedFrack_ = static_cast<uint16_t>(value);
      if (cmd[0] == 'N') dedRetryLimit_ = static_cast<uint8_t>(value);
      if (cmd[0] == 'O') dedMaxFrame_ = static_cast<uint8_t>(value);
      for (ChannelState& state : channels_) {
        if (state.link.state() != ax25::LinkState::Disconnected) continue;
        if (cmd[0] == 'F') state.initialRtt = dedFrack_;
        if (cmd[0] == 'N') state.retryLimit = dedRetryLimit_;
        if (cmd[0] == 'O') state.maxFrame = dedMaxFrame_;
      }
      saveDedConfig();
    } else {
      if (cmd[0] == 'F') channels_[index].initialRtt = static_cast<uint16_t>(value);
      if (cmd[0] == 'N') channels_[index].retryLimit = static_cast<uint8_t>(value);
      if (cmd[0] == 'O') channels_[index].maxFrame = static_cast<uint8_t>(value);
    }
    applyDedLinkConfig();
    sendDedShort(channel, 0);
    return;
  }

  if (cmd[0] == 'A' || cmd[0] == 'E' ||
      cmd[0] == 'P' || cmd[0] == 'R' || cmd[0] == 'T' || cmd[0] == 'W' ||
      cmd[0] == 'X' || cmd[0] == 'Z') {
    const char* arg = skipSpaces(cmd + 1);
    auto reply = [&](unsigned value) {
      char text[16]{};
      snprintf(text, sizeof(text), "%u", value);
      sendDedText(channel, 1, text);
    };
    if (*arg == '\0') {
      switch (cmd[0]) {
        case 'A': reply(dedAutoLf_ ? 1U : 0U); return;
        case 'E': reply(dedEcho_ ? 1U : 0U); return;
        case 'P': reply(kissParams_.persistence); return;
        case 'R': reply(digi_.enabled ? 1U : 0U); return;
        case 'T': reply(kissParams_.txDelay); return;
        case 'W': reply(kissParams_.slotTime); return;
        case 'X': reply(dedTxEnabled_ ? 1U : 0U); return;
        case 'Z': reply(static_cast<unsigned>((dedFlow_ ? 1 : 0) + (dedXonXoff_ ? 2 : 0))); return;
      }
    }
    unsigned parsed = 0;
    const unsigned maximum = cmd[0] == 'P' ? 255 : cmd[0] == 'W' ? 127
        : cmd[0] == 'T' ? 64 : cmd[0] == 'Z' ? 3 : 1;
    if (!parseValue(arg, maximum, parsed)) return;
    const uint8_t value = static_cast<uint8_t>(parsed);
    switch (cmd[0]) {
      case 'A': dedAutoLf_ = value != 0; break;
      case 'E': dedEcho_ = value != 0; break;
      case 'P': kissParams_.persistence = value; break;
      case 'R': digi_.enabled = value != 0; saveSettings(); break;
      case 'T': if (value > 127) { sendDedText(channel, 2, "INVALID PARAMETER"); return; }
                kissParams_.txDelay = value; break;
      case 'W': if (value > 127) { sendDedText(channel, 2, "INVALID PARAMETER"); return; }
                kissParams_.slotTime = value; break;
      case 'X': dedTxEnabled_ = value != 0; break;
      case 'Z': if (value > 3) { sendDedText(channel, 2, "INVALID PARAMETER"); return; }
                dedFlow_ = (value & 1) != 0; dedXonXoff_ = (value & 2) != 0; break;
      default:
        sendDedText(channel, 2, "INVALID PARAMETER");
        return;
    }
    saveDedConfig();
    sendDedShort(channel, 0);
    return;
  }

  if (cmd[0] == 'S') {
    if (dedHostMode_) { invalidCommand(); return; }
    const char* arg = skipSpaces(cmd + 1);
    if (*arg == '\0') {
      char text[12]{};
      snprintf(text, sizeof(text), "%u", dedSelectedChannel_);
      sendDedText(channel, 1, text);
      return;
    }
    uint8_t value = 0;
    if (!parseByteValue(arg, value) || value > CHANNEL_COUNT) {
      sendDedText(channel, 2, "INVALID PARAMETER");
      return;
    }
    dedSelectedChannel_ = value;
    handleDedCommand(value, "C", 1);
    return;
  }

  if (cmd[0] == 'U') {
    const char* arg = skipSpaces(cmd + 1);
    if (*arg == '\0') {
      char text[96]{};
      snprintf(text, sizeof(text), "%u%s%s", static_cast<unsigned>(dedCtextMode_),
               dedUnattendedText_[0] == '\0' ? "" : " ", dedUnattendedText_);
      sendDedText(channel, 1, text);
      return;
    }
    unsigned value = 0;
    const char* text = nullptr;
    if (!ded::parseNumber(arg, 2, value, &text)) {
      sendDedText(channel, 2, "INVALID PARAMETER");
      return;
    }
    if (strlen(text) > 72) {
      sendDedText(channel, 2, "MESSAGE TOO LONG");
      return;
    }
    if (*text != '\0') strcpy(dedUnattendedText_, text);
    if (value != 0 && dedUnattendedText_[0] == '\0') {
      dedCtextMode_ = 0;
      dedUnattended_ = false;
      sendDedText(channel, 2, "NO MESSAGE AVAILABLE");
      return;
    }
    dedCtextMode_ = static_cast<uint8_t>(value);
    dedUnattended_ = value != 0;
    saveDedConfig();
    sendDedShort(channel, 0);
    return;
  }

  if (strcmp(cmd, "QRES") == 0) {
    sendDedShort(channel, 0);
    Serial.flush();
#if defined(NRF52_PLATFORM)
    NVIC_SystemReset();
#else
    ESP.restart();
#endif
    return;
  }
  invalidCommand();
}

void Tnc::sendDedShort(uint8_t channel, uint8_t code) {
  if (!dedHostMode_) return;
  uint8_t out[2] = {channel, code};
  writeDedReply(out, sizeof(out));
}

void Tnc::writeDedReply(const uint8_t* data, size_t len) {
  const uint32_t startMs = millis();
  const bool complete = serialWriteAll(data, len);
  if (complete) Serial.flush();
  const uint32_t elapsedMs = millis() - startMs;
  if (elapsedMs > dedOutBlockMsMax_) dedOutBlockMsMax_ = elapsedMs;
  if (!complete) {
    ++dedOutPartialWrites_;
    LOG_ERR("serial WA8DED write timeout len=%u", static_cast<unsigned>(len));
  }
}

void Tnc::sendDedText(uint8_t channel, uint8_t code, const char* text) {
  if (!dedHostMode_) {
    Serial.printf("* %s *%s\r%s", text == nullptr ? "" : text,
                  code == 2 ? "\a" : "", dedAutoLf_ ? "\n" : "");
    return;
  }
  const size_t len = text != nullptr ? strlen(text) : 0;
  uint8_t out[259]{};
  if (len > sizeof(out) - 3) {
    LOG_ERR("serial WA8DED text reply too long len=%u", static_cast<unsigned>(len));
    sendDedText(channel, 2, "REPLY TOO LONG");
    return;
  }
  out[0] = channel;
  out[1] = code;
  if (len > 0) memcpy(out + 2, text, len);
  writeDedReply(out, len + 3);
}

void Tnc::sendDedCounted(uint8_t channel, uint8_t code, const uint8_t* data, size_t len) {
  if (len == 0 || len > 256 || data == nullptr) {
    LOG_ERR("invalid WA8DED counted reply length=%u", static_cast<unsigned>(len));
    sendDedText(channel, 2, "INVALID PAYLOAD");
    return;
  }
  const size_t capped = len > 256 ? 256 : len;
  uint8_t out[259]{};
  out[0] = channel;
  out[1] = code;
  out[2] = static_cast<uint8_t>(capped - 1);
  if (capped > 0) memcpy(out + 3, data, capped);
  writeDedReply(out, capped + 3);
}

unsigned Tnc::dedFreeBufferUnits(uint8_t channel) const {
  (void)channel;
  // TF2.7 reports twice its 64-byte free-buffer count, not byte credits.
  const size_t raw = rawTxQueue_.free() + rawTxPriorityQueue_.free();
  size_t bytes = raw * MAX_PACKET_BYTES + dedEvents_.free() * 256;
  for (const ChannelState& state : channels_)
    bytes += state.link.connectedQueueFree() * ax25::MAX_INFO_LEN;
  return static_cast<unsigned>(bytes / 32);
}

size_t Tnc::dedConnectedAcceptFrameCapacity(uint8_t chIdx) const {
  if (chIdx >= CHANNEL_COUNT) return 0;
  const ax25::LinkLayer& link = channels_[chIdx].link;
  const auto st = link.state();
  const bool live = (st == ax25::LinkState::Connected ||
                     st == ax25::LinkState::Recovery);
  if (!live ||
      link.connectedQueueFree() == 0) {
    return 0;
  }

  size_t frames = link.connectedQueueFree();
  if (st == ax25::LinkState::Connected && !link.connectedPeerBusy() &&
      link.connectedQueueSize() == 0 && link.outstandingFrameCount() == 0) {
    frames += link.connectedWindowFree();
  }
  return frames;
}

bool Tnc::enqueueDedEvent(uint8_t channel, uint8_t code, const uint8_t* data, size_t len) {
  if ((code == 6 || code == 7) && len == 0) return true;
  if (len > 256) {
    LOG_ERR("WA8DED event exceeds maximum payload");
    return false;
  }
  DedEvent event{};
  event.channel = channel;
  event.code    = code;
  event.len     = len > sizeof(event.data) ? sizeof(event.data) : len;
  if (data != nullptr && event.len > 0) memcpy(event.data, data, event.len);
  if (!dedEvents_.push(event)) {
    LOG_WARN("WA8DED event queue full ch=%u code=%u", channel, code);
    return false;
  }
  return true;
}

size_t Tnc::pendingDedEvents(uint8_t channel, uint8_t wanted) {
  size_t pending = 0;
  const size_t count = dedEvents_.size();
  for (size_t i = 0; i < count; ++i) {
    DedEvent event{};
    if (!dedEvents_.peek(i, event)) return pending;
    const bool channelOk = event.channel == channel;
    const bool typeOk = (wanted == 0) || (event.code == wanted);
    if (channelOk && typeOk) ++pending;
  }
  return pending;
}

bool Tnc::popDedEvent(uint8_t channel, uint8_t wanted, DedEvent& out) {
  const size_t count = dedEvents_.size();
  for (size_t i = 0; i < count; ++i) {
    DedEvent event{};
    if (!dedEvents_.peek(i, event)) return false;
    // LinFBB validates that the returned channel equals the channel it polled.
    // Channel 0 is therefore a real hostmode channel here, not a wildcard.
    const bool channelOk = event.channel == channel;
    const bool typeOk = wanted == 0 || event.code == wanted ||
                        (channel == 0 && wanted == 4 && event.code == 5);
    if (channelOk && typeOk) return dedEvents_.remove(i, out);
  }
  return false;
}

void Tnc::handleQuietEscape(uint8_t byte) {
  if (byte == '\r' || byte == '\n') {
    escapeLine_[escapePos_] = '\0';
    if (strcmp(escapeLine_, "console") == 0) {
      setSerialMode(SerialMode::Console); saveSerialMode(SerialMode::Console);
      Serial.println("mode=console");
    }
    escapePos_ = 0;
    return;
  }
  if (escapePos_ < sizeof(escapeLine_) - 1) escapeLine_[escapePos_++] = static_cast<char>(byte);
  else escapePos_ = 0;
}

// ---------------------------------------------------------------------------
// Digipeater
// ---------------------------------------------------------------------------

bool Tnc::maybeDigipeat(const ax25::Frame& frame) {
  if (!digi_.enabled) return false;
  const bool uiFrame = isUiFrame(frame);
  if (!uiFrame && !digi_.allFrames) return false;
  if (ax25::addressEquals(frame.source, local_)) return false;
  const int repeaterIndex = findNextRepeater(frame);
  if (repeaterIndex < 0) return false;

  if (uiFrame) {
    const uint16_t infoCrc = axlora::util::crc16Ccitt(frame.info, frame.infoLen);
    const uint16_t pathCrc = digiPathCrc(frame);
    if (digiSeen(frame, infoCrc, pathCrc)) { ++digiDupes_; return false; }
    rememberDigi(frame, infoCrc, pathCrc);
  }

  ax25::Frame repeated = frame;
  repeated.repeaters[repeaterIndex].repeated = true;
  uint8_t bytes[MAX_PACKET_BYTES]{};
  size_t len = 0;
  if (!ax25::encodeFrame(repeated, bytes, sizeof(bytes), len, true)) { ++digiDrops_; return false; }
  if (!transmitRaw(bytes, len)) { ++digiDrops_; return false; }
  ++digiTx_;
  if (uiFrame) ++digiUiTx_;
  else ++digiConnTx_;
  return true;
}

int Tnc::findNextRepeater(const ax25::Frame& frame) const {
  for (uint8_t i = 0; i < frame.repeaterCount; ++i) {
    if (!frame.repeaters[i].repeated) {
      return matchesDigiAddress(frame.repeaters[i]) ? static_cast<int>(i) : -1;
    }
  }
  return -1;
}

bool Tnc::matchesDigiAddress(const ax25::Address& address) const {
  return ax25::addressEquals(address, local_) ||
         (digi_.hasAlias && ax25::addressEquals(address, digi_.alias));
}

uint16_t Tnc::digiPathCrc(const ax25::Frame& frame) const {
  uint8_t raw[ax25::MAX_REPEATERS * 8]{};
  size_t p = 0;
  for (uint8_t i = 0; i < frame.repeaterCount && p + 8 <= sizeof(raw); ++i) {
    if (!ax25::encodeAddress(frame.repeaters[i], i == frame.repeaterCount - 1, &raw[p])) break;
    p += 7;
    raw[p++] = frame.repeaters[i].repeated ? 1 : 0;
  }
  return axlora::util::crc16Ccitt(raw, p);
}

bool Tnc::digiSeen(const ax25::Frame& frame, uint16_t infoCrc, uint16_t pathCrc) const {
  const uint32_t now = axlora::util::nowMs();
  for (const DigiCacheEntry& entry : digiCache_) {
    if (entry.active && !axlora::util::elapsed(now, entry.seenMs, 30000) &&
        entry.control == frame.control && entry.infoCrc == infoCrc &&
        entry.pathCrc == pathCrc && entry.repeaterCount == frame.repeaterCount &&
        ax25::addressEquals(entry.source, frame.source) &&
        ax25::addressEquals(entry.destination, frame.destination)) {
      return true;
    }
  }
  return false;
}

void Tnc::rememberDigi(const ax25::Frame& frame, uint16_t infoCrc, uint16_t pathCrc) {
  const uint32_t now = axlora::util::nowMs();
  DigiCacheEntry* slot = nullptr;
  for (DigiCacheEntry& entry : digiCache_) {
    if (!entry.active || axlora::util::elapsed(now, entry.seenMs, 30000)) {
      slot = &entry; break;
    }
  }
  if (slot == nullptr) slot = &digiCache_[0];
  slot->active      = true;
  slot->source      = frame.source;
  slot->destination = frame.destination;
  slot->control     = frame.control;
  slot->infoCrc     = infoCrc;
  slot->pathCrc     = pathCrc;
  slot->repeaterCount = frame.repeaterCount;
  slot->seenMs      = now;
}

void Tnc::printDigipeater() const {
  char alias[12]{};
  if (digi_.hasAlias) ax25::formatAddress(digi_.alias, alias, sizeof(alias));
  else strcpy(alias, "off");
  Serial.printf("digipeat=%s mode=%s alias=%s tx=%lu ui_tx=%lu conn_tx=%lu dupes=%lu drops=%lu\n",
                digi_.enabled ? "on" : "off", digi_.allFrames ? "all" : "ui", alias,
                static_cast<unsigned long>(digiTx_),
                static_cast<unsigned long>(digiUiTx_),
                static_cast<unsigned long>(digiConnTx_),
                static_cast<unsigned long>(digiDupes_),
                static_cast<unsigned long>(digiDrops_));
}

// ---------------------------------------------------------------------------
// Mheard
// ---------------------------------------------------------------------------

void Tnc::observeHeard(const ax25::Frame& frame, float rssi, float snr) {
  if (ax25::addressEquals(frame.source, local_)) return;
  const uint32_t now = axlora::util::nowMs();
  MheardEntry* slot = nullptr;
  for (MheardEntry& entry : mheard_) {
    if (entry.active && ax25::addressEquals(entry.source, frame.source)) { slot = &entry; break; }
  }
  if (slot == nullptr) {
    for (MheardEntry& entry : mheard_) {
      if (!entry.active) { slot = &entry; break; }
    }
  }
  if (slot == nullptr) {
    slot = &mheard_[0];
    for (MheardEntry& entry : mheard_) {
      if (entry.lastHeardMs < slot->lastHeardMs) slot = &entry;
    }
  }
  if (!slot->active) {
    *slot = MheardEntry{};
    slot->active       = true;
    slot->source       = frame.source;
    slot->firstHeardMs = now;
  }
  slot->destination       = frame.destination;
  slot->lastHeardMs       = now;
  ++slot->frames;
  slot->lastRssi          = rssi;
  slot->lastSnr           = snr;
  slot->lastRepeaterCount = frame.repeaterCount;
  slot->viaDigipeater     = false;
  for (uint8_t i = 0; i < frame.repeaterCount; ++i) {
    if (frame.repeaters[i].repeated) { slot->viaDigipeater = true; break; }
  }
}

void Tnc::printMheard() const {
  Serial.println("callsign   dest       first    last     frames  rssi   snr  path");
  for (const MheardEntry& entry : mheard_) {
    if (!entry.active) continue;
    char source[12]{}, destination[12]{};
    char first[12]{}, last[12]{};
    ax25::formatAddress(entry.source,      source,      sizeof(source));
    ax25::formatAddress(entry.destination, destination, sizeof(destination));
    formatUptime(entry.firstHeardMs, first, sizeof(first));
    formatUptime(entry.lastHeardMs,  last,  sizeof(last));
    Serial.printf("%-10s %-10s %s %s %6lu %6.1f %5.1f  %s/%u\n",
                  source, destination, first, last,
                  static_cast<unsigned long>(entry.frames),
                  static_cast<double>(entry.lastRssi),
                  static_cast<double>(entry.lastSnr),
                  entry.viaDigipeater ? "via" : "direct",
                  entry.lastRepeaterCount);
  }
}

void Tnc::clearMheard() {
  memset(mheard_, 0, sizeof(mheard_));
}

// ---------------------------------------------------------------------------
// Beacon
// ---------------------------------------------------------------------------

bool Tnc::sendBeacon() {
  ax25::Frame frame{};
  frame.destination  = beacon_.destination;
  frame.source       = local_;
  frame.repeaterCount = beacon_.pathCount;
  for (uint8_t i = 0; i < beacon_.pathCount; ++i) frame.repeaters[i] = beacon_.path[i];
  frame.control  = ax25::CTRL_UI;
  frame.pid      = ax25::PID_NO_LAYER3;
  frame.infoLen  = strnlen(beacon_.text, sizeof(beacon_.text));
  memcpy(frame.info, beacon_.text, frame.infoLen);
  uint8_t bytes[MAX_PACKET_BYTES]{};
  size_t len = 0;
  if (!ax25::encodeFrame(frame, bytes, sizeof(bytes), len, true)) { ++beaconDrops_; return false; }
  if (!transmitRaw(bytes, len)) { ++beaconDrops_; return false; }
  ++beaconTx_;
  return true;
}

void Tnc::printBeacon() const {
  char destination[12]{};
  ax25::formatAddress(beacon_.destination, destination, sizeof(destination));
  Serial.printf("beacon=%s dest=%s interval_s=%lu tx=%lu drops=%lu text=\"%s\"\n",
                beacon_.enabled ? "on" : "off", destination,
                static_cast<unsigned long>(beacon_.intervalMs / 1000),
                static_cast<unsigned long>(beaconTx_),
                static_cast<unsigned long>(beaconDrops_),
                beacon_.text);
  Serial.print("beacon path=");
  if (beacon_.pathCount == 0) { Serial.println("off"); return; }
  for (uint8_t i = 0; i < beacon_.pathCount; ++i) {
    char path[12]{};
    ax25::formatAddress(beacon_.path[i], path, sizeof(path));
    if (i > 0) Serial.print(",");
    Serial.print(path);
  }
  Serial.println();
}

bool Tnc::setBeaconPath(const char* path) {
  beacon_.pathCount = 0;
  if (path == nullptr || strcmp(path, "off") == 0 || path[0] == '\0') return true;
  char buffer[80]{};
  strncpy(buffer, path, sizeof(buffer) - 1);
  char* token = strtok(buffer, ",");
  while (token != nullptr && beacon_.pathCount < ax25::MAX_REPEATERS) {
    while (*token == ' ') ++token;
    if (!ax25::parseAddress(token, beacon_.path[beacon_.pathCount])) {
      beacon_.pathCount = 0; return false;
    }
    ++beacon_.pathCount;
    token = strtok(nullptr, ",");
  }
  return token == nullptr;
}

// ---------------------------------------------------------------------------
// NET/ROM
// ---------------------------------------------------------------------------

void Tnc::observeNetrom(const ax25::Frame& frame) {
  if (ax25::kind(frame.control) != ax25::FrameKind::U ||
      ax25::uType(frame.control) != ax25::UFrameType::UI ||
      frame.pid != 0xCF || frame.infoLen < 15 || frame.info[0] != 0xFF) return;
  size_t p = 1;
  while (p + 14 <= frame.infoLen) {
    NetromRoute route{};
    route.active = true;
    memcpy(route.alias, &frame.info[p], 6);
    route.alias[6] = '\0';
    for (int i = 5; i >= 0; --i) {
      if (route.alias[i] == ' ') route.alias[i] = '\0'; else break;
    }
    p += 6;
    bool last = false;
    if (!ax25::decodeAddress(&frame.info[p], route.node, last)) return;
    p += 7;
    route.quality    = frame.info[p++];
    route.heardFrom  = frame.source;
    route.lastHeardMs = axlora::util::nowMs();

    NetromRoute* slot = nullptr;
    for (NetromRoute& existing : netromRoutes_) {
      if (existing.active && ax25::addressEquals(existing.node, route.node)) { slot = &existing; break; }
    }
    if (slot == nullptr) {
      for (NetromRoute& existing : netromRoutes_) {
        if (!existing.active) { slot = &existing; break; }
      }
    }
    if (slot == nullptr) slot = &netromRoutes_[0];
    *slot = route;
    ++netromRoutesHeard_;
  }
}

bool Tnc::sendNetromBroadcast() {
  ax25::Frame frame{};
  ax25::parseAddress("NODES", frame.destination);
  frame.source  = local_;
  frame.control = ax25::CTRL_UI;
  frame.pid     = 0xCF;
  frame.info[0] = 0xFF;
  memset(&frame.info[1], ' ', 6);
  const size_t aliasLen = strnlen(netrom_.alias, sizeof(netrom_.alias));
  memcpy(&frame.info[1], netrom_.alias, aliasLen > 6 ? 6 : aliasLen);
  ax25::encodeAddress(local_, true, &frame.info[7]);
  frame.info[14] = 200;
  frame.infoLen  = 15;
  uint8_t bytes[MAX_PACKET_BYTES]{};
  size_t len = 0;
  if (!ax25::encodeFrame(frame, bytes, sizeof(bytes), len, true)) return false;
  if (!transmitRaw(bytes, len)) return false;
  ++netromBroadcasts_;
  return true;
}

void Tnc::printNetrom() const {
  Serial.printf("netrom=%s alias=%s ident=\"%s\" interval_s=%lu broadcasts=%lu routes_heard=%lu\n",
                netrom_.enabled ? "on" : "off", netrom_.alias, netrom_.ident,
                static_cast<unsigned long>(netrom_.broadcastIntervalMs / 1000),
                static_cast<unsigned long>(netromBroadcasts_),
                static_cast<unsigned long>(netromRoutesHeard_));
}

void Tnc::printNetromRoutes() const {
  const uint32_t now = axlora::util::nowMs();
  Serial.println("alias node via qual age_s obs");
  for (const NetromRoute& route : netromRoutes_) {
    if (!route.active) continue;
    char node[12]{}, via[12]{};
    ax25::formatAddress(route.node,      node, sizeof(node));
    ax25::formatAddress(route.heardFrom, via,  sizeof(via));
    Serial.printf("%s %s %s %u %lu %u\n",
                  route.alias, node, via, route.quality,
                  static_cast<unsigned long>((now - route.lastHeardMs) / 1000),
                  route.obsolescence);
  }
}

// ---------------------------------------------------------------------------
// Display info
// ---------------------------------------------------------------------------

void Tnc::fillDisplayInfo(display::DisplayInfo& out) const {
  // ---- page 1 ----
  ax25::formatAddress(local_, out.callsign, sizeof(out.callsign));

  const char* mname = serialModeName();
  size_t mi = 0;
  for (; mname[mi] && mi < sizeof(out.mode) - 1; ++mi) {
    const char c = mname[mi];
    out.mode[mi] = (c >= 'a' && c <= 'z') ? static_cast<char>(c - 32) : c;
  }
  out.mode[mi] = '\0';

  out.freqMHz  = radioConfig_.frequencyMHz;
  out.sf       = radioConfig_.spreadingFactor;
  out.powerDbm = radioConfig_.powerDbm;

  const radio::Stats& rs = radio::stats();
  out.txCount  = rs.txOk;
  out.rxCount  = rs.rxOk;
  out.lastRssi = lastRssi_;
  out.lastSnr  = lastSnr_;

  out.anyConnected = false;
  for (uint8_t i = 0; i < CHANNEL_COUNT; ++i) {
    const ax25::LinkState s = channels_[i].link.state();
    if (s == ax25::LinkState::Disconnected) continue;
    out.anyConnected = true;
    out.connChannel  = i + 1;
    ax25::formatAddress(channels_[i].link.peer(), out.connPeer, sizeof(out.connPeer));
    switch (s) {
      case ax25::LinkState::Connecting:    strncpy(out.connState, "CON", 4); break;
      case ax25::LinkState::Connected:     strncpy(out.connState, "OK",  4); break;
      case ax25::LinkState::Disconnecting: strncpy(out.connState, "DIS", 4); break;
      case ax25::LinkState::Recovery:      strncpy(out.connState, "REC", 4); break;
      default:                             strncpy(out.connState, "?",   4); break;
    }
    break;
  }

  // ---- page 2: RF config + baud rates ----
  out.bandwidthKhz = radioConfig_.bandwidthKhz;
  out.codingRate   = radioConfig_.codingRate;
  out.baudKiss     = baudKiss_;
  out.baudDed      = baudDed_;
  out.dedHostMode  = dedHostMode_;
  out.dedSelectedChannel = dedSelectedChannel_;
  out.dedLastCallsignChannel = dedLastCallsignChannel_;
  strncpy(out.dedLastCallsign, dedLastCallsign_, sizeof(out.dedLastCallsign) - 1);
  out.dedLastConnectChannel = dedLastConnectChannel_;
  strncpy(out.dedLastConnect, dedLastConnect_, sizeof(out.dedLastConnect) - 1);

  // ---- page 3: AX.25 stats aggregated across all channels ----
  out.l2Retries    = 0;
  out.l2FcsDrops   = 0;
  out.l2RejTx      = 0;
  out.l2SrejTx     = 0;
  out.l2FrmrTx     = 0;
  out.l2QueueDrops = 0;
  for (uint8_t i = 0; i < CHANNEL_COUNT; ++i) {
    const ax25::L2Stats& s = channels_[i].link.stats();
    out.l2Retries    += s.retries;
    out.l2FcsDrops   += s.fcsDrops;
    out.l2RejTx      += s.rejTx;
    out.l2SrejTx     += s.srejTx;
    out.l2FrmrTx     += s.frmrTx;
    out.l2QueueDrops += s.queueDrops;
  }
#if defined(NRF52_PLATFORM)
  out.freeHeapBytes = static_cast<uint32_t>(xPortGetFreeHeapSize());
#else
  out.freeHeapBytes = static_cast<uint32_t>(ESP.getFreeHeap());
#endif

  // ---- page 4: mheard — top MHEARD_COUNT stations by most recent ----
  {
    const uint32_t now = axlora::util::nowMs();
    struct { uint32_t t; int idx; } sorted[20]{};
    uint8_t total = 0;
    for (int i = 0; i < 20; ++i) {
      if (mheard_[i].active) {
        sorted[total].t   = mheard_[i].lastHeardMs;
        sorted[total].idx = i;
        ++total;
      }
    }
    // Bubble sort descending by time (at most 20 entries — negligible cost)
    for (uint8_t i = 0; i < total; ++i) {
      for (uint8_t j = static_cast<uint8_t>(i + 1); j < total; ++j) {
        if (sorted[j].t > sorted[i].t) {
          auto tmp = sorted[i]; sorted[i] = sorted[j]; sorted[j] = tmp;
        }
      }
    }
    for (uint8_t i = 0; i < display::DisplayInfo::MHEARD_COUNT; ++i) {
      out.mheard[i].active = false;
    }
    const uint8_t fill = total < display::DisplayInfo::MHEARD_COUNT
                       ? total
                       : display::DisplayInfo::MHEARD_COUNT;
    for (uint8_t i = 0; i < fill; ++i) {
      const MheardEntry& m = mheard_[sorted[i].idx];
      out.mheard[i].active = true;
      ax25::formatAddress(m.source, out.mheard[i].call, sizeof(out.mheard[i].call));
      out.mheard[i].rssi  = m.lastRssi;
      out.mheard[i].snr   = m.lastSnr;
      out.mheard[i].agoMs = now - m.lastHeardMs;
    }
  }

  // ---- page 5: services ----
  out.digiEnabled      = digi_.enabled;
  out.digiTx           = digiTx_;
  out.beaconEnabled    = beacon_.enabled;
  out.beaconTx         = beaconTx_;
  out.beaconIntervalMs = beacon_.intervalMs;
  out.netromEnabled    = netrom_.enabled;
  out.netromRoutes     = 0;
  for (const NetromRoute& r : netromRoutes_) {
    if (r.active) ++out.netromRoutes;
  }
  out.bbsMsgCount = 0;
  for (uint8_t i = 0; i < mailbox_.maxMessages(); ++i) {
    const Mailbox::Message* m = mailbox_.get(i);
    if (m && m->active) ++out.bbsMsgCount;
  }
  out.uptimeMs = axlora::util::nowMs();
}

// ---------------------------------------------------------------------------
// Settings (NVS)
// ---------------------------------------------------------------------------

void Tnc::loadSettings() {
  Preferences prefs;
#if defined(AXLORA_VARIANT_SENSECAP_SOLAR)
  settingsReady_ = false;
#endif
  if (prefs.begin("axloratnc", true)) {
#if defined(AXLORA_VARIANT_SENSECAP_SOLAR)
    settingsReady_ = true;
#endif
    prefs.getString("callsign", savedCallsign_, sizeof(savedCallsign_));
#if defined(AXLORA_VARIANT_SENSECAP_SOLAR)
    if (prefs.isKey("manager")) {
      prefs.getString("manager", manager_, sizeof(manager_));
      copySanitizedNodeText(manager_, sizeof(manager_), manager_);
    }
#endif
    radioConfig_.frequencyMHz    = prefs.getFloat("r_freq", variant::DEFAULT_FREQUENCY_MHZ);
    radioConfig_.frequencyCorrectionMHz = prefs.getFloat("r_fcorr", variant::DEFAULT_FREQUENCY_CORRECTION_MHZ);
    radioConfig_.bandwidthKhz    = prefs.getFloat("r_bw",   variant::DEFAULT_BANDWIDTH_KHZ);
    radioConfig_.spreadingFactor = prefs.getUChar("r_sf",   variant::DEFAULT_SPREADING_FACTOR);
    radioConfig_.codingRate      = prefs.getUChar("r_cr",   variant::DEFAULT_CODING_RATE);
    radioConfig_.powerDbm        = static_cast<int8_t>(prefs.getChar("r_pwr", variant::DEFAULT_TX_POWER_DBM));
    dutyCycleEnabled_            = prefs.getBool("duty_en", true);
    dutyCyclePpm_                = prefs.getULong("duty_ppm", variant::DUTY_CYCLE_PPM);
    serialMode_ = static_cast<SerialMode>(
        prefs.getUChar("mode", static_cast<uint8_t>(SerialMode::Console)));
    digi_.enabled  = prefs.getBool("digi_en",       digi_.enabled);
    digi_.hasAlias = prefs.getBool("digi_alias_en",  digi_.hasAlias);
    digi_.allFrames = prefs.getBool("digi_all",      digi_.allFrames);
    char addr[16]{};
    prefs.getString("digi_alias", addr, sizeof(addr));
    textToAddress(addr, digi_.alias);
    beacon_.enabled    = prefs.getBool("bc_en",  beacon_.enabled);
    beacon_.intervalMs = prefs.getULong("bc_int", beacon_.intervalMs);
    prefs.getString("bc_text", beacon_.text, sizeof(beacon_.text));
    copySanitizedNodeText(beacon_.text, sizeof(beacon_.text), beacon_.text);
    prefs.getString("bc_dest", addr, sizeof(addr));
    textToAddress(addr, beacon_.destination);
    beacon_.pathCount = prefs.getUChar("bc_path_n", 0);
    if (beacon_.pathCount > ax25::MAX_REPEATERS) beacon_.pathCount = 0;
    for (uint8_t i = 0; i < beacon_.pathCount; ++i) {
      char key[12]{};
      snprintf(key, sizeof(key), "bc_p%u", static_cast<unsigned>(i));
      prefs.getString(key, addr, sizeof(addr));
      if (!textToAddress(addr, beacon_.path[i])) { beacon_.pathCount = 0; break; }
    }
    netrom_.enabled             = prefs.getBool("nr_en",   netrom_.enabled);
    netrom_.broadcastIntervalMs = prefs.getULong("nr_int", netrom_.broadcastIntervalMs);
    prefs.getString("nr_alias", netrom_.alias, sizeof(netrom_.alias));
    prefs.getString("nr_ident", netrom_.ident, sizeof(netrom_.ident));
    copySanitizedNodeText(netrom_.ident, sizeof(netrom_.ident), netrom_.ident);
    // WA8DED / link parameters
    dedFrack_            = prefs.getUShort("d_frack",  dedFrack_);
    dedRetryLimit_       = prefs.getUChar ("d_n2",     dedRetryLimit_);
    dedMaxFrame_         = prefs.getUChar ("d_maxfr",  dedMaxFrame_);
    kissParams_.txDelay      = prefs.getUChar("d_txdel", kissParams_.txDelay);
    kissParams_.slotTime     = prefs.getUChar("d_slot",  kissParams_.slotTime);
    kissParams_.persistence  = prefs.getUChar("d_pers",  kissParams_.persistence);
    kissParams_.fullDuplex   = prefs.getUChar("d_fdup",  kissParams_.fullDuplex);
    dedTxEnabled_        = prefs.getBool  ("d_txen",   dedTxEnabled_);
    dedMaxIncoming_      = prefs.getUChar ("d_maxin",  dedMaxIncoming_);
    dedFlow_             = prefs.getBool  ("d_flow",   dedFlow_);
    dedXonXoff_          = prefs.getBool  ("d_xon",    dedXonXoff_);
    dedT2_               = prefs.getUShort("d_t2",     dedT2_);
    dedT3_               = prefs.getUShort("d_t3",     dedT3_);
    dedIPollFrameLength_ = prefs.getType("d_ipoll") == PT_U16
        ? prefs.getUShort("d_ipoll", dedIPollFrameLength_)
        : prefs.getUChar("d_ipoll", static_cast<uint8_t>(dedIPollFrameLength_));
    dedDamaTimeout_      = prefs.getUShort("d_dama",   dedDamaTimeout_);
    dedHeardMode_        = prefs.getUChar ("d_heard",  dedHeardMode_);
    dedEcho_             = prefs.getBool  ("d_echo",   dedEcho_);
    dedAutoLf_           = prefs.getBool  ("d_autolf", dedAutoLf_);
    dedTimestamp_        = prefs.getUChar ("d_ts",     dedTimestamp_);
    dedFlags_            = prefs.getBool("d_flags", false);
    dedUiPoll_           = prefs.getBool("d_uipoll", false);
    dedModemMode_        = prefs.getUChar("d_modem", 3);
    dedHdlcBaud_         = prefs.getUShort("d_hbaud", 1200);
    dedTerminalBaud_     = prefs.getUShort("d_tbaud", 9600);
    dedEightBitTerminal_ = prefs.getBool  ("d_8bit",   dedEightBitTerminal_);
    dedValidateCallsign_ = prefs.getBool  ("d_valcs",  dedValidateCallsign_);
    dedCtextMode_        = prefs.getUChar ("d_ctext",  dedCtextMode_);
    dedUnattended_       = dedCtextMode_ != 0;
    prefs.getString("d_utext",  dedUnattendedText_,  sizeof(dedUnattendedText_));
    copySanitizedNodeText(dedUnattendedText_, sizeof(dedUnattendedText_), dedUnattendedText_);
    prefs.getString("d_mon",    dedMonitorMode_,      sizeof(dedMonitorMode_));
    monitorEnabled_ = (strchr(dedMonitorMode_, 'N') == nullptr);
    char monitorFilter[96]{};
    prefs.getString("d_mfilter", monitorFilter, sizeof(monitorFilter));
    if (monitorFilter[0] != '\0' && !setDedMonitor(monitorFilter))
      LOG_WARN("invalid saved WA8DED monitor callsign filter");
    char uproto[16]{};
    prefs.getString("d_uproto", uproto, sizeof(uproto));
    if (uproto[0] != '\0') textToAddress(uproto, dedUnprotoDestination_);
    char savedPath[96]{};
    prefs.getString("d_upath", savedPath, sizeof(savedPath));
    if (savedPath[0] != '\0') {
      char destination[12]{}, value[128]{};
      ax25::formatAddress(dedUnprotoDestination_, destination, sizeof(destination));
      snprintf(value, sizeof(value), "%s %s", destination, savedPath);
      ax25::Address parsedDestination{}, path[ax25::MAX_REPEATERS]{};
      uint8_t count = 0;
      if (parseDedPath(value, parsedDestination, path, count, false)) {
        memcpy(dedUnprotoPath_, path, sizeof(path));
        dedUnprotoPathCount_ = count;
      } else {
        LOG_WARN("invalid saved WA8DED unproto path");
      }
    }
    baudKiss_ = prefs.getULong("baud_kiss", 9600U);
    baudDed_  = prefs.getULong("baud_ded",  9600U);
    if (!isValidBaud(baudKiss_)) baudKiss_ = 9600U;
    if (!isValidBaud(baudDed_))  baudDed_  = 9600U;
    prefs.end();
  }
#if defined(AXLORA_VARIANT_STATION_G2)
  if (radioConfig_.powerDbm < -9 || radioConfig_.powerDbm > variant::MAX_TX_POWER_DBM) {
    LOG_WARN("saved radio power outside Station G2 limits; using default");
    radioConfig_.powerDbm = variant::DEFAULT_TX_POWER_DBM;
  }
#endif
  if (serialMode_ != SerialMode::Console &&
      serialMode_ != SerialMode::Kiss &&
      serialMode_ != SerialMode::Wa8ded) {
    serialMode_ = SerialMode::Console;
  }
  axlora::util::setLogEnabled(serialMode_ == SerialMode::Console);
}

void Tnc::saveSettings() {
  Preferences prefs;
  if (!prefs.begin("axloratnc", false)) return;
  prefs.putBool("digi_en",      digi_.enabled);
  prefs.putBool("digi_alias_en", digi_.hasAlias);
  prefs.putBool("digi_all",      digi_.allFrames);
  char text[16]{};
  addressToText(digi_.alias, text, sizeof(text));
  prefs.putString("digi_alias", text);
  prefs.putBool("bc_en",   beacon_.enabled);
  prefs.putULong("bc_int", beacon_.intervalMs);
  prefs.putString("bc_text", beacon_.text);
  addressToText(beacon_.destination, text, sizeof(text));
  prefs.putString("bc_dest", text);
  prefs.putUChar("bc_path_n", beacon_.pathCount);
  for (uint8_t i = 0; i < beacon_.pathCount; ++i) {
    char key[12]{};
    snprintf(key, sizeof(key), "bc_p%u", static_cast<unsigned>(i));
    addressToText(beacon_.path[i], text, sizeof(text));
    prefs.putString(key, text);
  }
  prefs.putBool("nr_en",   netrom_.enabled);
  prefs.putULong("nr_int", netrom_.broadcastIntervalMs);
  prefs.putString("nr_alias", netrom_.alias);
  prefs.putString("nr_ident", netrom_.ident);
  prefs.end();
}

void Tnc::saveDedConfig() {
  Preferences prefs;
  if (!prefs.begin("axloratnc", false)) return;
  prefs.putUShort("d_frack",  dedFrack_);
  prefs.putUChar ("d_n2",     dedRetryLimit_);
  prefs.putUChar ("d_maxfr",  dedMaxFrame_);
  prefs.putUChar ("d_txdel",  kissParams_.txDelay);
  prefs.putUChar ("d_slot",   kissParams_.slotTime);
  prefs.putUChar ("d_pers",   kissParams_.persistence);
  prefs.putUChar ("d_fdup",   kissParams_.fullDuplex);
  prefs.putBool  ("d_txen",   dedTxEnabled_);
  prefs.putUChar ("d_maxin",  dedMaxIncoming_);
  prefs.putBool  ("d_flow",   dedFlow_);
  prefs.putBool  ("d_xon",    dedXonXoff_);
  prefs.putUShort("d_t2",     dedT2_);
  prefs.putUShort("d_t3",     dedT3_);
  prefs.putUShort("d_ipoll",  dedIPollFrameLength_);
  prefs.putUShort("d_dama",   dedDamaTimeout_);
  prefs.putUChar ("d_heard",  dedHeardMode_);
  prefs.putBool  ("d_echo",   dedEcho_);
  prefs.putBool  ("d_autolf", dedAutoLf_);
  prefs.putUChar ("d_ts",     dedTimestamp_);
  prefs.putBool("d_flags", dedFlags_);
  prefs.putBool("d_uipoll", dedUiPoll_);
  prefs.putUChar("d_modem", dedModemMode_);
  prefs.putUShort("d_hbaud", dedHdlcBaud_);
  prefs.putUShort("d_tbaud", dedTerminalBaud_);
  prefs.putBool  ("d_8bit",   dedEightBitTerminal_);
  prefs.putBool  ("d_valcs",  dedValidateCallsign_);
  prefs.putUChar ("d_ctext",  dedCtextMode_);
  prefs.putString("d_utext",  dedUnattendedText_);
  prefs.putString("d_mon",    dedMonitorMode_);
  char filter[96]{};
  if (dedMonitorCallFilter_ != '\0') {
    filter[0] = dedMonitorCallFilter_;
    for (uint8_t i = 0; i < dedMonitorCallCount_; ++i) {
      char call[12]{};
      ax25::formatAddress(dedMonitorCalls_[i], call, sizeof(call));
      const size_t pos = strlen(filter);
      snprintf(filter + pos, sizeof(filter) - pos, " %s", call);
    }
  }
  prefs.putString("d_mfilter", filter);
  char uproto[16]{};
  addressToText(dedUnprotoDestination_, uproto, sizeof(uproto));
  prefs.putString("d_uproto", uproto);
  char path[96]{};
  for (uint8_t i = 0; i < dedUnprotoPathCount_; ++i) {
    char call[12]{};
    ax25::formatAddress(dedUnprotoPath_[i], call, sizeof(call));
    const size_t pos = strlen(path);
    snprintf(path + pos, sizeof(path) - pos, "%s%s", i == 0 ? "" : " ", call);
  }
  prefs.putString("d_upath", path);
  prefs.end();
}

void Tnc::saveKissConfig() {
  Preferences prefs;
  if (!prefs.begin("axloratnc", false)) return;
  prefs.putUChar("d_txdel", kissParams_.txDelay);
  prefs.putUChar("d_slot",  kissParams_.slotTime);
  prefs.putUChar("d_pers",  kissParams_.persistence);
  prefs.putUChar("d_fdup",  kissParams_.fullDuplex);
  prefs.end();
}

void Tnc::saveRadioConfig() {
  Preferences prefs;
  if (!prefs.begin("axloratnc", false)) return;
  prefs.putFloat("r_freq", radioConfig_.frequencyMHz);
  prefs.putFloat("r_fcorr", radioConfig_.frequencyCorrectionMHz);
  prefs.putFloat("r_bw",   radioConfig_.bandwidthKhz);
  prefs.putUChar("r_sf",   radioConfig_.spreadingFactor);
  prefs.putUChar("r_cr",   radioConfig_.codingRate);
  prefs.putChar("r_pwr",   radioConfig_.powerDbm);
  prefs.putBool("duty_en", dutyCycleEnabled_);
  prefs.putULong("duty_ppm", dutyCyclePpm_);
  prefs.end();
}

void Tnc::applyRadioConfig() {
  auto& drv = radio::driver();
  drv.setDutyCycle(dutyCycleEnabled_, dutyCyclePpm_);
  drv.setFrequencyCorrection(radioConfig_.frequencyCorrectionMHz);
  drv.setFrequency(radioConfig_.frequencyMHz);
  drv.setSpreadingFactor(radioConfig_.spreadingFactor);
  drv.setBandwidth(radioConfig_.bandwidthKhz);
  drv.setCodingRate(radioConfig_.codingRate);
  drv.setPower(radioConfig_.powerDbm);
  LOG_RADIO("radio config applied: freq=%.*f corr=%+.1fkHz sf=%u bw=%.1f cr=%u pwr=%d",
            FREQUENCY_DECIMALS, static_cast<double>(radioConfig_.frequencyMHz),
            static_cast<double>(radioConfig_.frequencyCorrectionMHz * 1000.0f),
            radioConfig_.spreadingFactor,
            static_cast<double>(radioConfig_.bandwidthKhz),
            radioConfig_.codingRate,
            radioConfig_.powerDbm);
}

void Tnc::saveSerialMode(SerialMode /*mode*/) {
  // Mode is intentionally not persisted — always starts in Console after reset.
}

void Tnc::setSerialMode(SerialMode mode) {
#if defined(AXLORA_VARIANT_STATION_G2)
  if (mode != SerialMode::Console && wifiKiss_.active()) {
    LOG_WARN("USB host taking over from KISS TCP");
    wifiKiss_.closeClient();
  }
#endif
  serialMode_   = mode;
  dedHostMode_  = false;
  kiss_.reset();
  linePos_      = 0;
  escapePos_    = 0;
  dedTerminalCommand_ = false;
  dedOutputPaused_ = false;
  dedHeaderPos_ = 0;
  dedDataPos_   = 0;
  dedDataLen_   = 0;
  dedMonitorPayloadPending_ = false;
  for (size_t i = 0; i < dedEvents_.size();) {
    DedEvent event{};
    if (!dedEvents_.peek(i, event)) break;
    if (event.channel == 0 && event.code >= 4 && event.code <= 6)
      dedEvents_.remove(i, event);
    else ++i;
  }
  applySerialBaud();
  axlora::util::setLogEnabled(serialMode_ == SerialMode::Console);
}

void Tnc::applySerialBaud() {
  uint32_t baud = variant::SERIAL_BAUD;
  if (serialMode_ == SerialMode::Kiss)   baud = baudKiss_;
  if (serialMode_ == SerialMode::Wa8ded) baud = baudDed_;
#if defined(NRF52_PLATFORM) || (defined(ARDUINO_USB_CDC_ON_BOOT) && ARDUINO_USB_CDC_ON_BOOT)
  Serial.begin(baud);
#else
  Serial.updateBaudRate(baud);
  // Hardware RTS/CTS flow control: only active in non-console modes where the
  // host may be an old DOS application that drives flow-control lines.
  // Disabled at compile time when PIN_UART_RTS/CTS are -1 (all current variants).
  if constexpr (variant::PIN_UART_RTS >= 0 && variant::PIN_UART_CTS >= 0) {
    if (serialMode_ == SerialMode::Wa8ded || serialMode_ == SerialMode::Kiss) {
      // Re-assign UART0 pins so the hardware CTS/RTS lines are connected.
      Serial.setPins(3 /*RX*/, 1 /*TX*/,
                     variant::PIN_UART_CTS, variant::PIN_UART_RTS);
      Serial.setHwFlowCtrlMode(UART_HW_FLOWCTRL_CTS_RTS, 64);
    } else {
      Serial.setHwFlowCtrlMode(UART_HW_FLOWCTRL_DISABLE, 0);
    }
  }
#endif
}

const char* Tnc::serialModeName() const {
  switch (serialMode_) {
    case SerialMode::Console: return "console";
    case SerialMode::Kiss:    return "kiss";
    case SerialMode::Wa8ded:  return "ded";
    default:                  return "?";
  }
}

}  // namespace axlora::tnc
