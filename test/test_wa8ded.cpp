#include "native/Arduino.h"
#include "native/Preferences.h"
#include <cassert>
#include <iostream>
#include "ax25/ax25_fcs.h"
#include "tnc/tnc.h"
#include "tnc/ded_protocol.h"
#include "variant/sensecap_solar_power.h"

using namespace axlora;

namespace axlora::tnc {
struct TncTestAccess {
  static void hostmode(Tnc& modem) {
    modem.serialMode_ = SerialMode::Wa8ded;
    modem.dedHostMode_ = true;
    modem.dedTxEnabled_ = false;
  }
  static void command(Tnc& modem, uint8_t channel, const char* text) {
    modem.handleDedHostFrame(channel, 1, reinterpret_cast<const uint8_t*>(text), strlen(text));
  }
  static void receive(Tnc& modem) { modem.serviceRadio(); }
  static void status(Tnc& modem) { modem.checkLinkStatusEvents(); }
  static void byte(Tnc& modem, uint8_t byte) { modem.serviceWa8dedHost(byte); }
  static void loopback(Tnc& modem) { modem.serviceLoopbackTx(); }
  static void terminalLine(Tnc& modem, uint8_t channel, const char* text) {
    modem.dedSelectedChannel_ = channel;
    modem.handleDedTerminalLine(text, false);
  }
  static void clearEvents(Tnc& modem) { modem.dedEvents_.clear(); }
  static bool enqueue(Tnc& modem, uint8_t channel, uint8_t code, const uint8_t* data, size_t len) {
    return modem.enqueueDedEvent(channel, code, data, len);
  }
  static void kiss(Tnc& modem, const ax25::KissFrame& frame) { modem.handleKiss(frame); }
  static void enableTransmit(Tnc& modem) { modem.dedTxEnabled_ = true; }
  static bool raw(Tnc& modem, const uint8_t* data, size_t len) {
    return modem.enqueueRawTx(data, len, 0);
  }
  static bool popRawBytes(Tnc& modem, std::vector<uint8_t>& bytes) {
    Tnc::PendingRawTx tx{};
    if (!modem.rawTxQueue_.pop(tx)) return false;
    bytes.assign(tx.data, tx.data + tx.len);
    return true;
  }
#if defined(AXLORA_VARIANT_SENSECAP_SOLAR)
  static void console(Tnc& modem, char* text) { modem.handleConsoleLine(text); }
  static void relayResponses(Tnc& modem, uint8_t channel) { modem.serviceRelayResponses(channel); }
  static void updateRelayUptime(Tnc& modem) { modem.updateRelayUptime(); }
  static void input(Tnc& modem, uint8_t channel, const char* text) {
    modem.processNodeInput(channel, reinterpret_cast<const uint8_t*>(text), strlen(text));
  }
  static void heard(Tnc& modem, const ax25::Frame& frame) { modem.observeHeard(frame, -90, 8); }
  static bool repeat(Tnc& modem, const ax25::Frame& frame) { return modem.maybeDigipeat(frame); }
  static bool popRaw(Tnc& modem, ax25::Frame& frame, bool priority = false) {
    Tnc::PendingRawTx tx{};
    auto& queue = priority ? modem.rawTxPriorityQueue_ : modem.rawTxQueue_;
    return queue.pop(tx) && ax25::decodeFrame(tx.data, tx.len, frame, true);
  }
  static bool mheardPending(Tnc& modem, uint8_t channel) {
    return modem.channels_[channel].relayMheardPending;
  }
  static const char* manager(Tnc& modem) { return modem.manager_; }
  static ax25::LinkState state(Tnc& modem, uint8_t channel) {
    return modem.channels_[channel].link.state();
  }
  static void connectedInput(Tnc& modem, uint8_t channel, uint8_t nr, const char* text) {
    ax25::LinkLayer& link = modem.channels_[channel].link;
    ax25::Frame frame{};
    frame.source = link.peer();
    frame.destination = link.local();
    frame.command = true;
    frame.control = ax25::makeI(0, nr, false);
    frame.pid = ax25::PID_NO_LAYER3;
    frame.infoLen = strlen(text);
    assert(frame.infoLen <= sizeof(frame.info));
    memcpy(frame.info, text, frame.infoLen);
    uint8_t encoded[MAX_PACKET_BYTES]{};
    size_t length = 0;
    assert(ax25::encodeFrame(frame, encoded, sizeof(encoded), length, true));
    link.receive(encoded, length);
  }
  static void acknowledge(Tnc& modem, uint8_t channel, uint8_t& next) {
    ax25::LinkLayer& link = modem.channels_[channel].link;
    const uint8_t count = link.outstandingFrameCount();
    if (count == 0) return;
    next = static_cast<uint8_t>((next + count) & 7);
    ax25::Frame frame{};
    frame.source = link.peer();
    frame.destination = link.local();
    frame.command = false;
    frame.control = ax25::makeS(ax25::SFrameType::RR, next, false);
    uint8_t encoded[MAX_PACKET_BYTES]{};
    size_t length = 0;
    assert(ax25::encodeFrame(frame, encoded, sizeof(encoded), length, true));
    link.receive(encoded, length);
  }
  static bool relayDrained(Tnc& modem, uint8_t channel) {
    const auto& cs = modem.channels_[channel];
    return !cs.relayMheardPending && cs.link.connectedQueueSize() == 0 &&
           cs.link.outstandingFrameCount() == 0;
  }
#endif
};
}

#if defined(AXLORA_VARIANT_SENSECAP_SOLAR)
namespace axlora::variant {
bool testBatteryAvailable = true;
uint16_t testBatteryMillivolts = 3975;
bool readBatteryMillivolts(uint16_t& millivolts) {
  if (!testBatteryAvailable) return false;
  millivolts = testBatteryMillivolts;
  return true;
}
}
#endif

namespace axlora::util {
bool logEnabled() { return false; }
void setLogEnabled(bool) {}
}

namespace axlora::radio {
class TestRadio : public Driver {
 public:
  std::deque<RxPacket> packets;
  unsigned frequencyUpdates = 0;
  Result init() override { return Result::Ok; }
  Result send(const uint8_t*, size_t) override { return Result::Ok; }
  Result receive(RxPacket& packet) override {
    if (packets.empty()) return Result::NoPacket;
    packet = packets.front(); packets.pop_front(); return Result::Ok;
  }
  Result setFrequency(float) override { ++frequencyUpdates; return Result::Ok; }
  Result setFrequencyCorrection(float) override { return Result::Ok; }
  Result setPower(int8_t) override { return Result::Ok; }
  Result setSpreadingFactor(uint8_t) override { return Result::Ok; }
  Result setBandwidth(float) override { return Result::Ok; }
  Result setCodingRate(uint8_t) override { return Result::Ok; }
  void setDutyCycle(bool, uint32_t) override {}
  bool dutyCycleEnabled() const override { return false; }
  uint32_t dutyCyclePpm() const override { return 0; }
  float getRSSI() override { return 0; }
  float getSNR() override { return 0; }
  void sleep() override {}
  void standby() override {}
} testRadio;
Driver& driver() { return testRadio; }
Stats& stats() { static Stats value; return value; }
const char* resultName(Result) { return "TEST"; }
}

static tnc::Tnc modem;

std::vector<uint8_t> command(uint8_t channel, const char* text) {
  Serial.output.clear();
  tnc::TncTestAccess::command(modem, channel, text);
  return Serial.output;
}

void reply(uint8_t channel, const char* text, uint8_t code, const char* payload = nullptr) {
  std::vector<uint8_t> expected{channel, code};
  if (payload != nullptr) {
    expected.insert(expected.end(), payload, payload + strlen(payload));
    expected.push_back(0);
  }
  const auto actual = command(channel, text);
  if (actual != expected) {
    std::cerr << "Unexpected reply to ch" << unsigned(channel) << " " << text << "\n";
    for (uint8_t byte : actual) std::cerr << unsigned(byte) << " ";
    std::cerr << "\n";
    std::abort();
  }
}

void inject(const ax25::Frame& frame) {
  radio::RxPacket packet{};
  assert(ax25::encodeFrame(frame, packet.data, sizeof(packet.data), packet.len, true));
  radio::testRadio.packets.push_back(packet);
  testMillis += 200;
  tnc::TncTestAccess::receive(modem);
}

void testCommands() {
  reply(0, "F500", 0);
  reply(0, "F", 1, "500");
  reply(1, "F1200", 0);
  reply(1, "F", 1, "1200");
  reply(2, "F", 1, "500");
  reply(0, "F", 1, "500");
  reply(1, "N7", 0); reply(1, "N", 1, "7"); reply(2, "N", 1, "20");
  reply(1, "O1", 0); reply(1, "O", 1, "1"); reply(2, "O", 1, "3");
  reply(0, "@T2 150", 0); reply(0, "@T2", 1, "150");
  reply(0, "@T3 18000", 0); reply(0, "@T3", 1, "18000");
  reply(0, "@V0", 0); reply(0, "@V", 1, "0");
  reply(0, "@D1", 0); reply(0, "@D", 1, "1");
  reply(0, "@D0", 0);
  reply(0, "@I256", 0); reply(0, "@I", 1, "256");
  reply(0, "@H2", 0); reply(0, "@H", 1, "2");
  reply(0, "@N9600", 0); reply(0, "@N", 1, "9600");
  reply(0, "@O19200", 0); reply(0, "@O", 1, "19200");
  reply(0, "@U1", 0); reply(0, "@U", 1, "1");
  reply(0, "@F1", 0); reply(0, "@F", 1, "1");
  reply(0, "JHOST", 1, "1");
  reply(0, "Y8", 0); reply(0, "Y", 1, "8 (0)");
  reply(0, "U1Away!", 0); reply(0, "U", 1, "1 Away!");
  reply(0, "U0", 0); reply(0, "U", 1, "0 Away!");
  reply(0, "C CQ N1DIG N2DIG", 0);
  reply(0, "C", 1, "CQ via N1DIG N2DIG");
  assert(Preferences::values.at("axloratnc:d_upath").text == "N1DIG N2DIG");
  reply(0, "F0", 2, "INVALID VALUE: 0");
  reply(0, "@I257", 2, "INVALID VALUE: 257");
  reply(0, "@N1234", 2, "INVALID VALUE: 1234");
  reply(0, "U1", 0);
  std::string oversized = "U1 " + std::string(73, 'x');
  reply(0, oversized.c_str(), 2, "MESSAGE TOO LONG");
  reply(0, "U0", 0);
  reply(1, "C", 1, "CHANNEL NOT CONNECTED");
  reply(1, "D", 1, "CHANNEL NOT CONNECTED");
  reply(0, "H", 2, "INVALID COMMAND: H");
  reply(0, "S", 2, "INVALID COMMAND: S");
  reply(0, "@S", 2, "INVALID EXTENDED COMMAND: S");
  reply(11, "I", 2, "INVALID CHANNEL NUMBER");
  reply(11, "G", 2, "INVALID CHANNEL NUMBER");
  reply(0, "A2", 2, "INVALID VALUE: 2");
  reply(0, "T65", 2, "INVALID VALUE: 65");
  reply(0, "G2", 2, "INVALID VALUE: 2");
  reply(0, "M IUN", 2, "INVALID PARAMETER");
  reply(0, "K 28.02.24", 0); reply(0, "K 23:59:59", 0); reply(0, "K2", 0);
  testMillis += 2000;
  reply(0, "K", 1, "2 29.02.24 00:00:01");
  reply(0, "K 31.02.24", 2, "INVALID PARAMETER");
  reply(0, "K0", 0);
}

void testMonitorAndPolling() {
  tnc::TncTestAccess::clearEvents(modem);
  reply(0, "M IUSC + N1SRC", 0);
  reply(0, "M", 1, "IUSC + N1SRC");
  ax25::Frame frame{};
  ax25::parseAddress("N1SRC-3", frame.source);
  ax25::parseAddress("CQ", frame.destination);
  frame.infoLen = 3; memcpy(frame.info, "\0\xC0\xDB", 3);
  inject(frame);
  reply(0, "L", 1, "0 1");
  assert(command(255, "G") == std::vector<uint8_t>({255, 1, 1, 0}));
  reply(0, "G1", 0);
  reply(0, "G 0", 5, "fm N1SRC-3 to CQ ctl UI^ pid F0");
  reply(0, "L", 1, "0 0");
  assert(command(0, "G0") == std::vector<uint8_t>({0, 6, 2, 0, 0xC0, 0xDB}));
  reply(0, "G", 0);
  reply(255, "G", 1, "");
  frame.control = ax25::makeS(ax25::SFrameType::RR, 3, false);
  frame.command = false; frame.infoLen = 0;
  inject(frame);
  reply(0, "G0", 4, "fm N1SRC-3 to CQ ctl RR3v");
  frame.control = ax25::makeI(2, 4, true); frame.command = true;
  char header[160]{};
  tnc::ded::monitorHeader(frame, header, sizeof(header));
  assert(strcmp(header, "fm N1SRC-3 to CQ ctl I42+ pid F0") == 0);
  reply(0, "M U -", 0);
  frame.control = ax25::makeU(ax25::UFrameType::SABM, true);
  inject(frame); reply(0, "G", 0);
  reply(0, "M S", 0); inject(frame);
  reply(0, "G0", 4, "fm N1SRC-3 to CQ ctl SABM+");
  tnc::TncTestAccess::clearEvents(modem);
  const uint8_t byte = 42;
  assert(tnc::TncTestAccess::enqueue(modem, 1, 7, &byte, 1));
  assert(tnc::TncTestAccess::enqueue(modem, 2, 7, &byte, 1));
  assert(tnc::TncTestAccess::enqueue(modem, 1, 3, reinterpret_cast<const uint8_t*>("status"), 6));
  reply(1, "G1", 3, "status");
  assert(command(1, "G0") == std::vector<uint8_t>({1, 7, 0, 42}));
  assert(command(2, "G0") == std::vector<uint8_t>({2, 7, 0, 42}));
}

void testChannelsAndMaximumPayload() {
  tnc::TncTestAccess::clearEvents(modem);
  reply(0, "M N", 0);
  ax25::Frame frame{};
  ax25::parseAddress("N1LOC", frame.destination);
  ax25::parseAddress("N1PEER", frame.source);
  ax25::parseAddress("N1DIG", frame.repeaters[0]);
  frame.repeaterCount = 1;
  frame.control = ax25::makeU(ax25::UFrameType::SABM, true);
  inject(frame);
  tnc::TncTestAccess::status(modem);
  reply(1, "G1", 3, "(1) CONNECTED to N1PEER via N1DIG");
  reply(1, "L", 1, "0 0 0 0 0 4");
  reply(0, "F600", 0); reply(1, "F", 1, "1200");
  inject(frame);
  tnc::TncTestAccess::status(modem);
  reply(1, "G1", 3, "(1) LINK RESET fm N1PEER via N1DIG");
  frame.control = ax25::makeI(0, 0, true);
  frame.infoLen = 256;
  for (size_t i = 0; i < frame.infoLen; ++i) frame.info[i] = static_cast<uint8_t>(i);
  inject(frame);
  reply(1, "L", 1, "0 1 0 0 0 4");
  std::vector<uint8_t> expected{1, 7, 255};
  expected.insert(expected.end(), frame.info, frame.info + 256);
  assert(command(1, "G0") == expected);
  const uint8_t data = 7;
  for (size_t i = 0; i < 32; ++i)
    assert(tnc::TncTestAccess::enqueue(modem, 2, 7, &data, 1));
  frame.control = ax25::makeU(ax25::UFrameType::DM, true);
  frame.infoLen = 0;
  inject(frame);
  tnc::TncTestAccess::status(modem);
  assert(command(2, "G0") == std::vector<uint8_t>({2, 7, 0, data}));
  tnc::TncTestAccess::status(modem);
  reply(1, "G1", 3, "(1) DISCONNECTED fm N1PEER via N1DIG");
  reply(1, "F", 1, "600");
  reply(0, "F500", 0);
  reply(1, "N", 1, "20");
  reply(1, "O", 1, "3");
  tnc::TncTestAccess::clearEvents(modem);
  Serial.output.clear();
  for (uint8_t byte : {uint8_t(1), uint8_t(0), uint8_t(255)})
    tnc::TncTestAccess::byte(modem, byte);
  for (unsigned i = 0; i < 256; ++i)
    tnc::TncTestAccess::byte(modem, static_cast<uint8_t>(i));
  assert(Serial.output == std::vector<uint8_t>({1, 0}));
}

void testIncomingConnectText() {
  reply(0, "C CQ", 0);
  reply(0, "U1 Welcome!", 0);
  reply(1, "C N1LOC", 0);
  tnc::TncTestAccess::status(modem);
  tnc::TncTestAccess::loopback(modem);
  tnc::TncTestAccess::status(modem);
  tnc::TncTestAccess::status(modem);
  tnc::TncTestAccess::loopback(modem);
  const auto received = command(1, "G0");
  const char text[] = "Welcome!\r";
  std::vector<uint8_t> expected{1, 7, sizeof(text) - 2};
  expected.insert(expected.end(), text, text + sizeof(text) - 1);
  assert(received == expected);
  reply(2, "G0", 0);
  tnc::TncTestAccess::terminalLine(modem, 1, "hi");
  tnc::TncTestAccess::loopback(modem);
  assert(command(2, "G0") == std::vector<uint8_t>({2, 7, 2, 'h', 'i', '\r'}));
  reply(0, "U0", 0);
}

struct Receiver {
  bool ready = false;
  std::vector<ax25::Frame> sent;
  std::vector<uint8_t> received;
};
bool captureTx(const uint8_t* data, size_t len, void* context) {
  ax25::Frame frame{};
  assert(ax25::decodeFrame(data, len, frame, true));
  static_cast<Receiver*>(context)->sent.push_back(frame);
  return true;
}
bool captureData(const uint8_t* data, size_t len, bool, void* context) {
  auto& receiver = *static_cast<Receiver*>(context);
  if (!receiver.ready) return false;
  receiver.received.insert(receiver.received.end(), data, data + len);
  return true;
}
bool receiverReady(void* context) { return static_cast<Receiver*>(context)->ready; }

void testReceiveBackpressure() {
  ax25::LinkLayer link;
  ax25::L2Config config{};
  ax25::parseAddress("N1LOC", config.local);
  Receiver receiver;
  link.begin(config, captureTx, captureData, &receiver, receiverReady);
  ax25::Frame incoming{};
  incoming.destination = config.local;
  ax25::parseAddress("N1PEER", incoming.source);
  auto deliver = [&] {
    uint8_t data[MAX_PACKET_BYTES]{}; size_t len = 0;
    assert(ax25::encodeFrame(incoming, data, sizeof(data), len, true));
    link.receive(data, len);
  };
  incoming.control = ax25::makeU(ax25::UFrameType::SABM, true); deliver();
  incoming.control = ax25::makeI(0, 0, true);
  incoming.infoLen = 3; memcpy(incoming.info, "abc", 3); deliver();
  assert(receiver.received.empty());
  assert(link.hostState() == 7);
  assert(ax25::sType(receiver.sent.back().control) == ax25::SFrameType::RNR);
  assert(ax25::nr(receiver.sent.back().control) == 0);
  receiver.ready = true; link.loop();
  assert(link.hostState() == 4);
  assert(ax25::sType(receiver.sent.back().control) == ax25::SFrameType::RR);
  assert(ax25::nr(receiver.sent.back().control) == 0);
  deliver();
  assert(receiver.received == std::vector<uint8_t>({'a', 'b', 'c'}));
  assert(ax25::nr(receiver.sent.back().control) == 1);
  deliver();
  assert(receiver.received.size() == 3);
  incoming.control = ax25::makeU(ax25::UFrameType::SABM, true);
  incoming.infoLen = 0;
  deliver();
  link.setIPollLength(4);
  assert(link.sendConnected(reinterpret_cast<const uint8_t*>("abc"), 3));
  testMillis += config.t1Ms + 1; link.loop();
  assert(link.hostState() == 6);
  assert(ax25::kind(receiver.sent.back().control) == ax25::FrameKind::I);
  assert((receiver.sent.back().control & 0x10) != 0);
  testMillis += config.t1Ms + 1; link.loop();
  testMillis += config.t1Ms + 1; link.loop();
  assert(ax25::kind(receiver.sent.back().control) == ax25::FrameKind::S);
  assert((receiver.sent.back().control & 0x10) != 0);
  deliver();
  link.setIPollLength(3);
  assert(link.sendConnected(reinterpret_cast<const uint8_t*>("abc"), 3));
  testMillis += config.t1Ms + 1; link.loop();
  assert(ax25::kind(receiver.sent.back().control) == ax25::FrameKind::S);
  assert((receiver.sent.back().control & 0x10) != 0);

  ax25::Address path{};
  ax25::parseAddress("N1DIG", path);
  link.setPath(&path, 1);
  ax25::Address uiPath{};
  ax25::parseAddress("N2DIG", uiPath);
  assert(link.sendUi(incoming.source, reinterpret_cast<const uint8_t*>("ui"), 2,
                     &uiPath, 1, true));
  assert(ax25::addressEquals(link.path()[0], path));
  assert(ax25::addressEquals(receiver.sent.back().repeaters[0], uiPath));
  assert((receiver.sent.back().control & 0x10) != 0);
  assert(link.sendUi(incoming.source, reinterpret_cast<const uint8_t*>("ui"), 2, &uiPath, 0));
  assert(receiver.sent.back().repeaterCount == 0);
  ax25::Address uiSource{};
  ax25::parseAddress("N2SRC", uiSource);
  assert(link.sendUi(incoming.source, reinterpret_cast<const uint8_t*>("ui"), 2,
                     &uiPath, 0, false, &uiSource));
  assert(ax25::addressEquals(receiver.sent.back().source, uiSource));
  assert(ax25::addressEquals(link.local(), config.local));
}

void testOversizedRadioFrames() {
  static tnc::Tnc sender;
  sender.begin("N1LOC");
  tnc::TncTestAccess::enableTransmit(sender);
  ax25::Frame frame{};
  assert(ax25::parseAddress("N1LOC", frame.source));
  assert(ax25::parseAddress("N1PEER", frame.destination));
  frame.control = ax25::makeI(3, 5, true);
  frame.infoLen = 237;
  memset(frame.info, 'x', sizeof(frame.info));
  ax25::KissFrame kiss{};
  assert(ax25::encodeFrame(frame, kiss.data, sizeof(kiss.data), kiss.len, false));
  std::vector<uint8_t> sent;
  tnc::TncTestAccess::kiss(sender, kiss);
  assert(tnc::TncTestAccess::popRawBytes(sender, sent));
  assert(sent.size() == MAX_LORA_PACKET_BYTES);
  assert(memcmp(sent.data(), kiss.data, kiss.len) == 0);
  assert(ax25::checkFcs(sent.data(), sent.size()));

  frame.repeaterCount = ax25::MAX_REPEATERS;
  for (auto& repeater : frame.repeaters) assert(ax25::parseAddress("N1DIG", repeater));
  frame.infoLen = 196;
  assert(ax25::encodeFrame(frame, kiss.data, sizeof(kiss.data), kiss.len, false));
  tnc::TncTestAccess::kiss(sender, kiss);
  assert(tnc::TncTestAccess::popRawBytes(sender, sent));
  assert(sent.size() <= MAX_LORA_PACKET_BYTES);
  assert(memcmp(sent.data(), kiss.data, 58) == 0);
  ax25::Frame decoded{};
  assert(ax25::decodeFrame(sent.data(), sent.size(), decoded, true));
  assert(decoded.control == frame.control);
  assert(std::string(reinterpret_cast<const char*>(decoded.info), decoded.infoLen) ==
         "\rAXLoRa: payload too large. Max payload 195 bytes, payload 196 bytes "
         "(packet 256 bytes, max 255).\r");

  uint8_t raw[MAX_PACKET_BYTES]{};
  size_t rawLen = 0;
  assert(ax25::encodeFrame(frame, raw, sizeof(raw), rawLen, true));
  assert(tnc::TncTestAccess::raw(sender, raw, rawLen));
  assert(tnc::TncTestAccess::popRawBytes(sender, sent));
  assert(ax25::decodeFrame(sent.data(), sent.size(), decoded, true));
  assert(decoded.control == frame.control && decoded.repeaterCount == frame.repeaterCount);
  raw[rawLen - 1] ^= 1;
  assert(!tnc::TncTestAccess::raw(sender, raw, rawLen));

  frame.control = ax25::makeU(ax25::UFrameType::UI, false);
  frame.repeaterCount = 0;
  frame.infoLen = 237;
  assert(ax25::encodeFrame(frame, kiss.data, sizeof(kiss.data), kiss.len, false));
  memmove(kiss.data + 17, kiss.data + 16, frame.infoLen);
  kiss.data[15] = ax25::PID_ESCAPE;
  kiss.data[16] = ax25::PID_NO_LAYER3;
  ++kiss.len;
  tnc::TncTestAccess::kiss(sender, kiss);
  assert(tnc::TncTestAccess::popRawBytes(sender, sent));
  assert(memcmp(sent.data(), kiss.data, 17) == 0);
  assert(ax25::decodeFrame(sent.data(), sent.size(), decoded, true));
  assert(std::string(reinterpret_cast<const char*>(decoded.info), decoded.infoLen) ==
         "\rAXLoRa: payload too large. Max payload 236 bytes, payload 237 bytes "
         "(packet 256 bytes, max 255).\r");

  frame.infoLen = 256;
  assert(ax25::encodeFrame(frame, kiss.data, sizeof(kiss.data), kiss.len, false));
  memset(kiss.data + kiss.len, 'z', sizeof(kiss.data) - kiss.len);
  kiss.len = sizeof(kiss.data);
  tnc::TncTestAccess::kiss(sender, kiss);
  assert(tnc::TncTestAccess::popRawBytes(sender, sent));
  assert(ax25::decodeFrame(sent.data(), sent.size(), decoded, true));
  assert(std::string(reinterpret_cast<const char*>(decoded.info), decoded.infoLen) ==
         "\rAXLoRa: payload too large. Max payload 237 bytes, payload 314 bytes "
         "(packet 332 bytes, max 255).\r");

  frame.control = ax25::makeU(ax25::UFrameType::TEST, false);
  assert(ax25::encodeFrame(frame, raw, sizeof(raw), rawLen, true));
  assert(!tnc::TncTestAccess::raw(sender, raw, rawLen));
  assert(!tnc::TncTestAccess::popRawBytes(sender, sent));
}

#if defined(AXLORA_VARIANT_SENSECAP_SOLAR)
void testSolarRelay() {
  testMillis = 0;
  Preferences::values.clear();
  Preferences::failBegin = true;
  static tnc::Tnc unavailable;
  unavailable.begin("N1DIG");
  assert(!unavailable.settingsReady());
  Preferences::failBegin = false;
  static tnc::Tnc relay;
  relay.begin("N1DIG");
  assert(relay.settingsReady());
  char manager[] = "manager Example Operator";
  tnc::TncTestAccess::console(relay, manager);
  assert(Preferences::values.at("axloratnc:manager").text == "Example Operator");
  static tnc::Tnc reloaded;
  reloaded.begin("N1DIG");
  assert(strcmp(tnc::TncTestAccess::manager(reloaded), "Example Operator") == 0);
  radio::testRadio.frequencyUpdates = 0;
  relay.loop(true);
  relay.loop(false);
  relay.loop(true);
  assert(radio::testRadio.frequencyUpdates == 2);

  ax25::Frame frame{};
  ax25::parseAddress("N1PEER", frame.source);
  ax25::parseAddress("N1DEST", frame.destination);
  ax25::parseAddress("N1DIG", frame.repeaters[0]);
  frame.repeaterCount = 1;
  frame.control = ax25::makeU(ax25::UFrameType::UI, false);
  assert(tnc::TncTestAccess::repeat(relay, frame));
  ax25::Frame repeated{};
  assert(tnc::TncTestAccess::popRaw(relay, repeated));
  assert(repeated.repeaters[0].repeated);
  frame.control = ax25::makeU(ax25::UFrameType::SABM, true);
  assert(tnc::TncTestAccess::repeat(relay, frame));
  assert(tnc::TncTestAccess::popRaw(relay, repeated, true));
  assert(repeated.repeaters[0].repeated);
  ax25::parseAddress("N2DIG", frame.repeaters[0]);
  assert(!tnc::TncTestAccess::repeat(relay, frame));

  frame.repeaterCount = 0;
  ax25::parseAddress("N1DIG", frame.destination);
  radio::RxPacket packet{};
  assert(ax25::encodeFrame(frame, packet.data, sizeof(packet.data), packet.len, true));
  radio::testRadio.packets.push_back(packet);
  Serial.output.clear();
  tnc::TncTestAccess::receive(relay);
  assert(Serial.output.empty());
  tnc::TncTestAccess::status(relay);
  tnc::TncTestAccess::relayResponses(relay, 0);
  assert(tnc::TncTestAccess::popRaw(relay, repeated));
  assert(std::string(reinterpret_cast<const char*>(repeated.info), repeated.infoLen) ==
         "This station is managed by Example Operator.\r");

  tnc::TncTestAccess::input(relay, 0, "ignored\r//mHeArD\r");
  tnc::TncTestAccess::relayResponses(relay, 0);
  assert(tnc::TncTestAccess::popRaw(relay, repeated));
  assert(std::string(reinterpret_cast<const char*>(repeated.info), repeated.infoLen).find("callsign") == 0);
  assert(tnc::TncTestAccess::popRaw(relay, repeated));
  assert(std::string(reinterpret_cast<const char*>(repeated.info), repeated.infoLen).find("N1PEER") == 0);
  assert(!tnc::TncTestAccess::mheardPending(relay, 0));
  for (unsigned i = 0; i < 20; ++i) {
    char call[8]{};
    snprintf(call, sizeof(call), "N%uABC", i);
    assert(ax25::parseAddress(call, frame.source));
    tnc::TncTestAccess::heard(relay, frame);
  }
  tnc::TncTestAccess::input(relay, 0, "//MH\r");
  tnc::TncTestAccess::relayResponses(relay, 0);
  assert(tnc::TncTestAccess::mheardPending(relay, 0));
  uint8_t acknowledged = 0;
  unsigned lines = 0;
  for (unsigned attempt = 0; attempt < 40; ++attempt) {
    tnc::TncTestAccess::acknowledge(relay, 0, acknowledged);
    tnc::TncTestAccess::relayResponses(relay, 0);
    while (tnc::TncTestAccess::popRaw(relay, repeated)) {
      if (ax25::kind(repeated.control) == ax25::FrameKind::I) ++lines;
    }
    if (tnc::TncTestAccess::relayDrained(relay, 0)) break;
  }
  assert(tnc::TncTestAccess::relayDrained(relay, 0));
  assert(lines == 21);

  assert(variant::batteryPercentageFromMillivolts(3000) == 0);
  assert(variant::batteryPercentageFromMillivolts(3750) == 50);
  assert(variant::batteryPercentageFromMillivolts(4200) == 100);
  assert(variant::batteryPercentageFromMillivolts(4500) == 100);

  radio::stats().txOk = 8;
  radio::stats().rxOk = 19;
  testMillis = 61UL * 60000;
  tnc::TncTestAccess::input(relay, 0, "//i\r");
  tnc::TncTestAccess::relayResponses(relay, 0);
  assert(tnc::TncTestAccess::popRaw(relay, repeated));
  assert(std::string(reinterpret_cast<const char*>(repeated.info), repeated.infoLen) ==
         "Battery: 3.975 V (75%)  TX: 8  RX: 19  Uptime: 01 hour and 01 minute\r");
  tnc::TncTestAccess::acknowledge(relay, 0, acknowledged);

  variant::testBatteryAvailable = false;
  testMillis = (23UL * 60 + 59) * 60000;
  tnc::TncTestAccess::input(relay, 0, "//I\r");
  tnc::TncTestAccess::relayResponses(relay, 0);
  assert(tnc::TncTestAccess::popRaw(relay, repeated));
  assert(std::string(reinterpret_cast<const char*>(repeated.info), repeated.infoLen) ==
         "Battery: unavailable  TX: 8  RX: 19  Uptime: 23 hours and 59 minutes\r");
  tnc::TncTestAccess::acknowledge(relay, 0, acknowledged);

  variant::testBatteryAvailable = true;
  testMillis = 24UL * 60 * 60000;
  tnc::TncTestAccess::input(relay, 0, "//I\r");
  tnc::TncTestAccess::relayResponses(relay, 0);
  assert(tnc::TncTestAccess::popRaw(relay, repeated));
  assert(std::string(reinterpret_cast<const char*>(repeated.info), repeated.infoLen) ==
         "Battery: 3.975 V (75%)  TX: 8  RX: 19  Uptime: 1 day and 00 hours\r");
  tnc::TncTestAccess::acknowledge(relay, 0, acknowledged);

  testMillis = 25UL * 60 * 60000;
  tnc::TncTestAccess::input(relay, 0, "//I\r");
  tnc::TncTestAccess::relayResponses(relay, 0);
  assert(tnc::TncTestAccess::popRaw(relay, repeated));
  assert(std::string(reinterpret_cast<const char*>(repeated.info), repeated.infoLen) ==
         "Battery: 3.975 V (75%)  TX: 8  RX: 19  Uptime: 1 day and 01 hour\r");
  tnc::TncTestAccess::acknowledge(relay, 0, acknowledged);

  testMillis = 0xFFFFFF00;
  tnc::TncTestAccess::updateRelayUptime(relay);
  testMillis = 3600000UL - 256;
  tnc::TncTestAccess::input(relay, 0, "//I\r");
  tnc::TncTestAccess::relayResponses(relay, 0);
  assert(tnc::TncTestAccess::popRaw(relay, repeated));
  assert(std::string(reinterpret_cast<const char*>(repeated.info), repeated.infoLen) ==
         "Battery: 3.975 V (75%)  TX: 8  RX: 19  Uptime: 49 days and 18 hours\r");
  tnc::TncTestAccess::acknowledge(relay, 0, acknowledged);

  while (tnc::TncTestAccess::popRaw(relay, repeated, true)) {}
  tnc::TncTestAccess::input(relay, 0, "//I\r");
  tnc::TncTestAccess::relayResponses(relay, 0);
  ax25::parseAddress("N3UNIQ", frame.source);
  ax25::parseAddress("N1DEST", frame.destination);
  ax25::parseAddress("N1DIG", frame.repeaters[0]);
  frame.repeaterCount = 1;
  frame.control = ax25::makeU(ax25::UFrameType::UI, false);
  assert(tnc::TncTestAccess::repeat(relay, frame));

  tnc::TncTestAccess::connectedInput(relay, 0, acknowledged, "//q\r");
  assert(tnc::TncTestAccess::state(relay, 0) == ax25::LinkState::Connected);
  tnc::TncTestAccess::relayResponses(relay, 0);
  assert(tnc::TncTestAccess::state(relay, 0) == ax25::LinkState::Disconnecting);
  assert(tnc::TncTestAccess::popRaw(relay, repeated, true));
  assert(ax25::uType(repeated.control) == ax25::UFrameType::DISC);
  assert(tnc::TncTestAccess::popRaw(relay, repeated));
  assert(ax25::addressEquals(repeated.source, frame.source));
  assert(!tnc::TncTestAccess::popRaw(relay, repeated));
}
#endif

int main() {
  modem.begin("N1LOC");
  tnc::TncTestAccess::hostmode(modem);
  testCommands();
  testMonitorAndPolling();
  testChannelsAndMaximumPayload();
  testIncomingConnectText();
  testReceiveBackpressure();
  testOversizedRadioFrames();
#if defined(AXLORA_VARIANT_SENSECAP_SOLAR)
  testSolarRelay();
#endif
  std::cout << "WA8DED command, monitor, polling and receive-backpressure tests passed\n";
}
