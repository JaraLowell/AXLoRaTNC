#include "native/Arduino.h"
#include "native/Preferences.h"
#include <cassert>
#include <iostream>
#include "tnc/tnc.h"
#include "tnc/ded_protocol.h"

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
};
}

namespace axlora::util {
bool logEnabled() { return false; }
void setLogEnabled(bool) {}
}

namespace axlora::radio {
class TestRadio : public Driver {
 public:
  std::deque<RxPacket> packets;
  Result init() override { return Result::Ok; }
  Result send(const uint8_t*, size_t) override { return Result::Ok; }
  Result receive(RxPacket& packet) override {
    if (packets.empty()) return Result::NoPacket;
    packet = packets.front(); packets.pop_front(); return Result::Ok;
  }
  Result setFrequency(float) override { return Result::Ok; }
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

int main() {
  modem.begin("N1LOC");
  tnc::TncTestAccess::hostmode(modem);
  testCommands();
  testMonitorAndPolling();
  testChannelsAndMaximumPayload();
  testIncomingConnectText();
  testReceiveBackpressure();
  std::cout << "WA8DED command, monitor, polling and receive-backpressure tests passed\n";
}
