// seatalk_gateway.ino
// ESP32 SeaTalkng / NMEA2000 gateway.
//   * Reads the CAN bus via the ESP32 built-in TWAI controller (250 kbit/s).
//   * Decodes PGNs with the shared, Arduino-independent decoder (n2k_decoder).
//   * Streams decoded values as newline-delimited JSON over BOTH:
//       - USB serial  (wired path; Serial Monitor @115200)
//       - WiFi TCP server on port 2000 (wireless path; connect from your PC)
//
// The JSON wire format is identical to the host simulator (sim/sim_server),
// so PC software you built against the simulator works unchanged here.
//
// Wiring (ESP32 <-> SN65HVD230 / TJA1050 transceiver):
//   CAN_TX_GPIO -> transceiver TX (D / CTX)
//   CAN_RX_GPIO -> transceiver RX (R / CRX)
//   CANH/CANL   -> SeaTalkng / NMEA2000 backbone (drop cable)
//   3V3 + GND   -> transceiver power (prefer a 3.3V part, e.g. SN65HVD230)
//
// Board: "ESP32 Dev Module" (or your variant). Open this folder in Arduino IDE.

#include <WiFi.h>
#include "driver/twai.h"
#include "n2k_decoder.h"

// ===== user config =========================================================
static const char* WIFI_SSID = "YOUR_WIFI_SSID";
static const char* WIFI_PASS = "YOUR_WIFI_PASSWORD";
static const uint16_t TCP_PORT = 2000;

// adjust to your wiring
#define CAN_TX_GPIO GPIO_NUM_5
#define CAN_RX_GPIO GPIO_NUM_4

// Listen-only = pure sniffer, never disturbs the boat's bus. Switch to
// TWAI_MODE_NORMAL when you add transmit (autopilot control — see TODO below).
#define TWAI_MODE   TWAI_MODE_LISTEN_ONLY
// ===========================================================================

static const int MAX_CLIENTS = 4;
WiFiServer  tcpServer(TCP_PORT);
WiFiClient  clients[MAX_CLIENTS];

static void wifiConnect() {
  Serial.printf("WiFi: connecting to \"%s\" ...\n", WIFI_SSID);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  uint32_t start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 20000) {
    delay(300);
    Serial.print(".");
  }
  Serial.println();
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("WiFi: connected. IP = ");
    Serial.println(WiFi.localIP());
    Serial.printf("TCP: connect your PC to %s:%u\n",
                  WiFi.localIP().toString().c_str(), TCP_PORT);
  } else {
    Serial.println("WiFi: FAILED (continuing on USB serial only).");
  }
}

static void acceptClients() {
  WiFiClient incoming = tcpServer.available();
  if (!incoming) return;
  for (int i = 0; i < MAX_CLIENTS; i++) {
    if (!clients[i] || !clients[i].connected()) {
      clients[i] = incoming;
      Serial.printf("TCP: client %d connected\n", i);
      return;
    }
  }
  incoming.stop();  // table full
}

static void broadcast(const String& line) {
  for (int i = 0; i < MAX_CLIENTS; i++) {
    if (clients[i] && clients[i].connected()) clients[i].print(line);
  }
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\nSeaTalkng/NMEA2000 gateway starting...");

  wifiConnect();
  tcpServer.begin();
  tcpServer.setNoDelay(true);

  twai_general_config_t g =
      TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_GPIO, CAN_RX_GPIO, TWAI_MODE);
  twai_timing_config_t t = TWAI_TIMING_CONFIG_250KBITS();
  twai_filter_config_t f = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  if (twai_driver_install(&g, &t, &f) != ESP_OK) {
    Serial.println("ERROR: twai_driver_install failed");
    while (true) delay(1000);
  }
  if (twai_start() != ESP_OK) {
    Serial.println("ERROR: twai_start failed");
    while (true) delay(1000);
  }
  Serial.println("TWAI started @250kbit/s. Waiting for frames...");
}

void loop() {
  acceptClients();

  twai_message_t msg;
  if (twai_receive(&msg, pdMS_TO_TICKS(50)) != ESP_OK) return;
  if (!msg.extd) return;  // NMEA2000 uses 29-bit extended ids only

  n2k::CanFrame fr;
  fr.id  = msg.identifier;
  fr.len = msg.data_length_code > 8 ? 8 : msg.data_length_code;
  for (uint8_t i = 0; i < fr.len; i++) fr.data[i] = msg.data[i];

  std::string json = n2k::toJson(fr);
  if (json.empty()) return;        // unhandled PGN
  String line = String(json.c_str()) + "\n";

  Serial.print(line);              // wired path
  broadcast(line);                 // wireless path
}

// ----------------------------------------------------------------------------
// TODO (mid/long term): bidirectional control (autopilot, etc.)
//   1. Set TWAI_MODE -> TWAI_MODE_NORMAL so the ESP32 can transmit & ACK.
//   2. Accept commands from a TCP client (read clients[i].available()), parse
//      JSON, map to the proper PGN, and twai_transmit() the frame.
//   3. Add an encoder side to n2k_decoder (n2k::encodeXxx -> CanFrame) so the
//      host simulator can round-trip and validate TX before touching the boat.
//   Reference: the open-source autopilot project you downloaded for PGN specs.
// ----------------------------------------------------------------------------
