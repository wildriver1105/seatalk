// n2k_decoder.h
// SeaTalkng / NMEA2000 decoder — pure C++ (no Arduino dependency).
// Compiles both on the host (g++/clang for logic tests) and on the ESP32
// (included directly by the Arduino sketch).
#ifndef N2K_DECODER_H
#define N2K_DECODER_H

#include <cstdint>
#include <string>

namespace n2k {

// A raw CAN frame as received from the bus (ESP32 TWAI) or simulated on host.
struct CanFrame {
  uint32_t id;        // 29-bit extended CAN identifier
  uint8_t  len;       // data length (0-8)
  uint8_t  data[8];   // payload
};

// J1939 / NMEA2000 header decoded from the 29-bit CAN id.
struct Header {
  uint8_t  priority;  // 0 (highest) .. 7
  uint32_t pgn;       // Parameter Group Number
  uint8_t  src;       // source address
  uint8_t  dst;       // destination (PDU1) or 0xFF (PDU2 / broadcast)
};

Header   parseHeader(uint32_t canId);
// Build a 29-bit id from priority/pgn/source (used by tests and TX).
uint32_t buildId(uint8_t priority, uint32_t pgn, uint8_t src);

// ---- Typed decoders for the PGNs we handle -------------------------------
// Each carries `valid` (false when the field is the NMEA2000 "not available"
// sentinel or the frame is too short).

struct VesselHeading {       // PGN 127250
  double  headingDeg = 0;
  uint8_t reference  = 0;    // 0=True, 1=Magnetic
  bool    valid      = false;
};

struct SpeedWater {          // PGN 128259
  double speedKn = 0;        // speed through water
  bool   valid   = false;
};

struct WaterDepth {          // PGN 128267
  double depthM  = 0;        // depth below transducer
  double offsetM = 0;        // transducer offset (+keel / -waterline)
  bool   valid   = false;
};

struct WindData {            // PGN 130306
  double  speedKn  = 0;
  double  angleDeg = 0;      // 0..360
  uint8_t reference = 0;     // 0=True(ground) 1=Magnetic 2=Apparent 3=True(boat) 4=True(water)
  bool    valid    = false;
};

struct Position {            // PGN 129025
  double latDeg = 0;
  double lonDeg = 0;
  bool   valid  = false;
};

VesselHeading decodeVesselHeading(const CanFrame& f);
SpeedWater    decodeSpeedWater(const CanFrame& f);
WaterDepth    decodeWaterDepth(const CanFrame& f);
WindData      decodeWind(const CanFrame& f);
Position      decodePosition(const CanFrame& f);

// ---- Generic dispatch ----------------------------------------------------
struct Decoded {
  uint32_t    pgn  = 0;
  uint8_t     src  = 0;
  bool        known = false;   // PGN recognized and decoded
  std::string name;            // human readable PGN name
  std::string summary;         // human readable field summary
};

Decoded decode(const CanFrame& f);

// Convenience: a one-line "PGN ... src=... summary" string.
std::string formatLine(const CanFrame& f);

// One compact JSON object for a known/valid PGN (no trailing newline),
// or "" for unrecognized PGNs / "not available" fields. This is the exact
// wire format streamed over WiFi/TCP and by the host simulator.
std::string toJson(const CanFrame& f);

} // namespace n2k

#endif // N2K_DECODER_H
