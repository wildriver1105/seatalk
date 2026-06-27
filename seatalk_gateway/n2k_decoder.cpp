// n2k_decoder.cpp — SeaTalkng / NMEA2000 decoding logic (Arduino-independent).
#include "n2k_decoder.h"
#include <cstdio>

namespace n2k {

// ---- little-endian field helpers ----------------------------------------
static inline uint16_t le16(const uint8_t* p)  { return (uint16_t)(p[0] | (p[1] << 8)); }
static inline uint32_t le32(const uint8_t* p)  {
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}
static inline int16_t  sle16(const uint8_t* p) { return (int16_t)le16(p); }
static inline int32_t  sle32(const uint8_t* p) { return (int32_t)le32(p); }

// NMEA2000 "data not available" sentinels.
static const uint16_t NA_U16 = 0xFFFF;
static const uint32_t NA_U32 = 0xFFFFFFFFu;
static const int16_t  NA_S16 = (int16_t)0x7FFF;
static const int32_t  NA_S32 = (int32_t)0x7FFFFFFF;

static const double RAD2DEG = 57.295779513082323;
static const double MS2KN   = 1.9438444924406046;

// ---- header --------------------------------------------------------------
Header parseHeader(uint32_t id) {
  Header h;
  h.priority = (uint8_t)((id >> 26) & 0x07);
  uint8_t pf = (uint8_t)((id >> 16) & 0xFF);   // PDU format
  uint8_t ps = (uint8_t)((id >> 8)  & 0xFF);   // PDU specific
  uint8_t dp = (uint8_t)((id >> 24) & 0x03);   // data page + extended data page
  h.src = (uint8_t)(id & 0xFF);
  if (pf < 240) {                 // PDU1 — destination specific
    h.pgn = ((uint32_t)dp << 16) | ((uint32_t)pf << 8);
    h.dst = ps;
  } else {                        // PDU2 — broadcast
    h.pgn = ((uint32_t)dp << 16) | ((uint32_t)pf << 8) | ps;
    h.dst = 0xFF;
  }
  return h;
}

uint32_t buildId(uint8_t priority, uint32_t pgn, uint8_t src) {
  uint8_t pf = (uint8_t)((pgn >> 8) & 0xFF);
  uint32_t id;
  if (pf < 240) {                 // PDU1: low byte of PGN is 0, PS = dest (0xFF)
    id = ((uint32_t)(priority & 7) << 26) | ((pgn & 0x3FF00u) << 8) | (0xFFu << 8) | src;
  } else {                        // PDU2
    id = ((uint32_t)(priority & 7) << 26) | ((pgn & 0x3FFFFu) << 8) | src;
  }
  return id;
}

// ---- typed decoders ------------------------------------------------------
VesselHeading decodeVesselHeading(const CanFrame& f) {
  VesselHeading r;
  if (f.len < 7) return r;
  uint16_t raw = le16(f.data + 1);
  if (raw == NA_U16) return r;
  double deg = raw * 0.0001 * RAD2DEG;
  if (deg < 0) deg += 360.0;
  r.headingDeg = deg;
  r.reference  = (uint8_t)(f.data[7] & 0x03);
  r.valid = true;
  return r;
}

SpeedWater decodeSpeedWater(const CanFrame& f) {
  SpeedWater r;
  if (f.len < 3) return r;
  uint16_t raw = le16(f.data + 1);
  if (raw == NA_U16) return r;
  r.speedKn = raw * 0.01 * MS2KN;     // 0.01 m/s units
  r.valid = true;
  return r;
}

WaterDepth decodeWaterDepth(const CanFrame& f) {
  WaterDepth r;
  if (f.len < 7) return r;
  uint32_t raw = le32(f.data + 1);
  if (raw == NA_U32) return r;
  r.depthM = raw * 0.01;              // 0.01 m units
  int16_t off = sle16(f.data + 5);
  r.offsetM = (off == NA_S16) ? 0.0 : off * 0.001;  // 0.001 m units
  r.valid = true;
  return r;
}

WindData decodeWind(const CanFrame& f) {
  WindData r;
  if (f.len < 6) return r;
  uint16_t spd = le16(f.data + 1);
  uint16_t ang = le16(f.data + 3);
  if (spd == NA_U16 || ang == NA_U16) return r;
  r.speedKn  = spd * 0.01 * MS2KN;    // 0.01 m/s
  double deg = ang * 0.0001 * RAD2DEG; // 0.0001 rad
  if (deg < 0) deg += 360.0;
  r.angleDeg = deg;
  r.reference = (uint8_t)(f.data[5] & 0x07);
  r.valid = true;
  return r;
}

Position decodePosition(const CanFrame& f) {
  Position r;
  if (f.len < 8) return r;
  int32_t lat = sle32(f.data + 0);
  int32_t lon = sle32(f.data + 4);
  if (lat == NA_S32 || lon == NA_S32) return r;
  r.latDeg = lat * 1e-7;
  r.lonDeg = lon * 1e-7;
  r.valid = true;
  return r;
}

// ---- dispatch + formatting ----------------------------------------------
static const char* windRefStr(uint8_t r) {
  switch (r) {
    case 0: return "True(ground)";
    case 1: return "Magnetic";
    case 2: return "Apparent";
    case 3: return "True(boat)";
    case 4: return "True(water)";
    default: return "?";
  }
}

Decoded decode(const CanFrame& f) {
  Header h = parseHeader(f.id);
  Decoded d;
  d.pgn = h.pgn;
  d.src = h.src;
  char buf[160];

  switch (h.pgn) {
    case 127250: {
      d.name = "Vessel Heading";
      VesselHeading v = decodeVesselHeading(f);
      if (v.valid) {
        snprintf(buf, sizeof(buf), "heading=%.1f deg (%s)",
                 v.headingDeg, v.reference == 0 ? "True" : "Magnetic");
        d.summary = buf; d.known = true;
      }
      break;
    }
    case 128259: {
      d.name = "Speed (Water Referenced)";
      SpeedWater v = decodeSpeedWater(f);
      if (v.valid) {
        snprintf(buf, sizeof(buf), "STW=%.2f kn", v.speedKn);
        d.summary = buf; d.known = true;
      }
      break;
    }
    case 128267: {
      d.name = "Water Depth";
      WaterDepth v = decodeWaterDepth(f);
      if (v.valid) {
        snprintf(buf, sizeof(buf), "depth=%.2f m (offset %.2f m)", v.depthM, v.offsetM);
        d.summary = buf; d.known = true;
      }
      break;
    }
    case 130306: {
      d.name = "Wind Data";
      WindData v = decodeWind(f);
      if (v.valid) {
        snprintf(buf, sizeof(buf), "wind=%.2f kn @ %.1f deg [%s]",
                 v.speedKn, v.angleDeg, windRefStr(v.reference));
        d.summary = buf; d.known = true;
      }
      break;
    }
    case 129025: {
      d.name = "Position, Rapid Update";
      Position v = decodePosition(f);
      if (v.valid) {
        snprintf(buf, sizeof(buf), "lat=%.6f lon=%.6f", v.latDeg, v.lonDeg);
        d.summary = buf; d.known = true;
      }
      break;
    }
    default:
      d.name = "Unhandled";
      break;
  }
  return d;
}

std::string toJson(const CanFrame& f) {
  Header h = parseHeader(f.id);
  char buf[256];
  switch (h.pgn) {
    case 127250: {
      VesselHeading v = decodeVesselHeading(f);
      if (!v.valid) break;
      snprintf(buf, sizeof(buf),
        "{\"pgn\":127250,\"src\":%u,\"type\":\"heading\","
        "\"heading_deg\":%.1f,\"ref\":\"%s\"}",
        h.src, v.headingDeg, v.reference == 0 ? "True" : "Magnetic");
      return buf;
    }
    case 128259: {
      SpeedWater v = decodeSpeedWater(f);
      if (!v.valid) break;
      snprintf(buf, sizeof(buf),
        "{\"pgn\":128259,\"src\":%u,\"type\":\"stw\",\"stw_kn\":%.2f}",
        h.src, v.speedKn);
      return buf;
    }
    case 128267: {
      WaterDepth v = decodeWaterDepth(f);
      if (!v.valid) break;
      snprintf(buf, sizeof(buf),
        "{\"pgn\":128267,\"src\":%u,\"type\":\"depth\","
        "\"depth_m\":%.2f,\"offset_m\":%.2f}",
        h.src, v.depthM, v.offsetM);
      return buf;
    }
    case 130306: {
      WindData v = decodeWind(f);
      if (!v.valid) break;
      snprintf(buf, sizeof(buf),
        "{\"pgn\":130306,\"src\":%u,\"type\":\"wind\","
        "\"speed_kn\":%.2f,\"angle_deg\":%.1f,\"ref\":\"%s\"}",
        h.src, v.speedKn, v.angleDeg, windRefStr(v.reference));
      return buf;
    }
    case 129025: {
      Position v = decodePosition(f);
      if (!v.valid) break;
      snprintf(buf, sizeof(buf),
        "{\"pgn\":129025,\"src\":%u,\"type\":\"position\","
        "\"lat\":%.6f,\"lon\":%.6f}",
        h.src, v.latDeg, v.lonDeg);
      return buf;
    }
    default: break;
  }
  return std::string();
}

std::string formatLine(const CanFrame& f) {
  Decoded d = decode(f);
  char buf[200];
  if (d.known) {
    snprintf(buf, sizeof(buf), "PGN %6u src=%-3u %-26s | %s",
             d.pgn, d.src, d.name.c_str(), d.summary.c_str());
  } else {
    snprintf(buf, sizeof(buf), "PGN %6u src=%-3u %-26s |",
             d.pgn, d.src, d.name.c_str());
  }
  return std::string(buf);
}

} // namespace n2k
