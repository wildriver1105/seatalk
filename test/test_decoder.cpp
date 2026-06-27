// test_decoder.cpp
// Host-side logic verification for the SeaTalkng/NMEA2000 decoder.
// Builds simulated CAN frames with known values, runs them through the same
// decoder the ESP32 firmware uses, and asserts the decoded numbers.
//
// Build & run:  make test   (from the test/ directory)
#include "n2k_decoder.h"
#include <cstdio>
#include <cmath>
#include <cstring>
#include <vector>

using namespace n2k;

static int g_pass = 0, g_fail = 0;

static void check(const char* what, bool ok) {
  printf("  [%s] %s\n", ok ? "PASS" : "FAIL", what);
  if (ok) g_pass++; else g_fail++;
}
static bool approx(double a, double b, double eps) { return std::fabs(a - b) <= eps; }

// little-endian writers for building payloads
static void w16(uint8_t* p, uint16_t v) { p[0] = v & 0xFF; p[1] = (v >> 8) & 0xFF; }
static void w32(uint8_t* p, uint32_t v) {
  p[0] = v & 0xFF; p[1] = (v >> 8) & 0xFF; p[2] = (v >> 16) & 0xFF; p[3] = (v >> 24) & 0xFF;
}

static CanFrame makeFrame(uint8_t prio, uint32_t pgn, uint8_t src) {
  CanFrame f{};
  f.id = buildId(prio, pgn, src);
  f.len = 8;
  memset(f.data, 0xFF, sizeof(f.data)); // default to "not available"
  return f;
}

int main() {
  std::vector<CanFrame> bus;

  printf("== Header round-trip (buildId / parseHeader) ==\n");
  {
    uint32_t id = buildId(2, 130306, 35);
    Header h = parseHeader(id);
    check("pgn  == 130306", h.pgn == 130306);
    check("src  == 35",     h.src == 35);
    check("prio == 2",      h.priority == 2);
  }

  printf("\n== PGN 130306 Wind Data ==\n");
  {
    CanFrame f = makeFrame(2, 130306, 35);
    f.data[0] = 0x10;                 // SID
    w16(f.data + 1, 1000);            // 10.00 m/s  -> 19.44 kn
    w16(f.data + 3, 7854);            // 0.7854 rad -> ~45.0 deg
    f.data[5] = 2;                    // Apparent
    WindData w = decodeWind(f);
    check("valid",                w.valid);
    check("speed ~ 19.44 kn",     approx(w.speedKn, 19.438, 0.01));
    check("angle ~ 45.0 deg",     approx(w.angleDeg, 45.0, 0.1));
    check("ref == Apparent(2)",   w.reference == 2);
    bus.push_back(f);
  }

  printf("\n== PGN 128267 Water Depth ==\n");
  {
    CanFrame f = makeFrame(3, 128267, 35);
    f.data[0] = 0x10;                 // SID
    w32(f.data + 1, 1234);            // 12.34 m
    w16(f.data + 5, 300);             // +0.300 m offset
    f.data[7] = 0xFF;
    WaterDepth d = decodeWaterDepth(f);
    check("valid",              d.valid);
    check("depth  ~ 12.34 m",   approx(d.depthM, 12.34, 0.001));
    check("offset ~ 0.30 m",    approx(d.offsetM, 0.30, 0.001));
    bus.push_back(f);
  }

  printf("\n== PGN 128259 Speed (Water Referenced) ==\n");
  {
    CanFrame f = makeFrame(2, 128259, 35);
    f.data[0] = 0x10;
    w16(f.data + 1, 617);             // 6.17 m/s -> ~11.99 kn
    SpeedWater s = decodeSpeedWater(f);
    check("valid",            s.valid);
    check("STW ~ 11.99 kn",   approx(s.speedKn, 11.99, 0.02));
    bus.push_back(f);
  }

  printf("\n== PGN 127250 Vessel Heading ==\n");
  {
    CanFrame f = makeFrame(2, 127250, 35);
    f.data[0] = 0x10;
    w16(f.data + 1, 15708);           // 1.5708 rad -> ~90.0 deg
    f.data[7] = 0x00;                 // True
    VesselHeading h = decodeVesselHeading(f);
    check("valid",            h.valid);
    check("heading ~ 90 deg", approx(h.headingDeg, 90.0, 0.1));
    check("ref == True(0)",   h.reference == 0);
    bus.push_back(f);
  }

  printf("\n== PGN 129025 Position, Rapid Update ==\n");
  {
    CanFrame f = makeFrame(2, 129025, 35);
    w32(f.data + 0, (uint32_t)(int32_t)(37.5665 * 1e7));  // Seoul-ish lat
    w32(f.data + 4, (uint32_t)(int32_t)(126.9780 * 1e7)); // lon
    Position p = decodePosition(f);
    check("valid",          p.valid);
    check("lat ~ 37.5665",  approx(p.latDeg, 37.5665, 1e-4));
    check("lon ~ 126.978",  approx(p.lonDeg, 126.978, 1e-4));
    bus.push_back(f);
  }

  printf("\n== 'Not available' sentinel handling ==\n");
  {
    CanFrame f = makeFrame(2, 130306, 35); // payload all 0xFF
    WindData w = decodeWind(f);
    check("wind invalid when 0xFFFF", !w.valid);
  }

  printf("\n== Simulated bus replay (formatLine) ==\n");
  for (const auto& f : bus) {
    printf("  %s\n", formatLine(f).c_str());
  }

  printf("\n========================================\n");
  printf("RESULT: %d passed, %d failed\n", g_pass, g_fail);
  return g_fail == 0 ? 0 : 1;
}
