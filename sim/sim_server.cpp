// sim_server.cpp
// Host-side SeaTalkng/NMEA2000 *simulator*. Streams the exact same JSON lines
// the ESP32 firmware emits over WiFi/TCP, but generated from simulated frames
// run through the shared decoder. Lets you build & test the computer-side
// software now, with no ESP32 and no real bus. Later, point your client at the
// ESP32's IP instead of this — the wire format is identical.
//
//   make            # build
//   ./sim_server     # listen on 0.0.0.0:2000, stream ~2 msgs/sec per client
//   ./sim_server 9000
#include "n2k_decoder.h"
#include <cstdio>
#include <cstring>
#include <cstdint>
#include <string>
#include <vector>
#include <ctime>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <signal.h>

using namespace n2k;

static void w16(uint8_t* p, uint16_t v) { p[0] = v & 0xFF; p[1] = (v >> 8) & 0xFF; }
static void w32(uint8_t* p, uint32_t v) {
  p[0] = v & 0xFF; p[1] = (v >> 8) & 0xFF; p[2] = (v >> 16) & 0xFF; p[3] = (v >> 24) & 0xFF;
}
static CanFrame base(uint8_t prio, uint32_t pgn, uint8_t src) {
  CanFrame f{}; f.id = buildId(prio, pgn, src); f.len = 8;
  memset(f.data, 0xFF, sizeof(f.data)); return f;
}

// Build one cycle of frames; `t` drives gentle variation so the stream looks live.
static std::vector<CanFrame> cycle(int t) {
  std::vector<CanFrame> v;
  double wob = (t % 20) - 10;                 // -10..+9 wobble

  CanFrame wind = base(2, 130306, 35);
  wind.data[0] = 0x10;
  w16(wind.data + 1, (uint16_t)(1000 + wob * 20));        // ~10 m/s
  w16(wind.data + 3, (uint16_t)(7854 + wob * 50));        // ~45 deg
  wind.data[5] = 2;                                       // Apparent
  v.push_back(wind);

  CanFrame depth = base(3, 128267, 35);
  depth.data[0] = 0x10;
  w32(depth.data + 1, (uint32_t)(1234 + wob * 10));       // ~12.3 m
  w16(depth.data + 5, 300);                               // +0.3 m offset
  v.push_back(depth);

  CanFrame stw = base(2, 128259, 35);
  stw.data[0] = 0x10;
  w16(stw.data + 1, (uint16_t)(617 + wob));               // ~6.2 m/s
  v.push_back(stw);

  CanFrame hdg = base(2, 127250, 35);
  hdg.data[0] = 0x10;
  w16(hdg.data + 1, (uint16_t)(15708 + wob * 30));        // ~90 deg
  hdg.data[7] = 0x00;
  v.push_back(hdg);

  CanFrame pos = base(2, 129025, 35);
  w32(pos.data + 0, (uint32_t)(int32_t)(37.5665 * 1e7));
  w32(pos.data + 4, (uint32_t)(int32_t)(126.9780 * 1e7));
  v.push_back(pos);

  return v;
}

int main(int argc, char** argv) {
  signal(SIGPIPE, SIG_IGN);                 // don't die when a client disconnects
  int port = (argc > 1) ? atoi(argv[1]) : 2000;

  int srv = socket(AF_INET, SOCK_STREAM, 0);
  int yes = 1;
  setsockopt(srv, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));
  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = INADDR_ANY;
  addr.sin_port = htons(port);
  if (bind(srv, (sockaddr*)&addr, sizeof(addr)) < 0) { perror("bind"); return 1; }
  if (listen(srv, 4) < 0) { perror("listen"); return 1; }

  printf("[sim] SeaTalkng/NMEA2000 simulator listening on 0.0.0.0:%d\n", port);
  printf("[sim] connect e.g.:  nc localhost %d   |   python3 ../tools/n2k_client.py localhost %d\n",
         port, port);

  for (;;) {
    sockaddr_in cli{}; socklen_t clen = sizeof(cli);
    int fd = accept(srv, (sockaddr*)&cli, &clen);
    if (fd < 0) continue;
    printf("[sim] client connected: %s\n", inet_ntoa(cli.sin_addr));

    int t = 0;
    bool alive = true;
    while (alive) {
      for (const auto& f : cycle(t)) {
        std::string line = toJson(f);
        if (line.empty()) continue;
        line += "\n";
        if (write(fd, line.c_str(), line.size()) < 0) { alive = false; break; }
        usleep(100 * 1000);                 // 100ms between messages
      }
      t++;
    }
    close(fd);
    printf("[sim] client disconnected\n");
  }
  return 0;
}
