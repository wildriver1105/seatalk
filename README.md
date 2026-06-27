# seatalk

ESP32 기반 **SeaTalkng / NMEA2000** 게이트웨이. CAN 버스(250 kbit/s, 29비트 확장 ID)에서
PGN 메시지를 수신해 사람이 읽을 수 있는 값으로 디코딩한다.

설계 원칙: **디코더 로직은 Arduino에 종속되지 않은 순수 C++** (`n2k_decoder.*`)로 작성한다.
같은 소스를
- Mac에서 g++/clang으로 컴파일해 시뮬레이션 프레임으로 검증하고 (`test/`),
- ESP32 스케치가 그대로 `#include` 해서 실제 TWAI 수신에 쓴다 (`seatalk_gateway/`).

코드 중복이 없으므로, 호스트에서 통과한 로직이 곧 펌웨어 로직이다.

## 데이터 흐름
```
CAN 버스 ──> ESP32 TWAI ──> n2k_decoder ──> JSON ──┬─> USB 시리얼 (유선)
                                                   └─> WiFi TCP :2000 (무선) ──> PC
```
유선(USB)과 무선(WiFi)은 동시에 출력된다. 별도 배선을 뺄 필요 없이, USB 케이블이 유선
경로이고 WiFi가 무선 경로다.

JSON 형식(줄 단위, 예):
```json
{"pgn":130306,"src":35,"type":"wind","speed_kn":15.55,"angle_deg":42.1,"ref":"Apparent"}
{"pgn":128267,"src":35,"type":"depth","depth_m":11.34,"offset_m":0.30}
```
**ESP32와 호스트 시뮬레이터의 JSON 형식은 완전히 동일하다.** 시뮬레이터로 개발한 PC
소프트웨어는 나중에 IP만 ESP32로 바꾸면 그대로 동작한다.

## 구조
```
seatalk/
├── seatalk_gateway/          # Arduino IDE에서 이 폴더를 연다
│   ├── seatalk_gateway.ino   # ESP32 TWAI 리스너 + WiFi TCP 서버
│   ├── n2k_decoder.h         # 디코더 인터페이스 (호스트/ESP32 공용)
│   └── n2k_decoder.cpp       # 디코더 + JSON 출력 (Arduino 비종속)
├── sim/
│   ├── sim_server.cpp        # 호스트 시뮬레이터: ESP32와 동일한 JSON을 TCP로 송출
│   └── Makefile
├── tools/
│   └── n2k_client.py         # PC측 수신 클라이언트 (개발 출발점)
└── test/
    ├── test_decoder.cpp      # 시뮬레이션 프레임 검증 하니스
    └── Makefile
```

## 1) 디코더 로직 검증 (실제 버스 불필요)
```
cd test && make
```
헤더 파싱(buildId/parseHeader)과 각 PGN 디코딩을 시뮬레이션 프레임으로 검증. 현재 19개 통과.

## 2) WiFi 스트림 경로를 지금 검증 (ESP32 불필요)
호스트 시뮬레이터가 ESP32와 동일한 JSON을 TCP로 내보낸다. PC 소프트웨어를 미리 개발·테스트:
```
# 터미널 A
cd sim && make && ./sim_server 2000

# 터미널 B — 원하는 클라이언트로 접속
nc localhost 2000
python3 tools/n2k_client.py localhost 2000        # 라이브 계기판
```

## 3) ESP32에 올려 실제 버스 수신
1. `seatalk_gateway/` 폴더를 Arduino IDE에서 연다.
2. Board: **ESP32 Dev Module**(또는 사용하는 변형) 선택.
3. `seatalk_gateway.ino` 상단의 설정을 채운다:
   ```c
   static const char* WIFI_SSID = "YOUR_WIFI_SSID";
   static const char* WIFI_PASS = "YOUR_WIFI_PASSWORD";
   #define CAN_TX_GPIO GPIO_NUM_5   // 배선에 맞게
   #define CAN_RX_GPIO GPIO_NUM_4
   ```
4. 업로드 후 Serial Monitor를 **115200 baud**로 연다. 부팅 시 WiFi IP가 출력된다.
5. PC에서 **이름으로** 접속 (mDNS, macOS는 Bonjour로 기본 지원):
   ```
   python3 tools/n2k_client.py seatalk.local 2000
   ```
   IP를 직접 써도 된다: `python3 tools/n2k_client.py <ESP32_IP> 2000`
   (OpenCPN/Signal K도 TCP, host=`seatalk.local` 또는 IP, port=2000으로 연결.)

### WiFi 모드: STA (권장)
ESP32가 **스타링크 WiFi에 접속(STA)**한다. 컴퓨터는 스타링크에 그대로 붙어 있어 **인터넷을
유지**하면서 같은 LAN의 ESP32로 데이터를 받는다. ESP32를 핫스팟(AP)으로 만들면 컴퓨터가
ESP32 WiFi로 갈아타야 해서 인터넷이 끊기므로, 인터넷이 필요하면 STA가 맞다.

### 배선 (ESP32 ↔ SN65HVD230 / TJA1050)
| ESP32        | 트랜시버         |
|--------------|------------------|
| CAN_TX_GPIO  | TX (D / CTX)     |
| CAN_RX_GPIO  | RX (R / CRX)     |
| 3V3, GND     | VCC, GND         |
| —            | CANH/CANL → 백본 |

> 3.3V 트랜시버(SN65HVD230) 권장. TJA1050은 5V 부품이라 RX 라인 레벨에 주의.
> 종단 저항(120Ω)은 보통 백본 양 끝에 이미 있으므로 드롭 케이블에는 추가하지 않는다.

펌웨어는 **LISTEN_ONLY** 모드라 버스를 방해하지 않고 스니핑만 한다. 송신이 필요해지면
`.ino`의 `TWAI_MODE_LISTEN_ONLY`를 `TWAI_MODE_NORMAL`로 바꾼다.

## 현재 디코딩하는 PGN
| PGN     | 이름                       | 출력           |
|---------|----------------------------|----------------|
| 127250  | Vessel Heading             | heading, 기준  |
| 128259  | Speed (Water Referenced)   | STW (kn)       |
| 128267  | Water Depth                | depth, offset  |
| 130306  | Wind Data                  | 풍속/풍향/기준 |
| 129025  | Position, Rapid Update     | lat/lon        |

모두 단일 프레임 PGN이다.

## 다음 단계 (TODO)
- **Fast-Packet 재조립**: GNSS(129029), AIS, 제품정보(126996) 등 8바이트 초과 PGN.
- PGN 추가: 127245 Rudder, 127257 Attitude, 130310/130311 Environmental 등.
- 출력 포맷: 사람이 읽는 형식 외에 NMEA0183/JSON 출력 옵션.
- 실제 트랜시버로 루프백/버스 테스트 후 핀·타이밍 확정.
