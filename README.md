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

## 프로젝트 현황

### ✅ 구현 + 검증 완료
- **공용 디코더 `n2k_decoder`** — Arduino 비종속 순수 C++. PGN 5종 디코딩 + JSON 출력.
- **호스트 테스트 `test/`** — 시뮬레이션 프레임으로 헤더 파싱 + 디코딩 검증, **19개 전부 통과**.
- **호스트 시뮬레이터 `sim/sim_server`** — ESP32와 동일한 JSON을 TCP로 송출. Mac에서 동작 확인.
- **PC 클라이언트 `tools/n2k_client.py`** — 라이브 계기판(HDG/STW/WIND/DEPTH/POS). Mac에서 확인.
- **ESP32 펌웨어 `seatalk_gateway.ino`**
  - TWAI(CAN) 250 kbit/s **listen-only** 수신
  - WiFi **STA** 접속 + **mDNS**(`seatalk.local`) 등록
  - **TCP 서버 :2000** (다중 클라이언트), **USB 시리얼 + WiFi 동시 출력**
  - `DEBUG_RAW` — 미디코딩 PGN 포함 모든 프레임 hex 덤프 (배선 점검용)
- **secrets.h 분리** — WiFi 비밀번호를 gitignore된 파일로 분리(GitHub 안전).
- **실기(實機) 검증** — ESP32 업로드 성공, WiFi 접속(IP `192.168.1.93`),
  `seatalk.local` 이름해석 + TCP 2000 연결 **Mac에서 성공**. 무선 데이터 경로 확보.

### 🔲 준비해야 하는 것 (다음 단계)
- **케이블 도착 대기** — **M12 5핀 NMEA2000 케이블 + A06045 어댑터 + 데이터용 USB-C 케이블**.
  (결선 설계는 [데이터 허브 연결](#데이터-허브nmea2000-백본-연결--사용-케이블) 섹션 참고.)
- **NMEA2000 버스 물리 연결** — 아직 미연결. 데이터를 받으려면 이게 마지막 관문:
  - 백본 → A06045 → M12 케이블 → 트랜시버. 흰=CAN-H, 파랑=CAN-L, 검정=GND.
  - **전원은 테스트 단계 USB-C(5V)만**, M12 **V+(빨강, 12V)는 연결 금지** ⚠️ (전원 충돌)
  - 트랜시버 전원 **3.3V**, CAN 핀 GPIO4(RX)/GPIO5(TX) 확인
  - **데이터가 안 나오면 TX↔RX 스왑부터** (모듈 라벨 관례 차이, 1순위 원인)
  - 종단저항 120Ω은 백본 양 끝에 이미 있어야 함(기존 배 네트워크면 보통 존재)
- **버스 연결 후 RAW 덤프 확인** → 실제로 송신 중인 PGN 파악 → 필요한 PGN 디코더 추가.
- **영구 설치(나중)** — USB-C 대신 M12 **V+(12V) → 벅 컨버터 → ESP32 5V**, USB는 분리.

### 🗺️ 로드맵 (소프트웨어)
- **Fast-Packet 재조립** — 8바이트 초과 PGN(129029 GNSS, AIS, 126996 제품정보 등).
- **PGN 추가** — 127245 Rudder, 127257 Attitude, 127251 ROT, 130310/130311 Environmental 등.
- **출력 연동** — NMEA0183 / **Signal K** 출력 옵션 (OpenCPN·Signal K 생태계 연결).
- **로깅/표시** — 클라이언트 `--log` 파일 저장, 계기 화면 앱.
- **중장기: 양방향 제어(오토파일럿)** — `TWAI_MODE_NORMAL` 전환 + TCP 명령 수신 +
  인코더(`encodeXxx → CanFrame`). 실선 송신 전 **시뮬레이터로 TX round-trip 검증**.

## 설계도 (아키텍처)

### 계층 구조 — 단일 디코더, 호스트와 펌웨어가 공유
```
            ┌──────────────────────────────────────────────┐
            │   n2k_decoder.{h,cpp}  (순수 C++, Arduino 무관)  │
            │   parseHeader / decode* / toJson / formatLine  │
            └──────────────┬─────────────────┬───────────────┘
                           │ 같은 소스 공유      │
              ┌────────────┴──────┐   ┌────────┴─────────────┐
              │  HOST (Mac, g++)   │   │  ESP32 (Arduino)      │
              │  test_decoder      │   │  seatalk_gateway.ino  │
              │  sim_server        │   │  TWAI + WiFi + TCP     │
              └───────────────────┘   └──────────────────────┘
   → 호스트에서 통과한 디코딩 로직 = 펌웨어 로직 (코드 중복 0)
```

### 런타임 데이터 흐름 (실제 동작 시)
```
 [계기들]              [게이트웨이: ESP32-WROOM]                 [PC / 앱]
 풍향계·측심기  ─NMEA2000─▶ SN65HVD230 ─▶ TWAI ─▶ n2k_decoder ─▶ JSON
 GPS·풍속계      (CAN 250k)  (트랜시버)   (CAN수신)  (디코딩)        │
                                                                 ├▶ USB 시리얼 (유선)
                                                                 └▶ WiFi TCP :2000
                                                                    (seatalk.local)
                                                                        │
                                                          스타링크 LAN ──┘─▶ nc / python /
                                                                              OpenCPN / Signal K
```

### 개발 흐름 — 실물 없이 PC측을 먼저 (검증 완료)
```
 sim_server (Mac) ─JSON/TCP:2000─▶ n2k_client.py / 내 앱
        ▲                                  │
   같은 JSON 형식                     실물 준비되면 host만
        ▼                            localhost → seatalk.local 로 교체
 seatalk_gateway (ESP32) ─JSON/TCP:2000─▶ (동일 클라이언트, 무수정)
```

### 양방향 제어 확장 자리 (중장기, 미구현)
```
 PC ─명령(JSON)─▶ TCP :2000 ─▶ [파싱] ─▶ encodeXxx ─▶ TWAI(NORMAL) ─▶ CAN ─▶ 오토파일럿
                                        └ 시뮬레이터로 송신 프레임 round-trip 검증 후 실선 적용
```

## 구조
```
seatalk/
├── seatalk_gateway/             # Arduino IDE에서 이 폴더를 연다
│   ├── seatalk_gateway.ino      # ESP32: TWAI 수신 + WiFi(STA)+mDNS + TCP 서버
│   ├── n2k_decoder.h            # 디코더 인터페이스 (호스트/ESP32 공용)
│   ├── n2k_decoder.cpp          # 디코더 + JSON 출력 (Arduino 비종속)
│   ├── secrets.h                # WiFi 자격증명 (gitignore됨, 커밋 안 됨)
│   └── secrets.h.example        # secrets.h 템플릿 (커밋됨)
├── sim/
│   ├── sim_server.cpp           # 호스트 시뮬레이터: ESP32와 동일한 JSON을 TCP로 송출
│   └── Makefile
├── tools/
│   └── n2k_client.py            # PC측 수신 클라이언트 (개발 출발점)
├── test/
│   ├── test_decoder.cpp         # 시뮬레이션 프레임 검증 하니스 (19개 통과)
│   └── Makefile
└── .gitignore                   # secrets.h + 빌드 산출물 제외
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
2. Board: **ESP32 Dev Module**(WROOM 등) + Port: `/dev/cu.usbserial-*` 선택.
   (보드가 회색이면 Boards Manager에서 `esp32 by Espressif Systems` 설치 먼저.)
3. WiFi 자격증명을 `secrets.h`에 넣는다 (커밋되지 않음):
   ```c
   // secrets.h  — secrets.h.example을 복사해서 채운다
   #define WIFI_SSID "YOUR_WIFI_SSID"
   #define WIFI_PASS "YOUR_WIFI_PASSWORD"
   ```
   CAN 핀은 `.ino` 상단에서 배선에 맞춘다 (`CAN_TX_GPIO=5`, `CAN_RX_GPIO=4`).
4. 업로드 후 Serial Monitor를 **115200 baud**로 연다. 부팅 시 WiFi IP가 출력된다.
   (로그가 안 보이면 보드의 `EN`/`RST` 버튼을 눌러 재부팅 → 처음부터 출력됨.)
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

### 데이터 허브(NMEA2000 백본) 연결 — 사용 케이블

게이트웨이를 배의 NMEA2000 네트워크(데이터 허브)에 물리적으로 연결하는 부분.

**사용 부품**
| 부품 | 용도 |
|------|------|
| **M12 5핀 NMEA2000(DeviceNet) 케이블** | 백본 ↔ 트랜시버. 한쪽은 M12 male, 반대쪽은 잘라서 5선 노출(field-cut) |
| **A06045 어댑터** | M12 케이블을 백본의 빈 스퍼/T 포트에 연결 |
| **USB-C 케이블 (데이터 가능 제품)** | ESP32 전원(5V) + 펌웨어 업로드. ※충전 전용은 업로드 안 됨 |

**전체 결선 체인**
```
[NMEA2000 백본의 빈 스퍼/T 포트]
        │
   A06045 어댑터
        │
   M12 5핀 케이블 (male 끝을 어댑터에 꽂음)
        │   반대편 잘린 끝 피복 벗겨 5선 노출
        ├─ 흰색  CAN-H ───────▶ 트랜시버 CANH
        ├─ 파랑  CAN-L ───────▶ 트랜시버 CANL
        ├─ 검정  V−/GND ──────▶ 트랜시버 GND  (= ESP32 GND 공통)
        ├─ 빨강  V+(12V) ─────▶ ✗ 연결 안 함 (테스트 단계)
        └─ 맨선  Shield ──────▶ (지금은 미연결)
                                   │
   트랜시버 ── GPIO5(TX)/GPIO4(RX), 3V3/GND ── ESP32 ── USB-C(5V) ── PC/전원
```

**M12 커넥터 핀맵 / 표준 선색**
| M12 핀 | 신호 | 표준 색 | 연결 위치 |
|:-----:|------|--------|-----------|
| 4 | CAN-H | 흰색 | 트랜시버 CANH |
| 5 | CAN-L | 파랑 | 트랜시버 CANL |
| 3 | V− / GND | 검정 | 트랜시버 GND (공통) |
| 2 | V+ (12V) | 빨강 | **연결 안 함** (테스트) ⚠️ |
| 1 | Shield | 맨선 | 미연결(선택) |

> ⚠️ 선색은 표준일 뿐 제조사마다 다를 수 있다. 결선 전 **멀티미터로 핀↔색을 확인**할 것.

**전원은 두 길 중 하나만 (충돌 금지)** ⚠️
- **테스트 단계** = **USB-C(5V)로만** 전원. M12의 **V+(빨강, 12V)는 연결 안 함.**
- **영구 설치(나중)** = M12 V+(12V) → **벅 컨버터** → ESP32 5V핀. 이때 **USB는 분리.**
- → 두 전원을 동시에 연결하면 충돌한다. 항상 **둘 중 하나만**.

**GND 공통 지점 (중요)** — CAN 통신이 되려면 세 GND가 한 점에 묶여야 한다:
ESP32 GND + 트랜시버 GND + M12 V−(검정). 트랜시버 GND 핀(또는 브레드보드 한 줄)에 셋을 모은다.

**결선 순서 (케이블 도착 후)**
1. M12 male 끝 → A06045 어댑터에 꽂기
2. 어댑터 → 백본 빈 스퍼 포트에 꽂기
3. 케이블 잘린 끝 피복 벗겨 5선 노출
4. 흰색 CAN-H → 트랜시버 CANH
5. 파랑 CAN-L → 트랜시버 CANL
6. 검정 GND → 트랜시버 GND (ESP32 GND와 공통)
7. 빨강 V+, Shield → **연결 안 함**
8. ESP32에 USB-C 꽂아 전원 → Serial Monitor의 `RAW ...` 덤프로 수신 확인

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

모두 단일 프레임 PGN이다. 추가 예정 PGN은 위의 [프로젝트 현황 → 로드맵](#로드맵-소프트웨어) 참고.
