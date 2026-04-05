/*
 * Cable Rig Controller v3 - Dual TMC2209 + WiFi/WebSocket
 * Arduino Nano RP2040 Connect
 *
 * v3.0:
 *   - WiFiNINA web server serves HTML control panel
 *   - WebSocket server for real-time bidirectional control
 *   - Both USB Serial and WebSocket accept commands
 *   - Status push to all connected WebSocket clients
 *   - Multi-client support (up to 4 simultaneous)
 *
 * v2.2:
 *   - Non-blocking wind/unwind (serial stays responsive)
 *   - Proper DIR pin setup time before STEP pulse
 *   - Reliable direction for pay out / unwind
 *
 * PINOUT:
 *   Motor A (Left - vertical):  D2=STEP, D3=DIR, D4=EN
 *   Motor B (Right - horizontal): D5=STEP, D6=DIR, D7=EN
 *   Both share Serial1 TX (via 1k each) for TMC2209 UART
 *   Driver A addr=0 (MS1=GND, MS2=GND)
 *   Driver B addr=1 (MS1=3.3V, MS2=GND)
 *   12V PSU -> VM, Nano 3.3V -> VIO, shared GND
 *
 * REQUIRES:
 *   - TMCStepper library
 *   - WiFiNINA library
 *
 * PROTOCOL (115200 baud, newline-terminated for Serial; framed for WebSocket):
 *   JL+100 / JL-100     Jog left +/- steps
 *   JL+100 C            Jog left + coupled R
 *   JR+100 / JR-100     Jog right +/- steps
 *   ML1000 / MR500      Absolute move
 *   MOVE L1000 R500 S800  Coordinated move
 *   SPEED 600           Steps/sec
 *   HOME                Set 0,0
 *   GOHOME              Return to 0,0
 *   POS                 Report positions
 *   STOP                Emergency stop
 *   ENABLE / DISABLE    Motor drivers
 *   WIND L/R            Continuous wind (send STOP to stop)
 *   UNWIND L/R          Continuous unwind (send STOP to stop)
 *   INVL 1/0            Invert left direction
 *   INVR 1/0            Invert right direction
 *   COUPLE 0.5          Coupling ratio (0=off)
 *   CURRENT 400         RMS current mA
 *   MICRO 16            Microstepping
 *   STEALTHCHOP 1/0     Quiet mode
 *   PING / STATUS
 */

#include <TMCStepper.h>
#include <SPI.h>
#include <WiFiNINA.h>
#include <WiFiUdp.h>
#include <ArduinoMDNS.h>
#include "wifi_config.h"
#include "html_content.h"

// ============ PINS ============
#define L_STEP  2
#define L_DIR   3
#define L_EN    4

#define R_STEP  5
#define R_DIR   6
#define R_EN    7

// ============ UART ============
#define SERIAL_PORT Serial1
#define DRIVER_BAUD 115200
#define R_SENSE     0.11f

TMC2209Stepper driverA(&SERIAL_PORT, R_SENSE, 0);
TMC2209Stepper driverB(&SERIAL_PORT, R_SENSE, 1);

// ============ STATE ============
volatile long posL = 0, posR = 0;
long targetL = 0, targetR = 0;

float stepsPerSec = 600.0;
uint16_t rmsCurrent = 300;
uint16_t microsteps = 16;
bool stealthChop = true;

bool invertL = false, invertR = false;
float coupleRatio = 0.0;

// Winding state
bool windingL = false, windingR = false;
int windDirL = 0, windDirR = 0;

// Coordinated move
bool coordMoving = false;
long coordTargetL = 0, coordTargetR = 0;
float coordSpeed = 0;

// Step timing (non-blocking)
unsigned long lastStepTimeL = 0, lastStepTimeR = 0;

bool motorsEnabled = true;
String inputBuffer = "";

// ============ WiFi + WebSocket ============
const char DEVICE_HOSTNAME[] = "magic-lamp";
const char MDNS_HTTP_SERVICE[] = "Magic Lamp._http";
WiFiServer server(80);
WiFiUDP mdnsUDP;
MDNS mdns(mdnsUDP);
bool mdnsReady = false;

#define MAX_WS_CLIENTS 4
WiFiClient wsClients[MAX_WS_CLIENTS];
bool wsReady[MAX_WS_CLIENTS] = {false};  // true after handshake complete
String wsInputBuf[MAX_WS_CLIENTS];       // per-client command buffer

unsigned long lastStatusPush = 0;
const unsigned long STATUS_PUSH_MS = 200;
long lastPushedL = -99999, lastPushedR = -99999;

// WiFi state machine (non-blocking)
enum WiFiState { WIFI_IDLE, WIFI_DISCONNECTING, WIFI_WAITING, WIFI_CONNECTING, WIFI_CONNECTED };
WiFiState wifiState = WIFI_IDLE;
unsigned long wifiStateTime = 0;
const unsigned long WIFI_RETRY_MS = 3000;
const unsigned long WIFI_DISCONNECT_MS = 200;

// Client watchdog
unsigned long lastClientSeen = 0;
const unsigned long CLIENT_TIMEOUT_MS = 10000;  // 10s with no client = auto-stop

// ============ SHA-1 for WebSocket handshake ============
// Minimal SHA-1 implementation (RFC 3174)

static uint32_t sha1_rol(uint32_t v, int n) { return (v << n) | (v >> (32 - n)); }

void sha1(const uint8_t* data, size_t len, uint8_t hash[20]) {
  uint32_t h0 = 0x67452301, h1 = 0xEFCDAB89, h2 = 0x98BADCFE, h3 = 0x10325476, h4 = 0xC3D2E1F0;
  uint64_t bits = (uint64_t)len * 8;

  // Pad: data + 0x80 + zeros + 8-byte length (big-endian)
  size_t padLen = ((len + 8) / 64 + 1) * 64;
  uint8_t* msg = (uint8_t*)calloc(padLen, 1);
  if (!msg) return;
  memcpy(msg, data, len);
  msg[len] = 0x80;
  for (int i = 0; i < 8; i++) msg[padLen - 1 - i] = (uint8_t)(bits >> (i * 8));

  for (size_t offset = 0; offset < padLen; offset += 64) {
    uint32_t w[80];
    for (int i = 0; i < 16; i++)
      w[i] = ((uint32_t)msg[offset + i * 4] << 24) | ((uint32_t)msg[offset + i * 4 + 1] << 16) |
             ((uint32_t)msg[offset + i * 4 + 2] << 8) | msg[offset + i * 4 + 3];
    for (int i = 16; i < 80; i++)
      w[i] = sha1_rol(w[i - 3] ^ w[i - 8] ^ w[i - 14] ^ w[i - 16], 1);

    uint32_t a = h0, b = h1, c = h2, d = h3, e = h4;
    for (int i = 0; i < 80; i++) {
      uint32_t f, k;
      if (i < 20)      { f = (b & c) | (~b & d);       k = 0x5A827999; }
      else if (i < 40) { f = b ^ c ^ d;                 k = 0x6ED9EBA1; }
      else if (i < 60) { f = (b & c) | (b & d) | (c & d); k = 0x8F1BBCDC; }
      else              { f = b ^ c ^ d;                 k = 0xCA62C1D6; }
      uint32_t temp = sha1_rol(a, 5) + f + e + k + w[i];
      e = d; d = c; c = sha1_rol(b, 30); b = a; a = temp;
    }
    h0 += a; h1 += b; h2 += c; h3 += d; h4 += e;
  }
  free(msg);

  uint32_t hh[5] = {h0, h1, h2, h3, h4};
  for (int i = 0; i < 5; i++) {
    hash[i * 4]     = (hh[i] >> 24) & 0xFF;
    hash[i * 4 + 1] = (hh[i] >> 16) & 0xFF;
    hash[i * 4 + 2] = (hh[i] >> 8) & 0xFF;
    hash[i * 4 + 3] = hh[i] & 0xFF;
  }
}

// Base64 encode
static const char b64[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
String base64Encode(const uint8_t* data, size_t len) {
  String out = "";
  out.reserve((len + 2) / 3 * 4);
  for (size_t i = 0; i < len; i += 3) {
    uint32_t n = (uint32_t)data[i] << 16;
    if (i + 1 < len) n |= (uint32_t)data[i + 1] << 8;
    if (i + 2 < len) n |= data[i + 2];
    out += b64[(n >> 18) & 0x3F];
    out += b64[(n >> 12) & 0x3F];
    out += (i + 1 < len) ? b64[(n >> 6) & 0x3F] : '=';
    out += (i + 2 < len) ? b64[n & 0x3F] : '=';
  }
  return out;
}

// ============ WebSocket frame helpers ============

// Send a text frame to one client
void wsSendFrame(WiFiClient& client, const String& msg) {
  if (!client.connected()) return;
  size_t len = msg.length();
  uint8_t header[4];
  int hLen;

  header[0] = 0x81; // FIN + text opcode
  if (len < 126) {
    header[1] = (uint8_t)len;
    hLen = 2;
  } else {
    header[1] = 126;
    header[2] = (uint8_t)(len >> 8);
    header[3] = (uint8_t)(len & 0xFF);
    hLen = 4;
  }
  client.write(header, hLen);
  client.write((const uint8_t*)msg.c_str(), len);
}

// Send pong frame
void wsSendPong(WiFiClient& client, const uint8_t* payload, size_t len) {
  if (!client.connected()) return;
  uint8_t header[2] = {0x8A, (uint8_t)(len & 0x7F)}; // FIN + pong
  client.write(header, 2);
  if (len > 0) client.write(payload, len);
}

// Send close frame
void wsSendClose(WiFiClient& client) {
  if (!client.connected()) return;
  uint8_t header[2] = {0x88, 0x00}; // FIN + close, 0 length
  client.write(header, 2);
}

// Read and process one WebSocket frame from a client, returns command or ""
// Requires at least 2 bytes (header) before reading to avoid consuming partial frames.
// If the frame is incomplete, the connection is closed since the stream is now corrupted.
String wsReadFrame(WiFiClient& client) {
  if (client.available() < 2) return "";  // need both header bytes before touching the buffer

  uint8_t b0 = client.read();
  uint8_t b1 = client.read();
  uint8_t opcode = b0 & 0x0F;
  bool masked = (b1 & 0x80) != 0;
  size_t payloadLen = b1 & 0x7F;

  if (payloadLen == 126) {
    unsigned long t = millis() + 20;
    while (client.available() < 2 && millis() < t) {}
    if (client.available() < 2) { client.stop(); return ""; }  // stream corrupted
    uint8_t ext[2];
    client.read(ext, 2);
    payloadLen = ((size_t)ext[0] << 8) | ext[1];
  } else if (payloadLen == 127) {
    unsigned long t = millis() + 20;
    while (client.available() < 8 && millis() < t) {}
    if (client.available() < 8) { client.stop(); return ""; }  // stream corrupted
    uint8_t ext[8];
    client.read(ext, 8);
    payloadLen = ((size_t)ext[6] << 8) | ext[7]; // only use lower 16 bits
  }

  uint8_t mask[4] = {0, 0, 0, 0};
  if (masked) {
    unsigned long t = millis() + 20;
    while (client.available() < 4 && millis() < t) {}
    if (client.available() < 4) { client.stop(); return ""; }  // stream corrupted
    client.read(mask, 4);
  }

  // Read payload (cap at 256 bytes for safety)
  size_t toRead = min(payloadLen, (size_t)256);
  uint8_t payload[256];
  size_t got = 0;
  unsigned long timeout = millis() + 20;  // 200ms → 20ms: local WiFi delivers in <5ms
  while (got < toRead && millis() < timeout) {
    if (client.available()) {
      payload[got] = client.read();
      got++;
    }
  }
  if (got < toRead) { client.stop(); return ""; }  // incomplete frame would desync the stream
  // Skip any remaining bytes we didn't read
  for (size_t i = got; i < payloadLen && client.available(); i++) client.read();

  // Unmask
  if (masked) {
    for (size_t i = 0; i < got; i++) payload[i] ^= mask[i % 4];
  }

  // Handle by opcode
  if (opcode == 0x08) {
    // Close frame
    wsSendClose(client);
    client.stop();
    return "";
  }
  if (opcode == 0x09) {
    // Ping -> Pong
    wsSendPong(client, payload, got);
    return "";
  }
  if (opcode == 0x01 || opcode == 0x02) {
    // Text or binary frame -> return as command
    String msg = "";
    msg.reserve(got);
    for (size_t i = 0; i < got; i++) msg += (char)payload[i];
    return msg;
  }
  return "";
}

// ============ SETUP ============

void setup() {
  Serial.begin(115200);

  pinMode(L_STEP, OUTPUT); pinMode(L_DIR, OUTPUT); pinMode(L_EN, OUTPUT);
  pinMode(R_STEP, OUTPUT); pinMode(R_DIR, OUTPUT); pinMode(R_EN, OUTPUT);

  // Set DIR pins to known state
  digitalWrite(L_DIR, LOW);
  digitalWrite(R_DIR, LOW);

  // Enable drivers
  digitalWrite(L_EN, LOW);
  digitalWrite(R_EN, LOW);

  // Init TMC UART
  SERIAL_PORT.begin(DRIVER_BAUD);
  delay(200);  // Let drivers fully boot

  configureDriver(driverA, 'A');
  configureDriver(driverB, 'B');

  delay(50);
  Serial.println("READY v3.0");
  reportStatus();

  setupWiFi();
}

void configureDriver(TMC2209Stepper &drv, char label) {
  drv.begin();
  drv.toff(4);
  drv.blank_time(24);
  drv.rms_current(rmsCurrent);
  drv.microsteps(microsteps);
  drv.TCOOLTHRS(0xFFFFF);
  drv.semin(5);
  drv.semax(2);
  drv.shaft(false);

  if (stealthChop) {
    drv.en_spreadCycle(false);
  } else {
    drv.en_spreadCycle(true);
  }

  drv.pwm_autoscale(true);
  drv.pwm_autograd(true);

  uint8_t result = drv.test_connection();
  Serial.print("TMC "); Serial.print(label);
  if (result == 0) Serial.println(" OK");
  else { Serial.print(" FAIL:"); Serial.println(result); }
}

bool startMDNS() {
  int result = mdns.begin(WiFi.localIP(), DEVICE_HOSTNAME);
  if (result != 1) {
    mdnsReady = false;
    Serial.print("mDNS start failed: ");
    Serial.println(result);
    return false;
  }

  mdnsReady = true;
  mdns.removeAllServiceRecords();

  int svc = mdns.addServiceRecord(MDNS_HTTP_SERVICE, 80, MDNSServiceTCP);
  if (svc != 1) {
    Serial.print("mDNS service add failed: ");
    Serial.println(svc);
  }

  Serial.print("Hostname: http://");
  Serial.print(DEVICE_HOSTNAME);
  Serial.println(".local");
  return true;
}

void setupWiFi() {
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("WiFi module not found");
    wifiState = WIFI_WAITING;
    wifiStateTime = millis();
    return;
  }

  String fv = WiFi.firmwareVersion();
  Serial.print("WiFi FW: ");
  Serial.println(fv);
  if (fv < WIFI_FIRMWARE_LATEST_VERSION) {
    Serial.println("WARNING: WiFi firmware outdated");
  }

  wifiState = WIFI_IDLE;  // tickWiFi() will start connection
}

// Stop all motors immediately — called when communication is lost
void safetyStop() {
  bool wasMoving = windingL || windingR || coordMoving ||
                   (targetL != posL) || (targetR != posR);

  windingL = false;
  windingR = false;
  coordMoving = false;
  targetL = posL;
  targetR = posR;

  if (wasMoving) {
    Serial.println("SAFETY STOP: WiFi lost while motors moving");
  }
}

void startWiFiConnect() {
  WiFi.disconnect();
  WiFi.end();
  mdnsReady = false;
  WiFi.setHostname(DEVICE_HOSTNAME);

  // Drop all WebSocket clients cleanly
  for (int i = 0; i < MAX_WS_CLIENTS; i++) {
    if (wsReady[i]) {
      wsSendClose(wsClients[i]);
      wsClients[i].stop();
      wsReady[i] = false;
    }
  }

  wifiState = WIFI_DISCONNECTING;
  wifiStateTime = millis();
  Serial.print("WiFi: connecting to [");
  Serial.print(WIFI_SSID);
  Serial.println("]");
}

void pushStatus() {
  unsigned long nowMs = millis();
  if (nowMs - lastStatusPush >= STATUS_PUSH_MS) {
    lastStatusPush = nowMs;
    if (posL != lastPushedL || posR != lastPushedR) {
      String msg = "POS L:" + String(posL) + " R:" + String(posR);
      wsBroadcast(msg);
      lastPushedL = posL;
      lastPushedR = posR;
    }
  }
}

void tickWiFi() {
  unsigned long now = millis();

  switch (wifiState) {
    case WIFI_IDLE:
      startWiFiConnect();
      break;

    case WIFI_DISCONNECTING:
      if (now - wifiStateTime >= WIFI_DISCONNECT_MS) {
        WiFi.begin(WIFI_SSID, WIFI_PASS);
        wifiState = WIFI_CONNECTING;
        wifiStateTime = now;
      }
      break;

    case WIFI_CONNECTING:
      if (WiFi.status() == WL_CONNECTED) {
        wifiState = WIFI_CONNECTED;
        wifiStateTime = now;
        lastClientSeen = now;
        WiFi.noLowPowerMode();
        Serial.print("IP: ");
        Serial.println(WiFi.localIP());
        server.begin();
        Serial.println("Server :80 (HTTP + WS)");
        startMDNS();
      } else if (now - wifiStateTime >= 15000) {
        Serial.println("WiFi connect timeout - will retry");
        wifiState = WIFI_WAITING;
        wifiStateTime = now;
      }
      break;

    case WIFI_CONNECTED:
      if (WiFi.status() != WL_CONNECTED) {
        Serial.println("WiFi lost");
        safetyStop();
        mdnsReady = false;
        wifiState = WIFI_WAITING;
        wifiStateTime = now;
      } else {
        if (mdnsReady) mdns.run();
        pollWSClients();
        // IMPORTANT: process existing WS traffic before calling server.available().
        // On this WiFi stack, server.available() can surface an already-upgraded
        // WebSocket client when it has pending bytes. If we call handleNewClient()
        // first, those WS frame bytes get misread as a fresh HTTP request and the
        // board replies with "HTTP/1.1 200 OK", corrupting the WS stream.
        handleNewClient();
        pushStatus();
      }
      break;

    case WIFI_WAITING:
      if (now - wifiStateTime >= WIFI_RETRY_MS) {
        startWiFiConnect();
      }
      break;
  }
}

// ============ MAIN LOOP ============

void loop() {
  // 1. ALWAYS read serial first -- never block this
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (inputBuffer.length() > 0) {
        processCommand(inputBuffer);
        inputBuffer = "";
      }
    } else {
      if (inputBuffer.length() < 64) {
        inputBuffer += c;
      }
    }
  }

  // 2. WiFi: non-blocking state machine
  tickWiFi();

  // 3. Client watchdog: stop motors if no client for 10s
  if (wifiState == WIFI_CONNECTED && millis() - lastClientSeen >= CLIENT_TIMEOUT_MS) {
    bool motorsActive = windingL || windingR || coordMoving ||
                        (targetL != posL) || (targetR != posR);
    if (motorsActive) {
      safetyStop();
      Serial.println("WATCHDOG: No client for 10s, motors stopped");
      lastClientSeen = millis();  // reset so we don't spam
    }
  }

  // 4. Motion (unchanged from v2.2)
  unsigned long now = micros();
  unsigned long stepDelay = calcDelay(stepsPerSec);

  // Continuous winding -- NON-BLOCKING using timing
  if (windingL && now - lastStepTimeL >= stepDelay) {
    stepMotor('L', windDirL);
    lastStepTimeL = now;
  }
  if (windingR && now - lastStepTimeR >= stepDelay) {
    stepMotor('R', windDirR);
    lastStepTimeR = now;
  }

  // Coordinated move
  if (coordMoving) {
    runCoordMove();
    return;
  }

  // Individual target moves (only when not winding)
  if (!windingL && !windingR) {
    runMoves();
  }
}

// ============ CLIENT HANDLER (HTTP + WebSocket on single port) ============

void handleNewClient() {
  WiFiClient client = server.available();
  if (!client) return;
  if (!client.available()) return;
  if (client.peek() != 'G') return;  // existing WS frame data can reappear here; ignore non-HTTP traffic

  // Read HTTP request headers, look for WebSocket upgrade
  String wsKey = "";
  String line = "";
  unsigned long timeout = millis() + 200;  // keep short — 1000ms was starving the main loop

  while (client.connected() && millis() < timeout) {
    if (client.available()) {
      char c = client.read();
      if (c == '\n') {
        line.trim();
        if (line.length() == 0) break; // End of headers
        if (line.startsWith("Sec-WebSocket-Key:")) {
          wsKey = line.substring(19);
          wsKey.trim();
        }
        line = "";
      } else if (c != '\r') {
        line += c;
      }
    }
  }

  if (wsKey.length() > 0) {
    // --- WebSocket upgrade ---
    int slot = -1;
    for (int i = 0; i < MAX_WS_CLIENTS; i++) {
      if (!wsClients[i].connected() && !wsReady[i]) { slot = i; break; }
    }
    if (slot < 0) { client.stop(); return; }

    // Compute accept key: SHA1(key + magic GUID), then base64
    String combined = wsKey + "258EAFA5-E914-47DA-95CA-C5AB0DC85B11";
    uint8_t hash[20];
    sha1((const uint8_t*)combined.c_str(), combined.length(), hash);
    String acceptKey = base64Encode(hash, 20);

    client.println("HTTP/1.1 101 Switching Protocols");
    client.println("Upgrade: websocket");
    client.println("Connection: Upgrade");
    client.print("Sec-WebSocket-Accept: ");
    client.println(acceptKey);
    client.println();

    wsClients[slot] = client;
    wsReady[slot] = true;
    wsInputBuf[slot] = "";

    sendStatusToClient(slot);
    Serial.print("WS client connected: slot ");
    Serial.println(slot);
  } else {
    // --- Regular HTTP → serve HTML ---
    client.println("HTTP/1.1 200 OK");
    client.println("Content-Type: text/html; charset=utf-8");
    client.println("Connection: close");
    client.println();

    const int CHUNK = 512;
    int len = strlen_P(HTML_CONTENT);
    char buf[CHUNK];
    for (int i = 0; i < len; i += CHUNK) {
      int toRead = min(CHUNK, len - i);
      memcpy_P(buf, HTML_CONTENT + i, toRead);
      client.write(buf, toRead);
    }
    client.stop();
  }
}

void pollWSClients() {
  bool anyClient = false;
  for (int i = 0; i < MAX_WS_CLIENTS; i++) {
    if (!wsReady[i]) continue;
    if (!wsClients[i].connected()) {
      wsReady[i] = false;
      Serial.print("WS client disconnected: slot ");
      Serial.println(i);
      continue;
    }
    anyClient = true;
    if (wsClients[i].available()) {
      String cmd = wsReadFrame(wsClients[i]);
      cmd.trim();
      if (cmd.length() > 0) {
        processCommand(cmd);
      }
    }
  }
  if (anyClient) {
    lastClientSeen = millis();
  }
}

void sendStatusToClient(int slot) {
  if (!wsReady[slot]) return;
  wsSendFrame(wsClients[slot], "POS L:" + String(posL) + " R:" + String(posR));
  wsSendFrame(wsClients[slot], "SPEED " + String(stepsPerSec));
  wsSendFrame(wsClients[slot], "CURRENT " + String(rmsCurrent) + "mA");
  wsSendFrame(wsClients[slot], "MICRO " + String(microsteps));
  wsSendFrame(wsClients[slot], String("STEALTH ") + (stealthChop ? "ON" : "OFF"));
  wsSendFrame(wsClients[slot], String("INVL ") + (invertL ? "1" : "0"));
  wsSendFrame(wsClients[slot], String("INVR ") + (invertR ? "1" : "0"));
  wsSendFrame(wsClients[slot], "COUPLE " + String(coupleRatio, 3));
}

// ============ DUAL OUTPUT ============

void wsBroadcast(const String& msg) {
  for (int i = 0; i < MAX_WS_CLIENTS; i++) {
    if (wsReady[i] && wsClients[i].connected()) {
      wsSendFrame(wsClients[i], msg);
    }
  }
}

void respond(const String& msg) {
  Serial.println(msg);
  wsBroadcast(msg);
}

// ============ COMMANDS ============

void processCommand(String cmd) {
  cmd.trim();
  String upper = cmd;
  upper.toUpperCase();

  if (upper == "PING") { respond("PONG"); return; }

  if (upper == "STOP") {
    windingL = false; windingR = false;
    coordMoving = false;
    targetL = posL; targetR = posR;
    respond("OK STOP");
    reportPosition();
    return;
  }

  if (upper == "POS") { reportPosition(); return; }
  if (upper == "STATUS") { reportStatus(); return; }

  if (upper == "HOME") {
    posL = 0; posR = 0; targetL = 0; targetR = 0;
    respond("OK HOME");
    reportPosition();
    return;
  }

  if (upper == "GOHOME") {
    windingL = false; windingR = false;
    coordTargetL = 0; coordTargetR = 0;
    coordSpeed = stepsPerSec;
    coordMoving = true;
    respond("OK GOHOME");
    return;
  }

  if (upper == "ENABLE") {
    digitalWrite(L_EN, LOW); digitalWrite(R_EN, LOW);
    motorsEnabled = true;
    respond("OK ENABLED");
    return;
  }

  if (upper == "DISABLE") {
    digitalWrite(L_EN, HIGH); digitalWrite(R_EN, HIGH);
    motorsEnabled = false;
    windingL = false; windingR = false; coordMoving = false;
    respond("OK DISABLED");
    return;
  }

  // DIRECTION INVERT
  if (upper.startsWith("INVL ")) {
    invertL = (upper.charAt(5) == '1');
    respond(String("OK INVL ") + (invertL ? "ON" : "OFF"));
    return;
  }
  if (upper.startsWith("INVR ")) {
    invertR = (upper.charAt(5) == '1');
    respond(String("OK INVR ") + (invertR ? "ON" : "OFF"));
    return;
  }

  // COUPLING
  if (upper.startsWith("COUPLE ")) {
    coupleRatio = cmd.substring(7).toFloat();
    coupleRatio = constrain(coupleRatio, 0.0, 5.0);
    respond("OK COUPLE " + String(coupleRatio, 3));
    return;
  }

  // CURRENT
  if (upper.startsWith("CURRENT ")) {
    rmsCurrent = constrain(cmd.substring(8).toInt(), 100, 1200);
    driverA.rms_current(rmsCurrent);
    driverB.rms_current(rmsCurrent);
    respond("OK CURRENT " + String(rmsCurrent) + "mA");
    return;
  }

  // MICROSTEPPING
  if (upper.startsWith("MICRO ")) {
    microsteps = cmd.substring(6).toInt();
    driverA.microsteps(microsteps);
    driverB.microsteps(microsteps);
    respond("OK MICRO " + String(microsteps));
    return;
  }

  // STEALTHCHOP
  if (upper.startsWith("STEALTHCHOP ")) {
    stealthChop = (upper.charAt(12) == '1');
    driverA.en_spreadCycle(!stealthChop);
    driverB.en_spreadCycle(!stealthChop);
    respond(String("OK STEALTH ") + (stealthChop ? "ON" : "OFF"));
    return;
  }

  // JOG: JL+100, JR-200, JL+100 C
  if (upper.startsWith("JL") || upper.startsWith("JR")) {
    windingL = false; windingR = false;

    char motor = upper.charAt(1);
    bool coupled = upper.indexOf('C') > 2;

    String numPart = "";
    for (int i = 2; i < (int)upper.length(); i++) {
      char ch = upper.charAt(i);
      if (ch == '+' || ch == '-' || (ch >= '0' && ch <= '9')) numPart += ch;
      else break;
    }
    long steps = numPart.toInt();

    if (motor == 'L') {
      targetL = posL + steps;
      if (coupled && coupleRatio > 0 && steps != 0) {
        long rSteps = (long)(abs(steps) * coupleRatio);
        targetR = posR + (steps > 0 ? rSteps : -rSteps);
      }
    } else {
      targetR = posR + steps;
    }
    coordMoving = false;

    String resp = "OK JOG " + String(motor) + " " + String(steps);
    if (coupled && motor == 'L') {
      resp += " +R:" + String(targetR - posR);
    }
    respond(resp);
    return;
  }

  // ABSOLUTE MOVE
  if (upper.startsWith("ML") || upper.startsWith("MR")) {
    windingL = false; windingR = false;
    char motor = upper.charAt(1);
    long pos = cmd.substring(2).toInt();
    if (motor == 'L') targetL = pos; else targetR = pos;
    coordMoving = false;
    respond("OK MOVE " + String(motor) + " TO " + String(pos));
    return;
  }

  // SET POSITION
  if (upper.startsWith("SETL ") || upper.startsWith("SETR ")) {
    char motor = upper.charAt(3);
    long val = cmd.substring(5).toInt();
    if (motor == 'L') { posL = val; targetL = val; }
    else { posR = val; targetR = val; }
    respond("OK SET " + String(motor) + "=" + String(val));
    reportPosition();
    return;
  }

  // COORDINATED MOVE
  if (upper.startsWith("MOVE ")) {
    windingL = false; windingR = false;
    coordTargetL = posL; coordTargetR = posR;
    coordSpeed = stepsPerSec;

    int idx = upper.indexOf('L');
    if (idx >= 0) coordTargetL = extractNum(upper, idx + 1);
    idx = upper.indexOf('R');
    if (idx >= 0) coordTargetR = extractNum(upper, idx + 1);
    idx = upper.indexOf('S', 5);
    if (idx >= 0) coordSpeed = extractNum(upper, idx + 1);

    coordMoving = true;
    respond("OK COORD L:" + String(coordTargetL) + " R:" + String(coordTargetR) + " S:" + String(coordSpeed));
    return;
  }

  // SPEED
  if (upper.startsWith("SPEED ")) {
    stepsPerSec = constrain(cmd.substring(6).toFloat(), 1, 5000);
    respond("OK SPEED " + String(stepsPerSec));
    return;
  }

  // WIND / UNWIND
  if (upper.startsWith("WIND ") || upper.startsWith("UNWIND ")) {
    bool isWind = upper.startsWith("WIND");
    char motor = upper.charAt(upper.length() - 1);
    int dir = isWind ? 1 : -1;

    coordMoving = false;
    targetL = posL; targetR = posR;
    windingL = false; windingR = false;

    if (motor == 'L') { windingL = true; windDirL = dir; }
    else if (motor == 'R') { windingR = true; windDirR = dir; }

    respond(String("OK ") + (isWind ? "WIND " : "UNWIND ") + String(motor));
    return;
  }

  respond("ERR " + cmd);
}

long extractNum(String s, int start) {
  String num = "";
  for (int i = start; i < (int)s.length(); i++) {
    char c = s.charAt(i);
    if (c == '-' || c == '+' || (c >= '0' && c <= '9')) num += c;
    else if (num.length() > 0) break;
  }
  return num.toInt();
}

// ============ MOTION ============

void runCoordMove() {
  long dL = coordTargetL - posL;
  long dR = coordTargetR - posR;

  if (dL == 0 && dR == 0) {
    coordMoving = false;
    respond("DONE");
    reportPosition();
    return;
  }

  long absL = abs(dL), absR = abs(dR);
  long maxD = max(absL, absR);

  float spdL = maxD > 0 ? coordSpeed * ((float)absL / maxD) : 0;
  float spdR = maxD > 0 ? coordSpeed * ((float)absR / maxD) : 0;

  unsigned long now = micros();

  if (dL != 0 && spdL > 0 && now - lastStepTimeL >= calcDelay(spdL)) {
    stepMotor('L', dL > 0 ? 1 : -1);
    lastStepTimeL = now;
  }
  if (dR != 0 && spdR > 0 && now - lastStepTimeR >= calcDelay(spdR)) {
    stepMotor('R', dR > 0 ? 1 : -1);
    lastStepTimeR = now;
  }
}

void runMoves() {
  long dL = targetL - posL;
  long dR = targetR - posR;
  unsigned long now = micros();
  unsigned long d = calcDelay(stepsPerSec);

  if (dL != 0 && now - lastStepTimeL >= d) {
    stepMotor('L', dL > 0 ? 1 : -1);
    lastStepTimeL = now;
  }
  if (dR != 0 && now - lastStepTimeR >= d) {
    stepMotor('R', dR > 0 ? 1 : -1);
    lastStepTimeR = now;
  }

  static bool pML = false, pMR = false;
  bool mL = (dL != 0), mR = (dR != 0);
  if (pML && !mL) { respond("DONEL"); reportPosition(); }
  if (pMR && !mR) { respond("DONER"); reportPosition(); }
  pML = mL; pMR = mR;
}

// Step a motor one step in the given logical direction
// dir: +1 = reel in, -1 = pay out
void stepMotor(char motor, int dir) {
  if (!motorsEnabled) return;

  int stepPin, dirPin;
  bool inv;

  if (motor == 'L') {
    stepPin = L_STEP; dirPin = L_DIR; inv = invertL;
  } else {
    stepPin = R_STEP; dirPin = R_DIR; inv = invertR;
  }

  // Apply inversion
  int actualDir = inv ? -dir : dir;

  // Set direction pin FIRST with setup time
  digitalWrite(dirPin, actualDir > 0 ? HIGH : LOW);
  delayMicroseconds(5);  // TMC2209 needs >=20ns DIR setup, 5us is safe margin

  // Pulse STEP
  digitalWrite(stepPin, HIGH);
  delayMicroseconds(3);  // TMC2209 needs >=100ns pulse, 3us is safe
  digitalWrite(stepPin, LOW);

  // Track logical position
  if (motor == 'L') posL += dir;
  else posR += dir;
}

unsigned long calcDelay(float speed) {
  if (speed <= 0) return 1000000;
  return (unsigned long)(1000000.0 / speed);
}

void reportPosition() {
  respond("POS L:" + String(posL) + " R:" + String(posR));
}

void reportStatus() {
  reportPosition();
  respond("SPEED " + String(stepsPerSec));
  respond("CURRENT " + String(rmsCurrent) + "mA");
  respond("MICRO " + String(microsteps));
  respond(String("STEALTH ") + (stealthChop ? "ON" : "OFF"));
  respond(String("INVL ") + (invertL ? "1" : "0"));
  respond(String("INVR ") + (invertR ? "1" : "0"));
  respond("COUPLE " + String(coupleRatio, 3));
  Serial.print("TMC_A "); Serial.println(driverA.test_connection() == 0 ? "OK" : "FAIL");
  Serial.print("TMC_B "); Serial.println(driverB.test_connection() == 0 ? "OK" : "FAIL");
}
