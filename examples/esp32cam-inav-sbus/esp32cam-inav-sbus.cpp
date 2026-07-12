/**************************************************
 * ESP32-CAM iNav iBUS Web Controller & Streamer
 * Authored for ESPNowCam project integration
 **************************************************/

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <esp_wifi.h>
#include <drivers/CamAIThinker.h>

// Wi-Fi Access Point Config
const char* ap_ssid = "ESP32-CAM-Drone";
const char* ap_password = "password123";

// iBUS configuration
#define IBUS_TX_PIN 12
#define IBUS_RX_PIN 13  // Unused but needed for Serial1.begin

// Onboard Flash LED
#define FLASH_LED_PIN GPIO_NUM_4

CamAIThinker Camera;
WebServer server(80);
WiFiServer streamServer(81);
WebSocketsServer controlSocket(82);

// Global channel array in RC microsecond values (1000 - 2000)
// Mode 2: Roll = Ch1, Pitch = Ch2, Throttle = Ch3, Yaw = Ch4
// Aux 1 = Ch5, Aux 2 = Ch6, Aux 3 = Ch7, Aux 4 = Ch8
volatile uint16_t rc_channels[8] = {1500, 1500, 1000, 1500, 1000, 1000, 1000, 1000};
volatile unsigned long last_rx_time = 0;
volatile uint16_t cached_rssi_channel = 1000;
volatile int8_t cached_rssi_dbm = -100;
volatile bool control_socket_connected = false;
portMUX_TYPE rcMux = portMUX_INITIALIZER_UNLOCKED;

// FreeRTOS task handle for IBUS transmitter
TaskHandle_t ibusTaskHandle = NULL;
TaskHandle_t streamTaskHandle = NULL;

void updateWifiRssi() {
  static float filtered_rssi = -100.0f;
  wifi_sta_list_t stations;
  if (esp_wifi_ap_get_sta_list(&stations) != ESP_OK || stations.num == 0) {
    filtered_rssi = -100.0f;
  } else {
    filtered_rssi += 0.2f * (stations.sta[0].rssi - filtered_rssi);
  }

  int32_t rssi = constrain((int32_t)lroundf(filtered_rssi), -100, -40);
  cached_rssi_dbm = rssi;
  cached_rssi_channel = map(rssi, -100, -40, 1000, 2000);
}

// Function to build and send the IBUS packet (32 bytes)
void sendIbusFrame() {
  uint8_t packet[32];
  packet[0] = 0x20; // Length
  packet[1] = 0x40; // Command/Type (0x40 for channels)

  uint16_t channels[8];
  unsigned long rx_time;
  portENTER_CRITICAL(&rcMux);
  memcpy(channels, (const void *)rc_channels, sizeof(channels));
  rx_time = last_rx_time;
  portEXIT_CRITICAL(&rcMux);
  bool failsafe = !control_socket_connected || (millis() - rx_time > 1000) || (rx_time == 0);

  uint16_t local_channels[14];
  for (int i = 0; i < 14; i++) {
    if (failsafe) {
      local_channels[i] = (i < 4 && i != 2) ? 1500 : 1000;
    } else {
      local_channels[i] = (i < 8) ? channels[i] : 1500;
    }
  }

  // iBUS carries 14 channels; expose the controller Wi-Fi link on channel 14.
  local_channels[13] = cached_rssi_channel;

  // Pack 14 channels (2 bytes per channel, little-endian)
  for (int i = 0; i < 14; i++) {
    packet[2 + i * 2] = local_channels[i] & 0xFF;
    packet[3 + i * 2] = (local_channels[i] >> 8) & 0xFF;
  }

  // Calculate Checksum (0xFFFF - Sum of first 30 bytes)
  uint16_t checksum = 0xFFFF;
  for (int i = 0; i < 30; i++) {
    checksum -= packet[i];
  }
  packet[30] = checksum & 0xFF;
  packet[31] = (checksum >> 8) & 0xFF;

  Serial1.write(packet, 32);
}

// FreeRTOS Task for running IBUS at strict intervals (~7ms to 10ms is standard, let's use 10ms)
void ibusTransmitterTask(void *pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  while (true) {
    sendIbusFrame();
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(10));
  }
}

// HTML Dashboard Code
const char* html_dashboard = R"rawhtml(
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no">
    <title>ESP32-CAM iNav Control HUD</title>
    <style>
        * {
            box-sizing: border-box;
            margin: 0;
            padding: 0;
            user-select: none;
            -webkit-user-select: none;
        }
        body {
            background-color: #0f111a;
            color: #00e5ff;
            font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
            overflow: hidden;
            display: flex;
            flex-direction: column;
            height: 100vh;
            height: 100dvh;
        }
        header {
            display: flex;
            justify-content: space-between;
            align-items: center;
            padding: 10px 20px;
            background: rgba(15, 17, 26, 0.95);
            border-bottom: 1px solid rgba(255, 255, 255, 0.1);
            z-index: 10;
        }
        h1 {
            font-size: 1.2rem;
            text-transform: uppercase;
            letter-spacing: 2px;
            text-shadow: 0 0 5px #00e5ff;
        }
        .status-panel {
            display: flex;
            gap: 15px;
            font-size: 0.9rem;
        }
        .status-item {
            display: flex;
            align-items: center;
            gap: 5px;
        }
        .status-dot {
            width: 8px;
            height: 8px;
            border-radius: 50%;
            background: #ff3366;
            box-shadow: 0 0 8px #ff3366;
        }
        .status-dot.active {
            background: #00ff66;
            box-shadow: 0 0 8px #00ff66;
        }
        .status-dot.warning {
            background: #ffaa00;
            box-shadow: 0 0 8px #ffaa00;
        }
        .main-container {
            display: flex;
            flex: 1;
            position: relative;
            justify-content: space-between;
            align-items: flex-end;
            padding: 20px;
        }
        .video-container {
            position: absolute;
            top: 0;
            left: 0;
            width: 100%;
            height: 100%;
            z-index: 1;
            display: flex;
            justify-content: center;
            align-items: center;
            background: #000;
        }
        .video-container img {
            width: 100%;
            height: 100%;
            object-fit: cover;
            transform: rotate(180deg);
        }
        .hud-overlay {
            position: absolute;
            top: 0;
            left: 0;
            width: 100%;
            height: 100%;
            z-index: 2;
            pointer-events: none;
            box-shadow: inset 0 0 100px rgba(0, 0, 0, 0.6);
        }
        .hud-crosshair {
            position: absolute;
            top: 50%;
            left: 50%;
            transform: translate(-50%, -50%);
            width: 60px;
            height: 60px;
            border: 2px solid rgba(0, 229, 255, 0.5);
            border-radius: 50%;
        }
        .hud-crosshair::before, .hud-crosshair::after {
            content: '';
            position: absolute;
            background: rgba(0, 229, 255, 0.5);
        }
        .hud-crosshair::before {
            top: 50%;
            left: -10px;
            width: 80px;
            height: 2px;
            transform: translateY(-50%);
        }
        .hud-crosshair::after {
            left: 50%;
            top: -10px;
            height: 80px;
            width: 2px;
            transform: translateX(-50%);
        }
        .control-widget {
            z-index: 5;
            background: rgba(15, 17, 26, 0.85);
            border: 1px solid rgba(0, 229, 255, 0.3);
            border-radius: 12px;
            padding: 15px;
            backdrop-filter: blur(10px);
            box-shadow: 0 4px 20px rgba(0,0,0,0.5);
        }
        .joystick-container {
            display: flex;
            justify-content: center;
            align-items: center;
            width: 160px;
            height: 160px;
            border: 2px solid #00e5ff;
            border-radius: 50%;
            position: relative;
            touch-action: none;
        }
        .joystick-knob {
            width: 50px;
            height: 50px;
            background: radial-gradient(circle, #00e5ff, #008ba3);
            border-radius: 50%;
            position: absolute;
            box-shadow: 0 0 15px #00e5ff;
            cursor: pointer;
        }
        .aux-panel {
            position: absolute;
            top: 20px;
            right: 20px;
            display: flex;
            flex-direction: column;
            gap: 12px;
            z-index: 5;
        }
        .switch-container {
            display: flex;
            align-items: center;
            justify-content: space-between;
            width: 150px;
            background: rgba(15, 17, 26, 0.85);
            border: 1px solid rgba(0, 229, 255, 0.3);
            padding: 8px 12px;
            border-radius: 6px;
        }
        .switch-label {
            font-size: 0.85rem;
            text-transform: uppercase;
        }
        .switch {
            position: relative;
            display: inline-block;
            width: 46px;
            height: 22px;
        }
        .switch input {
            opacity: 0;
            width: 0;
            height: 0;
        }
        .slider {
            position: absolute;
            cursor: pointer;
            top: 0; left: 0; right: 0; bottom: 0;
            background-color: #333;
            transition: .3s;
            border-radius: 34px;
            border: 1px solid #555;
        }
        .slider:before {
            position: absolute;
            content: "";
            height: 14px; width: 14px;
            left: 3px; bottom: 3px;
            background-color: white;
            transition: .3s;
            border-radius: 50%;
        }
        input:checked + .slider {
            background-color: #00ff66;
            border-color: #00ff66;
            box-shadow: 0 0 8px #00ff66;
        }
        input:checked + .slider:before {
            transform: translateX(24px);
        }
        .hud-telemetry {
            position: absolute;
            bottom: 20px;
            left: 50%;
            transform: translateX(-50%);
            background: rgba(15, 17, 26, 0.85);
            border: 1px solid rgba(0, 229, 255, 0.3);
            border-radius: 6px;
            padding: 8px 20px;
            display: flex;
            gap: 20px;
            font-size: 0.9rem;
            z-index: 5;
        }
        @media (max-width: 700px), (max-height: 500px) {
            header {
                padding: 5px 10px;
            }
            h1 {
                font-size: 0.8rem;
                letter-spacing: 1px;
            }
            .status-panel {
                gap: 8px;
                font-size: 0.7rem;
            }
            .main-container {
                padding: 8px;
                padding-bottom: max(8px, env(safe-area-inset-bottom));
                gap: 8px;
            }
            .control-widget {
                padding: 6px;
            }
            .joystick-container {
                width: min(34vw, 130px, calc(100dvh - 105px));
                height: min(34vw, 130px, calc(100dvh - 105px));
            }
            .joystick-knob {
                width: 38px;
                height: 38px;
            }
            .aux-panel {
                top: 8px;
                right: 8px;
                gap: 5px;
            }
            .switch-container {
                width: 125px;
                padding: 4px 7px;
            }
            .switch-label {
                font-size: 0.65rem;
            }
            .hud-telemetry {
                bottom: max(5px, env(safe-area-inset-bottom));
                padding: 4px 7px;
                gap: 7px;
                font-size: 0.65rem;
            }
        }
    </style>
</head>
<body>
    <header>
        <h1>ESP32-CAM iNav Control</h1>
        <div class="status-panel">
            <div class="status-item">
                <span class="status-label">RC Link:</span>
                <span id="link-dot" class="status-dot"></span>
            </div>
            <div class="status-item">
                <span class="status-label">Stream:</span>
                <span id="stream-dot" class="status-dot active"></span>
            </div>
            <div class="status-item"><span id="link-info">SAFE</span></div>
        </div>
    </header>

    <div class="main-container">
        <div class="video-container">
            <!-- Camera stream is rendered here -->
            <img id="camera-stream" alt="Live FPV Stream">
        </div>
        <div class="hud-overlay">
            <div class="hud-crosshair"></div>
        </div>

        <div class="aux-panel">
            <div class="switch-container">
                <span class="switch-label">ARM (AUX1)</span>
                <label class="switch">
                    <input type="checkbox" id="aux1" onchange="armChanged()">
                    <span class="slider"></span>
                </label>
            </div>
            <div class="switch-container">
                <span class="switch-label">ANGLE (AUX2)</span>
                <label class="switch">
                    <input type="checkbox" id="aux2" onchange="sendControl()">
                    <span class="slider"></span>
                </label>
            </div>
            <div class="switch-container">
                <span class="switch-label">FLASH LED</span>
                <label class="switch">
                    <input type="checkbox" id="led" onchange="sendControl()">
                    <span class="slider"></span>
                </label>
            </div>
        </div>

        <!-- Left Joystick: Throttle (Y) and Yaw (X) -->
        <div class="control-widget">
            <div style="text-align: center; margin-bottom: 5px; font-size: 0.75rem; text-transform: uppercase;">Throttle / Yaw</div>
            <div class="joystick-container" id="left-joystick">
                <div class="joystick-knob" id="left-knob"></div>
            </div>
        </div>

        <div class="hud-telemetry">
            <div>THR: <span id="tele-thr">0</span>%</div>
            <div>YAW: <span id="tele-yaw">1500</span></div>
            <div>PITCH: <span id="tele-pitch">1500</span></div>
            <div>ROLL: <span id="tele-roll">1500</span></div>
            <div>RSSI: <span id="tele-rssi">--</span></div>
            <div>RTT: <span id="tele-rtt">--</span>ms</div>
        </div>

        <!-- Right Joystick: Pitch (Y) and Roll (X) -->
        <div class="control-widget">
            <div style="text-align: center; margin-bottom: 5px; font-size: 0.75rem; text-transform: uppercase;">Pitch / Roll</div>
            <div class="joystick-container" id="right-joystick">
                <div class="joystick-knob" id="right-knob"></div>
            </div>
        </div>
    </div>

    <script>
        const leftJoy = document.getElementById('left-joystick');
        const leftKnob = document.getElementById('left-knob');
        const rightJoy = document.getElementById('right-joystick');
        const rightKnob = document.getElementById('right-knob');
        document.getElementById('camera-stream').src = `http://${location.hostname}:81/stream`;
        let controlSocket;
        let reconnectTimer;

        const state = {
            roll: 1500,
            pitch: 1500,
            throttle: 1000,
            yaw: 1500,
            aux1: 1000,
            aux2: 1000,
            led: 0
        };

        // Center knobs
        function centerKnob(knob, container, isLeft = false) {
            const rect = container.getBoundingClientRect();
            const center = rect.width / 2;
            const knobRect = knob.getBoundingClientRect();
            knob.style.left = (center - knobRect.width / 2) + 'px';
            if (isLeft) {
                // Throttle stays at bottom (zero) by default
                knob.style.top = (rect.height - knobRect.height) + 'px';
            } else {
                knob.style.top = (center - knobRect.height / 2) + 'px';
            }
        }

        setTimeout(() => {
            centerKnob(leftKnob, leftJoy, true);
            centerKnob(rightKnob, rightJoy, false);
        }, 100);

        function handleJoystick(e, knob, container, isLeft) {
            const rect = container.getBoundingClientRect();
            const knobRect = knob.getBoundingClientRect();
            const center = rect.width / 2;
            
            let clientX, clientY;
            if (e.touches && e.touches.length > 0) {
                clientX = e.touches[0].clientX;
                clientY = e.touches[0].clientY;
            } else {
                clientX = e.clientX;
                clientY = e.clientY;
            }

            let dx = clientX - (rect.left + center);
            let dy = clientY - (rect.top + center);

            // Limit to circle
            const distance = Math.min(center - knobRect.width / 2, Math.sqrt(dx*dx + dy*dy));
            const angle = Math.atan2(dy, dx);
            
            const px = distance * Math.cos(angle);
            const py = distance * Math.sin(angle);

            knob.style.left = (center + px - knobRect.width / 2) + 'px';
            knob.style.top = (center + py - knobRect.height / 2) + 'px';

            const maxVal = center - knobRect.width / 2;
            const normX = px / maxVal;
            const normY = -py / maxVal; // Invert Y-axis

            if (isLeft) {
                state.yaw = Math.round(1500 + normX * 500);
                state.throttle = Math.round(1500 + normY * 500);
                state.throttle = Math.max(1000, Math.min(2000, state.throttle));
            } else {
                state.roll = Math.round(1500 + normX * 500);
                state.pitch = Math.round(1500 + normY * 500);
            }

            updateTelemetry();
            sendControl();
        }

        function setupJoystickEvents(knob, container, isLeft) {
            let active = false;

            const start = (e) => { active = true; handleJoystick(e, knob, container, isLeft); };
            const move = (e) => { if (active) { handleJoystick(e, knob, container, isLeft); } };
            const end = () => {
                if (active) {
                    active = false;
                    if (isLeft) {
                        // Keep throttle where it is, center yaw
                        state.yaw = 1500;
                        leftKnob.style.left = (container.getBoundingClientRect().width / 2 - knob.getBoundingClientRect().width / 2) + 'px';
                    } else {
                        // Center right stick fully
                        state.roll = 1500;
                        state.pitch = 1500;
                        centerKnob(rightKnob, rightJoy, false);
                    }
                    updateTelemetry();
                    sendControl();
                }
            };

            container.addEventListener('mousedown', start);
            window.addEventListener('mousemove', move);
            window.addEventListener('mouseup', end);

            container.addEventListener('touchstart', start);
            window.addEventListener('touchmove', move);
            window.addEventListener('touchend', end);
        }

        setupJoystickEvents(leftKnob, leftJoy, true);
        setupJoystickEvents(rightKnob, rightJoy, false);

        function updateTelemetry() {
            document.getElementById('tele-thr').innerText = Math.round(((state.throttle - 1000) / 1000) * 100);
            document.getElementById('tele-yaw').innerText = state.yaw;
            document.getElementById('tele-pitch').innerText = state.pitch;
            document.getElementById('tele-roll').innerText = state.roll;
        }

        let lastSendTime = 0;
        let controlsEnabled = true;

        function connectControls() {
            clearTimeout(reconnectTimer);
            controlSocket = new WebSocket(`ws://${location.hostname}:82/`);
            controlSocket.onopen = () => {
                document.getElementById('link-info').innerText = 'SAFE';
                sendControl(true);
            };
            controlSocket.onmessage = event => {
                const status = JSON.parse(event.data);
                const linkDot = document.getElementById('link-dot');
                linkDot.classList.toggle('active', !status.failsafe);
                document.getElementById('tele-rssi').innerText = `${status.rssi}dBm`;
                document.getElementById('tele-rtt').innerText = Math.round(performance.now() - status.sent);
                document.getElementById('link-info').innerText = status.failsafe ? 'FAILSAFE' : 'LIVE';
            };
            controlSocket.onclose = () => {
                document.getElementById('link-dot').classList.remove('active');
                document.getElementById('link-info').innerText = 'LOST';
                reconnectTimer = setTimeout(connectControls, 500);
            };
            controlSocket.onerror = () => controlSocket.close();
        }

        function armChanged() {
            if (document.getElementById('aux1').checked &&
                !confirm('Arm the aircraft? Keep propellers clear.')) {
                document.getElementById('aux1').checked = false;
            }
            sendControl(true);
        }

        function safeControls() {
            controlsEnabled = false;
            state.throttle = 1000;
            state.roll = state.pitch = state.yaw = 1500;
            document.getElementById('aux1').checked = false;
            sendControl(true);
        }

        function sendControl(force = false) {
            state.aux1 = document.getElementById('aux1').checked ? 2000 : 1000;
            state.aux2 = document.getElementById('aux2').checked ? 2000 : 1000;
            state.led = document.getElementById('led').checked ? 1 : 0;

            const now = Date.now();
            if (!force && now - lastSendTime < 50) return;
            if (!controlSocket || controlSocket.readyState !== WebSocket.OPEN) return;
            lastSendTime = now;
            controlSocket.send(`${state.roll},${state.pitch},${state.throttle},${state.yaw},${state.aux1},${state.aux2},${state.led},${performance.now()}`);
        }

        // Refresh unchanged channel values so the receiver failsafe only trips
        // when the browser or Wi-Fi link is actually lost.
        setInterval(sendControl, 250);
        document.addEventListener('visibilitychange', () => {
            if (document.hidden) safeControls();
            else controlsEnabled = true;
        });
        window.addEventListener('pagehide', safeControls);
        connectControls();
    </script>
</body>
</html>
)rawhtml";

void handleRoot() {
  server.sendHeader("Cache-Control", "no-store, no-cache, must-revalidate");
  server.send(200, "text/html", html_dashboard);
}

void handleControlSocket(uint8_t client, WStype_t type, uint8_t *payload, size_t length) {
  if (type == WStype_DISCONNECTED) {
    control_socket_connected = false;
    return;
  }
  if (type == WStype_CONNECTED) {
    control_socket_connected = true;
    return;
  }
  if (type != WStype_TEXT || length >= 128) return;

  payload[length] = '\0';
  uint16_t incoming[6];
  int led;
  float sent;
  int fields = sscanf((char *)payload, "%hu,%hu,%hu,%hu,%hu,%hu,%d,%f",
                      &incoming[0], &incoming[1], &incoming[2], &incoming[3],
                      &incoming[4], &incoming[5], &led, &sent);
  if (fields != 8 || led < 0 || led > 1) return;
  for (uint16_t channel : incoming) {
    if (channel < 1000 || channel > 2000) return;
  }

  portENTER_CRITICAL(&rcMux);
  memcpy((void *)rc_channels, incoming, sizeof(incoming));
  last_rx_time = millis();
  portEXIT_CRITICAL(&rcMux);
  digitalWrite(FLASH_LED_PIN, led ? HIGH : LOW);

  char response[96];
  snprintf(response, sizeof(response),
           "{\"rssi\":%d,\"failsafe\":false,\"sent\":%.1f}", cached_rssi_dbm, sent);
  controlSocket.sendTXT(client, response);
}

void streamServerTask(void *pvParameters) {
  while (true) {
    WiFiClient client = streamServer.available();
    if (!client) {
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }

    // Consume the HTTP request headers before starting the MJPEG response.
    client.setTimeout(1000);
    while (client.connected()) {
      String line = client.readStringUntil('\n');
      if (line == "\r" || line.length() == 0) break;
    }
  
    client.println("HTTP/1.1 200 OK");
    client.println("Content-Type: multipart/x-mixed-replace; boundary=frame");
    client.println("Access-Control-Allow-Origin: *");
    client.println("Connection: close");
    client.println();

    while (client.connected()) {
      if (!Camera.get()) {
        vTaskDelay(pdMS_TO_TICKS(10));
        continue;
      }

      client.print("--frame\r\n");
      client.print("Content-Type: image/jpeg\r\n");
      client.printf("Content-Length: %d\r\n\r\n", Camera.fb->len);
      size_t written = client.write(Camera.fb->buf, Camera.fb->len);
      client.print("\r\n");

      Camera.free();
      if (written == 0) break;
      unsigned long link_age = millis() - last_rx_time;
      vTaskDelay(pdMS_TO_TICKS(link_age > 500 ? 100 : 40));
    }
    client.stop();
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("\n--- ESP32-CAM iNav iBUS Web Controller Starting ---");

  // Onboard LED Setup
  pinMode(FLASH_LED_PIN, OUTPUT);
  digitalWrite(FLASH_LED_PIN, LOW);

  // IBUS Hardware Serial setup (standard non-inverted 115200 8N1)
  Serial1.begin(115200, SERIAL_8N1, IBUS_RX_PIN, IBUS_TX_PIN, false);
  Serial.printf("IBUS output initialized on GPIO %d (115200 baud, 8N1)\n", IBUS_TX_PIN);

  // Setup Wi-Fi AP Mode
  WiFi.softAP(ap_ssid, ap_password);
  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);

  // Initialize Camera for JPEG streaming
  Camera.config.pixel_format = PIXFORMAT_JPEG;
  Camera.config.frame_size = FRAMESIZE_QVGA; // 320x240 for fluid frame rates over AP
  Camera.config.jpeg_quality = 12;
  Camera.config.fb_count = 2;

  if (!Camera.begin()) {
    Serial.println("Camera driver initialization failed");
  } else {
    Serial.println("Camera driver successfully initialized");
  }

  // Setup Web Server Handlers
  server.on("/", HTTP_GET, handleRoot);
  server.begin();
  Serial.println("HTTP server started");

  controlSocket.begin();
  controlSocket.onEvent(handleControlSocket);
  controlSocket.enableHeartbeat(500, 1500, 2);
  Serial.println("WebSocket controls started on port 82");

  // Keep the long-lived MJPEG connection away from the control web server.
  streamServer.begin();
  xTaskCreatePinnedToCore(
    streamServerTask, "MJPEG_Stream", 4096, NULL, 1, &streamTaskHandle, 1
  );
  Serial.println("MJPEG stream started on port 81");

  // Spawn low-jitter IBUS sender task on Core 0 (leaving Core 1 for WiFi/Webserver processing)
  xTaskCreatePinnedToCore(
    ibusTransmitterTask,
    "IBUS_Tx",
    2048,
    NULL,
    10, // High priority
    &ibusTaskHandle,
    0   // Core 0
  );
  Serial.println("IBUS FreeRTOS Task spawned on Core 0");
}

void loop() {
  static unsigned long last_rssi_update = 0;
  if (millis() - last_rssi_update >= 250) {
    last_rssi_update = millis();
    updateWifiRssi();
  }
  server.handleClient();
  controlSocket.loop();
  delay(1);
}
