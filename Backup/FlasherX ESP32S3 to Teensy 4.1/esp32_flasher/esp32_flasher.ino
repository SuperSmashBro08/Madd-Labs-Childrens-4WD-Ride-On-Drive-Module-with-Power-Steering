/*
 * ESP32-S3 Teensy 4.1 FlasherX Web Interface
 * 
 * Custom PCB Pin Configuration:
 * - Flashing: ESP32 TX0/RX0 (GPIO43/44) ↔ Teensy Serial2 (Pin 7/8)
 * - Debug:    ESP32 GPIO17/18 ↔ Teensy Serial3 (Pin 14/15)
 * 
 * Features:
 * - Web server at 192.168.1.29 for firmware uploads
 * - FlasherX protocol for Teensy 4.1 programming
 * - OTA updates for ESP32 itself
 * - Upload progress bar and error reporting
 * - Serial monitoring via WebSocket
 */

#include <WiFi.h>
#include <WebServer.h>
#include <Update.h>
#include <ArduinoJson.h>
#include <WebSocketsServer.h>

// WiFi Configuration
const char* ssid = "MySpectrumWiFib0";
const char* password = "jollywhale162";
IPAddress local_IP(192, 168, 1, 29);
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0);

// ============================================
// Version Information
// ============================================
#define ESP_FIRMWARE_VERSION "1.2.3"
#define ESP_BUILD_DATE __DATE__ " " __TIME__
String teensyVersion = "Unknown";

// ============================================
// Custom PCB UART Pin Configuration
// ============================================

// Flashing UART: ESP32 UART1 (GPIO43/44) ↔ Teensy Serial2 (Pin 7/8)
// Used for firmware upload via FlasherX protocol
#define TEENSY_FLASH_SERIAL Serial1
#define TEENSY_FLASH_TX_PIN 43  // ESP32 TX → Teensy RX2 (Pin 7)
#define TEENSY_FLASH_RX_PIN 44  // ESP32 RX ← Teensy TX2 (Pin 8)

// Debug/Monitor UART: ESP32 UART2 (GPIO17/18) ↔ Teensy Serial3 (Pin 14/15)
// Used for serial monitoring and debug output
#define TEENSY_DEBUG_SERIAL Serial2
#define TEENSY_DEBUG_TX_PIN 17  // ESP32 TX → Teensy RX3 (Pin 15)
#define TEENSY_DEBUG_RX_PIN 18  // ESP32 RX ← Teensy TX3 (Pin 14)

// Control pins for Teensy reset/program mode
#define TEENSY_RESET_PIN 4
#define TEENSY_PROGRAM_PIN 5

// RGB LED on pin 38 (WS2812/NeoPixel style on ESP32-S3)
#define RGB_LED_PIN 38

// ============================================
// FlasherX protocol constants
// ============================================
#define FLASHERX_SYNC 0x55
#define FLASHERX_ACK 0x06
#define FLASHERX_NAK 0x15
#define FLASHERX_BLOCK_SIZE 256
#define FLASHERX_TIMEOUT_MS 5000
#define CMD_PING 0x03
#define CMD_VERSION 0x04
#define CMD_UPDATE 0x05

// Web server and WebSocket
WebServer server(80);
WebSocketsServer webSocket(81);

// Timing for Teensy detection
unsigned long lastTeensyPing = 0;
#define TEENSY_PING_INTERVAL 5000

// Upload state
bool uploadInProgress = false;
size_t uploadedBytes = 0;
size_t totalBytes = 0;
String uploadError = "";
bool teensyConnected = false;
bool espUpdating = false;
bool teensyUpdating = false;

// RGB LED state
enum LedState {
    LED_OFF,
    LED_RED,           // Not connected
    LED_GREEN,         // Connected, no heartbeat
    LED_GREEN_PULSE,   // Connected with heartbeat
    LED_BLUE_PULSE     // Updating
};
LedState currentLedState = LED_OFF;
unsigned long lastHeartbeat = 0;
unsigned long lastLedUpdate = 0;
uint8_t ledBrightness = 0;
bool ledBrightnessUp = true;
#define HEARTBEAT_TIMEOUT 15000  // 15 seconds without heartbeat = no pulse
#define LED_PULSE_SPEED 10       // ms between brightness changes

// Serial buffer for monitoring
#define SERIAL_BUFFER_SIZE 4096
char serialBuffer[SERIAL_BUFFER_SIZE];
int serialBufferIndex = 0;
int g_hexLineCount = 0;  // Global hex line counter

// Forward declarations
void handleRoot();
void handleTeensyUpload();
void handleEsp32Ota();
void handleStatus();
void handleTeensyReset();
void webSocketEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t length);
bool flasherXSync();
bool flasherXSendBlock(uint8_t* data, size_t len, uint32_t address);
bool flasherXFinish();
void broadcastSerial(const char* msg);
void logMessage(const char* msg);
bool pingTeensy();
void requestTeensyVersion();

// HTML content (embedded)
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <title>ESP32-S3 Teensy FlasherX</title>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
        * { box-sizing: border-box; margin: 0; padding: 0; }
        body { font-family: 'Segoe UI', Arial, sans-serif; background: #1a1a2e; color: #eee; min-height: 100vh; }
        .container { max-width: 900px; margin: 0 auto; padding: 20px; }
        h1 { color: #00d4ff; text-align: center; margin-bottom: 30px; }
        .card { background: #16213e; border-radius: 10px; padding: 20px; margin-bottom: 20px; box-shadow: 0 4px 6px rgba(0,0,0,0.3); }
        .card h2 { color: #00d4ff; margin-bottom: 15px; font-size: 1.2em; }
        .upload-area { border: 2px dashed #00d4ff; border-radius: 10px; padding: 40px; text-align: center; cursor: pointer; transition: all 0.3s; }
        .upload-area:hover { background: rgba(0,212,255,0.1); }
        .upload-area.dragover { background: rgba(0,212,255,0.2); border-color: #00ff88; }
        input[type="file"] { display: none; }
        .btn { background: #00d4ff; color: #1a1a2e; border: none; padding: 12px 24px; border-radius: 5px; cursor: pointer; font-weight: bold; transition: all 0.3s; margin: 5px; }
        .btn:hover { background: #00ff88; }
        .btn:disabled { background: #555; cursor: not-allowed; }
        .btn-danger { background: #ff4444; color: white; }
        .btn-danger:hover { background: #ff6666; }
        .progress-container { background: #0f0f23; border-radius: 10px; overflow: hidden; height: 30px; margin: 15px 0; position: relative; }
        .progress-bar { height: 100%; background: linear-gradient(90deg, #00d4ff, #00ff88); width: 0%; transition: width 0.3s; }
        .progress-text { position: absolute; top: 50%; left: 50%; transform: translate(-50%, -50%); font-weight: bold; }
        .status { padding: 10px; border-radius: 5px; margin: 10px 0; }
        .status.success { background: rgba(0,255,136,0.2); border: 1px solid #00ff88; }
        .status.error { background: rgba(255,68,68,0.2); border: 1px solid #ff4444; }
        .status.info { background: rgba(0,212,255,0.2); border: 1px solid #00d4ff; }
        .serial-monitor { background: #0a0a0a; border-radius: 5px; padding: 15px; height: 300px; overflow-y: auto; font-family: 'Consolas', monospace; font-size: 12px; white-space: pre-wrap; word-wrap: break-word; }
        .serial-monitor .timestamp { color: #888; }
        .serial-monitor .esp { color: #00d4ff; }
        .serial-monitor .teensy { color: #00ff88; }
        .serial-monitor .error { color: #ff4444; }
        .controls { display: flex; gap: 10px; margin-top: 10px; }
        .indicator { display: inline-block; width: 12px; height: 12px; border-radius: 50%; margin-right: 8px; }
        .indicator.connected { background: #00ff88; box-shadow: 0 0 10px #00ff88; }
        .indicator.disconnected { background: #ff4444; }
        .flex-row { display: flex; gap: 20px; flex-wrap: wrap; }
        .flex-row > .card { flex: 1; min-width: 300px; }
        @media (max-width: 768px) { .flex-row { flex-direction: column; } }
    </style>
</head>
<body>
    <div class="container">
        <h1>ESP32-S3 Teensy FlasherX</h1>
        
        <div class="card">
            <h2>System Status</h2>
            <p><span class="indicator" id="espIndicator"></span>ESP32: <span id="espStatus">Checking...</span> <span id="espVersion" style="color:#888; font-size:0.9em;"></span></p>
            <p><span class="indicator" id="teensyIndicator"></span>Teensy 4.1: <span id="teensyStatus">Checking...</span> <span id="teensyVersion" style="color:#888; font-size:0.9em;"></span></p>
            <button class="btn" onclick="checkStatus()">Refresh Status</button>
            <button class="btn btn-danger" onclick="resetTeensy()">Reset Teensy</button>
        </div>

        <div class="flex-row">
            <div class="card">
                <h2>Flash Teensy 4.1</h2>
                <div class="upload-area" id="teensyUpload" onclick="document.getElementById('teensyFile').click()">
                    <p>Click or drag .hex file here</p>
                    <p style="color: #888; font-size: 0.9em; margin-top: 10px;">Teensy firmware upload via FlasherX</p>
                </div>
                <input type="file" id="teensyFile" accept=".hex,.bin" onchange="uploadTeensy(this.files[0])">
                <div class="progress-container" id="teensyProgress" style="display:none;">
                    <div class="progress-bar" id="teensyProgressBar"></div>
                    <span class="progress-text" id="teensyProgressText">0%</span>
                </div>
                <div id="teensyStatus2"></div>
            </div>

            <div class="card">
                <h2>ESP32 OTA Update</h2>
                <div class="upload-area" id="espUpload" onclick="document.getElementById('espFile').click()">
                    <p>Click or drag .bin file here</p>
                    <p style="color: #888; font-size: 0.9em; margin-top: 10px;">ESP32 firmware OTA update</p>
                </div>
                <input type="file" id="espFile" accept=".bin" onchange="uploadEsp(this.files[0])">
                <div class="progress-container" id="espProgress" style="display:none;">
                    <div class="progress-bar" id="espProgressBar"></div>
                    <span class="progress-text" id="espProgressText">0%</span>
                </div>
                <div id="espStatus2"></div>
            </div>
        </div>

        <div class="card">
            <h2>Serial Monitor</h2>
            <div class="serial-monitor" id="serialMonitor"></div>
            <div class="controls">
                <button class="btn" onclick="clearSerial()">Clear</button>
                <button class="btn" id="autoScrollBtn" onclick="toggleAutoScroll()">Auto-scroll: ON</button>
            </div>
        </div>
    </div>

    <script>
        let ws;
        let autoScroll = true;
        
        function initWebSocket() {
            ws = new WebSocket('ws://' + window.location.hostname + ':81');
            ws.onopen = () => logSerial('ESP', 'WebSocket connected');
            ws.onclose = () => { logSerial('ERROR', 'WebSocket disconnected'); setTimeout(initWebSocket, 2000); };
            ws.onmessage = (e) => {
                try {
                    const data = JSON.parse(e.data);
                    if (data.type === 'serial') logSerial(data.source, data.message);
                    else if (data.type === 'progress') updateProgress(data);
                    else if (data.type === 'status') updateStatus(data);
                    else if (data.type === 'ota_complete' && data.success) {
                        document.getElementById('espStatus2').innerHTML = '<div class="status success">OTA Update complete! Rebooting...</div>';
                        document.getElementById('espProgressBar').style.width = '100%';
                        document.getElementById('espProgressText').textContent = '100%';
                        logSerial('ESP', 'OTA successful - ESP32 rebooting...');
                        setTimeout(() => location.reload(), 5000);
                    }
                    else if (data.type === 'teensy_complete' && data.success) {
                        document.getElementById('teensyStatus2').innerHTML = '<div class="status success">Teensy flash complete!</div>';
                        document.getElementById('teensyProgressBar').style.width = '100%';
                        document.getElementById('teensyProgressText').textContent = '100%';
                        logSerial('ESP', 'Teensy flash successful');
                    }
                } catch { logSerial('TEENSY', e.data); }
            };
        }
        
        function logSerial(source, msg) {
            const monitor = document.getElementById('serialMonitor');
            const time = new Date().toLocaleTimeString();
            const cls = source === 'ERROR' ? 'error' : source === 'ESP' ? 'esp' : 'teensy';
            monitor.innerHTML += `<span class="timestamp">[${time}]</span> <span class="${cls}">[${source}]</span> ${escapeHtml(msg)}\n`;
            if (autoScroll) monitor.scrollTop = monitor.scrollHeight;
        }
        
        function escapeHtml(text) {
            const div = document.createElement('div');
            div.textContent = text;
            return div.innerHTML;
        }
        
        function clearSerial() { document.getElementById('serialMonitor').innerHTML = ''; }
        
        function toggleAutoScroll() {
            autoScroll = !autoScroll;
            document.getElementById('autoScrollBtn').textContent = 'Auto-scroll: ' + (autoScroll ? 'ON' : 'OFF');
        }
        
        async function checkStatus() {
            try {
                const res = await fetch('/status');
                const data = await res.json();
                document.getElementById('espStatus').textContent = 'Connected';
                document.getElementById('espIndicator').className = 'indicator connected';
                document.getElementById('espVersion').textContent = data.espVersion ? '(v' + data.espVersion + ')' : '';
                document.getElementById('teensyStatus').textContent = data.teensy ? 'Connected' : 'Not detected';
                document.getElementById('teensyIndicator').className = 'indicator ' + (data.teensy ? 'connected' : 'disconnected');
                document.getElementById('teensyVersion').textContent = data.teensyVersion && data.teensyVersion !== 'Unknown' ? '(v' + data.teensyVersion + ')' : '';
            } catch {
                document.getElementById('espStatus').textContent = 'Disconnected';
                document.getElementById('espIndicator').className = 'indicator disconnected';
            }
        }
        
        async function resetTeensy() {
            logSerial('ESP', 'Resetting Teensy...');
            await fetch('/teensy/reset', { method: 'POST' });
        }
        
        function updateProgress(data) {
            const prefix = data.target === 'teensy' ? 'teensy' : 'esp';
            document.getElementById(prefix + 'Progress').style.display = 'block';
            document.getElementById(prefix + 'ProgressBar').style.width = data.percent + '%';
            document.getElementById(prefix + 'ProgressText').textContent = data.percent + '%';
        }
        
        async function uploadTeensy(file) {
            if (!file) return;
            const formData = new FormData();
            formData.append('firmware', file);
            
            document.getElementById('teensyProgress').style.display = 'block';
            document.getElementById('teensyStatus2').innerHTML = '<div class="status info">Uploading...</div>';
            logSerial('ESP', 'Starting Teensy upload: ' + file.name);
            
            const xhr = new XMLHttpRequest();
            xhr.upload.onprogress = (e) => {
                if (e.lengthComputable) {
                    const pct = Math.round((e.loaded / e.total) * 100);
                    document.getElementById('teensyProgressBar').style.width = pct + '%';
                    document.getElementById('teensyProgressText').textContent = pct + '%';
                }
            };
            xhr.onload = () => {
                if (xhr.status === 200) {
                    document.getElementById('teensyStatus2').innerHTML = '<div class="status success">Upload complete!</div>';
                    logSerial('ESP', 'Teensy flash successful');
                } else {
                    document.getElementById('teensyStatus2').innerHTML = '<div class="status error">' + xhr.responseText + '</div>';
                    logSerial('ERROR', 'Teensy flash failed: ' + xhr.responseText);
                }
            };
            xhr.onerror = () => {
                document.getElementById('teensyStatus2').innerHTML = '<div class="status error">Upload failed</div>';
            };
            xhr.open('POST', '/teensy/upload');
            xhr.send(formData);
        }
        
        async function uploadEsp(file) {
            if (!file) return;
            const formData = new FormData();
            formData.append('firmware', file);
            
            document.getElementById('espProgress').style.display = 'block';
            document.getElementById('espStatus2').innerHTML = '<div class="status info">Uploading...</div>';
            logSerial('ESP', 'Starting ESP32 OTA update: ' + file.name);
            
            const xhr = new XMLHttpRequest();
            xhr.upload.onprogress = (e) => {
                if (e.lengthComputable) {
                    const pct = Math.round((e.loaded / e.total) * 100);
                    document.getElementById('espProgressBar').style.width = pct + '%';
                    document.getElementById('espProgressText').textContent = pct + '%';
                }
            };
            xhr.onload = () => {
                if (xhr.status === 200) {
                    document.getElementById('espStatus2').innerHTML = '<div class="status success">Update complete! Rebooting...</div>';
                    logSerial('ESP', 'OTA update successful, rebooting...');
                    setTimeout(() => location.reload(), 5000);
                } else {
                    document.getElementById('espStatus2').innerHTML = '<div class="status error">' + xhr.responseText + '</div>';
                    logSerial('ERROR', 'OTA failed: ' + xhr.responseText);
                }
            };
            xhr.onerror = () => {
                // Connection lost likely means ESP32 is rebooting after successful OTA
                document.getElementById('espStatus2').innerHTML = '<div class="status success">Update complete! Rebooting...</div>';
                logSerial('ESP', 'Connection lost - ESP32 likely rebooting after successful OTA');
                setTimeout(() => location.reload(), 5000);
            };
            xhr.open('POST', '/esp/ota');
            xhr.send(formData);
        }
        
        // Drag and drop support
        ['teensyUpload', 'espUpload'].forEach(id => {
            const el = document.getElementById(id);
            el.ondragover = (e) => { e.preventDefault(); el.classList.add('dragover'); };
            el.ondragleave = () => el.classList.remove('dragover');
            el.ondrop = (e) => {
                e.preventDefault();
                el.classList.remove('dragover');
                const file = e.dataTransfer.files[0];
                if (id === 'teensyUpload') uploadTeensy(file);
                else uploadEsp(file);
            };
        });
        
        initWebSocket();
        checkStatus();
        setInterval(checkStatus, 5000);
    </script>
</body>
</html>
)rawliteral";

// ============================================
// RGB LED Functions
// ============================================
void setLedColor(uint8_t r, uint8_t g, uint8_t b) {
    neopixelWrite(RGB_LED_PIN, r, g, b);
}

void updateLedState() {
    unsigned long now = millis();
    
    // Determine current state based on system status
    if (espUpdating || teensyUpdating) {
        currentLedState = LED_BLUE_PULSE;
    } else if (WiFi.status() != WL_CONNECTED) {
        currentLedState = LED_RED;
    } else if (!teensyConnected) {
        currentLedState = LED_RED;
    } else if (now - lastHeartbeat < HEARTBEAT_TIMEOUT) {
        currentLedState = LED_GREEN_PULSE;
    } else {
        currentLedState = LED_GREEN;
    }
    
    // Update LED based on state
    if (now - lastLedUpdate >= LED_PULSE_SPEED) {
        lastLedUpdate = now;
        
        switch (currentLedState) {
            case LED_OFF:
                setLedColor(0, 0, 0);
                break;
                
            case LED_RED:
                setLedColor(255, 0, 0);
                break;
                
            case LED_GREEN:
                setLedColor(0, 255, 0);
                break;
                
            case LED_GREEN_PULSE:
                // Pulse brightness up and down
                if (ledBrightnessUp) {
                    ledBrightness += 5;
                    if (ledBrightness >= 255) {
                        ledBrightness = 255;
                        ledBrightnessUp = false;
                    }
                } else {
                    ledBrightness -= 5;
                    if (ledBrightness <= 30) {
                        ledBrightness = 30;
                        ledBrightnessUp = true;
                    }
                }
                setLedColor(0, ledBrightness, 0);
                break;
                
            case LED_BLUE_PULSE:
                // Faster pulse for updating
                if (ledBrightnessUp) {
                    ledBrightness += 10;
                    if (ledBrightness >= 255) {
                        ledBrightness = 255;
                        ledBrightnessUp = false;
                    }
                } else {
                    ledBrightness -= 10;
                    if (ledBrightness <= 30) {
                        ledBrightness = 30;
                        ledBrightnessUp = true;
                    }
                }
                setLedColor(0, 0, ledBrightness);
                break;
        }
    }
}

void setup() {
    // Initialize USB Serial for debugging
    Serial.begin(115200);
    delay(1000);
    Serial.println("\n\n=== ESP32-S3 Teensy FlasherX ===");
    Serial.println("Custom PCB Configuration:");
    Serial.println("  Flash UART: GPIO43(TX)/GPIO44(RX) -> Teensy Serial2");
    Serial.println("  Debug UART: GPIO17(TX)/GPIO18(RX) -> Teensy Serial3");
    
    // Initialize RGB LED
    #ifdef RGB_BUILTIN
    neopixelWrite(RGB_LED_PIN, 0, 0, 0);  // Off initially
    #else
    pinMode(RGB_LED_PIN, OUTPUT);
    #endif
    
    // Initialize Teensy Flash Serial (Serial2 on Teensy - for firmware upload)
    TEENSY_FLASH_SERIAL.begin(115200, SERIAL_8N1, TEENSY_FLASH_RX_PIN, TEENSY_FLASH_TX_PIN);
    
    // Initialize Teensy Debug Serial (Serial3 on Teensy - for monitoring)
    TEENSY_DEBUG_SERIAL.begin(115200, SERIAL_8N1, TEENSY_DEBUG_RX_PIN, TEENSY_DEBUG_TX_PIN);
    
    // Initialize control pins
    pinMode(TEENSY_RESET_PIN, OUTPUT);
    pinMode(TEENSY_PROGRAM_PIN, OUTPUT);
    digitalWrite(TEENSY_RESET_PIN, HIGH);
    digitalWrite(TEENSY_PROGRAM_PIN, HIGH);
    
    // Configure WiFi with static IP
    if (!WiFi.config(local_IP, gateway, subnet)) {
        Serial.println("WiFi config failed!");
    }
    
    WiFi.begin(ssid, password);
    Serial.print("Connecting to WiFi");
    
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 30) {
        delay(500);
        Serial.print(".");
        // Red while connecting
        setLedColor(255, 0, 0);
        attempts++;
    }
    
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\nWiFi connected!");
        Serial.print("IP Address: ");
        Serial.println(WiFi.localIP());
        setLedColor(0, 255, 0);  // Green when connected
    } else {
        Serial.println("\nWiFi connection failed!");
        setLedColor(255, 0, 0);  // Red if failed
    }
    
    // Setup web server routes
    server.on("/", HTTP_GET, handleRoot);
    server.on("/status", HTTP_GET, handleStatus);
    server.on("/teensy/upload", HTTP_POST, []() {
        server.send(200);
    }, handleTeensyUpload);
    server.on("/teensy/reset", HTTP_POST, handleTeensyReset);
    server.on("/esp/ota", HTTP_POST, []() {
        server.send(200);
    }, handleEsp32Ota);
    
    server.begin();
    Serial.println("HTTP server started on port 80");
    
    // Start WebSocket server
    webSocket.begin();
    webSocket.onEvent(webSocketEvent);
    Serial.println("WebSocket server started on port 81");
    
    logMessage("System initialized");
}

void loop() {
    server.handleClient();
    webSocket.loop();
    
    // Update RGB LED state
    updateLedState();
    
    // Forward Teensy debug serial to WebSocket (Serial3 on Teensy)
    while (TEENSY_DEBUG_SERIAL.available()) {
        char c = TEENSY_DEBUG_SERIAL.read();
        if (serialBufferIndex < SERIAL_BUFFER_SIZE - 1) {
            serialBuffer[serialBufferIndex++] = c;
            if (c == '\n' || serialBufferIndex >= SERIAL_BUFFER_SIZE - 1) {
                serialBuffer[serialBufferIndex] = '\0';
                
                // Check for heartbeat message
                if (strstr(serialBuffer, "heartbeat") != NULL) {
                    lastHeartbeat = millis();
                }
                
                broadcastSerial(serialBuffer);
                serialBufferIndex = 0;
            }
        }
    }
    
    // Periodic Teensy ping to check connection
    if (!uploadInProgress && millis() - lastTeensyPing > TEENSY_PING_INTERVAL) {
        if (pingTeensy()) {
            requestTeensyVersion();
        }
        lastTeensyPing = millis();
    }
    
    delay(1);
}

void handleRoot() {
    server.send_P(200, "text/html", index_html);
}

void handleStatus() {
    StaticJsonDocument<512> doc;
    doc["esp"] = true;
    doc["wifi"] = WiFi.status() == WL_CONNECTED;
    doc["ip"] = WiFi.localIP().toString();
    doc["teensy"] = teensyConnected;
    doc["uploading"] = uploadInProgress;
    doc["espVersion"] = ESP_FIRMWARE_VERSION;
    doc["espBuild"] = ESP_BUILD_DATE;
    doc["teensyVersion"] = teensyVersion;
    
    String response;
    serializeJson(doc, response);
    server.send(200, "application/json", response);
}

void handleTeensyReset() {
    logMessage("Resetting Teensy...");
    digitalWrite(TEENSY_RESET_PIN, LOW);
    delay(100);
    digitalWrite(TEENSY_RESET_PIN, HIGH);
    delay(500);
    server.send(200, "text/plain", "Teensy reset");
    logMessage("Teensy reset complete");
}

// FlasherX upload handler - sends Intel HEX to Teensy via FLASH_SERIAL
void handleTeensyUpload() {
    HTTPUpload& upload = server.upload();
    static String hexLine = "";
    static bool updateStarted = false;
    
    if (upload.status == UPLOAD_FILE_START) {
        uploadInProgress = true;
        teensyUpdating = true;  // Set updating flag for LED
        uploadedBytes = 0;
        totalBytes = 0;
        uploadError = "";
        hexLine = "";
        updateStarted = false;
        g_hexLineCount = 0;  // Reset global counter
        
        logMessage(("Receiving: " + upload.filename).c_str());
        
        // Send update command to Teensy
        TEENSY_FLASH_SERIAL.write(CMD_UPDATE);
        
        // Wait for ACK
        unsigned long start = millis();
        while (millis() - start < 2000) {
            if (TEENSY_FLASH_SERIAL.available()) {
                uint8_t response = TEENSY_FLASH_SERIAL.read();
                if (response == FLASHERX_ACK) {
                    logMessage("Teensy ready for HEX upload");
                    updateStarted = true;
                    delay(500); // Give Teensy time to enter update mode
                    break;
                }
            }
        }
        
        if (!updateStarted) {
            uploadError = "Teensy not responding to update command";
            logMessage(uploadError.c_str());
        }
        
    } else if (upload.status == UPLOAD_FILE_WRITE) {
        if (uploadError.length() == 0 && updateStarted) {
            // Process hex file data - forward to Teensy line by line
            for (size_t i = 0; i < upload.currentSize; i++) {
                char c = upload.buf[i];
                
                // Treat both \r and \n as line endings
                if (c == '\n' || c == '\r') {
                    if (hexLine.length() > 0) {
                        // Send hex line to Teensy
                        TEENSY_FLASH_SERIAL.print(hexLine);
                        TEENSY_FLASH_SERIAL.write('\n');
                        uploadedBytes += hexLine.length();
                        g_hexLineCount++;
                        
                        // Delay to let Teensy process each line
                        delay(1);
                        hexLine = "";
                    }
                } else {
                    hexLine += c;
                }
            }
            
            totalBytes = upload.totalSize;
            
            // Update LED during upload (loop() not running frequently)
            updateLedState();
            
            // Send progress update
            int progress = (uploadedBytes * 100) / (totalBytes > 0 ? totalBytes : 1);
            StaticJsonDocument<128> doc;
            doc["type"] = "progress";
            doc["target"] = "teensy";
            doc["percent"] = min(progress, 99);
            String msg;
            serializeJson(doc, msg);
            webSocket.broadcastTXT(msg);
        }
        
    } else if (upload.status == UPLOAD_FILE_END) {
        // Send any remaining hex line
        if (uploadError.length() == 0 && hexLine.length() > 0) {
            TEENSY_FLASH_SERIAL.print(hexLine);
            TEENSY_FLASH_SERIAL.write('\n');
            g_hexLineCount++;
        }
        hexLine = "";
        
        if (uploadError.length() == 0) {
            // Flush serial to ensure all data is sent
            TEENSY_FLASH_SERIAL.flush();
            
            logMessage(("HEX file sent: " + String(g_hexLineCount) + " lines").c_str());
            
            // Send 100% progress
            StaticJsonDocument<128> doc;
            doc["type"] = "progress";
            doc["target"] = "teensy";
            doc["percent"] = 100;
            String msg;
            serializeJson(doc, msg);
            webSocket.broadcastTXT(msg);
            
            // Wait for Teensy to finish processing all hex lines
            delay(5000);
            
            // Send the exact line count to confirm flash (FlasherX requires user confirmation)
            char confirmBuf[16];
            snprintf(confirmBuf, sizeof(confirmBuf), "%d", g_hexLineCount);
            TEENSY_FLASH_SERIAL.println(confirmBuf);
            TEENSY_FLASH_SERIAL.flush();
            
            logMessage(("Sent confirmation: " + String(g_hexLineCount) + " lines").c_str());
            logMessage("Teensy flash command sent - check Teensy serial for status");
            
            // Send success via WebSocket before HTTP response
            StaticJsonDocument<128> successDoc;
            successDoc["type"] = "teensy_complete";
            successDoc["success"] = true;
            String successMsg;
            serializeJson(successDoc, successMsg);
            webSocket.broadcastTXT(successMsg);
            webSocket.loop();
            
            server.send(200, "text/plain", "HEX upload complete");
        } else {
            server.send(500, "text/plain", uploadError);
            logMessage(uploadError.c_str());
        }
        
        uploadInProgress = false;
        teensyUpdating = false;  // Clear updating flag
        
    } else if (upload.status == UPLOAD_FILE_ABORTED) {
        uploadInProgress = false;
        teensyUpdating = false;  // Clear updating flag
        uploadError = "Upload aborted";
        logMessage(uploadError.c_str());
    }
}

// ESP32 OTA update handler
void handleEsp32Ota() {
    HTTPUpload& upload = server.upload();
    
    if (upload.status == UPLOAD_FILE_START) {
        espUpdating = true;  // Set updating flag for LED
        logMessage(("ESP32 OTA: " + upload.filename).c_str());
        
        if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {
            uploadError = "OTA begin failed";
            logMessage(uploadError.c_str());
            espUpdating = false;
        }
        
    } else if (upload.status == UPLOAD_FILE_WRITE) {
        if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
            uploadError = "OTA write failed";
            logMessage(uploadError.c_str());
        }
        
        // Update LED during upload (loop() not running)
        updateLedState();
        
        // Send progress
        int progress = (upload.totalSize > 0) ? (upload.currentSize * 100 / upload.totalSize) : 0;
        StaticJsonDocument<128> doc;
        doc["type"] = "progress";
        doc["target"] = "esp";
        doc["percent"] = progress;
        String msg;
        serializeJson(doc, msg);
        webSocket.broadcastTXT(msg);
        
    } else if (upload.status == UPLOAD_FILE_END) {
        if (Update.end(true)) {
            logMessage("ESP32 OTA successful, rebooting...");
            
            // Send success via WebSocket before HTTP response
            StaticJsonDocument<128> doc;
            doc["type"] = "ota_complete";
            doc["success"] = true;
            String msg;
            serializeJson(doc, msg);
            webSocket.broadcastTXT(msg);
            webSocket.loop();
            
            // Send HTTP response
            server.send(200, "text/plain", "OTA complete - rebooting");
            server.client().flush();
            delay(500);
            
            // Give browser time to receive response
            for (int i = 0; i < 10; i++) {
                server.handleClient();
                webSocket.loop();
                delay(100);
            }
            
            ESP.restart();
        } else {
            uploadError = "OTA end failed";
            espUpdating = false;
            server.send(500, "text/plain", uploadError);
            logMessage(uploadError.c_str());
        }
    }
}

// FlasherX protocol implementation - uses TEENSY_FLASH_SERIAL
bool flasherXSync() {
    logMessage("FlasherX: Syncing via Serial2...");
    
    // Clear any pending data
    while (TEENSY_FLASH_SERIAL.available()) {
        TEENSY_FLASH_SERIAL.read();
    }
    
    // Send sync bytes
    for (int attempt = 0; attempt < 10; attempt++) {
        TEENSY_FLASH_SERIAL.write(FLASHERX_SYNC);
        TEENSY_FLASH_SERIAL.write(FLASHERX_SYNC);
        TEENSY_FLASH_SERIAL.write(FLASHERX_SYNC);
        
        unsigned long start = millis();
        while (millis() - start < 500) {
            if (TEENSY_FLASH_SERIAL.available()) {
                uint8_t response = TEENSY_FLASH_SERIAL.read();
                if (response == FLASHERX_ACK) {
                    logMessage("FlasherX: Sync OK");
                    teensyConnected = true;
                    return true;
                }
            }
        }
    }
    
    teensyConnected = false;
    return false;
}

bool flasherXSendBlock(uint8_t* data, size_t len, uint32_t address) {
    // Send block header
    TEENSY_FLASH_SERIAL.write(0x01);  // Block command
    TEENSY_FLASH_SERIAL.write((address >> 24) & 0xFF);
    TEENSY_FLASH_SERIAL.write((address >> 16) & 0xFF);
    TEENSY_FLASH_SERIAL.write((address >> 8) & 0xFF);
    TEENSY_FLASH_SERIAL.write(address & 0xFF);
    TEENSY_FLASH_SERIAL.write((len >> 8) & 0xFF);
    TEENSY_FLASH_SERIAL.write(len & 0xFF);
    
    // Calculate checksum and send data
    uint8_t checksum = 0;
    for (size_t i = 0; i < len; i++) {
        TEENSY_FLASH_SERIAL.write(data[i]);
        checksum += data[i];
    }
    TEENSY_FLASH_SERIAL.write(checksum);
    
    // Wait for ACK
    unsigned long start = millis();
    while (millis() - start < FLASHERX_TIMEOUT_MS) {
        if (TEENSY_FLASH_SERIAL.available()) {
            uint8_t response = TEENSY_FLASH_SERIAL.read();
            return (response == FLASHERX_ACK);
        }
    }
    
    return false;
}

bool flasherXFinish() {
    logMessage("FlasherX: Finishing...");
    
    // Send finish command
    TEENSY_FLASH_SERIAL.write(0x02);  // Finish command
    
    // Wait for ACK
    unsigned long start = millis();
    while (millis() - start < FLASHERX_TIMEOUT_MS) {
        if (TEENSY_FLASH_SERIAL.available()) {
            uint8_t response = TEENSY_FLASH_SERIAL.read();
            if (response == FLASHERX_ACK) {
                logMessage("FlasherX: Complete");
                return true;
            }
        }
    }
    
    return false;
}

void webSocketEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t length) {
    switch (type) {
        case WStype_CONNECTED:
            Serial.printf("WebSocket client %u connected\n", num);
            break;
        case WStype_DISCONNECTED:
            Serial.printf("WebSocket client %u disconnected\n", num);
            break;
        case WStype_TEXT:
            // Handle incoming commands from web interface
            break;
    }
}

void broadcastSerial(const char* msg) {
    StaticJsonDocument<512> doc;
    doc["type"] = "serial";
    doc["source"] = "TEENSY";
    doc["message"] = msg;
    
    String jsonMsg;
    serializeJson(doc, jsonMsg);
    webSocket.broadcastTXT(jsonMsg);
    
    Serial.print("[TEENSY] ");
    Serial.print(msg);
}

void logMessage(const char* msg) {
    StaticJsonDocument<512> doc;
    doc["type"] = "serial";
    doc["source"] = "ESP";
    doc["message"] = msg;
    
    String jsonMsg;
    serializeJson(doc, jsonMsg);
    webSocket.broadcastTXT(jsonMsg);
    
    Serial.print("[ESP] ");
    Serial.println(msg);
}

// Ping Teensy to check if it's connected
bool pingTeensy() {
    // Clear any pending data
    while (TEENSY_FLASH_SERIAL.available()) {
        TEENSY_FLASH_SERIAL.read();
    }
    
    // Send ping command
    TEENSY_FLASH_SERIAL.write(CMD_PING);
    
    // Wait for ACK
    unsigned long start = millis();
    while (millis() - start < 100) {
        if (TEENSY_FLASH_SERIAL.available()) {
            uint8_t response = TEENSY_FLASH_SERIAL.read();
            if (response == FLASHERX_ACK) {
                teensyConnected = true;
                return true;
            }
        }
    }
    
    teensyConnected = false;
    return false;
}

// Request version string from Teensy
void requestTeensyVersion() {
    // Clear any pending data
    while (TEENSY_FLASH_SERIAL.available()) {
        TEENSY_FLASH_SERIAL.read();
    }
    
    // Send version request command
    TEENSY_FLASH_SERIAL.write(CMD_VERSION);
    
    // Wait for version string (format: "VER:x.x.x\n")
    String versionStr = "";
    unsigned long start = millis();
    while (millis() - start < 200) {
        if (TEENSY_FLASH_SERIAL.available()) {
            char c = TEENSY_FLASH_SERIAL.read();
            if (c == '\n') break;
            versionStr += c;
        }
    }
    
    // Parse version if received
    if (versionStr.startsWith("VER:")) {
        teensyVersion = versionStr.substring(4);
        teensyConnected = true;
    }
}
