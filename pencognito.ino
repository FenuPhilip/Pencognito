#include "esp_camera.h"
#include <WiFi.h>
#include "esp_http_server.h"

// ==== WiFi credentials ====
const char* ssid     = "project";
const char* password = "12345689";

// ==== L298N #1 - drive motors ====
#define IN1 12
#define IN2 13
#define IN3 15
#define IN4 14

// ==== L298N #2 - slash motor (wired to its OUT3/OUT4) ====
#define SLASH_IN_A 2
#define SLASH_IN_B 4
#define SLASH_BURST_MS 400   // how long the slash motor runs per trigger - tune to taste

// ==== AI-Thinker ESP32-CAM pin map ====
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

httpd_handle_t control_httpd = NULL;
httpd_handle_t stream_httpd  = NULL;

// ---------------- Drive motor helpers ----------------
void motorsStop()    { digitalWrite(IN1, LOW);  digitalWrite(IN2, LOW);  digitalWrite(IN3, LOW);  digitalWrite(IN4, LOW); }
void motorsForward()  { digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW); }
void motorsReverse()  { digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH); digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH); }
void motorsLeft()     { digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH); digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW); }
void motorsRight()    { digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);  digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH); }

// ---------------- Slash motor helpers ----------------
// One-shot burst: run for SLASH_BURST_MS then stop, all within the handler.
// If it fires the wrong way, swap the motor's two leads on OUT3/OUT4 rather than this code.
void slashStop() { digitalWrite(SLASH_IN_A, LOW); digitalWrite(SLASH_IN_B, LOW); }
void slashFire() {
  digitalWrite(SLASH_IN_A, HIGH);
  digitalWrite(SLASH_IN_B, LOW);
  delay(SLASH_BURST_MS);
  slashStop();
}

// ---------------- Web remote control page ----------------
const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Pencognito</title>
<style>
body{background:#14171a;color:#e8e6e1;font-family:sans-serif;text-align:center;margin:0;padding:16px}
img{width:100%;max-width:400px;border:1px solid #2c3136;border-radius:4px;background:#000}
.dpad{display:grid;grid-template-columns:repeat(3,80px);grid-template-rows:repeat(3,60px);
gap:8px;justify-content:center;margin-top:16px}
button{border:1px solid #2c3136;background:#23272b;color:#e8e6e1;border-radius:4px;font-size:1.4rem}
button:active{background:#7a3d10;border-color:#ff7a1a}
.stop{background:#d94b3f;border-color:#d94b3f;font-weight:bold}
.slash{margin-top:12px;background:#5b2fb5;border-color:#5b2fb5;color:#fff;font-weight:bold;
padding:10px 24px;border-radius:4px;font-size:1rem}
</style></head><body>
<h2>pen<span style="color:#ff7a1a">cognito</span></h2>
<img src="STREAM_URL" id="stream">
<div class="dpad">
<div></div>
<button ontouchstart="cmd('forward')" ontouchend="cmd('stop')" onmousedown="cmd('forward')" onmouseup="cmd('stop')">&#9650;</button>
<div></div>
<button ontouchstart="cmd('left')" ontouchend="cmd('stop')" onmousedown="cmd('left')" onmouseup="cmd('stop')">&#9664;</button>
<button class="stop" onclick="cmd('stop')">&#9632;</button>
<button ontouchstart="cmd('right')" ontouchend="cmd('stop')" onmousedown="cmd('right')" onmouseup="cmd('stop')">&#9654;</button>
<div></div>
<button ontouchstart="cmd('reverse')" ontouchend="cmd('stop')" onmousedown="cmd('reverse')" onmouseup="cmd('stop')">&#9660;</button>
<div></div>
</div>
<button class="slash" onclick="cmd('slash')">SLASH</button>
<script>function cmd(c){fetch('/'+c).catch(()=>{});}</script>
</body></html>
)rawliteral";

// ---------------- Control handlers ----------------
static esp_err_t index_handler(httpd_req_t *req) {
  String page = String(INDEX_HTML);
  String streamUrl = "http://" + WiFi.localIP().toString() + ":81/stream";
  page.replace("STREAM_URL", streamUrl);
  httpd_resp_set_type(req, "text/html");
  return httpd_resp_send(req, page.c_str(), page.length());
}
static esp_err_t forward_handler(httpd_req_t *req) { motorsForward(); return httpd_resp_send(req, "ok", 2); }
static esp_err_t reverse_handler(httpd_req_t *req) { motorsReverse(); return httpd_resp_send(req, "ok", 2); }
static esp_err_t left_handler(httpd_req_t *req)    { motorsLeft();    return httpd_resp_send(req, "ok", 2); }
static esp_err_t right_handler(httpd_req_t *req)   { motorsRight();   return httpd_resp_send(req, "ok", 2); }
static esp_err_t stop_handler(httpd_req_t *req)    { motorsStop();    return httpd_resp_send(req, "ok", 2); }
static esp_err_t slash_handler(httpd_req_t *req)   { slashFire();     return httpd_resp_send(req, "ok", 2); }

// ---------------- Video stream handler ----------------
#define PART_BOUNDARY "123456789000000000000987654321"
static const char* STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char* STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

static esp_err_t stream_handler(httpd_req_t *req) {
  camera_fb_t *fb = NULL;
  esp_err_t res = httpd_resp_set_type(req, STREAM_CONTENT_TYPE);
  if (res != ESP_OK) return res;

  char part_buf[64];
  while (true) {
    fb = esp_camera_fb_get();
    if (!fb) { res = ESP_FAIL; break; }

    res = httpd_resp_send_chunk(req, STREAM_BOUNDARY, strlen(STREAM_BOUNDARY));
    if (res == ESP_OK) {
      size_t hlen = snprintf(part_buf, sizeof(part_buf), STREAM_PART, fb->len);
      res = httpd_resp_send_chunk(req, part_buf, hlen);
    }
    if (res == ESP_OK) res = httpd_resp_send_chunk(req, (const char *)fb->buf, fb->len);

    esp_camera_fb_return(fb);
    if (res != ESP_OK) break;
  }
  return res;
}

// ---------------- Server startup ----------------
void startControlServer() {
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = 80;
  config.max_uri_handlers = 8; // was headroom for 6; slash brings us to 7, bump it up

  httpd_uri_t index_uri   = {"/",         HTTP_GET, index_handler,   NULL};
  httpd_uri_t forward_uri = {"/forward",  HTTP_GET, forward_handler, NULL};
  httpd_uri_t reverse_uri = {"/reverse",  HTTP_GET, reverse_handler, NULL};
  httpd_uri_t left_uri    = {"/left",     HTTP_GET, left_handler,    NULL};
  httpd_uri_t right_uri   = {"/right",    HTTP_GET, right_handler,   NULL};
  httpd_uri_t stop_uri    = {"/stop",     HTTP_GET, stop_handler,    NULL};
  httpd_uri_t slash_uri   = {"/slash",    HTTP_GET, slash_handler,   NULL};

  if (httpd_start(&control_httpd, &config) == ESP_OK) {
    httpd_register_uri_handler(control_httpd, &index_uri);
    httpd_register_uri_handler(control_httpd, &forward_uri);
    httpd_register_uri_handler(control_httpd, &reverse_uri);
    httpd_register_uri_handler(control_httpd, &left_uri);
    httpd_register_uri_handler(control_httpd, &right_uri);
    httpd_register_uri_handler(control_httpd, &stop_uri);
    httpd_register_uri_handler(control_httpd, &slash_uri);
  }
}

void startStreamServer() {
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = 81;
  config.ctrl_port = 32769; // must differ from the control server's ctrl_port (32768 default)

  httpd_uri_t stream_uri = {"/stream", HTTP_GET, stream_handler, NULL};

  if (httpd_start(&stream_httpd, &config) == ESP_OK) {
    httpd_register_uri_handler(stream_httpd, &stream_uri);
  }
}

// ---------------- Setup / loop ----------------
void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(false);

  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  motorsStop();

  pinMode(SLASH_IN_A, OUTPUT); pinMode(SLASH_IN_B, OUTPUT);
  slashStop();

  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM; config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM; config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM; config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM; config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk    = XCLK_GPIO_NUM;
  config.pin_pclk    = PCLK_GPIO_NUM;
  config.pin_vsync   = VSYNC_GPIO_NUM;
  config.pin_href    = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn    = PWDN_GPIO_NUM;
  config.pin_reset   = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  if (psramFound()) {
    config.frame_size   = FRAMESIZE_VGA;   // 640x480, PSRAM can afford it
    config.jpeg_quality = 12;
    config.fb_count      = 2;
    config.fb_location   = CAMERA_FB_IN_PSRAM;
  } else {
    config.frame_size   = FRAMESIZE_QVGA;  // 320x240, safer without PSRAM
    config.jpeg_quality = 15;
    config.fb_count      = 1;
    config.fb_location   = CAMERA_FB_IN_DRAM;
  }

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x\n", err);
    return;
  }

  // Optional: force a fixed IP instead of relying on DHCP handing out
  // the same address every time. Adjust these to match your hotspot's
  // subnet (10.44.41.x based on what you saw on the Serial Monitor) -
  // pick a .xxx number your phone's DHCP pool won't also hand out.
  IPAddress local_IP(10, 44, 41, 200);
  IPAddress gateway(10, 44, 41, 1);
  IPAddress subnet(255, 255, 255, 0);
  WiFi.config(local_IP, gateway, subnet);

  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.print("Pencognito online. Open in a browser: http://");
  Serial.println(WiFi.localIP());
  Serial.println("(Video stream alone is at that same address on port 81/stream)");

  startControlServer();
  startStreamServer();
}

void loop() {
  delay(10000); // everything happens in the HTTP server callbacks
}
