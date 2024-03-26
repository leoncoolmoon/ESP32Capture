// Copyright 2015-2016 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <Arduino.h> // Include the necessary header file
#include "esp_timer.h"
#include "esp_camera.h"
#include "img_converters.h"
#include "esp32-hal-ledc.h"
#include "sdkconfig.h"
#include "camera_index.h"
#include <WebServer.h>
#include <Update.h>
#include "FS.h"
#include "SPIFFS.h"

#define FORMAT_SPIFFS_IF_FAILED true

// Enable LED FLASH setting
#define CONFIG_LED_ILLUMINATOR_ENABLED 1

// LED FLASH setup
#if CONFIG_LED_ILLUMINATOR_ENABLED

#define LED_LEDC_CHANNEL 2 // Using different ledc channel/timer than camera
#define CONFIG_LED_MAX_INTENSITY 255

int led_duty = 0;
bool isStreaming = false;

#endif

WebServer serverCamera(80); // 相机服务器监听80端口
WebServer serverStream(81); // 流服务器监听81端口

#define PART_BOUNDARY "123456789000000000000987654321"
static const char *_STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char *_STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char *_STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\nX-Timestamp: %d.%06d\r\n\r\n";

typedef struct
{
  WiFiClient client;
  size_t len;
} jpg_chunking_t;

#if CONFIG_LED_ILLUMINATOR_ENABLED
void enable_led(bool en)
{ // Turn LED On or Off
  int duty = en ? led_duty : 0;
  if (en && isStreaming && (led_duty > CONFIG_LED_MAX_INTENSITY))
  {
    duty = CONFIG_LED_MAX_INTENSITY;
  }
  ledcWrite(LED_LEDC_CHANNEL, duty);
  Serial.printf("Set LED intensity to %d", duty);
}
#endif

static void bmp_handler()
{
  camera_fb_t *fb = NULL;
  esp_err_t res = ESP_OK;
#if ARDUHAL_LOG_LEVEL >= ARDUHAL_LOG_LEVEL_INFO
  uint64_t fr_start = esp_timer_get_time();
#endif
  fb = esp_camera_fb_get();
  if (!fb)
  {
    Serial.println("Camera capture failed");
    serverCamera.send(500, "text/plain", "Camera capture failed");
    return;
  }
  serverCamera.sendHeader("Content-Type", "image/x-windows-bmp"); // 设置响应类型为bmp
  serverCamera.sendHeader("Content-Disposition", "inline; filename=capture.bmp");
  serverCamera.sendHeader("Access-Control-Allow-Origin", "*");
  char ts[32];
  snprintf(ts, 32, "%ld.%06ld", fb->timestamp.tv_sec, fb->timestamp.tv_usec);
  serverCamera.sendHeader("X-Timestamp", (const char *)ts);
  uint8_t *buf = NULL;
  size_t buf_len = 0;
  bool converted = frame2bmp(fb, &buf, &buf_len);
  esp_camera_fb_return(fb);
  if (!converted)
  {
    Serial.println("BMP Conversion failed");
    serverCamera.send(500, "text/plain", "BMP Conversion failed");
    return;
  }
  serverCamera.sendHeader("Content-Length", String(buf_len));
  serverCamera.send(200, "image/x-windows-bmp", (const char *)buf);
  free(buf);
#if ARDUHAL_LOG_LEVEL >= ARDUHAL_LOG_LEVEL_INFO
  uint64_t fr_end = esp_timer_get_time();
#endif
  Serial.printf("BMP: %uB", buf_len);
}
static void xclk_handler()
{
  if (!serverCamera.hasArg("xclk"))
  {
    serverCamera.send(404, "text/plain", "Not Found");
    return;
  }
  int xclk = serverCamera.arg("xclk").toInt();
  Serial.printf("Set XCLK: %d MHz\n", xclk);
  sensor_t *s = esp_camera_sensor_get();
  int res = s->set_xclk(s, LEDC_TIMER_0, xclk);
  if (res)
  {
    serverCamera.send(500, "text/plain", "Internal Server Error");
    return;
  }
  serverCamera.sendHeader("Access-Control-Allow-Origin", "*");
  serverCamera.send(200, "text/plain", "");
}

static void reg_handler()
{
  if (!serverCamera.hasArg("reg") || !serverCamera.hasArg("mask") || !serverCamera.hasArg("val"))
  {
    serverCamera.send(404, "text/plain", "Not Found");
    return;
  }

  int reg = serverCamera.arg("reg").toInt();
  int mask = serverCamera.arg("mask").toInt();
  int val = serverCamera.arg("val").toInt();
  Serial.printf("Set Register: reg: 0x%02x, mask: 0x%02x, value: 0x%02x\n", reg, mask, val);

  sensor_t *s = esp_camera_sensor_get();
  int res = s->set_reg(s, reg, mask, val);
  if (res)
  {
    serverCamera.send(500, "text/plain", "Internal Server Error");
    return;
  }

  serverCamera.sendHeader("Access-Control-Allow-Origin", "*");
  serverCamera.send(200, "text/plain", "");
}

static void greg_handler()
{
  if (!serverCamera.hasArg("reg") || !serverCamera.hasArg("mask"))
  {
    serverCamera.send(404, "text/plain", "Not Found");
    return;
  }

  int reg = serverCamera.arg("reg").toInt();
  int mask = serverCamera.arg("mask").toInt();

  sensor_t *s = esp_camera_sensor_get();
  int res = s->get_reg(s, reg, mask);
  if (res < 0)
  {
    serverCamera.send(500, "text/plain", "Internal Server Error");
    return;
  }
  Serial.printf("Get Register: reg: 0x%02x, mask: 0x%02x, value: 0x%02x\n", reg, mask, res);
  serverCamera.sendHeader("Access-Control-Allow-Origin", "*");
  serverCamera.send(200, "text/plain", String(res));
}

static int parse_get_var(const String &arg, int def)
{
  if (!serverCamera.hasArg(arg))
  {
    return def;
  }
  return serverCamera.arg(arg).toInt();
}

static void pll_handler()
{
  int bypass = parse_get_var("bypass", 0);
  int mul = parse_get_var("mul", 0);
  int sys = parse_get_var("sys", 0);
  int root = parse_get_var("root", 0);
  int pre = parse_get_var("pre", 0);
  int seld5 = parse_get_var("seld5", 0);
  int pclken = parse_get_var("pclken", 0);
  int pclk = parse_get_var("pclk", 0);
  Serial.printf("Set Pll: bypass: %d, mul: %d, sys: %d, root: %d, pre: %d, seld5: %d, pclken: %d, pclk: %d\n", bypass, mul, sys, root, pre, seld5, pclken, pclk);
  sensor_t *s = esp_camera_sensor_get();
  int res = s->set_pll(s, bypass, mul, sys, root, pre, seld5, pclken, pclk);
  if (res)
  {
    serverCamera.send(500, "text/plain", "Internal Server Error");
    return;
  }
  serverCamera.sendHeader("Access-Control-Allow-Origin", "*");
  serverCamera.send(200, "text/plain", "");
}
void win_handler()
{
  int startX = parse_get_var("sx", 0);
  int startY = parse_get_var("sy", 0);
  int endX = parse_get_var("ex", 0);
  int endY = parse_get_var("ey", 0);
  int offsetX = parse_get_var("offx", 0);
  int offsetY = parse_get_var("offy", 0);
  int totalX = parse_get_var("tx", 0);
  int totalY = parse_get_var("ty", 0);
  int outputX = parse_get_var("ox", 0);
  int outputY = parse_get_var("oy", 0);
  bool scale = parse_get_var("scale", 0) == 1;
  bool binning = parse_get_var("binning", 0) == 1;
  Serial.printf("Set Window: Start: %d %d, End: %d %d, Offset: %d %d, Total: %d %d, Output: %d %d, Scale: %u, Binning: %u\n", startX, startY, endX, endY, offsetX, offsetY, totalX, totalY, outputX, outputY, scale, binning);
  sensor_t *s = esp_camera_sensor_get();
  int res = s->set_res_raw(s, startX, startY, endX, endY, offsetX, offsetY, totalX, totalY, outputX, outputY, scale, binning);
  if (res)
  {
    serverCamera.send(500, "text/plain", "Internal Server Error");
    return;
  }
  serverCamera.sendHeader("Access-Control-Allow-Origin", "*");
  serverCamera.send(200, "text/plain", "");
}
static int print_reg(char *p, sensor_t *s, uint16_t reg, uint32_t mask)
{
  return sprintf(p, "\"0x%x\":%u,", reg, s->get_reg(s, reg, mask));
}


void loadFromCamera( char *json_response)
{
  //static char json_response[1024];
  sensor_t *s = esp_camera_sensor_get();
  char *p = json_response;
  *p++ = '{';
  if (s->id.PID == OV5640_PID || s->id.PID == OV3660_PID)
  {
    for (int reg = 0x3400; reg < 0x3406; reg += 2)
    {
      p += print_reg(p, s, reg, 0xFFF); // 12 bit
    }
    p += print_reg(p, s, 0x3406, 0xFF);
    p += print_reg(p, s, 0x3500, 0xFFFF0); // 16 bit
    p += print_reg(p, s, 0x3503, 0xFF);
    p += print_reg(p, s, 0x350a, 0x3FF);  // 10 bit
    p += print_reg(p, s, 0x350c, 0xFFFF); // 16 bit
    for (int reg = 0x5480; reg <= 0x5490; reg++)
    {
      p += print_reg(p, s, reg, 0xFF);
    }
    for (int reg = 0x5380; reg <= 0x538b; reg++)
    {
      p += print_reg(p, s, reg, 0xFF);
    }
    for (int reg = 0x5580; reg < 0x558a; reg++)
    {
      p += print_reg(p, s, reg, 0xFF);
    }
    p += print_reg(p, s, 0x558a, 0x1FF); // 9 bit
  }
  else if (s->id.PID == OV2640_PID)
  {
    p += print_reg(p, s, 0xd3, 0xFF);
    p += print_reg(p, s, 0x111, 0xFF);
    p += print_reg(p, s, 0x132, 0xFF);
  }
  p += sprintf(p, "\"xclk\":%u,", s->xclk_freq_hz / 1000000);
  p += sprintf(p, "\"pixformat\":%u,", s->pixformat);
  p += sprintf(p, "\"framesize\":%u,", s->status.framesize);
  p += sprintf(p, "\"quality\":%u,", s->status.quality);
  p += sprintf(p, "\"brightness\":%d,", s->status.brightness);
  p += sprintf(p, "\"contrast\":%d,", s->status.contrast);
  p += sprintf(p, "\"saturation\":%d,", s->status.saturation);
  p += sprintf(p, "\"sharpness\":%d,", s->status.sharpness);
  p += sprintf(p, "\"special_effect\":%u,", s->status.special_effect);
  p += sprintf(p, "\"wb_mode\":%u,", s->status.wb_mode);
  p += sprintf(p, "\"awb\":%u,", s->status.awb);
  p += sprintf(p, "\"awb_gain\":%u,", s->status.awb_gain);
  p += sprintf(p, "\"aec\":%u,", s->status.aec);
  p += sprintf(p, "\"aec2\":%u,", s->status.aec2);
  p += sprintf(p, "\"ae_level\":%d,", s->status.ae_level);
  p += sprintf(p, "\"aec_value\":%u,", s->status.aec_value);
  p += sprintf(p, "\"agc\":%u,", s->status.agc);
  p += sprintf(p, "\"agc_gain\":%u,", s->status.agc_gain);
  p += sprintf(p, "\"gainceiling\":%u,", s->status.gainceiling);
  p += sprintf(p, "\"bpc\":%u,", s->status.bpc);
  p += sprintf(p, "\"wpc\":%u,", s->status.wpc);
  p += sprintf(p, "\"raw_gma\":%u,", s->status.raw_gma);
  p += sprintf(p, "\"lenc\":%u,", s->status.lenc);
  p += sprintf(p, "\"hmirror\":%u,", s->status.hmirror);
  p += sprintf(p, "\"dcw\":%u,", s->status.dcw);
  p += sprintf(p, "\"colorbar\":%u", s->status.colorbar);
#if CONFIG_LED_ILLUMINATOR_ENABLED
  p += sprintf(p, ",\"led_intensity\":%u", led_duty);
#else
  p += sprintf(p, ",\"led_intensity\":%d", -1);
#endif
  *p++ = '}';
  *p++ = 0;

  //return json_response;
}
int setupCam(String param,sensor_t *s)
{
  String variable = param.substring(0, param.indexOf(":"));
  param.replace(variable + ": ", "");
  int val = param.toInt();
  int res = 0;
  if (variable.equals("framesize"))
  {
    if (s->pixformat == PIXFORMAT_JPEG)
    {
      res = s->set_framesize(s, (framesize_t)val);
    }
  }
  else if (variable.equals("quality"))
    res = s->set_quality(s, val);
  else if (variable.equals("contrast"))
    res = s->set_contrast(s, val);
  else if (variable.equals("brightness"))
    res = s->set_brightness(s, val);
  else if (variable.equals("saturation"))
    res = s->set_saturation(s, val);
  else if (variable.equals("gainceiling"))
    res = s->set_gainceiling(s, (gainceiling_t)val);
  else if (variable.equals("colorbar"))
    res = s->set_colorbar(s, val);
  else if (variable.equals("awb"))
    res = s->set_whitebal(s, val);
  else if (variable.equals("agc"))
    res = s->set_gain_ctrl(s, val);
  else if (variable.equals("aec"))
    res = s->set_exposure_ctrl(s, val);
  else if (variable.equals("hmirror"))
    res = s->set_hmirror(s, val);
  else if (variable.equals("vflip"))
    res = s->set_vflip(s, val);
  else if (variable.equals("awb_gain"))
    res = s->set_awb_gain(s, val);
  else if (variable.equals("agc_gain"))
    res = s->set_agc_gain(s, val);
  else if (variable.equals("aec_value"))
    res = s->set_aec_value(s, val);
  else if (variable.equals("aec2"))
    res = s->set_aec2(s, val);
  else if (variable.equals("dcw"))
    res = s->set_dcw(s, val);
  else if (variable.equals("bpc"))
    res = s->set_bpc(s, val);
  else if (variable.equals("wpc"))
    res = s->set_wpc(s, val);
  else if (variable.equals("raw_gma"))
    res = s->set_raw_gma(s, val);
  else if (variable.equals("lenc"))
    res = s->set_lenc(s, val);
  else if (variable.equals("special_effect"))
    res = s->set_special_effect(s, val);
  else if (variable.equals("wb_mode"))
    res = s->set_wb_mode(s, val);
  else if (variable.equals("ae_level"))
    res = s->set_ae_level(s, val);
  else
  {
    Serial.printf("Unknown command: %s", variable);
    res = -1;
  }
  
 return res;
}
int setupCams(String params){
params.replace("{", "");
params.replace("}", "");
//line by line trim and sent to setupCam
int start = 0;
int end = 0;
int res = 0;
sensor_t *s = esp_camera_sensor_get();
while (end < params.length())
{
  end = params.indexOf("\n", start);
  String line = params.substring(start, end);
  line.trim();
  if (line.length() > 0)
  {
    if (setupCam(line,s) < 0)
    {
      res = -1;
    }
  }
  start = end + 1;

}
return res;
}
void status_handler()
{//read SPIFFS
  File file = SPIFFS.open("/params.json", "r");
  if (!file)
  {
    Serial.println("There was an error opening the file for reading load params from camera");
    static char json_response[1024];
    loadFromCamera(json_response);
    serverCamera.sendHeader("Access-Control-Allow-Origin", "*");
    serverCamera.send(200, "application/json", json_response);
    return;
  }
 
  String params = file.readString();
  file.close();
//setup camera based on the params
  if (setupCams(params) < 0)
  {
    serverCamera.send(500, "application/json", "{\"params\":\"Internal Server Error\"}");
    return;
  }
  serverCamera.sendHeader("Access-Control-Allow-Origin", "*");
  serverCamera.send(200, "application/json", params);
}
void cmd_handler(){
  String variable = serverCamera.arg("var");
  String value = serverCamera.arg("val");
  int val = value.toInt();
  Serial.printf("%s = %d\n", variable.c_str(), val);
  sensor_t *s = esp_camera_sensor_get();
  if (setupCam(  variable + "\": " + value,s) < 0)
  {
    serverCamera.send(500, "text/plain", "Internal Server Error");
    return;
  }
  serverCamera.sendHeader("Access-Control-Allow-Origin", "*");
  serverCamera.send(200, "text/plain", "");
}
size_t jpg_encode_stream(void *arg, size_t index, const void *data, size_t len)
{
  jpg_chunking_t *j = (jpg_chunking_t *)arg;
  if (!index)
  {
    j->len = 0;
  }
  j->client.write((uint8_t *)data, len);
  j->len += len;
  return len;
}
static void capture_handler()
{
  camera_fb_t *fb = NULL;
  esp_err_t res = ESP_OK;
  WiFiClient client = serverCamera.client();
#if CONFIG_LED_ILLUMINATOR_ENABLED
  enable_led(true);
  vTaskDelay(150 / portTICK_PERIOD_MS); // The LED needs to be turned on ~150ms before the call to esp_camera_fb_get()
  fb = esp_camera_fb_get();             // or it won't be visible in the frame. A better way to do this is needed.
  enable_led(false);
#else
  fb = esp_camera_fb_get();
#endif
  if (!fb)
  {
    Serial.println("Camera capture failed");
    serverCamera.send(500, "text/plain", "Camera capture failed");
    return;
  }
  serverCamera.sendHeader("Content-Type", "image/jpeg");
  serverCamera.sendHeader("Content-Disposition", "inline; filename=capture.jpg");
  serverCamera.sendHeader("Access-Control-Allow-Origin", "*");
  char ts[32];
  snprintf(ts, 32, "%ld.%06ld", fb->timestamp.tv_sec, fb->timestamp.tv_usec);
  serverCamera.sendHeader("X-Timestamp", (const char *)ts);
  if (fb->format == PIXFORMAT_JPEG)
  {
    serverCamera.sendHeader("Content-Length", String(fb->len));
    serverCamera.sendHeader("Content-Type", "image/jpeg");
    serverCamera.sendContent((const char *)fb->buf, fb->len);
  }
  else
  {
    serverCamera.sendHeader("Content-Type", "multipart/x-mixed-replace; boundary=123456789000000000000987654321");
    jpg_chunking_t jchunk = {client, 0};
    frame2jpg_cb(fb, 80, jpg_encode_stream, &jchunk);
  }
  esp_camera_fb_return(fb);
  // Serial.printf("JPG: %uB ", (uint32_t)(fb_len) );
}

static void stream_handler()
{
  Serial.println("stream");
  camera_fb_t *fb = NULL;
  struct timeval _timestamp;
  esp_err_t res = ESP_OK;
  size_t _jpg_buf_len = 0;
  uint8_t *_jpg_buf = NULL;
  char *part_buf[128];

  static int64_t last_frame = 0;
  WiFiClient client = serverStream.client();

  if (!last_frame)
  {
    last_frame = esp_timer_get_time();
  }
  if (!client.connected())
  {
    Serial.println("Client disconnected-1");
    res = ESP_FAIL;
  }
  // serverStream.sendHeader("Content-Type", _STREAM_CONTENT_TYPE);
  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type:" + String(_STREAM_CONTENT_TYPE));
  client.println("Access-Control-Allow-Origin: *");
  client.println("X-Framerate: 60");

  //   serverStream.send(200); // 发送HTTP响应头
  //   serverStream.sendHeader("Access-Control-Allow-Origin", "*");
  //   serverStream.sendHeader("X-Framerate", "60");
  if (!client.connected())
  {
    Serial.println("Client disconnected0");
    res = ESP_FAIL;
  }
#if CONFIG_LED_ILLUMINATOR_ENABLED
  isStreaming = true;
  enable_led(true);
#endif

  while (true)
  {
    fb = esp_camera_fb_get();
    if (!fb)
    {
      Serial.println("Camera capture failed");
      res = ESP_FAIL;
    }
    else
    {
      _timestamp.tv_sec = fb->timestamp.tv_sec;
      _timestamp.tv_usec = fb->timestamp.tv_usec;

      if (fb->format != PIXFORMAT_JPEG)
      {
        bool jpeg_converted = frame2jpg(fb, 80, &_jpg_buf, &_jpg_buf_len);
        esp_camera_fb_return(fb);
        fb = NULL;
        if (!jpeg_converted)
        {
          Serial.println("JPEG compression failed");
          res = ESP_FAIL;
        }
      }
      else
      {
        _jpg_buf_len = fb->len;
        _jpg_buf = fb->buf;
      }
    }
    if (res == ESP_OK)
    {
      if (!client.connected())
      {
        Serial.println("Client disconnected1");
        res = ESP_FAIL;
      }
      // serverStream.sendContent(_STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
      client.write(_STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
    }
    if (res == ESP_OK)
    {
      size_t hlen = snprintf((char *)part_buf, 128, _STREAM_PART, _jpg_buf_len, _timestamp.tv_sec, _timestamp.tv_usec);
      // serverCamera.sendContent((const char *)part_buf, hlen);
      client.write((const uint8_t *)part_buf, hlen);
    }
    if (res == ESP_OK)
    {
      if (!client.connected())
      {
        Serial.println("Client disconnected2");
        res = ESP_FAIL;
      }
      Serial.printf("sent buf %d\n", _jpg_buf_len);
      // serverStream.sendContent((const char *)_jpg_buf, _jpg_buf_len);
      client.write((const uint8_t *)_jpg_buf, _jpg_buf_len);
      // check if the client has disconnected
      if (!client.connected())
      {
        Serial.println("Client disconnected3");
        res = ESP_FAIL;
      }
      //      WiFiClient client = serverCamera.client();
      //      size_t sentBytes = client.write((const uint8_t *)_jpg_buf, _jpg_buf_len);
      //      Serial.printf("sent %d vs buf %d\n",sentBytes,_jpg_buf_len);
      //      if (sentBytes != _jpg_buf_len) {
      //        res = ESP_FAIL;
      //      }
    }
    if (fb)
    {
      esp_camera_fb_return(fb);
      fb = NULL;
      _jpg_buf = NULL;
    }
    else if (_jpg_buf)
    {
      free(_jpg_buf);
      _jpg_buf = NULL;
    }
    if (res != ESP_OK)
    {
      Serial.println("Send frame failed");
      break;
    }
    int64_t fr_end = esp_timer_get_time();

    int64_t frame_time = fr_end - last_frame;
    frame_time /= 1000;
    Serial.printf("MJPG: %uB %ums (%.1ffps)",
                  (uint32_t)(_jpg_buf_len),
                  (uint32_t)frame_time, 1000.0 / (uint32_t)frame_time);
  }
#if CONFIG_LED_ILLUMINATOR_ENABLED
  isStreaming = false;
  enable_led(false);
#endif
}

static void index_handler()
{
  sensor_t *s = esp_camera_sensor_get();
  if (s != NULL)
  {
    serverCamera.sendHeader("Content-Encoding", "gzip");
    serverCamera.sendHeader("Content-Type", "text/html");
    serverCamera.send_P(200, "text/html", (const char *)INDEX_OV2640_HTML_GZ, sizeof(INDEX_OV2640_HTML_GZ));
  }
  else
  {
    Serial.println("Camera sensor not found");
    serverCamera.send(500, "text/plain", "Camera sensor not found");
  }
}
void loopServer()
{
  serverCamera.handleClient();
  serverStream.handleClient();
}
void update_handler()
{
  serverCamera.sendHeader("Connection", "close");
  serverCamera.send(200, "text/plain", (Update.hasError()) ? "FAIL" : "OK");
  ESP.restart();
}
void upload_handler()
{
  HTTPUpload &upload = serverCamera.upload();
  if (upload.status == UPLOAD_FILE_START)
  {
    Serial.printf("Update: %s\n", upload.filename.c_str());
    if (!Update.begin(UPDATE_SIZE_UNKNOWN))
    { // start with max available size
      Update.printError(Serial);
    }
  }
  else if (upload.status == UPLOAD_FILE_WRITE)
  {
    /* flashing firmware to ESP*/
    if (Update.write(upload.buf, upload.currentSize) != upload.currentSize)
    {
      Update.printError(Serial);
    }
  }
  else if (upload.status == UPLOAD_FILE_END)
  {
    if (Update.end(true))
    { // true to set the size to the current progress
      Serial.printf("Update Success: %u\nRebooting...\n", upload.totalSize);
    }
    else
    {
      Update.printError(Serial);
    }
  }
}

void premeter_save_handler()
{
  // 接收请求体
  String params = serverCamera.arg("plain");
  Serial.println(params);
  // 除掉已有文件
  if (SPIFFS.exists("/params.json"))
  {
    SPIFFS.remove("/params.json");
  }
  // 将字符串写入到文件

  File file = SPIFFS.open("/params.json", "w");
  if (!file)
  {
    Serial.println("There was an error opening the file for writing");
    serverCamera.send(500, "application/json", "{\"params\":\"There was an error opening the file for writing\"}");
    return;
  }
  file.print(params); // 将接收到的字符串直接写入文件
  file.close();
  // 发送响应
  serverCamera.send(200, "application/json", "{\"params\":\"Parameters updated successfully\"}");
}

void startCameraServer()
{
  // Route for root / web page
  serverCamera.on("/", HTTP_GET, index_handler);
  serverCamera.on("/bmp", HTTP_GET, bmp_handler);
  serverCamera.on("/status", HTTP_GET, status_handler);
  serverCamera.on("/control", HTTP_GET, cmd_handler);
  serverCamera.on("/capture", HTTP_GET, capture_handler);
  serverCamera.on("/xclk", HTTP_GET, xclk_handler);
  serverCamera.on("/reg", HTTP_GET, reg_handler);
  serverCamera.on("/greg", HTTP_GET, greg_handler);
  serverCamera.on("/pll", HTTP_GET, pll_handler);
  serverCamera.on("/resolution", HTTP_GET, win_handler);
  serverCamera.on("/update", HTTP_POST, update_handler, upload_handler);
  serverCamera.on("/keep", HTTP_POST, premeter_save_handler);
  serverStream.on("/stream", HTTP_GET, stream_handler);
  Serial.println("Starting Web Server");
  serverCamera.begin();
  serverStream.begin();
  // init spiffs
  if (!SPIFFS.begin(FORMAT_SPIFFS_IF_FAILED))
  {
    Serial.println("An Error has occurred while mounting SPIFFS");
    return;
  }
}
void setupLedFlash(int pin)
{
#if CONFIG_LED_ILLUMINATOR_ENABLED
  ledcSetup(LED_LEDC_CHANNEL, 5000, 8);
  ledcAttachPin(pin, LED_LEDC_CHANNEL);
#else
  Serial.println("LED flash is disabled -> CONFIG_LED_ILLUMINATOR_ENABLED = 0");
#endif
}
