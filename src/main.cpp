#include <Arduino.h>
#include <HTTPClient.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <esp_camera.h>
#include <esp_heap_caps.h>
#include <img_converters.h>
#include <mbedtls/base64.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Preferences.h>

#include <math.h>
#include <string.h>
#include <time.h>

#if __has_include("device_config.local.h")
#include "device_config.local.h"
#else
#include "device_config.h"
#endif
#include "page_confirmation.h"

// Verified OV2640 pin mapping for this ESP32-S3 N16R8 board.
static constexpr int PIN_PWDN = -1;
static constexpr int PIN_RESET = -1;
static constexpr int PIN_XCLK = 15;
static constexpr int PIN_SIOD = 4;
static constexpr int PIN_SIOC = 5;
static constexpr int PIN_Y2 = 11;
static constexpr int PIN_Y3 = 9;
static constexpr int PIN_Y4 = 8;
static constexpr int PIN_Y5 = 10;
static constexpr int PIN_Y6 = 12;
static constexpr int PIN_Y7 = 18;
static constexpr int PIN_Y8 = 17;
static constexpr int PIN_Y9 = 16;
static constexpr int PIN_VSYNC = 6;
static constexpr int PIN_HREF = 7;
static constexpr int PIN_PCLK = 13;

// Run page-turn detection as soon as the previous frame has been processed.
// The loop still yields for 5 ms below, so this does not busy-spin.
static constexpr uint32_t MONITOR_INTERVAL_MS = 0;
// Content confirmation is time-based as well as multi-frame: camera shake
// must settle before accepting a different page.
static constexpr uint8_t STABLE_FRAMES_REQUIRED = 2;
static constexpr float MOTION_THRESHOLD = 0.12f;
static constexpr float STABLE_THRESHOLD = 0.12f;
static constexpr float PAGE_CHANGE_THRESHOLD = 0.30f;
// Minimum event cooldown is separate from continuous upload pacing.
static constexpr uint32_t EVENT_COOLDOWN_MS = 700;

static constexpr int MOTION_W = 32;
static constexpr int MOTION_H = 24;
static constexpr int PAGE_W = 16;
static constexpr int PAGE_H = 16;
static constexpr int FLAT_W = 800;
static constexpr int FLAT_H = 600;
static constexpr int FLAT_JPEG_QUALITY = 78;
// The existing axis-aligned crop is not perspective/curvature flattening.
// Send sensor JPEG directly and let the server flatten it; avoid an expensive
// decode/resample/re-encode that does not remove the need for server flattening.
static constexpr bool LOCAL_PAGE_CROP = false;

enum class ReaderState : uint8_t { SEARCHING, SETTLING, STABLE, MOVING };

struct PageBox {
  int x0 = 0;
  int y0 = 0;
  int x1 = 0;
  int y1 = 0;
  float confidence = 0.0f;
  float spineSlope = 0.0f;
  float spineIntercept = 0.0f;
  float spineConfidence = 0.0f;
  bool spineValid = false;
  bool valid = false;
};

struct MonitorFeatures {
  int8_t motion[MOTION_W * MOTION_H] = {};
  int8_t page[PAGE_W * PAGE_H] = {};
  int8_t scene[PAGE_W * PAGE_H] = {};
  PageBox box;
};

static ReaderState readerState = ReaderState::SEARCHING;
static MonitorFeatures previousFeatures;
static bool havePrevious = false;
static int8_t acceptedPage[PAGE_W * PAGE_H] = {};
static int8_t acceptedScene[PAGE_W * PAGE_H] = {};
// Keep an anchor as well as an adaptive reference: gradual changes must not
// let the reference drift all the way into a different page.
static int8_t anchorPage[PAGE_W * PAGE_H] = {};
static int8_t anchorScene[PAGE_W * PAGE_H] = {};
static bool rebaseAfterCapture = false;
static uint32_t rebaseStartedAt = 0;
static bool haveAcceptedPage = false;
static int8_t candidatePage[PAGE_W * PAGE_H] = {};
static bool haveCandidatePage = false;
static PageConfirmation pageConfirmation;
static bool motionSeen = false;
static uint8_t stableFrames = 0;
static uint32_t lastMonitorAt = 0;
static uint32_t lastDetectionLogAt = 0;
static uint32_t lastEventAt = 0;
static uint32_t eventCounter = 0;
static uint32_t previousPageEventAt = 0;
static bool cameraReady = false;
static bool streamMode = false;
static bool readingPaused = false;
static uint32_t nextStreamAt = 0;
static constexpr uint32_t STREAM_INTERVAL_MS = 700;
static constexpr uint32_t DEVICE_HEARTBEAT_INTERVAL_MS = 4000;
static uint32_t lastHeartbeatAt = 0;
static uint32_t lastWiFiBeginAt = 0;
static bool wifiBeginIssued = false;
static TaskHandle_t heartbeatTaskHandle = nullptr;
static SemaphoreHandle_t networkRequestMutex = nullptr;
static String pendingHttpResponse;
static uint32_t previousStreamRequestAt = 0;

// Reused for every page upload.  HTTPClient keeps this TLS socket alive when
// reuse is enabled, avoiding a DNS/TCP/TLS setup on each image.
static WiFiClientSecure uploadClient;
static bool uploadClientReady = false;
static Preferences readerPreferences;
static String configuredWifiSsid;
static String configuredWifiPassword;
static String pendingWifiSsid;
static String pendingWifiPassword;
static String pendingWifiRequestId;
static String pendingBookRequestId;
static String activeBleRequestId;
static String currentBookId;
static String bleConfigAssembly;
static uint8_t bleConfigNextFragment = 0;
static uint8_t bleConfigFragmentCount = 0;
static volatile bool wifiConfigPending = false;
static volatile bool bookChangePending = false;
static constexpr const char *READER_USER_AGENT =
    "Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36 "
    "(KHTML, like Gecko) Chrome/132.0.0.0 Safari/537.36";
static constexpr const char *BLE_SERVICE_UUID = "12345678-1234-5678-1234-56789abcdef0";
static constexpr const char *BLE_DEVICE_ID_CHAR_UUID = "12345678-1234-5678-1234-56789abcdef1";
static constexpr const char *BLE_CONFIG_CHAR_UUID = "12345678-1234-5678-1234-56789abcdef2";
static constexpr const char *BLE_CONTROL_CHAR_UUID = "12345678-1234-5678-1234-56789abcdef3";
static constexpr const char *BLE_STATUS_CHAR_UUID = "12345678-1234-5678-1234-56789abcdef4";
static constexpr const char *BLE_NAME_PREFIX = "PicBook-";
static BLECharacteristic *bleStatusCharacteristic = nullptr;

static void resetBaseline();
static void notifyBleStatus(const String &status);
static String jsonStringField(const std::string &json, const char *key);
static bool assembleBleConfigFragment(const std::string &fragment, String &complete, bool &ready);
static void handleBookSwitch();
static bool switchFrameSize(framesize_t frameSize, int quality);
static String makeMetadata(uint32_t id, uint32_t width, uint32_t height,
                           float pageDistance, const PageBox &monitorBox,
                           bool cropped);

static const char *stateName(ReaderState state) {
  switch (state) {
    case ReaderState::SEARCHING: return "SEARCHING";
    case ReaderState::SETTLING: return "SETTLING";
    case ReaderState::STABLE: return "STABLE";
    case ReaderState::MOVING: return "MOVING";
  }
  return "UNKNOWN";
}

static void *allocPsram(size_t bytes) {
  return heap_caps_malloc(bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
}

static String encodeBase64NoWrap(const uint8_t *data, size_t length) {
  if (!data || length == 0) return String();
  const size_t capacity = 4 * ((length + 2) / 3);
  char *encoded = static_cast<char *>(allocPsram(capacity + 1));
  if (!encoded) return String();
  size_t written = 0;
  const int rc = mbedtls_base64_encode(
      reinterpret_cast<unsigned char *>(encoded), capacity + 1, &written, data, length);
  if (rc != 0 || written != capacity) {
    free(encoded);
    return String();
  }
  encoded[written] = '\0';
  String result(encoded);
  free(encoded);
  return result;
}

static bool writeAll(WiFiClientSecure &client, const uint8_t *data, size_t length) {
  size_t offset = 0;
  const uint32_t started = millis();
  while (offset < length && millis() - started < 60000) {
    const size_t chunk = min(static_cast<size_t>(1024), length - offset);
    const size_t written = client.write(data + offset, chunk);
    offset += written;
    if (written == 0) delay(2);
  }
  return offset == length;
}

static inline uint8_t grayAt(const uint8_t *rgb, int width, int x, int y) {
  const uint8_t *p = rgb + (static_cast<size_t>(y) * width + x) * 3;
  return static_cast<uint8_t>((static_cast<uint16_t>(p[0]) * 77 +
                               static_cast<uint16_t>(p[1]) * 150 +
                               static_cast<uint16_t>(p[2]) * 29) >> 8);
}

struct QuadPoint {
  float x;
  float y;
};

// Lightweight four-corner resampler. It uses bilinear interpolation across
// the detected page quad, avoiding OpenCV and a homography matrix on-device.
// The monitor detector currently supplies an axis-aligned quad; the function
// is intentionally independent so a future edge detector can provide tilted
// corners without changing the event pipeline.
static bool warpQuadRgb(const uint8_t *src, int srcW, int srcH,
                        const QuadPoint &tl, const QuadPoint &tr,
                        const QuadPoint &br, const QuadPoint &bl,
                        uint8_t *dst, int dstW, int dstH) {
  if (!src || !dst || srcW < 2 || srcH < 2 || dstW < 2 || dstH < 2) return false;
  for (int y = 0; y < dstH; ++y) {
    const float v = static_cast<float>(y) / static_cast<float>(dstH - 1);
    for (int x = 0; x < dstW; ++x) {
      const float u = static_cast<float>(x) / static_cast<float>(dstW - 1);
      const float topX = tl.x + (tr.x - tl.x) * u;
      const float topY = tl.y + (tr.y - tl.y) * u;
      const float botX = bl.x + (br.x - bl.x) * u;
      const float botY = bl.y + (br.y - bl.y) * u;
      const float sx = topX + (botX - topX) * v;
      const float sy = topY + (botY - topY) * v;
      const int x0 = constrain(static_cast<int>(floorf(sx)), 0, srcW - 1);
      const int y0 = constrain(static_cast<int>(floorf(sy)), 0, srcH - 1);
      const int x1 = min(x0 + 1, srcW - 1);
      const int y1 = min(y0 + 1, srcH - 1);
      const float ax = constrain(sx - x0, 0.0f, 1.0f);
      const float ay = constrain(sy - y0, 0.0f, 1.0f);
      const uint8_t *p00 = src + (static_cast<size_t>(y0) * srcW + x0) * 3;
      const uint8_t *p10 = src + (static_cast<size_t>(y0) * srcW + x1) * 3;
      const uint8_t *p01 = src + (static_cast<size_t>(y1) * srcW + x0) * 3;
      const uint8_t *p11 = src + (static_cast<size_t>(y1) * srcW + x1) * 3;
      uint8_t *out = dst + (static_cast<size_t>(y) * dstW + x) * 3;
      for (int c = 0; c < 3; ++c) {
        const float a = p00[c] + (p10[c] - p00[c]) * ax;
        const float b = p01[c] + (p11[c] - p01[c]) * ax;
        out[c] = static_cast<uint8_t>(constrain(lroundf(a + (b - a) * ay), 0L, 255L));
      }
    }
  }
  return true;
}

static PageBox detectPageBox(const uint8_t *rgb, int width, int height) {
  PageBox box;
  uint64_t sum = 0;
  uint8_t maximum = 0;
  uint32_t samples = 0;
  for (int y = 0; y < height; y += 2) {
    for (int x = 0; x < width; x += 2) {
      const uint8_t g = grayAt(rgb, width, x, y);
      sum += g;
      maximum = max(maximum, g);
      ++samples;
    }
  }
  if (samples == 0) return box;

  const int mean = static_cast<int>(sum / samples);
  const int threshold = constrain(mean + (maximum - mean) * 35 / 100, 105, 220);
  const int rowMinimum = max(8, width / 7);
  int top = -1;
  int bottom = -1;
  for (int y = 0; y < height; y += 2) {
    int bright = 0;
    for (int x = 0; x < width; x += 2) {
      if (grayAt(rgb, width, x, y) >= threshold) bright += 2;
    }
    if (bright >= rowMinimum) {
      if (top < 0) top = y;
      bottom = y;
    }
  }
  if (top < 0 || bottom - top < height / 5) return box;

  const int columnMinimum = max(6, (bottom - top + 1) / 7);
  int left = -1;
  int right = -1;
  for (int x = 0; x < width; x += 2) {
    int bright = 0;
    for (int y = top; y <= bottom; y += 2) {
      if (grayAt(rgb, width, x, y) >= threshold) bright += 2;
    }
    if (bright >= columnMinimum) {
      if (left < 0) left = x;
      right = x;
    }
  }
  if (left < 0 || right - left < width / 5) return box;

  const int insetX = max(2, (right - left) / 50);
  const int insetY = max(2, (bottom - top) / 50);
  box.x0 = constrain(left + insetX, 0, width - 2);
  box.y0 = constrain(top + insetY, 0, height - 2);
  box.x1 = constrain(right - insetX, box.x0 + 1, width - 1);
  box.y1 = constrain(bottom - insetY, box.y0 + 1, height - 1);
  const float areaRatio = static_cast<float>((box.x1 - box.x0 + 1) * (box.y1 - box.y0 + 1)) /
                          static_cast<float>(width * height);
  box.confidence = min(1.0f, areaRatio / 0.55f);
  box.valid = areaRatio >= 0.10f;

  // Approximate the book gutter with a low-cost line search on a 160x120
  // grayscale edge map. This is deliberately much smaller than the UXGA
  // capture and only runs on the QVGA monitor frame.
  if (box.valid && width >= 160 && height >= 120) {
    constexpr int SW = 160;
    constexpr int SH = 120;
    static uint8_t edge[SW * SH];
    for (int sy = 0; sy < SH; ++sy) {
      const int py = min(height - 1, sy * height / SH);
      for (int sx = 0; sx < SW; ++sx) {
        const int px = sx * width / SW;
        const int xa = max(0, px - max(1, width / 160));
        const int xb = min(width - 1, px + max(1, width / 160));
        edge[sy * SW + sx] = static_cast<uint8_t>(abs(
            static_cast<int>(grayAt(rgb, width, xb, py)) -
            static_cast<int>(grayAt(rgb, width, xa, py))));
      }
    }
    float bestScore = 0.0f;
    int bestIntercept = 0;
    int bestSlopeQ = 0;
    for (int slopeQ = -64; slopeQ <= 64; slopeQ += 8) {
      for (int intercept = 56; intercept <= 104; ++intercept) {
        uint32_t score = 0;
        for (int sy = 0; sy < SH; ++sy) {
          const int sx = constrain(intercept +
              (slopeQ * (sy - SH / 2)) / 256, 1, SW - 2);
          score += edge[sy * SW + sx];
        }
        if (score > bestScore) {
          bestScore = static_cast<float>(score);
          bestIntercept = intercept;
          bestSlopeQ = slopeQ;
        }
      }
    }
    box.spineIntercept = bestIntercept * static_cast<float>(width) / SW;
    box.spineSlope = static_cast<float>(bestSlopeQ) / 256.0f;
    const float boxWidth = static_cast<float>(box.x1 - box.x0);
    const float interceptInBox = box.spineIntercept - box.x0;
    box.spineConfidence = min(1.0f, bestScore / 900.0f);
    box.spineValid = bestScore >= 360.0f && abs(bestSlopeQ) < 56 &&
                     boxWidth >= 20.0f && interceptInBox >= boxWidth * 0.25f &&
                     interceptInBox <= boxWidth * 0.75f;
  }
  return box;
}

static bool warpSpreadRgb(const uint8_t *src, int srcW, int srcH,
                          int cropX0, int cropY0, int cropX1, int cropY1,
                          float spineSlope, float spineIntercept,
                          uint8_t *dst, int dstW, int dstH) {
  if (!src || !dst || dstW < 4 || dstH < 2 || cropX1 - cropX0 < 4 ||
      cropY1 - cropY0 < 2) return false;
  const int mid = dstW / 2;
  for (int y = 0; y < dstH; ++y) {
    const float v = static_cast<float>(y) / static_cast<float>(dstH - 1);
    const float sy = cropY0 + v * (cropY1 - cropY0);
    const float monitorY = sy * 240.0f / srcH;
    const float spineX = (spineIntercept + spineSlope * (monitorY - 120.0f)) *
                         srcW / 320.0f;
    if (spineX <= cropX0 + 2 || spineX >= cropX1 - 2) return false;
    for (int x = 0; x < dstW; ++x) {
      float sx;
      if (x < mid) {
        const float u = static_cast<float>(x) / static_cast<float>(mid - 1);
        sx = cropX0 + (spineX - cropX0) * u;
      } else {
        const float u = static_cast<float>(x - mid) /
                        static_cast<float>(dstW - mid - 1);
        sx = spineX + (cropX1 - spineX) * u;
      }
      const int x0 = constrain(static_cast<int>(floorf(sx)), cropX0, cropX1 - 1);
      const int y0 = constrain(static_cast<int>(floorf(sy)), cropY0, cropY1 - 1);
      const int x1 = min(x0 + 1, cropX1 - 1);
      const int y1 = min(y0 + 1, cropY1 - 1);
      const float ax = constrain(sx - x0, 0.0f, 1.0f);
      const float ay = constrain(sy - y0, 0.0f, 1.0f);
      const uint8_t *p00 = src + (static_cast<size_t>(y0) * srcW + x0) * 3;
      const uint8_t *p10 = src + (static_cast<size_t>(y0) * srcW + x1) * 3;
      const uint8_t *p01 = src + (static_cast<size_t>(y1) * srcW + x0) * 3;
      const uint8_t *p11 = src + (static_cast<size_t>(y1) * srcW + x1) * 3;
      uint8_t *out = dst + (static_cast<size_t>(y) * dstW + x) * 3;
      for (int c = 0; c < 3; ++c) {
        const float a = p00[c] + (p10[c] - p00[c]) * ax;
        const float b = p01[c] + (p11[c] - p01[c]) * ax;
        out[c] = static_cast<uint8_t>(constrain(lroundf(a + (b - a) * ay), 0L, 255L));
      }
    }
  }
  return true;
}

template <int GRID_W, int GRID_H>
static void makeNormalizedGrid(const uint8_t *rgb, int width, int height,
                               const PageBox &box, int8_t (&out)[GRID_W * GRID_H]) {
  int16_t raw[GRID_W * GRID_H];
  int32_t total = 0;
  const int cropW = box.x1 - box.x0 + 1;
  const int cropH = box.y1 - box.y0 + 1;

  for (int gy = 0; gy < GRID_H; ++gy) {
    const int ya = box.y0 + gy * cropH / GRID_H;
    const int yb = box.y0 + (gy + 1) * cropH / GRID_H;
    for (int gx = 0; gx < GRID_W; ++gx) {
      const int xa = box.x0 + gx * cropW / GRID_W;
      const int xb = box.x0 + (gx + 1) * cropW / GRID_W;
      uint32_t sum = 0;
      uint32_t count = 0;
      const int stepX = max(1, (xb - xa) / 3);
      const int stepY = max(1, (yb - ya) / 3);
      for (int y = ya; y < max(ya + 1, yb); y += stepY) {
        for (int x = xa; x < max(xa + 1, xb); x += stepX) {
          sum += grayAt(rgb, width, min(x, width - 1), min(y, height - 1));
          ++count;
        }
      }
      const int value = count ? static_cast<int>(sum / count) : 0;
      raw[gy * GRID_W + gx] = value;
      total += value;
    }
  }

  constexpr int cells = GRID_W * GRID_H;
  const float mean = static_cast<float>(total) / cells;
  float variance = 0.0f;
  for (int i = 0; i < cells; ++i) {
    const float delta = raw[i] - mean;
    variance += delta * delta;
  }
  const float standardDeviation = sqrtf(variance / cells);
  const float scale = standardDeviation > 2.0f ? 32.0f / standardDeviation : 1.0f;
  for (int i = 0; i < cells; ++i) {
    out[i] = static_cast<int8_t>(constrain(lroundf((raw[i] - mean) * scale), -127L, 127L));
  }
}

static float shiftedDistance(const int8_t *a, const int8_t *b, int width, int height,
                             int maxShift) {
  float best = 10.0f;
  for (int dy = -maxShift; dy <= maxShift; ++dy) {
    for (int dx = -maxShift; dx <= maxShift; ++dx) {
      uint32_t difference = 0;
      uint32_t count = 0;
      for (int y = max(0, -dy); y < min(height, height - dy); ++y) {
        for (int x = max(0, -dx); x < min(width, width - dx); ++x) {
          difference += abs(static_cast<int>(a[y * width + x]) -
                            static_cast<int>(b[(y + dy) * width + (x + dx)]));
          ++count;
        }
      }
      if (count) best = min(best, static_cast<float>(difference) / (count * 64.0f));
    }
  }
  return best;
}

static bool getMonitorFeatures(MonitorFeatures &features) {
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("MONITOR_FRAME_FAIL");
    return false;
  }
  const size_t rgbBytes = fb->width * fb->height * 3;
  uint8_t *rgb = static_cast<uint8_t *>(allocPsram(rgbBytes));
  if (!rgb) {
    Serial.printf("MONITOR_ALLOC_FAIL bytes=%u\n", static_cast<unsigned>(rgbBytes));
    esp_camera_fb_return(fb);
    return false;
  }
  const bool decoded = fmt2rgb888(fb->buf, fb->len, fb->format, rgb);
  const int width = static_cast<int>(fb->width);
  const int height = static_cast<int>(fb->height);
  esp_camera_fb_return(fb);
  if (!decoded) {
    Serial.println("MONITOR_DECODE_FAIL");
    free(rgb);
    return false;
  }

  features.box = detectPageBox(rgb, width, height);
  if (features.box.valid) {
    makeNormalizedGrid<MOTION_W, MOTION_H>(rgb, width, height, features.box, features.motion);
    makeNormalizedGrid<PAGE_W, PAGE_H>(rgb, width, height, features.box, features.page);
    // A second signature uses fixed sensor coordinates. Changes to the bright
    // page bounding box alone must not count as new page content.
    PageBox full;
    full.x1 = width - 1; full.y1 = height - 1;
    makeNormalizedGrid<PAGE_W, PAGE_H>(rgb, width, height, full, features.scene);
  }
  free(rgb);
  return features.box.valid;
}

static bool connectWiFi(uint32_t timeoutMs = 8000) {
  if (configuredWifiSsid.isEmpty()) return false;
  if (WiFi.status() == WL_CONNECTED) return true;
  if (WiFi.getMode() != WIFI_STA) WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  // Heartbeat retries every four seconds. Do not restart association on every
  // retry while the ESP32 is already trying to rejoin the access point.
  if (!wifiBeginIssued || millis() - lastWiFiBeginAt >= 15000) {
    WiFi.begin(configuredWifiSsid.c_str(), configuredWifiPassword.c_str());
    lastWiFiBeginAt = millis();
    wifiBeginIssued = true;
    Serial.println("WIFI_CONNECTING");
  }
  const uint32_t started = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - started < timeoutMs) delay(100);
  if (WiFi.status() == WL_CONNECTED) wifiBeginIssued = false;
  Serial.printf("WIFI_%s", WiFi.status() == WL_CONNECTED ? "OK" : "FAIL");
  if (WiFi.status() == WL_CONNECTED) Serial.printf(" ip=%s", WiFi.localIP().toString().c_str());
  Serial.println();
  return WiFi.status() == WL_CONNECTED;
}

static String jsonStringField(const std::string &json, const char *key) {
  const String input(json.c_str());
  const String marker = String("\"") + key + "\"";
  int pos = input.indexOf(marker);
  if (pos < 0) return String();
  pos = input.indexOf(':', pos + marker.length());
  if (pos < 0) return String();
  ++pos;
  while (pos < static_cast<int>(input.length()) && isspace(input[pos])) ++pos;
  if (pos >= static_cast<int>(input.length()) || input[pos] != '"') return String();
  ++pos;
  String result;
  bool escaped = false;
  for (; pos < static_cast<int>(input.length()); ++pos) {
    const char c = input[pos];
    if (escaped) { result += c; escaped = false; continue; }
    if (c == '\\') { escaped = true; continue; }
    if (c == '"') break;
    result += c;
  }
  return result;
}

// Optional application fragmentation for MTUs smaller than the Wi-Fi JSON.
// Wire form: @<zero-based-index>/<total>:<UTF-8 chunk>
static bool assembleBleConfigFragment(const std::string &fragment, String &complete, bool &ready) {
  ready = false;
  const String part(fragment.c_str());
  if (!part.startsWith("@")) { complete = part; ready = true; return true; }
  const int slash = part.indexOf('/');
  const int colon = part.indexOf(':');
  if (slash < 2 || colon <= slash + 1) { bleConfigAssembly = ""; bleConfigNextFragment = 0; return false; }
  const int index = part.substring(1, slash).toInt();
  const int total = part.substring(slash + 1, colon).toInt();
  if (total < 1 || total > 16 || index < 0 || index >= total) {
    bleConfigAssembly = ""; bleConfigNextFragment = 0; return false;
  }
  if (index == 0) {
    bleConfigAssembly = ""; bleConfigNextFragment = 0;
    bleConfigFragmentCount = static_cast<uint8_t>(total);
  }
  if (total != bleConfigFragmentCount || index != bleConfigNextFragment ||
      bleConfigAssembly.length() + part.length() - colon - 1 > 512) {
    bleConfigAssembly = ""; bleConfigNextFragment = 0; bleConfigFragmentCount = 0;
    return false;
  }
  bleConfigAssembly += part.substring(colon + 1);
  ++bleConfigNextFragment;
  if (bleConfigNextFragment == bleConfigFragmentCount) {
    complete = bleConfigAssembly;
    ready = true;
    bleConfigAssembly = ""; bleConfigNextFragment = 0; bleConfigFragmentCount = 0;
  }
  return true;
}

static void startBleIdentity() {
  // BLE advertising names are limited to 31 bytes. The device ID is exposed
  // through the read-only characteristic, so keep the advertised name short.
  String bleName = "PicBook";
  BLEDevice::init(bleName.c_str());
  BLEDevice::setMTU(517);
  BLEServer *server = BLEDevice::createServer();
  BLEService *service = server->createService(BLE_SERVICE_UUID);
  BLECharacteristic *identity = service->createCharacteristic(
      BLE_DEVICE_ID_CHAR_UUID, BLECharacteristic::PROPERTY_READ);
  identity->setValue(READER_DEVICE_ID);
  BLECharacteristic *configuration = service->createCharacteristic(
      BLE_CONFIG_CHAR_UUID, BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
  BLECharacteristic *status = service->createCharacteristic(
      BLE_STATUS_CHAR_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  status->addDescriptor(new BLE2902());
  status->setValue("{\"type\":\"status\",\"status\":\"ready\"}");
  bleStatusCharacteristic = status;
  class ConfigCallbacks final : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *characteristic) override {
      const std::string value = characteristic->getValue();
      String payload;
      bool ready = false;
      if (value.empty() || value.size() > 512 || !assembleBleConfigFragment(value, payload, ready)) {
        notifyBleStatus("ERROR:CONFIG_SIZE");
        return;
      }
      if (!ready) return;
      const String ssid = jsonStringField(value, "ssid");
      const String password = jsonStringField(value, "password");
      const String requestId = jsonStringField(value, "requestId");
      if (!payload.startsWith("{") || ssid.isEmpty() || requestId.isEmpty()) {
        notifyBleStatus("ERROR:CONFIG_FORMAT");
        return;
      }
      if (ssid.isEmpty() || ssid.length() > 32 || password.length() > 63) {
        notifyBleStatus("ERROR:CONFIG_VALUE");
        return;
      }
      pendingWifiSsid = ssid;
      pendingWifiPassword = password;
      pendingWifiRequestId = requestId;
      wifiConfigPending = true;
    }
  };
  configuration->setCallbacks(new ConfigCallbacks());
  BLECharacteristic *events = service->createCharacteristic(
      BLE_CONTROL_CHAR_UUID, BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
  class EventCallbacks final : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *characteristic) override {
      const std::string value = characteristic->getValue();
      const String type = jsonStringField(value, "type");
      const String requestId = jsonStringField(value, "requestId");
      if (type == "switch_book" && !requestId.isEmpty()) {
        pendingBookRequestId = requestId;
        bookChangePending = true;
        readingPaused = true;
      } else notifyBleStatus("ERROR:UNKNOWN_CONTROL");
    }
  };
  events->setCallbacks(new EventCallbacks());
  service->start();
  BLEAdvertising *advertising = BLEDevice::getAdvertising();
  advertising->addServiceUUID(BLE_SERVICE_UUID);
  advertising->setScanResponse(true);
  BLEDevice::startAdvertising();
  Serial.printf("BLE_READY name=%s service=%s id_char=%s config_char=%s control_char=%s status_char=%s device_id=%s\n",
                bleName.c_str(), BLE_SERVICE_UUID, BLE_DEVICE_ID_CHAR_UUID,
                BLE_CONFIG_CHAR_UUID, BLE_CONTROL_CHAR_UUID, BLE_STATUS_CHAR_UUID,
                READER_DEVICE_ID);
}

static void notifyBleStatus(const String &status) {
  if (!bleStatusCharacteristic) return;
  const bool wifi = status.startsWith("WIFI_") || status.startsWith("ERROR:CONFIG") ||
                    status == "ERROR:STORAGE";
  const String type = wifi ? "wifi_status" : "book_switch_status";
  const String requestId = !activeBleRequestId.isEmpty() ? activeBleRequestId :
      (wifi ? pendingWifiRequestId : pendingBookRequestId);
  String state;
  if (status == "WIFI_CONNECTED") state = "connected";
  else if (status == "WIFI_CONNECT_FAILED") state = "failed";
  else if (status == "WIFI_SAVED") state = "saved";
  else if (status == "BOOK_IDENTIFIED") state = "book_identified";
  else if (status == "BOOK_IDENTIFY_FAILED") state = "failed";
  else if (status == "BOOK_SWITCHING") state = "switching";
  else if (status.startsWith("ERROR:")) state = "failed";
  else { state = status; state.toLowerCase(); }
  String payload = String("{\"type\":\"") + type + "\",\"requestId\":\"" + requestId +
      "\",\"status\":\"" + state + "\"";
  if (!wifi && !currentBookId.isEmpty()) payload += String(",\"bookId\":\"") + currentBookId + "\"";
  payload += "}";
  bleStatusCharacteristic->setValue(payload.c_str());
  bleStatusCharacteristic->notify();
}

static void handleBleRequests() {
  if (wifiConfigPending) {
    wifiConfigPending = false;
    const String ssid = pendingWifiSsid;
    const String password = pendingWifiPassword;
    activeBleRequestId = pendingWifiRequestId;
    if (!readerPreferences.begin("reader", false)) {
      notifyBleStatus("ERROR:STORAGE");
      return;
    }
    const size_t storedSsid = readerPreferences.putString("ssid", ssid);
    const size_t storedPassword = readerPreferences.putString("password", password);
    readerPreferences.end();
    if (storedSsid != ssid.length() || storedPassword != password.length()) {
      notifyBleStatus("ERROR:STORAGE");
      activeBleRequestId = "";
    } else {
      configuredWifiSsid = ssid;
      configuredWifiPassword = password;
      wifiBeginIssued = false;
      WiFi.disconnect(false, false);
      notifyBleStatus("WIFI_SAVED");
      Serial.println("BLE_WIFI_CONFIG_SAVED");
      if (connectWiFi(30000)) notifyBleStatus("WIFI_CONNECTED");
      else notifyBleStatus("WIFI_CONNECT_FAILED");
      activeBleRequestId = "";
    }
  }
  if (bookChangePending) {
    bookChangePending = false;
    activeBleRequestId = pendingBookRequestId;
    handleBookSwitch();
    activeBleRequestId = "";
  }
}

static bool identifyBookCover(const uint8_t *jpeg, size_t jpegLength, String &bookId) {
  if (!connectWiFi(30000)) return false;
  const String encoded = encodeBase64NoWrap(jpeg, jpegLength);
  if (encoded.isEmpty()) return false;
  // The current camera path captures the raw upright sensor image; it does not
  // perform geometric page/cover flattening, so report flattened=false honestly.
  const String body = String("{\"imageBytesBase64\":\"") + encoded +
      "\",\"flattened\":false}";
  WiFiClientSecure client;
  client.setInsecure();
  HTTPClient http;
  http.setTimeout(15000);
  const String url = String("https://") + READER_API_HOST + "/api/v1/devices/" +
      READER_DEVICE_ID + "/book-swap";
  if (!http.begin(client, url)) return false;
  http.addHeader("User-Agent", READER_USER_AGENT);
  http.addHeader("Content-Type", "application/json");
#ifdef READER_DEVICE_TOKEN
  if (strlen(READER_DEVICE_TOKEN)) http.addHeader("X-Device-Token", READER_DEVICE_TOKEN);
#endif
  const int status = http.POST(body);
  const String response = status > 0 ? http.getString() : String();
  http.end();
  bookId = jsonStringField(std::string(response.c_str()), "bookId");
  Serial.printf("BOOK_SWAP status=%d response_bytes=%u book_id=%s\n", status,
                static_cast<unsigned>(response.length()), bookId.isEmpty() ? "missing" : "received");
  if (status != 200) Serial.printf("BOOK_SWAP_ERROR %.220s\n", response.c_str());
  return status == 200 && !bookId.isEmpty();
}

static void handleBookSwitch() {
  const String previousBookId = currentBookId;
  readingPaused = true;
  streamMode = false;
  notifyBleStatus("BOOK_SWITCHING");
  Serial.println("BOOK_SWITCH_BEGIN capture=cover");
  if (!connectWiFi(30000)) Serial.println("BOOK_SWITCH_WIFI_UNAVAILABLE");
  bool identified = false;
  String newBookId;
  if (cameraReady && switchFrameSize(FRAMESIZE_SVGA, 10)) {
    camera_fb_t *cover = esp_camera_fb_get();
    if (cover) {
      identified = identifyBookCover(cover->buf, cover->len, newBookId);
      esp_camera_fb_return(cover);
    } else Serial.println("BOOK_COVER_CAPTURE_FAIL");
    switchFrameSize(FRAMESIZE_QVGA, 16);
  }
  if (identified) {
    currentBookId = newBookId;
    if (readerPreferences.begin("reader", false)) {
      readerPreferences.putString("bookId", currentBookId);
      readerPreferences.end();
    }
    eventCounter = 0;
    resetBaseline();
    readingPaused = false;
    notifyBleStatus("BOOK_IDENTIFIED");
    Serial.println("BOOK_SWITCH_DONE baseline_reset=yes");
  } else {
    currentBookId = previousBookId;
    readingPaused = true;
    notifyBleStatus("BOOK_IDENTIFY_FAILED");
    Serial.println("BOOK_SWITCH_FAIL reading_paused=yes retry_switch_book");
  }
}

static void reportDeviceHeartbeat() {
  if (!connectWiFi(1000)) return;
  const time_t now = time(nullptr);
  if (now < 1700000000) {
    Serial.println("HEARTBEAT_TIME_UNSYNCED");
    return;
  }
  if (networkRequestMutex) xSemaphoreTake(networkRequestMutex, portMAX_DELAY);
  WiFiClientSecure heartbeatClient;
  heartbeatClient.setInsecure();
  heartbeatClient.setTimeout(10000);
  HTTPClient http;
  const String endpoint = String("https://") + READER_API_HOST + "/api/v1/devices/" + READER_DEVICE_ID + "/statuses";
  if (!http.begin(heartbeatClient, endpoint)) {
    if (networkRequestMutex) xSemaphoreGive(networkRequestMutex);
    Serial.println("HEARTBEAT_BEGIN_FAIL");
    return;
  }
  http.setTimeout(10000);
  http.addHeader("User-Agent", READER_USER_AGENT);
  http.addHeader("Content-Type", "application/json");
#ifdef READER_DEVICE_TOKEN
  if (strlen(READER_DEVICE_TOKEN)) http.addHeader("X-Device-Token", READER_DEVICE_TOKEN);
#endif
  // Keep the heartbeat body compatible with the currently deployed server.
  // v1.2 optional fields can be enabled after that server is deployed.
  const String body = String("{\"alive\":true,\"timestamp\":") +
      String(static_cast<unsigned long>(now)) + "}";
  const int status = http.POST(body);
  http.end();
  if (networkRequestMutex) xSemaphoreGive(networkRequestMutex);
  Serial.printf("HEARTBEAT status=%d\n", status);
}

static void heartbeatTask(void *) {
  TickType_t lastWake = xTaskGetTickCount();
  for (;;) {
    reportDeviceHeartbeat();
    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(DEVICE_HEARTBEAT_INTERVAL_MS));
  }
}

static void runHttpTest() {
  const uint32_t started = millis();
  if (!connectWiFi(15000)) {
    Serial.println("HTTP_TEST WIFI_FAIL");
    return;
  }
  const uint32_t wifiReadyAt = millis();
  WiFiClientSecure client;
  client.setInsecure();
  HTTPClient http;
  http.setTimeout(20000);
  const uint32_t beginAt = millis();
  if (!http.begin(client, "https://example.com/")) {
    Serial.printf("HTTP_TEST BEGIN_FAIL wifi_ms=%u total_ms=%u\n",
                  static_cast<unsigned>(wifiReadyAt - started),
                  static_cast<unsigned>(millis() - started));
    return;
  }
  const uint32_t beginDoneAt = millis();
  const int status = http.GET();
  const uint32_t doneAt = millis();
  const int length = http.getSize();
  http.end();
  Serial.printf("HTTP_TEST status=%d content_length=%d wifi_ms=%u begin_ms=%u request_ms=%u total_ms=%u rssi=%d ip=%s\n",
                status, length, static_cast<unsigned>(wifiReadyAt - started),
                static_cast<unsigned>(beginDoneAt - beginAt),
                static_cast<unsigned>(doneAt - beginDoneAt),
                static_cast<unsigned>(doneAt - started), WiFi.RSSI(),
                WiFi.localIP().toString().c_str());
}

static bool uploadMultipart(const uint8_t *jpeg, size_t jpegLength, const String &metadata,
                            bool flattened, const String &clientEventId) {
  if (currentBookId.isEmpty()) {
    Serial.println("HTTP_UPLOAD_SKIPPED reason=no_book_id");
    return false;
  }
  if (!connectWiFi()) return false;
  String encoded = encodeBase64NoWrap(jpeg, jpegLength);
  if (encoded.length() != 4 * ((jpegLength + 2) / 3)) {
    Serial.println("HTTP_BASE64_ENCODE_FAIL");
    return false;
  }
  String body;
  body.reserve(encoded.length() + 320);
  body = "{\"imageBytesBase64\":\"" + encoded + "\",\"flattened\":" +
         (flattened ? "true" : "false");
  body += String(",\"bookId\":\"") + currentBookId + "\",\"clientEventId\":\"" +
          clientEventId + "\"}";
  Serial.printf("HTTP_IMAGE flattened=%s bytes=%u\n", flattened ? "true" : "false",
                static_cast<unsigned>(jpegLength));
  Serial.printf("HTTP_UPLOAD_BEGIN jpeg_bytes=%u body_bytes=%u rssi=%d\n",
                static_cast<unsigned>(jpegLength),
                static_cast<unsigned>(body.length()), WiFi.RSSI());
  /*
  const String boundary = "----ShoulderReaderBoundary7MA4YWxk";
  const String prefix = "--" + boundary + "\r\n"
      "Content-Disposition: form-data; name=\"metadata\"\r\n"
      "Content-Type: application/json\r\n\r\n" + metadata + "\r\n--" + boundary + "\r\n"
      "Content-Disposition: form-data; name=\"page\"; filename=\"page.jpg\"\r\n"
      "Content-Type: image/jpeg\r\n\r\n";
  const String suffix = "\r\n--" + boundary + "--\r\n";
  const size_t bodyLength = prefix.length() + jpegLength + suffix.length();
  uint8_t *body = static_cast<uint8_t *>(allocPsram(bodyLength));
  if (!body) {
    Serial.printf("HTTP_ALLOC_FAIL bytes=%u\n", static_cast<unsigned>(bodyLength));
    return false;
  }
  memcpy(body, prefix.c_str(), prefix.length());
  memcpy(body + prefix.length(), jpeg, jpegLength);
  memcpy(body + prefix.length() + jpegLength, suffix.c_str(), suffix.length());
  */

  // A page-turning event must have a clean HTTP message boundary. Do not
  // reuse the stream socket here: a delayed response on a stale keep-alive
  // socket can make the server parse two JSON bodies as one request.
  WiFiClientSecure eventClient;
  eventClient.setInsecure();
  eventClient.setTimeout(60000);
  if (networkRequestMutex) xSemaphoreTake(networkRequestMutex, portMAX_DELAY);
  if (!eventClient.connect(READER_API_HOST, 443)) {
    if (networkRequestMutex) xSemaphoreGive(networkRequestMutex);
    Serial.println("HTTP_CONNECT_FAIL");
    return false;
  }
  const String request = String("POST /api/v1/devices/") + READER_DEVICE_ID +
      "/page-turning HTTP/1.1\r\nHost: " READER_API_HOST +
      "\r\nUser-Agent: " + READER_USER_AGENT +
      "\r\nContent-Type: application/json\r\nConnection: close\r\n" +
#ifdef READER_DEVICE_TOKEN
      String("X-Device-Token: ") + READER_DEVICE_TOKEN + "\r\n" +
#endif
      "Content-Length: " +
      String(body.length()) + "\r\n\r\n";
  bool sent = writeAll(eventClient, reinterpret_cast<const uint8_t *>(request.c_str()), request.length());
  if (sent) sent = writeAll(eventClient, reinterpret_cast<const uint8_t *>(body.c_str()), body.length());
  Serial.printf("HTTP_UPLOAD_SENT headers=%u body=%u connected=%s\n",
                static_cast<unsigned>(request.length()),
                sent ? static_cast<unsigned>(body.length()) : 0,
                eventClient.connected() ? "yes" : "no");
  String statusLine = eventClient.readStringUntil('\n');
  int status = -1;
  if (statusLine.startsWith("HTTP/1.")) status = statusLine.substring(9, 12).toInt();
  String responseBody;
  const uint32_t responseStarted = millis();
  // Successful async submissions need only the status line. Do not hold the
  // capture loop for up to 10 s waiting for the remote connection to close.
  while (sent && status >= 400 && eventClient.connected() && millis() - responseStarted < 2000) {
    while (eventClient.available() && responseBody.length() < 512) responseBody += static_cast<char>(eventClient.read());
    if (!eventClient.available()) delay(5);
  }
  eventClient.stop();
  if (networkRequestMutex) xSemaphoreGive(networkRequestMutex);
  if (!sent) status = -3;
  const int responseSeparator = responseBody.indexOf("\r\n\r\n");
  if (responseSeparator >= 0) {
    String bodyText = responseBody.substring(responseSeparator + 4);
    bodyText.replace("\r", " " );
    bodyText.replace("\n", " " );
    Serial.printf("HTTP_RESPONSE_BODY %.300s\n", bodyText.c_str());
  }
  Serial.printf("HTTP_RESULT status=%d bytes=%u\n", status, static_cast<unsigned>(jpegLength));
  if (status >= 400) {
    responseBody.replace("\r", " ");
    responseBody.replace("\n", " ");
    Serial.printf("HTTP_ERROR_BODY %.240s\n", responseBody.c_str());
  }
  return status >= 200 && status < 300;
}

static void streamOneFrame() {
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) { Serial.println("STREAM_CAPTURE_FAIL"); return; }
  const uint32_t id = ++eventCounter;
  const uint32_t started = millis();
  // Consume any already-arrived responses without waiting. HTTP/1.1 responses
  // remain ordered on this persistent socket while requests are pipelined.
  while (uploadClient.available()) {
    const int ch = uploadClient.read();
    if (ch >= 0) pendingHttpResponse += static_cast<char>(ch);
  }
  int statusLine = pendingHttpResponse.indexOf("HTTP/1.");
  while (statusLine >= 0) {
    const int lineEnd = pendingHttpResponse.indexOf('\n', statusLine);
    if (lineEnd < 0) break;
    String line = pendingHttpResponse.substring(statusLine, lineEnd);
    Serial.printf("STREAM_RESPONSE %s\n", line.c_str());
    pendingHttpResponse.remove(0, lineEnd + 1);
    statusLine = pendingHttpResponse.indexOf("HTTP/1.");
  }
  if (pendingHttpResponse.length() > 2048) pendingHttpResponse.remove(0, pendingHttpResponse.length() - 256);

  String encoded = encodeBase64NoWrap(fb->buf, fb->len);
  if (encoded.isEmpty()) {
    Serial.println("STREAM_BASE64_ENCODE_FAIL");
    esp_camera_fb_return(fb);
    return;
  }
  String body;
  body.reserve(encoded.length() + 64);
  body = "{\"imageBytesBase64\":\"" + encoded + "\",\"flattened\":false}";
  Serial.printf("STREAM_UPLOAD_BEGIN jpeg_bytes=%u body_bytes=%u rssi=%d\n",
                static_cast<unsigned>(fb->len),
                static_cast<unsigned>(body.length()), WiFi.RSSI());
  bool ok = true;
  if (WiFi.status() != WL_CONNECTED) ok = false;
  if (ok && !uploadClientReady) {
    uploadClient.setInsecure();
    uploadClient.setTimeout(60000);
    uploadClientReady = true;
  }
  if (ok && !uploadClient.connected()) {
    uploadClient.stop();
    ok = uploadClient.connect(READER_API_HOST, 443);
    if (ok) uploadClient.setTimeout(60000);
  }
  if (ok) {
    // Continuous preview frames use /page.  Only confirmed page changes use
    // /page-turning; sending every preview frame there makes the server emit
    // the same narration repeatedly.
    String headers = "POST /api/v1/devices/" READER_DEVICE_ID "/page HTTP/1.1\r\n";
    headers += "Host: " READER_API_HOST "\r\n";
    headers += "User-Agent: "; headers += READER_USER_AGENT; headers += "\r\n";
    headers += "Content-Type: application/json\r\nConnection: keep-alive\r\n";
    headers += "Content-Length: "; headers += body.length(); headers += "\r\n\r\n";
    if (previousStreamRequestAt) {
      while (millis() - previousStreamRequestAt < STREAM_INTERVAL_MS) delay(1);
    }
    const uint32_t requestAt = millis();
    const uint32_t requestGap = previousStreamRequestAt ? requestAt - previousStreamRequestAt : 0;
    previousStreamRequestAt = requestAt;
    ok = uploadClient.write(reinterpret_cast<const uint8_t *>(headers.c_str()), headers.length()) == headers.length();
    if (ok) ok = uploadClient.write(reinterpret_cast<const uint8_t *>(body.c_str()), body.length()) == body.length();
    Serial.printf("STREAM_REQUEST_GAP_MS %u\n", static_cast<unsigned>(requestGap));
  }
  Serial.printf("STREAM_RESULT id=%u status=%s bytes=%u elapsed_ms=%u\n", id,
                ok ? "sent" : "send_fail", static_cast<unsigned>(fb->len),
                static_cast<unsigned>(millis() - started));
  esp_camera_fb_return(fb);
}

static void sendSerialEvent(const uint8_t *jpeg, size_t jpegLength, const String &metadata) {
#if READER_SERIAL_FALLBACK
  Serial.printf("EVENT_BEGIN %u %u\n", static_cast<unsigned>(metadata.length()),
                static_cast<unsigned>(jpegLength));
  Serial.write(reinterpret_cast<const uint8_t *>(metadata.c_str()), metadata.length());
  Serial.write(jpeg, jpegLength);
  Serial.write('\n');
  Serial.println("EVENT_END");
  Serial.flush();
#else
  (void)jpeg;
  (void)jpegLength;
  (void)metadata;
#endif
}

static String makeMetadata(uint32_t id, uint32_t width, uint32_t height,
                           float pageDistance, const PageBox &monitorBox,
                           bool cropped) {
  String json;
  json.reserve(320);
  json += "{\"device_id\":\"" READER_DEVICE_ID "\",\"event_id\":";
  json += id;
  json += ",\"captured_at_ms\":";
  json += millis();
  json += ",\"image_width\":";
  json += width;
  json += ",\"image_height\":";
  json += height;
  json += ",\"vector_distance\":";
  json += String(pageDistance, 4);
  json += ",\"cropped\":";
  json += cropped ? "true" : "false";
  json += ",\"monitor_box\":[";
  json += monitorBox.x0; json += ','; json += monitorBox.y0; json += ',';
  json += monitorBox.x1; json += ','; json += monitorBox.y1; json += "],\"spine\":[";
  json += String(monitorBox.spineSlope, 4); json += ',';
  json += String(monitorBox.spineIntercept, 2); json += ',';
  json += monitorBox.spineValid ? "true" : "false"; json += "]}";
  return json;
}

static bool switchFrameSize(framesize_t frameSize, int quality) {
  sensor_t *sensor = esp_camera_sensor_get();
  if (!sensor) return false;
  if (sensor->set_framesize(sensor, frameSize) != 0) return false;
  sensor->set_quality(sensor, quality);
  delay(180);
  // Give automatic exposure several complete frames to settle after a large
  // resolution change. A single discarded frame stayed severely underexposed
  // in the shoulder-mounted indoor test.
  for (int i = 0; i < 3; ++i) {
    camera_fb_t *stale = esp_camera_fb_get();
    if (stale) esp_camera_fb_return(stale);
    delay(70);
  }
  return true;
}

static bool captureEvent(float pageDistance, const PageBox &monitorBox, bool manual = false,
                         bool initial = false) {
  const uint32_t captureStarted = millis();
  // Automatic narration is uploaded at 800x600. Capture at that resolution
  // directly to avoid decoding 1600x1200 only to resize it down again.
  if (!switchFrameSize(manual ? FRAMESIZE_UXGA : FRAMESIZE_SVGA, 10)) {
    Serial.println("UXGA_SWITCH_FAIL");
    return false;
  }
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("UXGA_FRAME_FAIL");
    switchFrameSize(FRAMESIZE_QVGA, 16);
    return false;
  }

  uint8_t *eventJpeg = fb->buf;
  size_t eventLength = fb->len;
  uint32_t outputWidth = fb->width;
  uint32_t outputHeight = fb->height;
  bool ownsJpeg = false;
  bool cropped = false;

  const size_t rgbBytes = fb->width * fb->height * 3;
  // Manual captures are always sent as the untouched UXGA sensor JPEG. This
  // keeps the COM9 diagnostic path independent of page detection/warping.
  uint8_t *rgb = LOCAL_PAGE_CROP && !manual && monitorBox.valid
                     ? static_cast<uint8_t *>(allocPsram(rgbBytes))
                     : nullptr;
  if (rgb && fmt2rgb888(fb->buf, fb->len, fb->format, rgb)) {
    int x0 = monitorBox.valid ? monitorBox.x0 * static_cast<int>(fb->width) / 320 : 0;
    int y0 = monitorBox.valid ? monitorBox.y0 * static_cast<int>(fb->height) / 240 : 0;
    int x1 = monitorBox.valid ? (monitorBox.x1 + 1) * static_cast<int>(fb->width) / 320 : fb->width;
    int y1 = monitorBox.valid ? (monitorBox.y1 + 1) * static_cast<int>(fb->height) / 240 : fb->height;
    x0 = constrain(x0, 0, static_cast<int>(fb->width) - 2);
    y0 = constrain(y0, 0, static_cast<int>(fb->height) - 2);
    x1 = constrain(x1, x0 + 2, static_cast<int>(fb->width));
    y1 = constrain(y1, y0 + 2, static_cast<int>(fb->height));
    uint8_t *flatRgb = static_cast<uint8_t *>(allocPsram(static_cast<size_t>(FLAT_W) * FLAT_H * 3));
    const QuadPoint tl{static_cast<float>(x0), static_cast<float>(y0)};
    const QuadPoint tr{static_cast<float>(x1 - 1), static_cast<float>(y0)};
    const QuadPoint br{static_cast<float>(x1 - 1), static_cast<float>(y1 - 1)};
    const QuadPoint bl{static_cast<float>(x0), static_cast<float>(y1 - 1)};
    const bool warped = flatRgb && warpQuadRgb(rgb, fb->width, fb->height, tl, tr, br, bl,
                                                flatRgb, FLAT_W, FLAT_H);
    uint8_t *croppedJpeg = nullptr;
    size_t croppedLength = 0;
    const bool encoded = warped && fmt2jpg(flatRgb, static_cast<size_t>(FLAT_W) * FLAT_H * 3,
                                           FLAT_W, FLAT_H, PIXFORMAT_RGB888, FLAT_JPEG_QUALITY,
                                           &croppedJpeg, &croppedLength);
    const bool completeJpeg = encoded && croppedJpeg && croppedLength >= 4 &&
                              croppedJpeg[0] == 0xff && croppedJpeg[1] == 0xd8 &&
                              croppedJpeg[croppedLength - 2] == 0xff &&
                              croppedJpeg[croppedLength - 1] == 0xd9;
    if (completeJpeg) {
      eventJpeg = croppedJpeg;
      eventLength = croppedLength;
      outputWidth = FLAT_W;
      outputHeight = FLAT_H;
      ownsJpeg = true;
      cropped = true;
    } else {
      Serial.printf("FLAT_JPEG_FAIL encoded=%s warped=%s bytes=%u fallback=uxga\n",
                    encoded ? "yes" : "no", warped ? "yes" : "no",
                    static_cast<unsigned>(croppedLength));
      if (croppedJpeg) free(croppedJpeg);
    }
    if (flatRgb) free(flatRgb);
  }
  if (rgb) free(rgb);

  const uint32_t id = ++eventCounter;
  Serial.printf("CAPTURE_READY processing_ms=%u source=%ux%u\n",
                static_cast<unsigned>(millis() - captureStarted), fb->width, fb->height);
  const String metadata = makeMetadata(id, outputWidth, outputHeight, pageDistance, monitorBox, cropped);
  // Manual CAPTURE commands use the same upload path as automatic page turns;
  // this makes a serial-triggered image a real server upload as well.
  // Neither raw sensor JPEG nor rectangular crop is fully flattened. Only a
  // future successful flattening stage may change this value to true.
  const bool flattened = false;
  const String clientEventId = String(READER_DEVICE_ID "-page-") +
      String(static_cast<unsigned long>(time(nullptr))) + "-" + String(millis()) + "-" + String(id);
  bool uploaded = false;
  for (uint8_t attempt = 0; attempt < 3 && !uploaded; ++attempt) {
    if (attempt) delay(static_cast<uint32_t>(attempt) * 1000);
    uploaded = uploadMultipart(eventJpeg, eventLength, metadata, flattened, clientEventId);
    Serial.printf("PAGE_UPLOAD_ATTEMPT event=%s attempt=%u result=%s\n",
                  clientEventId.c_str(), static_cast<unsigned>(attempt + 1), uploaded ? "ok" : "failed");
  }
  if (!uploaded) sendSerialEvent(eventJpeg, eventLength, metadata);
  const uint32_t eventGap = previousPageEventAt ? millis() - previousPageEventAt : 0;
  previousPageEventAt = millis();
  Serial.printf("PAGE_EVENT id=%u reason=%s gap_ms=%u distance=%.3f jpeg=%u size=%ux%u cropped=%s\n",
                id, manual ? "manual" : (initial ? "initial" : "turn"),
                static_cast<unsigned>(eventGap), pageDistance,
                static_cast<unsigned>(eventLength), outputWidth, outputHeight,
                cropped ? "yes" : "no");

  if (ownsJpeg) free(eventJpeg);
  esp_camera_fb_return(fb);
  const bool restored = switchFrameSize(FRAMESIZE_QVGA, 16);
  if (!restored) Serial.println("QVGA_RESTORE_FAIL");
  lastEventAt = millis();
  havePrevious = false;
  stableFrames = 0;
  rebaseAfterCapture = restored && haveAcceptedPage;
  rebaseStartedAt = millis();
  return restored;
}

static void resetBaseline() {
  readerState = ReaderState::SEARCHING;
  havePrevious = false;
  haveAcceptedPage = false;
  rebaseAfterCapture = false;
  haveCandidatePage = false;
  pageConfirmation.reset();
  motionSeen = false;
  stableFrames = 0;
  Serial.println("BASELINE_RESET");
}

static void processMonitorFrame() {
  const uint32_t frameStarted = millis();
  MonitorFeatures current;
  if (!getMonitorFeatures(current)) {
    readerState = ReaderState::SEARCHING;
    havePrevious = false;
    stableFrames = 0;
    haveCandidatePage = false;
    pageConfirmation.reset();
    return;
  }

  if (!havePrevious) {
    previousFeatures = current;
    havePrevious = true;
    stableFrames = 1;
    readerState = ReaderState::SETTLING;
    return;
  }

  const float motion = shiftedDistance(previousFeatures.motion, current.motion, MOTION_W, MOTION_H, 1);
  if (millis() - lastDetectionLogAt >= 2000) {
    lastDetectionLogAt = millis();
    Serial.printf("DETECT_SAMPLE frame_ms=%u motion=%.3f stable=%u baseline=%s\n",
                  static_cast<unsigned>(millis() - frameStarted), motion, stableFrames,
                  haveAcceptedPage ? "yes" : "no");
  }
  previousFeatures = current;
  if (motion >= MOTION_THRESHOLD) {
    readerState = ReaderState::MOVING;
    motionSeen = true;
    stableFrames = 0;
    haveCandidatePage = false;
    pageConfirmation.reset();
    return;
  }
  if (motion > STABLE_THRESHOLD) {
    readerState = ReaderState::SETTLING;
    stableFrames = 0;
    haveCandidatePage = false;
    pageConfirmation.reset();
    return;
  }

  readerState = ReaderState::SETTLING;
  if (stableFrames < 255) ++stableFrames;
  if (stableFrames < STABLE_FRAMES_REQUIRED) return;

  readerState = ReaderState::STABLE;
  // Resolution changes alter auto exposure. Learn the monitor baseline after
  // returning to QVGA instead of interpreting that change as another turn.
  if (rebaseAfterCapture) {
    if (millis() - rebaseStartedAt < 800) return;
    memcpy(acceptedPage, current.page, sizeof(acceptedPage));
    memcpy(acceptedScene, current.scene, sizeof(acceptedScene));
    memcpy(anchorPage, current.page, sizeof(anchorPage));
    memcpy(anchorScene, current.scene, sizeof(anchorScene));
    rebaseAfterCapture = false;
    motionSeen = false;
    Serial.println("MONITOR_BASELINE_RESTORED");
    return;
  }
  const bool initial = !haveAcceptedPage;
  // Compare page content with translation tolerance, not just motion.
  const float pageDistance = initial ? 0.0f :
      shiftedDistance(acceptedPage, current.page, PAGE_W, PAGE_H, 2);
  const float sceneDistance = initial ? 0.0f :
      shiftedDistance(acceptedScene, current.scene, PAGE_W, PAGE_H, 2);
  const float anchorDistance = initial ? 0.0f :
      shiftedDistance(anchorPage, current.page, PAGE_W, PAGE_H, 2);
  const float anchorSceneDistance = initial ? 0.0f :
      shiftedDistance(anchorScene, current.scene, PAGE_W, PAGE_H, 2);
  // Reference detector: refresh similar accepted content, otherwise require
  // consecutive similar changed frames. No prerequisite motion spike.
  if (!initial && pageDistance < 0.20f && sceneDistance < 0.16f &&
      anchorDistance < PAGE_CHANGE_THRESHOLD && anchorSceneDistance < 0.22f) {
    memcpy(acceptedPage, current.page, sizeof(acceptedPage));
    memcpy(acceptedScene, current.scene, sizeof(acceptedScene));
  }
  const bool eligible = initial || ((pageDistance >= PAGE_CHANGE_THRESHOLD ||
      anchorDistance >= PAGE_CHANGE_THRESHOLD) &&
      (sceneDistance >= 0.22f || anchorSceneDistance >= 0.22f) &&
      millis() - lastEventAt >= EVENT_COOLDOWN_MS);
  const bool matches = haveCandidatePage &&
      shiftedDistance(candidatePage, current.page, PAGE_W, PAGE_H, 1) < 0.16f;
  const bool confirmed = pageConfirmation.update(millis(), eligible, matches, initial);
  if (!eligible) {
    haveCandidatePage = false;
    if (pageDistance < 0.15f && sceneDistance < 0.15f) motionSeen = false;
    return;
  }
  if (!matches) {
    memcpy(candidatePage, current.page, sizeof(candidatePage));
    haveCandidatePage = true;
  }
  if (!confirmed) return;
  Serial.printf("PAGE_CONFIRMED reason=%s at_ms=%u distance=%.3f scene=%.3f motion=%.3f\n",
                initial ? "initial" : "turn", static_cast<unsigned>(millis()), pageDistance,
                sceneDistance, motion);
  memcpy(acceptedPage, current.page, sizeof(acceptedPage));
  memcpy(acceptedScene, current.scene, sizeof(acceptedScene));
  memcpy(anchorPage, current.page, sizeof(anchorPage));
  memcpy(anchorScene, current.scene, sizeof(anchorScene));
  haveAcceptedPage = true;
  haveCandidatePage = false;
  pageConfirmation.reset();
  motionSeen = false;
  if (initial) Serial.println("BASELINE_ACCEPTED FIRST_PAGE_UPLOAD");
  captureEvent(pageDistance, current.box, false, initial);
}

static void processSerialCommand() {
  if (!Serial.available()) return;
  String command = Serial.readStringUntil('\n');
  command.trim();
  command.toUpperCase();
  if (command == "SNAPSHOT") {
    // Inspect the same sensor orientation/resolution as automatic uploads,
    // without submitting a page-turn event to the server.
    readingPaused = true;
    streamMode = false;
    if (switchFrameSize(FRAMESIZE_SVGA, 10)) {
      camera_fb_t *frame = esp_camera_fb_get();
      if (frame) {
        Serial.printf("SNAPSHOT_BEGIN %u\n", static_cast<unsigned>(frame->len));
        Serial.write(frame->buf, frame->len);
        Serial.println("\nSNAPSHOT_END");
        Serial.flush();
        esp_camera_fb_return(frame);
      } else Serial.println("SNAPSHOT_FAIL");
    }
    switchFrameSize(FRAMESIZE_QVGA, 16);
  } else if (command == "CAPTURE") {
    MonitorFeatures current;
    if (getMonitorFeatures(current)) {
      Serial.printf("MANUAL_PAGE_BOX x0=%d y0=%d x1=%d y1=%d confidence=%.2f spine_valid=%s slope=%.3f intercept=%.1f spine_confidence=%.2f\n",
                    current.box.x0, current.box.y0, current.box.x1, current.box.y1,
                    current.box.confidence, current.box.spineValid ? "yes" : "no",
                    current.box.spineSlope, current.box.spineIntercept,
                    current.box.spineConfidence);
      captureEvent(0.0f, current.box, true);
    } else {
      Serial.println("MANUAL_PAGE_NOT_FOUND using=raw");
      PageBox full;
      captureEvent(0.0f, full, true);
    }
  } else if (command == "CAPTURE_RAW") {
    PageBox full;
    captureEvent(0.0f, full, true);
  } else if (command == "STREAM_ON") {
    if (!streamMode && switchFrameSize(FRAMESIZE_QVGA, 24)) {
      if (!uploadClientReady) {
        uploadClient.setInsecure();
        uploadClient.setTimeout(60000);
        uploadClientReady = true;
      }
      const bool connected = uploadClient.connected() ||
          uploadClient.connect(READER_API_HOST, 443);
      if (connected) {
        streamMode = true;
        nextStreamAt = millis();
        Serial.println("STREAM_ON target_request_gap_ms=700");
      } else {
        Serial.println("STREAM_TLS_CONNECT_FAIL");
      }
    } else if (!streamMode) Serial.println("STREAM_START_FAIL");
  } else if (command == "READING_PAUSE") {
    readingPaused = true;
    streamMode = false;
    Serial.println("READING_PAUSED");
  } else if (command == "READING_START") {
    streamMode = false;
    if (currentBookId.isEmpty()) {
      Serial.println("READING_START_BLOCKED reason=no_book_id");
    } else {
      switchFrameSize(FRAMESIZE_QVGA, 16);
      resetBaseline();
      readingPaused = false;
      Serial.println("READING_STARTED FIRST_PAGE_PENDING");
    }
  } else if (command == "STREAM_OFF") {
    streamMode = false;
    switchFrameSize(FRAMESIZE_QVGA, 16);
    Serial.println("STREAM_OFF");
  } else if (command == "HTTP_TEST" || command == "PING_EXAMPLE") {
    runHttpTest();
  } else if (command == "INFO" || command == "STATUS") {
    Serial.printf("STATUS camera=%s psram=%s free_psram=%u state=%s wifi=%d baseline=%s events=%u\n",
                  cameraReady ? "yes" : "no", psramFound() ? "yes" : "no",
                  static_cast<unsigned>(ESP.getFreePsram()), stateName(readerState),
                  static_cast<int>(WiFi.status()), haveAcceptedPage ? "yes" : "no", eventCounter);
  } else if (command == "RESET" || command == "RESET_BASELINE") {
    resetBaseline();
  } else if (command == "BOOK_CHANGED" || command == "CHANGE_BOOK") {
    bookChangePending = true;
  } else if (command.length()) {
    Serial.printf("UNKNOWN_COMMAND %s\n", command.c_str());
  }
}

static bool initializeCamera() {
  camera_config_t config = {};
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = PIN_Y2;
  config.pin_d1 = PIN_Y3;
  config.pin_d2 = PIN_Y4;
  config.pin_d3 = PIN_Y5;
  config.pin_d4 = PIN_Y6;
  config.pin_d5 = PIN_Y7;
  config.pin_d6 = PIN_Y8;
  config.pin_d7 = PIN_Y9;
  config.pin_xclk = PIN_XCLK;
  config.pin_pclk = PIN_PCLK;
  config.pin_vsync = PIN_VSYNC;
  config.pin_href = PIN_HREF;
  config.pin_sccb_sda = PIN_SIOD;
  config.pin_sccb_scl = PIN_SIOC;
  config.pin_pwdn = PIN_PWDN;
  config.pin_reset = PIN_RESET;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  // Allocate the camera frame buffer for the largest event frame. The sensor
  // is switched to QVGA immediately after init for the low-cost monitor loop.
  config.frame_size = FRAMESIZE_UXGA;
  config.jpeg_quality = 10;
  config.fb_count = 1;
  config.fb_location = psramFound() ? CAMERA_FB_IN_PSRAM : CAMERA_FB_IN_DRAM;
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  const esp_err_t error = esp_camera_init(&config);
  if (error != ESP_OK) {
    Serial.printf("CAMERA_INIT_FAIL 0x%x\n", static_cast<unsigned>(error));
    return false;
  }
  sensor_t *sensor = esp_camera_sensor_get();
  if (sensor) {
    Serial.printf("CAMERA_SENSOR pid=0x%02x ver=0x%02x mid=0x%02x%02x\n",
                  sensor->id.PID, sensor->id.VER, sensor->id.MIDH, sensor->id.MIDL);
  }
  if (!sensor || sensor->set_framesize(sensor, FRAMESIZE_QVGA) != 0) {
    Serial.println("CAMERA_QVGA_SWITCH_FAIL");
    return false;
  }
  sensor->set_quality(sensor, 16);
  sensor->set_gain_ctrl(sensor, 1);
  sensor->set_exposure_ctrl(sensor, 1);
  sensor->set_aec2(sensor, 1);
  sensor->set_ae_level(sensor, 2);
  sensor->set_gainceiling(sensor, GAINCEILING_32X);
  // The shoulder mount presents the sensor upside down. Apply a 180-degree
  // rotation in the sensor so OCR and narration see the page upright.
  sensor->set_hmirror(sensor, 0);
  sensor->set_vflip(sensor, 1);
  sensor->set_brightness(sensor, 1);
  sensor->set_contrast(sensor, 2);
  sensor->set_sharpness(sensor, 2);
  delay(180);
  for (int i = 0; i < 3; ++i) {
    camera_fb_t *stale = esp_camera_fb_get();
    if (stale) esp_camera_fb_return(stale);
    delay(70);
  }
  return true;
}

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(80);
  delay(1200);
  Serial.println("SHOULDER_READER_START");
  networkRequestMutex = xSemaphoreCreateMutex();
  readerPreferences.begin("reader", true);
  configuredWifiSsid = readerPreferences.getString("ssid", READER_WIFI_SSID);
  configuredWifiPassword = readerPreferences.getString("password", READER_WIFI_PASSWORD);
  currentBookId = readerPreferences.getString("bookId", "");
  readerPreferences.end();
  readingPaused = currentBookId.isEmpty();
  WiFi.persistent(false);
  // Keep modem sleep enabled: ESP-IDF requires it when Wi-Fi and BLE coexist.
  WiFi.onEvent([](WiFiEvent_t event, WiFiEventInfo_t info) {
    if (event == ARDUINO_EVENT_WIFI_STA_DISCONNECTED) {
      wifiBeginIssued = false;
      Serial.printf("WIFI_DISCONNECTED reason=%d\n", info.wifi_sta_disconnected.reason);
    } else if (event == ARDUINO_EVENT_WIFI_STA_GOT_IP) {
      Serial.printf("WIFI_GOT_IP ip=%s rssi=%d\n",
                    WiFi.localIP().toString().c_str(), WiFi.RSSI());
    }
  });
  cameraReady = initializeCamera();
  Serial.printf("CAMERA_INIT_%s psram=%s free_psram=%u\n", cameraReady ? "OK" : "FAIL",
                psramFound() ? "yes" : "no", static_cast<unsigned>(ESP.getFreePsram()));
  // Start the BLE controller before Wi-Fi. On this ESP32-S3 Arduino BLE stack,
  // enabling the controller after Wi-Fi startup can abort in coex_enable().
  startBleIdentity();
  if (!configuredWifiSsid.isEmpty()) connectWiFi();
  configTime(0, 0, "pool.ntp.org", "time.nist.gov");
  xTaskCreatePinnedToCore(heartbeatTask, "device-heartbeat", 8192, nullptr, 1,
                          &heartbeatTaskHandle, 0);
  Serial.println("COMMANDS CAPTURE CAPTURE_RAW STREAM_ON STREAM_OFF HTTP_TEST INFO RESET_BASELINE");
}

void loop() {
  processSerialCommand();
  handleBleRequests();
  if (streamMode) {
    if (millis() >= nextStreamAt) {
      streamOneFrame();
      nextStreamAt = millis();
    }
    delay(2);
    return;
  }
  if (!readingPaused && cameraReady && millis() - lastMonitorAt >= MONITOR_INTERVAL_MS) {
    lastMonitorAt = millis();
    processMonitorFrame();
  }
  delay(5);
}
