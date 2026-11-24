#pragma once

// Copy this file to device_config.local.h and fill in values for your board.
#define READER_WIFI_SSID "your-hotspot-name"
#define READER_WIFI_PASSWORD "your-hotspot-password"
#define READER_DEVICE_ID "your-device-id"
#define READER_API_HOST "jerrywumodelscope-picture-book-reading-server.ms.show"
#define READER_CLIENT_ENDPOINT "https://" READER_API_HOST "/api/v1/devices/" READER_DEVICE_ID "/page-turning"
#define READER_SERIAL_FALLBACK 0

// Optional, only if the server has DEVICE_TOKEN enabled:
// #define READER_DEVICE_TOKEN "your-device-token"
