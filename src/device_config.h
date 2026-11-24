#pragma once

// Safe defaults for a public source repository. Create device_config.local.h
// with the actual local values; that file is excluded from Git.
#define READER_WIFI_SSID ""
#define READER_WIFI_PASSWORD ""
#define READER_DEVICE_ID "replace-with-device-id"
#define READER_API_HOST "jerrywumodelscope-picture-book-reading-server.ms.show"
#define READER_CLIENT_ENDPOINT "https://" READER_API_HOST "/api/v1/devices/" READER_DEVICE_ID "/page-turning"

#define READER_SERIAL_FALLBACK 0
