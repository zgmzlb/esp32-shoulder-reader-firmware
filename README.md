# ESP32-S3 shoulder reader firmware

Minimal PlatformIO firmware for the ESP32-S3 N16R8 reader board and its camera. This directory contains only the production application, the page confirmation helper, the custom board definition, and build/configuration files.

## Configure and build

Install PlatformIO, then copy src/device_config.local.example.h to src/device_config.local.h and set the local Wi-Fi credentials and device ID. The local file is ignored by Git.

    python -m platformio run
    python -m platformio run --target upload --upload-port COM7
    python -m platformio device monitor --port COM7 --baud 115200

Use the COM port shown by Windows if it is not COM7.

## Device behavior

- Reads camera frames and confirms stable page changes before upload.
- Advertises BLE service 12345678-1234-5678-1234-56789abcdef0 for device ID, Wi-Fi setup, book-switch control, and status.
- Sends heartbeat requests every four seconds.
- On a book-switch request, captures a cover and calls POST /api/v1/devices/{device_id}/book-swap. A successful HTTP 200 response supplies data.bookId.
- Sends page events to POST /api/v1/devices/{device_id}/page-turning with the active bookId and a retry-stable clientEventId.

The camera path does not currently perform perspective/curvature flattening, so it reports flattened:false.
