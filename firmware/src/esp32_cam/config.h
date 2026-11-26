#pragma once
// ============================================================================
//  Pharma Rover ESP32-CAM settings
// ============================================================================

// Same hotspot as the laptop and the main ESP32 (2.4 GHz)
#define WIFI_SSID      "66C-2.4G"
#define WIFI_PASSWORD  "ninetyone"
#define DEVICE_HOSTNAME "pharmacam"

// Leave SERVER_IP empty ("") to find the laptop automatically (UDP discovery).
// If discovery doesn't work on your network, put the laptop's IP here,
// e.g. "192.168.43.100" (run `ipconfig` on the laptop to see it).
#define SERVER_IP      "192.168.18.13"
#define SERVER_PORT    8000
#define UPLOAD_PATH    "/api/vision/hand_frame"
#define DISCOVERY_UDP_PORT 4210

// Picture settings
#define FRAME_SIZE     FRAMESIZE_VGA   // 640x480 - good for YOLO and barcodes
#define JPEG_QUALITY   12              // 10 = better quality / bigger, 20 = smaller
#define MAX_FPS        8               // frames uploaded per second (max)
#define FLIP_VERTICAL  0               // set to 1 if the picture is upside down
#define MIRROR_IMAGE   0

// Flash LED (GPIO4). 0 = off, 255 = full. The server can also change this
// while it is scanning the shelf (see vision.hand_led_duty in config.yaml).
#define FLASH_LED_PIN      4
#define FLASH_DEFAULT_DUTY 0
