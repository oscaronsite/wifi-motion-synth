/*
 * WiFi CSI firmware for the LilyGo T-Display S3 (ESP32-S3)
 * -------------------------------------------------------
 * - Joins your WiFi. The network name and password are kept on the board and can be
 *   changed from the laptop page (the first ones come from sdkconfig.defaults.esp32s3).
 *   If a new network does not work, the board goes back to the previous one by itself.
 *   If no known network works, it opens its own WiFi called "CSI-Setup" so the page
 *   can send it new settings.
 * - Listens on UDP port 5005 for small "poke" packets from the laptop page. Every packet
 *   the WiFi router transmits to the board gives one CSI reading, sent back to the laptop.
 * - Shows the signal on the built-in screen as a hi-fi style spectrum, with the IP address.
 * - BOOT button: mute / unmute the page's sound.
 * - Second button: short press = screen on/off, hold 3 s = setup mode.
 *
 * Built and tested to compile with ESP-IDF v5.5.6, target esp32s3.
 *
 * WiFi Motion Synth - https://github.com/oscaronsite/wifi-motion-synth
 * Copyright 2026 oscaronsite
 * Based on the csi_recv_router example from Espressif esp-csi
 * (https://github.com/espressif/esp-csi), Copyright Espressif Systems.
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"

#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "esp_heap_caps.h"
#include "driver/gpio.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_io_i80.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_st7789.h"

#include "lwip/sockets.h"
#include "sdkconfig.h"

#if !CONFIG_IDF_TARGET_ESP32S3
#error "This firmware is for the ESP32-S3 (run: idf.py set-target esp32s3)"
#endif

/* ---------------------------------------------------------------- settings */
#define UDP_PORT            5005
#define CSI_MAX_LEN         256
#define HEADER_LEN          12
#define MAX_CREDIT          8
#define PEER_TIMEOUT_US     (3 * 1000000LL)
#define REPORT_PERIOD_US    (5 * 1000000LL)
#define LINK_TIMEOUT_US     (30 * 1000000LL)
#define CONNECT_TIMEOUT_MS  20000
#define SETUP_AP_SSID       "CSI-Setup"
#define SETUP_MIN_US        (5 * 60 * 1000000LL)   /* setup mode retries the networks after 5 min... */
#define SETUP_IDLE_US       (60 * 1000000LL)       /* ...if no laptop talked to it for 1 min */

#ifdef CONFIG_EXAMPLE_WIFI_SSID
#define DEFAULT_SSID CONFIG_EXAMPLE_WIFI_SSID
#define DEFAULT_PASS CONFIG_EXAMPLE_WIFI_PASSWORD
#else
#define DEFAULT_SSID ""
#define DEFAULT_PASS ""
#endif

/* LilyGo T-Display S3 pins (from LilyGo's pin_config.h) */
#define PIN_LCD_BL      38
#define PIN_POWER_ON    15
#define PIN_LCD_RES     5
#define PIN_LCD_CS      6
#define PIN_LCD_DC      7
#define PIN_LCD_WR      8
#define PIN_LCD_RD      9
#define PIN_BUTTON_2    14
#define PIN_BUTTON_BOOT 0       /* BOOT button: free to use once the board is running */
static const int LCD_DATA_PINS[8] = {39, 40, 41, 42, 45, 46, 47, 48};
#define LCD_PCLK_HZ     (10 * 1000 * 1000)

static const char *TAG = "csi_board";

/* ---------------------------------------------------------------- shared state */
#define NBARS 24

typedef enum { ST_BOOT, ST_CONNECTING, ST_ONLINE, ST_SETUP, ST_SWITCHING } board_state_t;

typedef struct {
    board_state_t state;
    char ssid[33];
    char ip[16];
    int rssi;
    uint32_t readings;
    int64_t last_reading_us;
    bool have_shape;
    float bar[NBARS];     /* shape of the signal, around 1.0 */
    float heat[NBARS];    /* 0 = calm, 1 = strong change */
    float level;          /* overall movement 0..1 */
} disp_t;

static disp_t s_disp = { .state = ST_BOOT, .rssi = 0 };
static portMUX_TYPE s_disp_mux = portMUX_INITIALIZER_UNLOCKED;

static void disp_set_state(board_state_t st, const char *ssid)
{
    portENTER_CRITICAL(&s_disp_mux);
    s_disp.state = st;
    if (ssid) {
        strlcpy(s_disp.ssid, ssid, sizeof(s_disp.ssid));
    }
    portEXIT_CRITICAL(&s_disp_mux);
}

// ===== DRAWING BEGIN (plain C, no ESP-IDF calls) =====
/* 8x14 bitmap font for ASCII 32-126 (generated from DejaVu Sans Mono Bold) */
static const uint8_t FONT8X14[95][14] = {
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x18,0x18,0x18,0x18,0x18,0x18,0x00,0x18,0x18,0x00,0x00,0x00},
    {0x00,0x00,0x64,0x64,0x64,0x64,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x00,0x12,0x16,0x7f,0x24,0x2c,0xfe,0x68,0x48,0x00,0x00,0x00},
    {0x00,0x10,0x10,0x3c,0x78,0x70,0x3c,0x1e,0x16,0x7e,0x3c,0x10,0x10,0x00},
    {0x00,0x00,0x70,0xd0,0xd0,0x72,0x18,0x4e,0x0b,0x0b,0x0e,0x00,0x00,0x00},
    {0x00,0x00,0x38,0x64,0x30,0x30,0x7b,0xcf,0xce,0x6e,0x3f,0x00,0x00,0x00},
    {0x00,0x00,0x18,0x18,0x18,0x18,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
    {0x0c,0x18,0x18,0x18,0x10,0x30,0x30,0x10,0x18,0x18,0x18,0x0c,0x00,0x00},
    {0x30,0x10,0x18,0x18,0x18,0x08,0x08,0x18,0x18,0x18,0x10,0x30,0x00,0x00},
    {0x00,0x00,0x18,0x5a,0x3c,0x3c,0x5a,0x18,0x00,0x00,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x00,0x18,0x18,0x18,0xfe,0xfe,0x18,0x18,0x18,0x00,0x00,0x00},
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x18,0x18,0x18,0x30,0x00},
    {0x00,0x00,0x00,0x00,0x00,0x00,0x3c,0x3c,0x00,0x00,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x18,0x18,0x00,0x00,0x00},
    {0x00,0x00,0x06,0x04,0x0c,0x0c,0x08,0x18,0x10,0x30,0x20,0x60,0x40,0x00},
    {0x00,0x00,0x3c,0x66,0x66,0x66,0x66,0x66,0x66,0x66,0x3c,0x00,0x00,0x00},
    {0x00,0x00,0x78,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x7e,0x00,0x00,0x00},
    {0x00,0x00,0x3c,0x4c,0x06,0x0e,0x0c,0x18,0x30,0x60,0x7e,0x00,0x00,0x00},
    {0x00,0x00,0x3c,0x46,0x06,0x3c,0x06,0x06,0x06,0x46,0x3c,0x00,0x00,0x00},
    {0x00,0x00,0x0c,0x1c,0x3c,0x6c,0x4c,0x7e,0x0c,0x0c,0x0c,0x00,0x00,0x00},
    {0x00,0x00,0x7c,0x60,0x60,0x7c,0x4e,0x06,0x06,0x4e,0x3c,0x00,0x00,0x00},
    {0x00,0x00,0x3c,0x60,0x60,0x7c,0x66,0x66,0x66,0x66,0x3c,0x00,0x00,0x00},
    {0x00,0x00,0x7e,0x06,0x0c,0x0c,0x1c,0x18,0x18,0x30,0x30,0x00,0x00,0x00},
    {0x00,0x00,0x3c,0x66,0x66,0x3c,0x66,0x66,0x66,0x66,0x3c,0x00,0x00,0x00},
    {0x00,0x00,0x3c,0x6e,0x66,0x66,0x6e,0x3e,0x06,0x4c,0x38,0x00,0x00,0x00},
    {0x00,0x00,0x00,0x00,0x18,0x18,0x00,0x00,0x00,0x18,0x18,0x00,0x00,0x00},
    {0x00,0x00,0x00,0x00,0x18,0x18,0x00,0x00,0x00,0x18,0x18,0x18,0x30,0x00},
    {0x00,0x00,0x00,0x00,0x02,0x1e,0x78,0xe0,0x78,0x1e,0x02,0x00,0x00,0x00},
    {0x00,0x00,0x00,0x00,0x00,0xfe,0xfe,0x00,0xfe,0xfe,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x00,0x00,0x40,0x70,0x1e,0x06,0x1e,0x70,0x40,0x00,0x00,0x00},
    {0x00,0x00,0x3c,0x06,0x06,0x0c,0x18,0x18,0x00,0x18,0x18,0x00,0x00,0x00},
    {0x00,0x00,0x3c,0x62,0x5e,0xd2,0xb2,0xb2,0xb2,0xd2,0x5e,0x62,0x1e,0x00},
    {0x00,0x00,0x18,0x3c,0x3c,0x2c,0x64,0x7e,0x66,0x46,0xc3,0x00,0x00,0x00},
    {0x00,0x00,0x7c,0x66,0x66,0x66,0x7c,0x66,0x66,0x66,0x7c,0x00,0x00,0x00},
    {0x00,0x00,0x1c,0x32,0x60,0x60,0x60,0x60,0x60,0x32,0x1c,0x00,0x00,0x00},
    {0x00,0x00,0x7c,0x6e,0x66,0x66,0x66,0x66,0x66,0x6e,0x7c,0x00,0x00,0x00},
    {0x00,0x00,0x7e,0x60,0x60,0x60,0x7e,0x60,0x60,0x60,0x7e,0x00,0x00,0x00},
    {0x00,0x00,0x7e,0x60,0x60,0x60,0x7e,0x60,0x60,0x60,0x60,0x00,0x00,0x00},
    {0x00,0x00,0x1c,0x72,0x60,0x60,0x6e,0x66,0x66,0x36,0x3e,0x00,0x00,0x00},
    {0x00,0x00,0x66,0x66,0x66,0x66,0x7e,0x66,0x66,0x66,0x66,0x00,0x00,0x00},
    {0x00,0x00,0x7e,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x7e,0x00,0x00,0x00},
    {0x00,0x00,0x3c,0x0c,0x0c,0x0c,0x0c,0x0c,0x0c,0x4c,0x7c,0x00,0x00,0x00},
    {0x00,0x00,0x66,0x6c,0x7c,0x78,0x78,0x6c,0x6c,0x66,0x67,0x00,0x00,0x00},
    {0x00,0x00,0x60,0x60,0x60,0x60,0x60,0x60,0x60,0x60,0x7e,0x00,0x00,0x00},
    {0x00,0x00,0xe6,0xe6,0xfe,0xfe,0xda,0xda,0xc2,0xc2,0xc2,0x00,0x00,0x00},
    {0x00,0x00,0x66,0x66,0x76,0x76,0x5e,0x4e,0x4e,0x4e,0x46,0x00,0x00,0x00},
    {0x00,0x00,0x3c,0x66,0x66,0x66,0x66,0x66,0x66,0x66,0x3c,0x00,0x00,0x00},
    {0x00,0x00,0x7c,0x66,0x66,0x66,0x66,0x7c,0x60,0x60,0x60,0x00,0x00,0x00},
    {0x00,0x00,0x3c,0x66,0x66,0x66,0x66,0x66,0x66,0x66,0x3c,0x0e,0x04,0x00},
    {0x00,0x00,0x7c,0x66,0x66,0x66,0x66,0x7c,0x6c,0x66,0x67,0x00,0x00,0x00},
    {0x00,0x00,0x3c,0x62,0x60,0x70,0x3c,0x0e,0x06,0x46,0x3c,0x00,0x00,0x00},
    {0x00,0x00,0x7e,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x00,0x00,0x00},
    {0x00,0x00,0x66,0x66,0x66,0x66,0x66,0x66,0x66,0x66,0x3c,0x00,0x00,0x00},
    {0x00,0x00,0xc6,0x66,0x66,0x66,0x64,0x3c,0x3c,0x3c,0x38,0x00,0x00,0x00},
    {0x00,0x00,0xc3,0xc3,0xdb,0xda,0x5a,0x7e,0x6e,0x66,0x66,0x00,0x00,0x00},
    {0x00,0x00,0xe6,0x66,0x3c,0x3c,0x18,0x3c,0x3c,0x66,0xc6,0x00,0x00,0x00},
    {0x00,0x00,0xc7,0x66,0x6e,0x3c,0x38,0x18,0x18,0x18,0x18,0x00,0x00,0x00},
    {0x00,0x00,0x7e,0x06,0x0e,0x1c,0x18,0x38,0x70,0x60,0x7e,0x00,0x00,0x00},
    {0x1c,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x1c,0x00,0x00},
    {0x00,0x00,0x40,0x60,0x20,0x30,0x10,0x18,0x08,0x0c,0x0c,0x04,0x06,0x00},
    {0x38,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x38,0x00,0x00},
    {0x00,0x00,0x18,0x3c,0x6c,0x46,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xff},
    {0x00,0x30,0x10,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x00,0x00,0x3c,0x46,0x06,0x7e,0x66,0x66,0x7e,0x00,0x00,0x00},
    {0x60,0x60,0x60,0x60,0x7c,0x66,0x66,0x66,0x66,0x66,0x7c,0x00,0x00,0x00},
    {0x00,0x00,0x00,0x00,0x1c,0x32,0x60,0x60,0x60,0x32,0x1c,0x00,0x00,0x00},
    {0x06,0x06,0x06,0x06,0x3e,0x6e,0x66,0x66,0x66,0x6e,0x3e,0x00,0x00,0x00},
    {0x00,0x00,0x00,0x00,0x3c,0x66,0x66,0x7e,0x60,0x62,0x3c,0x00,0x00,0x00},
    {0x0e,0x18,0x18,0x18,0x7e,0x18,0x18,0x18,0x18,0x18,0x18,0x00,0x00,0x00},
    {0x00,0x00,0x00,0x00,0x3e,0x6e,0x66,0x66,0x66,0x6e,0x3e,0x06,0x46,0x3c},
    {0x60,0x60,0x60,0x60,0x7c,0x66,0x66,0x66,0x66,0x66,0x66,0x00,0x00,0x00},
    {0x18,0x18,0x00,0x00,0x78,0x18,0x18,0x18,0x18,0x18,0x7e,0x00,0x00,0x00},
    {0x18,0x18,0x00,0x00,0x38,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x78},
    {0x60,0x60,0x60,0x60,0x66,0x6c,0x78,0x78,0x6c,0x66,0x66,0x00,0x00,0x00},
    {0x70,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x18,0x1e,0x00,0x00,0x00},
    {0x00,0x00,0x00,0x00,0xfe,0xda,0xda,0xda,0xda,0xda,0xda,0x00,0x00,0x00},
    {0x00,0x00,0x00,0x00,0x7c,0x66,0x66,0x66,0x66,0x66,0x66,0x00,0x00,0x00},
    {0x00,0x00,0x00,0x00,0x3c,0x66,0x66,0x66,0x66,0x66,0x3c,0x00,0x00,0x00},
    {0x00,0x00,0x00,0x00,0x7c,0x66,0x66,0x66,0x66,0x66,0x7c,0x60,0x60,0x60},
    {0x00,0x00,0x00,0x00,0x3e,0x6e,0x66,0x66,0x66,0x6e,0x3e,0x06,0x06,0x06},
    {0x00,0x00,0x00,0x00,0x3e,0x38,0x30,0x30,0x30,0x30,0x30,0x00,0x00,0x00},
    {0x00,0x00,0x00,0x00,0x3c,0x64,0x70,0x3c,0x0e,0x46,0x3c,0x00,0x00,0x00},
    {0x00,0x00,0x30,0x30,0x7e,0x30,0x30,0x30,0x30,0x18,0x1e,0x00,0x00,0x00},
    {0x00,0x00,0x00,0x00,0x66,0x66,0x66,0x66,0x66,0x6e,0x3e,0x00,0x00,0x00},
    {0x00,0x00,0x00,0x00,0x66,0x66,0x66,0x2c,0x3c,0x3c,0x18,0x00,0x00,0x00},
    {0x00,0x00,0x00,0x00,0xc3,0xc3,0xda,0x5a,0x7e,0x6e,0x66,0x00,0x00,0x00},
    {0x00,0x00,0x00,0x00,0x66,0x3c,0x3c,0x18,0x3c,0x6c,0x66,0x00,0x00,0x00},
    {0x00,0x00,0x00,0x00,0x66,0x66,0x66,0x3c,0x3c,0x3c,0x18,0x18,0x38,0x70},
    {0x00,0x00,0x00,0x00,0x7e,0x0e,0x1c,0x18,0x30,0x70,0x7e,0x00,0x00,0x00},
    {0x0e,0x18,0x18,0x18,0x18,0x18,0x70,0x18,0x18,0x18,0x18,0x0e,0x00,0x00},
    {0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x00},
    {0x70,0x18,0x18,0x18,0x18,0x18,0x0e,0x18,0x18,0x18,0x18,0x70,0x00,0x00},
    {0x00,0x00,0x00,0x00,0x00,0x00,0x70,0x0e,0x00,0x00,0x00,0x00,0x00,0x00},
};

#define SCR_W        320
#define SCR_H        170
#define SEGS         16
#define SEG_PITCH    8
#define SEG_LIT      6
#define SPEC_TOP     24
#define SPEC_BOTTOM  (SPEC_TOP + SEGS * SEG_PITCH)      /* 152 */
#define BAR_PITCH    13
#define BAR_W        10
#define BAR_X0       ((SCR_W - (NBARS * BAR_PITCH - (BAR_PITCH - BAR_W))) / 2)
#define MB_TOP       158
#define MB_H         4

typedef struct {
    float h[NBARS];        /* lit segments, 0..SEGS */
    float heat[NBARS];     /* 0..1 */
    float level;           /* overall movement 0..1 */
    bool dim;              /* bars dimmed behind a message */
    bool muted;            /* page sound muted with the BOOT button */
    char top_left[24];
    char top_right[24];
    char msg1[24];         /* big line */
    char msg2[40];
    char msg3[40];
} view_t;

/* RGB565, bytes swapped for the 8-bit bus (high byte first). */
static inline uint16_t px565(int r, int g, int b)
{
    uint16_t c = (uint16_t)(((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3));
    return (uint16_t)((c >> 8) | (c << 8));
}

static inline float clampf(float v, float a, float b) { return v < a ? a : (v > b ? b : v); }

/* Same palette as the laptop page: calm -> warm -> hot. */
static void heat_rgb(float t, float bright, int *r, int *g, int *b)
{
    static const float CALM[3] = {124, 195, 207}, WARM[3] = {255, 180, 84}, HOT[3] = {255, 90, 110};
    const float *a = t < 0.5f ? CALM : WARM, *c = t < 0.5f ? WARM : HOT;
    float u = t < 0.5f ? t * 2.0f : (t - 0.5f) * 2.0f;
    *r = (int)clampf((a[0] + (c[0] - a[0]) * u) * bright, 0, 255);
    *g = (int)clampf((a[1] + (c[1] - a[1]) * u) * bright, 0, 255);
    *b = (int)clampf((a[2] + (c[2] - a[2]) * u) * bright, 0, 255);
}

static uint16_t heat_px(float t, float bright)
{
    int r, g, b;
    heat_rgb(t, bright, &r, &g, &b);
    return px565(r, g, b);
}

static void text_row(uint16_t *row, int y, int x, int top, const char *s, uint16_t col, int scale)
{
    if (y < top || y >= top + 14 * scale) {
        return;
    }
    int gy = (y - top) / scale;
    for (int i = 0; s[i]; i++) {
        unsigned char ch = (unsigned char)s[i];
        if (ch < 32 || ch > 126) {
            ch = '?';
        }
        uint8_t bits = FONT8X14[ch - 32][gy];
        if (!bits) {
            continue;
        }
        for (int gx = 0; gx < 8; gx++) {
            if (bits & (0x80 >> gx)) {
                for (int sx = 0; sx < scale; sx++) {
                    int px = x + (i * 8 + gx) * scale + sx;
                    if (px >= 0 && px < SCR_W) {
                        row[px] = col;
                    }
                }
            }
        }
    }
}

static int text_width(const char *s, int scale) { return (int)strlen(s) * 8 * scale; }

static void centered_line(uint16_t *row, int y, int top, const char *s, uint16_t fg, uint16_t bg, int scale)
{
    if (!s[0] || y < top - 4 || y >= top + 14 * scale + 4) {
        return;
    }
    int w = text_width(s, scale), x = (SCR_W - w) / 2;
    for (int px = x - 8; px < x + w + 8; px++) {      /* dark box so the text reads over the bars */
        if (px >= 0 && px < SCR_W) {
            row[px] = bg;
        }
    }
    text_row(row, y, x, top, s, fg, scale);
}

/* Draws rows y0 .. y0+h-1 of the whole screen into buf (SCR_W pixels per row). */
static void render_strip(uint16_t *buf, int y0, int h, const view_t *v)
{
    const uint16_t BG = px565(5, 8, 18), INK = px565(236, 232, 220), DIMTXT = px565(151, 161, 192);
    for (int y = y0; y < y0 + h; y++) {
        uint16_t *row = buf + (y - y0) * SCR_W;
        for (int x = 0; x < SCR_W; x++) {
            row[x] = BG;
        }

        /* the spectrum */
        if (y >= SPEC_TOP && y < SPEC_BOTTOM) {
            int from_bottom = SPEC_BOTTOM - 1 - y;
            int seg = from_bottom / SEG_PITCH;
            if (from_bottom % SEG_PITCH < SEG_LIT) {
                for (int b = 0; b < NBARS; b++) {
                    int lit = (int)(v->h[b] + 0.5f);
                    uint16_t col;
                    if (seg < lit) {
                        col = heat_px(v->heat[b], (v->dim ? 0.3f : 0.72f + 0.28f * seg / (SEGS - 1)));
                    } else {
                        col = heat_px(v->heat[b], 0.12f);      /* the faint unlit "LED" */
                    }
                    int x0 = BAR_X0 + b * BAR_PITCH;
                    for (int x = x0; x < x0 + BAR_W; x++) {
                        row[x] = col;
                    }
                }
            }
        }

        /* thin movement meter */
        if (y >= MB_TOP && y < MB_TOP + MB_H) {
            int x0 = BAR_X0, x1 = SCR_W - BAR_X0;
            int fill = x0 + (int)((x1 - x0) * clampf(v->level, 0, 1));
            uint16_t on = heat_px(v->level, 1.0f), off = px565(30, 36, 56);
            for (int x = x0; x < x1; x++) {
                row[x] = x < fill ? on : off;
            }
        }

        /* top line: IP address on the left, signal and readings on the right */
        text_row(row, y, BAR_X0, 3, v->top_left, DIMTXT, 1);
        text_row(row, y, SCR_W - BAR_X0 - text_width(v->top_right, 1), 3, v->top_right, DIMTXT, 1);
        if (v->muted) {
            text_row(row, y, (SCR_W - text_width("MUTED", 1)) / 2, 3, "MUTED", px565(255, 180, 84), 1);
        }

        /* messages */
        centered_line(row, y, 52, v->msg1, INK, BG, 2);
        centered_line(row, y, 96, v->msg2, DIMTXT, BG, 1);
        centered_line(row, y, 118, v->msg3, DIMTXT, BG, 1);
    }
}
// ===== DRAWING END =====

/* ---------------------------------------------------------------- CSI readings */
typedef struct {
    uint32_t seq;
    int8_t   rssi;
    uint8_t  first_word_invalid;
    uint16_t len;
    int8_t   buf[CSI_MAX_LEN];
} csi_item_t;

static QueueHandle_t s_queue;
static uint8_t s_bssid[6];
/* Settings sent by the laptop page (they reset to these defaults when the board restarts,
   and the page sends them again every few seconds). */
static volatile float s_smooth_keep = 0.67f;   /* noise filter: share of the old picture kept per frame */
static volatile float s_sens = 1.0f;           /* Sensitivity, 1/3 .. 3 (1 = middle of the slider) */
static volatile bool s_muted;                  /* page sound muted (state comes back from the page) */
static volatile uint8_t s_btn_seq;             /* counts BOOT button presses */

/* Called by the WiFi driver for every received packet that carries CSI. Keep it short. */
static void wifi_csi_rx_cb(void *ctx, wifi_csi_info_t *info)
{
    static csi_item_t item;     /* static is fine: this always runs in the WiFi task */
    static uint32_t seq;

    if (!info || !info->buf || info->len == 0) {
        return;
    }
    if (memcmp(info->mac, s_bssid, 6)) {
        return;                 /* only packets transmitted by our own WiFi router */
    }
    item.seq = seq++;
    item.rssi = info->rx_ctrl.rssi;
    item.first_word_invalid = info->first_word_invalid ? 1 : 0;
    item.len = info->len > CSI_MAX_LEN ? CSI_MAX_LEN : info->len;
    memcpy(item.buf, info->buf, item.len);
    xQueueSend(s_queue, &item, 0);   /* if the queue is full the reading is dropped */
}

static void wifi_csi_init(void)
{
    /* Same settings as Espressif's router example: the long training field only. */
    wifi_csi_config_t csi_config = {
        .lltf_en           = true,
        .htltf_en          = false,
        .stbc_htltf2_en    = false,
        .ltf_merge_en      = true,
        .channel_filter_en = true,
        .manu_scale        = true,
        .shift             = true,
    };
    ESP_ERROR_CHECK(esp_wifi_set_csi_config(&csi_config));
    ESP_ERROR_CHECK(esp_wifi_set_csi_rx_cb(wifi_csi_rx_cb, NULL));
    ESP_ERROR_CHECK(esp_wifi_set_csi(true));
}

/* Turns one reading into the screen's bars and movement level (same idea as the laptop page). */
#define WIN 20
static void process_reading(const csi_item_t *it)
{
    static int cur_len, warm, n_idx, win_n, win_pos;
    static float acc[CSI_MAX_LEN / 2];
    static uint8_t idx[CSI_MAX_LEN / 2];
    static float win[WIN][NBARS], base[NBARS];
    static bool have_base;
    static float floor_v, level;

    int nsc = it->len / 2;
    if (nsc < NBARS) {
        return;
    }
    float amp[CSI_MAX_LEN / 2];
    for (int i = 0; i < nsc; i++) {
        float im = it->buf[2 * i], re = it->buf[2 * i + 1];
        amp[i] = sqrtf(im * im + re * re);
    }
    if (it->first_word_invalid) {
        amp[0] = amp[1] = 0;
    }
    if (nsc != cur_len) {            /* different packet type: start learning again */
        cur_len = nsc;
        warm = n_idx = win_n = win_pos = 0;
        have_base = false;
        floor_v = level = 0;
        memset(acc, 0, sizeof(acc));
    }
    if (n_idx == 0) {                /* learn which frequencies carry signal */
        for (int i = 0; i < nsc; i++) {
            acc[i] += amp[i];
        }
        if (++warm < 40) {
            return;
        }
        float sorted[CSI_MAX_LEN / 2];
        int m = 0;
        for (int i = 0; i < nsc; i++) {
            float v = acc[i] / warm;
            if (v > 0.01f) {
                int j = m++;
                while (j > 0 && sorted[j - 1] > v) {
                    sorted[j] = sorted[j - 1];
                    j--;
                }
                sorted[j] = v;
            }
        }
        float median = m ? sorted[m / 2] : 0;
        for (int i = 0; i < nsc; i++) {
            if (acc[i] / warm > 0.3f * median) {
                idx[n_idx++] = (uint8_t)i;
            }
        }
        if (n_idx < NBARS) {
            n_idx = 0;
            warm = 0;
            memset(acc, 0, sizeof(acc));
        }
        return;
    }

    float mean = 0;
    for (int k = 0; k < n_idx; k++) {
        mean += amp[idx[k]];
    }
    mean /= n_idx;
    if (mean <= 0) {
        return;
    }
    float bins[NBARS];
    for (int b = 0; b < NBARS; b++) {
        int k0 = b * n_idx / NBARS, k1 = (b + 1) * n_idx / NBARS;
        float s = 0;
        for (int k = k0; k < k1; k++) {
            s += amp[idx[k]] / mean;    /* divide by the average: only the shape counts */
        }
        bins[b] = k1 > k0 ? s / (k1 - k0) : 0;
    }

    memcpy(win[win_pos], bins, sizeof(bins));
    win_pos = (win_pos + 1) % WIN;
    if (win_n < WIN) {
        win_n++;
    }
    if (win_n >= WIN) {
        float m = 0;
        for (int b = 0; b < NBARS; b++) {
            float mu = 0, var = 0;
            for (int j = 0; j < WIN; j++) {
                mu += win[j][b];
            }
            mu /= WIN;
            for (int j = 0; j < WIN; j++) {
                float d = win[j][b] - mu;
                var += d * d;
            }
            m += sqrtf(var / WIN);
        }
        m /= NBARS;
        /* floor_v = the wobble of an empty room: follows quickly down, slowly up */
        if (floor_v == 0) {
            floor_v = m;
        } else if (m < floor_v) {
            floor_v += (m - floor_v) * 0.1f;
        } else if (m < floor_v * 1.6f) {
            floor_v += (m - floor_v) * 0.004f;
        } else {
            floor_v += (m - floor_v) * 0.0001f;
        }
        if (floor_v < 1e-4f) {
            floor_v = 1e-4f;
        }
        float ratio = m / floor_v;
        level = 1.0f - expf(-fmaxf(0.0f, ratio - 1.6f) / 5.0f);
    }
    if (!have_base) {
        memcpy(base, bins, sizeof(bins));
        have_base = true;
    }
    float alpha = level < 0.12f ? 0.015f : 0.0015f;
    float heat[NBARS], hot[NBARS];
    float gate = 2.5f * floor_v / s_sens;          /* the change needed before a bar turns yellow */
    for (int b = 0; b < NBARS; b++) {
        float dev = fabsf(bins[b] - base[b]);
        base[b] += (bins[b] - base[b]) * alpha;
        heat[b] = floor_v > 0 ? clampf((dev - gate) / (4.8f * gate), 0, 1) : 0;
        hot[b] = heat[b] >= 0.1f ? heat[b] : 0;    /* below 0.1 it still looks blue */
    }
    /* Movement meter = how much colour the bars show (same rule as the page):
       mean of the most coloured 15 % (at least 3 bars), held briefly. */
    static float amount_env;
    for (int i = 1; i < NBARS; i++) {             /* sort descending (24 values) */
        float v = hot[i];
        int j = i;
        while (j > 0 && hot[j - 1] < v) {
            hot[j] = hot[j - 1];
            j--;
        }
        hot[j] = v;
    }
    const int top = NBARS * 15 / 100 > 3 ? NBARS * 15 / 100 : 3;
    float amount = 0;
    for (int i = 0; i < top; i++) {
        amount += hot[i];
    }
    amount /= top;
    amount_env = fmaxf(amount, amount_env * 0.94f);

    portENTER_CRITICAL(&s_disp_mux);
    memcpy(s_disp.bar, bins, sizeof(bins));
    memcpy(s_disp.heat, heat, sizeof(heat));
    s_disp.level = win_n >= WIN ? amount_env : 0;
    s_disp.have_shape = true;
    s_disp.readings++;
    s_disp.last_reading_us = esp_timer_get_time();
    portEXIT_CRITICAL(&s_disp_mux);
}

/* ---------------------------------------------------------------- saved WiFi settings */
/* Slot 0 = current network, slot 1 = previous network (the fallback). */
typedef struct {
    char ssid[33];
    char pass[65];
} cred_t;

static bool cred_load(int slot, cred_t *c)
{
    nvs_handle_t h;
    memset(c, 0, sizeof(*c));
    if (nvs_open("csiwifi", NVS_READONLY, &h) != ESP_OK) {
        return false;
    }
    char ks[8], kp[8];
    snprintf(ks, sizeof(ks), "ssid%d", slot);
    snprintf(kp, sizeof(kp), "pass%d", slot);
    size_t ls = sizeof(c->ssid), lp = sizeof(c->pass);
    bool ok = nvs_get_str(h, ks, c->ssid, &ls) == ESP_OK && c->ssid[0];
    if (ok && nvs_get_str(h, kp, c->pass, &lp) != ESP_OK) {
        c->pass[0] = 0;
    }
    nvs_close(h);
    return ok;
}

static void cred_store(const cred_t *cur, const cred_t *prev)
{
    nvs_handle_t h;
    if (nvs_open("csiwifi", NVS_READWRITE, &h) != ESP_OK) {
        return;
    }
    nvs_set_str(h, "ssid0", cur->ssid);
    nvs_set_str(h, "pass0", cur->pass);
    if (prev && prev->ssid[0]) {
        nvs_set_str(h, "ssid1", prev->ssid);
        nvs_set_str(h, "pass1", prev->pass);
    }
    nvs_commit(h);
    nvs_close(h);
}

static void current_cred(cred_t *c)
{
    if (!cred_load(0, c)) {          /* nothing saved yet: use the build-time settings */
        strlcpy(c->ssid, DEFAULT_SSID, sizeof(c->ssid));
        strlcpy(c->pass, DEFAULT_PASS, sizeof(c->pass));
    }
}

static void set_setup_flag(bool on)
{
    nvs_handle_t h;
    if (nvs_open("csiwifi", NVS_READWRITE, &h) == ESP_OK) {
        nvs_set_u8(h, "setup", on ? 1 : 0);
        nvs_commit(h);
        nvs_close(h);
    }
}

static bool get_setup_flag(void)
{
    nvs_handle_t h;
    uint8_t v = 0;
    if (nvs_open("csiwifi", NVS_READONLY, &h) == ESP_OK) {
        nvs_get_u8(h, "setup", &v);
        nvs_close(h);
    }
    return v == 1;
}

static void restart_into_setup(void)
{
    set_setup_flag(true);
    vTaskDelay(pdMS_TO_TICKS(300));
    esp_restart();
}

/* ---------------------------------------------------------------- WiFi connection */
#define BIT_GOT_IP  BIT0
#define BIT_DISC    BIT1
static EventGroupHandle_t s_wifi_events;
static volatile bool s_online;
static bool s_setup_mode;

static void wifi_event_handler(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_CONNECTED) {
        wifi_event_sta_connected_t *e = (wifi_event_sta_connected_t *)data;
        memcpy(s_bssid, e->bssid, 6);   /* readings are only taken from this router */
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        xEventGroupClearBits(s_wifi_events, BIT_GOT_IP);
        xEventGroupSetBits(s_wifi_events, BIT_DISC);
        if (s_online) {
            esp_wifi_connect();         /* once online, keep trying to come back */
        }
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *e = (ip_event_got_ip_t *)data;
        portENTER_CRITICAL(&s_disp_mux);
        snprintf(s_disp.ip, sizeof(s_disp.ip), IPSTR, IP2STR(&e->ip_info.ip));
        portEXIT_CRITICAL(&s_disp_mux);
        xEventGroupSetBits(s_wifi_events, BIT_GOT_IP);
    }
}

static bool try_network(const cred_t *c)
{
    if (!c->ssid[0]) {
        return false;
    }
    ESP_LOGI(TAG, "trying network \"%s\"", c->ssid);
    disp_set_state(ST_CONNECTING, c->ssid);

    wifi_config_t cfg = {0};
    strlcpy((char *)cfg.sta.ssid, c->ssid, sizeof(cfg.sta.ssid));
    strlcpy((char *)cfg.sta.password, c->pass, sizeof(cfg.sta.password));
    cfg.sta.threshold.authmode = c->pass[0] ? WIFI_AUTH_WPA_PSK : WIFI_AUTH_OPEN;
    cfg.sta.scan_method = WIFI_ALL_CHANNEL_SCAN;
    cfg.sta.sort_method = WIFI_CONNECT_AP_BY_SIGNAL;   /* strongest box if several share the name */
    cfg.sta.pmf_cfg.capable = true;
    cfg.sta.sae_pwe_h2e = WPA3_SAE_PWE_BOTH;
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &cfg));

    int64_t deadline = esp_timer_get_time() + CONNECT_TIMEOUT_MS * 1000LL;
    while (esp_timer_get_time() < deadline) {
        xEventGroupClearBits(s_wifi_events, BIT_GOT_IP | BIT_DISC);
        esp_wifi_connect();
        int left_ms = (int)((deadline - esp_timer_get_time()) / 1000);
        EventBits_t bits = xEventGroupWaitBits(s_wifi_events, BIT_GOT_IP | BIT_DISC, pdFALSE, pdFALSE,
                                               pdMS_TO_TICKS(left_ms > 0 ? left_ms : 1));
        if (bits & BIT_GOT_IP) {
            ESP_LOGI(TAG, "connected to \"%s\"", c->ssid);
            return true;
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    ESP_LOGW(TAG, "could not join \"%s\"", c->ssid);
    esp_wifi_disconnect();
    vTaskDelay(pdMS_TO_TICKS(300));
    return false;
}

static void start_station(void)
{
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t init = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&init));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));    /* our own copy lives in "csiwifi" */
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    esp_wifi_set_ps(WIFI_PS_NONE);      /* keep the radio awake: steadier readings */
}

static void start_setup_ap(void)
{
    s_setup_mode = true;
    esp_netif_create_default_wifi_ap();
    wifi_init_config_t init = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&init));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    wifi_config_t ap = {0};
    strlcpy((char *)ap.ap.ssid, SETUP_AP_SSID, sizeof(ap.ap.ssid));
    ap.ap.ssid_len = strlen(SETUP_AP_SSID);
    ap.ap.channel = 6;
    ap.ap.authmode = WIFI_AUTH_OPEN;
    ap.ap.max_connection = 2;
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap));
    ESP_ERROR_CHECK(esp_wifi_start());
    portENTER_CRITICAL(&s_disp_mux);
    strlcpy(s_disp.ip, "192.168.4.1", sizeof(s_disp.ip));
    portEXIT_CRITICAL(&s_disp_mux);
    disp_set_state(ST_SETUP, SETUP_AP_SSID);
    ESP_LOGI(TAG, "setup mode: join the WiFi \"%s\" and open the page", SETUP_AP_SSID);
}

/* ---------------------------------------------------------------- talking to the laptop */
typedef struct {
    int sock;
    struct sockaddr_in peer;
    bool have_peer;
    int64_t peer_seen;
    int64_t last_rx;
    int credit;
} link_t;

static void handle_set_wifi(link_t *l, const char *in, int n, struct sockaddr_in *from, socklen_t from_len)
{
    /* "SETW", name length, name, password length, password */
    const char *why = NULL;
    int sl = n > 4 ? (uint8_t)in[4] : 0;
    int pl = (n > 5 + sl) ? (uint8_t)in[5 + sl] : -1;
    if (sl < 1 || sl > 32) {
        why = "network name must be 1 to 32 characters";
    } else if (pl < 0 || 6 + sl + pl > n) {
        why = "message was cut";
    } else if (pl != 0 && (pl < 8 || pl > 63)) {
        why = "password must be empty or 8 to 63 characters";
    }
    if (why) {
        char out[80];
        int len = snprintf(out, sizeof(out), "SETW-ERR %s", why);
        sendto(l->sock, out, len, 0, (struct sockaddr *)from, from_len);
        return;
    }
    cred_t next = {0}, cur;
    memcpy(next.ssid, in + 5, sl);
    memcpy(next.pass, in + 6 + sl, pl);
    current_cred(&cur);
    bool same = !strcmp(cur.ssid, next.ssid) && !strcmp(cur.pass, next.pass);
    /* the network it was on becomes the fallback */
    cred_store(&next, (same || !cur.ssid[0]) ? NULL : &cur);
    sendto(l->sock, "SETW-OK", 7, 0, (struct sockaddr *)from, from_len);
    ESP_LOGI(TAG, "new network \"%s\" saved, restarting", next.ssid);
    disp_set_state(ST_SWITCHING, next.ssid);
    set_setup_flag(false);
    vTaskDelay(pdMS_TO_TICKS(1200));
    esp_restart();
}

/* Read every waiting packet from the laptop. */
static void drain_incoming(link_t *l)
{
    char in[128];
    for (;;) {
        struct sockaddr_in from;
        socklen_t from_len = sizeof(from);
        int n = recvfrom(l->sock, in, sizeof(in), MSG_DONTWAIT, (struct sockaddr *)&from, &from_len);
        if (n < 0) {
            return;
        }
        l->last_rx = esp_timer_get_time();
        if (n >= 4 && memcmp(in, "CSI?", 4) == 0) {
            /* "Is there a CSI board here?" -> "Yes, on this network." */
            char out[40];
            portENTER_CRITICAL(&s_disp_mux);
            int len = snprintf(out, sizeof(out), "CSI!%s", s_disp.ssid);
            portEXIT_CRITICAL(&s_disp_mux);
            sendto(l->sock, out, len, 0, (struct sockaddr *)&from, from_len);
            continue;
        }
        if (n >= 4 && memcmp(in, "SETW", 4) == 0) {
            handle_set_wifi(l, in, n, &from, from_len);
            continue;
        }
        if (n == 6 && memcmp(in, "SMT2", 4) == 0) {
            /* noise filter as a settling time in milliseconds (0 = off, up to 10 s) */
            int ms = (uint8_t)in[4] | ((uint8_t)in[5] << 8);
            if (ms > 10000) {
                ms = 10000;
            }
            s_smooth_keep = ms <= 0 ? 0.0f : expf(-1.0f / (25.0f * ms / 1000.0f));   /* 25 frames/s */
            continue;
        }
        if (n == 5 && memcmp(in, "SENS", 4) == 0) {
            /* Sensitivity slider position: 0..200 = left..right, 100 = middle */
            int v = (uint8_t)in[4];
            if (v > 200) {
                v = 200;
            }
            s_sens = powf(3.0f, (v - 100) / 100.0f);
            continue;
        }
        if (n == 5 && memcmp(in, "MUTE", 4) == 0) {
            s_muted = in[4] != 0;                    /* shown as MUTED on the screen */
            continue;
        }
        if (n == 5 && memcmp(in, "SMTH", 4) == 0) {
            int v = (uint8_t)in[4];                  /* older page versions */
            s_smooth_keep = (v > 95 ? 95 : v) / 100.0f;
            continue;
        }
        /* anything else is a poke: refresh the peer and add one credit */
        l->peer = from;
        l->have_peer = true;
        l->peer_seen = l->last_rx;
        if (l->credit < MAX_CREDIT) {
            l->credit++;
        }
    }
}

static int open_socket(void)
{
    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock < 0) {
        return -1;
    }
    struct sockaddr_in addr = {
        .sin_family = AF_INET,
        .sin_port = htons(UDP_PORT),
        .sin_addr.s_addr = htonl(INADDR_ANY),
    };
    if (bind(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        close(sock);
        return -1;
    }
    return sock;
}

static void udp_task(void *arg)
{
    static csi_item_t item;
    static uint8_t out[HEADER_LEN + CSI_MAX_LEN];
    link_t l = { .sock = -1 };
    uint32_t sent = 0, seen = 0;
    int64_t now = esp_timer_get_time();
    int64_t start = now, last_report = now, last_link_ok = now, last_rssi = 0;
    uint8_t btn_sent = s_btn_seq;
    int btn_repeats = 0;

    for (;;) {
        if (l.sock < 0) {
            l.sock = open_socket();
            if (l.sock < 0) {
                ESP_LOGE(TAG, "cannot open UDP port %d, retrying", UDP_PORT);
                vTaskDelay(pdMS_TO_TICKS(1000));
                continue;
            }
            ESP_LOGI(TAG, "listening on UDP port %d", UDP_PORT);
        }

        drain_incoming(&l);

        if (xQueueReceive(s_queue, &item, pdMS_TO_TICKS(10)) == pdTRUE) {
            seen++;
            process_reading(&item);
            drain_incoming(&l);     /* the poke that caused this reading may just have arrived */
            if (l.have_peer && l.credit > 0) {
                l.credit--;
                memcpy(out, "CSI1", 4);
                out[4] = item.seq & 0xff;
                out[5] = (item.seq >> 8) & 0xff;
                out[6] = (item.seq >> 16) & 0xff;
                out[7] = (item.seq >> 24) & 0xff;
                out[8] = (uint8_t)item.rssi;
                out[9] = item.first_word_invalid;
                out[10] = item.len & 0xff;
                out[11] = (item.len >> 8) & 0xff;
                memcpy(out + HEADER_LEN, item.buf, item.len);
                if (sendto(l.sock, out, HEADER_LEN + item.len, 0,
                           (struct sockaddr *)&l.peer, sizeof(l.peer)) >= 0) {
                    sent++;
                }
            }
        }

        /* BOOT button pressed: tell the page (3 copies, the page ignores repeats) */
        if (s_btn_seq != btn_sent) {
            btn_sent = s_btn_seq;
            btn_repeats = l.have_peer ? 3 : 0;
        }
        if (btn_repeats > 0) {
            uint8_t msg[5] = {'B', 'T', 'N', '0', btn_sent};
            sendto(l.sock, msg, sizeof(msg), 0, (struct sockaddr *)&l.peer, sizeof(l.peer));
            btn_repeats--;
        }

        now = esp_timer_get_time();
        if (l.have_peer && now - l.peer_seen > PEER_TIMEOUT_US) {
            l.have_peer = false;
            l.credit = 0;
            ESP_LOGI(TAG, "laptop went quiet, waiting for it to come back");
        }

        if (s_setup_mode) {
            if (now - start > SETUP_MIN_US && now - l.last_rx > SETUP_IDLE_US) {
                ESP_LOGI(TAG, "nobody came to set up, trying the known networks again");
                esp_restart();
            }
            continue;
        }

        if (now - last_rssi >= 1000000) {
            wifi_ap_record_t ap;
            if (esp_wifi_sta_get_ap_info(&ap) == ESP_OK) {
                last_link_ok = now;
                portENTER_CRITICAL(&s_disp_mux);
                s_disp.rssi = ap.rssi;
                portEXIT_CRITICAL(&s_disp_mux);
            } else if (now - last_link_ok > LINK_TIMEOUT_US) {
                ESP_LOGE(TAG, "WiFi lost for 30 s, restarting");
                esp_restart();
            }
            last_rssi = now;
        }

        if (now - last_report >= REPORT_PERIOD_US) {
            ESP_LOGI(TAG, "readings/s: %u seen, %u sent to laptop", (unsigned)(seen / 5), (unsigned)(sent / 5));
            sent = 0;
            seen = 0;
            last_report = now;
        }
    }
}

/* ---------------------------------------------------------------- the screen */
static SemaphoreHandle_t s_lcd_free;
static esp_lcd_panel_handle_t s_panel;
#define STRIP_H 17      /* the screen is drawn in 10 strips of 17 rows */

static bool lcd_done_cb(esp_lcd_panel_io_handle_t io, esp_lcd_panel_io_event_data_t *e, void *ctx)
{
    BaseType_t woken = pdFALSE;
    xSemaphoreGiveFromISR(s_lcd_free, &woken);
    return woken == pdTRUE;
}

static void lcd_init(void)
{
    gpio_config_t out = {
        .pin_bit_mask = (1ULL << PIN_POWER_ON) | (1ULL << PIN_LCD_RD) | (1ULL << PIN_LCD_BL),
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&out);
    gpio_set_level(PIN_POWER_ON, 1);    /* needed for the screen when running on battery */
    gpio_set_level(PIN_LCD_RD, 1);
    gpio_set_level(PIN_LCD_BL, 0);

    gpio_config_t in = {
        .pin_bit_mask = (1ULL << PIN_BUTTON_2) | (1ULL << PIN_BUTTON_BOOT),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };
    gpio_config(&in);

    s_lcd_free = xSemaphoreCreateCounting(2, 2);

    esp_lcd_i80_bus_handle_t bus = NULL;
    esp_lcd_i80_bus_config_t bus_cfg = {
        .dc_gpio_num = PIN_LCD_DC,
        .wr_gpio_num = PIN_LCD_WR,
        .clk_src = LCD_CLK_SRC_DEFAULT,
        .bus_width = 8,
        .max_transfer_bytes = SCR_W * STRIP_H * sizeof(uint16_t),
        .dma_burst_size = 64,
    };
    for (int i = 0; i < 8; i++) {
        bus_cfg.data_gpio_nums[i] = LCD_DATA_PINS[i];
    }
    ESP_ERROR_CHECK(esp_lcd_new_i80_bus(&bus_cfg, &bus));

    esp_lcd_panel_io_handle_t io = NULL;
    esp_lcd_panel_io_i80_config_t io_cfg = {
        .cs_gpio_num = PIN_LCD_CS,
        .pclk_hz = LCD_PCLK_HZ,
        .trans_queue_depth = 10,
        .on_color_trans_done = lcd_done_cb,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
        .dc_levels = { .dc_idle_level = 0, .dc_cmd_level = 0, .dc_dummy_level = 0, .dc_data_level = 1 },
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i80(bus, &io_cfg, &io));

    esp_lcd_panel_dev_config_t panel_cfg = {
        .reset_gpio_num = PIN_LCD_RES,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
        .bits_per_pixel = 16,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(io, &panel_cfg, &s_panel));
    esp_lcd_panel_reset(s_panel);
    esp_lcd_panel_init(s_panel);
    esp_lcd_panel_invert_color(s_panel, true);
    esp_lcd_panel_swap_xy(s_panel, true);
    esp_lcd_panel_mirror(s_panel, false, true);    /* landscape, USB connector on the left */
    esp_lcd_panel_set_gap(s_panel, 0, 35);

    /* Extra start-up commands LilyGo uses for the newer version of this screen. */
    static const struct { uint8_t cmd; uint8_t data[14]; uint8_t len; } init_cmds[] = {
        {0x11, {0}, 0x80},
        {0x3A, {0x05}, 1},
        {0xB2, {0x0B, 0x0B, 0x00, 0x33, 0x33}, 5},
        {0xB7, {0x75}, 1},
        {0xBB, {0x28}, 1},
        {0xC0, {0x2C}, 1},
        {0xC2, {0x01}, 1},
        {0xC3, {0x1F}, 1},
        {0xC6, {0x13}, 1},
        {0xD0, {0xA7}, 1},
        {0xD0, {0xA4, 0xA1}, 2},
        {0xD6, {0xA1}, 1},
        {0xE0, {0xF0, 0x05, 0x0A, 0x06, 0x06, 0x03, 0x2B, 0x32, 0x43, 0x36, 0x11, 0x10, 0x2B, 0x32}, 14},
        {0xE1, {0xF0, 0x08, 0x0C, 0x0B, 0x09, 0x24, 0x2B, 0x22, 0x43, 0x38, 0x15, 0x16, 0x2F, 0x37}, 14},
    };
    for (size_t i = 0; i < sizeof(init_cmds) / sizeof(init_cmds[0]); i++) {
        esp_lcd_panel_io_tx_param(io, init_cmds[i].cmd, init_cmds[i].data, init_cmds[i].len & 0x7f);
        if (init_cmds[i].len & 0x80) {
            vTaskDelay(pdMS_TO_TICKS(120));
        }
    }
    esp_lcd_panel_disp_on_off(s_panel, true);
}

static void screen_task(void *arg)
{
    lcd_init();
    uint16_t *bufs[2];
    for (int i = 0; i < 2; i++) {
        bufs[i] = heap_caps_malloc(SCR_W * STRIP_H * sizeof(uint16_t), MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
        if (!bufs[i]) {
            ESP_LOGE(TAG, "not enough memory for the screen");
            vTaskDelete(NULL);
        }
    }

    static view_t v;
    static float dh[NBARS], hs[NBARS], sm[NBARS];
    float lo = 0.6f, hi = 1.4f, lvl = 0;
    bool scale_ready = false, backlight = true, first = true, setup_requested = false;
    uint32_t last_count = 0;
    int64_t last_rate_t = esp_timer_get_time(), press_start = 0, boot_last = 0;
    bool boot_prev = false;
    int rate = 0, which = 0;
    TickType_t wake = xTaskGetTickCount();

    for (;;) {
        disp_t d;
        portENTER_CRITICAL(&s_disp_mux);
        d = s_disp;
        portEXIT_CRITICAL(&s_disp_mux);
        int64_t now = esp_timer_get_time();

        if (now - last_rate_t >= 1000000) {
            rate = (int)((d.readings - last_count) * 1000000LL / (now - last_rate_t));
            last_count = d.readings;
            last_rate_t = now;
        }
        bool live = d.state == ST_ONLINE && d.have_shape && now - d.last_reading_us < 2000000;

        /* Bars. A bar that is calm (blue) is smoothed to hide the noise flicker; as soon as
           it shows any change (orange or red) it follows the readings directly. */
        if (live) {
            float mn = d.bar[0], mx = d.bar[0];
            for (int b = 1; b < NBARS; b++) {
                mn = fminf(mn, d.bar[b]);
                mx = fmaxf(mx, d.bar[b]);
            }
            if (!scale_ready) {
                lo = mn;
                hi = mx;
                scale_ready = true;
            } else {
                lo += (mn - lo) * 0.05f;
                hi += (mx - hi) * 0.05f;
            }
        }
        float range = fmaxf(hi - lo, 0.05f);
        float keep = s_smooth_keep;
        for (int b = 0; b < NBARS; b++) {
            if (!live) {
                sm[b] = d.bar[b];
            } else if (d.heat[b] > 0.0f) {
                sm[b] = d.bar[b];                           /* any orange: show it as it is */
            } else {
                sm[b] += (d.bar[b] - sm[b]) * (1.0f - keep); /* blue: smooth out the noise */
            }
            float target = live ? clampf(1.0f + (sm[b] - lo) / range * (SEGS - 1.5f), 0, SEGS) : 0;
            dh[b] = fmaxf(dh[b] - 0.6f, target);
            hs[b] += ((live ? d.heat[b] : 0) - hs[b]) * 0.3f;
            v.h[b] = dh[b];
            v.heat[b] = hs[b];
        }
        lvl += ((live ? d.level : 0) - lvl) * 0.25f;
        v.level = lvl;

        /* texts */
        strlcpy(v.top_left, d.ip, sizeof(v.top_left));
        v.top_right[0] = v.msg1[0] = v.msg2[0] = v.msg3[0] = 0;
        v.dim = false;
        switch (d.state) {
        case ST_BOOT:
            strlcpy(v.msg1, "Starting", sizeof(v.msg1));
            break;
        case ST_CONNECTING:
            strlcpy(v.msg1, "Connecting", sizeof(v.msg1));
            strlcpy(v.msg2, d.ssid, sizeof(v.msg2));
            break;
        case ST_SWITCHING:
            strlcpy(v.msg1, "Switching to", sizeof(v.msg1));
            strlcpy(v.msg2, d.ssid, sizeof(v.msg2));
            break;
        case ST_SETUP:
            strlcpy(v.msg1, "Setup mode", sizeof(v.msg1));
            strlcpy(v.msg2, "Join the WiFi " SETUP_AP_SSID, sizeof(v.msg2));
            strlcpy(v.msg3, "then open the page", sizeof(v.msg3));
            break;
        case ST_ONLINE:
            snprintf(v.top_right, sizeof(v.top_right), "%d dBm  %d/s", d.rssi, rate);
            if (!live) {
                strlcpy(v.msg1, "Waiting for", sizeof(v.msg1));
                strlcpy(v.msg2, "the page on the laptop", sizeof(v.msg2));
                strlcpy(v.msg3, d.ssid, sizeof(v.msg3));
            }
            break;
        }

        /* BOOT button: each press asks the page to mute or unmute */
        bool boot = gpio_get_level(PIN_BUTTON_BOOT) == 0;
        if (boot && !boot_prev && now - boot_last > 250000) {
            s_btn_seq++;
            boot_last = now;
        }
        boot_prev = boot;
        v.muted = s_muted && d.state == ST_ONLINE;

        /* second button: short press = screen on/off, hold 3 s = setup mode */
        bool pressed = gpio_get_level(PIN_BUTTON_2) == 0;
        if (pressed && !press_start) {
            press_start = now;
        }
        if (pressed && now - press_start > 1000000) {
            backlight = true;
            if (now - press_start > 3000000) {
                strlcpy(v.msg1, "Setup mode", sizeof(v.msg1));
                strlcpy(v.msg2, "restarting...", sizeof(v.msg2));
                v.msg3[0] = 0;
                setup_requested = true;
            } else {
                strlcpy(v.msg1, "Keep holding", sizeof(v.msg1));
                strlcpy(v.msg2, "for setup mode", sizeof(v.msg2));
                v.msg3[0] = 0;
            }
        }
        if (!pressed && press_start) {
            if (now - press_start < 1000000) {
                backlight = !backlight;
            }
            press_start = 0;
        }
        v.dim = v.msg1[0] != 0;

        if (backlight) {
            for (int y = 0; y < SCR_H; y += STRIP_H) {
                int h = SCR_H - y < STRIP_H ? SCR_H - y : STRIP_H;
                xSemaphoreTake(s_lcd_free, portMAX_DELAY);
                render_strip(bufs[which], y, h, &v);
                esp_lcd_panel_draw_bitmap(s_panel, 0, y, SCR_W, y + h, bufs[which]);
                which ^= 1;
            }
        }
        if (first) {
            gpio_set_level(PIN_LCD_BL, 1);  /* light up only once there is a picture */
            first = false;
        } else {
            gpio_set_level(PIN_LCD_BL, backlight ? 1 : 0);
        }
        if (setup_requested) {
            vTaskDelay(pdMS_TO_TICKS(500));
            restart_into_setup();
        }
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(40));
    }
}

/* ---------------------------------------------------------------- start-up */
void app_main(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    xTaskCreatePinnedToCore(screen_task, "screen", 6144, NULL, 3, NULL, 1);

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    s_wifi_events = xEventGroupCreate();
    s_queue = xQueueCreate(16, sizeof(csi_item_t));

    if (get_setup_flag()) {
        set_setup_flag(false);
        start_setup_ap();
        xTaskCreate(udp_task, "csi_udp", 4096, NULL, 5, NULL);
        return;
    }

    start_station();
    cred_t cur, prev;
    current_cred(&cur);
    bool have_prev = cred_load(1, &prev) && strcmp(prev.ssid, cur.ssid) != 0;

    bool ok = try_network(&cur);
    if (!ok && have_prev) {
        ok = try_network(&prev);
        if (ok) {
            cred_store(&prev, &cur);    /* the previous network works: make it the current one again */
        }
    }
    if (!ok) {
        ESP_LOGW(TAG, "no known network works, opening setup mode");
        restart_into_setup();
    }

    s_online = true;
    disp_set_state(ST_ONLINE, NULL);
    char ip[16];
    portENTER_CRITICAL(&s_disp_mux);
    strlcpy(ip, s_disp.ip, sizeof(ip));
    portEXIT_CRITICAL(&s_disp_mux);
    ESP_LOGI(TAG, "==== board IP address: %s ====", ip);
    wifi_csi_init();
    xTaskCreate(udp_task, "csi_udp", 4096, NULL, 5, NULL);
}
