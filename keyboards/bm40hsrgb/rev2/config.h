/* Copyright 2022 bdtc123
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#pragma once

#include "config_common.h"

/* USB Device descriptor parameter */
#define VENDOR_ID       0x4B50
#define PRODUCT_ID      0x1141
#define DEVICE_VER      0x0002
#define MANUFACTURER    KP Republic
#define PRODUCT         BM40V2

/* key matrix size */
#define MATRIX_ROWS 4
#define MATRIX_COLS 12

/*
 * Keyboard Matrix Assignments
 *
 * Change this to how you wired your keyboard
 * COLS: AVR pins used for columns, left to right
 * ROWS: AVR pins used for rows, top to bottom
 * DIODE_DIRECTION: COL2ROW = COL = Anode (+), ROW = Cathode (-, marked on diode)
 *                  ROW2COL = ROW = Anode (+), COL = Cathode (-, marked on diode)
 *
 */
#define MATRIX_ROW_PINS { D7, F7, F6, F5 }
#define MATRIX_COL_PINS { B2, B3, D5, D3, D2, B7, F0, B4, B5, B6, C6, C7 }

/* COL2ROW, ROW2COL*/
#define DIODE_DIRECTION ROW2COL

/* Debounce reduces chatter (unintended double-presses) - set 0 if debouncing is not needed */
#define DEBOUNCE 5

/* Keyboard boots with NKRO on */
#define FORCE_NKRO

//rgb light setting
#define RGBLIGHT_LIMIT_VAL 255
#define RGBLED_NUM          6
#define RGB_DI_PIN          B0
#define RGB_MATRIX_HUE_STEP   16
#define RGB_MATRIX_SAT_STEP   32
#define RGB_MATRIX_VAL_STEP   10
#define RGBLIGHT_DISABLE_KEYCODES

#define RGB_MATRIX_KEYPRESSES

#define RGB_DISABLE_WHEN_USB_SUSPENDED // turn off effects when suspended
#define RGB_MATRIX_MAXIMUM_BRIGHTNESS 180 // Limit to vendor-recommended value
// RGB Matrix Animation modes. Explicitly enabled
// For full list of effects, see:
// https://docs.qmk.fm/#/feature_rgb_matrix?id=rgb-matrix-effects

//some effects disabled arbitarily to save space

#   define ENABLE_RGB_MATRIX_JELLYBEAN_RAINDROPS

#define RGB_MATRIX_FRAMEBUFFER_EFFECTS
#   define ENABLE_RGB_MATRIX_TYPING_HEATMAP
#   define RGB_MATRIX_TYPING_HEATMAP_SPREAD 0
#   define RGB_MATRIX_TYPING_HEATMAP_DECREASE_DELAY_MS 24

#define RGB_MATRIX_KEYPRESSES
#   define ENABLE_RGB_MATRIX_SOLID_REACTIVE_SIMPLE
#   define ENABLE_RGB_MATRIX_SOLID_REACTIVE_MULTIWIDE
#   define ENABLE_RGB_MATRIX_SOLID_REACTIVE_MULTICROSS
#   define ENABLE_RGB_MATRIX_MULTISPLASH
#   define ENABLE_RGB_MATRIX_SOLID_MULTISPLASH


#   define DRIVER_ADDR_1 0b1010000
#   define DRIVER_COUNT 2
#   define DRIVER_1_LED_TOTAL 47
#   define DRIVER_LED_TOTAL DRIVER_1_LED_TOTAL
