#pragma once

// Maximum number of scancode instructions (1 instruction = 1 byte)
#define SCANCODE_SEND_LIMIT 100
// Maximum number of RGB LED instructions (1 instruction = 1 byte)
#define LED_INSTRUCTION_LIMIT 100
#define TAPPING_TERM 180
#define BOOTLDR_HOLD_TIME 2000
#define QUOT_HOLD_TIME 240
// rate of backspace being spammed
#define BACKSPACE_SPAM_DELAY 50
#define BACKSPACE_HOLD_THRESHOLD 400

#define RGB_MATRIX_LED_PROCESS_LIMIT 12
#define RGB_MATRIX_STARTUP_MODE RGB_MATRIX_CUSTOM_splash_jelly

// hsl(158,75%,50%)

#define RGB_MATRIX_STARTUP_HUE 158
#define RGB_MATRIX_STARTUP_SAT 186
#define RGB_MATRIX_STARTUP_VAL 120

#define RGBLIGHT_DEFAULT_HUE 158
#define RGBLIGHT_DEFAULT_SAT 148
#define RGBLIGHT_DEFAULT_VAL 0

#define NO_ACTION_TAPPING // save some space, not using this anyways.

#define MOUSEKEY_DELAY 0
#define MOUSEKEY_MOVE_DELTA 4
#define MOUSEKEY_INTERVAL 12
#define MOUSEKEY_MAX_SPEED 3
#define MOUSEKEY_TIME_TO_MAX 30
#define MOUSEKEY_WHEEL_DELAY 100
#define MOUSEKEY_WHEEL_INTERVAL 40
#define MOUSEKEY_WHEEL_MAX_SPEED 3
#define MOUSEKEY_WHEEL_TIME_TO_MAX 50


// Special scancodes for send_scancodes
// Note
#define CTLDOWN 0xF0
#define SFTDOWN 0xF1
#define ALTDOWN 0xF2
#define CTLUP 0xD0
#define SFTUP 0xD1
#define ALTUP 0xD2
#define LBRACE 0xAF
#define RBRACE 0xB0
#define SHIFT(x) (x | 0x80)

// LED OPCODES

// [1 byte] This command does nothing.
// Set a command to this value to reserve space for conditional LEDs that
// might be memcpy'd into this position later.
#define LED_NOOP 0xFF
// [1 byte] This command denotes the end of the LED processing.
// Always end a LED instruction list with LED_TERMINATE, otherwise
// it would continue operating on garbage values and have unexpected behavior.
#define LED_TERMINATE 0xFE
// [4 bytes] This command stores this RGB value for use in subsequent LEDs.
#define LED_SETCOLOR(r, g, b) 0xF1, r, g, b
// [4 bytes] This command stores this RGB value, and sets LED at index K to this color.
#define LED_SETCOLOR_ON(k, r, g, b) (k + 0x50), r, g, b
// [3 bytes] This command sets LEDs ranging from k1 to k2 (inclusive) to the stored RGB color.
#define LED_RANGE(k1, k2) 0xF2, k1, k2

// Qwerty layout LED indices
#define LED_QW_Q 1
#define LED_QW_W 2
#define LED_QW_E 3
#define LED_QW_R 4
#define LED_QW_T 5
#define LED_QW_Y 6
#define LED_QW_U 7
#define LED_QW_I 8
#define LED_QW_O 9
#define LED_QW_P 10
#define LED_QW_A 13
#define LED_QW_S 14
#define LED_QW_D 15
#define LED_QW_F 16
#define LED_QW_G 17
#define LED_QW_H 18
#define LED_QW_J 19
#define LED_QW_K 20
#define LED_QW_L 21
#define LED_QW_SCLN 22
#define LED_QW_Z 25
#define LED_QW_X 26
#define LED_QW_C 27
#define LED_QW_V 28
#define LED_QW_B 29
#define LED_QW_N 30
#define LED_QW_M 31
