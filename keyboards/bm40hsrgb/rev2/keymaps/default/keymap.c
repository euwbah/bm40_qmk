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
#include QMK_KEYBOARD_H
#include "config.h" // so that intellisense works.
#include "lib/lib8tion/lib8tion.h"

// Contains what keycodes to send when send_scancodes() is called.
static uint8_t* scancodes;
// Contains led opcode instructions to send when update_led() is called.
static uint8_t* leds;

static uint8_t backlights[6] = {0, 0, 0, 0, 0, 0};

void keyboard_post_init_user(void) {
    // initialize pointers once and keep them in heap.
    scancodes = malloc(SCANCODE_SEND_LIMIT * sizeof(uint8_t));
    leds = malloc(LED_INSTRUCTION_LIMIT * sizeof(uint8_t));

    rgblight_sethsv(0, 0, 0);
}

/*
Send a string of scancodes

only basic scancodes 0x00-0x7F are supported,
if SHIFT(x) shorthand is used, only supports scancodes 0x00-0x4F (no arrow keys/keypad)

by default, calls tap_code

prefix a scancode with 0xFE to call register_code with the following scancode
prefix a scancode with 0xFF to call unregister_code with the following scancode

Special codes

0x03 (unused):      will be ignored (helpful to leave blank spaces)
                    within an array which allows for if statements to conditionally fill them
                    in and save even more ROM.

0xF0 (LCTL + 0x10): Will hold down LCTL
0xF1 (LSFT + 0x10): Will hold down LSFT
0xF2 (LALT + 0x10): Will hold down LALT

0xD0 (LCTL - 0x10): Will release LCTL
0xD1 (LSFT - 0x10): Will release LSFT
0xD2 (LALT - 0x10): Will release LALT

0xAF (LBRC + 0x80): Will tap '{'
0xB0 (RBRC + 0x80): Will tap '}'
Any keycode + 0x80: will be equal to shift + keycode

Yes, there is send_string, but ROM is precious...
*/
void send_scancodes(void) {
    // -1 if following scan code should be register_code
    // 1 if following scan code should be unregister_code
    // 0 if following scan code should be tap_code
    int8_t send_type = 0;

    // set a hard limit for how many scancodes to be processed at maximum
    // to prevent an infinite loop in case of bad code that didn't end with null terminator.
    for(uint8_t i = 0; i < SCANCODE_SEND_LIMIT; i++) {
        uint8_t curr = *(scancodes + i);
        if(!curr) {
            break;
        } else if (curr == 254) {
            send_type = -1;
        } else if (curr == 255) {
            send_type = 1;
        } else if (curr >= 0xF0) {
            // Handle holding down modifier keys
            register_code(curr - 0x10);
        } else if ((curr & 0xD0) == 0xD0) {
            // handle releasing modifier keys
            unregister_code(curr + 0x10);
        } else if (curr >= 0x80) {
            // handle open/close brace { }
            register_code(KC_LSFT);
            tap_code(curr - 0x80);
            unregister_code(KC_LSFT);
        } else if (curr != 3) {
            switch (send_type) {
                case -1:
                    register_code(curr);
                    break;
                case 1:
                    unregister_code(curr);
                    break;
                default:
                    tap_code(curr);
                    break;
            }
            send_type = 0;
        }
    }
}

void set_led_color(uint8_t i, uint8_t r, uint8_t g, uint8_t b) {
    uint8_t val = dim8_video(rgb_matrix_get_val());
    rgb_matrix_set_color(i, r * val/255, g * val/255, b * val/255);
}

void set_all_backlights(uint8_t val) {
    memset(backlights, val, sizeof(backlights));
}

/*
Updates LED matrix using a custom instruction set to preserve memory.

Instructions:

0xFF: NOOP

0xF0: Terminate

0xF1 0xRR 0xGG 0xBB: Store color RGB

0 - 60 (0 - 0x3C): Set LED indexed by instruction value to stored color

80 - 140 (0x50 - 0x8C) 0xRR 0xGG 0xBB: Store color RGB, then set LED indexed by instruction value - 80

0xF2 0xXX 0xYY: Set LEDs ranging from index XX to YY to stored color
*/

void update_led(void) {
    uint8_t RED = 0, GREEN = 0, BLUE = 0;
    for(uint8_t i = 0; i < LED_INSTRUCTION_LIMIT; i++) {
        uint8_t inst = leds[i];
        switch(inst) {
            case LED_NOOP:
                break;
            case LED_TERMINATE:
                return;
            case 0xF1:
                RED = leds[++i];
                GREEN = leds[++i];
                BLUE = leds[++i];
                break;
            case 0xF2: ;
                uint8_t j = leds[++i];
                uint8_t lim = leds[++i];
                for( ; j <= lim; j++) {
                    set_led_color(j, RED, GREEN, BLUE);
                }
                break;
            default:
                if(inst < 0x50) {
                    set_led_color(inst, RED, GREEN, BLUE);
                } else {
                    RED = leds[++i];
                    GREEN = leds[++i];
                    BLUE = leds[++i];
                    set_led_color(inst - 0x50, RED, GREEN, BLUE);
                }
                break;
        }
    }
}

#define RESTORE_RGB restore_rgb

static uint8_t _prev_rgb_mode = 0;

void restore_rgb(void) {
    // rgb_matrix_sethsv_noeeprom(rgb_matrix_get_hue(), rgb_matrix_get_sat(), _prev_rgb_val);
    rgb_matrix_mode(_prev_rgb_mode);
}

// Doesn't actually turn rgb off, but sets it into SOLID_REACTIVE_SIMPLE
// mode which allows only used keys to
#define OFF_RGB off_rgb

void off_rgb(void) {
    _prev_rgb_mode = rgb_matrix_get_mode();
    rgb_matrix_mode_noeeprom(RGB_MATRIX_SOLID_REACTIVE_SIMPLE);
    // _prev_rgb_val = rgb_matrix_get_val();
    // rgb_matrix_sethsv_noeeprom(rgb_matrix_get_hue(), rgb_matrix_get_sat(), 0);
}

enum my_keycodes {
    LOWER1 = SAFE_RANGE,
    RAISE1,
    RAISE2,
    TOGLAY, // toggle between qwerty/colemak default layout
    BOOTLDR, // bootloader
    LAYCLR, // clear layers and leave only default.

    // keys for math macros
    // they will organized by mnemonic on the qwerty layout

    // katex environments
    M_EQN,      // E: equation*+split / equation+split (shift)
    M_ALIGN,    // A: align* / alignat*(alt) / align (shift) / alignat (shift+alt)
    M_MATRX,    // M: pmatrix / augmented pmatrix (alt) / bmatrix (shift) / augmented bmatrix (alt)
    M_CASES,    // C: cases / rcases (shift)

    // fonts and formatting
    M_RLAP,     // R: rlap+hspace / rlap (ctrl)
    M_CANC,     // K: cancel
    M_TEXT,     // T: text / textbf (shift) / textit (alt)
    M_MATH,     // B: mathbb / mathbf (shift) / mathcal (alt) / mathfrak (ctrl)
    M_VEC,      // V: vec

    // symbols/elements
    M_SPC,      // SPC: hspace{4cm}
    M_BSLS,     // QUOT: backslash / \left|\right| (shift) / \left\|\right\| (alt)
    M_PAR,      // O: \left(\right) / \left[\right] (shift)
    M_SET,      // S: \left\{\,\,\right\} / \left\{\,\;\mid\;\right\} (shift)
    M_LIM,      // L: \lim_{\to }
    M_FRAC,     // F: { \over }

    // custom shiftable keys:
    C_USCR,    // _ or - (shift)
};

enum layouts {
    _COLEMAK,
    _COLEMAK_MOD_DH, // colemak dh for matrix/ortho
    _QWERTY,
    _LOWER1_HOLD, // Meta settings
    _LOWER1_TAP, // mouse keys, bootloader
    _RAISE1, // Number row stuff
    _RAISE2, // FN Keys and navigation keys
    _MATH_MACROS, // Latex macros
};

const uint16_t PROGMEM keymaps[][MATRIX_ROWS][MATRIX_COLS] = {
    [_QWERTY] = LAYOUT_planck_mit(
        KC_ESC,  KC_Q,    KC_W,    KC_E,    KC_R,    KC_T,    KC_Y,    KC_U,    KC_I,    KC_O,    KC_P,    C_USCR,
        KC_TAB,  KC_A,    KC_S,    KC_D,    KC_F,    KC_G,    KC_H,    KC_J,    KC_K,    KC_L,    KC_SCLN, KC_QUOT,
        KC_BSPC, KC_Z,    KC_X,    KC_C,    KC_V,    KC_B,    KC_N,    KC_M,    KC_COMM, KC_DOT,  KC_UP,   KC_ENT,
        KC_LCTL, KC_LGUI, KC_LALT, LOWER1,  KC_LSFT,      KC_SPC,      RAISE1,  RAISE2,  KC_LEFT, KC_DOWN, KC_RGHT
    ),
    [_COLEMAK] = LAYOUT_planck_mit(
        KC_ESC,  KC_Q,    KC_W,    KC_F,    KC_P,    KC_G,    KC_J,    KC_L,    KC_U,    KC_Y,    KC_SCLN, C_USCR,
        KC_TAB,  KC_A,    KC_R,    KC_S,    KC_T,    KC_D,    KC_H,    KC_N,    KC_E,    KC_I,    KC_O,    KC_QUOT,
        KC_BSPC, KC_Z,    KC_X,    KC_C,    KC_V,    KC_B,    KC_K,    KC_M,    KC_COMM, KC_DOT,  KC_UP,   KC_ENT,
        KC_LCTL, KC_LGUI, KC_LALT, LOWER1,  KC_LSFT,      KC_SPC,      RAISE1,  RAISE2,  KC_LEFT, KC_DOWN, KC_RGHT
    ),
    [_COLEMAK_MOD_DH] = LAYOUT_planck_mit(
        KC_ESC,  KC_Q,    KC_W,    KC_F,    KC_P,    KC_B,    KC_J,    KC_L,    KC_U,    KC_Y,    KC_SCLN, C_USCR,
        KC_TAB,  KC_A,    KC_R,    KC_S,    KC_T,    KC_G,    KC_M,    KC_N,    KC_E,    KC_I,    KC_O,    KC_QUOT,
        KC_BSPC, KC_Z,    KC_X,    KC_C,    KC_D,    KC_V,    KC_K,    KC_H,    KC_COMM, KC_DOT,  KC_UP,   KC_ENT,
        KC_LCTL, KC_LGUI, KC_LALT, LOWER1,  KC_LSFT,      KC_SPC,      RAISE1,  RAISE2,  KC_LEFT, KC_DOWN, KC_RGHT
    ),
    [_MATH_MACROS] = LAYOUT_planck_mit(
        LAYCLR,  _______, _______, M_EQN,   M_RLAP,  M_TEXT,  _______, _______, _______, M_PAR,   _______, _______,
        _______, M_ALIGN, M_SET,   _______, M_FRAC,  _______, _______, _______, M_CANC,  M_LIM,   _______, M_BSLS,
        _______, _______, _______, M_CASES, M_VEC,   M_MATH,  _______, M_MATRX, _______, _______, _______, _______,
        _______, _______, _______, _______, _______,      M_SPC,       _______, _______, _______, _______, _______
    ),
    [_RAISE1] = LAYOUT_planck_mit(
        KC_GRV,  KC_1,    KC_2,    KC_3,    KC_4,    KC_5,    KC_6,    KC_7,    KC_8,    KC_9,    KC_0,    C_USCR,
        KC_TILD, KC_EXLM, KC_AT,   KC_HASH, KC_DLR,  KC_PERC, KC_CIRC, KC_LCBR, KC_RCBR, KC_LBRC, KC_RBRC, KC_BSLS,
        _______, _______, _______, KC_LPRN, KC_RPRN, KC_LT,   KC_GT,   KC_PLUS, KC_MINS, KC_ASTR, KC_SLSH, KC_PIPE,
        _______, _______, _______, _______, _______,     _______,      _______, KC_EQL,  KC_AMPR, KC_QUES, _______
    ),
    [_RAISE2] = LAYOUT_planck_mit(
        KC_F1,   KC_F2,   KC_F3,   KC_F4,   KC_F5,   KC_F6,   KC_F7,   KC_F8,   KC_F9,   KC_F10,  KC_F11,  KC_F12,
        KC_F13,  KC_F14,  KC_F15,  KC_F16,  KC_F17,  KC_F18,  KC_F19,  KC_F20,  KC_F21,  KC_F22,  KC_PSCR, KC_SCRL,
        _______, _______, _______, _______, _______, _______, _______, _______, _______, KC_INS,  KC_PGUP, KC_DEL,
        _______, _______, _______, _______, _______,     _______,      _______, _______, KC_HOME, KC_PGDN, KC_END
    ),
    [_LOWER1_HOLD] = LAYOUT_planck_mit(
        _______, KC_1,    KC_2,    KC_3,    KC_4,    _______, _______, RGB_HUI, RGB_SAI, RGB_VAI, RGB_SPI, RGB_MOD,
        TOGLAY,  KC_4,    KC_5,    KC_6,    _______, _______, _______, RGB_HUD, RGB_SAD, RGB_VAD, RGB_SPD, RGB_RMOD,
        _______, KC_7,    KC_8,    KC_9,    _______, _______, _______, _______, _______, _______, KC_VOLU, _______,
        _______, KC_0,    _______, _______, _______,     _______,      _______, _______, KC_BRID, KC_VOLD, KC_BRIU
    ),
    [_LOWER1_TAP] = LAYOUT_planck_mit(
        LAYCLR,  KC_1,    KC_2,    KC_3,    KC_4,    _______, _______, _______, _______, KC_PWR,  KC_WAKE, BOOTLDR,
        _______, KC_4,    KC_5,    KC_BSLS, _______, _______, _______, _______, _______, KC_BTN4, KC_BTN3, KC_BTN5,
        _______, A(KC_1), A(KC_2), A(KC_3), A(KC_4), _______, _______, KC_WH_L, KC_WH_R, KC_BTN1, KC_MS_U, KC_BTN2,
        _______, _______, _______, LAYCLR,  _______,     _______,      KC_WH_D, KC_WH_U, KC_MS_L, KC_MS_D, KC_MS_R
    ),
};


// Contains what key modifiers are already in effect
static uint8_t mod_state;

// How many cm to use in hspace in llap/rlap macro
static uint8_t lap_hspace_cm = 12;

static bool lower1_otherkeydownup = false;
static uint16_t lower1_timer = 0;

static uint16_t bootldr_timer = 0;
static uint8_t bootldr_loading_bar = 0;

uint32_t bootldr_keyhold_callback(uint32_t trigger_time, void *cb_arg) {
    if(bootldr_timer != 0) {
        if (timer_elapsed(bootldr_timer) > BOOTLDR_HOLD_TIME) {
            bootloader_jump();
            return 0;
        }

        bootldr_loading_bar = timer_elapsed(bootldr_timer) / (BOOTLDR_HOLD_TIME / 47);
        return BOOTLDR_HOLD_TIME / 47;
    }

    bootldr_loading_bar = 0;
    return 0;
}

// This value is
uint16_t quot_timer = 0;
uint8_t quot_loading_bar = 0;

uint32_t quot_keyhold_callback(uint32_t trigger_time, void *cb_arg) {
    if(quot_timer != 0) {
        if (timer_elapsed(quot_timer) > QUOT_HOLD_TIME) {
            layer_move(_MATH_MACROS);
            quot_loading_bar = 0;
            set_all_backlights(255);
            return 0;
        }

        quot_loading_bar = timer_elapsed(quot_timer) / (QUOT_HOLD_TIME / 12);
        return QUOT_HOLD_TIME / 12;
    }

    quot_loading_bar = 0;
    return 0;
}

// bitmasks for drawing pixels
// right-aligned, LSb b is rightmost pixel
uint16_t dots_r1 = 0; // [0, 2^13 - 1]
uint16_t dots_r2 = 0; // [0, 2^13 - 1]
uint16_t dots_r3 = 0; // [0, 2^13 - 1]
uint16_t dots_r4 = 0; // [0, 2^13 - 1] (space is represented by bit6 | bit7)

bool is_backspace_down = false;

void left(uint8_t left_amount) {
    for (uint8_t i = 0; i < left_amount; i++) {
        tap_code(KC_LEFT);
    }
}

bool process_record_user(uint16_t keycode, keyrecord_t *record) {
    if (keycode != LOWER1) {
        lower1_otherkeydownup = true;
    }

    if (record->event.pressed) {
        // Handle keydown events

        // Do reactive backlight stuff
        int8_t backlight_pos = record->event.key.col / 2;
        for (int8_t i = 0; i < 6; i++) {
            uint8_t brightness_inc = 0;
            if (i == backlight_pos) {
                brightness_inc = 150;
            } else if (i == backlight_pos - 1 || i == backlight_pos + 1) {
                brightness_inc = 75;
            } else if (i == backlight_pos - 2 || i == backlight_pos + 2) {
                brightness_inc = 35;
            }
            if (backlights[i] > 255 - brightness_inc) {
                backlights[i] = 255;
            } else {
                backlights[i] += brightness_inc;
            }
        }

        switch (keycode) {
            case KC_BSPC:
                if (mod_state & MOD_MASK_ALT) {
                    tap_code(KC_CAPS);
                    return false;
                }
                is_backspace_down = true;
                return true;
            case KC_LCTL:
                if (is_backspace_down) {
                    register_code(KC_LCTL);
                    tap_code(KC_BSPC);
                    unregister_code(KC_LCTL);
                    return false;
                }
                return true;
            case C_USCR:
                if (mod_state & MOD_MASK_SHIFT) {
                    unregister_code(KC_LSFT);
                    register_code(KC_MINS);
                } else {
                    register_code(KC_LSFT);
                    register_code(KC_MINS);
                }
                return true;
            case TOGLAY:
                set_all_backlights(127);
                switch (biton32(default_layer_state))
                {
                case _COLEMAK:
                    set_single_persistent_default_layer(_COLEMAK_MOD_DH);
                    break;
                case _COLEMAK_MOD_DH:
                    set_single_persistent_default_layer(_QWERTY);
                    break;
                case _QWERTY:
                    set_single_persistent_default_layer(_COLEMAK);
                    break;
                }
                return false;
            case LOWER1:
                set_all_backlights(127);
                layer_move(_LOWER1_HOLD);
                lower1_timer = timer_read();
                lower1_otherkeydownup = false;
                return false;
            case RAISE1:
                set_all_backlights(127);
                layer_move(_RAISE1);
                return false;
            case RAISE2:
                set_all_backlights(127);
                layer_move(_RAISE2);
                return false;
            case BOOTLDR:
                bootldr_timer = timer_read();
                defer_exec(BOOTLDR_HOLD_TIME/47, bootldr_keyhold_callback, NULL);
                return false;
            case KC_QUOT:
                quot_timer = timer_read();
                defer_exec(QUOT_HOLD_TIME/12, quot_keyhold_callback, NULL);
                return false;
            case LAYCLR:
                set_all_backlights(127);
                layer_clear();
                return false;
        }

        // If any key is pressed while quote was held, send the quote first
        // then stop the quote hold callback, and process this key normally.
        if (quot_timer != 0 && !IS_LAYER_ON(_MATH_MACROS)) {
            quot_timer = 0;
            tap_code(KC_QUOT);
            return true;
        }

        // Epic math macros time
        bool actually_send_macro = true;
        uint8_t left_amount = 0;
        #define SET_SCANCODES(...) { \
            uint8_t tmp[] = {__VA_ARGS__, 0};  \
            memcpy(scancodes, tmp, sizeof(tmp)); \
        }
        #define LEFT(amt) left_amount = amt;

        switch (keycode) {
            // katex environments
            // E: equation+split
            // A: align* / alignat*(alt) / align (shift) / alignat (shift+alt)
            // P: pmatrix / augmented pmatrix (alt) / bmatrix (shift) / augmented bmatrix (alt)
            // C: cases / rcases (shift)
            case M_EQN: ;
                // equation* + split / equation + split (shift)
                uint8_t eqn_template[] = {
                    KC_BSLS, KC_B, KC_E, KC_G, KC_I, KC_N, // 6
                    LBRACE, KC_E, KC_Q, KC_U, KC_A, KC_T, KC_I, KC_O, KC_N, KC_PAST, RBRACE, // 11 [17]
                    KC_BSLS, KC_B, KC_E, KC_G, KC_I, KC_N, LBRACE, KC_S, KC_P, KC_L, KC_I, KC_T, RBRACE, // 13 [30]
                    KC_BSLS, KC_E, KC_N, KC_D, LBRACE, KC_S, KC_P, KC_L, KC_I, KC_T, RBRACE, // 11 [41]
                    KC_BSLS, KC_E, KC_N, KC_D // 4 [45]
                };
                LEFT(26);
                if (mod_state & MOD_MASK_SHIFT) {
                    // no asterisk if shift
                    eqn_template[15] = 3;
                    left_amount--;
                }
                memcpy(&scancodes[0], eqn_template, sizeof(eqn_template));
                memcpy(&scancodes[45], &scancodes[6], 11); // copy over the environment
                scancodes[45 + 11] = 0; // terminate
                break;
            case M_ALIGN: ;
                // align* / alignat*(alt) / align (shift) / alignat (shift+alt)
                uint8_t align_template[] = {
                    KC_BSLS, KC_B, KC_E, KC_G, KC_I, KC_N, // 6
                    LBRACE, KC_A, KC_L, KC_I, KC_G, KC_N, 3, 3, KC_PAST, RBRACE, // 10 [16]
                    KC_BSLS, KC_E, KC_N, KC_D // 4 [20]
                };
                LEFT(12);
                if (mod_state & MOD_MASK_SHIFT) {
                    // no asterisk if shift
                    align_template[14] = 3;
                    left_amount--;
                }
                if (mod_state & MOD_MASK_ALT) {
                    // alignat
                    align_template[12] = KC_A;
                    align_template[13] = KC_T;
                    left_amount += 2;
                }
                memcpy(scancodes, align_template, sizeof(align_template));
                memcpy(&scancodes[20], &scancodes[6], 10); // copy over the environment
                scancodes[20 + 10] = 0; // terminate
                break;
            case M_MATRX: ;
                // pmatrix / augmented pmatrix (alt) / bmatrix (shift) / augmented bmatrix (shift+alt)
                if (mod_state & MOD_MASK_ALT) {
                    SET_SCANCODES(
                        KC_BSLS, KC_L, KC_E, KC_F, KC_T, SHIFT(KC_9), // 6
                        KC_BSLS, KC_B, KC_E, KC_G, KC_I, KC_N, // 6 [12]
                        LBRACE, KC_A, KC_R, KC_R, KC_A, KC_Y, RBRACE, // 7 [19]
                        LBRACE, KC_C, KC_C, KC_C, SHIFT(KC_BSLS), KC_C, RBRACE, // 7 [26]
                        KC_BSLS, KC_E, KC_N, KC_D, // 4 [30]
                        LBRACE, KC_A, KC_R, KC_R, KC_A, KC_Y, RBRACE, // 7 [37]
                        KC_BSLS, KC_R, KC_I, KC_G, KC_H, KC_T, SHIFT(KC_0) // 7 [44]
                    );
                    LEFT(18);
                    if (mod_state & MOD_MASK_SHIFT) {
                        scancodes[5] = KC_LBRC;
                        scancodes[43] = KC_RBRC;
                    }
                } else {
                    uint8_t matrix_template[] = {
                        KC_BSLS, KC_B, KC_E, KC_G, KC_I, KC_N, // 6
                        LBRACE, KC_P, KC_M, KC_A, KC_T, KC_R, KC_I, KC_X, RBRACE, // 9 [15]
                        KC_BSLS, KC_E, KC_N, KC_D // 4 [19]
                    };
                    if (mod_state & MOD_MASK_SHIFT) {
                        matrix_template[7] = KC_B;
                    }
                    memcpy(scancodes, matrix_template, sizeof(matrix_template));
                    memcpy(&scancodes[19], &scancodes[6], 9); // copy over the environment
                    scancodes[19 + 9] = 0; // terminate
                    LEFT(13);
                }
                break;

            // fonts and formatting
            // R: rlap+hspace / rlap (ctrl)
            // K: cancel
            // T: text / textbf (shift) / textit (alt)
            // B: mathbb / mathbf (shift) / mathcal (alt) / mathfrak (ctrl)
            // V: vec
            case M_RLAP:
                // rlap+hspace{xxcm} / rlap (ctrl)
                // spacing given by lap_hspace_cm
                if (mod_state & MOD_MASK_CTRL) {
                    SET_SCANCODES(
                        KC_BSLS, KC_R, KC_L, KC_A, KC_P, LBRACE, RBRACE // 7
                    );
                    LEFT(1);
                } else {
                    uint8_t TENS = lap_hspace_cm / 10;
                    if (!TENS) TENS = 10; // keycode 0 is 9 indices above 1

                    uint8_t ONES = lap_hspace_cm % 10;
                    if (!ONES) ONES = 10; // keycode 0 is 9 indices above 1

                    SET_SCANCODES(
                        KC_BSLS, KC_R, KC_L, KC_A, KC_P, LBRACE, RBRACE, // 7
                        KC_BSLS, KC_H, KC_S, KC_P, KC_A, KC_C, KC_E, LBRACE, KC_1 - 1 + TENS, KC_1 - 1 + ONES, KC_C, KC_M, RBRACE // 13 [20]
                    );
                    LEFT(14);
                }
                break;
            case M_CANC:
                // cancel
                SET_SCANCODES(
                    KC_BSLS, KC_C, KC_A, KC_N, KC_C, KC_E, KC_L, LBRACE, RBRACE // 9
                );
                LEFT(1);
                break;
            case M_TEXT:
                // text / textbf (shift) / textit (alt)
                SET_SCANCODES(
                    KC_BSLS, KC_T, KC_E, KC_X, KC_T, 3, 3, LBRACE, RBRACE // 7
                );
                LEFT(1);
                if (mod_state & MOD_MASK_SHIFT) {
                    scancodes[5] = KC_B;
                    scancodes[6] = KC_F;
                } else if (mod_state & MOD_MASK_ALT) {
                    scancodes[5] = KC_I;
                    scancodes[6] = KC_T;
                }
                break;
            case M_MATH:
                // mathbb / mathbf (shift) / mathcal (alt) / mathfrak (ctrl)
                SET_SCANCODES(
                    KC_BSLS, KC_M, KC_A, KC_T, KC_H, KC_B, KC_B, 3, 3, LBRACE, RBRACE // 7
                );
                if (mod_state & MOD_MASK_SHIFT) {
                    scancodes[6] = KC_F;
                } else if (mod_state & MOD_MASK_ALT) {
                    scancodes[5] = KC_C;
                    scancodes[6] = KC_A;
                    scancodes[7] = KC_L;
                } else if (mod_state & MOD_MASK_CTRL) {
                    scancodes[5] = KC_F;
                    scancodes[6] = KC_R;
                    scancodes[7] = KC_A;
                    scancodes[8] = KC_K;
                }
                LEFT(1);
                break;
            case M_CASES:
                // cases / rcases (shift)
                SET_SCANCODES(
                    KC_BSLS, KC_B, KC_E, KC_G, KC_I, KC_N, // 6
                    LBRACE, 3, KC_C, KC_A, KC_S, KC_E, KC_S, RBRACE, // 8 [14]
                    KC_BSLS, KC_E, KC_N, KC_D // 4 [18]
                );
                LEFT(11);
                if(mod_state & MOD_MASK_SHIFT) {
                    scancodes[7] = KC_R;
                    left_amount++;
                }
                memcpy(&scancodes[18], &scancodes[6], 8); // copy {cases}/{rcases} after \end
                scancodes[18 + 8] = 0; // terminate
                break;
            case M_VEC:
                // vec
                SET_SCANCODES(
                    KC_BSLS, KC_V, KC_E, KC_C, LBRACE, RBRACE // 5
                );
                LEFT(1);
                break;
            // symbols/elements
            // SPC: hspace{4cm}
            // QUOT: backslash / \left|\right| (shift) / \left\|\right\| (alt)
            // O: \left(\right) / \left[\right] (shift)
            // S: \left\{\,\,\right\} / \left\{\,\;\mid\;\right\} (shift)
            // L: \lim_{\to }
            // F: { \over }

            case M_SPC:
                // hspace{4cm}
                SET_SCANCODES(
                    KC_BSLS, KC_H, KC_S, KC_P, KC_A, KC_C, KC_E, LBRACE, KC_4, KC_C, KC_M, RBRACE // 12
                );
                break;
            case M_BSLS:
                // backslash / \left|\right| (shift) / \left\|\right\| (alt)
                SET_SCANCODES(
                    KC_BSLS, KC_B, KC_A, KC_C, KC_K, KC_S, KC_L, KC_A, KC_S, KC_H // 10
                );
                if (mod_state & MOD_MASK_SHIFT) {
                    // \left|\right|
                    SET_SCANCODES(
                        KC_BSLS, KC_L, KC_E, KC_F, KC_T, SHIFT(KC_BSLS), // 6
                        KC_BSLS, KC_R, KC_I, KC_G, KC_H, KC_T, SHIFT(KC_BSLS) // 7 [13]
                    );
                    LEFT(7);
                } else if (mod_state & MOD_MASK_ALT) {
                    // \left\|\right\|
                    SET_SCANCODES(
                        KC_BSLS, KC_L, KC_E, KC_F, KC_T, KC_BSLS, SHIFT(KC_BSLS), // 7
                        KC_BSLS, KC_R, KC_I, KC_G, KC_H, KC_T, KC_BSLS, SHIFT(KC_BSLS) // 8 [15]
                    );
                    LEFT(8);
                }
                break;
            case M_PAR:
                // \left(\right) / \left[\right] (shift)
                SET_SCANCODES(
                    KC_BSLS, KC_L, KC_E, KC_F, KC_T, SHIFT(KC_9), // 6
                    KC_BSLS, KC_R, KC_I, KC_G, KC_H, KC_T, SHIFT(KC_0) // 7 [13]
                );
                if (mod_state & MOD_MASK_SHIFT) {
                    scancodes[5] = KC_LBRC;
                    scancodes[12] = KC_RBRC;
                }
                LEFT(7);
                break;
            case M_SET:
                // \left\{\,\,\right\} / \left\{\,\mid\,\right\} (shift)
                SET_SCANCODES(
                    KC_BSLS, KC_L, KC_E, KC_F, KC_T, KC_BSLS, LBRACE, // 7
                    KC_BSLS, KC_COMM, // 2 [9]
                //  \  m  i  d
                    3, 3, 3, 3, // 4 [13]
                    KC_BSLS, KC_COMM, // 2 [15]
                    KC_BSLS, KC_R, KC_I, KC_G, KC_H, KC_T, KC_BSLS, RBRACE // 8 [23]
                );
                LEFT(10);
                if (mod_state & MOD_MASK_SHIFT) {
                    scancodes[9] = KC_BSLS;
                    scancodes[10] = KC_M;
                    scancodes[11] = KC_I;
                    scancodes[12] = KC_D;
                    left_amount += 4;
                }
                break;
            case M_LIM:
                // \lim_{ \to }
                SET_SCANCODES(
                    KC_BSLS, KC_L, KC_I, KC_M, SHIFT(KC_MINS), LBRACE, KC_SPC, KC_BSLS, KC_T, KC_O, KC_SPC, RBRACE // 9
                );
                LEFT(5);
                break;
            case M_FRAC:
                // { \over }
                SET_SCANCODES(
                    KC_BSLS, KC_F, KC_R, KC_A, KC_C, LBRACE, RBRACE, LBRACE, RBRACE // 9
                );
                LEFT(3);
                break;

            // not a math macro:
            default:
                actually_send_macro = false;
        }

        if (actually_send_macro) {
            clear_mods();
            // flush clearing of the mods otherwise it seems like modifiers are active.
            send_keyboard_report();
            // wait_ms(5); no need for this anymore i think.
            send_scancodes();
            left(left_amount);
            layer_off(_MATH_MACROS);
            return false;
        }
    } else {
        // Handle keyup events
        switch (keycode) {
            case KC_BSPC:
                is_backspace_down = false;
                return true;
            case C_USCR:
                // be careful here, since when underscore_equal is pressed, it will
                // invert the state of shift until it is released.
                // This means that the respective keyup events will need to check for
                // the opposite shift state.
                if (!(mod_state & MOD_MASK_SHIFT)) {
                    unregister_code(KC_MINS);
                    register_code(KC_LSFT);
                } else {
                    unregister_code(KC_MINS);
                    unregister_code(KC_LSFT);
                }
                return true;
            case LOWER1:
                layer_off(_LOWER1_HOLD);
                if (!lower1_otherkeydownup && timer_elapsed(lower1_timer) < TAPPING_TERM) {
                    set_all_backlights(255);
                    layer_on(_LOWER1_TAP);
                }
                lower1_timer = 0;
                return false;
            case RAISE1:
                layer_off(_RAISE1);
                return false;
            case RAISE2:
                layer_off(_RAISE2);
                return false;
            case BOOTLDR:
                bootldr_timer = 0;
                return false;
            case KC_QUOT:
                if (timer_elapsed(quot_timer) < QUOT_HOLD_TIME) {
                    tap_code(KC_QUOT);
                }
                quot_timer = 0;
                return false;
        }

        if(IS_LAYER_ON(_LOWER1_TAP)) {
            // Exit tap after genshin macros
            if ((keycode >= KC_1 && keycode <= KC_5)
                || (keycode >= A(KC_1) && keycode <= A(KC_5))
                || keycode == KC_BSLS) {
                layer_off(_LOWER1_TAP);
                return true;
            }
        }
    }

    if(IS_LAYER_ON(_MATH_MACROS)) {
        // if we're in math macros, we don't want any modifiers/keypresses to be sent
        // (modifiers may trigger keyboard shortcuts)

        // but, we need to keep mod state updated.
        switch(keycode) {
        case KC_LSFT:
        case KC_RSFT:
            // Manually update mod_state
            if (record->event.pressed) {
                add_mods(MOD_MASK_SHIFT);
            } else {
                del_mods(MOD_MASK_SHIFT);
            }
            break;
        case KC_LCTL:
        case KC_RCTL:
            if (record->event.pressed) {
                add_mods(MOD_MASK_CTRL);
            } else {
                del_mods(MOD_MASK_CTRL);
            }
            break;
        case KC_LALT:
        case KC_RALT:
            if (record->event.pressed) {
                add_mods(MOD_MASK_ALT);
            } else {
                del_mods(MOD_MASK_ALT);
            }
            break;
        }
        return false;
    }
    return true;
}

// Drawing pixels based on dots_r1, dots_r2, dots_r3, dots_r4
void draw_dots(uint8_t R, uint8_t G, uint8_t B) {
    for(uint8_t col = 0; col < 12; col++) {
        if (dots_r1 & (1 << col)) {
            set_led_color(11 - col, R, G, B);
        }
        if (dots_r2 & (1 << col)) {
            set_led_color(23 - col, R, G, B);
        }
        if (dots_r3 & (1 << col)) {
            set_led_color(35 - col, R, G, B);
        }
        if (dots_r4 & (1 << col)) {
            set_led_color(47 - col, R, G, B);
        }
    }
}

void rgb_matrix_indicators_advanced_user(uint8_t led_min, uint8_t led_max) {
    mod_state = get_mods();

    memset(leds, LED_NOOP, LED_INSTRUCTION_LIMIT);

    if (IS_LAYER_ON(_RAISE1)) {
        uint8_t raise1_leds[] = {
            LED_SETCOLOR(0, 0xff, 0),
            LED_RANGE(1, 10), // number row in #00ff00
            29, 30, // < and > in #00ff00
            LED_SETCOLOR(0, 0xaa, 0xcc),
            // shifted number row (1 to 6) in #00aacc
            LED_RANGE(13, 18),
            // eql, ampr, ques in #00aacc
            43, 44, 45,
            // Set backtick, tilde, del, ampr, ques, to #dd9900
            LED_SETCOLOR_ON(0, 0xdd, 0x99, 0), 11, 12,
            // set LBRC, RBRC, LCBR, RCBR, LPRN, RPRN to #ee00ee
            LED_SETCOLOR_ON(21, 0xee, 0, 0xee), 22, 19, 20, 27, 28,
            // set backslash, plus, minus, asterisk, slash to #ddff00
            LED_SETCOLOR_ON(23, 0xdd, 0xff, 0), 31, 32, 33, 34,
            LED_TERMINATE
        };
        memcpy(leds, raise1_leds, sizeof(raise1_leds));
        update_led();
    } else if (IS_LAYER_ON(_RAISE2)) {
        uint8_t raise2_leds[] = {
            LED_SETCOLOR(0xdd, 0x99, 0),
            LED_RANGE(0, 21), // FN1-FN22 in #dd9900
            LED_SETCOLOR_ON(44, 0x00, 0x30, 0xff), 45, 46, 34, // set home, end, page up/down to #0030ff
            LED_SETCOLOR_ON(33, 0xa0, 0x30, 0), 35, // set insert, delete to #a03000
            LED_SETCOLOR_ON(22, 0x50, 0xb0, 0x30), 23,  // set prt scr, scrlock to #50b030
            LED_TERMINATE
        };
        memcpy(leds, raise2_leds, sizeof(raise2_leds));
        update_led();
    } else if (IS_LAYER_ON(_LOWER1_HOLD)) {
        // Hue, Sat, Val
        HSV hsv = rgb_matrix_get_hsv();
        hsv.v = 255;
        RGB sat_disp = hsv_to_rgb(hsv);
        hsv.s = 255;
        RGB hue_only = hsv_to_rgb(hsv);
        hsv.h = timer_elapsed(lower1_timer) / 8 % 255;
        RGB rotating_hue = hsv_to_rgb(hsv);

        // Speed
        uint8_t spd_up = LED_NOOP;
        uint8_t spd_down = LED_NOOP;
        if ((timer_elapsed(lower1_timer) * (1 + 5 * rgb_matrix_get_speed() / 255)) % 1000 < 400) {
            spd_up = 10, spd_down = 22;
        }

        uint8_t early_terminate = LED_NOOP;
        if (timer_elapsed(lower1_timer) % 800 < 400) {
            early_terminate = LED_TERMINATE;
        }

        uint8_t terminate_if_c = LED_NOOP;
        if (biton32(default_layer_state) == _COLEMAK) {
            terminate_if_c = LED_TERMINATE;
        }

        uint8_t terminate_if_qwerty = LED_NOOP;
        if (biton32(default_layer_state) == _QWERTY) {
            terminate_if_qwerty = LED_TERMINATE;
        }

        uint8_t lower1_hold_leds[] = {
            LED_SETCOLOR_ON(12, 0xdd, 0xdd, 0xdd), // set tab to #dddddd
            LED_SETCOLOR_ON(44, 0xf0, 0xdd, 0x22), 46, // set brightness up/down to #f0dd22
            LED_SETCOLOR_ON(45, 0xdd, 0x22, 0xaa), 34, // set volume up/down to #dd22aa
            LED_SETCOLOR_ON(7, hue_only.r, hue_only.g, hue_only.b), 19, // set hue selectors
            LED_SETCOLOR_ON(8, sat_disp.r, sat_disp.g, sat_disp.b), 20, 9, 21, // set sat and val selectors
            spd_up, spd_down, // blink speed selectors
            LED_SETCOLOR_ON(11, rotating_hue.r, rotating_hue.g, rotating_hue.b), 23, // change rgb mode
            early_terminate,
            1, 2, 3, 13, 25, 26, 27, terminate_if_c, // draw c
            15, 39, terminate_if_qwerty, // draw q
            14, 37, 38, // blot out everything if colemak DH
            LED_TERMINATE,
        };

        memcpy(leds, lower1_hold_leds, sizeof(lower1_hold_leds));
        update_led();
    } else if (IS_LAYER_ON(_LOWER1_TAP)) {
        uint8_t blink = (timer_elapsed(lower1_timer) % 500 < 200) ? LED_NOOP : LED_TERMINATE;

        uint8_t lower1_tap_leds[] = {
            LED_SETCOLOR_ON(34, 0x30, 0x60, 0xc0), 44, 45, 46,      // set mouse movement keys to #3060c0
            31, 32, 42, 43,                                         // set mouse scroll keys to #3060c0
            LED_SETCOLOR_ON(33, 0x22, 0xbb, 0x99), 35, 15,          // left/right click, Backslash (genshin party) #22bb99
            LED_SETCOLOR_ON(21, 0x40, 0xa0, 0x00), 22, 23,          // set mouse fwd/back/middle click to #40a000
            LED_SETCOLOR_ON(1, 0xee, 0xa0, 0x50), 2, 3, 4, 13, 14,  // 1-5 (Genshin change character) #eea050
            blink,
            LED_SETCOLOR_ON(11, 0x30, 0xa0, 0xe0), 25,      // bootloader button, ALT+1 in #30a0e0
            LED_SETCOLOR_ON(0, 0xff, 0x40, 0x30), 9, 26,    // exit layer, power button, ALT+2, #ff4030
            LED_SETCOLOR_ON(10, 0x22, 0xee, 0x22), 27,      // wake button, ALT+3 #22ee22
            LED_SETCOLOR_ON(28, 0xee, 0xa0, 0x50),          // ALT+4 #eea050
            LED_TERMINATE
        };

        memcpy(leds, lower1_tap_leds, sizeof(lower1_tap_leds));
        update_led();
    } else if (IS_LAYER_ON(_MATH_MACROS)) {
        // katex environments
        // E: equation*+split / equation+split (shift)
        // A: align* / alignat*(alt) / align (shift) / alignat (shift+alt)
        // M: pmatrix / augmented pmatrix (alt) / bmatrix (shift) / augmented bmatrix (alt)
        // C: cases / rcases (shift)

        // fonts and formatting
        // R: rlap+hspace / rlap (ctrl)
        // K: cancel
        // T: text / textbf (shift) / textit (alt)
        // B: mathbb / mathbf (shift) / mathcal (alt) / mathfrak (ctrl)
        // V: vec

        // symbols/elements
        // SPC: hspace{4cm}
        // QUOT: backslash / \left|\right| (shift) / \left\|\right\| (alt)
        // O: \left(\right) / \left[\right] (shift)
        // S: \left\{\,\,\right\} / \left\{\,\;\mid\;\right\} (shift)
        // L: \lim_{\to }
        // F: { \over }

        uint8_t blink = (timer_elapsed(lower1_timer) % 500 < 200) ? LED_NOOP : LED_TERMINATE;

        uint8_t math_leds[] = {
            LED_SETCOLOR(0xe0, 0x90, 0xc0), // katex environments #e090c0
            // idx 4-7
            LED_QW_E, LED_QW_A, LED_QW_M, LED_QW_C,
            LED_SETCOLOR(0x50, 0xc0, 0xe0), // fonts and formatting #50c0e0
            // idx 12-16
            LED_QW_R, LED_QW_K, LED_QW_T, LED_QW_B, LED_QW_V,
            LED_SETCOLOR(0x50, 0xe0, 0xb0), // symbols/elements #50e0b0
            // idx 21-26
            41, 23, LED_QW_O, LED_QW_S, LED_QW_L, LED_QW_F,
            blink,
            LED_SETCOLOR_ON(0, 0xff, 0x99, 0x00), // exit math macros #ff9900
            LED_TERMINATE,
        };

        if (mod_state & MOD_MASK_SHIFT) {
            // remove buttons that doesn't have shift function:
            // r, k, v, spc, l, f
            math_leds[12] = LED_NOOP;
            math_leds[13] = LED_NOOP;
            math_leds[16] = LED_NOOP;
            math_leds[21] = LED_NOOP;
            math_leds[25] = LED_NOOP;
            math_leds[26] = LED_NOOP;
        }

        if (mod_state & MOD_MASK_ALT) {
            // remove buttons that doesn't have alt function:
            // e, c, r, k, v, spc, o, s, l, f
            math_leds[4] = LED_NOOP;
            math_leds[7] = LED_NOOP;
            math_leds[12] = LED_NOOP;
            math_leds[13] = LED_NOOP;
            math_leds[16] = LED_NOOP;
            math_leds[21] = LED_NOOP;
            math_leds[23] = LED_NOOP;
            math_leds[24] = LED_NOOP;
            math_leds[25] = LED_NOOP;
            math_leds[26] = LED_NOOP;
        }

        if (mod_state & MOD_MASK_CTRL) {
            // remove buttons that doesn't have ctrl function:
            // e, a, m, c, k, t, v, spc, quot, o, s, l, f
            math_leds[4] = LED_NOOP;
            math_leds[5] = LED_NOOP;
            math_leds[6] = LED_NOOP;
            math_leds[7] = LED_NOOP;
            math_leds[13] = LED_NOOP;
            math_leds[14] = LED_NOOP;
            math_leds[16] = LED_NOOP;
            memset(&math_leds[21], LED_NOOP, 6);
        }

        memcpy(leds, math_leds, sizeof(math_leds));
        update_led();
    }

    if (bootldr_loading_bar != 0) {
        for(uint8_t i = 0; i < bootldr_loading_bar; i++) {
            uint8_t x = 255 * i / 47;
            if ((i/12) % 2 == 0) { // 1st and 3rd row, start from right
                set_led_color((i/12) * 12 + (11 - i%12), x, x, 255);
            } else { // 2nd and 4th row, start from left
                set_led_color(i, x, x, 255);
            }
        }
    }

    if (quot_loading_bar != 0) {
        for(uint8_t i = 0; i < quot_loading_bar; i++) {
            uint8_t x = 255 * i / 12;
            set_led_color(11 - i, x, 255, x);
            set_led_color(23 - i, x, 255, x);
            set_led_color(35 - i, x, 255, x);
        }
    }

    HSV hsv = rgb_matrix_get_hsv();
    // render backlights
    for (uint8_t i = 0; i < 6; i++) {
        uint8_t val= hsv.v > 127 ? 255 : hsv.v + 128;
        rgblight_sethsv_at(hsv.h, hsv.s, val * backlights[i] / 255, i);
        if (backlights[i] > 0) {
            backlights[i] -= backlights[i] > 190 ? 3 : 1;
        }
    }
}
