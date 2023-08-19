#ifdef RGB_MATRIX_KEYREACTIVE_ENABLED

RGB_MATRIX_EFFECT(splash_jelly)

#   ifdef RGB_MATRIX_CUSTOM_EFFECT_IMPLS

#define JELLY_BRIGHTNESS_PER_KEYPRESS 255

HSV jelly_math(HSV hsv, int16_t dx, int16_t dy, uint8_t dist, uint16_t tick) {
    uint16_t effect = tick * 2 - dist;
    if (effect > JELLY_BRIGHTNESS_PER_KEYPRESS) effect = JELLY_BRIGHTNESS_PER_KEYPRESS;
    uint8_t val = scale8(JELLY_BRIGHTNESS_PER_KEYPRESS - effect, 255 - dist);
    hsv.h += 13 * dx + dist * g_last_hit_tracker.count + 19 * dy; // generate some pseudorandom hue
    hsv.s = 228 - (dist + dx * g_last_hit_tracker.count * 15) % 40; // desaturate a bit for jelly like effect
    hsv.v = qadd8(hsv.v, val);
    return hsv;
}

static bool splash_jelly(effect_params_t* params) {
    return effect_runner_reactive_splash(0, params, &jelly_math);
}

#   endif
#endif         // RGB_MATRIX_KEYREACTIVE_ENABLED
