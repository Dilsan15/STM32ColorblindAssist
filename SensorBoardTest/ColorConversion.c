// C Functions to convert between RGB --> HSV and RGB --> HEX


#include <math.h>

float* rgbToHsv(int red, int green, int blue) {
    static float hsv[3]; // Array to store hue, saturation, value

    float gNorm = green / 255.0f;
    float rNorm = red / 255.0f;
    float bNorm = blue / 255.0f;

    float max = fmaxf(rNorm, fmaxf(gNorm, bNorm));
    float min = fminf(rNorm, fminf(gNorm, bNorm));
    float d = max - min;

    if (d == 0) {
        hsv[0] = 0; 
    } else if (max == rNorm) {
        hsv[0] = fmodf(((gNorm - bNorm) / d), 6.0f) * 60.0f;
        if (hsv[0] < 0) hsv[0] += 360.0f;
    } else if (max == gNorm) {
        hsv[0] = ((bNorm - rNorm) / d + 2.0f) * 60.0f;
    } else {
        hsv[0] = ((rNorm - gNorm) / d + 4.0f) * 60.0f;
    }

    hsv[1] = (max == 0) ? 0 : (d / max);

    hsv[2] = max;

    return hsv;
}

const char* rgbToHex(int red, int green, int blue) {
    static char hex[8];
    sprintf(hex, "#%02X%02X%02X", red, green, blue);
    return hex; 
}