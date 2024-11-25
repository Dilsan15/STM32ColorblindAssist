#include <stdio.h>
#include <math.h>

const char* hsv_string(float hue, float saturation, float value){

    float all_colors_hsv[][3] = {
        {0, 1.00, 1.00},
        {120, 1.00, 1.00},
        {240, 1.00, 1.00},
        {60, 1.00, 1.00},
        {0, 0.00, 0.00},
        {0, 0.00, 1.00},
        {0, 0.00, 0.50},
        {39, 1.00, 1.00},
        {350, 0.25, 1.00},
        {0, 0.75, 0.65},
        {300, 1.00, 0.50}
    };

    const char* color_names[] = {
        "red", "green", "blue", "yellow", "black", "white", "grey", "orange", "pink", "brown", "purple"
    };

    float min_dis = 10000;
    int all_c_index = 0;

    float distance;
    float hue_diff;

    for (int i = 0; i < 11; i++){
        hue_diff = fmin(fabs(hue - all_colors_hsv[i][0]), 360 - fabs(hue - all_colors_hsv[i][0]));
        distance = sqrt(pow(hue_diff, 2) + 
                        pow(saturation - all_colors_hsv[i][1], 2) + 
                        pow(value - all_colors_hsv[i][2], 2));

        if (distance < min_dis){
            all_c_index = i;
            min_dis = distance;
        }
    }

    if (all_c_index == 0) return "Red";
    if (all_c_index == 1) return "Green";
    if (all_c_index == 2) return "Blue";
    if (all_c_index == 3) return "Yellow";
    if (all_c_index == 4) return "Black";
    if (all_c_index == 5) return "White";
    if (all_c_index == 6) return "Gray";
    if (all_c_index == 7) return "Orange";
    if (all_c_index == 8) return "Pink";
    if (all_c_index == 9) return "Brown";
    if (all_c_index == 10) return "Purple";

}

int main() {

    float hue, saturation, value;

    printf("Enter hue (0.0 to 360.0): ");
    scanf("%f", &hue);

    printf("Enter saturation (0.0 to 1.0): ");
    scanf("%f", &saturation);

    printf("Enter value (0.0 to 1.0): ");
    scanf("%f", &value);

    const char* closest_color = hsv_string(hue, saturation, value);

    printf("Closest color: %s\n", closest_color);

    return 0;
}
