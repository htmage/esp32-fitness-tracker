#include "ui.h"

#define bg_color_1 0xde38
#define bg_color_2 0x7f51
#define bg_color_3 0x755d
#define margin 10

static const int32_t base_text_width = 6;
static const int32_t base_text_height = 8;

void update_screen_1(LGFX *lcd, int heart_rate, int step_count, int selecting_item, float stride_length)
{

    static int last_selected_item = 0;
    static bool init = false;
    float distance = step_count * stride_length;

    if (selecting_item == 0 && !init)
    {
        lcd->fillScreen(0xffff);
        lcd->fillRoundRect(220, 20, 80, 200, 20, bg_color_1);
        lcd->fillRoundRect(120, 20, 80, 200, 20, bg_color_2);
        lcd->fillRoundRect(20, 20, 80, 200, 20, bg_color_3);
        lcd->drawBitmap(230, 30, distance_icon, 60, 60, bg_color_1, 0x0000);
        lcd->drawBitmap(130, 30, step_icon, 60, 60, bg_color_2, 0x015f);
        lcd->drawBitmap(30, 30, heart_pulse_icon, 60, 60, bg_color_3, 0xf800);

        lcd->setTextSize(2);
        lcd->setTextColor(0x0000);
        lcd->setCursor(243, 100);
        lcd->print("Run");
        lcd->setCursor(234, 120);
        lcd->print("Dist.");

        lcd->setCursor(137, 100);
        lcd->print("Step");
        lcd->setCursor(132, 120);
        lcd->print("Count");

        lcd->setCursor(32, 100);
        lcd->print("Heart");
        lcd->setCursor(38, 120);
        lcd->print("Rate");
        init = true;
    }
    else
    {
        // printf("selecting_item: %d\n", selecting_item);
        // printf("last_selected_item: %d\n", last_selected_item);
        if (last_selected_item != selecting_item)
        {
            // printf("change item\n");
            switch (selecting_item)
            {
            case 1:
                // printf("case 1\n");
                lcd->fillScreen(0xffff);
                lcd->fillRoundRect(220, 20, 80, 200, 20, bg_color_1);
                lcd->fillRoundRect(120, 20, 80, 200, 20, bg_color_2);
                lcd->fillRoundRect(20 - margin, 20 - margin, 80 + 2 * margin, 200 + 2 * margin, 20, bg_color_3);
                break;
            case 2:
                // printf("case 2\n");
                lcd->fillScreen(0xffff);
                lcd->fillRoundRect(220, 20, 80, 200, 20, bg_color_1);
                lcd->fillRoundRect(120 - margin, 20 - margin, 80 + 2 * margin, 200 + 2 * margin, 20, bg_color_2);
                lcd->fillRoundRect(20, 20, 80, 200, 20, bg_color_3);
                break;
            case 3:
                // printf("case 3\n");
                lcd->fillScreen(0xffff);
                lcd->fillRoundRect(220 - margin, 20 - margin, 80 + 2 * margin, 200 + 2 * margin, 20, bg_color_1);
                lcd->fillRoundRect(120, 20, 80, 200, 20, bg_color_2);
                lcd->fillRoundRect(20, 20, 80, 200, 20, bg_color_3);
                break;
            }
            // lcd->setRotation(3);
            lcd->drawBitmap(230, 30, distance_icon, 60, 60, bg_color_1, 0x0000);
            lcd->drawBitmap(130, 30, step_icon, 60, 60, bg_color_2, 0x015f);
            lcd->drawBitmap(30, 30, heart_pulse_icon, 60, 60, bg_color_3, 0xf800);

            lcd->setTextSize(2);
            lcd->setTextColor(0x0000);
            lcd->setCursor(243, 100);
            lcd->print("Run");
            lcd->setCursor(234, 120);
            lcd->print("Dist.");

            lcd->setCursor(137, 100);
            lcd->print("Step");
            lcd->setCursor(132, 120);
            lcd->print("Count");

            lcd->setCursor(32, 100);
            lcd->print("Heart");
            lcd->setCursor(38, 120);
            lcd->print("Rate");
        }
    }
    // heart rate
    if (heart_rate < 10 || heart_rate > 200)
    {
        lcd->setTextSize(4);
        lcd->fillRect(34, 165, 51, 28, bg_color_3);
        lcd->drawNumber(68, 38, 165); // default value 68
    }
    else if (heart_rate < 100)
    {
        lcd->setTextSize(4);
        lcd->fillRect(34, 165, 51, 28, bg_color_3);
        lcd->drawNumber(heart_rate, 38, 165);
    }
    else
    {
        lcd->setTextSize(3);
        lcd->fillRect(34, 165, 51, 28, bg_color_3);
        lcd->drawNumber(heart_rate, 34, 168);
    }
    // step count
    if (step_count < 10)
    {
        // 1 digit
        lcd->setTextSize(4);
        lcd->fillRect(150, 165, 20, 28, bg_color_2);
        lcd->drawNumber(step_count, 150, 165);
    }
    else if (step_count < 100)
    {
        // 2 digit
        lcd->setTextSize(4);
        lcd->fillRect(138, 165, 44, 28, bg_color_2);
        lcd->drawNumber(step_count, 138, 165);
    }
    else
    {
        // 3 digit
        lcd->setTextSize(3);
        lcd->fillRect(134, 165, 51, 28, bg_color_2);
        lcd->drawNumber(step_count, 134, 168);
    }
    // distance
    if (distance < 10.0)
    {
        // 1 digit
        lcd->setTextSize(4);
        lcd->fillRect(250, 165, 20, 28, bg_color_1);
        lcd->drawNumber(distance, 250, 165);
    }
    else if (distance < 100.0)
    {
        // 2 digit
        lcd->setTextSize(4);
        lcd->fillRect(238, 165, 44, 28, bg_color_1);
        lcd->drawNumber(distance, 238, 165);
    }
    else if (distance < 1000.0)
    {
        // 3 digit
        lcd->setTextSize(3);
        lcd->fillRect(234, 165, 51, 28, bg_color_1);
        lcd->drawNumber(distance, 234, 168);
    }
    else
    {
        // 4 digit
        lcd->setTextSize(2);
        lcd->fillRect(234, 168, 51, 21, bg_color_1);
        lcd->drawNumber(distance, 237, 172);
    }

    last_selected_item = selecting_item;
}

void update_screen_2(LGFX *lcd, int *buf, int *scale)
{
        lcd->fillScreen(0xffff);
        lcd->setTextSize(2);
        lcd->setCursor(41, 20);
        lcd->print("HEART RATE THRESHOLD");

        lcd->setTextSize(4);
        lcd->setCursor(41, 80);
        lcd->print("NOW:");

        lcd->drawNumber(*buf, 203, 80);

        lcd->setCursor(41, 152);
        lcd->print("SET:");

        lcd->drawNumber(*buf + *scale, 203, 152);
}

void update_screen_3(LGFX *lcd, float *buf, int *scale)
{
        lcd->fillScreen(0xffff);
        lcd->setTextSize(2);
        lcd->setCursor(59, 20);
        lcd->print("STRIDE LENGTH (M)");

        lcd->setTextSize(4);
        lcd->setCursor(41, 80);
        lcd->print("NOW:");

        lcd->drawFloat(*buf, 1, 203, 80);

        lcd->setCursor(41, 152);
        lcd->print("SET:");

        lcd->drawFloat(*buf + 0.1 * (*scale), 1, 203, 152);
}