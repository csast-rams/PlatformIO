

#include "Display.h"

Display::Display() : tft(TFT_CS, TFT_DC, TFT_RST), lastPercent1(-1), lastPercent2(-1), lastPercent3(-1) {}

void Display::initTFT() {
    tft.initR(INITR_BLACKTAB);
    tft.fillScreen(BLACK);
}

void Display::StrtValues() {
    tft.drawRect(3, 3, 125, 46, 0xFEC0);
    tft.drawRect(3, 51, 125, 46, 0x4C3F);
    // tft.drawRect(3, 99, 125, 46, 0xEC3F);

    tft.setCursor(6, 6);
    tft.setTextColor(0xFEC0); tft.setTextSize(1);
    tft.print("VOut:");

    tft.setCursor(6, 54);
    tft.setTextColor(0x4C3F); tft.setTextSize(1);
    tft.print("PWM %:");

    tft.setCursor(6, 102);
    tft.setTextColor(0xEC3F); tft.setTextSize(1);
    tft.print("Freq:");

    tft.setCursor(85, 113);
    tft.setTextColor(0xEC3F); tft.setTextSize(2);
    tft.print("Hz");

    // Draw initial progress bar frames:
    tft.drawRect(6, 28, 102, 18, WHITE);
    tft.drawRect(6, 74, 102, 18, WHITE);
    // tft.drawRect(6, 120, 102, 18, WHITE);
}

void Display::drawProgressBar(int x, int y, int w, int h, int percent, int &lastPercent, char16_t color) {
    percent = constrain(percent, 0, 100);

    if (percent != lastPercent) {
        // Clear previous progress bar area
        tft.fillRect(x, y, w, h, BLACK);

        // Draw filled portion
        int filledWidth = (w * percent) / 100;
        if (filledWidth > 0)
            tft.fillRect(x, y, filledWidth, h, color);

        // Redraw bar border
        tft.drawRect(x - 1, y - 1, w + 2, h + 2, WHITE);

        lastPercent = percent;
    }
}

void Display::updateTextValue(int x, int y, float val, int color) {
    tft.setTextColor(color, BLACK);
    tft.setCursor(x, y);
    tft.setTextSize(2);
    tft.fillRect(x, y, 70, 16, BLACK);
    tft.print(val, 1);
}

void Display::updateProgress(float val1, float val2, float val3) {
    int percent1 = constrain(map(val1 * 10, 0, 255, 0, 100), 0, 100)*2.6;
    int percent2 = constrain(map(val2, 0, 255, 0, 100), 0, 100)*2.6;
    // int percent3 = constrain(map(val3, 10, 500, 0, 100), 0, 100);

    drawProgressBar(6, 28, 100, 18, percent1, lastPercent1, 0xFEC0);
    drawProgressBar(6, 74, 100, 18, percent2, lastPercent2, 0x4C3F);
    // drawProgressBar(6, 120, 100, 18, percent3, lastPercent3);

    // Update numeric values
    updateTextValue(50, 7, val1, 0xFEC0);
    updateTextValue(50, 55, val2, 0x4C3F);
    updateTextValue(10, 113, val3, 0xEC3F);
}