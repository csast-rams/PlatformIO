// Display.h
#ifndef DISPLAY_H
#define DISPLAY_H

#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>

#define BLACK   0x0000
#define WHITE   0xFFFF

#define TFT_CS  10
#define TFT_RST 9
#define TFT_DC  8

class Display {
public:
    Display();
    void initTFT();
    void StrtValues();
    void updateProgress(float val1, float val2, float val3);

private:
    Adafruit_ST7735 tft;
    int lastPercent1;
    int lastPercent2;
    int lastPercent3;

    void drawProgressBar(int x, int y, int w, int h, int percent, int &lastPercent, char16_t color);
    void updateTextValue(int x, int y, float val, int color);
};

#endif

// // #ifndef DISPLAY_H
// // #define DISPLAY_H

// // #include <Adafruit_GFX.h>
// // #include <Adafruit_ST7735.h>

// // #define BLACK    0x0000
// // #define BLUE     0x001F
// // #define RED      0xF800
// // #define GREEN    0x07E0
// // #define CYAN     0x07FF
// // #define MAGENTA  0xF81F
// // #define YELLOW   0xFFE0
// // #define WHITE    0xFFFF

// // // Pins definition
// // #define TFT_CS  10
// // #define TFT_RST 9
// // #define TFT_DC  8

// // class Display {
// // public:
// //     Display();                        // Constructor
// //     void initTFT();
// //     void drawCharacters(float val1, float val2, float val3);
// //     void StrtValues();
// //     void Flush();
// //     void Progressbar(int x, int y, int w, int h, int percent);
// //     void drawProgressBar(int x, int y, int w, int h, int percent);
// //     // void ValToString(int Val1, int Val2, int Val3);
// //     Adafruit_ST7735& getTFT();        // if you need external access

// // private:
// //     Adafruit_ST7735 tft;              // Private member instance
// //     // tft.flush();
// // };

// // //MAX1 = length of the string "PWM %: "
// // //MAX2 = length of the string "Freq: "
// // //MAX3 = length of the string "VOut: "
// // #define MAX1 7
// // #define MAX2 6
// // #define MAX3 6



// // #endif

// #ifndef DISPLAY_H
// #define DISPLAY_H

// #include <Adafruit_GFX.h>
// #include <Adafruit_ST7735.h>

// #define BLACK   0x0000
// #define WHITE   0xFFFF

// // Pins definition
// #define TFT_CS  10
// #define TFT_RST 9
// #define TFT_DC  8

// class Display {
// public:
//     Display();
//     void initTFT();
//     void drawStaticScreen();
//     void updateProgress(float val1, float val2, float val3);
//     void updateTextValue(int x, int y, float val, int color, float lastVal);
//     void drawCharacters(float val1, float val2, float val3);
//     void Flush();

//     Adafruit_ST7735& getTFT();
    
// private:
//     Adafruit_ST7735 tft;
//     void drawProgressBar(int x, int y, int w, int h, int percent, int lastPercent);
//     int lastVal1, lastVal2, lastVal3;
// };

// #endif