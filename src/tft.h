#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_GFX.h>    // Core graphics library
#include <TFT_eSPI.h> // Hardware-specific library
int recColor[] = {TFT_RED, TFT_BLUE, TFT_GREEN, TFT_YELLOW, TFT_CYAN, TFT_MAGENTA};
TFT_eSPI tft = TFT_eSPI();
void setupLCD() {
  tft.begin();
  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);
  tft.invertDisplay( true );
  tft.setTextFont(1);
  tft.setTextSize(1);
  tft.setCursor(96, 0);
  tft.setTextColor(recColor[5]);
  tft.print("Ready");

}
void tftPrint(String label, int i) {
  tft.setCursor(96,  16);
  tft.setTextColor(recColor[i]);
  tft.print(label);
}
void tftPrints(String label, int x, int y, int bheight, int bwidth, int i) {
  tft.drawRect(x * 2, y * 2 - 8, bwidth * 2, bheight * 2, recColor[i]);
  tft.setCursor(96, i * 16 + 16);
  tft.setTextColor(recColor[i]);
  tft.print(label);

}
void tftClear() {
  tft.fillRect( 96, 0, 64, 80, 0);
}