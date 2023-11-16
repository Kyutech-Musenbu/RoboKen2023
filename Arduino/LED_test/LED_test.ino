#include <FastLED.h>
#define numberOfLEDs 11
#define controlPin 4
#define controlPin2 5

#define change_step 2


#define RED 'r'
#define BLUE 'b'
#define GREEN 'g'

CRGB leds[numberOfLEDs];

void setHSVColor(int hue, int saturation, int value) {
  for (int thisLED = 0; thisLED < numberOfLEDs; thisLED++) {
    leds[thisLED] = CHSV(hue, saturation, value);
  }
  FastLED.show();
}

int BLUE_HUE = 171;
int RED_HUE = 0;
int GREEN_HUE = 100;

int VALUE = 255;
int SATURATION = 255;

void setup() {
  Serial.begin(9600);
  FastLED.addLeds<WS2812B, controlPin, GRB>(leds, numberOfLEDs);
  FastLED.addLeds<WS2812B, controlPin2, GRB>(leds, numberOfLEDs);
}
char inkey;       // 入力文字格納用

void loop() {

  // 読み込み情報の有無確認
  if( Serial.available()) {
    //　シリアルから値の読み込み
    inkey = Serial.read();
    // シリアルポートへ出力
    Serial.print( inkey );
    // delay(100);
  }

   switch(inkey){
    case BLUE:
      Serial.print("blue mode");
        for(int value=0; value < 255; value += change_step){
            setHSVColor(BLUE_HUE, SATURATION, value);  // 色相、彩度、明度を設定
            delay(10);
        }

        for(int value=255; value > 0; value -= change_step){
            setHSVColor(BLUE_HUE, SATURATION, value);  // 色相、彩度、明度を設定
            delay(10);
        }
        break;



  case RED:
    Serial.print("red mode");
    setHSVColor(RED_HUE, SATURATION, VALUE);
    break;


  case GREEN:
    Serial.print("green mode");
     setHSVColor(GREEN_HUE, SATURATION, VALUE);
    break;

  default:
      break;

  }


}
