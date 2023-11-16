#include "ros.h"
#include <std_msgs/Empty.h>
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


ros::NodeHandle nh;

bool mode = 0;
void messageCb( const std_msgs::Empty& toggle_msg) {
  mode = 1;
}

ros::Subscriber<std_msgs::Empty> sub("toggle_led", &messageCb );

void setup() {

  FastLED.addLeds<WS2812B, controlPin, GRB>(leds, numberOfLEDs);
  FastLED.addLeds<WS2812B, controlPin2, GRB>(leds, numberOfLEDs);
  nh.getHardware()->setBaud(115200);
  nh.initNode();

  nh.subscribe(sub);
}

void loop() {

  if(mode){
    setHSVColor(RED_HUE, SATURATION, VALUE);
  }else{
    setHSVColor(BLUE_HUE, SATURATION, VALUE);
  }

  nh.spinOnce();
  delay(30);

}
