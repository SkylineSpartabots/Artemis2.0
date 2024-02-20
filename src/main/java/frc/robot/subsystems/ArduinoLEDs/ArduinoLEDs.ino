#include <FastLED.h>

#define NUM_LEDS 300

#define DATA_PIN 3


// Define the array of leds
CRGB leds[NUM_LEDS];



//Ant Variables - ag and b are function specific - passed as params - k is global and used by methods and kflip and kreset so it needs to be global, starts at 0 
int k = 0; // Turns every nth LED off
// int g = 6; // Size of colored section
// int b = 3; // Size of other colored sections - eats into g size; set b = 0 to have g = g; otherwise g will appear as g - b; adjust g accordingly (desired g of 7 andb of 2 means g should be 9)
// b is also used to flip k so as to make it set what was previously the first color to the other in cautionJump

int inByte = 0;
int selected;
boolean antEnabled = false;

void setup() {
  Serial.begin(9600);

  FastLED.addLeds<NEOPIXEL, DATA_PIN>(leds, NUM_LEDS);  // GRB ordering is assumed
  FastLED.clearData();
}

void loop() {
  if (Serial.available() > 0){
    inByte = Serial.read();
    Serial.println(inByte);
    //TODO decide what each means and which should be ants (rem 9 - rn its just for demo)
    switch (inByte) {
      case '0': //OFF
        setSolid(0, 0, 0);
        break;
      case '1'://RED
        setSolid(0, 255, 255);
        break;
      case '2'://ORANGE
       setSolid(15, 255, 255);
        break;
      case '3'://YELLOW
        setSolid(55, 255, 255);
        break;
      case '4'://GREEN
        setSolid(96, 255, 255);
        break;
      case '5'://BLUE
        setSolid(160, 255, 255);
        break;
      case '6'://PURPLE
        setSolid(192, 255, 255);
        break;
      case '7'://PINK
        setSolid(224, 255, 255);
        break;
      case '8'://WHITE
        setSolid(0, 0, 127);
        break;
      case '9'://redAnt - just demo really rn
        antEnabled = true;
        runAnt(7, 3, 35);
        break;
    }
  }
  if (antEnabled){
    runAnt(7, 3, 35);
  }
  delay(0);
}

void runAnt(int g, int b, int delayMS){// Add differing color functionality
  for (int i = 0; i < NUM_LEDS; i++) {
    if((i - k)% g ==0) { 
      for (int j = 0; j < b; j++){
        if (i - j >= 0){
          leds[i - j] = CRGB::Black;
        }
      }
    }
    else{leds[i] = CHSV(0, 255, 255);}
  } 
  FastLED.show();
  delay(delayMS);
  kReset(g);
}
void setSolid(int H, int S, int V){
  antEnabled = false;
  for (int i =0 ; i < NUM_LEDS; i++){
    leds[i].setHSV(H, S, V);
  }
  FastLED.show();
}
// Extra Modes
void cautionAnt(int g, int b, int delayMS){
  
  for (int i = 0; i < NUM_LEDS; i++) {
    if((i - k)% g ==0) { 
      for (int j = 0; j < b; j++){ // set all the LEDs behind i but before b
        if (i - j >= 0){ // make sure we arent trying to set an LED that isnt on the strip
          leds[i - j].setHSV(25, 255, 150);
        }
      }
    }
    else{leds[i] = CHSV(0, 255, 255);}
  }
  FastLED.show();
  delay(delayMS);
  kReset(g);
}
void cautionJump(int g, int b, int delayMS){

  for (int i = 0; i < NUM_LEDS; i++) {
    if((i - k)% g == 0) { 
      for (int j = 0; j < b; j++){ // set all the LEDs behind i but before b
        if (i - j >= 0){ // make sure we arent trying to set an LED that isnt on the strip
          leds[i - j].setHSV(25, 255, 150);
        }
      }
    }
    else{leds[i] = CHSV(0, 255, 255);}
  }
  FastLED.show();
  delay(delayMS);
 
  kFlip(b);
  
}


// --Utility Functions--
void kReset(int g){
  if (k == g){
    k = 0;
  }
  else {
    k++;
  }
}
void kFlip(int b){

  if (k == b){
    k = 0;
  }
  else {
    k = b;
  }  
}

