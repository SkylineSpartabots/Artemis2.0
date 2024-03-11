#include <FastLED.h>

#define NUM_LEDS 300

#define DATA_PIN 4


// Define the array of leds
CRGB leds[NUM_LEDS];


// --Ant Variables--
int k = 0; // Turns every nth LED off -- global; is flipped/reset when incremented to primarySize
// Set each time function is run - can be changed
int primarySize; // Size of colored (primary) section
int secondarySize; // Size of black (secondary) sections - eats into g size; set b = 0 to have g = g; otherwise g will appear as g - b; adjust g accordingly (desired g of 7 and b of 2 means g should be 9)

int hueAnt; // Hue input into an ant method call
int satAnt; // Saturation input into an ant method call
int valAnt; // Value/Brightness input into an ant method call

int antDelay; // Delay input into an ant method call
boolean antEnabled = false;

      // MAKE ALL THESE TRIPPLE VARIABLES INTO TUPLES
// --Flash Variables--
int hueFlash; // Hue input into a flash method call
int satFlash; // Saturation input into a flash method call
int valFlash; // Value/Brightness input into a flash method call

int flashOnDelay; // On delay input into a flash method call
int flashOffDelay; // Off delay input into a flash method call

int flashCountLimit; // Number of times to flash on/off
int currentFlashCount = 1; // Current amount flashed. Incremented each time flashSolid is called

boolean flashEnabled = false;


// --Serial and Mode--
byte inData = 0;
int selected;


// --Color Tuples HSV--
int  OFF[] = {0, 0, 0};
int  RED[] = {0, 255, 255};
int  ORANGE[] = {15, 255, 255};
int  YELLOW[] = {55, 255, 255};
int  GREEN[] = {96, 255, 255};
int  BLUE[] = {160, 255, 255};
int  PURPLE[] = {192, 255, 255};
int  PINK[] = {224, 255, 255};
int  WHITE[] = {0, 0, 127}; // Dimmed right now cause i dont think the lil arduino can supply enough power at full bright


void setup() {
  Serial.begin(9600);

  FastLED.addLeds<NEOPIXEL, DATA_PIN>(leds, NUM_LEDS);  // GRB ordering is assumed
  FastLED.clearData();
}

void loop() {
  if (Serial.available() > 0){
    
    digitalWrite(13, 1); // Debug

    inData = Serial.read();
    Serial.println(inData);
    
//     //TODO Decide on mode meanings and what patterns they should be
//     switch (inData) {
// //    ****WHEN CHANGING CASES YOU NEED TO CHANGE ENUM NAME IN ROBOT CODE****
//       case '0': //OFF
//         setSolid(0, 0, 0);
//         break;
//       case '1'://RED
//         setSolid(0, 255, 255);
//         break;
//       case '2'://ORANGE
//        setSolid(15, 255, 255);
//         break;
//       case '3'://YELLOW
//         setSolid(55, 255, 255);
//         break;
//       case '4'://GREEN
//         setSolid(96, 255, 255);
//         break;
//       case '5'://BLUE
//         setSolid(160, 255, 255);
//         break;
//       case '6'://PURPLE
//         setSolid(192, 255, 255);
//         break;
//       case '7'://PINK
//         flashSolid(224, 255, 255, 200, 200);
//         // setSolid(224, 255, 255);
//         break;
//       case '8'://WHITE
//         setSolid(0, 0, 127); // Dimmed right now cause i dont think the lil arduino can supply enough power at full bright
//         break;
//       case '9'://redAnt - just demo really rn
//         runAnt(PURPLE, 7, 3, 35);
//         break;
//     }
    switch (inData) { //  trying what the video did and using byte and hexadecimal codes
    //https://www.aqua-calc.com/convert/number/hexadecimal-to-decimal
//    ****WHEN CHANGING CASES YOU NEED TO CHANGE ENUM NAME IN ROBOT CODE****
      case 0x0: //OFF
        setSolid(0, 0, 0);
        break;
      case 0x1://RED
        setSolid(0, 255, 255);
        break;
      case 0x2://ORANGE
       setSolid(15, 255, 255);
        break;
      case 0x3://YELLOW
        setSolid(55, 255, 255);
        break;
      case 0x4://GREEN
        setSolid(96, 255, 255);
        break;
      case 5://BLUE
        setSolid(160, 255, 255);
        break;
      case '6'://PURPLE
        setSolid(192, 255, 255);
        break;
      case '7'://PINK
        flashSolid(224, 255, 255, 200, 200, 2);
        // setSolid(224, 255, 255);
        break;
      case '8'://WHITE
        setSolid(0, 0, 127); // Dimmed right now cause i dont think the lil arduino can supply enough power at full bright
        break;
      case '9'://redAnt - just demo really rn
        runAnt(PURPLE, 7, 3, 35);
        break;
    } 
  digitalWrite(13, 0); // Debug
  }

  if (antEnabled){runAnt(hueAnt, satAnt, valAnt, primarySize, secondarySize, antDelay);}
  else if (flashEnabled && currentFlashCount <= flashCountLimit) {
    flashSolid(hueFlash, satFlash, valFlash, flashOnDelay, flashOffDelay, flashCountLimit);
    }
  
  delay(0);
}

// --Color Mode Methods--
void runAnt(int color[], int g, int b, int delayMS){// Add differing color functionality
  // Set the variables so they can be used every loop cycle
  hueAnt = color[0];
  satAnt = color[1];
  valAnt = color[2];
  primarySize = g;
  secondarySize = b;
  antDelay = delayMS;
  antEnabled = true;
  flashEnabled = false;

  for (int i = 0; i < NUM_LEDS; i++) {
    if((i - k)% g ==0) {
      for (int j = 0; j < b; j++){
        if (i - j >= 0){
          leds[i - j] = CRGB::Black;
        }
      }
    }
    else{leds[i] = CHSV(color[0], color[1], color[2]);}
  }
  FastLED.show();
  delay(delayMS);
  kReset(g);
}
void setSolid(int color[]){
  antEnabled = false;
  flashEnabled = false;

  for (int i =0 ; i < NUM_LEDS; i++){
    leds[i].setHSV(color[0], color[1], color[2]);
  }
  FastLED.show();
}
void flashSolid(int color[], int onMS, int offMS, int counts){
  // Set the variables so they can be used every loop cycle
  hueFlash = color[0];
  satFlash = color[1];
  valFlash = color[2];
  flashOnDelay = onMS;
  flashOffDelay = offMS;
  flashCountLimit = counts;
  antEnabled = false;
  flashEnabled = true;

  for (int i = 0; i < NUM_LEDS; i++){
    leds[i].setHSV(color[0], color[1], color[2]);
  }
  FastLED.show();
  delay(onMS);

  for (int i = 0; i < NUM_LEDS; i++){
    leds[i] = CRGB::Black;
  }
  FastLED.show();
  delay(offMS);
  currentFlashCount++;

}


// --Overloading--
void runAnt(int H, int S, int V, int g, int b, int delayMS){
  // Set the variables so they can be used every loop cycle
  hueAnt = H;
  satAnt = S;
  valAnt = V;
  primarySize = g;
  secondarySize = b;
  antDelay = delayMS;
  antEnabled = true;
  flashEnabled = false;

  for (int i = 0; i < NUM_LEDS; i++) {
    if((i - k)% g ==0) {
      for (int j = 0; j < b; j++){
        if (i - j >= 0){
          leds[i - j] = CRGB::Black;
        }
      }
    }
    else{leds[i].setHSV(H, S, V);}
  }
  FastLED.show();
  delay(delayMS);
  kReset(g);
}
void setSolid(int H, int S, int V){
  antEnabled = false;
  flashEnabled = false;

  for (int i =0 ; i < NUM_LEDS; i++){
    leds[i].setHSV(H, S, V);
  }
  FastLED.show();
}
void flashSolid(int H, int S, int V, int onMS, int offMS, int counts){
  // Set the variables so they can be used every loop cycle
  hueFlash = H;
  satFlash = S;
  valFlash = V;
  flashOnDelay = onMS;
  flashOffDelay = offMS;
  flashCountLimit = counts;
  antEnabled = false;
  flashEnabled = true;

  for (int i = 0; i < NUM_LEDS; i++){
    leds[i].setHSV(H, S, V);
  }
  FastLED.show();
  delay(onMS);

  for (int i = 0; i < NUM_LEDS; i++){
    leds[i] = CRGB::Black;
  }
  FastLED.show();
  delay(offMS);

}


// --Extra Modes--
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

