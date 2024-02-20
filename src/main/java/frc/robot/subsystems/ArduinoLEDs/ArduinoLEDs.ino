#include <FastLED.h>

#define NUM_LEDS 300

#define DATA_PIN 3


// Define the array of leds
CRGB leds[NUM_LEDS];
//Math Explainer
/* Assuming a primarySize (g) of 5 and a blackOffset (k) of 0 and a secondarySize (b) of 5
The goal is to turn every 5th (g) LED (i) off 
To find if i is the 5th LED we subtract k from i to get some number which if it is the 5th LED it should be evenly divisible by 5 (g)
If (i - k) % g == 0 then the LED at i should be set to black
If (i - k) % g != 0 then the LED at i should be set to the desired color, in this case red

We first progress through every LED in the strip, every value of i
The above formula will turn every 5th LED off
(0 - 0) % 5 == 0 and (5 - 0) % 5 == 0 but (2 - 0) % 5 != 0


But we want the black spaces to move to create the animation

To do this we must increment k after we have looped throught the whole LED strip
This makes it so that every 5th LED is still off but it is now one value of i farther down the strip
For example: (1 - 1) % 5 == 0 and  (6 - 1) % 5 == 0 mean that LEDs 1 & 6 are black
but the previously black LED with i value of 5 will be red because (5 - 1) % 5 != 0

**Remember that i of 1 is actually the second LED in the stip as 0 is the first**

We continue this looping through every i value and then incrementing k until k is equal to g
When k == g we can reset it to be 0 because (5 - 0) % 5 == 0 and (5 - 5) % 5 == 0
This helps to keep numbers small

Now one final thing, what if we want more than one LED to be turned off
Simple we create a b value which tells us how many LEDs behind the currently off LED should also be set to black
It looks like this: if (i - k) % g == 0 then we will set the LEDs from i - b to i as black

*/

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


// --Flash Variables--
int hueFlash; // Hue input into a flash method call
int satFlash; // Saturation input into a flash method call
int valFlash; // Value/Brightness input into a flash method call

int flashOnDelay; // On delay input into a flash method call
int flashOffDelay; // Off delay input into a flash method call
boolean flashEnabled = false;


// --Serial and Mode--
int inByte = 0;
int selected;


// --Color Tuples--
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
    inByte = Serial.read();
    Serial.println(inByte);
    //TODO Decide on mode meanings and what patterns they should be
    switch (inByte) { 
//    ****WHEN CHANGING CASES YOU NEED TO CHANGE ENUM NAME IN ROBOT CODE****
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
        flashSolid(224, 255, 255, 200, 200);
        // setSolid(224, 255, 255);
        break;
      case '8'://WHITE
        setSolid(0, 0, 127); // Dimmed right now cause i dont think the lil arduino can supply enough power at full bright
        break;
      case '9'://redAnt - just demo really rn
        runAnt(PURPLE, 7, 3, 35);
        break;
    }
  }
  digitalWrite(13, 0); // Debug
  if (antEnabled){runAnt(hueAnt, satAnt, valAnt, primarySize, secondarySize, antDelay);} 
  else if (flashEnabled) {flashSolid(hueFlash, satFlash, valFlash, flashOnDelay, flashOffDelay);}
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
void flashSolid(int color[], int onMS, int offMS){
  // Set the variables so they can be used every loop cycle
  hueFlash = color[0];
  satFlash = color[1];
  valFlash = color[2];
  flashOnDelay = onMS;
  flashOffDelay = offMS;
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
void flashSolid(int H, int S, int V, int onMS, int offMS){
  // Set the variables so they can be used every loop cycle
  hueFlash = H;
  satFlash = S;
  valFlash = V;
  flashOnDelay = onMS;
  flashOffDelay = offMS;
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

