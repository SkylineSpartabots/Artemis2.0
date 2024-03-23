#include <FastLED.h>

#define NUM_LEDS 300

#define DATA_PIN 4

const int dataPin0 = 8;
const int dataPin1 = 9;
const int dataPin2 = 10;
const int dataPin3 = 11;

// Define the array of leds
CRGB leds[NUM_LEDS];


// --Ant Variables--
int k = 0; // Turns every nth LED off -- global; is flipped/reset when incremented to primarySize
// Set each time function is run - can be changed
int primarySize; // Size of colored (primary) section
int secondarySize; // Size of black (secondary) sections - eats into g size; set b = 0 to have g = g; otherwise g will appear as g - b; adjust g accordingly (desired g of 7 and b of 2 means g should be 9)

int antHSV[] = {0, 0, 0}; // Hue, Saturation, Value/Brightness input into an ant method call. Default {0, 0, 0}

int antDelay; // Delay input into an ant method call
boolean antEnabled = false;


// --Flash Variables--
int flashHSV[] = {0, 0, 0}; // Hue, Saturation, Value/Brightness input into a flash method call. Default {0, 0, 0}

int flashOnDelay; // On delay input into a flash method call
int flashOffDelay; // Off delay input into a flash method call

boolean flashEnabled = false;

boolean solidEnabled = false;

// --DIO and Mode--
int input0;
int input1;
int input2; 
int input3;

int inputAsNumber; // Int converted from binary
int prevInput; // Previously read input from rio, prevents running switch every loop so things dont come back on after time

unsigned long solidOnTime = millis();
unsigned long solidOffDelay = 5000;


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

  pinMode(dataPin0, INPUT);
  pinMode(dataPin1, INPUT);
  pinMode(dataPin2, INPUT);
  pinMode(dataPin3, INPUT);
}

void loop() {
  input0 = digitalRead(dataPin0);
  input1 = digitalRead(dataPin1);
  input2 = digitalRead(dataPin2); 
  input3 = digitalRead(dataPin3);

  inputAsNumber = input3 + (2*input2) + (4*input1) + (8*input0);
  // Serial.println(input0);
  //   Serial.println(input1);
  // Serial.println(input2);
  // Serial.println(input3);
  // Serial.println("\n");
    // Serial.println(inputAsNumber);

  if (inputAsNumber != prevInput){
    Serial.println("TRUE");
    //TODO Decide on mode meanings and what patterns they should be
    switch (inputAsNumber) {
      //****WHEN CHANGING CASES YOU NEED TO CHANGE ENUM NAME IN ROBOT CODE****
      case 0: //OFF
        setSolid(OFF);
              Serial.println("Case 0");

        break;
      case 1://RED
        setSolid(RED);
              Serial.println("Case 1");

        break;
      case 2://Shooter At Speed
        Serial.println("setting orange");
              Serial.println("Case 2");

        setSolid(ORANGE);
        break;
      case 3://Shooter Ramping
            Serial.println("Case 3");

        flashSolid(ORANGE, 200, 200);
        break;
      case 4://Intake Success
            Serial.println("Case 4");

        setSolid(GREEN);
        break;
      case 5://Intaking
            Serial.println("Case 5");

        flashSolid(GREEN, 200, 200);
        break;
      case 6://PURPLE
            Serial.println("Case 6");

        setSolid(PURPLE);
        break;
      case 7://PINK
      Serial.println("Case 7");
        flashSolid(PINK, 200, 200);
        // setSolid(224, 255, 255);
        break;
      case 8://WHITE
            Serial.println("Case 8");

        setSolid(WHITE); // Dimmed right now cause i dont think the lil arduino can supply enough power at full bright
        break;
      case 9://redAnt - just demo really rn
            Serial.println("Case 9");

        runAnt(PURPLE, 7, 3, 35);
        break;
    }
  }
  prevInput = inputAsNumber;
  if ((millis() - solidOnTime > solidOffDelay) && solidEnabled == true){   // 10 - 4 > 5
    Serial.println("OFF");
    solidEnabled = false;
    // Serial.println(solidEnabled);
    setSolid(OFF);
  }  
  // Shouldnt matter anymore cause the DIO pins should always be written to on the rio - they wont dissappear
  // if (antEnabled){runAnt(antHSV[0], antHSV[1], antHSV[2], primarySize, secondarySize, antDelay);}
  // else if (flashEnabled ) { //&& currentFlashCount <= flashCountLimit
  //   flashSolid(flashHSV[0], flashHSV[1], flashHSV[2], flashOnDelay, flashOffDelay);
  //   }
 

  digitalWrite(13, 0); // Debug  
  delay(0);
}

// --Color Mode Methods Tuple--
void runAnt(int color[], int g, int b, int delayMS){// Add differing color functionality
  // Set the variables so they can be used every loop cycle
  antHSV[0] = color[0];
  antHSV[1] = color[1];
  antHSV[2] = color[2];
  primarySize = g;
  secondarySize = b;
  antDelay = delayMS;
  antEnabled = true;
  flashEnabled = false;
  solidEnabled = false;

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
void flashSolid(int color[], int onMS, int offMS){

  // Set the variables so they can be used every loop cycle
  flashHSV[0] = color[0];
  flashHSV[1] = color[1];
  flashHSV[2] = color[2];
  flashOnDelay = onMS;
  flashOffDelay = offMS;
  antEnabled = false;
  flashEnabled = true;
  solidEnabled = false;

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
void setSolid(int color[]){
  antEnabled = false;
  flashEnabled = false;

  if (color != OFF){
      solidEnabled = true;

    Serial.println("NOT OFF");
      solidOnTime = millis();
  } else {solidEnabled = false;}

  Serial.println("ON1");


  for (int i =0 ; i < NUM_LEDS; i++){
    leds[i].setHSV(color[0], color[1], color[2]);
  }
  FastLED.show();
}


// --Color Mode Methods Integers--
void runAnt(int H, int S, int V, int g, int b, int delayMS){
  // Set the variables so they can be used every loop cycle
  antHSV[0] = H;
  antHSV[1] = S;
  antHSV[2] = V;
  primarySize = g;
  secondarySize = b;
  antDelay = delayMS;
  antEnabled = true;
  flashEnabled = false;
  solidEnabled = false;

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
void flashSolid(int H, int S, int V, int onMS, int offMS){
  // Set the variables so they can be used every loop cycle
  flashHSV[0] = H;
  flashHSV[1] = S;
  flashHSV[2] = V;
  flashOnDelay = onMS;
  flashOffDelay = offMS;
  antEnabled = false;
  flashEnabled = true;
  solidEnabled = false;

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
void setSolid(int H, int S, int V){
  antEnabled = false;
  flashEnabled = false;
  solidEnabled = true;

  solidOnTime = millis();
  Serial.println("ON2");

  for (int i =0 ; i < NUM_LEDS; i++){
    leds[i].setHSV(H, S, V);
  }
  FastLED.show();
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
