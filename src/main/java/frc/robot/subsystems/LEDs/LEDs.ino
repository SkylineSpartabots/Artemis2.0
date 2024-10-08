#include <FastLED.h>
#define NUM_LEDS 300

#define DATA_PIN 12

CRGB leds[NUM_LEDS];

// DIO Pin States
boolean pin2 = false;
boolean pin3 = false;
boolean pin4 = false;
boolean pin5 = false;

int inputAsNumber = 0; // LED mode converted to decimal from binary DIO inputs

// --Color Tuples HSV--
int OFF[] = {0, 0, 0};
int RED[] = {0, 255, 255};
int ORANGE[] = {15, 255, 255};
int YELLOW[] = {55, 255, 255};
int GREEN[] = {96, 255, 255};
int BLUE[] = {160, 255, 255};
int PURPLE[] = {192, 255, 255};
int PINK[] = {224, 255, 255};
int WHITE[] = {0, 0, 127}; // Dimmed right now cause i dont think the lil arduino can supply enough power at full bright

// Ant Variables
int k = 0; // Turns every nth LED off -- global; is flipped/reset when incremented to primarySize


// Debug
boolean state = true; // State of the onboard LED
unsigned long count = 0; // Used to flip onboard LED state

void setup() {
  Serial.begin(9600);

  FastLED.addLeds<NEOPIXEL, DATA_PIN>(leds, NUM_LEDS); 
  FastLED.clearData();

  pinMode(LED_BUILTIN, OUTPUT);

  pinMode(DATA_PIN, OUTPUT);
  pinMode(2, INPUT);
  pinMode(3, INPUT);
  pinMode(4, INPUT);
  pinMode(5, INPUT);
}

void loop() {
  // Read and convert from DIO pins
  pin2 = digitalRead(2);
  pin3 = digitalRead(3);
  pin4 = digitalRead(4);
  pin5 = digitalRead(5);
  inputAsNumber = pin5 + (2 * pin4) + (4 * pin3) + (8 * pin2); // Convert to decimal

  // Switching between modes based upon read value
  switch (inputAsNumber) {
  case 0:
    setSolid(OFF);
    break;
  case 1:
    setSolid(RED);
    break;
  case 2:
    setSolid(ORANGE); // Shooter at Speed
    break;
  case 3:
    flashSolid(50, 250, ORANGE); // Shooter Ramping
    break;
  case 4:
    setSolid(GREEN); // Intake Success
    break;
  case 5:
    flashSolid(50, 50, GREEN); // Intaking
    break;
  case 6:
    setSolid(PURPLE);
    break;
  case 7:
    setSolid(PINK);
    break;
  case 8:
    setSolid(WHITE);
    break;
  case 9:
    // runAnt(PURPLE, ORANGE, 10, 5, 10);
    runAnt(PURPLE, 10, 5, 10);  
    break;
  default:
    setSolid(OFF);
    break;
  }

  // Debug
  Serial.print(pin2);
  Serial.print(pin3);
  Serial.print(pin4);
  Serial.println(pin5);
  Serial.print("Num: ");
  Serial.println(inputAsNumber);

  digitalWrite(LED_BUILTIN, state);

  if (count > 10) {
    state = !state;
  }
  count++;

  delay(50);
}

// ---METHODS--- //
void setSolid(int color[]) { // Display a solid color
  for (int i = 0; i < NUM_LEDS; i++) { // Loop through LED strand
    leds[i].setHSV(color[0], color[1], color[2]); // Set the color of that LED
  }
  FastLED.show(); // Display the strand once done
}

void flashSolid(int onMS, int offMS, int PRIMARY[]) { // Flash provided color on and off for given times

  setSolid(PRIMARY);
  
  FastLED.show();
  delay(onMS);

  setSolid(OFF);

  FastLED.show();
  delay(offMS);
}

void flashSolid(int primaryMS, int secondaryMS, int PRIMARY[], int SECONDARY[]){ // Alternate between given colors for given times

  setSolid(PRIMARY);

  FastLED.show();
  delay(primaryMS);

  setSolid(SECONDARY);

  FastLED.show();
  delay(secondaryMS);
}
void runAnt(int PRIMARY[], int groupSize, int backSize, int delayMS){
  runAnt(PRIMARY, OFF, groupSize, backSize, delayMS); // Just call the other one but with black as the Secondary
}

void runAnt(int PRIMARY[], int SECONDARY[], int groupSize, int backSize, int delayMS){

  for (int i = 0; i < NUM_LEDS; i++) { // Loop through the LED strand
    if((i - k) % groupSize ==0) { // If the current LED is divisible by the groupsize it should be off
      for (int j = 0; j < backSize; j++){ // Loop backwards to create a larger black section
        if (i - j >= 0){ // While j is before i set the LED off
          leds[i - j] = CHSV(SECONDARY[0], SECONDARY[1], SECONDARY[2]);
        }
      }
    }
    // If the LED isnt divisible by the groupsize then make it a color
    else{leds[i] = CHSV(PRIMARY[0], PRIMARY[1], PRIMARY[2]);} 
  }

  FastLED.show();
  delay(delayMS);
  kReset(groupSize);
}

// --Utility Functions--
void kReset(int g){ // Checks if k reached the groupsize
  if (k == g){
    k = 0;
  }
  else {
    k++;
  }
}