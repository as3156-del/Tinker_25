#include <TM1637Display.h>

// === CONFIG ===
const int numRoads = 4;
const int baseGreen = 10;       // default seconds
const int minGreen = 10;
const int maxGreen = 40;
const int extraPool = 30;       // extra seconds distributed proportionally
const unsigned long samplingWindowMs = 3000; // sampling period to count vehicles
const int detectionThresholdCm = 50; // distance below which an object is considered present
const int yellowTime = 2;       // yellow duration seconds

// === PINS ===
// Traffic lights: for each road: {RED, YELLOW, GREEN}
const int lightPins[numRoads][3] = {
  {2, 3, 4},    // Road A
  {5, 6, 7},    // Road B
  {8, 9, 10},   // Road C
  {11, 12, 13}  // Road D
};

// Ultrasonic (Trig, Echo) per road
const int trigPins[numRoads] = {22, 24, 26, 28};
const int echoPins[numRoads] = {23, 25, 27, 29};

// TM1637 displays (CLK, DIO) per road
const int dispCLK[numRoads] = {30, 32, 34, 36};
const int dispDIO[numRoads] = {31, 33, 35, 37};
TM1637Display displays[numRoads] = {
  TM1637Display(dispCLK[0], dispDIO[0]),
  TM1637Display(dispCLK[1], dispDIO[1]),
  TM1637Display(dispCLK[2], dispDIO[2]),
  TM1637Display(dispCLK[3], dispDIO[3])
};

// === STATE ===
volatile int counts[numRoads];
bool objectPresent[numRoads];
int allocated[numRoads];

// Utility: read distance in cm (simple blocking read)
unsigned int readDistanceCM(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  unsigned long duration = pulseIn(echoPin, HIGH, 30000UL); // timeout 30 ms
  if (duration == 0) return 999; // no echo
  unsigned int cm = duration / 58;
  return cm;
}

// Show a number on a TM1637 display, showing up to 4 digits
void showNumberTM(int idx, int val) {
  if (val < 0) val = 0;
  if (val > 9999) val = 9999;
  displays[idx].showNumberDec(val, false);
}

void setup() {
  // LEDs
  for (int i=0;i<numRoads;i++){
    for (int j=0;j<3;j++){
      pinMode(lightPins[i][j], OUTPUT);
      digitalWrite(lightPins[i][j], LOW); // Start with all OFF
    }
  }
  // Ultrasonic pins
  for (int i=0;i<numRoads;i++){
    pinMode(trigPins[i], OUTPUT);
    pinMode(echoPins[i], INPUT);
    digitalWrite(trigPins[i], LOW);
    counts[i] = 0;
    objectPresent[i] = false;
  }
  // Displays init
  for (int i=0;i<numRoads;i++){
    displays[i].setBrightness(6); // 0..7
    showNumberTM(i, 0);
  }
  // Serial for debug
  Serial.begin(9600);
  delay(1000);
  Serial.println("=== Traffic System Started ===");
  Serial.println("Testing LED wiring...");
  Serial.println("If LEDs are ON when they should be OFF, you need to swap HIGH/LOW in code");
  
  // Test: Turn on only RED lights to verify wiring
  delay(2000);
  Serial.println("All RED lights should be ON now for 3 seconds...");
  allRed();
  delay(3000);
}

// Count vehicles during sampling window by polling all sensors frequently
void sampleCounts() {
  unsigned long start = millis();
  for (int i=0;i<numRoads;i++) counts[i] = 0;
  // reset objectPresent states
  for (int i=0;i<numRoads;i++) objectPresent[i] = false;

  while (millis() - start < samplingWindowMs) {
    for (int i=0;i<numRoads;i++) {
      unsigned int d = readDistanceCM(trigPins[i], echoPins[i]);
      bool nowPresent = (d <= detectionThresholdCm);
      // detect rising edge (no object -> object)
      if (nowPresent && !objectPresent[i]) {
        counts[i]++;           // new object arrived
        objectPresent[i] = true;
      } else if (!nowPresent && objectPresent[i]) {
        // object left
        objectPresent[i] = false;
      }
      // tiny pause to avoid hammering sensors
      delay(60);
    }
  }
  // debug
  Serial.print("Vehicle Counts: ");
  for (int i=0;i<numRoads;i++) {
    Serial.print("Road ");
    Serial.print(char('A' + i));
    Serial.print("=");
    Serial.print(counts[i]);
    Serial.print(i < numRoads-1 ? ", " : "\n");
  }
}

// Compute allocation proportionally to counts
void computeAllocation() {
  int total = 0;
  for (int i=0;i<numRoads;i++) total += counts[i];

  if (total == 0) {
    for (int i=0;i<numRoads;i++) allocated[i] = baseGreen;
  } else {
    // distribute extraPool proportionally
    for (int i=0;i<numRoads;i++) {
      float frac = (float)counts[i] / (float)total;
      int extra = (int)round(frac * extraPool);
      allocated[i] = baseGreen + extra;
      if (allocated[i] < minGreen) allocated[i] = minGreen;
      if (allocated[i] > maxGreen) allocated[i] = maxGreen;
    }
  }
  
  // Debug output
  Serial.print("Green Time Allocated: ");
  for (int i=0;i<numRoads;i++) {
    Serial.print("Road ");
    Serial.print(char('A' + i));
    Serial.print("=");
    Serial.print(allocated[i]);
    Serial.print("s");
    Serial.print(i < numRoads-1 ? ", " : "\n");
  }
}

// Turn all roads red
void allRed() {
  for (int i=0;i<numRoads;i++) {
    digitalWrite(lightPins[i][0], HIGH);  // RED ON
    digitalWrite(lightPins[i][1], LOW);   // YELLOW OFF
    digitalWrite(lightPins[i][2], LOW);   // GREEN OFF
  }
}

void loop() {
  Serial.println("\n========== NEW TRAFFIC CYCLE ==========");
  
  // 1) Sample traffic counts
  Serial.println("Sampling traffic...");
  sampleCounts();

  // 2) Compute allocated green times
  computeAllocation();

  // 3) Start with all roads RED
  Serial.println("\nAll roads: RED\n");
  allRed();
  delay(1000);

  // 4) Run traffic cycle for each road
  for (int r=0; r<numRoads; r++) {
    int greenTime = allocated[r];
    
    Serial.println("----------------------------");
    Serial.print("Road ");
    Serial.print(char('A' + r));
    Serial.println("'s Turn:");
    
    // Calculate total wait time for other roads (sum of remaining road times)
    int waitTimes[numRoads];
    for (int i=0; i<numRoads; i++) {
      if (i == r) {
        waitTimes[i] = 0; // Current road doesn't wait
      } else {
        // Calculate how long this road must wait
        int wait = 0;
        for (int j=r; j<numRoads; j++) {
          if (j != i) {
            wait += allocated[j] + yellowTime; // green time + yellow time
          }
        }
        // If road already passed, add full cycle time
        if (i < r) {
          for (int j=0; j<numRoads; j++) {
            wait += allocated[j] + yellowTime;
          }
        }
        waitTimes[i] = wait;
      }
    }
    
    // PHASE 1: Show YELLOW on current road for 2 seconds
    // All others show RED with countdown to their turn
    Serial.print("Road ");
    Serial.print(char('A' + r));
    Serial.println(": YELLOW (2s warning)");
    Serial.println("Other roads: RED (showing wait time)");
    
    for (int i=0; i<numRoads; i++) {
      if (i == r) {
        // Current road: YELLOW ON
        digitalWrite(lightPins[i][0], LOW);   // RED OFF
        digitalWrite(lightPins[i][1], HIGH);  // YELLOW ON
        digitalWrite(lightPins[i][2], LOW);   // GREEN OFF
      } else {
        // Other roads: RED ON
        digitalWrite(lightPins[i][0], HIGH);  // RED ON
        digitalWrite(lightPins[i][1], LOW);   // YELLOW OFF
        digitalWrite(lightPins[i][2], LOW);   // GREEN OFF
      }
    }
    
    // During yellow, update all displays
    for (int sec = yellowTime; sec >= 1; sec--) {
      for (int i=0; i<numRoads; i++) {
        if (i == r) {
          showNumberTM(i, sec); // Show yellow countdown
        } else {
          showNumberTM(i, waitTimes[i]); // Show wait time for other roads
        }
      }
      delay(1000);
      // Decrement wait times
      for (int i=0; i<numRoads; i++) {
        if (i != r) waitTimes[i]--;
      }
    }
    
    // PHASE 2: Show GREEN on current road
    // All others stay RED with countdown
    Serial.print("Road ");
    Serial.print(char('A' + r));
    Serial.print(": GREEN (");
    Serial.print(greenTime);
    Serial.println("s to pass)");
    Serial.println("Other roads: RED (showing wait time)");
    
    for (int i=0; i<numRoads; i++) {
      if (i == r) {
        // Current road: GREEN ON
        digitalWrite(lightPins[i][0], LOW);   // RED OFF
        digitalWrite(lightPins[i][1], LOW);   // YELLOW OFF
        digitalWrite(lightPins[i][2], HIGH);  // GREEN ON
      } else {
        // Other roads: RED ON
        digitalWrite(lightPins[i][0], HIGH);  // RED ON
        digitalWrite(lightPins[i][1], LOW);   // YELLOW OFF
        digitalWrite(lightPins[i][2], LOW);   // GREEN OFF
      }
    }
    
    // During green, countdown on current road, wait time on others
    for (int sec = greenTime; sec >= 1; sec--) {
      for (int i=0; i<numRoads; i++) {
        if (i == r) {
          showNumberTM(i, sec); // Show time left to pass
        } else {
          showNumberTM(i, waitTimes[i]); // Show wait time
        }
      }
      delay(1000);
      // Decrement wait times
      for (int i=0; i<numRoads; i++) {
        if (i != r) waitTimes[i]--;
      }
    }
    
    // PHASE 3: Back to all RED
    Serial.print("Road ");
    Serial.print(char('A' + r));
    Serial.println(": GREEN -> RED");
    
    allRed();
    
    // Update displays to show wait times
    for (int i=0; i<numRoads; i++) {
      if (waitTimes[i] > 0) {
        showNumberTM(i, waitTimes[i]);
      } else {
        showNumberTM(i, 0);
      }
    }
    
    delay(500);
  }

  Serial.println("\n========== CYCLE COMPLETE ==========");
}
