#include <Arduino.h>

struct Ultrasonic {
  byte trig;
  byte echo;
  int  last;      
};

struct Motor {
  byte en;        
  byte in1;       
  byte in2;       
  int  trim;      
  bool invert;    
};

struct StuckMonitor {
  int           ref[3];      
  unsigned long since;       
  int           level;       
  unsigned long lastEvent;   
};

enum { FRONT = 0, LEFT = 1, RIGHT = 2 };
Ultrasonic sensors[3] = {
  {12, 13, 200},   // FRONT  (trig, echo, last)
  {A0, A1, 200},   // LEFT
  {A2, A3, 200}    // RIGHT
};

Motor leftMotor  = {6, 9,  10, 10,  false};
Motor rightMotor = {5, 7,  8,  0,   false};


const int  CRUISE_SPEED   = 115;   
const int  SLOW_SPEED     = 105;   
const int  TURN_SPEED     = 130;   
const int  BACK_SPEED     = 125;   

const int  BACK_ON_MS     = 60;
const int  BACK_OFF_MS    = 70;
const int  TURN_ON_MS     = 40;
const int  TURN_OFF_MS    = 70;
const int  TURN_PULSES_90 = 6;     

// Distances (cm)
const int  FRONT_SLOW_CM  = 45;    
const int  WALL_CM        = 10;    
const int  BACKOFF_CM     = 14;    
const int  NOSE_MIN_CM    = 3;     
const int  FRONT_MIN_CM   = 4;     
const int  SIDE_OPEN_CM   = 13;    
const int  TARGET_SIDE_CM = 3;     
const int  CLEAR_FRONT_CM = 25;    
const int  GAP_CLEAR_CM   = 12;    

const int  BACK_PULSES_MAX   = 10; 
const int  NO_TARGET         = 999;

// Steering
const float KP            = 6.0;   
const int  MAX_CORR       = 40;    

// Thinking
const int  THINK_PAUSE_MS = 150;   
const int  LOOK_SAMPLES   = 3;     
const int  LOOK_DELAY_MS  = 20;    
const int  BRAKE_MS       = 40;    

const float CM_PER_SEC    = 25.0;  
const int  ADVANCE_CM     = 12;    
const int  GAP_CONFIRM    = 2;     
const int  SETTLE_MS      = 500;   

// Stuck detection
const int  STUCK_DELTA_CM    = 2;     
const unsigned long STUCK_MS_DRIVE = 900UL;  
const unsigned long STUCK_MS_TURN  = 500UL;  
const int  STUCK_BLIND_CM    = 150;   
const unsigned long STUCK_FORGET_MS = 3000UL; 
const unsigned long REPEAT_WALL_MS  = 1500UL; 

// Stopping
const unsigned long RUN_LIMIT_MS = 85000UL;  // always stop before the 90 s Auto phase ends
const bool PARK_STOP_ENABLED = true;   // stop when the front sees far open space
const int  PARK_FAR_CM    = 150;       // "far open space" = at least this far
const int  PARK_CONFIRM   = 4;         // readings in a row
const unsigned long PARK_MIN_RUN_MS = 6000UL;   // never park-stop in the first 6 s
const bool DEBUG          = true;      // print what the robot thinks (only visible with USB)

// ---------- GLOBALS ----------
int  dist[3];                          // latest FRONT, LEFT, RIGHT distances (cm)
StuckMonitor mon = {{0, 0, 0}, 0, 0, 0};
unsigned long startTime;
unsigned long ignoreSideUntil = 0;
unsigned long lastWallAt = 0;
int leftGapCount = 0;
int frontCount = 0;
int parkCount = 0;

// ---------- FUNCTIONS: motors ----------

// speed: -255..255 (negative = backward). Uses a POINTER to a Motor struct.
void setMotor(const Motor *m, int speed) {
  int mag = abs(speed);
  if (mag > 0) mag = constrain(mag + m->trim, 0, 255);
  bool forward = (speed >= 0);
  if (m->invert) forward = !forward;
  digitalWrite(m->in1, forward ? HIGH : LOW);
  digitalWrite(m->in2, forward ? LOW : HIGH);
  analogWrite(m->en, mag);
}

// Electric brake: both inputs HIGH shorts the motor, so it stops almost at once.
void brakeMotor(const Motor *m) {
  digitalWrite(m->in1, HIGH);
  digitalWrite(m->in2, HIGH);
  analogWrite(m->en, 255);
}

void driveMotors(int l, int r) {
  setMotor(&leftMotor, l);
  setMotor(&rightMotor, r);
}

void stopMotors() { driveMotors(0, 0); }

void brakeMotors() {
  brakeMotor(&leftMotor);
  brakeMotor(&rightMotor);
  delay(BRAKE_MS);
  stopMotors();
}

// ---------- FUNCTIONS: sensors ----------

// One distance reading in cm, or -1 if no echo came back.
int readCm(const Ultrasonic *s) {
  digitalWrite(s->trig, LOW);  delayMicroseconds(2);
  digitalWrite(s->trig, HIGH); delayMicroseconds(10);
  digitalWrite(s->trig, LOW);
  unsigned long dur = pulseIn(s->echo, HIGH, 12000UL);   // ~2 m max
  if (dur == 0) return -1;
  return

  (int)(dur / 58);
}

// One quick reading, remembered in the struct (POINTER).
// No echo while very close before = still too close (sensors cannot see under ~2 cm).
int readFiltered(Ultrasonic *s) {
  int c = readCm(s);
  if (c < 0) s->last = (s->last < 8) ? 1 : 200;
  else       s->last = c;
  delay(4);                       // let the echo fade before the next sensor fires
  return s->last;
}

// Fill d[] with FRONT, LEFT, RIGHT quick readings.
void readAll(int d[]) {
  for (int i = 0; i < 3; i++) d[i] = readFiltered(&sensors[i]);
}

// Careful reading while standing still: median of several readings.
int medianReading(Ultrasonic *s) {
  int v[5];
  int n = constrain(LOOK_SAMPLES, 1, 5);
  for (int i = 0; i < n; i++) {
    int c = readCm(s);
    if (c < 0) c = (s->last < 8) ? 1 : 200;
    v[i] = c;
    delay(LOOK_DELAY_MS);
  }
  for (int i = 0; i < n - 1; i++)                 // tiny bubble sort
    for (int j = 0; j < n - 1 - i; j++)
      if (v[j] > v[j + 1]) { int t = v[j]; v[j] = v[j + 1]; v[j + 1] = t; }
  s->last = v[n / 2];
  return s->last;
}

// LOOK: careful readings of FRONT, LEFT, RIGHT (robot must stand still).
void lookAround(int d[]) {
  for (int i = 0; i < 3; i++) d[i] = medianReading(&sensors[i]);
}

// ---------- FUNCTIONS: stuck detection ----------

// Start watching from now: remember the current sensor values.
void resetStuck(StuckMonitor *m) {
  for (int i = 0; i < 3; i++) m->ref[i] = dist[i];
  m->since = millis();
}

// Call this after every look while the motors are running.
// Returns true if NO sensor has changed by STUCK_DELTA_CM for limitMs.
bool updateStuck(StuckMonitor *m, const int cur[], unsigned long limitMs) {
  bool changed = false;
  for (int i = 0; i < 3; i++) {
    if (abs(cur[i] - m->ref[i]) >= STUCK_DELTA_CM) changed = true;
  }
  if (changed || cur[FRONT] >= STUCK_BLIND_CM) {   // moving, or too open to judge
    for (int i = 0; i < 3; i++) m->ref[i] = cur[i];
    m->since = millis();
    return false;
  }
  return (millis() - m->since) > limitMs;
}

// ---------- FUNCTIONS: movement in small steps ----------

// Move for onMs, stop for offMs, then LOOK (dist[] is refreshed).
void pulse(int l, int r, int onMs, int offMs) {
  driveMotors(l, r);
  delay(onMs);
  brakeMotors();
  delay(offMs);
  readAll(dist);
}

// Back off SLOWLY in small steps. Stops when the front sensor reads targetFrontCm
// or more, or after maxPulses steps (there is no rear sensor, so the limit matters!).
void backOffSlowly(int targetFrontCm, int maxPulses) {
  for (int i = 0; i < maxPulses; i++) {
    if (dist[FRONT] >= targetFrontCm) break;
    pulse(-BACK_SPEED, -BACK_SPEED, BACK_ON_MS, BACK_OFF_MS);
  }
  delay(THINK_PAUSE_MS);
}

// One small turn step. dir = LEFT or RIGHT.
void turnPulse(int dir) {
  if (dir == LEFT) pulse(-TURN_SPEED,  TURN_SPEED, TURN_ON_MS, TURN_OFF_MS);
  else             pulse( TURN_SPEED, -TURN_SPEED, TURN_ON_MS, TURN_OFF_MS);
}

// Turn in small steps, looking after every step.
//   quarterTurns : 1 = 90 degrees, 2 = U-turn
//   minPercent   : never stop before this % of the nominal turn
//   clearCm      : stop when the front sees at least this (twice in a row)
// Safety: backs up one step if the nose gets too close to a wall, and wiggles
// free if the sensors stop changing (wheel stuck). Gives up after 3x nominal.
void turnUntilClear(int dir, int quarterTurns, int minPercent, int clearCm) {
  int nominal   = TURN_PULSES_90 * quarterTurns;
  int minPulses = nominal * minPercent / 100;
  int maxPulses = nominal * 3;
  int clearCount = 0;
  int wiggles = 0;
  resetStuck(&mon);
  for (int p = 0; p < maxPulses; p++) {
    turnPulse(dir);

    if (dist[FRONT] < NOSE_MIN_CM) {                // nose about to touch: back up a step
      if (DEBUG) Serial.println("  nose close -> back step");
      backOffSlowly(NO_TARGET, 2);
      resetStuck(&mon);
      clearCount = 0;
      continue;
    }
    if (updateStuck(&mon, dist, STUCK_MS_TURN)) {   // wheel stuck while turning
      if (DEBUG) Serial.println("  stuck in turn -> wiggle");
      backOffSlowly(NO_TARGET, 3);

(int)(dur / 58);
}

// One quick reading, remembered in the struct (POINTER).
// No echo while very close before = still too close (sensors cannot see under ~2 cm).
int readFiltered(Ultrasonic *s) {
  int c = readCm(s);
  if (c < 0) s->last = (s->last < 8) ? 1 : 200;
  else       s->last = c;
  delay(4);                       // let the echo fade before the next sensor fires
  return s->last;
}

// Fill d[] with FRONT, LEFT, RIGHT quick readings.
void readAll(int d[]) {
  for (int i = 0; i < 3; i++) d[i] = readFiltered(&sensors[i]);
}

// Careful reading while standing still: median of several readings.
int medianReading(Ultrasonic *s) {
  int v[5];
  int n = constrain(LOOK_SAMPLES, 1, 5);
  for (int i = 0; i < n; i++) {
    int c = readCm(s);
    if (c < 0) c = (s->last < 8) ? 1 : 200;
    v[i] = c;
    delay(LOOK_DELAY_MS);
  }
  for (int i = 0; i < n - 1; i++)                 // tiny bubble sort
    for (int j = 0; j < n - 1 - i; j++)
      if (v[j] > v[j + 1]) { int t = v[j]; v[j] = v[j + 1]; v[j + 1] = t; }
  s->last = v[n / 2];
  return s->last;
}

// LOOK: careful readings of FRONT, LEFT, RIGHT (robot must stand still).
void lookAround(int d[]) {
  for (int i = 0; i < 3; i++) d[i] = medianReading(&sensors[i]);
}

// ---------- FUNCTIONS: stuck detection ----------

// Start watching from now: remember the current sensor values.
void resetStuck(StuckMonitor *m) {
  for (int i = 0; i < 3; i++) m->ref[i] = dist[i];
  m->since = millis();
}

// Call this after every look while the motors are running.
// Returns true if NO sensor has changed by STUCK_DELTA_CM for limitMs.
bool updateStuck(StuckMonitor *m, const int cur[], unsigned long limitMs) {
  bool changed = false;
  for (int i = 0; i < 3; i++) {
    if (abs(cur[i] - m->ref[i]) >= STUCK_DELTA_CM) changed = true;
  }
  if (changed || cur[FRONT] >= STUCK_BLIND_CM) {   // moving, or too open to judge
    for (int i = 0; i < 3; i++) m->ref[i] = cur[i];
    m->since = millis();
    return false;
  }
  return (millis() - m->since) > limitMs;
}

// ---------- FUNCTIONS: movement in small steps ----------

// Move for onMs, stop for offMs, then LOOK (dist[] is refreshed).
void pulse(int l, int r, int onMs, int offMs) {
  driveMotors(l, r);
  delay(onMs);
  brakeMotors();
  delay(offMs);
  readAll(dist);
}

// Back off SLOWLY in small steps. Stops when the front sensor reads targetFrontCm
// or more, or after maxPulses steps (there is no rear sensor, so the limit matters!).
void backOffSlowly(int targetFrontCm, int maxPulses) {
  for (int i = 0; i < maxPulses; i++) {
    if (dist[FRONT] >= targetFrontCm) break;
    pulse(-BACK_SPEED, -BACK_SPEED, BACK_ON_MS, BACK_OFF_MS);
  }
  delay(THINK_PAUSE_MS);
}

// One small turn step. dir = LEFT or RIGHT.
void turnPulse(int dir) {
  if (dir == LEFT) pulse(-TURN_SPEED,  TURN_SPEED, TURN_ON_MS, TURN_OFF_MS);
  else             pulse( TURN_SPEED, -TURN_SPEED, TURN_ON_MS, TURN_OFF_MS);
}

// Turn in small steps, looking after every step.
//   quarterTurns : 1 = 90 degrees, 2 = U-turn
//   minPercent   : never stop before this % of the nominal turn
//   clearCm      : stop when the front sees at least this (twice in a row)
// Safety: backs up one step if the nose gets too close to a wall, and wiggles
// free if the sensors stop changing (wheel stuck). Gives up after 3x nominal.
void turnUntilClear(int dir, int quarterTurns, int minPercent, int clearCm) {
  int nominal   = TURN_PULSES_90 * quarterTurns;
  int minPulses = nominal * minPercent / 100;
  int maxPulses = nominal * 3;
  int clearCount = 0;
  int wiggles = 0;
  resetStuck(&mon);
  for (int p = 0; p < maxPulses; p++) {
    turnPulse(dir);

    if (dist[FRONT] < NOSE_MIN_CM) {                // nose about to touch: back up a step
      if (DEBUG) Serial.println("  nose close -> back step");
      backOffSlowly(NO_TARGET, 2);
      resetStuck(&mon);
      clearCount = 0;
      continue;
    }
    if (updateStuck(&mon, dist, STUCK_MS_TURN)) {   // wheel stuck while turning
      if (DEBUG) Serial.println("  stuck in turn -> wiggle");
      backOffSlowly(NO_TARGET, 3);

resetStuck(&mon);
      if (++wiggles >= 3) break;
      continue;
    }
    if (p + 1 >= minPulses && dist[FRONT] >= clearCm) clearCount++;
    else                                              clearCount = 0;
    if (clearCount >= 2) break;
  }
  brakeMotors();
}

// Drive forward for ms milliseconds, but stop early if a wall is very close in front.
void driveForwardChecked(int ms) {
  unsigned long t0 = millis();
  driveMotors(TURN_SPEED, TURN_SPEED);
  while (millis() - t0 < (unsigned long)ms) {
    if (readFiltered(&sensors[FRONT]) <= FRONT_MIN_CM) break;
  }
  brakeMotors();
  delay(60);
}

// Which side has room? Returns LEFT, RIGHT, or -1 if neither (dead end).
int pickSide(const int d[]) {
  if (d[LEFT]  > SIDE_OPEN_CM) return LEFT;
  if (d[RIGHT] > SIDE_OPEN_CM) return RIGHT;
  return -1;
}

// After any turn: stand still, look, forget old gap/stuck info.
void afterTurn() {
  delay(THINK_PAUSE_MS);
  readAll(dist);
  ignoreSideUntil = millis() + SETTLE_MS;
  leftGapCount = 0;
  frontCount = 0;
  resetStuck(&mon);
}

// ---------- FUNCTIONS: behaviours ----------

// STUCK: the motors run but nothing changes. Recovery grows with every repeat.
void recoverFromStuck() {
  unsigned long now = millis();
  if (now - mon.lastEvent > STUCK_FORGET_MS) mon.level = 0;
  mon.lastEvent = now;
  mon.level++;
  brakeMotors();
  if (DEBUG) { Serial.print("STUCK level "); Serial.println(mon.level); }

  int d[3];
  if (mon.level == 1) {
    // Probably a wheel/corner scraping a wall: back off a little, nudge away from the closer wall
    backOffSlowly(NO_TARGET, 4);
    lookAround(d);
    int away = (d[LEFT] < d[RIGHT]) ? RIGHT : LEFT;
    for (int i = 0; i < 2; i++) turnPulse(away);
  } else if (mon.level == 2) {
    // Still stuck: back off more, look, and turn to the open side
    backOffSlowly(NO_TARGET, 8);
    lookAround(d);
    int side = pickSide(d);
    if (side < 0) side = (d[LEFT] >= d[RIGHT]) ? LEFT : RIGHT;
    turnUntilClear(side, 1, 50, CLEAR_FRONT_CM);
  } else {
    // Really stuck: back off a lot and turn around
    backOffSlowly(NO_TARGET, 12);
    lookAround(d);
    int side = (d[LEFT] >= d[RIGHT]) ? LEFT : RIGHT;
    turnUntilClear(side, 2, 50, CLEAR_FRONT_CM);
    mon.level = 0;
  }
  afterTurn();
}

// WALL IN FRONT: stop, back off slowly, look, turn to the open side (or turn around).
void handleFrontWall() {
  brakeMotors();
  delay(THINK_PAUSE_MS);

  // A second wall right after the last one = the turn did not solve it = stuck in a corner
  if (millis() - lastWallAt < REPEAT_WALL_MS) {
    lastWallAt = millis();
    if (DEBUG) Serial.println("WALL again too soon -> stuck recovery");
    recoverFromStuck();
    return;
  }
  lastWallAt = millis();

  int d[3];
  lookAround(d);
  if (DEBUG) {
    Serial.print("WALL F:"); Serial.print(d[FRONT]);
    Serial.print(" L:"); Serial.print(d[LEFT]);
    Serial.print(" R:"); Serial.println(d[RIGHT]);
  }
  if (d[FRONT] > WALL_CM + 3) return;                // false alarm, the way is clear

  for (int i = 0; i < 3; i++) dist[i] = d[i];
  backOffSlowly(BACKOFF_CM, BACK_PULSES_MAX);        // 1) back off, slowly
  lookAround(d);                                     // 2) look again

  int side = pickSide(d);                            // 3) decide and turn
  if (side == LEFT) {
    if (DEBUG) Serial.println(" -> LEFT");
    turnUntilClear(LEFT, 1, 50, CLEAR_FRONT_CM);
  } else if (side == RIGHT) {
    if (DEBUG) Serial.println(" -> RIGHT");
    turnUntilClear(RIGHT, 1, 50, CLEAR_FRONT_CM);
  } else {
    if (DEBUG) Serial.println(" -> DEAD END, turn around");
    turnUntilClear(LEFT, 2, 50, CLEAR_FRONT_CM);
  }
  afterTurn();
}

// GAP ON THE LEFT: drive on until the wheels are in the middle of the gap, then turn left.
void handleLeftGap() {
  int advanceMs = (int)(ADVANCE_CM * 1000.0 / CM_PER_SEC);
  if (DEBUG) Serial.println("GAP LEFT");
  driveForwardChecked(advanceMs);
  readAll(dist);
  turnUntilClear(LEFT, 1, 90, GAP_CLEAR_CM);         // gap turn: do (almost) the whole 90
  afterTurn();
}

// ---------- SETUP ----------
void setup() {
  Serial.begin(9600);

for (int i = 0; i < 3; i++) {
    pinMode(sensors[i].trig, OUTPUT);
    pinMode(sensors[i].echo, INPUT);
  }
  Motor *ms[2] = {&leftMotor, &rightMotor};
  for (int i = 0; i < 2; i++) {
    pinMode(ms[i]->en, OUTPUT);
    pinMode(ms[i]->in1, OUTPUT);
    pinMode(ms[i]->in2, OUTPUT);
  }
  stopMotors();
  delay(2000);               // time to place the robot in the maze
  readAll(dist);
  resetStuck(&mon);
  startTime = millis();
}

// ---------- MAIN LOOP ----------
void loop() {
  // Always stop by ourselves before the Auto phase ends
  if (millis() - startTime > RUN_LIMIT_MS) {
    stopMotors();
    while (true) {}
  }

  readAll(dist);                                   // LOOK, every single loop

  if (DEBUG) {
    Serial.print("F:"); Serial.print(dist[FRONT]);
    Serial.print(" L:"); Serial.print(dist[LEFT]);
    Serial.print(" R:"); Serial.println(dist[RIGHT]);
  }

  // 0) Parking: front sees far open space (the parking zone opens toward the arena)
  if (PARK_STOP_ENABLED && millis() - startTime > PARK_MIN_RUN_MS) {
    if (dist[FRONT] >= PARK_FAR_CM) parkCount++;
    else                            parkCount = 0;
    if (parkCount >= PARK_CONFIRM) {
      brakeMotors();
      while (true) {}
    }
  }

  // 1) Wall in front -> stop, back off slowly, turn
  if (dist[FRONT] < WALL_CM) frontCount++;
  else                       frontCount = 0;
  if (frontCount >= 2) {
    frontCount = 0;
    leftGapCount = 0;
    handleFrontWall();
    return;
  }

  // 2) Gap on the left (seen several times in a row) -> go in
  bool sideCheckOn = (millis() > ignoreSideUntil);
  if (sideCheckOn && dist[LEFT] > SIDE_OPEN_CM) leftGapCount++;
  else                                          leftGapCount = 0;
  if (leftGapCount >= GAP_CONFIRM) {
    leftGapCount = 0;
    handleLeftGap();
    return;
  }

  // 3) Stuck? Motors are running but no sensor value changes.
  if (updateStuck(&mon, dist, STUCK_MS_DRIVE)) {
    recoverFromStuck();
    return;
  }

  // 4) Drive: calm when clear, slower near a wall, centered between the walls
  int speed = CRUISE_SPEED;
  if (dist[FRONT] < FRONT_SLOW_CM) {
    long f = constrain(dist[FRONT], WALL_CM, FRONT_SLOW_CM);
    speed = SLOW_SPEED + (long)(CRUISE_SPEED - SLOW_SPEED) * (f - WALL_CM) / (FRONT_SLOW_CM - WALL_CM);
  }

  int l = constrain(dist[LEFT],  0, SIDE_OPEN_CM);
  int r = constrain(dist[RIGHT], 0, SIDE_OPEN_CM);
  float error;
  if (dist[RIGHT] > SIDE_OPEN_CM)     error = l - TARGET_SIDE_CM;   // follow the left wall only
  else if (dist[LEFT] > SIDE_OPEN_CM) error = TARGET_SIDE_CM - r;   // follow the right wall only
  else                                error = l - r;                // center between both
  int corr = constrain((int)(KP * error), -MAX_CORR, MAX_CORR);
  driveMotors(speed - corr, speed + corr);
}

