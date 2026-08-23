/* ==========================================================================================
 *  2-CHANNEL EOG/EMG ROBOT CAR | Pure EOG Directions | v6 SACCADE PATCHED + PLOTTER ON
 * ==========================================================================================
 */

#include <Arduino.h>

/* ------------------------------------------------------------------ 0. ENUMS ------------- */
enum Command : uint8_t { CMD_NONE, CMD_FORWARD, CMD_RIGHT, CMD_LEFT, CMD_REVERSE, CMD_ESTOP };
enum Motor   : uint8_t { M_STOP, M_FWD, M_REV, M_LEFT, M_RIGHT };

/* ------------------------------------------------------------------ 1. BUILD OPTIONS ----- */
#define CALIBRATION_MODE      0     
#define VERT_POLARITY       (+1)    
#define HORZ_POLARITY       (+1)    
#define TELEMETRY_DIVIDER     4     

/* ------------------------------------------------------------------ 2. HARDWARE PINS ----- */
constexpr uint8_t PIN_VERT  = 4;    // Pill #1 OUT (Vertical)
constexpr uint8_t PIN_HORZ  = 1;    // Pill #2 OUT (Horizontal)
constexpr uint8_t PIN_FWD   = 5;
constexpr uint8_t PIN_REV   = 6;
constexpr uint8_t PIN_LEFT  = 7;
constexpr uint8_t PIN_RIGHT = 8;

/* ------------------------------------------------------------------ 3. SAMPLING ---------- */
constexpr uint32_t TICK_LONG_US     = 3907;
constexpr uint32_t TICK_SHORT_US    = 3906;
constexpr uint32_t MAX_CATCHUP      = 5;
constexpr uint32_t SETTLE_MS        = 5000;   // 5s for the new 0.3Hz filter to settle
constexpr uint8_t  DC_PRIME_SAMPLES = 16;     

/* ------------------------------------------------------------------ 4. CH1 VERTICAL ------ */
constexpr float    BLINK_THRESHOLD      = 88.0f;  // Positive spike (FORWARD)
constexpr float    LOOK_DOWN_THRESHOLD  = -120.0f;// Negative spike (REVERSE)
constexpr float    BLINK_RELEASE        = 65.0f;  
constexpr uint32_t GAP_BRIDGE_MS        = 50;     
constexpr uint32_t MIN_BLINK_MS         = 40;     
constexpr uint32_t MAX_BLINK_MS         = 180;    
constexpr uint32_t REFRACTORY_MS        = 180;    

constexpr float    EMG_THRESHOLD        = 42.0f;  // Jaw Clench (E-STOP)
constexpr uint32_t SQUEEZE_MS           = 280;    
constexpr uint32_t EMG_GATE_MS          = 60;     
constexpr uint32_t POST_SQUEEZE_HOLD_MS = 220;

/* ------------------------------------------------------------------ 5. CH2 HORIZONTAL ---- */
constexpr float    HORZ_THRESHOLD    = 250.0f; // TUNE ME based on your graph
constexpr uint32_t HORZ_MIN_BLANK_MS = 400;
constexpr uint32_t HORZ_QUIET_MS     = 150;
constexpr float    HORZ_QUIET_FRAC   = 0.25f;
constexpr float    HORZ_VETO_RATIO   = 1.0f;

// 0 = Plotter Graph ON! (1 = Text Debug)
#define HORZ_DEBUG 0

/* ------------------------------------------------------------------ 6. MOTOR TIMING ------ */
constexpr uint32_t DUR_FORWARD_MS = 1500;
constexpr uint32_t DUR_TURN_MS    = 1000;
constexpr uint32_t DUR_REVERSE_MS = 1000;

/* ------------------------------------------------------------------ 7. BIQUAD CORE ------- */
struct Biquad { float b0, b1, b2, a1, a2; float z1, z2; };

static inline float biquadStep(Biquad &s, float x) {
  const float y = s.b0 * x + s.z1;
  s.z1 = s.b1 * x - s.a1 * y + s.z2;
  s.z2 = s.b2 * x - s.a2 * y;
  return y;
}
static inline float chainStep(Biquad *s, uint8_t n, float x) {
  for (uint8_t i = 0; i < n; ++i) x = biquadStep(s[i], x);
  return x;
}

/* ------------------------------------------------------------------ 8. FILTER BANK ------- */
static Biquad notchV = { 0.94211961f, -0.63478107f, 0.94211961f, -0.63478107f, 0.88423922f, 0, 0 };
static Biquad notchH = { 0.94211961f, -0.63478107f, 0.94211961f, -0.63478107f, 0.88423922f, 0, 0 };

static Biquad bpVert[4] = {
  { 0.00735282f,  0.01470564f, 0.00735282f, -0.95391350f, 0.25311356f, 0, 0 },
  { 1.00000000f,  2.00000000f, 1.00000000f, -1.20596630f, 0.60558332f, 0, 0 },
  { 1.00000000f, -2.00000000f, 1.00000000f, -1.97690645f, 0.97706395f, 0, 0 },
  { 1.00000000f, -2.00000000f, 1.00000000f, -1.99071687f, 0.99086813f, 0, 0 }
};

// 0.3 - 10Hz Saccade Bandpass (keeps 27% more signal)
static Biquad bpHorz[2] = {
  { 0.01208733f,  0.02417466f, 0.01208733f, -1.67285157f, 0.72159836f, 0, 0 },
  { 1.00000000f, -2.00000000f, 1.00000000f, -1.98962325f, 0.98968038f, 0, 0 }
};

static Biquad emgHP15 = { 0.77043329f, -1.54086657f, 0.77043329f, -1.48745243f, 0.59428071f, 0, 0 };

/* ------------------------------------------------------------------ 9. DESPIKER ---------- */
class Median5 {
  float buf[5] = {0, 0, 0, 0, 0};
  uint8_t idx = 0;
public:
  void prime(float v) { for (uint8_t i = 0; i < 5; ++i) buf[i] = v; }
  float step(float x) {
    buf[idx] = x;
    idx = (idx + 1) % 5;
    float t[5] = { buf[0], buf[1], buf[2], buf[3], buf[4] };
    for (uint8_t i = 1; i < 5; ++i) {              
      float k = t[i]; int8_t j = i - 1;
      while (j >= 0 && t[j] > k) { t[j + 1] = t[j]; --j; }
      t[j + 1] = k;
    }
    return t[2];
  }
};
static Median5 despikeV, despikeH;                 

/* ------------------------------------------------------------------ 10. EMG ENVELOPE ----- */
class Envelope {
  static constexpr uint8_t N = 32;
  float ring[N] = {0};
  float sum = 0.0f;
  uint8_t idx = 0;
public:
  float step(float x) {
    const float a = fabsf(x);
    sum += a - ring[idx];
    ring[idx] = a;
    idx = (idx + 1) & (N - 1);
    if (sum < 0.0f) sum = 0.0f;                    
    return sum * (1.0f / N);
  }
} emgEnvelope;

/* ------------------------------------------------------------------ 11. STATE ------------ */
static float    dcVert = 2048.0f, dcHorz = 2048.0f;
static float    accVert = 0.0f,   accHorz = 0.0f;
static uint8_t  primeCount = 0;
static bool     dcCaptured = false;

static float    vertFiltered = 0.0f;   
static float    horzFiltered = 0.0f;   
static float    emgLevel     = 0.0f;   

static uint32_t nextTickUs = 0;
static uint8_t  tickPhase  = 0;
static uint32_t bootMs     = 0;

static bool     spikeActive   = false;
static uint32_t spikeStartMs  = 0;
static uint32_t spikeLastHiMs = 0;
static uint32_t belowSinceMs  = 0;
static uint32_t blockUntilMs  = 0;

static uint32_t emgHighSinceMs = 0;
static bool     emgAbove       = false;
static bool     squeezeLatched = false;

static bool     vertDownArmed   = true;
static uint32_t vertDownFiredMs = 0;

static bool     horzArmed     = true;
static uint32_t horzFiredMs   = 0;
static uint32_t horzQuietFrom = 0;

#if HORZ_DEBUG
static float    dbgPeak      = 0.0f;   
static float    dbgPeakVeto  = 0.0f;   
static float    dbgVertAtPk  = 0.0f;   
static uint32_t dbgVetoTicks = 0, dbgTicks = 0;
static uint32_t dbgLastRepMs = 0, dbgLastWhyMs = 0;
#endif

static Motor    motorState   = M_STOP;
static uint32_t motorUntilMs = 0;
static float    triggerMark  = 0.0f;
static uint32_t triggerUntil = 0;
static uint8_t  telemetryDiv = 0;

/* ------------------------------------------------------------------ 12. MOTORS ----------- */
static inline void motorsAllOff() {
  digitalWrite(PIN_FWD, LOW);  digitalWrite(PIN_REV,   LOW);
  digitalWrite(PIN_LEFT, LOW); digitalWrite(PIN_RIGHT, LOW);
}

static void motorDrive(Motor m, uint32_t durationMs) {
#if CALIBRATION_MODE
  (void)m; (void)durationMs;
  motorState = M_STOP; motorsAllOff(); return;      
#else
  motorsAllOff();
  motorState = m;
  switch (m) {
    case M_FWD:   digitalWrite(PIN_FWD,   HIGH); break;
    case M_REV:   digitalWrite(PIN_REV,   HIGH); break;
    case M_LEFT:  digitalWrite(PIN_LEFT,  HIGH); break;
    case M_RIGHT: digitalWrite(PIN_RIGHT, HIGH); break;
    default: break;
  }
  motorUntilMs = millis() + durationMs;
#endif
}
static inline void motorStop() { motorsAllOff(); motorState = M_STOP; motorUntilMs = 0; }
static void motorService(uint32_t now) {
  if (motorState != M_STOP && (int32_t)(now - motorUntilMs) >= 0) motorStop();
}

/* ------------------------------------------------------------------ 13. COMMANDS --------- */
static void executeCommand(Command c, uint32_t now) {
  switch (c) {
    case CMD_FORWARD: motorDrive(M_FWD,   DUR_FORWARD_MS); triggerMark =  300.0f; break;
    case CMD_RIGHT:   motorDrive(M_RIGHT, DUR_TURN_MS);    triggerMark =  340.0f; break;
    case CMD_LEFT:    motorDrive(M_LEFT,  DUR_TURN_MS);    triggerMark = -340.0f; break;
    case CMD_REVERSE: motorDrive(M_REV,   DUR_REVERSE_MS); triggerMark = -300.0f; break;
    case CMD_ESTOP:   motorStop();                         triggerMark =  390.0f; break;
    default: return;
  }
  triggerUntil = now + 150;      
}

/* ------------------------------------------------------------------ 14. EOG DETECTORS ---- */
static void horzService(float h, uint32_t now, bool armed) {
  const float ah = fabsf(h);
  const float av = fabsf(vertFiltered);

#if HORZ_DEBUG
  ++dbgTicks;
  if (ah > dbgPeak) { dbgPeak = ah; dbgVertAtPk = av; }
  if (armed && horzArmed && spikeActive && av > ah * HORZ_VETO_RATIO) {
    ++dbgVetoTicks;
    if (ah > dbgPeakVeto) dbgPeakVeto = ah;
  }
  if ((uint32_t)(now - dbgLastRepMs) >= 500) {
    Serial.printf("[H] pk=%6.0f th=%4.0f armed=%d vertAtPk=%6.0f vetoPk=%6.0f veto=%3lu%% "
                  "emg=%5.0f settled=%d\n",
                  dbgPeak, HORZ_THRESHOLD, (int)horzArmed, dbgVertAtPk, dbgPeakVeto,
                  (unsigned long)(dbgTicks ? (100UL * dbgVetoTicks) / dbgTicks : 0UL),
                  emgLevel, (int)armed);
    dbgPeak = dbgPeakVeto = dbgVertAtPk = 0.0f;
    dbgVetoTicks = dbgTicks = 0;
    dbgLastRepMs = now;
  }
#endif

  if (!armed) return;

  /* ---------------- ARMED: look for a signed saccade ---------------- */
  if (horzArmed) {
    // FIXED VETO: only stand down when CH1 genuinely dominates.
    if (spikeActive && av > ah * HORZ_VETO_RATIO) {
#if HORZ_DEBUG
      if (ah > HORZ_THRESHOLD * 0.5f && (uint32_t)(now - dbgLastWhyMs) > 200) {
        Serial.printf("[H] REJECT veto : |h|=%.0f but |vert|=%.0f dominates (ratio %.2f)\n",
                      ah, av, (double)(av / (ah > 1.0f ? ah : 1.0f)));
        dbgLastWhyMs = now;
      }
#endif
      return;
    }

    if (h > HORZ_THRESHOLD) {
      executeCommand(CMD_RIGHT, now);
      horzArmed = false; horzFiredMs = now; horzQuietFrom = 0;
#if HORZ_DEBUG
      Serial.printf("[H] FIRE  RIGHT  h=%.0f\n", h);
#endif
    } else if (h < -HORZ_THRESHOLD) {
      executeCommand(CMD_LEFT, now);
      horzArmed = false; horzFiredMs = now; horzQuietFrom = 0;
#if HORZ_DEBUG
      Serial.printf("[H] FIRE  LEFT   h=%.0f\n", h);
#endif
    }
#if HORZ_DEBUG
    else if (ah > HORZ_THRESHOLD * 0.5f && (uint32_t)(now - dbgLastWhyMs) > 200) {
      Serial.printf("[H] REJECT small: |h|=%.0f < th=%.0f  (need %.0f%% more)\n",
                    ah, HORZ_THRESHOLD, (double)(100.0f * (HORZ_THRESHOLD - ah) / ah));
      dbgLastWhyMs = now;
    }
#endif
    return;
  }

  /* ---------------- BLANKED: wait for a genuine return to baseline ---------------- */
  if ((uint32_t)(now - horzFiredMs) < HORZ_MIN_BLANK_MS) return;

  if (ah < HORZ_THRESHOLD * HORZ_QUIET_FRAC) {
    if (horzQuietFrom == 0) horzQuietFrom = now;
    if ((uint32_t)(now - horzQuietFrom) >= HORZ_QUIET_MS) {
      horzArmed = true;
#if HORZ_DEBUG
      Serial.printf("[H] re-armed after %lu ms\n", (unsigned long)(now - horzFiredMs));
#endif
    }
  } else {
    horzQuietFrom = 0;                                          
#if HORZ_DEBUG
    if ((uint32_t)(now - horzFiredMs) > 3000 && (uint32_t)(now - dbgLastWhyMs) > 1000) {
      Serial.printf("[H] STUCK blanked %lus: |h|=%.0f never < %.0f\n",
                    (unsigned long)((now - horzFiredMs) / 1000), ah,
                    (double)(HORZ_THRESHOLD * HORZ_QUIET_FRAC));
      dbgLastWhyMs = now;
    }
#endif
  }
}

// --- LOOK DOWN (Reverse) ---
static void lookDownService(float v, uint32_t now, bool armed) {
  if (!armed || (int32_t)(now - blockUntilMs) < 0) return; 

  if (vertDownArmed) {
    if (v < LOOK_DOWN_THRESHOLD) {
      executeCommand(CMD_REVERSE, now);
      vertDownArmed = false; 
      vertDownFiredMs = now;
    }
  } else {
    if ((now - vertDownFiredMs) >= 500 && v > (LOOK_DOWN_THRESHOLD * 0.5f)) {
      vertDownArmed = true;
    }
  }
}

/* ------------------------------------------------------------------ 15. BLINK & CLENCH --- */
static void squeezeService(uint32_t now, bool armed) {
  if (emgLevel > EMG_THRESHOLD) {
    if (!emgAbove) { emgAbove = true; emgHighSinceMs = now; }

    if (armed && !squeezeLatched && (now - emgHighSinceMs) >= SQUEEZE_MS) {
      executeCommand(CMD_ESTOP, now);           
      squeezeLatched = true;
      spikeActive = false;       
    }
  } else {
    emgAbove = false;
    if (emgLevel < EMG_THRESHOLD * 0.5f) squeezeLatched = false;
  }

  if (squeezeLatched || (emgAbove && (now - emgHighSinceMs) >= EMG_GATE_MS)) {
    blockUntilMs = now + POST_SQUEEZE_HOLD_MS;
    spikeActive  = false;
  }
}

static void blinkService(float s, uint32_t now, bool armed) {
  if (!armed || (int32_t)(now - blockUntilMs) < 0) { spikeActive = false; return; }

  if (!spikeActive) {
    if (s > BLINK_THRESHOLD) {
      spikeActive = true; spikeStartMs = now; spikeLastHiMs = now; belowSinceMs = 0;
    }
    return;
  }
  
  if (s > BLINK_RELEASE) { spikeLastHiMs = now; belowSinceMs = 0; return; }

  if (belowSinceMs == 0) belowSinceMs = now;
  if ((now - belowSinceMs) <= GAP_BRIDGE_MS) return;    

  const uint32_t widthMs = spikeLastHiMs - spikeStartMs;
  spikeActive  = false;
  blockUntilMs = now + REFRACTORY_MS;                   

  if (widthMs < MIN_BLINK_MS)  return;
  if (widthMs <= MAX_BLINK_MS) {
    executeCommand(CMD_FORWARD, now);
  }
}

/* ------------------------------------------------------------------ 16. TELEMETRY -------- */
static void telemetryService() {
  if (++telemetryDiv < TELEMETRY_DIVIDER) return;
  telemetryDiv = 0;

#if !HORZ_DEBUG 
  char line[128];
  const int n = snprintf(line, sizeof(line),
        "PlotMin:-400,PlotMax:400,ThV:%d,ThH:%d,ThD:%d,VERT:%.0f,HORZ:%.0f,EMG:%.0f,Trig:%.0f\n",
        (int)BLINK_THRESHOLD, (int)HORZ_THRESHOLD, (int)LOOK_DOWN_THRESHOLD,
        vertFiltered, horzFiltered, emgLevel, triggerMark);
  if (n > 0 && Serial.availableForWrite() >= n) Serial.write((const uint8_t *)line, n);
#endif
}

/* ------------------------------------------------------------------ 17. SAMPLE TICK ------ */
static void processSample(uint32_t now) {
  const int rawV = analogRead(PIN_VERT);
  const int rawH = analogRead(PIN_HORZ);

  if (!dcCaptured) {
    accVert += (float)rawV; accHorz += (float)rawH;
    ++primeCount;
    if (primeCount < DC_PRIME_SAMPLES) return;
    dcVert = accVert / (float)DC_PRIME_SAMPLES;
    dcHorz = accHorz / (float)DC_PRIME_SAMPLES;
    dcCaptured = true;
    despikeV.prime(0.0f); despikeH.prime(0.0f);
  }

  // ---- CH1 vertical: blinks + look down + clench ----
  float v = ((float)rawV - dcVert) * (float)VERT_POLARITY;
  v = biquadStep(notchV, v);                 
  v = despikeV.step(v);                      
  vertFiltered = chainStep(bpVert, 4, v);    
  emgLevel = emgEnvelope.step(biquadStep(emgHP15, vertFiltered));   

  // ---- CH2 horizontal: saccades ----
  float h = ((float)rawH - dcHorz) * (float)HORZ_POLARITY;
  h = biquadStep(notchH, h);
  h = despikeH.step(h);
  horzFiltered = chainStep(bpHorz, 2, h);    

  const bool armed = (now - bootMs) > SETTLE_MS;

  squeezeService(now, armed);                
  blinkService(vertFiltered, now, armed);
  lookDownService(vertFiltered, now, armed); 
  horzService(horzFiltered, now, armed);

  telemetryService();
}

/* ------------------------------------------------------------------ 18. SETUP / LOOP ----- */
void setup() {
  Serial.begin(115200);

  pinMode(PIN_FWD, OUTPUT);  pinMode(PIN_REV,   OUTPUT);
  pinMode(PIN_LEFT, OUTPUT); pinMode(PIN_RIGHT, OUTPUT);
  motorsAllOff();                            

  analogReadResolution(12);                                  
  analogSetPinAttenuation(PIN_VERT, ADC_11db);               
  analogSetPinAttenuation(PIN_HORZ, ADC_11db);

  bootMs     = millis();
  nextTickUs = micros();
}

void loop() {
  const uint32_t nowUs = micros();

  if ((int32_t)(nowUs - nextTickUs) >= 0) {
    nextTickUs += (tickPhase == 0) ? TICK_LONG_US : TICK_SHORT_US;
    tickPhase = (tickPhase + 1) & 0x03;   

    if ((int32_t)(micros() - nextTickUs) > (int32_t)(MAX_CATCHUP * TICK_LONG_US)) {
      nextTickUs = micros() + TICK_SHORT_US;
    }
    processSample(millis());
  }

  const uint32_t nowMs = millis();
  motorService(nowMs);
  if (triggerMark != 0.0f && (int32_t)(nowMs - triggerUntil) >= 0) triggerMark = 0.0f;
}