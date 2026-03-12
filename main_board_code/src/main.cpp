#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <SD.h>

// ============================================================
// ======================== Broches ============================
// ============================================================
const int fanPwmPin       = 18;
const int fcTempPin       = 40;
const int fcVoltagePin    = 25;
const int fcCurrentPin    = 27;
const int evPurgePin      = 3;

// LEDs de statut analogique
const int ledLowVoltagePin  = 34;
const int ledHighCurrentPin = 33;

// ============================================================
// ========================= DEBUG =============================
// ============================================================
#define DEBUG_SERIAL 1

#if DEBUG_SERIAL
  #define DBG_BEGIN(...)    Serial.begin(__VA_ARGS__)
  #define DBG_PRINT(...)    Serial.print(__VA_ARGS__)
  #define DBG_PRINTLN(...)  Serial.println(__VA_ARGS__)
#else
  #define DBG_BEGIN(...)
  #define DBG_PRINT(...)
  #define DBG_PRINTLN(...)
#endif

static const uint32_t DEBUG_PERIOD_MS = 1000;
static uint32_t lastDebugTime = 0;

// ============================================================
// ========================= CAN ===============================
// ============================================================
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> canBus;

// Identifiants CAN
static const uint32_t ID_MOTORCONTROL1  = 0x18FF3330; // extended
static const uint32_t ID_PC_TRIGGER_STD = 0x123;      // standard

// Paramètres temporels de la séquence couple
static const uint32_t CAN_PERIOD_MS = 10;    // émission à 100 Hz
static const uint32_t CAN_RAMP_MS   = 5000;  // rampe fixe de 5 s

// Paramètres d'encodage de la consigne de couple
static const float   TORQUE_RES    = 0.015f;
static const int32_t TORQUE_OFFSET = -32127;

// Limites de sécurité sur les valeurs reçues
static const float    TORQUE_ABS_LIMIT = 8.0f;
static const uint32_t HOLD_MS_LIMIT    = 30000;

// Machine d'état de la séquence CAN
enum class CanSequenceMode : uint8_t { IDLE, RAMP, HOLD };
static CanSequenceMode canMode = CanSequenceMode::IDLE;

static uint32_t canSequenceStartTime = 0;
static uint32_t canLastTxTime = 0;

static float canTorqueTarget = 0.0f;
static uint32_t canHoldDurationMs = 0;

// ============================================================
// ============== Variation ventilateur de test ===============
// ============================================================
unsigned long lastFanChange = 0;
const unsigned long fanChangeInterval = 10000;
uint8_t fanTestSpeed = 0;
bool fanIncreasing = true;

// ============================================================
// ====================== Modes système ========================
// ============================================================
enum SystemMode { STARTUP, NORMAL };
SystemMode mode = STARTUP;

unsigned long startupStartTime      = 0;
const unsigned long startupDuration = 10000;

// ============================================================
// ======================== Purge ==============================
// ============================================================
unsigned long lastPurgeTime = 0;
const unsigned long purgeIntervalStartup = 10000;
const unsigned long purgeIntervalNormal  = 30000;
const unsigned long purgeDurationMs = 500;
bool purgeActive = false;
unsigned long purgeStartTime = 0;

// ============================================================
// ======================= Sécurité ============================
// ============================================================
const float MIN_VOLTAGE = 20.0;
const float MAX_TEMP    = 70.0;

// ============================================================
// ========= Mesure tension / courant / calibration ===========
// ============================================================

// Configuration ADC
const float ADC_REF_VOLTAGE = 3.3f;
const int   ADC_MAX_VALUE   = 4095;   // résolution 12 bits

// Pont diviseur de la mesure tension
// Vin --- 40.2k --- ADC --- 6.8k --- GND
const float VOLTAGE_R_TOP    = 40200.0f;
const float VOLTAGE_R_BOTTOM = 6800.0f;

// Seuils de pilotage des LEDs de test
const float LOW_VOLTAGE_LED_THRESHOLD  = 18.0f;
const float HIGH_CURRENT_LED_THRESHOLD = 1.0f;

// Paramètres de conversion du capteur de courant
const float CURRENT_ADC_ZERO_VOLTAGE        = 0.475f;   // tension ADC à 0 A
const float CURRENT_ADC_SENSITIVITY_V_PER_A = 0.0175f; // sensibilité ADC en V/A

// Nombre d'échantillons utilisé pour lisser les mesures analogiques
const int ANALOG_SAMPLES = 20;

// ============================================================
// ================= Outils généraux ==========================
// ============================================================

static float clampf(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

// Lecture d'un uint16 little-endian depuis un buffer CAN
static uint16_t u16le(const uint8_t *b) {
  return (uint16_t)b[0] | ((uint16_t)b[1] << 8);
}

static const char* systemModeToStr(SystemMode m) {
  switch (m) {
    case STARTUP: return "STARTUP";
    case NORMAL:  return "NORMAL";
    default:      return "?";
  }
}

static const char* canModeToStr(CanSequenceMode m) {
  switch (m) {
    case CanSequenceMode::IDLE: return "IDLE";
    case CanSequenceMode::RAMP: return "RAMP";
    case CanSequenceMode::HOLD: return "HOLD";
    default:                    return "?";
  }
}

void debugLogEvent(const char* msg) {
#if DEBUG_SERIAL
  DBG_PRINT("[EVENT] ");
  DBG_PRINTLN(msg);
#endif
}

// ============================================================
// ================= Fonctions de mesure ADC ==================
// ============================================================

// Lit une tension analogique moyennée sur plusieurs échantillons.
float readAverageAdcVoltage(int pin, int samples = ANALOG_SAMPLES) {
  uint32_t sum = 0;

  for (int i = 0; i < samples; i++) {
    sum += analogRead(pin);
    delayMicroseconds(200);
  }

  float averageCounts = static_cast<float>(sum) / samples;
  return averageCounts * ADC_REF_VOLTAGE / ADC_MAX_VALUE;
}

// Reconstruit la tension amont du pont diviseur à partir de la tension ADC.
float readFuelCellVoltage() {
  float adcVoltage = readAverageAdcVoltage(fcVoltagePin);
  return adcVoltage * (VOLTAGE_R_TOP + VOLTAGE_R_BOTTOM) / VOLTAGE_R_BOTTOM;
}

float readFuelCellCurrent() {
  float adcVoltage = readAverageAdcVoltage(fcCurrentPin);
  float current = (adcVoltage - CURRENT_ADC_ZERO_VOLTAGE) / CURRENT_ADC_SENSITIVITY_V_PER_A;

  if (current < 0.0f) {
    current = 0.0f;
  }

  return current;
}

// Met à jour les LEDs de statut à partir des mesures analogiques.
void updateStatusLeds(float measuredVoltage, float measuredCurrent) {
  digitalWrite(ledLowVoltagePin,  (measuredVoltage < LOW_VOLTAGE_LED_THRESHOLD)  ? HIGH : LOW);
  digitalWrite(ledHighCurrentPin, (measuredCurrent > HIGH_CURRENT_LED_THRESHOLD) ? HIGH : LOW);
}

// ============================================================
// ===================== CAN - Packing =========================
// ============================================================

// Insère une valeur encodée bit à bit dans la trame CAN.
static void writeMessage(uint8_t *command,
                         uint16_t start,
                         uint8_t length,
                         float value,
                         float resolution,
                         int32_t offset)
{
  uint32_t rawValue = (int32_t)(value / resolution) - offset;

  if (rawValue < (1UL << length)) {
    for (uint8_t i = 0; i < length; i++) {
      uint8_t bufferPosition = (start + i) / 8;
      uint8_t bitPosition    = (start + i) % 8;

      uint8_t valueToPush = rawValue & 0x01;
      rawValue >>= 1;

      uint8_t mask = ~(1U << bitPosition);
      command[bufferPosition] = (command[bufferPosition] & mask) | (valueToPush << bitPosition);
    }
  }
}

// Construit et envoie la trame MotorControl1 avec la consigne de couple demandée.
static void sendMotorControl1(float torqueAsked)
{
  uint8_t command[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

  if (torqueAsked > 0.0f) {
    writeMessage(command, 16, 16, torqueAsked, TORQUE_RES, TORQUE_OFFSET);
  }

  CAN_message_t msg;
  msg.id = ID_MOTORCONTROL1;
  msg.len = 8;
  msg.flags.extended = 1;
  memcpy(msg.buf, command, 8);

  canBus.write(msg);
}

// ============================================================
// ==================== CAN - Séquence =========================
// ============================================================

// Analyse les trames reçues et démarre une séquence si la trame trigger
// attendue est reçue.
void handleIncomingCAN(unsigned long now) {
  CAN_message_t rx;

  while (canBus.read(rx)) {
    if (!rx.flags.extended && rx.id == ID_PC_TRIGGER_STD && rx.len >= 4) {
      uint16_t torqueMilli = u16le(&rx.buf[0]);
      uint16_t holdMsIn    = u16le(&rx.buf[2]);

      canTorqueTarget   = clampf((float)torqueMilli / 1000.0f, 0.0f, TORQUE_ABS_LIMIT);
      canHoldDurationMs = min<uint32_t>(holdMsIn, HOLD_MS_LIMIT);

      canSequenceStartTime = now;
      canLastTxTime = 0;
      canMode = CanSequenceMode::RAMP;

#if DEBUG_SERIAL
      DBG_PRINT("[CAN RX] Trigger reçu | torqueTarget=");
      DBG_PRINT(canTorqueTarget, 3);
      DBG_PRINT(" Nm | hold=");
      DBG_PRINT(canHoldDurationMs);
      DBG_PRINTLN(" ms");
#endif
    }
  }
}

// Gère l'émission périodique de la séquence de couple.
void handleCanSequence(unsigned long now) {
  if (canMode == CanSequenceMode::IDLE) {
    return;
  }

  if (now - canLastTxTime < CAN_PERIOD_MS) {
    return;
  }
  canLastTxTime = now;

  if (canMode == CanSequenceMode::RAMP) {
    uint32_t elapsed = now - canSequenceStartTime;
    if (elapsed > CAN_RAMP_MS) {
      elapsed = CAN_RAMP_MS;
    }

    float alpha = (float)elapsed / (float)CAN_RAMP_MS;
    float torqueCmd = alpha * canTorqueTarget;

    sendMotorControl1(torqueCmd);

    if ((now - canSequenceStartTime) >= CAN_RAMP_MS) {
      canMode = CanSequenceMode::HOLD;
      canSequenceStartTime = now;
      debugLogEvent("Séquence CAN -> HOLD");
    }
  }
  else if (canMode == CanSequenceMode::HOLD) {
    sendMotorControl1(canTorqueTarget);

    if ((now - canSequenceStartTime) >= canHoldDurationMs) {
      sendMotorControl1(0.0f);
      canMode = CanSequenceMode::IDLE;
      debugLogEvent("Séquence CAN -> IDLE (fin maintien, couple à 0)");
    }
  }
}

// ============================================================
// ================= Variation ventilateur ====================
// ============================================================
void handleFanVariation(unsigned long now) {
  if (now - lastFanChange >= fanChangeInterval) {
    lastFanChange = now;

    if (fanIncreasing) {
      fanTestSpeed += 50;
      if (fanTestSpeed >= 250) {
        fanTestSpeed = 255;
        fanIncreasing = false;
      }
    } else {
      if (fanTestSpeed > 50) {
        fanTestSpeed -= 50;
      } else {
        fanTestSpeed = 0;
        fanIncreasing = true;
      }
    }

    analogWrite(fanPwmPin, fanTestSpeed);

#if DEBUG_SERIAL
    DBG_PRINT("[FAN] Nouvelle consigne PWM = ");
    DBG_PRINTLN(fanTestSpeed);
#endif
  }
}

// ============================================================
// ========================= Purge =============================
// ============================================================
void handlePurgeCycle(unsigned long now) {
  unsigned long interval = (mode == STARTUP) ? purgeIntervalStartup : purgeIntervalNormal;

  if (!purgeActive && now - lastPurgeTime >= interval) {
    digitalWrite(evPurgePin, HIGH);
    purgeStartTime = now;
    purgeActive = true;

#if DEBUG_SERIAL
    DBG_PRINT("[PURGE] EV ON | mode=");
    DBG_PRINT(systemModeToStr(mode));
    DBG_PRINT(" | interval=");
    DBG_PRINT(interval);
    DBG_PRINTLN(" ms");
#endif
  }

  if (purgeActive && now - purgeStartTime >= purgeDurationMs) {
    digitalWrite(evPurgePin, LOW);
    purgeActive = false;
    lastPurgeTime = now;

#if DEBUG_SERIAL
    DBG_PRINT("[PURGE] EV OFF | durée=");
    DBG_PRINT(purgeDurationMs);
    DBG_PRINTLN(" ms");
#endif
  }
}

// ============================================================
// ====================== Debug périodique =====================
// ============================================================
void handlePeriodicDebug(unsigned long now, float measuredVoltage, float measuredCurrent) {
#if DEBUG_SERIAL
  if (now - lastDebugTime >= DEBUG_PERIOD_MS) {
    lastDebugTime = now;

    DBG_PRINT("[STAT] t=");
    DBG_PRINT(now);
    DBG_PRINT(" ms | mode=");
    DBG_PRINT(systemModeToStr(mode));
    DBG_PRINT(" | Vfc=");
    DBG_PRINT(measuredVoltage, 2);
    DBG_PRINT(" V | Ifc=");
    DBG_PRINT(measuredCurrent, 2);
    DBG_PRINT(" A | FanPWM=");
    DBG_PRINT((int)fanTestSpeed);
    DBG_PRINT(" | Purge=");
    DBG_PRINT(purgeActive ? "ON" : "OFF");
    DBG_PRINT(" | CAN=");
    DBG_PRINT(canModeToStr(canMode));
    DBG_PRINT(" | TorqueTarget=");
    DBG_PRINT(canTorqueTarget, 3);
    DBG_PRINT(" Nm | Hold=");
    DBG_PRINT(canHoldDurationMs);
    DBG_PRINTLN(" ms");
  }
#endif
}

// ============================================================
// ========================= Setup =============================
// ============================================================
void setup() {
  DBG_BEGIN(115200);
  while (!Serial && millis() < 2000);
  delay(300);

  DBG_PRINTLN("");
  DBG_PRINTLN("===== Boot Teensy =====");

  // Initialisation de la sortie PWM du ventilateur
  pinMode(fanPwmPin, OUTPUT);
  analogWriteFrequency(fanPwmPin, 25000);
  analogWrite(fanPwmPin, 0);

  // Initialisation de la commande d'électrovanne de purge
  pinMode(evPurgePin, OUTPUT);
  digitalWrite(evPurgePin, LOW);

  // Initialisation des LEDs de statut
  pinMode(ledLowVoltagePin, OUTPUT);
  pinMode(ledHighCurrentPin, OUTPUT);
  digitalWrite(ledLowVoltagePin, LOW);
  digitalWrite(ledHighCurrentPin, LOW);

  // Configuration ADC
  analogReadResolution(12);
  analogReadAveraging(16);

  // Initialisation du bus CAN3
  canBus.begin();
  canBus.setBaudRate(250000);

  // Initialisation des états système
  canMode = CanSequenceMode::IDLE;
  startupStartTime = millis();

  DBG_PRINTLN("[INIT] PWM fan OK");
  DBG_PRINTLN("[INIT] Purge EV OK");
  DBG_PRINTLN("[INIT] ADC OK");
  DBG_PRINTLN("[INIT] CAN3 @ 250000 bps OK");
  DBG_PRINTLN("[INIT] Mode système = STARTUP");
}

// ============================================================
// ========================== Loop =============================
// ============================================================
void loop() {
  unsigned long now = millis();

  // Transition automatique du mode STARTUP vers NORMAL
  if (mode == STARTUP && (now - startupStartTime >= startupDuration)) {
    mode = NORMAL;
    debugLogEvent("Transition STARTUP -> NORMAL");
  }

  // Réception CAN et lancement éventuel d'une séquence
  handleIncomingCAN(now);

  // Émission périodique de la séquence CAN
  handleCanSequence(now);

  // Variation de vitesse du ventilateur pour test
  handleFanVariation(now);

  // Gestion cyclique de la purge
  handlePurgeCycle(now);

  // Mesures analogiques
  float measuredVoltage = readFuelCellVoltage();
  float measuredCurrent = readFuelCellCurrent();

  // Mise à jour des LEDs d'état à partir des mesures analogiques
  updateStatusLeds(measuredVoltage, measuredCurrent);

  // Debug périodique
  handlePeriodicDebug(now, measuredVoltage, measuredCurrent);
}