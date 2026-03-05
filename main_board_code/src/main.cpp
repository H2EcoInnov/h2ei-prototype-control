#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <SD.h>

// ==== Pins ====
const int fanPwmPin       = 18;
const int fcTempPin       = 40;
const int fcVoltagePin    = 28;
const int fcCurrentPin    = 27;
const int evPurgePin      = 3;

// ==== CAN ====
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> canBus;

// ==== CAN test / resend ====
unsigned long lastCanTestTx = 0;
const unsigned long canTestIntervalMs = 10000; // 10 secondes

// Mémorisation dernière trame reçue
static CAN_message_t lastRxMsg;
static bool haveLastRxMsg = false;

// ==== Variation ventilateur test ====
unsigned long lastFanChange = 0;
const unsigned long fanChangeInterval = 10000; // 10 secondes
uint8_t fanTestSpeed = 0;
bool fanIncreasing = true;

// ==== Modes ====
enum SystemMode { STARTUP, NORMAL };
SystemMode mode = STARTUP;

unsigned long startupStartTime      = 0;
const unsigned long startupDuration = 10000;

// ==== Purge ====
unsigned long lastPurgeTime = 0;
const unsigned long purgeIntervalStartup = 10000;
const unsigned long purgeIntervalNormal  = 30000;
const unsigned long purgeDurationMs = 500;
bool purgeActive = false;
unsigned long purgeStartTime = 0;

// ==== Sécurité ====
const float MIN_VOLTAGE = 20.0;
const float MAX_TEMP = 70.0;

// =======================
// ===== CAN RX =========
// =======================
void handleIncomingCAN() {
  CAN_message_t msg;

  while (canBus.read(msg)) {
    // Sauvegarde la dernière trame reçue
    lastRxMsg = msg;
    haveLastRxMsg = true;

    Serial.print("[CAN RX] ID=0x");
    Serial.print(msg.id, HEX);
    Serial.print(" LEN=");
    Serial.print(msg.len);
    Serial.print(" DATA=");
    for (uint8_t i = 0; i < msg.len; i++) {
      if (msg.buf[i] < 0x10) Serial.print('0');
      Serial.print(msg.buf[i], HEX);
      Serial.print(" ");
    }
    Serial.println();
  }
}

// =======================
// ===== CAN TX =========
// =======================
void sendCanTestFrame(unsigned long now) {
  if (now - lastCanTestTx >= canTestIntervalMs) {
    lastCanTestTx = now;

    CAN_message_t msg;
    msg.id  = 0x555;
    msg.len = 8;

    msg.buf[0] = 0x54; // T
    msg.buf[1] = 0x45; // E
    msg.buf[2] = 0x53; // S
    msg.buf[3] = 0x54; // T

    msg.buf[4] = (now >> 24) & 0xFF;
    msg.buf[5] = (now >> 16) & 0xFF;
    msg.buf[6] = (now >> 8)  & 0xFF;
    msg.buf[7] = (now)       & 0xFF;

    bool ok = canBus.write(msg);

    Serial.print("[CAN TX] Test frame ");
    Serial.println(ok ? "OK" : "FAIL");
  }
}

// Renvoie la dernière trame reçue (si dispo), sinon envoie la trame de test
void resendLastRxOrSendTest(unsigned long now) {
  if (now - lastCanTestTx >= canTestIntervalMs) {
    lastCanTestTx = now;

    if (haveLastRxMsg) {
      // Attention: on renvoie exactement la dernière trame reçue
      bool ok = canBus.write(lastRxMsg);

      Serial.print("[CAN TX] Resend last RX ");
      Serial.println(ok ? "OK" : "FAIL");
    } else {
      // Pas encore reçu de trame => on envoie la trame de test
      CAN_message_t msg;
      msg.id  = 0x555;
      msg.len = 8;

      msg.buf[0] = 0x54; // T
      msg.buf[1] = 0x45; // E
      msg.buf[2] = 0x53; // S
      msg.buf[3] = 0x54; // T

      msg.buf[4] = (now >> 24) & 0xFF;
      msg.buf[5] = (now >> 16) & 0xFF;
      msg.buf[6] = (now >> 8)  & 0xFF;
      msg.buf[7] = (now)       & 0xFF;

      bool ok = canBus.write(msg);

      Serial.print("[CAN TX] Test frame ");
      Serial.println(ok ? "OK" : "FAIL");
    }
  }
}

// ================================
// ===== Variation ventilateur ====
// ================================
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

    Serial.print("Vitesse ventilateur : ");
    Serial.println(fanTestSpeed);
  }
}

// =======================
// ===== Purge ===========
// =======================
void handlePurgeCycle(unsigned long now) {

  unsigned long interval = (mode == STARTUP) ? purgeIntervalStartup : purgeIntervalNormal;

  if (!purgeActive && now - lastPurgeTime >= interval) {
    digitalWrite(evPurgePin, HIGH);
    purgeStartTime = now;
    purgeActive = true;
    Serial.println("Purge activée");
  }

  if (purgeActive && now - purgeStartTime >= purgeDurationMs) {
    digitalWrite(evPurgePin, LOW);
    purgeActive = false;
    lastPurgeTime = now;
    Serial.println("Purge terminée");
  }
}

// =======================
// ===== SETUP ===========
// =======================
void setup() {

  Serial.begin(115200);
  delay(200);

  pinMode(fanPwmPin, OUTPUT);
  analogWriteFrequency(fanPwmPin, 25000);
  analogWrite(fanPwmPin, 0);

  pinMode(evPurgePin, OUTPUT);
  digitalWrite(evPurgePin, LOW);

  // ==== CAN INIT ====
  canBus.begin();
  canBus.setBaudRate(500000); // adapte si besoin
  Serial.println("CAN3 initialisé à 500k");

  startupStartTime = millis();
}

// =======================
// ===== LOOP ============
// =======================
void loop() {

  unsigned long now = millis();

  // ===== CAN =====
  handleIncomingCAN();
  resendLastRxOrSendTest(now); // <-- remplace l'envoi test direct

  // ===== Variation ventilateur =====
  handleFanVariation(now);

  // ===== Purge =====
  handlePurgeCycle(now);
}