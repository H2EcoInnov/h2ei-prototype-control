#include <Arduino.h>
#include "driver/twai.h"

// ============================================================
// ==================== Configuration CAN ======================
// ============================================================

// Choisis ici les GPIO reliés au transceiver CAN
// Exemple courant sur ESP32 dev board + SN65HVD230 :
static const gpio_num_t CAN_TX_PIN = GPIO_NUM_21;
static const gpio_num_t CAN_RX_PIN = GPIO_NUM_22;

// Identifiants CAN
static const uint32_t ID_MOTORCONTROL1  = 0x18FF3330; // extended 29 bits
static const uint32_t ID_PC_TRIGGER_STD = 0x123;      // standard 11 bits

// Paramètres temporels de la séquence couple
static const uint32_t CAN_PERIOD_MS = 10;    // 100 Hz
static const uint32_t CAN_RAMP_MS   = 5000;  // rampe de 5 s

// Paramètres d'encodage de la consigne de couple
static const float   TORQUE_RES    = 0.015f;
static const int32_t TORQUE_OFFSET = -32127;

// Limites de sécurité sur les valeurs reçues
static const float    TORQUE_ABS_LIMIT = 8.0f;
static const uint32_t HOLD_MS_LIMIT    = 30000;

// ============================================================
// ========================= DEBUG =============================
// ============================================================
#define DEBUG_SERIAL 1

#if DEBUG_SERIAL
  #define DBG_BEGIN(...)    Serial.begin(__VA_ARGS__)
  #define DBG_PRINT(...)    Serial.print(__VA_ARGS__)
  #define DBG_PRINTLN(...)  Serial.println(__VA_ARGS__)
  #define DBG_PRINTF(...)   Serial.printf(__VA_ARGS__)
#else
  #define DBG_BEGIN(...)
  #define DBG_PRINT(...)
  #define DBG_PRINTLN(...)
  #define DBG_PRINTF(...)
#endif

static const uint32_t DEBUG_PERIOD_MS = 1000;
static uint32_t lastDebugTime = 0;

// ============================================================
// =================== Machine d'état CAN ======================
// ============================================================
enum class CanSequenceMode : uint8_t { IDLE, RAMP, HOLD };
static CanSequenceMode canMode = CanSequenceMode::IDLE;

static uint32_t canSequenceStartTime = 0;
static uint32_t canLastTxTime = 0;

static float canTorqueTarget = 0.0f;
static uint32_t canHoldDurationMs = 0;

// ============================================================
// ===================== Outils généraux =======================
// ============================================================
static float clampf(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

static uint16_t u16le(const uint8_t *b) {
  return (uint16_t)b[0] | ((uint16_t)b[1] << 8);
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
// ===================== Packing CAN ===========================
// ============================================================
static void writeMessage(uint8_t *command,
                         uint16_t start,
                         uint8_t length,
                         float value,
                         float resolution,
                         int32_t offset)
{
  int32_t signedRaw = (int32_t)(value / resolution) - offset;
  if (signedRaw < 0) {
    return;
  }

  uint32_t rawValue = (uint32_t)signedRaw;

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

// ============================================================
// ==================== Envoi MotorControl1 ====================
// ============================================================
static bool sendMotorControl1(float torqueAsked)
{
  uint8_t command[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

  if (torqueAsked > 0.0f) {
    writeMessage(command, 16, 16, torqueAsked, TORQUE_RES, TORQUE_OFFSET);
  }

  twai_message_t msg = {};
  msg.identifier = ID_MOTORCONTROL1;
  msg.extd = 1;     // trame étendue 29 bits
  msg.rtr = 0;
  msg.data_length_code = 8;
  memcpy(msg.data, command, 8);

  esp_err_t err = twai_transmit(&msg, pdMS_TO_TICKS(10));
  if (err != ESP_OK) {
#if DEBUG_SERIAL
    DBG_PRINTF("[CAN TX] Erreur twai_transmit(): %d\n", (int)err);
#endif
    return false;
  }

  return true;
}

// ============================================================
// ================== Réception / trigger ======================
// ============================================================
void handleIncomingCAN(uint32_t now)
{
  twai_message_t rx;

  while (twai_receive(&rx, 0) == ESP_OK) {
    if (!rx.extd && rx.identifier == ID_PC_TRIGGER_STD && rx.data_length_code >= 4) {
      uint16_t torqueMilli = u16le(&rx.data[0]);
      uint16_t holdMsIn    = u16le(&rx.data[2]);

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

// ============================================================
// =================== Séquence rampe/hold =====================
// ============================================================
void handleCanSequence(uint32_t now)
{
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
// ===================== Debug périodique ======================
// ============================================================
void handlePeriodicDebug(uint32_t now)
{
#if DEBUG_SERIAL
  if (now - lastDebugTime >= DEBUG_PERIOD_MS) {
    lastDebugTime = now;

    twai_status_info_t statusInfo;
    twai_get_status_info(&statusInfo);

    DBG_PRINT("[STAT] t=");
    DBG_PRINT(now);
    DBG_PRINT(" ms | CAN=");
    DBG_PRINT(canModeToStr(canMode));
    DBG_PRINT(" | TorqueTarget=");
    DBG_PRINT(canTorqueTarget, 3);
    DBG_PRINT(" Nm | Hold=");
    DBG_PRINT(canHoldDurationMs);
    DBG_PRINT(" ms | TX queued=");
    DBG_PRINT(statusInfo.msgs_to_tx);
    DBG_PRINT(" | RX queued=");
    DBG_PRINT(statusInfo.msgs_to_rx);
    DBG_PRINT(" | TX err=");
    DBG_PRINT(statusInfo.tx_error_counter);
    DBG_PRINT(" | RX err=");
    DBG_PRINT(statusInfo.rx_error_counter);
    DBG_PRINTLN("");
  }
#endif
}

// ============================================================
// ====================== Init TWAI ============================
// ============================================================
bool initCan()
{
  // Mode normal : il faut un autre noeud qui ACK sur le bus
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);

  // 250 kbit/s comme sur ta Teensy
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_250KBITS();

  // Filtre ouvert : on accepte tout
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  esp_err_t err = twai_driver_install(&g_config, &t_config, &f_config);
  if (err != ESP_OK) {
    DBG_PRINTF("[INIT] twai_driver_install() erreur: %d\n", (int)err);
    return false;
  }

  err = twai_start();
  if (err != ESP_OK) {
    DBG_PRINTF("[INIT] twai_start() erreur: %d\n", (int)err);
    return false;
  }

  DBG_PRINTLN("[INIT] TWAI démarré à 250 kbit/s");
  return true;
}

// ============================================================
// ========================== Setup ============================
// ============================================================
void setup()
{
  DBG_BEGIN(115200);
  delay(500);

  DBG_PRINTLN("");
  DBG_PRINTLN("===== Boot ESP32 TWAI =====");

  if (!initCan()) {
    DBG_PRINTLN("[FATAL] Init CAN échouée");
    while (true) {
      delay(1000);
    }
  }

  canMode = CanSequenceMode::IDLE;
}

// ============================================================
// =========================== Loop ============================
// ============================================================
void loop()
{
  uint32_t now = millis();

  handleIncomingCAN(now);
  handleCanSequence(now);
  handlePeriodicDebug(now);

  delay(1);
}