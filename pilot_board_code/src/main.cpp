#include <Arduino.h>
#include "driver/twai.h"

// ============================================================
// ==================== Configuration CAN ======================
// ============================================================

// ATTENTION : GPIO1 / GPIO3 = UART0 sur beaucoup d'ESP32
// Si tu utilises ces pins pour le CAN, le debug Serial peut être perturbé.
static const gpio_num_t CAN_TX_PIN = GPIO_NUM_1;
static const gpio_num_t CAN_RX_PIN = GPIO_NUM_3;

// ID CAN de la trame couple
static const uint32_t ID_MOTORCONTROL1 = 0x18FF3330; // extended 29 bits

// Période d'envoi CAN
static const uint32_t CAN_PERIOD_MS = 10; // 100 Hz

// ============================================================
// ================= Configuration potentiomètre ===============
// ============================================================

// Pin analogique du potentiomètre
static const int POT_PIN = 15;   // GPIO15

// Bornes réelles mesurées
static const int POT_RAW_MIN = 1025;  // ~0.854 V = 0%
static const int POT_RAW_MAX = 3215;  // ~2.510 V = 100%

// Couple max correspondant à 100%
static const float TORQUE_MAX_NM = 8.0f;  // à régler ici

// Seuil d'activation : en dessous -> 0 torque
// 0.02f = 2%
static const float TORQUE_ENABLE_THRESHOLD = 0.02f;  // <-- réglable ici

// Filtrage simple
static const float FILTER_ALPHA = 0.1f;   // 1.0 = pas de filtre

// ============================================================
// ================= Encodage consigne couple ==================
// ============================================================
static const float   TORQUE_RES    = 0.015f;
static const int32_t TORQUE_OFFSET = -32127;

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

static const uint32_t DEBUG_PERIOD_MS = 200;
static uint32_t lastDebugTime = 0;
static uint32_t lastCanTxTime = 0;

// ============================================================
// ====================== Variables globales ===================
// ============================================================
static int rawAdc = 0;
static float potNormFiltered = 0.0f;   // 0.0 -> 1.0
static float torqueCommandNm = 0.0f;

// ============================================================
// ===================== Outils généraux =======================
// ============================================================
static float clampf(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
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

  torqueAsked = clampf(torqueAsked, 0.0f, TORQUE_MAX_NM);

  if (torqueAsked > 0.0f) {
    writeMessage(command, 16, 16, torqueAsked, TORQUE_RES, TORQUE_OFFSET);
  }

  twai_message_t msg = {};
  msg.identifier = ID_MOTORCONTROL1;
  msg.extd = 1;
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
// ====================== Lecture potentiomètre ================
// ============================================================
static float readPotNormalized()
{
  rawAdc = analogRead(POT_PIN);

  float norm = (float)(rawAdc - POT_RAW_MIN) / (float)(POT_RAW_MAX - POT_RAW_MIN);
  norm = clampf(norm, 0.0f, 1.0f);

  return norm;
}

static void updateTorqueCommand()
{
  float potNorm = readPotNormalized();

  potNormFiltered += FILTER_ALPHA * (potNorm - potNormFiltered);
  potNormFiltered = clampf(potNormFiltered, 0.0f, 1.0f);

  // En dessous du seuil : 0 torque
  if (potNormFiltered < TORQUE_ENABLE_THRESHOLD) {
    torqueCommandNm = 0.0f;
  } else {
    // Remap pour que :
    // threshold -> 0 Nm
    // 100%      -> TORQUE_MAX_NM
    float effectiveNorm =
        (potNormFiltered - TORQUE_ENABLE_THRESHOLD) / (1.0f - TORQUE_ENABLE_THRESHOLD);
    effectiveNorm = clampf(effectiveNorm, 0.0f, 1.0f);

    torqueCommandNm = effectiveNorm * TORQUE_MAX_NM;
  }
}

// ============================================================
// ===================== Debug périodique ======================
// ============================================================
static void handlePeriodicDebug(uint32_t now)
{
#if DEBUG_SERIAL
  if (now - lastDebugTime >= DEBUG_PERIOD_MS) {
    lastDebugTime = now;

    twai_status_info_t statusInfo;
    twai_get_status_info(&statusInfo);

    DBG_PRINT("raw=");
    DBG_PRINT(rawAdc);
    DBG_PRINT(" | pot=");
    DBG_PRINT(potNormFiltered * 100.0f, 1);
    DBG_PRINT("% | torque=");
    DBG_PRINT(torqueCommandNm, 3);
    DBG_PRINT(" Nm | threshold=");
    DBG_PRINT(TORQUE_ENABLE_THRESHOLD * 100.0f, 1);
    DBG_PRINT("% | TX queued=");
    DBG_PRINT(statusInfo.msgs_to_tx);
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
static bool initCan()
{
  twai_general_config_t g_config =
      TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);

  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_250KBITS();
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

  DBG_PRINTLN("[INIT] TWAI demarre a 250 kbit/s");
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
  DBG_PRINTLN("===== Boot ESP32 POT -> CAN TORQUE =====");

  analogReadResolution(12);

  // Optionnel selon la carte / core :
  // analogSetPinAttenuation(POT_PIN, ADC_11db);

  if (!initCan()) {
    DBG_PRINTLN("[FATAL] Init CAN echouee");
    while (true) {
      delay(1000);
    }
  }
}

// ============================================================
// =========================== Loop ============================
// ============================================================
void loop()
{
  uint32_t now = millis();

  updateTorqueCommand();

  if (now - lastCanTxTime >= CAN_PERIOD_MS) {
    lastCanTxTime = now;
    sendMotorControl1(torqueCommandNm);
  }

  handlePeriodicDebug(now);

  delay(1);
}