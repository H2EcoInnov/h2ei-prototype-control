#include <Arduino.h>
#include <FlexCAN_T4.h>

// ====== CAN Teensy 4.1 ======
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> Can3;

// ====== LED ======
#define LED_PIN 33

// ====== IDs ======
static const uint32_t ID_MOTORCONTROL1  = 0x18FF3330; // extended
static const uint32_t ID_PC_TRIGGER_STD = 0x123;      // standard

// ====== Timing ======
static const uint32_t PERIOD_MS = 10;     // 100 Hz
static const uint32_t RAMP_MS   = 5000;   // rampe fixe 5 s

// ====== Torque encoding (comme ton Arduino) ======
static const float   TORQUE_RES    = 0.015f;
static const int32_t TORQUE_OFFSET = -32127;

// ====== Garde-fous ======
static const float   TORQUE_ABS_LIMIT = 8.0f;     // sécurité hard (à ajuster)
static const uint32_t HOLD_MS_LIMIT   = 30000;    // max maintien 30 s (à ajuster)

// ====== State machine ======
enum class Mode : uint8_t { IDLE, RAMP, HOLD };
static Mode mode = Mode::IDLE;

static uint32_t tStart = 0;
static uint32_t lastTx = 0;

static float torqueTarget = 0.0f;     // torque max demandé
static uint32_t holdMs = 0;           // durée maintien après rampe

// ====== Packing identique Arduino (LSB-first) ======
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

static float clampf(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

static void sendMotorControl1(float torqueAsked)
{
  uint8_t command[8] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

  if (torqueAsked > 0.0f) {
    writeMessage(command, 16, 16, torqueAsked, TORQUE_RES, TORQUE_OFFSET);
  }

  CAN_message_t msg;
  msg.id = ID_MOTORCONTROL1;
  msg.len = 8;
  msg.flags.extended = 1;
  memcpy(msg.buf, command, 8);

  Can3.write(msg);
}

// Lit uint16 little-endian depuis rx.buf
static uint16_t u16le(const uint8_t *b) {
  return (uint16_t)b[0] | ((uint16_t)b[1] << 8);
}

void setup()
{
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  Can3.begin();
  Can3.setBaudRate(250000); // moteur en 250k

  mode = Mode::IDLE;
}

void loop()
{
  const uint32_t now = millis();

  // ---- Réception du trigger paramétré ----
  CAN_message_t rx;
  while (Can3.read(rx)) {
    if (!rx.flags.extended && rx.id == ID_PC_TRIGGER_STD && rx.len >= 4) {
      // payload PC:
      // [0..1] torque_milli (ex 2500 => 2.500)
      // [2..3] hold_ms
      uint16_t torqueMilli = u16le(&rx.buf[0]);
      uint16_t holdMsIn    = u16le(&rx.buf[2]);

      torqueTarget = clampf((float)torqueMilli / 1000.0f, 0.0f, TORQUE_ABS_LIMIT);
      holdMs       = min<uint32_t>(holdMsIn, HOLD_MS_LIMIT);

      // start sequence
      tStart = now;
      lastTx = 0; // force envoi immédiat
      mode   = Mode::RAMP;

      digitalWrite(LED_PIN, HIGH);
    }
  }

  // ---- Émission selon mode ----
  if (mode == Mode::IDLE) return;

  // Envoi périodique à 10 ms
  if (now - lastTx < PERIOD_MS) return;
  lastTx = now;

  if (mode == Mode::RAMP) {
    uint32_t elapsed = now - tStart;
    if (elapsed > RAMP_MS) elapsed = RAMP_MS;

    float alpha = (float)elapsed / (float)RAMP_MS;  // 0..1
    float torqueCmd = alpha * torqueTarget;

    sendMotorControl1(torqueCmd);

    if ((now - tStart) >= RAMP_MS) {
      // passe en HOLD
      mode = Mode::HOLD;
      tStart = now; // tStart devient le début du maintien
    }
  }
  else if (mode == Mode::HOLD) {
    sendMotorControl1(torqueTarget);

    if ((now - tStart) >= holdMs) {
      // stop + relâche
      sendMotorControl1(0.0f);
      mode = Mode::IDLE;
      digitalWrite(LED_PIN, LOW);
    }
  }
}