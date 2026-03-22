// Receptor (ESP32-S3) com controle remoto via ESP-NOW:
// - Canal A (L/R): segue o Transmissor A
// - Canal B (L/R): segue o Transmissor B

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <math.h>

static const int CHANNEL_A_LEFT_PIN = 4;
static const int CHANNEL_A_RIGHT_PIN = 5;
static const int CHANNEL_B_LEFT_PIN = 6;
static const int CHANNEL_B_RIGHT_PIN = 7;
static const uint8_t ESPNOW_WIFI_CHANNEL = 1;
static const int CHANNEL_A_LED_PIN = 47;
static const int CHANNEL_B_LED_PIN = 48;

static const float BASE_SIGNAL_FREQUENCY_HZ = 1000.0f;
static const float REFERENCE_RPM = 33.3f;
static const float REFERENCE_DPS = REFERENCE_RPM * 6.0f;
static const float MIN_SIGNAL_FREQUENCY_HZ = 0.0f;
static const float MAX_SIGNAL_FREQUENCY_HZ = 3000.0f;
static const unsigned long SIGNAL_TIMEOUT_MS = 100;
static const float START_THRESHOLD_DPS = 6.0f;
static const float STOP_THRESHOLD_DPS = 2.5f;
static const unsigned long DEBUG_INTERVAL_MS = 250;
static const unsigned long STATUS_LED_BLINK_MS = 250;
static const float FREQUENCY_SMOOTHING = 0.18f;

static const uint8_t TRANSMITTER_A_ID = 1;
static const uint8_t TRANSMITTER_B_ID = 2;

struct MotionMessage {
  int32_t phaseTicks;
  int16_t speedDeltaDpsTenths;
  int16_t absoluteGyroZDpsTenths;
  uint8_t motionActive;
  uint8_t transmitterId;
  uint32_t sequence;
};

struct ChannelState {
  volatile int16_t speedDeltaDpsTenths;
  volatile int16_t absoluteGyroZDpsTenths;
  volatile uint8_t motionActive;
  volatile uint32_t sequence;
  volatile unsigned long lastPacketMs;
  unsigned long lastMotionSeenMs;
  bool enabled;
  bool reverse;
  float phase;
  float currentFrequencyHz;
  float targetFrequencyHz;
};

ChannelState channelAState = {0, 0, 0, 0, 0, 0, false, false, 0.0f, 0.0f, 0.0f};
ChannelState channelBState = {0, 0, 0, 0, 0, 0, false, false, 0.0f, 0.0f, 0.0f};

unsigned long lastDebugMs = 0;
unsigned long lastPhaseMicros = 0;
unsigned long lastLedBlinkMs = 0;
bool blinkState = false;

void writeStereoFromPhase(int leftPin, int rightPin, float phaseCycles,
                          bool reverse) {
  float wrapped = phaseCycles - floorf(phaseCycles);
  if (wrapped < 0.0f) {
    wrapped += 1.0f;
  }

  const float quarterCycle = 0.25f;
  const float rightPhase =
      reverse ? wrapped - quarterCycle : wrapped + quarterCycle;
  float wrappedRight = rightPhase - floorf(rightPhase);
  if (wrappedRight < 0.0f) {
    wrappedRight += 1.0f;
  }

  const bool leftLevel = wrapped < 0.5f;
  const bool rightLevel = wrappedRight < 0.5f;

  digitalWrite(leftPin, leftLevel ? HIGH : LOW);
  digitalWrite(rightPin, rightLevel ? HIGH : LOW);
}

void updateDirection(ChannelState &state, bool reverse, const char *label) {
  if (state.reverse == reverse) {
    return;
  }

  state.reverse = reverse;
  Serial.print(label);
  Serial.println(reverse ? " reverso" : " frente");
}

void updateEnabled(ChannelState &state, bool enabled, int leftPin, int rightPin,
                   const char *label) {
  if (state.enabled == enabled) {
    return;
  }

  state.enabled = enabled;
  if (!enabled) {
    digitalWrite(leftPin, LOW);
    digitalWrite(rightPin, LOW);
    state.currentFrequencyHz = 0.0f;
    state.targetFrequencyHz = 0.0f;
  } else {
    state.phase = 0.0f;
    state.currentFrequencyHz = BASE_SIGNAL_FREQUENCY_HZ;
    state.targetFrequencyHz = BASE_SIGNAL_FREQUENCY_HZ;
    writeStereoFromPhase(leftPin, rightPin, state.phase, state.reverse);
  }

  Serial.print(label);
  Serial.println(enabled ? " ligado" : " parado");
}

void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData,
                int len) {
  if (len != sizeof(MotionMessage)) {
    return;
  }

  MotionMessage message;
  memcpy(&message, incomingData, sizeof(message));

  ChannelState *target = nullptr;
  if (message.transmitterId == TRANSMITTER_A_ID) {
    target = &channelAState;
  } else if (message.transmitterId == TRANSMITTER_B_ID) {
    target = &channelBState;
  } else {
    return;
  }

  target->speedDeltaDpsTenths = message.speedDeltaDpsTenths;
  target->absoluteGyroZDpsTenths = message.absoluteGyroZDpsTenths;
  target->motionActive = message.motionActive;
  target->sequence = message.sequence;
  target->lastPacketMs = millis();
}

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_channel(ESPNOW_WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);

  pinMode(CHANNEL_A_LEFT_PIN, OUTPUT);
  pinMode(CHANNEL_A_RIGHT_PIN, OUTPUT);
  pinMode(CHANNEL_B_LEFT_PIN, OUTPUT);
  pinMode(CHANNEL_B_RIGHT_PIN, OUTPUT);
  pinMode(CHANNEL_A_LED_PIN, OUTPUT);
  pinMode(CHANNEL_B_LED_PIN, OUTPUT);
  digitalWrite(CHANNEL_A_LEFT_PIN, LOW);
  digitalWrite(CHANNEL_A_RIGHT_PIN, LOW);
  digitalWrite(CHANNEL_B_LEFT_PIN, LOW);
  digitalWrite(CHANNEL_B_RIGHT_PIN, LOW);
  digitalWrite(CHANNEL_A_LED_PIN, LOW);
  digitalWrite(CHANNEL_B_LED_PIN, LOW);

  if (esp_now_init() != ESP_OK) {
    Serial.println("Erro ao iniciar ESP-NOW");
    return;
  }

  esp_now_register_recv_cb(onDataRecv);

  Serial.print("Canal A L -> GPIO");
  Serial.println(CHANNEL_A_LEFT_PIN);
  Serial.print("Canal A R -> GPIO");
  Serial.println(CHANNEL_A_RIGHT_PIN);
  Serial.print("Canal B L -> GPIO");
  Serial.println(CHANNEL_B_LEFT_PIN);
  Serial.print("Canal B R -> GPIO");
  Serial.println(CHANNEL_B_RIGHT_PIN);
  Serial.print("MAC Receptor: ");
  Serial.println(WiFi.macAddress());
  Serial.print("Canal ESP-NOW: ");
  Serial.println(ESPNOW_WIFI_CHANNEL);
  Serial.println(
      "Transmissor A controla Canal A e Transmissor B controla Canal B");
  Serial.print("LED Canal A -> GPIO");
  Serial.println(CHANNEL_A_LED_PIN);
  Serial.print("LED Canal B -> GPIO");
  Serial.println(CHANNEL_B_LED_PIN);
  lastPhaseMicros = micros();
}

void processChannel(ChannelState &state, int leftPin, int rightPin,
                    const char *label, unsigned long nowMs, float dtSeconds) {
  const float speedDeltaDps = state.speedDeltaDpsTenths / 10.0f;
  const float absoluteGyroZDps = state.absoluteGyroZDpsTenths / 10.0f;
  const bool haveRecentPacket =
      (nowMs - state.lastPacketMs) <= SIGNAL_TIMEOUT_MS;

  if (haveRecentPacket && state.motionActive &&
      fabsf(absoluteGyroZDps) >= STOP_THRESHOLD_DPS) {
    state.lastMotionSeenMs = nowMs;
  }

  bool shouldEnable = state.enabled;
  if (!state.enabled) {
    shouldEnable = haveRecentPacket && state.motionActive &&
                   fabsf(absoluteGyroZDps) >= START_THRESHOLD_DPS;
  } else {
    shouldEnable = haveRecentPacket && state.motionActive &&
                   ((nowMs - state.lastMotionSeenMs) <= SIGNAL_TIMEOUT_MS);
  }

  updateDirection(state, absoluteGyroZDps < 0.0f, label);

  if (shouldEnable && !state.enabled) {
    updateEnabled(state, true, leftPin, rightPin, label);
  }

  if (shouldEnable) {
    const float relativeFactor = fabsf(absoluteGyroZDps) / REFERENCE_DPS;
    state.targetFrequencyHz = BASE_SIGNAL_FREQUENCY_HZ * relativeFactor;
    if (state.targetFrequencyHz > MAX_SIGNAL_FREQUENCY_HZ) {
      state.targetFrequencyHz = MAX_SIGNAL_FREQUENCY_HZ;
    }
    if (fabsf(absoluteGyroZDps) < STOP_THRESHOLD_DPS) {
      state.targetFrequencyHz = 0.0f;
    }
  } else {
    state.targetFrequencyHz = 0.0f;
  }

  state.currentFrequencyHz +=
      (state.targetFrequencyHz - state.currentFrequencyHz) *
      FREQUENCY_SMOOTHING;

  if (state.enabled && state.currentFrequencyHz <= 0.5f && !shouldEnable) {
    state.currentFrequencyHz = 0.0f;
    updateEnabled(state, false, leftPin, rightPin, label);
  }

  if (state.enabled && state.currentFrequencyHz > MIN_SIGNAL_FREQUENCY_HZ) {
    state.phase += state.currentFrequencyHz * dtSeconds;
    writeStereoFromPhase(leftPin, rightPin, state.phase, state.reverse);
  }
}

void loop() {
  const unsigned long nowMs = millis();
  const unsigned long nowMicros = micros();
  const float dtSeconds = (nowMicros - lastPhaseMicros) / 1000000.0f;
  lastPhaseMicros = nowMicros;

  processChannel(channelAState, CHANNEL_A_LEFT_PIN, CHANNEL_A_RIGHT_PIN,
                 "Canal A", nowMs, dtSeconds);
  processChannel(channelBState, CHANNEL_B_LEFT_PIN, CHANNEL_B_RIGHT_PIN,
                 "Canal B", nowMs, dtSeconds);

  const bool channelAPaired =
      (nowMs - channelAState.lastPacketMs) <= SIGNAL_TIMEOUT_MS;
  const bool channelBPaired =
      (nowMs - channelBState.lastPacketMs) <= SIGNAL_TIMEOUT_MS;

  if ((nowMs - lastLedBlinkMs) >= STATUS_LED_BLINK_MS) {
    lastLedBlinkMs = nowMs;
    blinkState = !blinkState;
  }

  digitalWrite(CHANNEL_A_LED_PIN,
               channelAPaired ? HIGH : (blinkState ? HIGH : LOW));
  digitalWrite(CHANNEL_B_LED_PIN,
               channelBPaired ? HIGH : (blinkState ? HIGH : LOW));

  if (nowMs - lastDebugMs >= DEBUG_INTERVAL_MS) {
    lastDebugMs = nowMs;

    Serial.print("A seq: ");
    Serial.print(channelAState.sequence);
    Serial.print(" mov: ");
    Serial.print(channelAState.motionActive ? "1" : "0");
    Serial.print(" abs: ");
    Serial.print(channelAState.absoluteGyroZDpsTenths / 10.0f);
    Serial.print(" | B seq: ");
    Serial.print(channelBState.sequence);
    Serial.print(" mov: ");
    Serial.print(channelBState.motionActive ? "1" : "0");
    Serial.print(" abs: ");
    Serial.print(channelBState.absoluteGyroZDpsTenths / 10.0f);
    Serial.print(" | LED A: ");
    Serial.print(channelAPaired ? "fixo" : "piscando");
    Serial.print(" | LED B: ");
    Serial.println(channelBPaired ? "fixo" : "piscando");
  }
}
