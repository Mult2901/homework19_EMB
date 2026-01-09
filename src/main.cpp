#include <Arduino.h>

constexpr uint8_t RELAY_CTRL_PIN = 10;
constexpr uint8_t RELAY_CONTACT_PIN = 11;

constexpr uint8_t MEASUREMENTS_COUNT = 10;
constexpr uint32_t RELAY_OFF_TIME_MS = 1000;

volatile bool contactEvent = false;

uint32_t tRelayOn = 0;
uint32_t measurements[MEASUREMENTS_COUNT];
uint8_t measureIndex = 0;

bool relayOn = false;
uint32_t lastActionTime = 0;

// ISR — тільки прапорець
void IRAM_ATTR relayContactISR()
{
  contactEvent = true;
}

void setup()
{
  Serial.begin(115200);

  pinMode(RELAY_CTRL_PIN, OUTPUT);
  digitalWrite(RELAY_CTRL_PIN, LOW);

  pinMode(RELAY_CONTACT_PIN, INPUT_PULLUP);

  attachInterrupt(
      digitalPinToInterrupt(RELAY_CONTACT_PIN),
      relayContactISR,
      FALLING);

  Serial.println("Розпочато вимірювання часу спрацьовування реле");
}

void loop()
{
  uint32_t now = millis();

  // Увімкнути реле для нового вимірювання
  if (!relayOn && measureIndex < MEASUREMENTS_COUNT &&
      (now - lastActionTime >= RELAY_OFF_TIME_MS))
  {
    relayOn = true;
    digitalWrite(RELAY_CTRL_PIN, HIGH);
    tRelayOn = now;

    Serial.print("Вимірювання ");
    Serial.print(measureIndex + 1);
    Serial.println(": реле ON");

    lastActionTime = now;
  }

  // Обробка фактичного спрацювання контакту
  if (relayOn && contactEvent)
  {
    contactEvent = false;

    uint32_t responseTime = millis() - tRelayOn;
    measurements[measureIndex] = responseTime;

    Serial.print("  Час відгуку: ");
    Serial.print(responseTime);
    Serial.println(" мс");

    measureIndex++;

    // Вимкнути реле
    relayOn = false;
    digitalWrite(RELAY_CTRL_PIN, LOW);
    lastActionTime = millis();
  }

  // Після завершення всіх вимірювань
  if (measureIndex == MEASUREMENTS_COUNT)
  {
    uint32_t sum = 0;

    for (uint8_t i = 0; i < MEASUREMENTS_COUNT; i++)
      sum += measurements[i];

    float average = sum / (float)MEASUREMENTS_COUNT;

    Serial.println("\n--- Результати ---");
    Serial.print("Вимірювання: ");
    Serial.println(MEASUREMENTS_COUNT);
    Serial.print("Середній час відгуку: ");
    Serial.print(average, 2);
    Serial.println(" мс");

    measureIndex++; // блокуємо повтор
  }
}