/********************************************************************
 *  ESP8266 RF-TRANSMITTER + BUTTON (no Wi-Fi / no Telegram)
 *  ---------------------------------------------------------------
 *  Board:  NodeMCU 1.0 (ESP-12E module)
 *  Wiring: Button between D7 and GND (uses INPUT_PULLUP)
 ********************************************************************/
#include <Arduino.h>
#include <pgmspace.h>

// ---------------- Pin assignments ----------------
const uint8_t DATA_PIN    = D1;  // RF data
const uint8_t ENABLE_PIN  = D2;  // RF power enable
const uint8_t BUTTON_PIN  = D7;  // Button to GND

// ---------------- Pulse tables ----------------

const int16_t vibSignal[] PROGMEM = {1345,-804,235,-786,193,-798,747,-276,727,-286,709,-302,711,-288,219,-798,193,-794,227,-792,223,-792,709,-312,693,-284,747,-264,737,-258,733,-288,731,-286,213,-792,225,-792,225,-790,193,-790,225,-790,227,-790,739,-278,217,-794,207,-800,195,-800,225,-792,709,-312,217,-778,721,-294,229,-778,195,-798,745,-274,217,-798,211,-784,725,-308,217,-768,219,-800,209,-792,739,-258,213,-828,193,-796,225};
const int16_t sonSignal[] PROGMEM = {1363, -784, 227, -806, 193, -796, 711, -310, 691, -318, 695, -296, 737, -258, 215, -798, 227, -794, 195, -822, 195, -790, 739, -278, 729, -282, 727, -292, 725, -260, 737, -308, 689, -316, 215, -792, 205, -796, 195, -802, 225, -794, 195, -822, 193, -792, 739, -278, 733, -282, 215, -778, 229, -774, 227, -794, 195, -822, 193, -794, 235, -770, 219, -800, 207, -824, 205, -794, 723, -310, 689, -282, 729, -300, 709, -260, 737, -310, 689, -314, 215, -792, 205, -796, 195, -798, 225, -1026, 1363, -788, 229, -808, 195, -798, 711, -312, 691, -320, 711, -266, 741, -258, 215, -796, 227, -794, 225, -792, 195, -824, 703, -284, 731, -278, 729, -284, 715, -292, 733, -290, 705, -294, 213, -794, 193, -824, 193, -792, 227, -790, 225, -794, 193, -796, 717, -316, 715, -258, 225, -810, 195, -800, 237, -790, 193, -826, 193, -794, 223, -798, 189, -798, 245, -786, 205, -794, 721, -310, 689, -284, 731, -302, 709, -258, 737, -292, 731, -272, 215, -810, 211, -792, 199, -814, 195, -1688};
const int16_t nivel1Signal[] PROGMEM = {1333, -822, 199, -814, 193, -796, 711, -310, 723, -286, 697, -296, 739, -258, 215, -826, 193, -796, 225, -792, 195, -822, 707, -312, 691, -320, 711, -262, 739, -258, 737, -292, 731, -276, 215, -798, 213, -782, 231, -806, 193, -796, 225, -792, 195, -820, 195, -792, 743, -278, 215, -784, 245, -772, 229, -770, 227, -794, 223, -794, 203, -800, 205, -794, 713, -292, 221, -806, 715, -310, 689, -282, 739, -276, 719, -290, 713, -292, 213, -794, 747, -276, 215, -796, 211, -782, 229, -1172, 1341, -786, 245, -774, 227, -804, 715, -274, 725, -284, 729, -298, 709, -290, 183, -832, 193, -796, 225, -792, 195, -820, 709, -314, 691, -320, 677, -296, 737, -258, 737, -294, 727, -288, 211, -790, 225, -792, 193, -790, 227, -790, 227, -792, 225, -792, 193, -792, 737, -284, 217, -794, 201, -816, 193, -798, 225, -792, 195, -822, 195, -790, 225, -792, 741, -280, 215, -786, 711, -290, 715, -292, 731, -310, 689, -318, 699, -312, 213, -772, 723, -310, 215, -764, 211, -786, 231, -1172};

// ------------- RF helper -------------------------
template<size_t N>
void sendSignal(const int16_t (&signal)[N], uint8_t repeats = 10, const char *name = "") {
  Serial.printf(">>> Sending signal: %s (%u pulses x %u repeats)\n", name, N, repeats);
  digitalWrite(ENABLE_PIN, HIGH);
  unsigned long start = micros();
  for (uint8_t r = 0; r < repeats; r++) {
    for (size_t i = 0; i < N; i++) {
      int16_t v = pgm_read_word_near(signal + i);
      digitalWrite(DATA_PIN, (v > 0) ? HIGH : LOW);
      delayMicroseconds(abs(v));
    }
  }
  digitalWrite(DATA_PIN, LOW);
  digitalWrite(ENABLE_PIN, LOW);
  unsigned long elapsed = micros() - start;
  Serial.printf("<<< Finished signal: %s (%.2f ms)\n", name, elapsed / 1000.0);
}

// ------------- Button handling -------------------
volatile uint8_t pressCount = 0;
const unsigned long DEBOUNCE_MS = 35;

bool lastStableState = HIGH;
bool lastReadState   = HIGH;
unsigned long lastChangeMs = 0;

bool checkButtonPressedOnce() {
  bool reading = digitalRead(BUTTON_PIN);
  if (reading != lastReadState) {
    lastChangeMs = millis();
    lastReadState = reading;
    Serial.printf("[DBG] Button raw change detected: %s at %lu ms\n", reading ? "HIGH" : "LOW", millis());
  }

  if ((millis() - lastChangeMs) > DEBOUNCE_MS) {
    if (lastStableState != reading) {
      lastStableState = reading;
      Serial.printf("[DBG] Stable state changed to: %s at %lu ms\n", reading ? "HIGH" : "LOW", millis());
      if (lastStableState == LOW) {
        Serial.println("[DBG] -> Button press detected (FALLING edge)");
        return true;
      }
    }
  }
  return false;
}

// ------------- SETUP -----------------------------
void setup() {
  pinMode(DATA_PIN, OUTPUT);
  pinMode(ENABLE_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  digitalWrite(DATA_PIN, LOW);
  digitalWrite(ENABLE_PIN, LOW);

  Serial.begin(115200);
  delay(200);
  Serial.println();
  Serial.println("=========================================");
  Serial.println(" ESP8266 RF-Transmitter Button Controller");
  Serial.println("=========================================");
  Serial.printf("DATA_PIN=%u, ENABLE_PIN=%u, BUTTON_PIN=%u\n", DATA_PIN, ENABLE_PIN, BUTTON_PIN);
  Serial.println("Setup complete. Waiting for button...");
}

// ------------- LOOP ------------------------------
void loop() {
  if (checkButtonPressedOnce()) {
    pressCount++;
    Serial.printf("[LOG] Button press #%u at %lu ms\n", pressCount, millis());

    if (pressCount < 7) {
      Serial.println("[ACT] Sending snd + vib combo...");
      sendSignal(sonSignal, 10, "snd");
      sendSignal(vibSignal, 10, "vib");
    } else {
      Serial.println("[ACT] 7th press reached! Sending snd + level1...");
      sendSignal(sonSignal, 10, "snd");
      sendSignal(nivel1Signal, 10, "level1");
      pressCount = 0;
      Serial.println("[ACT] Counter reset to 0");
    }

    Serial.println("[LOG] Action complete, waiting for next press...\n");
  }

  // Optional: small delay to keep serial readable
  delay(10);
}
