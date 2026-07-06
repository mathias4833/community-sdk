#include "InputManager.h"

// Custom ESP32-C6 reader ADC ladders. Thresholds are midpoints between the
// expected 12-bit ADC readings for each resistor ladder state.
// left: none ~= 4095, button 1 ~= 2048, button 2 ~= 1024, button 3 ~= 0.
// right: none ~= 4095, button 1 ~= 2048, button 2 ~= 0.
const int InputManager::ADC_EXPECTED_1[] = {2048, 1024, 0};
const int InputManager::ADC_TOLERANCE_1[] = {600, 450, 350};
const int InputManager::ADC_EXPECTED_2[] = {2048, 0};
const int InputManager::ADC_TOLERANCE_2[] = {600, 350};
const char* InputManager::BUTTON_NAMES[] = {"Back", "Confirm", "Left", "Right", "Up", "Down", "Power"};

namespace {
constexpr uint8_t NON_POWER_BUTTON_MASK = (1 << InputManager::BTN_BACK) | (1 << InputManager::BTN_CONFIRM) |
                                          (1 << InputManager::BTN_LEFT) | (1 << InputManager::BTN_RIGHT) |
                                          (1 << InputManager::BTN_UP) | (1 << InputManager::BTN_DOWN);
constexpr uint8_t POWER_BUTTON_MASK = (1 << InputManager::BTN_POWER);

uint8_t countBits(uint8_t value) {
  uint8_t count = 0;
  while (value != 0) {
    count += value & 1;
    value >>= 1;
  }
  return count;
}
}  // namespace

InputManager::InputManager()
    : currentState(0),
      lastState(0),
      pressedEvents(0),
      releasedEvents(0),
      lastDebounceTime(0),
      buttonPressStart(0),
      buttonPressFinish(0),
      powerButtonPressStart(0),
      powerButtonPressFinish(0) {}

void InputManager::begin() {
  pinMode(BUTTON_ADC_PIN_1, INPUT);
  pinMode(BUTTON_ADC_PIN_2, INPUT);
  pinMode(POWER_BUTTON_PIN, INPUT_PULLUP);
  analogSetAttenuation(ADC_11db);
}

int InputManager::readAdcMedian(const int adcPin) const {
  int samples[ADC_SAMPLE_COUNT];

  // Discard one conversion after switching channels; ESP ADC readings can
  // otherwise inherit charge from the previous channel and create ghost presses.
  (void)analogRead(adcPin);

  for (uint8_t i = 0; i < ADC_SAMPLE_COUNT; i++) {
    delayMicroseconds(ADC_SAMPLE_DELAY_US);
    samples[i] = analogRead(adcPin);
  }

  for (uint8_t i = 1; i < ADC_SAMPLE_COUNT; i++) {
    const int value = samples[i];
    uint8_t j = i;
    while (j > 0 && samples[j - 1] > value) {
      samples[j] = samples[j - 1];
      j--;
    }
    samples[j] = value;
  }

  return samples[ADC_SAMPLE_COUNT / 2];
}

int InputManager::getButtonFromADC(const int adcValue, const int expectedValues[], const int tolerances[],
                                   const int numButtons) {
  for (int i = 0; i < numButtons; i++) {
    const int delta = adcValue >= expectedValues[i] ? adcValue - expectedValues[i] : expectedValues[i] - adcValue;
    if (delta <= tolerances[i]) {
      return i;
    }
  }

  return -1;
}

uint8_t InputManager::sanitizeState(const uint8_t rawState) const {
  const uint8_t powerState = rawState & POWER_BUTTON_MASK;
  const uint8_t rawButtons = rawState & NON_POWER_BUTTON_MASK;
  const uint8_t heldButtons = currentState & NON_POWER_BUTTON_MASK;

  if (rawButtons == 0) {
    return powerState;
  }

  if (countBits(heldButtons) == 1) {
    return heldButtons | powerState;
  }

  if (countBits(rawButtons) != 1) {
    return powerState;
  }

  return rawButtons | powerState;
}

uint8_t InputManager::getState() {
  uint8_t rawState = 0;

  // Read left ADC ladder
  const int adcValue1 = readAdcMedian(BUTTON_ADC_PIN_1);
  const int button1 = getButtonFromADC(adcValue1, ADC_EXPECTED_1, ADC_TOLERANCE_1, NUM_BUTTONS_1);
  if (button1 >= 0) {
    rawState |= (1 << button1);
  }

  // Read right ADC ladder
  const int adcValue2 = readAdcMedian(BUTTON_ADC_PIN_2);
  const int button2 = getButtonFromADC(adcValue2, ADC_EXPECTED_2, ADC_TOLERANCE_2, NUM_BUTTONS_2);
  if (button2 >= 0) {
    rawState |= (1 << (button2 + 4));
  }

  // Read power button (digital, active LOW)
  if (digitalRead(POWER_BUTTON_PIN) == LOW) {
    rawState |= POWER_BUTTON_MASK;
  }

  return sanitizeState(rawState);
}

void InputManager::update() {
  const unsigned long currentTime = millis();
  const uint8_t state = getState();

  // Always clear events first
  pressedEvents = 0;
  releasedEvents = 0;

  // Debounce
  if (state != lastState) {
    lastDebounceTime = currentTime;
    lastState = state;
  }

  if ((currentTime - lastDebounceTime) > DEBOUNCE_DELAY) {
    if (state != currentState) {
      // Calculate pressed and released events
      pressedEvents = state & ~currentState;
      releasedEvents = currentState & ~state;

      // If pressing buttons and wasn't before, start recording time
      if (pressedEvents > 0 && currentState == 0) {
        buttonPressStart = currentTime;
      }

      // If releasing a button and no other buttons being pressed, record finish time
      if (releasedEvents > 0 && state == 0) {
        buttonPressFinish = currentTime;
      }

      // Track power button press time separately
      if (pressedEvents & (1 << BTN_POWER)) {
        powerButtonPressStart = currentTime;
      }

      // Track power button release
      if (releasedEvents & (1 << BTN_POWER)) {
        powerButtonPressFinish = currentTime;
      }

      currentState = state;
    }
  }
}

bool InputManager::isPressed(const uint8_t buttonIndex) const {
  return currentState & (1 << buttonIndex);
}

bool InputManager::wasPressed(const uint8_t buttonIndex) const {
  return pressedEvents & (1 << buttonIndex);
}

bool InputManager::wasAnyPressed() const {
  return pressedEvents > 0;
}

bool InputManager::wasReleased(const uint8_t buttonIndex) const {
  return releasedEvents & (1 << buttonIndex);
}

bool InputManager::wasAnyReleased() const {
  return releasedEvents > 0;
}

unsigned long InputManager::getHeldTime() const {
  // Still hold a button
  if (currentState > 0) {
    return millis() - buttonPressStart;
  }

  return buttonPressFinish - buttonPressStart;
}

unsigned long InputManager::getPowerButtonHeldTime() const {
  // Power button is currently pressed
  if (isPressed(BTN_POWER)) {
    return millis() - powerButtonPressStart;
  }

  // Power button was released
  return powerButtonPressFinish - powerButtonPressStart;
}

const char* InputManager::getButtonName(const uint8_t buttonIndex) {
  if (buttonIndex <= BTN_POWER) {
    return BUTTON_NAMES[buttonIndex];
  }
  return "Unknown";
}

bool InputManager::isPowerButtonPressed() const {
  return isPressed(BTN_POWER);
}
