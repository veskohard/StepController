
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <math.h>

// =====================================================
// 1. ОСНОВНИ НАСТРОЙКИ
// =====================================================

#define MOTOR_STEPS_PER_REV 200

#define MAX_RPM 300.0f
#define MIN_RPM 30.0f

// Ускорение: 60 RPM за секунда
#define ACCELERATION_RPM 60.0f

// Обновяване на ускорението
#define RAMP_INTERVAL_MS 100

// =====================================================
// 2. РЕЖИМИ
// =====================================================

enum ControlMode {
  MODE_MODBUS = 1,
  MODE_STEP_DIR = 2,
  MODE_MANUAL = 3
};

ControlMode currentMode = MODE_MODBUS;

volatile bool stepDirEnabled = false;

// =====================================================
// 3. ПИНОВЕ L298N
// =====================================================

const int IN1 = PA0;
const int IN2 = PA1;
const int IN3 = PA2;
const int IN4 = PA3;

// =====================================================
// 4. STEP / DIR
// =====================================================

const int STEP_PIN = PB3;
const int DIR_PIN = PB4;

volatile uint32_t pendingForwardSteps = 0;
volatile uint32_t pendingBackwardSteps = 0;

// =====================================================
// 5. ЕНКОДЕР
// =====================================================

const int ENC_CLK = PB12;
const int ENC_DT  = PB13;
const int ENC_SW  = PB14;

int lastClkState = HIGH;

// =====================================================
// 6. ОБЩИ ПАРАМЕТРИ НА МОТОРА
// =====================================================

float motorRPM = 0.0f;

unsigned long lastRampTime = 0;
unsigned long lastMotorStepTime = 0;

bool motorEnergized = false;

// =====================================================
// 7. MANUAL
// =====================================================

bool manualRunRequest = false;
bool manualDirection = true;

bool manualDirectionPending = false;
bool pendingDirection = true;

// Начална зададена скорост
float manualTargetRPM = 60.0f;

// Интервал между стъпките
unsigned long manualStepDelay = 5000;

// Регулиране през 50 микросекунди
const unsigned long SPEED_STEP_US = 50;

const unsigned long MIN_DELAY_US =
  60000000UL / (MOTOR_STEPS_PER_REV * 300UL);

const unsigned long MAX_DELAY_US =
  60000000UL / (MOTOR_STEPS_PER_REV * 30UL);

// =====================================================
// 8. БУТОН
// =====================================================

bool buttonRawState = HIGH;
bool buttonStableState = HIGH;

unsigned long buttonDebounceTime = 0;
unsigned long buttonPressTime = 0;

const unsigned long DEBOUNCE_MS = 30;

// =====================================================
// 9. MODBUS / RS485
// =====================================================

#define RS485_SERIAL Serial

const int MAX485_DE_RE_PIN = PB1;

uint16_t holdingRegisters[16] = {
  0,       // 0: Steps
  1,       // 1: Direction
  2000,    // 2: Delay in us
  0,       // 3: Start/Stop
  0, 0, 0, 0, 0, 0,
  1,       // 10: Mode
  0, 0, 0, 0, 0
};

bool mbIsRunning = false;
bool mbStopRequested = false;

long mbStepsRemaining = 0;

bool mbDirection = true;

float mbTargetRPM = 150.0f;

// =====================================================
// 10. FULL STEP TABLE
// =====================================================

const int stepTable[4][4] = {
  {HIGH, LOW,  HIGH, LOW},
  {LOW,  HIGH, HIGH, LOW},
  {LOW,  HIGH, LOW,  HIGH},
  {HIGH, LOW,  LOW,  HIGH}
};

int currentStep = 0;

// =====================================================
// 11. OLED
// =====================================================

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

Adafruit_SSD1306 display(
  SCREEN_WIDTH,
  SCREEN_HEIGHT,
  &Wire,
  -1
);

unsigned long lastDisplayUpdate = 0;

// =====================================================
// 12. ИЗПЪЛНЕНИЕ НА ЕДНА СТЪПКА
// =====================================================

void executeStep(bool direction) {

  if (direction) {
    currentStep = (currentStep + 1) % 4;
  }
  else {
    currentStep = (currentStep - 1 + 4) % 4;
  }

  digitalWrite(IN1, stepTable[currentStep][0]);
  digitalWrite(IN2, stepTable[currentStep][1]);
  digitalWrite(IN3, stepTable[currentStep][2]);
  digitalWrite(IN4, stepTable[currentStep][3]);

  motorEnergized = true;
}

// =====================================================
// 13. ОСВОБОЖДАВАНЕ НА МОТОРА
// =====================================================

void releaseMotor() {

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);

  motorEnergized = false;
}

// =====================================================
// 14. RPM КЪМ МИКРОСЕКУНДИ
// =====================================================

unsigned long rpmToDelay(float rpm) {

  if (rpm <= 0.0f) return MAX_DELAY_US;

  float delayValue =
    60000000.0f /
    (MOTOR_STEPS_PER_REV * rpm);

  return (unsigned long)delayValue;
}

// =====================================================
// 15. МИКРОСЕКУНДИ КЪМ RPM
// =====================================================

float delayToRPM(unsigned long delayValue) {

  if (delayValue == 0) return 0;

  return 60000000.0f /
         (MOTOR_STEPS_PER_REV * delayValue);
}

// =====================================================
// 16. ПЛАВНО УСКОРЯВАНЕ И ЗАБАВЯНЕ
// =====================================================

void updateMotorRamp(float targetRPM) {

  unsigned long now = millis();

  if (now - lastRampTime < RAMP_INTERVAL_MS) {
    return;
  }

  lastRampTime = now;

  float change =
    ACCELERATION_RPM *
    (RAMP_INTERVAL_MS / 1000.0f);

  if (motorRPM < targetRPM) {

    motorRPM += change;

    if (motorRPM > targetRPM) {
      motorRPM = targetRPM;
    }
  }

  else if (motorRPM > targetRPM) {

    motorRPM -= change;

    if (motorRPM < targetRPM) {
      motorRPM = targetRPM;
    }
  }

  if (motorRPM < 0.5f) {
    motorRPM = 0.0f;
  }
}

// =====================================================
// 17. ГЕНЕРИРАНЕ НА СТЪПКИ СПОРЕД RPM
// =====================================================

bool generateMotorStep(bool direction) {

  if (motorRPM <= 0.0f) {
    return false;
  }

  unsigned long interval = rpmToDelay(motorRPM);

  unsigned long now = micros();

  if (now - lastMotorStepTime >= interval) {

    lastMotorStepTime = now;

    executeStep(direction);

    return true;
  }

  return false;
}

// =====================================================
// 18. СМЯНА НА РЕЖИМ
// =====================================================

void changeMode(ControlMode newMode) {

  manualRunRequest = false;
  manualDirectionPending = false;

  mbIsRunning = false;
  mbStopRequested = false;
  mbStepsRemaining = 0;

  motorRPM = 0.0f;

  releaseMotor();

  noInterrupts();

  stepDirEnabled = false;

  pendingForwardSteps = 0;
  pendingBackwardSteps = 0;

  currentMode = newMode;

  stepDirEnabled = (newMode == MODE_STEP_DIR);

  interrupts();

  holdingRegisters[10] = (uint16_t)newMode;

  lastMotorStepTime = micros();
  lastRampTime = millis();

  lastClkState = digitalRead(ENC_CLK);
}

// =====================================================
// 19. СЛЕДВАЩ РЕЖИМ
// =====================================================

void nextMode() {

  if (currentMode == MODE_MODBUS) {
    changeMode(MODE_STEP_DIR);
  }

  else if (currentMode == MODE_STEP_DIR) {
    changeMode(MODE_MANUAL);
  }

  else {
    changeMode(MODE_MODBUS);
  }
}

// =====================================================
// 20. STEP / DIR INTERRUPT
// =====================================================

void handleExternalStep() {

  if (!stepDirEnabled) return;

  if (digitalRead(DIR_PIN) == HIGH) {

    if (pendingForwardSteps < 100000UL) {
      pendingForwardSteps++;
    }
  }

  else {

    if (pendingBackwardSteps < 100000UL) {
      pendingBackwardSteps++;
    }
  }
}

// =====================================================
// 21. STEP / DIR ОБРАБОТКА
// =====================================================

void handleStepDir() {

  bool direction = false;
  bool hasStep = false;

  noInterrupts();

  if (pendingForwardSteps > 0) {

    pendingForwardSteps--;
    direction = true;
    hasStep = true;
  }

  else if (pendingBackwardSteps > 0) {

    pendingBackwardSteps--;
    direction = false;
    hasStep = true;
  }

  interrupts();

  if (hasStep) {
    executeStep(direction);
  }
}

// =====================================================
// 22. MANUAL - РЕГУЛИРАНЕ НА СКОРОСТТА
// =====================================================

void handleManualEncoderSpeed() {

  int clk = digitalRead(ENC_CLK);

  if (clk != lastClkState && clk == LOW) {

    bool rotationRight =
      (digitalRead(ENC_DT) != clk);

    if (rotationRight) {

      // Увеличаване на оборотите

      if (manualStepDelay > MIN_DELAY_US) {

        manualStepDelay -= SPEED_STEP_US;

        if (manualStepDelay < MIN_DELAY_US) {
          manualStepDelay = MIN_DELAY_US;
        }
      }
    }

    else {

      // Намаляване на оборотите

      if (manualStepDelay < MAX_DELAY_US) {

        manualStepDelay += SPEED_STEP_US;

        if (manualStepDelay > MAX_DELAY_US) {
          manualStepDelay = MAX_DELAY_US;
        }
      }
    }

    manualTargetRPM =
      delayToRPM(manualStepDelay);
  }

  lastClkState = clk;
}

// =====================================================
// 23. MANUAL - START / STOP / DIRECTION
// =====================================================

void handleManualButtonAction(unsigned long duration) {

  // Кратко натискане

  if (duration < 800) {

    manualRunRequest = !manualRunRequest;

    manualDirectionPending = false;

    if (manualRunRequest) {
      lastMotorStepTime = micros();
    }
  }

  // Средно задържане - смяна на посоката

  else if (duration < 3000) {

    if (manualRunRequest || motorRPM > 0.0f) {

      // Първо спираме плавно
      manualRunRequest = false;

      pendingDirection = !manualDirection;

      manualDirectionPending = true;
    }

    else {

      manualDirection = !manualDirection;
    }
  }

  // Над 3 секунди - смяна на режим

  else {

    nextMode();
  }
}

// =====================================================
// 24. MANUAL - ПЛАВНО ДВИЖЕНИЕ
// =====================================================

void handleManualMotor() {

  float targetRPM = 0.0f;

  if (manualRunRequest) {
    targetRPM = manualTargetRPM;
  }

  updateMotorRamp(targetRPM);

  // Когато достигнем нула при смяна на посоката

  if (motorRPM == 0.0f &&
      manualDirectionPending) {

    manualDirection = pendingDirection;

    manualDirectionPending = false;

    // Автоматично стартиране в новата посока
    manualRunRequest = true;

    lastMotorStepTime = micros();
  }

  // Плавно спиране

  if (motorRPM == 0.0f && !manualRunRequest) {

    releaseMotor();
    return;
  }

  generateMotorStep(manualDirection);
}

// =====================================================
// 25. MODBUS CRC16
// =====================================================

uint16_t calculateCRC(uint8_t *data, uint8_t length) {

  uint16_t crc = 0xFFFF;

  for (uint8_t i = 0; i < length; i++) {

    crc ^= data[i];

    for (uint8_t j = 0; j < 8; j++) {

      if (crc & 1) {
        crc = (crc >> 1) ^ 0xA001;
      }
      else {
        crc >>= 1;
      }
    }
  }

  return crc;
}

// =====================================================
// 26. MODBUS ОТГОВОР
// =====================================================

void sendModbusResponse(uint8_t *data, uint8_t length) {

  uint16_t crc = calculateCRC(data, length);

  data[length] = crc & 0xFF;
  data[length + 1] = crc >> 8;

  digitalWrite(MAX485_DE_RE_PIN, HIGH);

  delayMicroseconds(50);

  RS485_SERIAL.write(data, length + 2);
  RS485_SERIAL.flush();

  delayMicroseconds(50);

  digitalWrite(MAX485_DE_RE_PIN, LOW);
}

// =====================================================
// 27. MODBUS КОМАНДИ
// =====================================================

void pollModbusSlave() {

  if (RS485_SERIAL.available() < 8) return;

  uint8_t buffer[8];

  for (int i = 0; i < 8; i++) {
    buffer[i] = RS485_SERIAL.read();
  }

  if (buffer[0] != 1) return;

  uint16_t receivedCRC =
    buffer[6] | (buffer[7] << 8);

  uint16_t calculatedCRC =
    calculateCRC(buffer, 6);

  if (receivedCRC != calculatedCRC) return;

  uint16_t regAddr =
    (buffer[2] << 8) | buffer[3];

  // WRITE SINGLE REGISTER

  if (buffer[1] == 0x06) {

    uint16_t value =
      (buffer[4] << 8) | buffer[5];

    if (regAddr < 16) {

      holdingRegisters[regAddr] = value;

      sendModbusResponse(buffer, 6);
    }
  }

  // READ HOLDING REGISTERS

  else if (buffer[1] == 0x03) {

    uint16_t quantity =
      (buffer[4] << 8) | buffer[5];

    if (quantity == 0 ||
        quantity > 16 ||
        regAddr + quantity > 16) {
      return;
    }

    uint8_t response[37];

    response[0] = 1;
    response[1] = 0x03;
    response[2] = quantity * 2;

    for (uint16_t i = 0; i < quantity; i++) {

      response[3 + i * 2] =
        holdingRegisters[regAddr + i] >> 8;

      response[4 + i * 2] =
        holdingRegisters[regAddr + i] & 0xFF;
    }

    sendModbusResponse(response, 3 + quantity * 2);
  }
}

// =====================================================
// 28. MODBUS - ПЛАВНО УСКОРЯВАНЕ И СПИРАНЕ
// =====================================================

void handleModbusMotor() {

  // START

  if (!mbIsRunning && holdingRegisters[3] == 1) {

    mbStepsRemaining = holdingRegisters[0];

    mbDirection = holdingRegisters[1] != 0;

    unsigned long requestedDelay =
      holdingRegisters[2];

    if (requestedDelay < MIN_DELAY_US) {
      requestedDelay = MIN_DELAY_US;
    }

    if (requestedDelay > MAX_DELAY_US) {
      requestedDelay = MAX_DELAY_US;
    }

    mbTargetRPM = delayToRPM(requestedDelay);

    mbStopRequested = false;

    if (mbStepsRemaining > 0) {

      mbIsRunning = true;

      lastMotorStepTime = micros();
    }

    holdingRegisters[3] = 0;
  }

  // STOP команда = 2

  if (holdingRegisters[3] == 2) {

    mbStopRequested = true;

    holdingRegisters[3] = 0;
  }

  if (!mbIsRunning) {

    releaseMotor();
    return;
  }

  float desiredRPM = mbTargetRPM;

  // Плавно спиране при STOP

  if (mbStopRequested) {

    desiredRPM = 0.0f;
  }

  // Автоматично забавяне преди края на движението
  // Ускорение 60 RPM/s = 200 стъпки/s^2

  else if (mbStepsRemaining > 0) {

    float accelerationSteps = 200.0f;

    float brakingRPM = sqrtf(
      2.0f * accelerationSteps *
      (float)mbStepsRemaining
    ) * 60.0f / MOTOR_STEPS_PER_REV;

    if (brakingRPM < desiredRPM) {
      desiredRPM = brakingRPM;
    }
  }

  updateMotorRamp(desiredRPM);

  // Когато моторът спре

  if (motorRPM == 0.0f &&
      (mbStopRequested || mbStepsRemaining <= 0)) {

    mbIsRunning = false;
    mbStopRequested = false;

    releaseMotor();

    return;
  }

  // Генериране на стъпки

  if (generateMotorStep(mbDirection)) {

    if (mbStepsRemaining > 0) {
      mbStepsRemaining--;
    }
  }
}

// =====================================================
// 29. ОБРАБОТКА НА ЕНКОДЕРА И БУТОНА
// =====================================================

void handleEncoderButton() {

  bool button = digitalRead(ENC_SW);

  if (button != buttonRawState) {

    buttonRawState = button;

    buttonDebounceTime = millis();
  }

  if (millis() - buttonDebounceTime >= DEBOUNCE_MS) {

    if (button != buttonStableState) {

      buttonStableState = button;

      if (buttonStableState == LOW) {

        buttonPressTime = millis();
      }

      else {

        unsigned long duration =
          millis() - buttonPressTime;

        if (currentMode == MODE_MANUAL) {

          handleManualButtonAction(duration);
        }

        else {

          if (duration < 800 || duration >= 3000) {
            nextMode();
          }
        }
      }
    }
  }
}

// =====================================================
// 30. OLED ДИСПЛЕЙ
// =====================================================

void updateDisplay() {

  display.clearDisplay();

  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(0, 0);
  display.print("STEPPER CONTROLLER");

  display.setCursor(0, 12);
  display.print("Mode: ");

  if (currentMode == MODE_MODBUS) {
    display.print("MODBUS");
  }

  else if (currentMode == MODE_STEP_DIR) {
    display.print("STEP / DIR");
  }

  else {
    display.print("MANUAL");
  }

  display.setCursor(0, 25);

  display.print("RPM: ");

  display.print(motorRPM, 1);

  display.setCursor(0, 37);

  display.print("Target: ");

  if (currentMode == MODE_MANUAL) {
    display.print(manualTargetRPM, 1);
  }

  else if (currentMode == MODE_MODBUS) {
    display.print(mbTargetRPM, 1);
  }

  else {
    display.print("--");
  }

  display.setCursor(0, 49);

  if (currentMode == MODE_MANUAL) {

    display.print("Dir: ");
    display.print(manualDirection ? "RIGHT" : "LEFT");

    display.setCursor(0, 58);

    if (manualRunRequest) {
      display.print("RUNNING");
    }
    else if (motorRPM > 0.0f) {
      display.print("STOPPING...");
    }
    else {
      display.print("STOPPED");
    }
  }

  else if (currentMode == MODE_MODBUS) {

    display.print("Steps: ");
    display.print(mbStepsRemaining);

    display.setCursor(0, 58);

    display.print(mbIsRunning ? "RUNNING" : "STOPPED");
  }

  else {

    display.print("External pulses");

    display.setCursor(0, 58);
    display.print("Waiting...");
  }

  display.display();
}

// =====================================================
// 31. SETUP
// =====================================================

void setup() {

  // L298N

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  releaseMotor();

  // STEP / DIR

  pinMode(STEP_PIN, INPUT_PULLUP);
  pinMode(DIR_PIN, INPUT_PULLUP);

  attachInterrupt(
    digitalPinToInterrupt(STEP_PIN),
    handleExternalStep,
    RISING
  );

  // Енкодер

  pinMode(ENC_CLK, INPUT_PULLUP);
  pinMode(ENC_DT, INPUT_PULLUP);
  pinMode(ENC_SW, INPUT_PULLUP);

  lastClkState = digitalRead(ENC_CLK);

  buttonRawState = digitalRead(ENC_SW);
  buttonStableState = buttonRawState;

  // OLED

  display.begin(
    SSD1306_SWITCHCAPVCC,
    0x3C
  );

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.display();

  // RS485

  pinMode(MAX485_DE_RE_PIN, OUTPUT);

  digitalWrite(MAX485_DE_RE_PIN, LOW);

  RS485_SERIAL.begin(9600, SERIAL_8N1);

  // Начални времена

  lastRampTime = millis();
  lastMotorStepTime = micros();

  // Начална скорост

  manualStepDelay = 5000;
  manualTargetRPM = 60.0f;

  currentMode = MODE_MODBUS;
  stepDirEnabled = false;
}

// =====================================================
// 32. MAIN LOOP
// =====================================================

void loop() {

  // 1. Modbus комуникация

  pollModbusSlave();

  // 2. Синхронизация на режима

  int requestedMode = holdingRegisters[10];

  if (requestedMode >= 1 && requestedMode <= 3) {

    if (requestedMode != currentMode) {

      changeMode((ControlMode)requestedMode);
    }
  }

  // 3. Енкодер

  handleEncoderButton();

  // 4. Управление на мотора

  switch (currentMode) {

    case MODE_MODBUS:

      handleModbusMotor();

      break;

    case MODE_STEP_DIR:

      handleStepDir();

      break;

    case MODE_MANUAL:

      handleManualEncoderSpeed();

      handleManualMotor();

      break;
  }

  // 5. OLED

  if (millis() - lastDisplayUpdate >= 250) {

    lastDisplayUpdate = millis();

    updateDisplay();
  }
}
