
#include <Arduino.h>
#include <U8g2lib.h>
#include <SPI.h>
#include <math.h>

// =====================================================
// OLED SSD1306 128x64 - SOFTWARE SPI
// =====================================================

#define OLED_CLK   PA5
#define OLED_MOSI  PA7
#define OLED_CS    PA4
#define OLED_DC    PB5
#define OLED_RES   PB0

U8G2_SSD1306_128X64_NONAME_F_4W_SW_SPI display(
    U8G2_R0,
    OLED_CLK,
    OLED_MOSI,
    OLED_CS,
    OLED_DC,
    OLED_RES
);

// =====================================================
// MOTOR L298N
// =====================================================

#define IN1 PA0
#define IN2 PA1
#define IN3 PA2
#define IN4 PA3

#define MOTOR_STEPS_PER_REV 200

#define MAX_RPM 300.0f
#define MIN_RPM 30.0f
#define ACCELERATION_RPM 60.0f

// Пълна стъпка, две фази включени
const uint8_t stepTable[4][4] = {
    {HIGH, LOW,  HIGH, LOW},
    {LOW,  HIGH, HIGH, LOW},
    {LOW,  HIGH, LOW,  HIGH},
    {HIGH, LOW,  LOW,  HIGH}
};

int stepIndex = 0;
int motorDirection = 1;

float targetRPM = 60.0f;
float currentRPM = 0.0f;

uint32_t stepIntervalUs = 5000;
uint32_t lastStepUs = 0;
uint32_t lastRampMs = 0;

bool motorRunning = false;
bool motorRequested = false;

uint32_t stepsRemaining = 0;

// =====================================================
// STEP / DIR INPUT
// =====================================================

#define STEP_PIN PB3
#define DIR_PIN  PB4

volatile uint32_t pendingStepsForward = 0;
volatile uint32_t pendingStepsBackward = 0;

// =====================================================
// ENCODER
// =====================================================

#define ENC_CLK PB12
#define ENC_DT  PB13
#define ENC_SW  PB14

int lastEncoderCLK = HIGH;

bool lastButtonState = HIGH;
bool buttonPressed = false;

uint32_t buttonPressTime = 0;

const uint32_t SHORT_PRESS_MS = 800;
const uint32_t LONG_PRESS_MS = 3000;

// =====================================================
// RS485 / MODBUS
// =====================================================

#define RS485_DE_RE PB1

#define MODBUS_SLAVE_ID 1
#define MODBUS_BAUD 9600

// USART1: PA9 TX, PA10 RX
#define RS485_SERIAL Serial1

uint16_t holdingRegisters[16] = {0};

uint8_t modbusBuffer[32];
uint8_t modbusLength = 0;

uint32_t lastModbusByteMs = 0;

// =====================================================
// MODES
// =====================================================

enum OperatingMode {
    MODE_MODBUS = 1,
    MODE_STEP_DIR = 2,
    MODE_MANUAL = 3
};

OperatingMode currentMode = MODE_MANUAL;

// =====================================================
// DISPLAY
// =====================================================

uint32_t lastDisplayMs = 0;

const char* getModeName() {
    switch (currentMode) {
        case MODE_MANUAL:  return "MANUAL";
        case MODE_STEP_DIR: return "STEP/DIR";
        case MODE_MODBUS: return "MODBUS";
        default: return "UNKNOWN";
    }
}

// =====================================================
// MOTOR FUNCTIONS
// =====================================================

void releaseMotor() {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
}

void executeStep(int direction) {
    stepIndex += direction;

    if (stepIndex > 3) stepIndex = 0;
    if (stepIndex < 0) stepIndex = 3;

    digitalWrite(IN1, stepTable[stepIndex][0]);
    digitalWrite(IN2, stepTable[stepIndex][1]);
    digitalWrite(IN3, stepTable[stepIndex][2]);
    digitalWrite(IN4, stepTable[stepIndex][3]);
}

uint32_t rpmToDelay(float rpm) {
    if (rpm < MIN_RPM) rpm = MIN_RPM;
    if (rpm > MAX_RPM) rpm = MAX_RPM;

    return (uint32_t)(60000000.0f /
                      (rpm * MOTOR_STEPS_PER_REV));
}

float delayToRPM(uint32_t delayUs) {
    if (delayUs == 0) return MAX_RPM;

    float rpm = 60000000.0f /
                ((float)delayUs * MOTOR_STEPS_PER_REV);

    if (rpm < MIN_RPM) rpm = MIN_RPM;
    if (rpm > MAX_RPM) rpm = MAX_RPM;

    return rpm;
}

void startMotor() {
    motorRequested = true;
    motorRunning = true;

    if (currentRPM < MIN_RPM) {
        currentRPM = MIN_RPM;
    }

    lastStepUs = micros();
}

void stopMotor() {
    motorRequested = false;
}

void setDirection(int direction) {
    motorDirection = direction >= 0 ? 1 : -1;
}

// =====================================================
// ACCELERATION / DECELERATION
// =====================================================

void updateRamp() {
    uint32_t now = millis();

    if (now - lastRampMs < 100) return;

    float dt = (now - lastRampMs) / 1000.0f;
    lastRampMs = now;

    float desiredRPM = motorRequested ? targetRPM : 0.0f;

    float change = ACCELERATION_RPM * dt;

    if (currentRPM < desiredRPM) {
        currentRPM += change;

        if (currentRPM > desiredRPM) {
            currentRPM = desiredRPM;
        }
    }
    else if (currentRPM > desiredRPM) {
        currentRPM -= change;

        if (currentRPM < desiredRPM) {
            currentRPM = desiredRPM;
        }
    }

    if (currentRPM <= 0.0f && !motorRequested) {
        currentRPM = 0.0f;
        motorRunning = false;
        releaseMotor();
    }

    if (currentRPM > 0.0f) {
        stepIntervalUs = rpmToDelay(currentRPM);
    }
}

// =====================================================
// INTERNAL MOTOR STEPPING
// =====================================================

void updateMotor() {
    if (!motorRunning) return;

    if (currentMode == MODE_STEP_DIR) {
        return;
    }

    if (currentRPM <= 0.0f) return;

    uint32_t now = micros();

    if ((uint32_t)(now - lastStepUs) >= stepIntervalUs) {
        lastStepUs = now;

        executeStep(motorDirection);

        if (stepsRemaining > 0) {
            stepsRemaining--;

            if (stepsRemaining == 0) {
                stopMotor();
            }
        }
    }
}

// =====================================================
// EXTERNAL STEP INTERRUPTS
// =====================================================

void handleExternalStep() {
    if (digitalRead(DIR_PIN)) {
        pendingStepsForward++;
    } else {
        pendingStepsBackward++;
    }
}

void updateExternalSteps() {
    if (currentMode != MODE_STEP_DIR) return;

    uint32_t forward = 0;
    uint32_t backward = 0;

    noInterrupts();

    if (pendingStepsForward > 0) {
        forward = pendingStepsForward;
        pendingStepsForward = 0;
    }

    if (pendingStepsBackward > 0) {
        backward = pendingStepsBackward;
        pendingStepsBackward = 0;
    }

    interrupts();

    // Изпълняваме натрупаните външни стъпки
    while (forward > 0) {
        executeStep(1);
        forward--;
    }

    while (backward > 0) {
        executeStep(-1);
        backward--;
    }
}

// =====================================================
// ENCODER
// =====================================================

void nextMode() {
    if (currentMode == MODE_MANUAL) {
        currentMode = MODE_MODBUS;
    }
    else if (currentMode == MODE_MODBUS) {
        currentMode = MODE_STEP_DIR;
    }
    else {
        currentMode = MODE_MANUAL;
    }

    stopMotor();
    stepsRemaining = 0;
}


void updateEncoder() {
    static uint32_t lastEncoderChangeMs = 0;

    int clk = digitalRead(ENC_CLK);
    int dt  = digitalRead(ENC_DT);

    // Отчитаме промяна на CLK
    if (clk != lastEncoderCLK) {
        uint32_t now = millis();

        // Филтър срещу механични смущения
        if (now - lastEncoderChangeMs >= 2) {
            lastEncoderChangeMs = now;

            if (clk == LOW) {
                if (dt != clk) {
                    targetRPM += 5.0f;
                } else {
                    targetRPM -= 5.0f;
                }

                targetRPM = constrain(targetRPM, MIN_RPM, MAX_RPM);
            }
        }
    }

    lastEncoderCLK = clk;

    // Бутон на енкодера
    bool buttonState = digitalRead(ENC_SW);

    if (lastButtonState == HIGH && buttonState == LOW) {
        buttonPressed = true;
        buttonPressTime = millis();
    }

    if (lastButtonState == LOW &&
        buttonState == HIGH &&
        buttonPressed) {

        uint32_t pressDuration = millis() - buttonPressTime;
        buttonPressed = false;

        if (pressDuration < SHORT_PRESS_MS) {
            if (motorRequested) {
                stopMotor();
            } else {
                startMotor();
            }
        }
        else if (pressDuration < LONG_PRESS_MS) {
            setDirection(-motorDirection);
        }
        else {
            nextMode();
        }
    }

    lastButtonState = buttonState;
}


// =====================================================
// MODBUS CRC16
// =====================================================

uint16_t modbusCRC(uint8_t *data, uint8_t length) {
    uint16_t crc = 0xFFFF;

    for (uint8_t i = 0; i < length; i++) {
        crc ^= data[i];

        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 1) {
                crc >>= 1;
                crc ^= 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }

    return crc;
}

// =====================================================
// MODBUS RESPONSE
// =====================================================

void sendModbusResponse(uint8_t *data, uint8_t length) {
    uint16_t crc = modbusCRC(data, length);

    data[length] = crc & 0xFF;
    data[length + 1] = crc >> 8;

    digitalWrite(RS485_DE_RE, HIGH);
    delayMicroseconds(100);

    RS485_SERIAL.write(data, length + 2);
    RS485_SERIAL.flush();

    delayMicroseconds(100);
    digitalWrite(RS485_DE_RE, LOW);
}

// =====================================================
// APPLY MODBUS SETTINGS
// =====================================================

void applyModbusSettings() {
    setDirection(holdingRegisters[1] == 0 ? 1 : -1);

    uint32_t delayValue = holdingRegisters[2];

    if (delayValue < 1000) delayValue = 1000;
    if (delayValue > 10000) delayValue = 10000;

    targetRPM = delayToRPM(delayValue);

    uint16_t mode = holdingRegisters[10];

    if (mode >= MODE_MODBUS && mode <= MODE_MANUAL) {
        currentMode = (OperatingMode)mode;
    }

    if (holdingRegisters[3] == 1) {
        stepsRemaining = holdingRegisters[0];
        startMotor();
    } else {
        stopMotor();
    }
}

// =====================================================
// MODBUS FRAME PROCESSING
// FC03 - Read Holding Registers
// FC06 - Write Single Register
// =====================================================

void processModbusFrame() {
    if (modbusLength < 8) return;

    if (modbusBuffer[0] != MODBUS_SLAVE_ID) return;

    uint16_t receivedCRC =
        modbusBuffer[modbusLength - 2] |
        ((uint16_t)modbusBuffer[modbusLength - 1] << 8);

    uint16_t calculatedCRC =
        modbusCRC(modbusBuffer, modbusLength - 2);

    if (receivedCRC != calculatedCRC) return;

    uint8_t function = modbusBuffer[1];

    // FC03 - Read Holding Registers
    if (function == 0x03 && modbusLength == 8) {
        uint16_t startAddress =
            ((uint16_t)modbusBuffer[2] << 8) |
            modbusBuffer[3];

        uint16_t quantity =
            ((uint16_t)modbusBuffer[4] << 8) |
            modbusBuffer[5];

        if (quantity == 0 ||
            quantity > 16 ||
            startAddress + quantity > 16) {
            return;
        }

        uint8_t response[40];

        response[0] = MODBUS_SLAVE_ID;
        response[1] = 0x03;
        response[2] = quantity * 2;

        for (uint16_t i = 0; i < quantity; i++) {
            uint16_t value = holdingRegisters[startAddress + i];

            response[3 + i * 2] = value >> 8;
            response[4 + i * 2] = value & 0xFF;
        }

        sendModbusResponse(response, 3 + quantity * 2);
    }

    // FC06 - Write Single Register
    else if (function == 0x06 && modbusLength == 8) {
        uint16_t address =
            ((uint16_t)modbusBuffer[2] << 8) |
            modbusBuffer[3];

        uint16_t value =
            ((uint16_t)modbusBuffer[4] << 8) |
            modbusBuffer[5];

        if (address >= 16) return;

        holdingRegisters[address] = value;

        applyModbusSettings();

        uint8_t response[8];

        for (uint8_t i = 0; i < 6; i++) {
            response[i] = modbusBuffer[i];
        }

        sendModbusResponse(response, 6);
    }
}

// =====================================================
// MODBUS RECEIVE
// =====================================================

void updateModbus() {
    while (RS485_SERIAL.available()) {
        uint8_t b = RS485_SERIAL.read();

        if (modbusLength < sizeof(modbusBuffer)) {
            modbusBuffer[modbusLength++] = b;
        } else {
            modbusLength = 0;
        }

        lastModbusByteMs = millis();
    }

    // RTU frame: обработка след пауза между байтовете
    if (modbusLength > 0 &&
        millis() - lastModbusByteMs >= 5) {

        processModbusFrame();
        modbusLength = 0;
    }
}

// =====================================================
// OLED DISPLAY
// =====================================================



void updateDisplay() {
    display.clearBuffer();

    char buffer[32];

    // Режим - по-едър шрифт
    display.setFont(u8g2_font_7x14_tf);

    snprintf(buffer, sizeof(buffer), "%s", getModeName());
    display.drawStr(0, 13, buffer);

    // Състояние
    display.setFont(u8g2_font_6x10_tf);

    display.drawStr(88, 13,
                    motorRequested ? "RUN" : "STOP");

    // Текущи обороти - голям шрифт
    display.setFont(u8g2_font_logisoso24_tf);

    int rpmValue = (int)(currentRPM + 0.5f);

    snprintf(buffer, sizeof(buffer), "%d", rpmValue);
    display.drawStr(0, 43, buffer);

    // RPM надпис
    display.setFont(u8g2_font_6x10_tf);
    display.drawStr(75, 35, "RPM");

    // Зададени обороти
    int setValue = (int)(targetRPM + 0.5f);

    snprintf(buffer, sizeof(buffer), "SET: %d", setValue);
    display.drawStr(0, 60, buffer);

    // Посока
    if (motorDirection > 0) {
        display.drawStr(78, 60, "RIGHT");
    } else {
        display.drawStr(78, 60, "LEFT");
    }

    display.sendBuffer();
}



// =====================================================
// SETUP
// =====================================================

void setup() {
    // Освобождаваме PB3/PB4 от JTAG, като запазваме SWD.
    #if defined(__HAL_AFIO_REMAP_SWJ_NOJTAG)
        __HAL_AFIO_REMAP_SWJ_NOJTAG();
    #endif

    // L298N
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);

    releaseMotor();

    // STEP / DIR
    pinMode(STEP_PIN, INPUT_PULLDOWN);
    pinMode(DIR_PIN, INPUT_PULLDOWN);

    attachInterrupt(
        digitalPinToInterrupt(STEP_PIN),
        handleExternalStep,
        RISING
    );

    // Encoder
    pinMode(ENC_CLK, INPUT_PULLUP);
    pinMode(ENC_DT, INPUT_PULLUP);
    pinMode(ENC_SW, INPUT_PULLUP);

    lastEncoderCLK = digitalRead(ENC_CLK);

    // RS485
    pinMode(RS485_DE_RE, OUTPUT);
    digitalWrite(RS485_DE_RE, LOW);

    RS485_SERIAL.begin(MODBUS_BAUD, SERIAL_8N1);

    // OLED
    display.begin();
    display.setContrast(255);
    display.clearBuffer();
    display.setFont(u8g2_font_6x10_tf);
    display.drawStr(0, 20, "STEPPER CONTROLLER");
    display.drawStr(0, 40, "INITIALIZING...");
    display.sendBuffer();

    // Modbus defaults
    holdingRegisters[0] = 0;
    holdingRegisters[1] = 0;
    holdingRegisters[2] = 5000;
    holdingRegisters[3] = 0;
    holdingRegisters[10] = MODE_MANUAL;

    lastRampMs = millis();
    lastDisplayMs = millis();

    delay(1000);
}

// =====================================================
// MAIN LOOP
// =====================================================

void loop() {
    updateModbus();

    updateEncoder();

    updateExternalSteps();

    updateRamp();

    updateMotor();

    if (millis() - lastDisplayMs >= 250) {
        lastDisplayMs = millis();
        updateDisplay();
    }
}
