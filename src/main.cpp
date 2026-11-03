#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <DFRobot_RTU.h>

// Режими на работа
enum ControlMode {
  MODE_MODBUS = 1,
  MODE_STEP_DIR = 2,
  MODE_MANUAL = 3
};
ControlMode currentMode = MODE_MODBUS;

// Пинове за L298N мотора
const int IN1 = PA0;
const int IN2 = PA1;
const int IN3 = PA2;
const int IN4 = PA3;

// Пинове за Step/Dir входове
const int STEP_PIN = PB3;
const int DIR_PIN = PB4;

// Пинове за Ротационен енкодер
const int ENC_CLK = PB12;
const int ENC_DT  = PB13;
const int ENC_SW  = PB14;

// RS485 / Modbus сериен порт (Serial2 на STM32)
#define RS485_SERIAL Serial
const int MAX485_DE_RE_PIN = PB1;

// OLED Дисплей (I2C)
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// Таблица с фазите за пълен стъпка (Full-Step)
const int stepTable[4][4] = {
  {HIGH, LOW,  HIGH, LOW},  
  {LOW,  HIGH, HIGH, LOW},  
  {LOW,  HIGH, LOW,  HIGH}, 
  {HIGH, LOW,  LOW,  HIGH}  
};
int currentStep = 0;

// Променливи за енкодера
int lastClkState;
unsigned long lastButtonPress = 0;
unsigned long lastDisplayUpdate = 0;

// Modbus / Движение променливи
long mbStepsRemaining = 0;
bool mbDirection = true;
unsigned long mbStepDelay = 2000;
unsigned long lastMotorStepTime = 0;
bool mbIsRunning = false;

// Локални регистри за симулация на Slave отговори
uint16_t holdingRegisters[16] = {0, 1, 2000, 0, 0, 0, 0, 0, 0, 0, 1, 0}; 
// Reg 0: Steps, Reg 1: Dir, Reg 2: Delay, Reg 3: Start, Reg 10: Mode

void executeStep(bool dir) {
  if (dir) {
    currentStep = (currentStep + 1) % 4;
  } else {
    currentStep = (currentStep - 1 + 4) % 4;
  }
  digitalWrite(IN1, stepTable[currentStep][0]);
  digitalWrite(IN2, stepTable[currentStep][1]);
  digitalWrite(IN3, stepTable[currentStep][2]);
  digitalWrite(IN4, stepTable[currentStep][3]);
}

void handleExternalStep() {
  if (currentMode == MODE_STEP_DIR) {
    bool dirState = digitalRead(DIR_PIN);
    executeStep(dirState);
  }
}

void setup() {
  // Настройка на мотора
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);

  // Настройка на Step/Dir входове
  pinMode(STEP_PIN, INPUT_PULLUP);
  pinMode(DIR_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(STEP_PIN), handleExternalStep, RISING);

  // Настройка на Енкодер
  pinMode(ENC_CLK, INPUT_PULLUP);
  pinMode(ENC_DT, INPUT_PULLUP);
  pinMode(ENC_SW, INPUT_PULLUP);
  lastClkState = digitalRead(ENC_CLK);

  // Инициализация на дисплея
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  // Инициализация на RS485
  pinMode(MAX485_DE_RE_PIN, OUTPUT);
  digitalWrite(MAX485_DE_RE_PIN, LOW); // RX режим по подразбиране
  RS485_SERIAL.begin(9600, SERIAL_8N1);
}

// Проста функция за обслужване на входящи Modbus RTU заявки (Slave режим)
void pollModbusSlave() {
  if (RS485_SERIAL.available() >= 8) {
    uint8_t buffer[8];
    for (int i = 0; i < 8; i++) {
      buffer[i] = RS485_SERIAL.read();
    }

    // Проверка за адрес (напр. Slave ID = 1) и функция за писане (0x06 - Write Single Register)
    if (buffer[0] == 1 && buffer[1] == 0x06) {
      uint16_t regAddr = (buffer[2] << 8) | buffer[3];
      uint16_t regValue = (buffer[4] << 8) | buffer[5];

      if (regAddr < 16) {
        holdingRegisters[regAddr] = regValue;
      }

      // Връщане на отговор към Master (ехо на заявката при успешчен запис)
      digitalWrite(MAX485_DE_RE_PIN, HIGH); // Превключване в TX
      delayMicroseconds(500);
      RS485_SERIAL.write(buffer, 8);
      RS485_SERIAL.flush();
      delayMicroseconds(500);
      digitalWrite(MAX485_DE_RE_PIN, LOW);  // Обратно в RX
    }
  }
}

void loop() {
  // 1. Обслужване на комуникацията
  pollModbusSlave();

  // Синхронизиране на режима от регистър 10
  int modeReq = holdingRegisters[10];
  if (modeReq >= 1 && modeReq <= 3) {
    currentMode = (ControlMode)modeReq;
  }

  // 2. Четене на бутона на енкодера (смяна на режим)
  if (digitalRead(ENC_SW) == LOW) {
    unsigned long currentTime = millis();
    if (currentTime - lastButtonPress > 300) {
      lastButtonPress = currentTime;
      if (currentMode == MODE_MODBUS) currentMode = MODE_STEP_DIR;
      else if (currentMode == MODE_STEP_DIR) currentMode = MODE_MANUAL;
      else currentMode = MODE_MODBUS;
      
      holdingRegisters[10] = currentMode;
    }
  }

  // 3. Изпълнение на логиката според режима
  if (currentMode == MODE_MODBUS) {
    // Проверка за старт команда (Регистър 3 == 1)
    if (!mbIsRunning && holdingRegisters[3] == 1) {
      mbStepsRemaining = holdingRegisters[0];
      mbDirection = holdingRegisters[1];
      mbStepDelay = holdingRegisters[2];
      if (mbStepsRemaining > 0) mbIsRunning = true;
      holdingRegisters[3] = 0; // Изчистване на старта
    }

    // Неблокиращо движение
    if (mbIsRunning && mbStepsRemaining > 0) {
      unsigned long t = micros();
      if (t - lastMotorStepTime >= mbStepDelay) {
        lastMotorStepTime = t;
        executeStep(mbDirection);
        mbStepsRemaining--;
        if (mbStepsRemaining == 0) mbIsRunning = false;
      }
    }
  } 
  else if (currentMode == MODE_MANUAL) {
    // Ръчно управление с енкодера
    int currentClkState = digitalRead(ENC_CLK);
    if (currentClkState != lastClkState && currentClkState == LOW) {
      bool dir = digitalRead(ENC_DT) != currentClkState;
      executeStep(dir);
    }
    lastClkState = currentClkState;
  }

  // 4. Опресняване на дисплея на всеки 250мс
  if (millis() - lastDisplayUpdate >= 250) {
    lastDisplayUpdate = millis();
    display.clearDisplay();
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.print("Controller Active");

    display.setCursor(0, 15);
    display.print("Mode: ");
    if (currentMode == MODE_MODBUS) display.print("1.Modbuss");
    else if (currentMode == MODE_STEP_DIR) display.print("2.Step/Dirrr/");
    else display.print("3.Manual");

    display.setCursor(0, 35);
    if (currentMode == MODE_MODBUS) {
      display.print(mbIsRunning ? "Running..." : "Idle");
      display.setCursor(0, 50);
      display.print("Left: "); display.print(mbStepsRemaining);
    } else if (currentMode == MODE_STEP_DIR) {
      display.print("Waiting pulses...");
    } else {
      display.print("Turn encoder!");
    }
    display.display();
  }
}