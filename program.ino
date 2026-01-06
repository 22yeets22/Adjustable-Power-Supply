#include <Wire.h>
#include <TFT_eSPI.h>  // https://github.com/Bodmer/TFT_eSPI/
#include <Adafruit_INA228.h>  // https://github.com/adafruit/Adafruit_INA228/
#include <Adafruit_AHTX0.h>  // https://github.com/adafruit/Adafruit_AHTX0/
#include <Adafruit_SHT4x.h>  // https://github.com/adafruit/Adafruit_SHT4X/
#include <PID_v1.h>  // https://github.com/br3ttb/Arduino-PID-Library/

// ------------------- PIN DEFINITIONS -------------------
#define INTAKE_PWM_PIN   16
#define INTAKE_TACH_PIN  17
#define EXHAUST_PWM_PIN   2
#define EXHAUST_TACH_PIN  4

#define I2C_SDA        21
#define I2C_SCL        22

// ------------------- CONSTANTS -------------------
const float TARGET_TEMP = 37.5;
const float WARN_TEMP   = 50.0;
const float CRIT_TEMP   = 55.0;
const float WARN_CURRENT = 15.0;
const float CRIT_CURRENT = 20.0;
const float WARN_VOLTAGE = 35.0;
const float CRIT_VOLTAGE = 40.0;
const float WARN_POWER = 400.0;
const float CRIT_POWER = 450.0;
const float WARN_FAN_SPEED = 900.0;  // pwm signal
const float CRIT_FAN_SPEED = 1000.0;

const uint32_t UPDATE_MS = 2000; // update every 0.5hz bc temp sensors slow
const uint32_t TACH_INTERVAL_MS = 1000;

// ------------------- GLOBAL OBJECTS -------------------
TFT_eSPI tft = TFT_eSPI();
Adafruit_INA228 ina;
Adafruit_AHTX0 aht20;
Adafruit_SHT4x sht40;

double temperature;
double humidity;
double fanPWM; // PID Output but pid
double fanTargetTemp = TARGET_TEMP;

double Kp = 40.0, Ki = 5.0, Kd = 1.0; // 10-bit PWM range
// input, output, target, kp, ki, kd, i have no clue
PID fanPID(&temperature, &fanPWM, &fanTargetTemp, Kp, Ki, Kd, DIRECT);

volatile uint32_t fan1Pulses = 0;
volatile uint32_t fan2Pulses = 0;

uint32_t lastUpdate = 0;
uint32_t lastTachCalc = 0;
float fan1RPM = 0;
float fan2RPM = 0;

// fan failure stuff
bool fan1Failed = false;
bool fan2Failed = false;
const int MIN_RPM_THRESHOLD = 100; // RPM below this is considered a fail if PWM high 
const int MIN_PWM_THRESHOLD = 200; // PWM above this is fail if RPM low


// ------------------- TACH INTERRUPTS -------------------
void IRAM_ATTR fan1TachISR() { fan1Pulses++; }
void IRAM_ATTR fan2TachISR() { fan2Pulses++; }

// ------------------- SETUP -------------------
void setup() {
  Serial.begin(115200);
  Wire.begin(I2C_SDA, I2C_SCL);

  tft.init();
  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextSize(2);
  tft.setCursor(10, 10);
  tft.println("SYSTEM INITIALIZING...");

  // Sensors init
  if (!ina.begin()) { Serial.println("INA228 Not Found"); }
  // Calibration: the INA228 has a 0.002 ohm shunt
  ina.setShunt(0.002, 25.0);  // 2mohm, 25a random number
  ina.setVoltageConversionTime(INA228_TIME_1052_us);  // 1052 microseconds time to measure voltage
  ina.setCurrentConversionTime(INA228_TIME_1052_us);
  
  if (!aht20.begin()) { Serial.println("AHT20 Not Found"); }
  if (!sht40.begin()) { Serial.println("SHT40 Not Found"); }

  sht40.setPrecision(SHT4X_HIGH_PRECISION);  // theres also medium and low

  // Fans PWM (ESP32 Core 3.0+ syntax)
  ledcAttach(INTAKE_PWM_PIN, 25000, 10); // Pin, Freq, Resolution
  ledcAttach(EXHAUST_PWM_PIN, 25000, 10);

  // Tach pins
  pinMode(INTAKE_TACH_PIN, INPUT_PULLUP);
  pinMode(EXHAUST_TACH_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(INTAKE_TACH_PIN), fan1TachISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(EXHAUST_TACH_PIN), fan2TachISR, FALLING);

  // PID
  fanPID.SetOutputLimits(0, 1023); // Match 10-bit resolution
  fanPID.SetMode(AUTOMATIC);
}

void updateTemps() {
  sensors_event_t humidity_aht, humidity_sht, temp_aht, temp_sht;
  aht20.getEvent(&humidity_aht, &temp_aht);
  sht40.getEvent(&humidity_sht, &temp_sht);

  humidity = (humidity_aht.relative_humidity + humidity_sht.relative_humidity) * 0.5;  // average humidity
  temperature = max(temp_aht.temperature, temp_sht.temperature);  // higher temperature
}

// ------------------- LOOP -------------------
void loop() {
  uint32_t now = millis();

  if (now - lastTachCalc >= TACH_INTERVAL_MS) {
    // Fan stuff
    detachInterrupt(digitalPinToInterrupt(INTAKE_TACH_PIN));
    detachInterrupt(digitalPinToInterrupt(EXHAUST_TACH_PIN));
    
    fan1RPM = (fan1Pulses / 2.0) * (60000.0 / (now - lastTachCalc));
    fan2RPM = (fan2Pulses / 2.0) * (60000.0 / (now - lastTachCalc));
    
    fan1Pulses = 0;
    fan2Pulses = 0;
    lastTachCalc = now;
    
    attachInterrupt(digitalPinToInterrupt(INTAKE_TACH_PIN), fan1TachISR, FALLING);
    attachInterrupt(digitalPinToInterrupt(EXHAUST_TACH_PIN), fan2TachISR, FALLING);

    // Fan Failure Detection
    fan1Failed = (fanPWM > MIN_PWM_THRESHOLD && fan1RPM < MIN_RPM_THRESHOLD);  // if pwm high but no speed
    fan2Failed = (fanPWM > MIN_PWM_THRESHOLD && fan2RPM < MIN_RPM_THRESHOLD);
  }

  if (now - lastUpdate >= UPDATE_MS) {
    // UPDATE SCREEN
    lastUpdate = now;

    updateTemps();
    fanPID.Compute();

    // Fan 1 (Intake) is 15% faster than Fan 2 (Exhaust)
    double fan1PWM_Output = fanPWM * 1.15;
    double fan2PWM_Output = fanPWM;

    // Constrain to 10-bit limit (0-1023)
    if (fan1PWM_Output > 1023) fan1PWM_Output = 1023;
    if (fan2PWM_Output > 1023) fan2PWM_Output = 1023;

    ledcWrite(INTAKE_PWM_PIN, (uint32_t)fan1PWM_Output);
    ledcWrite(EXHAUST_PWM_PIN, (uint32_t)fan2PWM_Output);

    float voltage = ina.getBusVoltage_V();
    float current = ina.getCurrent_mA() / 1000.0;
    float power   = ina.getPower_mW() / 1000.0;

    displayUpdate(voltage, current, power);
  }

  delay(5);
}

uint16_t getColor(float value, float warn, float crit) {
    if (value >= crit) return TFT_RED;
    if (value >= warn) return TFT_YELLOW;
    return TFT_WHITE;
}

// ------------------- FUNCTIONS -------------------
void displayUpdate(float voltage, float current, float power) {
  tft.setCursor(0, 0);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  
  tft.setTextColor(getColor(voltage, WARN_VOLTAGE, CRIT_VOLTAGE), TFT_BLACK);
  tft.printf("Voltage: %5.2f V \n", voltage);

  tft.setTextColor(getColor(current, WARN_CURRENT, CRIT_CURRENT), TFT_BLACK);
  tft.printf("Current: %5.2f A \n", current);

  tft.setTextColor(getColor(power, WARN_POWER, CRIT_POWER), TFT_BLACK);
  tft.printf("Power:   %5.2f W \n\n", power);

  tft.setTextColor(getColor(temperature, WARN_TEMP, CRIT_TEMP), TFT_BLACK);
  tft.printf("Temp: %4.1f C \n", temperature);

  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.printf("Humidity: %4.1f rH \n", humidity);

  tft.setTextColor(getColor(fanPWM, WARN_FAN_SPEED, CRIT_FAN_SPEED), TFT_BLACK);
  tft.printf("Fan Speed: %4.0f%%    \n", (fanPWM / 1023.0 * 100.0));
  
  // Display RPM with failure color-coding
  tft.setTextColor(fan1Failed ? TFT_RED : TFT_GREEN, TFT_BLACK);
  tft.printf("F1 (In):  %4.0f RPM %s\n", fan1RPM, fan1Failed ? "FAIL" : "OK  ");
  
  tft.setTextColor(fan2Failed ? TFT_RED : TFT_GREEN, TFT_BLACK);
  tft.printf("F2 (Out): %4.0f RPM %s\n\n", fan2RPM, fan2Failed ? "FAIL" : "OK  ");

  // Status Alerts
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  if (fan1Failed || fan2Failed) {
    tft.setTextColor(TFT_RED, TFT_BLACK);
    tft.println("!! FAN FAILURE !! ");
  } else if (current > CRIT_CURRENT || temperature > CRIT_TEMP) {
    tft.setTextColor(TFT_RED, TFT_BLACK);
    tft.println("!! CRITICAL !!    ");
  } else {
    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    tft.println("SYSTEM NOMINAL    ");
  }
}
