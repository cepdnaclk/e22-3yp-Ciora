#include <Wire.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include "MAX30105.h"
#include "heartRate.h"

// Automatically enables Classic SerialBT if compiled on a standard ESP32,
// while preventing compilation errors on ESP32-C3 (which lacks Classic BT hardware).
#if defined(CONFIG_BT_CLASSIC_ENABLED) || !defined(CONFIG_IDF_TARGET_ESP32C3)
#include "BluetoothSerial.h"
BluetoothSerial SerialBT;
#define USE_CLASSIC_BT 1
#else
#define USE_CLASSIC_BT 0
#endif

// --- Pin Allocations (Matched to ESP32-C3 SuperMini PCB Layout) ---
#define I2C_SDA_PIN        2    // GPIO 2
#define I2C_SCL_PIN        1    // GPIO 1
#define MOTOR_PIN          10   // GPIO 10
#define THERMISTOR_PIN     0    // GPIO 0 (ADC1_CH0)
#define MPU_ADDR           0x68 // MPU-6050 AD0 is tied to GND

// --- Low-Side NTC Parameters (R1 = 100k to 3V3, C8 = NTC to GND) ---
const float SERIES_RESISTOR    = 100000.0;
const float NOMINAL_RESISTANCE = 100000.0;
const float NOMINAL_TEMP       = 25.0;
const float B_COEFFICIENT      = 3950.0;
const int   ADC_MAX_VALUE      = 4095;

// --- BLE Serial (Nordic UART Service) UUIDs ---
#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

BLEServer *pServer = NULL;
BLECharacteristic *pTxCharacteristic = NULL;
bool deviceConnected = false;
bool oldDeviceConnected = false;

MAX30105 particleSensor;
bool mpuActive = false;
bool maxActive = false;

// --- Motion, Gyro, Orientation & Step Tracking Variables ---
float ax = 0, ay = 0, az = 1.0;
float gyroX = 0, gyroY = 0, gyroZ = 0;
float totalGyro = 0;
float pitch = 0, roll = 0;
float mpuTemp = 36.5;
float totalAcc = 1.0;
float filteredMag = 1.0;

int stepCount = 0;
bool stepArmed = false;
unsigned long lastStepTime = 0;

// Upgraded deliberate step thresholds to filter out jitters/typing
const float STEP_PEAK_THRESH          = 0.40;  
const float STEP_RESET_THRESH         = 0.12;  
const unsigned long STEP_MIN_INTERVAL = 350;  

// --- Upgraded 4-Stage Fall Detection State Machine ---
enum FallState { MONITORING, FREE_FALL, POST_IMPACT_CHECK, ALARM_ACTIVE };
FallState fallState = MONITORING;
unsigned long stateStartTime = 0;
unsigned long lowGStartTime = 0;
unsigned long fallAlarmUntil = 0;
bool isFallAlarmActive = false;

const float FALL_LOW_G_THRESH     = 0.55;  
const unsigned long MIN_LOW_G_MS  = 60;    
const float FALL_IMPACT_G_THRESH  = 2.85;  
const float FALL_GYRO_THRESH      = 160.0; 
const float STILLNESS_MAX_DYN_G   = 0.18;  

// --- High-Accuracy Heart Rate & SpO2 Tracking Variables ---
const byte RATE_SIZE = 8;         // 8-beat rolling buffer for stable averaging
byte rates[RATE_SIZE];
byte rateSpot = 0;
byte validBeatsCount = 0;         // Tracks how many valid beats collected
long lastBeatTime = 0;

float instantBPM = 0;
int beatAvg = 0;                  // Accurate/stable BPM output

uint32_t latestIR = 0;
uint32_t latestRed = 0;
float irDC = 0;
float redDC = 0;
float irACSqSum = 0;
float redACSqSum = 0;
uint16_t spo2Samples = 0;
int spo2Val = 0;
bool skinContact = false;

// BLE Server Callbacks
class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
    }
    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
    }
};

// BLE RX Callback (Receiving commands over BLE)
class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      String rxValue = pCharacteristic->getValue();
      if (rxValue.length() > 0) {
        Serial.print("[BLE RX]: ");
        for (int i = 0; i < rxValue.length(); i++) {
          Serial.print(rxValue[i]);
        }
        Serial.println();
      }
    }
};

// Unified Printer: Outputs simultaneously to USB Serial, Classic SerialBT, and BLE Serial
void printBT(String msg) {
  Serial.print(msg);
#if USE_CLASSIC_BT
  if (SerialBT.hasClient()) {
    SerialBT.print(msg);
  }
#endif
  if (deviceConnected && pTxCharacteristic != NULL) {
    int len = msg.length();
    int chunkSize = 180; 
    for (int i = 0; i < len; i += chunkSize) {
      String chunk = msg.substring(i, min(i + chunkSize, len));
      pTxCharacteristic->setValue(chunk.c_str());
      pTxCharacteristic->notify();
      delay(5); 
    }
  }
}

void printlnBT(String msg) {
  printBT(msg + "\r\n");
}

float readNTC() {
  long adcSum = 0;
  for (int i = 0; i < 4; i++) {
    adcSum += analogRead(THERMISTOR_PIN);
    delay(1);
  }
  float adcAvg = adcSum / 4.0;
  if (adcAvg <= 15.0 || adcAvg >= (ADC_MAX_VALUE - 15.0)) return -999.0;

  float rNtc = SERIES_RESISTOR / ((ADC_MAX_VALUE / adcAvg) - 1.0);
  float steinhart = rNtc / NOMINAL_RESISTANCE;
  steinhart = log(steinhart);
  steinhart /= B_COEFFICIENT;
  steinhart += 1.0 / (NOMINAL_TEMP + 273.15);
  steinhart = 1.0 / steinhart;
  return steinhart - 273.15;
}

bool setupMPU() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0x01); // Clock = X-gyro PLL
  if (Wire.endTransmission(true) != 0) return false;
  delay(10);

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1C); // ACCEL_CONFIG: +/- 4g full scale (8192 LSB/g)
  Wire.write(0x08);
  Wire.endTransmission(true);

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1B); // GYRO_CONFIG: +/- 500 deg/s full scale (65.5 LSB/deg/s)
  Wire.write(0x08);
  Wire.endTransmission(true);

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1A); // DLPF 44Hz
  Wire.write(0x02);
  Wire.endTransmission(true);

  return true;
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(MOTOR_PIN, OUTPUT);
  digitalWrite(MOTOR_PIN, LOW);

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(100000);

#if USE_CLASSIC_BT
  SerialBT.begin("ESP32C3-HealthBand-BT");
  Serial.println("[+] Classic SerialBT Active. Device Name: 'ESP32C3-HealthBand-BT'");
#endif

  BLEDevice::init("ESP32C3-HealthBand");
  BLEDevice::setMTU(247); 
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *pService = pServer->createService(SERVICE_UUID);

  pTxCharacteristic = pService->createCharacteristic(
                        CHARACTERISTIC_UUID_TX,
                        BLECharacteristic::PROPERTY_NOTIFY
                      );
  pTxCharacteristic->addDescriptor(new BLE2902());

  BLECharacteristic *pRxCharacteristic = pService->createCharacteristic(
                                           CHARACTERISTIC_UUID_RX,
                                           BLECharacteristic::PROPERTY_WRITE |
                                           BLECharacteristic::PROPERTY_WRITE_NR
                                         );
  pRxCharacteristic->setCallbacks(new MyCallbacks());

  pService->start();

  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();

  Serial.println("\n--- Sensor Node Initializing ---");
  Serial.println("[+] BLE Serial Active. Device Name: 'ESP32C3-HealthBand'");

  if (setupMPU()) {
    mpuActive = true;
    Serial.println("[+] MPU-6050 Ready (Accel + Gyro + Tilt + Die Temp)");
  } else {
    Serial.println("[-] MPU-6050 Init Failed");
  }

  Wire.beginTransmission(0x57);
  Wire.write(0x09);
  Wire.write(0x40);
  Wire.endTransmission();
  delay(100);

  if (particleSensor.begin(Wire, I2C_SPEED_STANDARD)) {
    maxActive = true;
    Serial.println("[+] MAX30102 Ready (High-Accuracy FIFO Mode)");
    particleSensor.setup(0x24, 8, 2, 400, 411, 4096);
    particleSensor.setPulseAmplitudeRed(0x1F); 
    particleSensor.setPulseAmplitudeIR(0x24);  
  } else {
    Serial.println("[-] MAX30102 Init Failed");
  }
}

unsigned long lastTelemetry = 0;
unsigned long lastAlarmBuzz = 0;
unsigned long lastMpuRead = 0;

void loop() {
  unsigned long now = millis();

#if USE_CLASSIC_BT
  if (SerialBT.available()) {
    Serial.print("[BT RX]: ");
    while (SerialBT.available()) {
      Serial.write(SerialBT.read());
    }
    Serial.println();
  }
#endif

  if (!deviceConnected && oldDeviceConnected) {
    delay(500);
    pServer->startAdvertising();
    Serial.println("[*] BLE Advertising Restarted");
    oldDeviceConnected = deviceConnected;
  }
  if (deviceConnected && !oldDeviceConnected) {
    oldDeviceConnected = deviceConnected;
    Serial.println("[*] BLE Client Connected");
  }

  // 1. MAX30102 Processing
  if (maxActive) {
    particleSensor.check(); 

    while (particleSensor.available()) {
      latestIR  = particleSensor.getFIFOIR();
      latestRed = particleSensor.getFIFORed();
      particleSensor.nextSample();

      if (latestIR < 40000) {
        skinContact = false;
        validBeatsCount = 0;
        rateSpot = 0;
        beatAvg = 0;
        instantBPM = 0;
        spo2Val = 0;
        irDC = 0;
        redDC = 0;
        irACSqSum = 0;
        redACSqSum = 0;
        spo2Samples = 0;
        for (byte i = 0; i < RATE_SIZE; i++) rates[i] = 0;
      } else {
        skinContact = true;

        if (irDC == 0) {
          irDC = latestIR;
          redDC = latestRed;
        }
        irDC = 0.95 * irDC + 0.05 * latestIR;
        redDC = 0.95 * redDC + 0.05 * latestRed;

        float irAC = latestIR - irDC;
        float redAC = latestRed - redDC;
        irACSqSum += irAC * irAC;
        redACSqSum += redAC * redAC;
        spo2Samples++;

        if (spo2Samples >= 100) { 
          float R = (sqrt(redACSqSum / spo2Samples) / redDC) / (sqrt(irACSqSum / spo2Samples) / irDC);
          int calcSpo2 = (int)(110.0 - 25.0 * R);
          if (calcSpo2 > 100) calcSpo2 = 100;
          if (calcSpo2 < 70) calcSpo2 = 70;
          if (beatAvg > 0) spo2Val = calcSpo2;
          irACSqSum = 0;
          redACSqSum = 0;
          spo2Samples = 0;
        }

        if (checkForBeat(latestIR) == true) {
          unsigned long beatNow = millis();
          long delta = beatNow - lastBeatTime;
          lastBeatTime = beatNow;

          float candidateBPM = 60000.0 / (float)delta;

          if (candidateBPM >= 45.0 && candidateBPM <= 185.0) {
            bool isOutlier = false;
            if (validBeatsCount >= 3 && beatAvg > 0) {
              if (fabs(candidateBPM - beatAvg) > (beatAvg * 0.30)) {
                isOutlier = true;
              }
            }

            if (!isOutlier) {
              instantBPM = candidateBPM;
              rates[rateSpot++] = (byte)candidateBPM;
              rateSpot %= RATE_SIZE;
              if (validBeatsCount < RATE_SIZE) validBeatsCount++;

              int sum = 0;
              for (byte x = 0; x < validBeatsCount; x++) {
                sum += rates[x];
              }
              beatAvg = sum / validBeatsCount;
            }
          }
        }
      }
    }
  }

  // 2. Read MPU-6050 Using Split, Verified Transactions & Noise-Guarded Filters
  if (mpuActive && (now - lastMpuRead >= 20)) {
    lastMpuRead = now;

    uint8_t accBuf[8]; 
    uint8_t gyrBuf[6]; 

    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B);
    bool accSuccess = (Wire.endTransmission(false) == 0 && Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)8) == 8);
    if (accSuccess) {
      for (int i = 0; i < 8; i++) accBuf[i] = Wire.read();
    }

    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x43);
    bool gyrSuccess = (Wire.endTransmission(false) == 0 && Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)6) == 6);
    if (gyrSuccess) {
      for (int i = 0; i < 6; i++) gyrBuf[i] = Wire.read();
    }

    if (accSuccess && gyrSuccess) {
      int16_t rawAx   = (int16_t)((accBuf[0] << 8) | accBuf[1]);
      int16_t rawAy   = (int16_t)((accBuf[2] << 8) | accBuf[3]);
      int16_t rawAz   = (int16_t)((accBuf[4] << 8) | accBuf[5]);
      int16_t rawTemp = (int16_t)((accBuf[6] << 8) | accBuf[7]);

      int16_t rawGx   = (int16_t)((gyrBuf[0] << 8) | gyrBuf[1]);
      int16_t rawGy   = (int16_t)((gyrBuf[2] << 8) | gyrBuf[3]);
      int16_t rawGz   = (int16_t)((gyrBuf[4] << 8) | gyrBuf[5]);

      bool gyroLatched = (rawGx != 0 && rawGx == rawGy && rawGy == rawGz);
      bool accLatched  = (rawAx != 0 && rawAx == rawAy && rawAy == rawAz);

      if (!gyroLatched && !accLatched) {
        float candAx = rawAx / 16384.0; // Correct +/- 4g scaling factor
        float candAy = rawAy / 16384.0;
        float candAz = rawAz / 16384.0;
        float candAcc = sqrt((candAx * candAx) + (candAy * candAy) + (candAz * candAz));

        // Guard against transient bus spikes and clamp to valid range
        if (candAcc > 0.7 && candAcc < 1.8) {
          ax = (0.7 * ax) + (0.3 * candAx);
          ay = (0.7 * ay) + (0.3 * candAy);
          az = (0.7 * az) + (0.3 * candAz);
          totalAcc = sqrt((ax * ax) + (ay * ay) + (az * az));

          float targetPitch = atan2(-ax, sqrt((ay * ay) + (az * az))) * 57.2958;
          float targetRoll  = atan2(ay, az) * 57.2958;

          pitch = (0.8 * pitch) + (0.2 * targetPitch);
          roll  = (0.8 * roll)  + (0.2 * targetRoll);
        }

        if (rawTemp != 0) {
          float candTemp = (rawTemp / 340.0) + 36.53;
          if (candTemp > 10.0 && candTemp < 60.0) { 
            mpuTemp = (0.8 * mpuTemp) + (0.2 * candTemp);
          }
        }

        float candGx = rawGx / 65.5;
        float candGy = rawGy / 65.5;
        float candGz = rawGz / 65.5;
        
        if (fabs(candGx) < 500.0 && fabs(candGy) < 500.0 && fabs(candGz) < 500.0) {
          gyroX = (fabs(candGx) < 2.0) ? 0 : candGx;
          gyroY = (fabs(candGy) < 2.0) ? 0 : candGy;
          gyroZ = (fabs(candGz) < 2.0) ? 0 : candGz;
          totalGyro = sqrt((gyroX * gyroX) + (gyroY * gyroY) + (gyroZ * gyroZ));
        }
      }
    }
  }
  
  // 3. Step Counting State Machine
  filteredMag = (0.95 * filteredMag) + (0.05 * totalAcc);
  float dynamicMag = fabs(totalAcc - filteredMag);

  if (!isFallAlarmActive) {
    if (dynamicMag > STEP_PEAK_THRESH && !stepArmed) {
      if (now - lastStepTime > STEP_MIN_INTERVAL) {
        stepArmed = true;
      }
    } else if (dynamicMag < STEP_RESET_THRESH && stepArmed) {
      stepArmed = false;
      lastStepTime = now;
      stepCount++;
    }
  }

  // 4. Upgraded 4-Stage Fall Detection State Machine
  switch (fallState) {
    case MONITORING:
      if (totalAcc > 0.05 && totalAcc < FALL_LOW_G_THRESH) {
        if (lowGStartTime == 0) lowGStartTime = now;
        if (now - lowGStartTime >= MIN_LOW_G_MS) {
          fallState = FREE_FALL;
          stateStartTime = now;
          lowGStartTime = 0;
        }
      } else {
        lowGStartTime = 0;
      }
      break;

    case FREE_FALL:
      if (totalAcc > FALL_IMPACT_G_THRESH && totalGyro > FALL_GYRO_THRESH) {
        fallState = POST_IMPACT_CHECK;
        stateStartTime = now;
      } else if (now - stateStartTime > 500) {
        fallState = MONITORING;
      }
      break;

    case POST_IMPACT_CHECK:
      if (now - stateStartTime > 500 && now - stateStartTime <= 1500) {
        if (dynamicMag > STILLNESS_MAX_DYN_G && totalGyro > 120.0) {
          fallState = MONITORING;
        }
      } else if (now - stateStartTime > 1500) {
        fallState = ALARM_ACTIVE;
        isFallAlarmActive = true;
        fallAlarmUntil = now + 3000;
        printlnBT("\n[!] >>> FALL DETECTED! <<< [!]");
      }
      break;

    case ALARM_ACTIVE:
      if (now > fallAlarmUntil) {
        isFallAlarmActive = false;
        fallState = MONITORING;
      }
      break;
  }

  // 5. Vibration Alerts
  if (isFallAlarmActive) {
    digitalWrite(MOTOR_PIN, HIGH);
  } else if (skinContact && (beatAvg == 0 || (now - lastBeatTime > 2500))) {
    if (now - lastAlarmBuzz >= 2000) {
      lastAlarmBuzz = now;
      digitalWrite(MOTOR_PIN, HIGH);
    } else if (now - lastAlarmBuzz >= 80) {
      digitalWrite(MOTOR_PIN, LOW);
    }
  } else {
    digitalWrite(MOTOR_PIN, LOW);
  }

  // 6. Expanded Telemetry Output (Every 200ms)
  if (now - lastTelemetry >= 200) {
    lastTelemetry = now;
    float ntc = readNTC();
    float currentDynMag = fabs(totalAcc - filteredMag);

    String dataString = "SkinTemp:";
    if (ntc == -999.0) dataString += "ERR";
    else dataString += String(ntc, 1) + "C";

    dataString += " | BoardTemp:" + String(mpuTemp, 1) + "C";
    dataString += " | Contact:" + String(skinContact ? "YES" : "NO") + "(IR:" + String(latestIR) + ")";
    dataString += " | BPM:" + String(beatAvg);
    dataString += " | SpO2:" + String(spo2Val) + "%";
    dataString += " | Steps:" + String(stepCount);
    dataString += " | Acc:" + String(totalAcc, 2) + "g[" + String(ax, 1) + "," + String(ay, 1) + "," + String(az, 1) + "]";
    dataString += " | Gyro:" + String(gyroX, 0) + "," + String(gyroY, 0) + "," + String(gyroZ, 0) + "deg/s";
    dataString += " | Tilt[P,R]:" + String(pitch, 0) + "," + String(roll, 0) + "deg";
    dataString += " | State:" + String(isFallAlarmActive ? "**FALL ALARM**" : (currentDynMag < 0.05 ? "RESTING" : "ACTIVE"));

    printlnBT(dataString);
  }
}
