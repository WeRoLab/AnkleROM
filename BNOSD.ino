#include <Arduino.h>
// This demo explores two reports (SH2_ARVR_STABILIZED_RV and SH2_GYRO_INTEGRATED_RV) both can be used to give 
// quartenion and euler (yaw, pitch roll) angles.  Toggle the FAST_MODE define to see other report.  
// Note sensorValue.status gives calibration accuracy (which improves over time)
#include <Adafruit_BNO08x.h>
#include <Adafruit_TinyUSB.h> // for Serial
#include <SD.h>

// For SPI mode, we need a CS pin
#define BNO08X_CS 10
#define BNO08X_INT 9


// #define FAST_MODE

// For SPI mode, we also need a RESET 
//#define BNO08X_RESET 5
// but not for I2C or UART
#define BNO08X_RESET -1

struct euler_t {
  float yaw;
  float pitch;
  float roll;
} ypr;

Adafruit_BNO08x  bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;
unsigned long start=0;
unsigned long cycles=1;
unsigned long cycle_len=50; // millis
String logFile;
File myFile;

#ifdef FAST_MODE
  // Top frequency is reported to be 1000Hz (but freq is somewhat variable)
  sh2_SensorId_t reportType = SH2_GYRO_INTEGRATED_RV;
  long reportIntervalUs = 2000;
#else
  // Top frequency is about 250Hz but this report is more accurate
  sh2_SensorId_t reportType = SH2_ARVR_STABILIZED_RV;
  long reportIntervalUs = 5000;
#endif
void setReports(sh2_SensorId_t reportType, long report_interval) {
  Serial.println("Setting desired reports");
  if (! bno08x.enableReport(reportType, report_interval)) {
    Serial.println("Could not enable stabilized remote vector");
  }
}

void setup(void) {

  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);
  while (!Serial) delay(10);     // will pause Zero, Leonardo, etc until serial console opens

  // Try to initialize!
  while (!bno08x.begin_I2C()) {
  //if (!bno08x.begin_UART(&Serial1)) {  // Requires a device with > 300 byte UART buffer!
  //if (!bno08x.begin_SPI(BNO08X_CS, BNO08X_INT)) {
    Serial.println("Failed to find BNO08x chip");
    delay(1000);
  }
  Serial.println("BNO08x Found!");


  setReports(reportType, reportIntervalUs);
  Serial.println("Initializing SD Card...");
  while (!SD.begin(10)) {
    Serial.println("initialization failed!");
    delay(1000);
  }
  Serial.println("initialization done.");
  // Find empty file name
    for (int i = 0;; i++) {
        logFile = "IMU" + String(i) + ".txt";
        if (!SD.exists(logFile)) {
        myFile = SD.open(logFile, FILE_WRITE);
        break;
        }
    }
    Serial.print("Logging to: "); Serial.println(logFile);



  Serial.println("Reading events");
  start = millis();
  myFile = SD.open(logFile, FILE_WRITE);
}

void quaternionToEuler(float qr, float qi, float qj, float qk, euler_t* ypr, bool degrees = false) {

    float sqr = sq(qr);
    float sqi = sq(qi);
    float sqj = sq(qj);
    float sqk = sq(qk);

    ypr->yaw = atan2(2.0 * (qi * qj + qk * qr), (sqi - sqj - sqk + sqr));
    ypr->pitch = asin(-2.0 * (qi * qk - qj * qr) / (sqi + sqj + sqk + sqr));
    ypr->roll = atan2(2.0 * (qj * qk + qi * qr), (-sqi - sqj + sqk + sqr));

    if (degrees) {
      ypr->yaw *= RAD_TO_DEG;
      ypr->pitch *= RAD_TO_DEG;
      ypr->roll *= RAD_TO_DEG;
    }
}

void quaternionToEulerRV(sh2_RotationVectorWAcc_t* rotational_vector, euler_t* ypr, bool degrees = false) {
    quaternionToEuler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k, ypr, degrees);
}

void quaternionToEulerGI(sh2_GyroIntegratedRV_t* rotational_vector, euler_t* ypr, bool degrees = false) {
    quaternionToEuler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k, ypr, degrees);
}

void loop() {

  digitalToggle(LED_BUILTIN); // turn the LED on (HIGH is the voltage level)
  if (bno08x.wasReset()) {
    Serial.print("sensor was reset ");
    setReports(reportType, reportIntervalUs);
  }
  
  while (millis()<start+cycles*cycle_len) {
    delay(1);
  }
  unsigned long now2 = millis();
  if (bno08x.getSensorEvent(&sensorValue)) {
    // // in this demo only one report type will be received depending on FAST_MODE define (above)
    // switch (sensorValue.sensorId) {
    //   case SH2_ARVR_STABILIZED_RV:
    //     quaternionToEulerRV(&sensorValue.un.arvrStabilizedRV, &ypr, true);
    //   case SH2_GYRO_INTEGRATED_RV:
    //     // faster (more noise?)
    //     quaternionToEulerGI(&sensorValue.un.gyroIntegratedRV, &ypr, true);
    //     break;
    // }
    // Serial.print(cycles);             Serial.print("\t");
    // Serial.print(sensorValue.status);     Serial.print("\t");  // This is accuracy in the range of 0 to 3
    // Serial.print(ypr.yaw);                Serial.print("\t");
    // Serial.print(ypr.pitch);              Serial.print("\t");
    // Serial.println(ypr.roll);
    float qreal = sensorValue.un.gameRotationVector.real;
    float qi = sensorValue.un.gameRotationVector.i;
    float qj = sensorValue.un.gameRotationVector.j;
    float qk = sensorValue.un.gameRotationVector.k;
    Serial.print(cycles);             Serial.print("\t");
    Serial.print(sensorValue.status);     Serial.print("\t");  // This is accuracy in the range of 0 to 3
    Serial.print(qreal,5);                Serial.print("\t");
    Serial.print(qi,5);              Serial.print("\t");
    Serial.print(qj,5);              Serial.print("\t");
    Serial.print(qk,5);              Serial.print("\t");

    // Write to SD Card
    myFile.print(cycles);             myFile.print(",");
    myFile.print(sensorValue.status);     myFile.print(",");  // This is accuracy in the range of 0 to 3
    myFile.print(qreal,5);                myFile.print(",");
    myFile.print(qi,5);              myFile.print(",");
    myFile.print(qj,5);              myFile.print(",");
    myFile.print(qk,5);              myFile.print(",");
    myFile.println();
    myFile.flush();
  }
  unsigned long now = millis();
  Serial.print(" took "); Serial.print(now-now2); Serial.print("ms"); Serial.print("\t");
    Serial.print("Total Time: "); Serial.print((now-start)/1000); Serial.println("s");;
  cycles+=1;
}
