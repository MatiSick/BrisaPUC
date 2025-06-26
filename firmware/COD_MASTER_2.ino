// ————— Código Maestro con Veleta de 8 Direcciones —————

#include <Wire.h>
#include <MPU9250.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <TinyGPS++.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Adafruit_SHT31.h>
#include <ArduinoJson.h>


// ————— I2C y PCA9548A —————
#define SDA_PIN             21
#define SCL_PIN             22
#define PCA9548A_ADDR       0x70
#define MPU_CHANNEL         5 //2
#define BME_CHANNEL         7
#define SHT_CHANNEL         6 //4

// ————— MPU9250 —————
#define MPU_ADDRESS         0x68

// ————— BME280 —————
#define SEALEVELPRESSURE_HPA 1015

// ————— GPS (UART2) —————
#define GPS_BAUD            9600
#define GPS_RX_PIN          16
#define GPS_TX_PIN          17

// ————— DO Sensor —————
#define DO_PIN              32 //34
#define DO_VREF             3300
#define DO_ADC_RES          4095

// ————— TDS Sensor —————
#define TDS_PIN             34 //27
#define TDS_VREF            3.3f
#define TDS_SCOUNT          30

// ————— DS18B20 —————
#define ONEWIRE_PIN         2 //15 // todo: cambiar este pin. Al estar conectado, no deja cambiar firmware.

// ————— Anemómetro —————
#define ANEMO_PIN           13//14
#define ANEMO_VREF          3.3f
#define ANEMO_ADC_RES       4095
#define ANEMO_MIN_VOLTAGE   0.0075f
#define ANEMO_MAX_VOLTAGE   3.3f
#define ANEMO_MAX_WIND_SPEED 60.0f
#define MPS_TO_KMH          3.6f
#define MPS_TO_MPH          2.23694f

// ————— Veleta 8 Direcciones —————
#define VANE_PIN            12
#define VANE_VREF           3.3f
#define VANE_ADC_RES        4095
#define NUM_DIRECTIONS      8
const char* DIRECTIONS[NUM_DIRECTIONS] = {
  "N","NE","E","SE","S","SW","W","NW"
};

// PH
#define PH_PIN   27 //4           
int buf[10];
int temp = 0;
unsigned long inValue;

// ————— JSON —————
#define TXD1 19
#define RXD1 18
HardwareSerial mySerialJSON(1);
const long SERIAL_BAUD_RATE = 115200;
const int SEND_INTERVAL_MS  = 5000;
const float VALPARAISO_LAT_BASE = -33.0472f;
const float VALPARAISO_LON_BASE = -71.6127f;



// ————— Tabla O₂ (0–40 °C) —————
const uint16_t DO_Table[41] = {
 14460,14220,13820,13440,13090,12740,12420,12110,11810,11530,
 11260,11010,10770,10530,10300,10080,9860,9660,9460,9270,
 9080,8900,8730,8570,8410,8250,8110,7960,7820,7690,
 7560,7430,7300,7180,7070,6950,6840,6730,6630,6530,6410
};

// ————— Objetos Globales —————
MPU9250            mpu;
Adafruit_BME280    bme;
TinyGPSPlus        gps;
HardwareSerial     SerialGPS(2);
OneWire            oneWire(ONEWIRE_PIN);
DallasTemperature  dsSensor(&oneWire);
Adafruit_SHT31     sht31;

// --- Sensor reading variables ---
float mpu_ax, mpu_ay, mpu_az;
float mpu_gx, mpu_gy, mpu_gz;
float mpu_mx, mpu_my, mpu_mz;
float mpu_roll, mpu_pitch, mpu_yaw;
float bme_temp, bme_hum, bme_pres;
float gps_lat, gps_lon, gps_spd;
float do_mgl;
float tds_ppm;
float water_temp;
float air_temp, air_hum;
float wind_mps, wind_kmh, wind_mph;
String wind_dir;
float wind_dir_deg;
float ph; // pH sensor PLACEHOLDER CAMBIAR MATÍAS MORENO
float PHVol;


// ————— Prototipos —————
void selectMuxChannel(uint8_t ch);

// MPU
void initMPU();
void readMPU9250();
void printSensorData();
void printCalibration();

// BME
void initBME();
void readBME280();

// GPS
void initGPS();
void readGPS();

// DO
void initDO();
void readDOSensor();
int16_t readDOcalc(uint32_t voltage_mv, uint8_t temperature_c);

// TDS
void initTDS();
void readTDSSensor();
int getMedianNum(int bArray[], int iFilterLen);

// DS18B20
void initDS18B20();
void readDS18B20Sensor();

// SHT31
void initSHT31();
void readSHT31Sensor();

// Anemómetro
void initAnemometer();
void readAnemometer();

// Veleta
void initWindVane();
void readWindVane();

// JSON
void readSensors();
void sendMergedJSON();

// pH
void readPHSensor(); 

// ————— Setup y Loop —————
void setup() {
  Serial.begin(115200);
  delay(2000);
  Wire.begin(SDA_PIN, SCL_PIN);

  initMPU();
  initBME();
  initGPS();
  initDO();
  initTDS();
  initDS18B20();
  initSHT31();
  initAnemometer();
  initWindVane();
  initJSON();

}

void loop() {
  unsigned long now = millis();
  static unsigned long tFast, tSlow;

  // ~5000 ms
  if (now - tSlow >= 5000) {
    readSensors(); // falta de delay entre readSensors y sendMeredJSON() puede provocar race conditions.
    sendMergedJSON();
    tSlow = now;
  }
}
// ————— Implementaciones —————

// PCA9548A
void selectMuxChannel(uint8_t ch) {
  Wire.beginTransmission(PCA9548A_ADDR);
  Wire.write(1 << ch);
  Wire.endTransmission();
  delay(5);
}

// MPU9250
void initMPU() {
  selectMuxChannel(MPU_CHANNEL);
  Serial.println("Inicializando MPU9250...");
  if (!mpu.setup(MPU_ADDRESS)) while(1) { Serial.println("Error MPU"); delay(5000); }
  mpu.verbose(true); delay(5000); mpu.calibrateAccelGyro();
  delay(5000); mpu.calibrateMag(); mpu.verbose(false);
  printCalibration();
}
void readMPU9250() {
  selectMuxChannel(MPU_CHANNEL);
  if (mpu.update()) {
    // Acelerómetro
    mpu_ax = mpu.getAccX() / float(MPU9250::CALIB_ACCEL_SENSITIVITY);
    mpu_ay = mpu.getAccY() / float(MPU9250::CALIB_ACCEL_SENSITIVITY);
    mpu_az = mpu.getAccZ() / float(MPU9250::CALIB_ACCEL_SENSITIVITY);
    // Giroscopio
    mpu_gx = mpu.getGyroX() / float(MPU9250::CALIB_GYRO_SENSITIVITY);
    mpu_gy = mpu.getGyroY() / float(MPU9250::CALIB_GYRO_SENSITIVITY);
    mpu_gz = mpu.getGyroZ() / float(MPU9250::CALIB_GYRO_SENSITIVITY);
    // Magnetómetro
    mpu_mx = mpu.getMagX();
    mpu_my = mpu.getMagY();
    mpu_mz = mpu.getMagZ();
    // Ángulos
    mpu_roll  = mpu.getRoll();
    mpu_pitch = mpu.getPitch();
    mpu_yaw   = mpu.getYaw();
    //Serial.printf("Accel[g] X:%.3f Y:%.3f Z:%.3f\n", mpu_ax, mpu_ay, mpu_az);
    //Serial.printf("Gyro[°/s] X:%.3f Y:%.3f Z:%.3f\n", mpu_gx, mpu_gy, mpu_gz);
    //Serial.printf("Mag[mG]   X:%.1f Y:%.1f Z:%.1f\n", mpu_mx, mpu_my, mpu_mz);
    //Serial.printf("Angles[°] R:%.1f P:%.1f Y:%.1f\n", mpu_roll, mpu_pitch, mpu_yaw);
  }
}
void printSensorData() {
  float ax=mpu.getAccX()/float(MPU9250::CALIB_ACCEL_SENSITIVITY),
        ay=mpu.getAccY()/float(MPU9250::CALIB_ACCEL_SENSITIVITY),
        az=mpu.getAccZ()/float(MPU9250::CALIB_ACCEL_SENSITIVITY);
  Serial.printf("Accel[g] X:%.3f Y:%.3f Z:%.3f\n", ax, ay, az);
  float gx=mpu.getGyroX()/float(MPU9250::CALIB_GYRO_SENSITIVITY),
        gy=mpu.getGyroY()/float(MPU9250::CALIB_GYRO_SENSITIVITY),
        gz=mpu.getGyroZ()/float(MPU9250::CALIB_GYRO_SENSITIVITY);
  Serial.printf("Gyro[°/s] X:%.3f Y:%.3f Z:%.3f\n", gx, gy, gz);
  float mx=mpu.getMagX(), my=mpu.getMagY(), mz=mpu.getMagZ();
  Serial.printf("Mag[mG] X:%.1f Y:%.1f Z:%.1f\n", mx, my, mz);
  float roll=mpu.getRoll(), pitch=mpu.getPitch(), yaw=mpu.getYaw();
  Serial.printf("Angles[°] R:%.1f P:%.1f Y:%.1f\n\n", roll, pitch, yaw);
}
void printCalibration() {
  Serial.printf("Accel bias[g] X:%.3f Y:%.3f Z:%.3f\n",
    mpu.getAccBiasX()/float(MPU9250::CALIB_ACCEL_SENSITIVITY),
    mpu.getAccBiasY()/float(MPU9250::CALIB_ACCEL_SENSITIVITY),
    mpu.getAccBiasZ()/float(MPU9250::CALIB_ACCEL_SENSITIVITY));
}

// BME280
void initBME() {
  selectMuxChannel(BME_CHANNEL);
  Serial.println("Inicializando BME280...");
  if (!bme.begin(0x76)) while(1) { Serial.println("Error BME"); delay(5000); }
}
void readBME280() {
  selectMuxChannel(BME_CHANNEL);
  bme_temp = bme.readTemperature();
  bme_hum  = bme.readHumidity();
  bme_pres = bme.readPressure() / 100.0F;
  //Serial.printf("BME T=%.2f°C H=%.2f%% P=%.2f hPa\n", bme_temp, bme_hum, bme_pres);
}

// GPS
void initGPS() {
  SerialGPS.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
}
void readGPS() {
  while (SerialGPS.available()) gps.encode(SerialGPS.read());
  if (gps.location.isUpdated()) {
    gps_lat = gps.location.lat();
    gps_lon = gps.location.lng();
    gps_spd = gps.speed.kmph();
    //Serial.printf("GPS Lat=%.6f Lon=%.6f Spd=%.2fkm/h\n", gps_lat, gps_lon, gps_spd);
  }
}

// DO Sensor
void initDO() {
  analogReadResolution(12);
  analogSetPinAttenuation(DO_PIN, ADC_11db);
}
void readDOSensor() {
  uint16_t raw = analogRead(DO_PIN);
  uint32_t mv  = uint32_t(DO_VREF) * raw / DO_ADC_RES;
  do_mgl = readDOcalc(mv, 20); // reemplaza 20 con temperatura real si tienes
  //Serial.printf("DO=%.2f mg/L\n", do_mgl);
}
int16_t readDOcalc(uint32_t v, uint8_t t) {
  uint32_t Vsat = 406 + 35UL*t - 35UL*20;
  return v * DO_Table[t] / Vsat;
}

// TDS Sensor
void initTDS() {
  analogSetPinAttenuation(TDS_PIN, ADC_11db);
}
void readTDSSensor() {
  int buf[TDS_SCOUNT];
  for (int i = 0; i < TDS_SCOUNT; i++) {
    buf[i] = analogRead(TDS_PIN);
    delay(40);
  }
  int med = getMedianNum(buf, TDS_SCOUNT);
  float avgV = med * TDS_VREF / float(DO_ADC_RES);
  tds_ppm = (133.42 * avgV*avgV*avgV
           - 255.86 * avgV*avgV
           + 857.39 * avgV) * 0.5;
  //Serial.printf("TDS=%.0f ppm\n", tds_ppm);
}
int getMedianNum(int bArray[], int len) {
  int bTab[len]; memcpy(bTab,bArray,sizeof(bTab));
  for(int i=0;i<len-1;i++) for(int j=0;j<len-i-1;j++)
    if(bTab[j]>bTab[j+1]) std::swap(bTab[j],bTab[j+1]);
  return (len&1)?bTab[len/2]:(bTab[len/2]+bTab[len/2-1])/2;
}

// DS18B20
void initDS18B20() { dsSensor.begin(); }
void readDS18B20Sensor() {
  dsSensor.requestTemperatures();
  water_temp = dsSensor.getTempCByIndex(0);
  //Serial.printf("Water Temp=%.2f°C\n", water_temp);
}

// SHT31
void initSHT31() {
  selectMuxChannel(SHT_CHANNEL);
  if (!sht31.begin(0x44)) while(1) { Serial.println("Error SHT31"); delay(5000); }
}
void readSHT31Sensor() {
  selectMuxChannel(SHT_CHANNEL);
  air_temp = sht31.readTemperature();
  air_hum  = sht31.readHumidity();
  //Serial.printf("Air Temp=%.2f°C Hum=%.2f%%\n", air_temp, air_hum);
}

// Anemómetro
void initAnemometer() {
  Serial.println("Init Anemometer...");
  analogReadResolution(12);
  analogSetPinAttenuation(ANEMO_PIN, ADC_11db);
}
void readAnemometer() {
  int adc      = analogRead(ANEMO_PIN);
  float voltage = (adc / float(ANEMO_ADC_RES)) * ANEMO_VREF;
  voltage = constrain(voltage, ANEMO_MIN_VOLTAGE, ANEMO_MAX_VOLTAGE);
  wind_mps = ((voltage - ANEMO_MIN_VOLTAGE)
            / (ANEMO_MAX_VOLTAGE - ANEMO_MIN_VOLTAGE))
            * ANEMO_MAX_WIND_SPEED;
  wind_kmh = wind_mps * MPS_TO_KMH;
  wind_mph = wind_mps * MPS_TO_MPH;
  //Serial.printf("Wind: %.2f m/s, %.2f km/h, %.2f mph\n", wind_mps, wind_kmh, wind_mph);
}

// Veleta de Dirección
void initWindVane() {
  Serial.println("Inicializando veleta de dirección...");
  analogReadResolution(12);
  analogSetPinAttenuation(VANE_PIN, ADC_11db);
}

void readWindVane() {
  int adcValue = analogRead(VANE_PIN);
  float voltage = (adcValue / float(VANE_ADC_RES)) * VANE_VREF;
  voltage = constrain(voltage, 0.0f, VANE_VREF);

  int index = int(voltage / (VANE_VREF / NUM_DIRECTIONS));
  index = constrain(index, 0, NUM_DIRECTIONS - 1);

  wind_dir     = DIRECTIONS[index];
  wind_dir_deg = index * (360.0f / NUM_DIRECTIONS);
  //Serial.printf("Dir: %s (%.0f°)\n", wind_dir.c_str(), wind_dir_deg);
}


void readPhSensor() {
  int buf[10];
  unsigned long inValue = 0;
  
  // Toma 10 muestras
  for(int i = 0; i < 10; i++) {
    buf[i] = analogRead(PH_PIN);
    delay(10);
  }
  
  // Ordenamiento burbuja mejorado
  for(int i = 0; i < 9; i++) {
    for(int j = 0; j < 9 - i; j++) {
      if(buf[j] > buf[j+1]) {
        int temp = buf[j];
        buf[j] = buf[j+1];
        buf[j+1] = temp;
      }
    }
  }
  
  // Promedia las 6 muestras centrales
  for(int i = 2; i < 8; i++) {
    inValue += buf[i];
  }
  
  // Conversión a voltaje (mV) y cálculo de pH
  PHVol = (float)inValue * 3300.0 / (4095.0 * 6) + 200; // +200 hardcodeado según pruebas.
  ph = PHVol * -0.00718 + 26.6 ;  // Actualiza la variable global
}


// JSON
void initJSON() {
  mySerialJSON.begin(SERIAL_BAUD_RATE, SERIAL_8N1, RXD1, TXD1);
  while (!mySerialJSON) {;}
}


void readSensors() {
  readMPU9250();            // accel, gyro, mag, angles
  readGPS();                // gps_lat, gps_lon, gps_spd
  readAnemometer();         // wind_mps, wind_kmh, wind_mph
  readWindVane();           // wind_dir, wind_dir_deg
  readTDSSensor();          // tds_ppm
  readSHT31Sensor();        // air_temp, air_hum
  readDOSensor();           // do_mgl
  readBME280();             // bme_temp, bme_hum, bme_pres
  readDS18B20Sensor();      // water_temp
  readPhSensor();           // PLACEHOLDER PH CAMBIAR


  // SIMULACION
  // Inerciales
  // mpu_ax = random(-200, 201) / 100.0f;
  // mpu_ay = random(-200, 201) / 100.0f;
  // mpu_az = random(880, 1081) / 100.0f;
  // mpu_gx = random(-3000, 3001) / 100.0f;
  // mpu_gy = random(-3000, 3001) / 100.0f;
  // mpu_gz = random(-3000, 3001) / 100.0f;
  // mpu_mx = random(-100, 101);
  // mpu_my = random(-100, 101);
  // mpu_mz = random(-100, 101);
  // mpu_roll  = random(-18000, 18001) / 100.0f;
  // mpu_pitch = random(-9000, 9001) / 100.0f;
  // mpu_yaw   = random(0, 36000) / 100.0f;

  // // GPS
  // gps_lat = VALPARAISO_LAT_BASE + random(-50, 51)/100000.0f;
  // gps_lon = VALPARAISO_LON_BASE + random(-50, 51)/100000.0f;
  // gps_spd = random(0, 31) / 10.0f;

  // // Viento
  // wind_mps = random(0, 251) / 10.0f;
  // wind_kmh = wind_mps * MPS_TO_KMH;
  // wind_mph = wind_mps * MPS_TO_MPH;
  // wind_dir_deg = random(0, 360);

  // // Ambientales
  // air_temp = random(100, 351) / 10.0f;
  // air_hum  = random(300, 901) / 10.0f;
  // bme_temp = random(100, 351) / 10.0f;
  // bme_hum  = random(300, 901) / 10.0f;
  // bme_pres = random(9800, 10301) / 10.0f;

  // // Calidad del Agua
  // do_mgl     = random(50, 101) / 10.0f;
  // tds_ppm    = random(300, 1201);
  // water_temp = random(100, 201) / 10.0f;
  //ph = random(65, 86) / 10.0f; //Para pruebas
}

void sendMergedJSON() {
  // Tamaño calculado: JSON_OBJECT_SIZE(5) + 5*JSON_ARRAY_SIZE(3) = 152 bytes
  StaticJsonDocument<220> j;
  
  // Grupo 1: Inerciales (a)
  JsonArray a = j.createNestedArray("a");
  a.add(static_cast<long>(mpu_ax * 100000));
  a.add(static_cast<long>(mpu_ay * 100000));
  a.add(static_cast<long>(mpu_az * 100000));

  // Grupo 2: Orientación (o)
  JsonArray o = j.createNestedArray("o");
  o.add(static_cast<long>(mpu_roll * 100000));
  o.add(static_cast<long>(mpu_pitch * 100000));
  o.add(static_cast<long>(mpu_yaw * 100000));

  // Grupo 3: Posición (p)
  JsonArray p = j.createNestedArray("p");
  p.add(static_cast<long>(gps_lat * 100000));
  p.add(static_cast<long>(gps_lon * 100000));
  p.add(static_cast<long>(gps_spd * 100000));

  // Grupo 4: Ambiente (e)
  JsonArray e = j.createNestedArray("e");
  e.add(static_cast<long>(bme_temp * 100000));
  e.add(static_cast<long>(bme_hum * 100000));
  e.add(static_cast<long>(bme_pres * 100000));
  e.add(static_cast<long>(air_temp * 100000));
  e.add(static_cast<long>(air_hum * 100000));

  // Grupo 5: Agua (w)
  JsonArray w = j.createNestedArray("w");
  w.add(static_cast<long>(wind_mps * 100000));
  w.add(static_cast<int>(wind_dir_deg));
  w.add(static_cast<long>(do_mgl * 100000));
  w.add(static_cast<long>(tds_ppm));
  w.add(static_cast<long>(water_temp * 100000));
  w.add(static_cast<long>(ph * 100000));
  
  serializeJson(j, mySerialJSON);
  mySerialJSON.println();
  Serial.println("Merged JSON sent");
  //Serial.println(PHVol);
  //Serial.println(ph);
}

