#include <ArduinoJson.h>

// ————— Configuración JSON —————
#define TXD1 19
#define RXD1 18
HardwareSerial mySerialJSON(1);
const long SERIAL_BAUD_RATE = 115200;
const int SEND_INTERVAL_MS  = 5000;
const float VALPARAISO_LAT_BASE = -33.0472f;
const float VALPARAISO_LON_BASE = -71.6127f;

// ———— Constantes de conversión ————
#define MPS_TO_KMH          3.6f
#define MPS_TO_MPH          2.23694f

// ———— Variables globales simuladas ————
// Inerciales
float mpu_ax, mpu_ay, mpu_az;
float mpu_gx, mpu_gy, mpu_gz;
float mpu_mx, mpu_my, mpu_mz;
float mpu_roll, mpu_pitch, mpu_yaw;

// Ambientales
float bme_temp, bme_hum, bme_pres;
float air_temp, air_hum;

// GPS
float gps_lat, gps_lon, gps_spd;

// Agua
float do_mgl;
float tds_ppm;
float water_temp;
float ph;

// Viento
float wind_mps, wind_kmh, wind_mph;
float wind_dir_deg;

// ———— Prototipos ————
void initJSON();
void readSensors();
void sendMergedJSON();

// ———— Setup y Loop —————
void setup() {
  Serial.begin(115200);
  delay(2000);
  initJSON();
  randomSeed(analogRead(0));  // Inicializar generador aleatorio
}

void loop() {
  static unsigned long lastSend = 0;
  
  if (millis() - lastSend >= SEND_INTERVAL_MS) {
    readSensors();
    sendMergedJSON();
    lastSend = millis();
  }
}

// ———— Implementaciones ————
void initJSON() {
  mySerialJSON.begin(SERIAL_BAUD_RATE, SERIAL_8N1, RXD1, TXD1);
  while (!mySerialJSON) {;}
}

void readSensors() {
  // Inerciales
  mpu_ax = random(-200, 201) / 100.0f;
  mpu_ay = random(-200, 201) / 100.0f;
  mpu_az = random(880, 1081) / 100.0f;
  mpu_gx = random(-3000, 3001) / 100.0f;
  mpu_gy = random(-3000, 3001) / 100.0f;
  mpu_gz = random(-3000, 3001) / 100.0f;
  mpu_mx = random(-100, 101);
  mpu_my = random(-100, 101);
  mpu_mz = random(-100, 101);
  mpu_roll  = random(-18000, 18001) / 100.0f;
  mpu_pitch = random(-9000, 9001) / 100.0f;
  mpu_yaw   = random(0, 36000) / 100.0f;

  // GPS
  gps_lat = VALPARAISO_LAT_BASE + random(-50, 51)/100000.0f;
  gps_lon = VALPARAISO_LON_BASE + random(-50, 51)/100000.0f;
  gps_spd = random(0, 31) / 10.0f;

  // Viento
  wind_mps = random(0, 251) / 10.0f;
  wind_kmh = wind_mps * MPS_TO_KMH;
  wind_mph = wind_mps * MPS_TO_MPH;
  wind_dir_deg = random(0, 360);

  // Ambientales
  air_temp = random(100, 351) / 10.0f;
  air_hum  = random(300, 901) / 10.0f;
  bme_temp = random(100, 351) / 10.0f;
  bme_hum  = random(300, 901) / 10.0f;
  bme_pres = random(9800, 10301) / 10.0f;

  // Calidad del Agua
  do_mgl     = random(50, 101) / 10.0f;
  tds_ppm    = random(300, 1201);
  water_temp = random(100, 201) / 10.0f;
  ph         = random(65, 86) / 10.0f;
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
}