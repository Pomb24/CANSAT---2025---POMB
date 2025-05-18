#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#include <MPU6050_light.h>
#include <MQUnifiedsensor.h>
#include <GyverBME280.h>
#include <TinyGPS++.h>
#include "LoRa_E32.h"
#include <SoftwareSerial.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <SimpleKalmanFilter.h>

#define LED_PIN      PC13
#define SD_CS_PIN    PB12

#define DUST_LED_PIN PA8 
#define DUST_ANALOG_PIN PA7

#define DHTPIN PA1
#define DHTTYPE DHT22
DHT_Unified dht(DHTPIN, DHTTYPE);

#define placa "STM32"
#define Voltage_Resolution 5
#define pin PA0
#define type "MQ-135"
#define ADC_Bit_Resolution 12
#define RatioMQ135CleanAir 3.6//RS / R0 = 3.6 ppm  
MQUnifiedsensor MQ135(placa, Voltage_Resolution, ADC_Bit_Resolution, pin, type);

MPU6050 mpu(Wire);

TinyGPSPlus gps;
HardwareSerial Gps_Serial(PA10, PA9);

GyverBME280 bme;

HardwareSerial Lora(PA3, PA2);
LoRa_E32 e32ttl(&Lora, PA0, PB0, PB10);

SimpleKalmanFilter kalmanFilterX(2, 2, 0.01); // Q, R, và F
SimpleKalmanFilter kalmanFilterY(2, 2, 0.01);
SimpleKalmanFilter kalmanFilterZ(2, 2, 0.01);

bool sensorOK = false;

bool sdCardOK = false;

const int samplingTime = 280; 
const float sensitivity = 0.005; 
const float vCleanAir = 0.9;  
unsigned long lastDataTime = 0;
const int samplingInterval = 2000;

float calculateAltitude(float pressure, float seaLevelPressure = 101325) {
  // Công thức tính độ cao
  return 44330 * (1 - pow(pressure / seaLevelPressure, 1 / 5.255));
}

float readDustDensity() {
  float voMeasured = 0;
  float calcVoltage = 0;
  float dustDensity = 0;

  digitalWrite(DUST_LED_PIN, LOW);
  delayMicroseconds(samplingTime);
  voMeasured = analogRead(DUST_ANALOG_PIN);
digitalWrite(DUST_LED_PIN, HIGH);
  calcVoltage = voMeasured * (5.0 / 1024.0);
  dustDensity = (calcVoltage - vCleanAir) / sensitivity;
  if (dustDensity < 0) dustDensity = 0;
  return dustDensity;
}

void initBME280() {
  sensorOK = bme.begin();
  if (sensorOK) {
    Serial.println("Đã khởi tạo BME280 thành công!");
    bme.setHumOversampling(MODULE_DISABLE);
    bme.setStandbyTime(STANDBY_1000MS);
  }
  else {
    Serial.println("Không tìm thấy BME280!");
  }
}

void blinkError(int onTime, int offTime, int count) {
  for (int i = 0; i < count; i++) {
    digitalWrite(LED_PIN, LOW);
    delay(onTime);
    digitalWrite(LED_PIN, HIGH);
    delay(offTime);
  }
}

void saveDataToSD(String data) {
  bool newFile = !SD.exists("data.csv");

  File dataFile = SD.open("data.csv", FILE_WRITE);
  if (dataFile) {
    if (newFile) {
      dataFile.println("TEAM_ID,LEN,MSG_ID,TIME,ALTITUDE,TEMP,GPS_TIME,GPS_LAT,GPS_LONG,GPS_ALT,GPS_SAT,ACC_X,ACC_Y,ACC_Z,GYO_X,GYO_Y,GYO_Z,PRESS,HUMI,CO,PM");
    }
    dataFile.println(data);
    dataFile.close();
    Serial.println("Dữ liệu đã được lưu vào data.csv");
  } else {
    Serial.println("Không thể mở file data.csv để ghi!");
  }
}

void setup()
{
  Serial.begin(9600);
  Gps_Serial.begin(9600);
  while (!Serial) {;}
  Wire.begin();

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  pinMode(DUST_LED_PIN, OUTPUT);
  digitalWrite(DUST_LED_PIN, HIGH);
  pinMode(DUST_ANALOG_PIN, INPUT);

  dht.begin();
  sensor_t sensor;
  dht.temperature().getSensor(&sensor);
  dht.humidity().getSensor(&sensor);

  initBME280();

  MQ135.setRegressionMethod(1); //_PPM =  a*ratio^b
  MQ135.setA(605.18); MQ135.setB(-3.937); // Cấu hình phương trình để tính nồng độ CO
  MQ135.init();
  Serial.println("Đang hiệu chuẩn, vui lòng chờ.");
  float calcR0 = 0;
  for(int i = 1; i<=10; i ++)
  {
    MQ135.update();
    calcR0 += MQ135.calibrate(RatioMQ135CleanAir);
    Serial.print(".");
  }
  MQ135.setR0(calcR0/10);
  Serial.println("  Hoàn tất!");
  if(isinf(calcR0)) {Serial.println("Cảnh báo: Lỗi kết nối, R0 là vô hạn (Phát hiện mạch hở), vui lòng kiểm tra dây và nguồn cấp"); while(1);}
  if(calcR0 == 0){Serial.println("Cảnh báo: Lỗi kết nối, R0 bằng 0 (Chân analog bị nối tắt xuống đất), vui lòng kiểm tra dây và nguồn cấp"); while(1);}
  MQ135.serialDebug(true);

  byte status = mpu.begin();
  while(status!=0){};
  mpu.calcOffsets(true,true);

  e32ttl.begin();

  #ifdef SEND_WAKE_UP_MESSAGE
    e32ttl.setMode(MODE_1_WAKE_UP);
  #endif
}

void loop()
{
  unsigned long currentTime = millis();

  mpu.update(); // Update MPU6050 data
  float accX = kalmanFilterX.updateEstimate(mpu.getAccX());
  float accY = kalmanFilterY.updateEstimate(mpu.getAccY());
  float accZ = kalmanFilterZ.updateEstimate(mpu.getAccZ());

  sensors_event_t event;
  dht.temperature().getEvent(&event);
  dht.humidity().getEvent(&event);

  while (Gps_Serial.available()) {
    char c = Gps_Serial.read();  // Đọc dữ liệu từ GPS
    gps.encode(c);   // Đưa dữ liệu vào parser
  }

  float pressure = bme.readPressure(); // (Pa)
  float temperature = bme.readTemperature(); // (°C)
  float hum = bme.readHumidity();
  float altitude = calculateAltitude(pressure);

  if (temperature == 0 && hum == 0 && pressure == 0) {
    Serial.println("Cảm biến mất kết nối!");
    sensorOK = false;
    return;
  }

  if (currentTime - lastDataTime >= samplingInterval) {
    lastDataTime = currentTime;

    MQ135.update(); // Cập nhật giá trị cảm biến
    float coConcentration = MQ135.readSensor(); // Đọc nồng độ CO (ppm)
    float dust = readDustDensity(); // Đọc bụi mịn

    // Định danh
    static int messageCounter = 1;
    String teamID = "TEAM_01";
    String msgID = String(messageCounter++);

    // GPS data an toàn
    String gpsTime = String(gps.time.hour()) + ":" + String(gps.time.minute()) + ":" + String(gps.time.second());
    String lat = gps.location.isValid() ? String(gps.location.lat(), 4) : "0.0000";
    String lng = gps.location.isValid() ? String(gps.location.lng(), 4) : "0.0000";
    String gpsAlt = gps.altitude.isValid() ? String(gps.altitude.meters(), 1) : "0.0";
    String gpsSats = gps.satellites.isValid() ? String(gps.satellites.value()) : "0";

    // Payload CSV-style
    String dataString = "";
    dataString += teamID + ",";
    dataString += String(18) + ",";
    dataString += msgID + ",";
    dataString += String(millis()) + ",";
    dataString += String(altitude, 1) + ",";
    dataString += String(temperature, 2) + ",";
    dataString += gpsTime + ",";
    dataString += lat + "," + lng + "," + gpsAlt + "," + gpsSats + ",";
    dataString += String(accX, 2) + "," + String(accY, 2) + "," + String(accZ, 2) + ",";
    dataString += String(mpu.getGyroX(), 2) + "," + String(mpu.getGyroY(), 2) + "," + String(mpu.getGyroZ(), 2) + ",";
    dataString += String(pressure, 2) + ",";
    dataString += String(hum, 2) + ",";
    dataString += String(coConcentration, 2) + ",";
    dataString += String(dust, 2) + ",";

    // Gửi qua LoRa
    ResponseStatus rs = e32ttl.sendMessage(dataString);
    if (rs.code == 1) {
      Serial.println("Dữ liệu đã được gửi qua LoRa thành công!");
    } else {
      Serial.print("Lỗi khi gửi dữ liệu qua LoRa: ");
      Serial.println(rs.getResponseDescription());
    }

    // Lưu vào thẻ nhớ
    saveDataToSD(dataString);

    // In ra Serial
    Serial.println(dataString);
  }
}
