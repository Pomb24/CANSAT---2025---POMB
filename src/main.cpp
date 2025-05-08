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
HardwareSerial Serial1(PA10, PA9);

GyverBME280 bme;

HardwareSerial Serial2(PA3, PA2);
LoRa_E32 e32ttl(&Serial2, PA0, PB0, PB10);

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

void setup()
{
  Serial.begin(9600);
  Serial1.begin(9600);
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

  sensors_event_t event;
  dht.temperature().getEvent(&event);
  dht.humidity().getEvent(&event);

  while (Serial1.available()) {
    char c = Serial1.read();  // Đọc dữ liệu từ GPS
    gps.encode(c);   // Đưa dữ liệu vào parser
  }

  float pressure = bme.readPressure(); // Đọc áp suất từ BME280 (Pa)
  float temperature = bme.readTemperature(); // Đọc nhiệt độ từ BME280 (°C)
  float hum = bme.readHumidity();
  float altitude = calculateAltitude(pressure);

  if (temperature == 0 && hum == 0 && pressure == 0) {
    Serial.println("Cảm biến mất kết nối!");
    sensorOK = false;
    return;
  }

  if (currentTime - lastDataTime >= samplingInterval) {
    lastDataTime = currentTime;

    // Tạo chuỗi dữ liệu để gửi qua LoRa
    String dataString = "Altitude: " + String(altitude) + " m\n" +
                        "Temperature: " + String(temperature) + " *C\n" +
                        "Humidity: " + String(hum) + " %\n" +
                        "Pressure: " + String(pressure) + " Pa\n";

    if (gps.location.isUpdated()) {
      dataString += "Latitude: " + String(gps.location.lat(), 6) + "\n";
      dataString += "Longitude: " + String(gps.location.lng(), 6) + "\n";
    }

    dataString += "ACCELERO  X: " + String(mpu.getAccX()) + " Y: " + String(mpu.getAccY()) + " Z: " + String(mpu.getAccZ()) + "\n";
    dataString += "GYRO      X: " + String(mpu.getGyroX()) + " Y: " + String(mpu.getGyroY()) + " Z: " + String(mpu.getGyroZ()) + "\n";

    // Gửi dữ liệu qua LoRa
    ResponseStatus rs = e32ttl.sendMessage(dataString);
    if (rs.code == 1) {
      Serial.println("Dữ liệu đã được gửi qua LoRa thành công!");
    } else {
      Serial.print("Lỗi khi gửi dữ liệu qua LoRa: ");
      Serial.println(rs.getResponseDescription());
    }

    // Xuất dữ liệu qua Serial Monitor
    Serial.println(dataString);
  }
}