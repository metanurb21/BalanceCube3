/*
  ESP32 Balance cube based on REMRC: https://github.com/remrc/Self-Balancing-Cube/blob/main/README.md
  Some refactoring made to update for my needs, and BLE connection so it can work with my iPhone.
*/
#include <Arduino.h>
#include "setup.h"
#include <Wire.h>
#include <EEPROM.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <Adafruit_NeoPixel.h>
#include "Tone32.h"

#define DEVICE_NAME "ESP32_BLE_Cube"
#define SERVICE_UUID "0000FF00-0000-1000-8000-00805F9B34FB"
#define CHAR_UUID "0000FF01-0000-1000-8000-00805F9B34FB"

// BLE characteristic to receive data
BLECharacteristic *pCharacteristic;

// Lights
Adafruit_NeoPixel pixels(NUMPIXELS, neoPixelPin, NEO_GRB + NEO_KHZ800);

// Sate machine
VerticalState currentState = VERTICAL_UNKNOWN;

// Callback class to handle incoming data
class MyCallbacks : public BLECharacteristicCallbacks
{
  void onWrite(BLECharacteristic *pCharacteristic)
  {
    std::string receivedData = pCharacteristic->getValue();
    if (receivedData.length() > 0)
    {
      String command = String(receivedData.c_str());
      command.trim();
      // Serial.print("Received: ");
      Serial.println(command);
      if (command.equals("c") && !calibrating)
      {
        calibrating = true;
        Serial.println("Calibrating on.");
        Serial.println("Set the cube on vertex...");
        for (int i = 0; i < NUMPIXELS; i++)
        {
          pixels.setPixelColor(i, 0, 0, 255);
          pixels.setBrightness(255);
          pixels.show();
          delay(100);
        }
        playNotes(4186, 4699, 5274, 100);
        delay(1000);
        pixels.clear();
      }
      if (command.equals("o") && calibrating)
      {
        Serial.println("X: ");
        Serial.println(AcX);
        Serial.println(" Y: ");
        Serial.println(AcY);
        Serial.println(" Z: ");
        Serial.println(AcZ + 16384);
        if (abs(AcX) < 2000 && abs(AcY) < 2000)
        {
          offsets.ID = 96;
          offsets.acXv = AcX;
          offsets.acYv = AcY;
          offsets.acZv = AcZ + 16384;
          Serial.println("Vertex OK.");
          Serial.println("Set the cube on edge...");
          tone(BUZZER, freq + 800, dure / 3, channel);
          vertex_calibrated = true;
          for (int i = 0; i < NUMPIXELS; i++)
          {
            pixels.setPixelColor(i, 0, 255, 0);
            pixels.setBrightness(255);
            pixels.show();
            delay(100);
          }
          playNotes(4186, 4699, 5274, 100);
          delay(1000);
          pixels.clear();
          turnoff_pixel();
        }
        else if (abs(AcX) > 7000 && abs(AcX) < 10000 && abs(AcY) < 2000 && vertex_calibrated)
        {
          Serial.println("X: ");
          Serial.println(AcX);
          Serial.println(" Y: ");
          Serial.println(AcY);
          Serial.println(" Z: ");
          Serial.println(AcZ + 16384);
          Serial.println("Edge OK.");
          offsets.acXe = AcX;
          offsets.acYe = AcY;
          offsets.acZe = AcZ + 16384;
          tone(BUZZER, freq + 800, dure / 3, channel);
          delay(100);
          tone(BUZZER, freq + 1000, dure / 3, channel);
          vertex_calibrated = true;
          for (int i = 0; i < NUMPIXELS; i++)
          {
            pixels.setPixelColor(i, 128, 128, 128);
            pixels.setBrightness(255);
            pixels.show();
            delay(100);
          }
          turnoff_pixel();
          for (int i = 0; i < NUMPIXELS; i++)
          {
            pixels.setPixelColor(i, 128, 128, 128);
            pixels.setBrightness(255);
            pixels.show();
            delay(100);
          }
          delay(1000);
          turnoff_pixel();
          pixels.clear();
          save();
        }
        else
        {
          Serial.println("X: ");
          Serial.println(AcX);
          Serial.println(" Y: ");
          Serial.println(AcY);
          Serial.println(" Z: ");
          Serial.println(AcZ + 16384);
          Serial.println("The angles are wrong!!!");
          for (int i = 0; i < NUMPIXELS; i++)
          {
            pixels.setPixelColor(i, 255, 0, 0);
            pixels.setBrightness(255);
            pixels.show();
            delay(100);
          }
          turnoff_pixel();
          for (int i = 0; i < NUMPIXELS; i++)
          {
            pixels.setPixelColor(i, 255, 0, 0);
            pixels.setBrightness(255);
            pixels.show();
            delay(100);
          }
          delay(1000);
          turnoff_pixel();
          playNotes(4186, 4186, 4186, 100);
          delay(300);
        }
      }
    }
  }
};

void setup()
{
  Serial.begin(115200);
  pixels.begin(); // INITIALIZE NeoPixel strip object (REQUIRED)
  pixels.clear(); // Set all pixel colors to 'off'
  // Initialize BLE
  BLEDevice::init(DEVICE_NAME);

  // Create BLE Server
  BLEServer *pServer = BLEDevice::createServer();

  // Create BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create BLE Characteristic
  pCharacteristic = pService->createCharacteristic(
      CHAR_UUID,
      BLECharacteristic::PROPERTY_WRITE);

  // Set callback to handle data reception
  pCharacteristic->setCallbacks(new MyCallbacks());

  // Start the service
  pService->start();

  // Start advertising
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->start();

  Serial.println("BLE Server Started...");
  // SerialBT.begin("ESP32-Cube"); // Bluetooth device name
  EEPROM.begin(EEPROM_SIZE);

  // FastLED.addLeds<WS2812B, LED_PIN, RGB>(leds, NUM_PIXELS); // GRB ordering is typical

  pinMode(BUZZER, OUTPUT);
  pinMode(BRAKE, OUTPUT);
  digitalWrite(BRAKE, HIGH);

  for (int i = 0; i < NUMPIXELS; i++)
  {
    pixels.setPixelColor(i, 0, 0, 255);
    pixels.setBrightness(255);
    pixels.show();
    delay(100);
  }
  turnoff_pixel();
  for (int i = NUMPIXELS - 1; i > 0; i--)
  {
    pixels.setPixelColor(i, 0, 0, 255);
    pixels.setBrightness(255);
    pixels.show();
    delay(100);
  }
  turnoff_pixel();
  for (int i = 0; i < NUMPIXELS; i++)
  {
    pixels.setPixelColor(i, 0, 255, 0);
    pixels.setBrightness(255);
    pixels.show();
    delay(100);
  }
  turnoff_pixel();

  pinMode(DIR1, OUTPUT);
  pinMode(ENC1_1, INPUT);
  pinMode(ENC1_2, INPUT);
  attachInterrupt(ENC1_1, ENC1_READ, CHANGE);
  attachInterrupt(ENC1_2, ENC1_READ, CHANGE);
  ledcSetup(PWM1_CH, BASE_FREQ, TIMER_BIT);
  ledcAttachPin(PWM1, PWM1_CH);
  Motor_control(1, 0, motor1_speed, DIR1, PWM1_CH);

  pinMode(DIR2, OUTPUT);
  pinMode(ENC2_1, INPUT);
  pinMode(ENC2_2, INPUT);
  attachInterrupt(ENC2_1, ENC2_READ, CHANGE);
  attachInterrupt(ENC2_2, ENC2_READ, CHANGE);
  ledcSetup(PWM2_CH, BASE_FREQ, TIMER_BIT);
  ledcAttachPin(PWM2, PWM2_CH);
  Motor_control(2, 0, motor2_speed, DIR2, PWM2_CH);

  pinMode(DIR3, OUTPUT);
  pinMode(ENC3_1, INPUT);
  pinMode(ENC3_2, INPUT);
  attachInterrupt(ENC3_1, ENC3_READ, CHANGE);
  attachInterrupt(ENC3_2, ENC3_READ, CHANGE);
  ledcSetup(PWM3_CH, BASE_FREQ, TIMER_BIT);
  ledcAttachPin(PWM3, PWM3_CH);
  Motor_control(3, 0, motor3_speed, DIR3, PWM3_CH);

  EEPROM.get(0, offsets);
  if (offsets.ID == 96)
  {
    calibrated = true;
  }
  playNotes(4186, 4699, 5274, 100);
  angle_setup();
}

void loop()
{
  currentT = millis();

  // Handle angle calculations and motor speed updates
  if (currentT - previousT_1 >= loop_time)
  {
    updateMotorSpeeds();
    handleCurrentState();
    previousT_1 = currentT;
  }

  // Handle battery voltage and LED calibration indication
  if (currentT - previousT_2 >= 2000)
  {
    updateBatteryVoltage();
    handleCalibrationIndication();
    previousT_2 = currentT;
  }
}

void updateMotorSpeeds()
{
  angle_calc();
  motor1_speed = enc_count1;
  enc_count1 = 0;
  motor2_speed = enc_count2;
  enc_count2 = 0;
  motor3_speed = enc_count3;
  enc_count3 = 0;
  threeWay_to_XY(motor1_speed, motor2_speed, motor3_speed);
  motors_speed_Z = motor1_speed + motor2_speed + motor3_speed;
}

void handleCurrentState()
{
  if (currentState == VERTICAL_VERTEX && calibrated && !calibrating)
  {
    handleVerticalVertexState();
  }
  else if (currentState == VERTICAL_EDGE && calibrated && !calibrating)
  {
    handleVerticalEdgeState();
  }
  else
  {
    XYZ_to_threeWay(0, 0, 0);
    digitalWrite(BRAKE, LOW);
    motors_speed_X = 0;
    motors_speed_Y = 0;
  }
}

void updateBatteryVoltage()
{
  battVoltage((double)analogRead(VBAT) / 204); // value 204 must be selected by measuring battery voltage!
}

void handleCalibrationIndication()
{
  if (!calibrated && !calibrating)
  {
    Serial.println("first you need to calibrate the balancing points (over bluetooth)...");
    if (!calibrated_leds)
    {
      indicateCalibration();
      calibrated_leds = true;
    }
    else
    {
      pixels.clear();
      calibrated_leds = false;
    }
  }
}

void indicateCalibration()
{
  for (int i = 0; i < NUMPIXELS; i++)
  {
    pixels.setPixelColor(i, 255, 255, 255);
    pixels.setBrightness(255);
    pixels.show();
    delay(100);
  }
  delay(1000);
  turnoff_pixel();
}

void calibrateGyro()
{
  for (int i = 0; i < 512; i++)
  {
    angle_calc();
    GyZ_offset_sum += GyZ;
    delay(5);
  }
  GyZ_offset = GyZ_offset_sum >> 9;

  for (int i = 0; i < 512; i++)
  {
    angle_calc();
    GyY_offset_sum += GyY;
    delay(5);
  }
  GyY_offset = GyY_offset_sum >> 9;

  for (int i = 0; i < 512; i++)
  {
    angle_calc();
    GyX_offset_sum += GyX;
    delay(5);
  }
  GyX_offset = GyX_offset_sum >> 9;
}

void angle_setup()
{
  Wire.begin();
  delay(100);
  writeTo(MPU6050, PWR_MGMT_1, 0);
  writeTo(MPU6050, ACCEL_CONFIG, accSens << 3); // Specifying output scaling of accelerometer
  writeTo(MPU6050, GYRO_CONFIG, gyroSens << 3); // Specifying output scaling of gyroscope
  delay(100);

  pixels.setPixelColor(0, 0, 255, 0);
  pixels.setBrightness(255);
  pixels.show();
  for (int i = 0; i < 512; i++)
  {
    angle_calc();
    GyZ_offset_sum += GyZ;
    delay(5);
  }
  GyZ_offset = GyZ_offset_sum >> 9;
  Serial.print("GyZ offset value = ");
  Serial.println(GyZ_offset);

  pixels.setPixelColor(1, 0, 255, 0);
  pixels.setBrightness(255);
  pixels.show();

  for (int i = 0; i < 512; i++)
  {
    angle_calc();
    GyY_offset_sum += GyY;
    delay(5);
  }
  GyY_offset = GyY_offset_sum >> 9;
  Serial.print("GyY offset value = ");
  Serial.println(GyY_offset);

  pixels.setPixelColor(2, 0, 255, 0);
  pixels.setBrightness(255);
  pixels.show();

  for (int i = 0; i < 512; i++)
  {
    angle_calc();
    GyX_offset_sum += GyX;
    delay(5);
  }
  GyX_offset = GyX_offset_sum >> 9;
  Serial.print("GyX offset value = ");
  Serial.println(GyX_offset);

  for (int i = 0; i < NUMPIXELS; i++)
  {
    pixels.setPixelColor(i, 40, 205, 180);
    pixels.setBrightness(255);
    pixels.show();
  }
  playNotes(4186, 4186, 4186, 100);
  turnoff_pixel();
}

void readGyroData()
{
  Wire.beginTransmission(MPU6050);
  Wire.write(0x43);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050, 6, true);
  GyX = Wire.read() << 8 | Wire.read();
  GyY = Wire.read() << 8 | Wire.read();
  GyZ = Wire.read() << 8 | Wire.read();
}

void readAccelData()
{
  Wire.beginTransmission(MPU6050);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050, 6, true);
  AcX = Wire.read() << 8 | Wire.read();
  AcY = Wire.read() << 8 | Wire.read();
  AcZ = Wire.read() << 8 | Wire.read();
}

void angle_calc()
{
  readGyroData();
  readAccelData();

  if (abs(AcX) < 2000)
  {
    AcXc = AcX - offsets.acXv;
    AcYc = AcY - offsets.acYv;
    AcZc = AcZ - offsets.acZv;
  }
  else
  {
    AcXc = AcX - offsets.acXe;
    AcYc = AcY - offsets.acYe;
    AcZc = AcZ - offsets.acZe;
  }
  GyZ -= GyZ_offset;
  GyY -= GyY_offset;
  GyX -= GyX_offset;

  calculateAngles();
}

void calculateAngles()
{
  const float gyroScaleFactor = loop_time / 1000.0 / 65.536;
  const float radToDeg = 57.2958;

  // Update robot angles using gyroscope data
  robot_angleY += GyY * gyroScaleFactor;
  robot_angleX += GyX * gyroScaleFactor;

  // Calculate accelerometer angles
  Acc_angleY = atan2(AcXc, -AcZc) * radToDeg;
  Acc_angleX = -atan2(AcYc, -AcZc) * radToDeg;

  // Combine gyroscope and accelerometer data using complementary filter
  robot_angleY = robot_angleY * Gyro_amount + Acc_angleY * (1.0 - Gyro_amount);
  robot_angleX = robot_angleX * Gyro_amount + Acc_angleX * (1.0 - Gyro_amount);

  updateVerticalState();
}

void updateVerticalState()
{
  switch (currentState)
  {
  case VERTICAL_UNKNOWN:
    if (abs(AcX) < 2000 && abs(Acc_angleX) < 0.4 && abs(Acc_angleY) < 0.4)
    {
      robot_angleX = Acc_angleX;
      robot_angleY = Acc_angleY;
      currentState = VERTICAL_VERTEX;
    }
    else if (abs(AcX) > 7000 && abs(AcX) < 10000 && abs(Acc_angleX) < 0.3)
    {
      robot_angleX = Acc_angleX;
      robot_angleY = Acc_angleY;
      currentState = VERTICAL_EDGE;
    }
    else
    {
      currentState = VERTICAL_NONE;
    }
    break;

  case VERTICAL_VERTEX:
    if (abs(robot_angleX) > 7 || abs(robot_angleY) > 7)
    {
      currentState = VERTICAL_NONE;
    }
    break;

  case VERTICAL_EDGE:
    if (abs(robot_angleX) > 7 || abs(robot_angleY) > 7)
    {
      currentState = VERTICAL_NONE;
    }
    break;

  case VERTICAL_NONE:
    if (abs(AcX) < 2000 && abs(Acc_angleX) < 0.4 && abs(Acc_angleY) < 0.4)
    {
      robot_angleX = Acc_angleX;
      robot_angleY = Acc_angleY;
      currentState = VERTICAL_VERTEX;
    }
    else if (abs(AcX) > 7000 && abs(AcX) < 10000 && abs(Acc_angleX) < 0.3)
    {
      robot_angleX = Acc_angleX;
      robot_angleY = Acc_angleY;
      currentState = VERTICAL_EDGE;
    }
    break;
  }
}

void handleVerticalVertexState()
{
  digitalWrite(BRAKE, HIGH);

  // Normalize gyroscope readings
  const float gyroScale = 1.0 / 131.0;
  gyroX = GyX * gyroScale;
  gyroY = GyY * gyroScale;
  gyroZ = GyZ * gyroScale;

  // Apply low-pass filter to gyroscope readings
  gyroXfilt = alpha * gyroX + (1 - alpha) * gyroXfilt;
  gyroYfilt = alpha * gyroY + (1 - alpha) * gyroYfilt;

  // Introduce a 1 degree per second rotation on the z-axis
  // const float rotationRateZ = 1.0; // 1 degree per second
  // gyroZ += rotationRateZ;

  // Calculate PWM values using PID control
  int pwm_X = constrain(K1 * robot_angleX + K2 * gyroXfilt + K3 * speed_X + K4 * motors_speed_X, -255, 255);
  int pwm_Y = constrain(K1 * robot_angleY + K2 * gyroYfilt + K3 * speed_Y + K4 * motors_speed_Y, -255, 255);
  int pwm_Z = constrain(zK2 * gyroZ + zK3 * motors_speed_Z, -255, 255);

  // Update motor speeds
  motors_speed_X += speed_X / 5;
  motors_speed_Y += speed_Y / 5;

  // Control motors based on calculated PWM values
  XYZ_to_threeWay(-pwm_X, pwm_Y, -pwm_Z);
}

void handleVerticalEdgeState()
{
  digitalWrite(BRAKE, HIGH);
  gyroX = GyX / 131.0;
  gyroXfilt = alpha * gyroX + (1 - alpha) * gyroXfilt;

  int pwm_X = constrain(eK1 * robot_angleX + eK2 * gyroXfilt + eK3 * motor3_speed + eK4 * motors_speed_X, -255, 255);

  motors_speed_X += motor3_speed / 5;
  Motor_control(3, pwm_X, motor3_speed, DIR3, PWM3_CH);
}

void turnoff_pixel()
{
  for (int i = 0; i < NUMPIXELS; i++)
  {
    pixels.setPixelColor(i, 0, 0, 0);
    pixels.setBrightness(0);
    pixels.show();
    delay(10);
    pixels.clear();
  }
}

void writeTo(byte device, byte address, byte value)
{
  Wire.beginTransmission(device);
  Wire.write(address);
  Wire.write(value);
  Wire.endTransmission(true);
}

void playNotes(uint16_t note1, uint16_t note2, uint16_t note3, long duration)
{
  tone(BUZZER, note1, duration, channel);
  delay(duration);
  noTone(BUZZER, channel);
  tone(BUZZER, note2, duration, channel);
  delay(duration);
  noTone(BUZZER, channel);
  tone(BUZZER, note3, duration, channel);
  delay(duration);
  noTone(BUZZER, channel);
}

void tone(uint8_t pin, unsigned int frequency, unsigned long duration, uint8_t channel)
{
  if (ledcRead(channel))
  {
    log_e("Tone channel %d is already in use", ledcRead(channel));
    return;
  }
  ledcAttachPin(pin, channel);
  ledcWriteTone(channel, frequency);
  if (duration)
  {
    delay(duration);
    noTone(pin, channel);
  }
}

void noTone(uint8_t pin, uint8_t channel)
{
  ledcDetachPin(pin);
  ledcWrite(channel, 0);
}

void save()
{
  EEPROM.put(0, offsets);
  EEPROM.commit();
  EEPROM.get(0, offsets);
  if (offsets.ID == 96)
    calibrated = true;
  calibrating = false;
  Serial.println("Calibrating off.");
  playNotes(4186, 4699, 5274, 100);
  delay(300);
}

void XYZ_to_threeWay(float pwm_X, float pwm_Y, float pwm_Z)
{
  int16_t m1 = round((0.5 * pwm_X - 0.866 * pwm_Y) / 1.37 + pwm_Z);
  int16_t m2 = round((0.5 * pwm_X + 0.866 * pwm_Y) / 1.37 + pwm_Z);
  int16_t m3 = -pwm_X / 1.37 + pwm_Z;
  Motor_control(1, m1, motor1_speed, DIR1, PWM1_CH);
  Motor_control(2, m2, motor2_speed, DIR2, PWM2_CH);
  Motor_control(3, m3, motor3_speed, DIR3, PWM3_CH);
}

void threeWay_to_XY(int in_speed1, int in_speed2, int in_speed3)
{
  speed_X = ((in_speed3 - (in_speed2 + in_speed1) * 0.5) * 0.5) * 1.81;
  speed_Y = -(-0.866 * (in_speed2 - in_speed1)) / 1.1;
}

void battVoltage(double voltage)
{
  if (voltage > 8 && voltage <= 9.5)
  {
    digitalWrite(BUZZER, HIGH);
  }
  else
  {
    digitalWrite(BUZZER, LOW);
  }
}

void pwmSet(uint8_t channel, uint32_t value)
{
  ledcWrite(channel, value);
}

void Motor_control(int motor_number, int sp, int motor_speed, uint8_t dir_pin, uint8_t pwm_channel)
{
  sp = sp + motor_speed;
  if (sp < 0)
    digitalWrite(dir_pin, LOW);
  else
    digitalWrite(dir_pin, HIGH);
  pwmSet(pwm_channel, 255 - abs(sp));
}

void IRAM_ATTR ENC1_READ()
{
  static int state = 0;
  state = (state << 2 | (digitalRead(ENC1_1) << 1) | digitalRead(ENC1_2)) & 0x0f;
  if (state == 0x02 || state == 0x0d || state == 0x04 || state == 0x0b)
  {
    enc_count1++;
  }
  else if (state == 0x01 || state == 0x0e || state == 0x08 || state == 0x07)
  {
    enc_count1--;
  }
}

void IRAM_ATTR ENC2_READ()
{
  static int state = 0;
  state = (state << 2 | (digitalRead(ENC2_1) << 1) | digitalRead(ENC2_2)) & 0x0f;
  if (state == 0x02 || state == 0x0d || state == 0x04 || state == 0x0b)
  {
    enc_count2++;
  }
  else if (state == 0x01 || state == 0x0e || state == 0x08 || state == 0x07)
  {
    enc_count2--;
  }
}

void IRAM_ATTR ENC3_READ()
{
  static int state = 0;
  state = (state << 2 | (digitalRead(ENC3_1) << 1) | digitalRead(ENC3_2)) & 0x0f;
  if (state == 0x02 || state == 0x0d || state == 0x04 || state == 0x0b)
  {
    enc_count3++;
  }
  else if (state == 0x01 || state == 0x0e || state == 0x08 || state == 0x07)
  {
    enc_count3--;
  }
}