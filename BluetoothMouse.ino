#include <BleMouse.h>
#include<Wire.h>

// Define mouse Sensitivity
float mouse_sensitivity_X = 0.2;
float mouse_sensitivity_Y = 0.2;

// Define scroll Sensitivity
float scroll_sensitivity_X = 0.1;
float scroll_sensitivity_Y = 0.1;

// Define Maximum angle for mouse movement relative to current position
int yMinAngle = -40;
int yMaxAngle = 40;
int xMinAngle = -40;
int xMaxAngle = 40;

// Used for updating angle values using gyroscope
unsigned long previous_time;

// Calibration values for gyroscope
float RateCalibrationRoll, RateCalibrationPitch, RateCalibrationYaw;

// Variables for processing gyroscope readings
int16_t gyroX, gyroY, gyroZ;
float rateRoll, ratePitch, rateYaw;

// Angles Values
float MouseAngleRoll = 0, MouseAnglePitch = 0, MouseAngleYaw = 0;
float ScrollAngleRoll = 0, ScrollAnglePitch = 0, ScrollAngleYaw = 0;

// Bluetooth Mouse
BleMouse bleMouse;

// Flags used to reset mouse pointer and scroll flags
int move_flag = 0, scroll_flag = 0;

// Used to update scroll value
unsigned long last_scrolled_time;

void setup() {

  Serial.begin(115200);

  pinMode(4, INPUT);
  pinMode(5, INPUT);
  pinMode(18, INPUT);
  pinMode(15, INPUT);

  Wire.begin();
  setupMPU();
  bleMouse.begin();

  previous_time = millis();

  last_scrolled_time = millis();
}

void loop() {

  recordGyroRegisters();
  adjustGyroReadings();
  processAngles();

  if (bleMouse.isConnected()) {

    int pressed_state = digitalRead(5);
    int left_click = digitalRead(18);
    int right_click = digitalRead(15);
    int scroll = digitalRead(4);

    if(left_click == HIGH && !bleMouse.isPressed(MOUSE_LEFT))
    {
      bleMouse.press(MOUSE_LEFT);
    }
    else if(left_click == LOW)
    {
      bleMouse.release(MOUSE_LEFT);
    }

    if(right_click == HIGH && !bleMouse.isPressed(MOUSE_RIGHT))
    {
      bleMouse.press(MOUSE_RIGHT);
    }
    else if(right_click == LOW)
    {
      bleMouse.release(MOUSE_RIGHT);
    }

    if(scroll == HIGH)
    {
      if(scroll_flag == 0)
      {
        ScrollAngleRoll = 0;
        ScrollAngleYaw = 0;
        scroll_flag = 1;
      }
      else if(millis()-last_scrolled_time>100)
      {
        bleMouse.move(0,0,(int)min(yMaxAngle*scroll_sensitivity_Y,max(yMinAngle*scroll_sensitivity_Y, ScrollAngleRoll*scroll_sensitivity_Y)),-(int)min(xMaxAngle*scroll_sensitivity_X,max(xMinAngle*scroll_sensitivity_X, (ScrollAngleYaw)*scroll_sensitivity_X)));
        last_scrolled_time = millis();
      }
    }
    else
    {
      scroll_flag = 0;
    }

    if(pressed_state == HIGH)
    {
      if(move_flag == 0)
      {
        MouseAngleRoll = 0;
        MouseAngleYaw = 0;
        move_flag = 1;
      }
      else
      {
        bleMouse.move(-(int)min(yMaxAngle*mouse_sensitivity_X,max(yMinAngle*mouse_sensitivity_X, (MouseAngleYaw)*mouse_sensitivity_X)),-(int)min(xMaxAngle*mouse_sensitivity_Y,max(xMinAngle*mouse_sensitivity_Y, MouseAngleRoll*mouse_sensitivity_Y)));
      }
    }
    else
    {
      move_flag = 0;
    }

  }

  delay(5);
}

void setupMPU()
{
  Wire.beginTransmission(0b1101000); //I2C address of MPU6050
  Wire.write(0x6B); //Accessing the 6B register in MPU6050 - Power Management Mode Register
  Wire.write(0b00000000); //Set Sleep mode to 0.
  Wire.endTransmission(true);

  Wire.beginTransmission(0b1101000); //I2C address of MPU6050
  Wire.write(0x1B); //Accessing the 1B register in MPU6050 - Gyroscope Configuration Register
  Wire.write(0b00000000); //Setting Gyroscope to full scale +/- 250deg
  Wire.endTransmission(true);

  Wire.beginTransmission(0b1101000); //I2C address of MPU6050
  Wire.write(0x1C); //Accessing the 1C register in MPU6050 - Accelerometer Configuration Register
  Wire.write(0b00000000); //Setting Accelerometer to full scale +/- 2g
  Wire.endTransmission(true);

  caliberateGyroscope();
}

void caliberateGyroscope()
{
  RateCalibrationRoll = 0;
  RateCalibrationPitch = 0;
  RateCalibrationYaw = 0;
  for(int i = 0; i<2000; i++)
  {
    recordGyroRegisters();
    RateCalibrationRoll+=rateRoll;
    RateCalibrationPitch+=ratePitch;
    RateCalibrationYaw+=rateYaw;
    delay(1);
  }
  RateCalibrationRoll/=2000;
  RateCalibrationPitch/=2000;
  RateCalibrationYaw/=2000;
}

void recordGyroRegisters()
{
  Wire.beginTransmission(0b1101000);
  Wire.write(0x43); //Starting register for Gyro Readings
  Wire.endTransmission(false);
  Wire.requestFrom(0b1101000, 6); //Request Gyro Registers (43 - 48)
  while(Wire.available() < 6);
  gyroX = Wire.read()<<8 | Wire.read(); //Store first two bytes into gyroX
  gyroY = Wire.read()<<8 | Wire.read(); //Store second two bytes into gyroY
  gyroZ = Wire.read()<<8 | Wire.read(); //Store third two bytes into gyroZ
  processGyroData();
}

void processGyroData()
{
  rateRoll = gyroX / 131.0;
  ratePitch = gyroY / 131.0;
  rateYaw = gyroZ / 131.0;
}

void adjustGyroReadings()
{
  rateRoll-=RateCalibrationRoll;
  ratePitch-=RateCalibrationPitch;
  rateYaw-=RateCalibrationYaw;
}

void processAngles()
{

  unsigned long current_time = millis();
  float delta_time = (current_time - previous_time) / 1000.0;  // Convert ms to seconds
  previous_time = current_time;

  MouseAngleRoll += rateRoll * delta_time;
  MouseAnglePitch += ratePitch * delta_time;
  MouseAngleYaw += rateYaw * delta_time;

  ScrollAngleRoll += rateRoll * delta_time;
  ScrollAnglePitch += ratePitch * delta_time;
  ScrollAngleYaw += rateYaw * delta_time;
}