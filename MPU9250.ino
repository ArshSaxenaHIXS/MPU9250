/***************************************************************************
* Example sketch for the MPU6500_WE library
*
* This sketch shows how to get acceleration, gyroscocope and temperature 
* data from the MPU6500. In essence, the difference to the MPU9250 is the
* missing magnetometer. The shall only show how to "translate" all other 
* MPU9250 example sketches for use of the MPU6500
* 
* For further information visit my blog:
*
* https://wolles-elektronikkiste.de/mpu9250-9-achsen-sensormodul-teil-1  (German)
* https://wolles-elektronikkiste.de/en/mpu9250-9-axis-sensor-module-part-1  (English)
* 
***************************************************************************/

#include <MPU6500_WE.h>
#include <Wire.h>
#define MPU6500_ADDR 0x68

int lmf = 6;
int lmr = 7;
int rmf = 8;
int rmr = 9;
float angleZ = 0;
float dt = 0.01;
unsigned long prevTime;
/* There are several ways to create your MPU6500 object:
 * MPU6500_WE myMPU6500 = MPU6500_WE()              -> uses Wire / I2C Address = 0x68
 * MPU6500_WE myMPU6500 = MPU6500_WE(MPU6500_ADDR)  -> uses Wire / MPU6500_ADDR
 * MPU6500_WE myMPU6500 = MPU6500_WE(&wire2)        -> uses the TwoWire object wire2 / MPU6500_ADDR
 * MPU6500_WE myMPU6500 = MPU6500_WE(&wire2, MPU6500_ADDR) -> all together
 * Successfully tested with two I2C busses on an ESP32
 */
MPU6500_WE myMPU6500 = MPU6500_WE(MPU6500_ADDR);

void setup() {
  angleZ = 0;
  pinMode(LED_BUILTIN, OUTPUT);

  // mm(-100,-100);
  // delay(10000);
  // mm(100,100);
  // delay(10000);

  while (!BOOTSEL) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(500);
    digitalWrite(LED_BUILTIN, LOW);
  }





  Serial.begin(115200);
  Wire.setSCL(5);
  Wire.setSDA(4);
  pinMode(lmf, OUTPUT);
  pinMode(lmr, OUTPUT);
  pinMode(rmf, OUTPUT);
  pinMode(rmr, OUTPUT);
  Wire.begin();
  Wire.setClock(400000);
  if (!myMPU6500.init()) {
    Serial.println("MPU6500 does not respond");
  } else {
    Serial.println("MPU6500 is connected");
  }


  Serial.println("Position you MPU6500 flat and don't move it - calibrating...");
  delay(1000);
  myMPU6500.autoOffsets();
  Serial.println("Done!");

  myMPU6500.enableGyrDLPF();
  myMPU6500.enableGyrAxes(MPU6500_ENABLE_00Z);
  myMPU6500.setGyrDLPF(MPU6500_DLPF_6);
  myMPU6500.setSampleRateDivider(5);

  //myMPU6500.setAccRange(MPU6500_ACC_RANGE_2G);
  myMPU6500.setGyrRange(MPU6500_GYRO_RANGE_2000);
  // myMPU6500.enableAccDLPF(true);
  // myMPU6500.setAccDLPF(MPU6500_DLPF_6);
  delay(200);
}

void loop() {
  xyzFloat gValue = myMPU6500.getGValues();
  xyzFloat gyr = myMPU6500.getGyrValues();
  float temp = myMPU6500.getTemperature();
  float resultantG = myMPU6500.getResultantG(gValue);

  // Serial.println("Angle");
  Serial.println(abs(getAngleZ()));
  // Serial.println("Gyroscope data in degrees/s: ");
  // Serial.print(gyr.x);
  // Serial.print("   ");
  // Serial.print(gyr.y);
  // Serial.print("   ");
  // Serial.println(gyr.z);

  // Serial.print("Temperature in °C: ");
  // Serial.println(temp);

  turnRight();
  delay(1000);
  // mm(100,100);
  // delay(5000);
  // mm(-100,-100);
  // delay(5000);
  // mm(100,-100);
  // delay(5000);
  // mm(-100,100);
  // delay(5000);
}
void update() {

  unsigned long currentTime = millis();
  dt = (currentTime - prevTime) / 1000.0;
  prevTime = currentTime;
  xyzFloat gyr = myMPU6500.getGyrValues();
  angleZ += (gyr.z / 1) * dt;
}

float getAngleZ() {
  
  update();
  return angleZ;
}


void mm(int rms, int lms) {
  rms = constrain(rms, -255, 255);
  lms = constrain(lms, -255, 255);


  if (rms > 0) {
    analogWrite(rmf, rms);
    analogWrite(rmr, 0);
  } else {
    analogWrite(rmf, 0);
    analogWrite(rmr, -rms);
  }

  if (lms > 0) {
    analogWrite(lmf, lms);
    analogWrite(lmr, 0);
  } else {
    analogWrite(lmf, 0);
    analogWrite(lmr, -lms);
  }
}


void turnRight() {

  // angleZ = 0;
  update();

  while (true) {
    // angleZ=0;
    //
    update();

    Serial.println(abs(getAngleZ()));
    abs(getAngleZ());
    mm(100, -100);
    if (abs(getAngleZ()) > 90) {
      mm(0, 0);
      Serial.println("It Turned yayyyyyy");
      break;
    }
  }
}

void goStraight() {
  // angleZ = 0;


  update();
  abs(getAngleZ());
  Serial.println(abs(getAngleZ()));
  abs(getAngleZ());
  mm(100, 100);
  int error = (getAngleZ() - 0) * 0.7;
  mm(100 - error, 100 + error);
  
  
}

