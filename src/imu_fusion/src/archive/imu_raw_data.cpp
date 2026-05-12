// #include <Wire.h>
// #include <MPU9250_asukiaaa.h>

// MPU9250_asukiaaa imu;

// void setup() {
//   Serial.begin(115200);
//   Wire.begin();

//   imu.setWire(&Wire);

//   imu.beginAccel();
//   imu.beginGyro();
//   imu.beginMag();

//   delay(1000);

//   Serial.println("MPU9250 raw sensor data start");
//   Serial.println("ax, ay, az, gx, gy, gz, mx, my, mz");
// }

// void loop() {
//   imu.accelUpdate();
//   imu.gyroUpdate();
//   imu.magUpdate();

//   float ax = imu.accelX();
//   float ay = imu.accelY();
//   float az = imu.accelZ();

//   float gx = imu.gyroX();
//   float gy = imu.gyroY();
//   float gz = imu.gyroZ();

//   float mx = imu.magX();
//   float my = imu.magY();
//   float mz = imu.magZ();

//   // Serial.print("ax:");
//   // Serial.print(ax);
//   // Serial.print(" ay:");
//   // Serial.print(ay);
//   // Serial.print(" az:");
//   // Serial.println(az);

//   // Serial.print(" gx:");
//   // Serial.print(gx);
//   // Serial.print(" gy:");
//   // Serial.print(gy);
//   // Serial.print(" gz:");
//   // Serial.println(gz);

//   Serial.print(" mx:");
//   Serial.print(mx);
//   Serial.print(" my:");
//   Serial.print(my);
//   Serial.print(" mz:");
//   Serial.println(mz);

//   delay(50);
// }