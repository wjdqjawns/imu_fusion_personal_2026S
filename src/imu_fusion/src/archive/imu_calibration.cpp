// #include <Wire.h>
// #include <MPU9250_asukiaaa.h>

// MPU9250_asukiaaa imu;

// const int N = 1000;

// // flat condition 기준
// // 정지 상태에서 az ≈ -1g 이 정상인 경우
// const float AX_EXPECTED = 0.0;
// const float AY_EXPECTED = 0.0;
// const float AZ_EXPECTED = -1.0;

// void setup() {
//   Serial.begin(115200);
//   Wire.begin();

//   imu.setWire(&Wire);
//   imu.beginAccel();
//   imu.beginGyro();

//   delay(1000);

//   Serial.println("=================================");
//   Serial.println("MPU9250 Calibration Start");
//   Serial.println("Keep the sensor stationary and flat");
//   Serial.println("=================================");

//   delay(3000);

//   float ax_sum = 0.0;
//   float ay_sum = 0.0;
//   float az_sum = 0.0;

//   float gx_sum = 0.0;
//   float gy_sum = 0.0;
//   float gz_sum = 0.0;

//   for (int i = 0; i < N; i++) {
//     imu.accelUpdate();
//     imu.gyroUpdate();

//     ax_sum += imu.accelX();
//     ay_sum += imu.accelY();
//     az_sum += imu.accelZ();

//     gx_sum += imu.gyroX();
//     gy_sum += imu.gyroY();
//     gz_sum += imu.gyroZ();

//     delay(3);
//   }

//   float ax_avg = ax_sum / N;
//   float ay_avg = ay_sum / N;
//   float az_avg = az_sum / N;

//   float gx_avg = gx_sum / N;
//   float gy_avg = gy_sum / N;
//   float gz_avg = gz_sum / N;

//   float ax_bias = ax_avg - AX_EXPECTED;
//   float ay_bias = ay_avg - AY_EXPECTED;
//   float az_bias = az_avg - AZ_EXPECTED;

//   float gx_bias = gx_avg;
//   float gy_bias = gy_avg;
//   float gz_bias = gz_avg;

//   Serial.println();
//   Serial.println("===== Average Sensor Output =====");
//   Serial.print("ax_avg = "); Serial.println(ax_avg, 6);
//   Serial.print("ay_avg = "); Serial.println(ay_avg, 6);
//   Serial.print("az_avg = "); Serial.println(az_avg, 6);

//   Serial.print("gx_avg = "); Serial.println(gx_avg, 6);
//   Serial.print("gy_avg = "); Serial.println(gy_avg, 6);
//   Serial.print("gz_avg = "); Serial.println(gz_avg, 6);

//   Serial.println();
//   Serial.println("===== Calibration Bias =====");
//   Serial.print("ax_bias = "); Serial.println(ax_bias, 6);
//   Serial.print("ay_bias = "); Serial.println(ay_bias, 6);
//   Serial.print("az_bias = "); Serial.println(az_bias, 6);

//   Serial.print("gx_bias = "); Serial.println(gx_bias, 6);
//   Serial.print("gy_bias = "); Serial.println(gy_bias, 6);
//   Serial.print("gz_bias = "); Serial.println(gz_bias, 6);

//   Serial.println();
//   Serial.println("===== Copy this to your estimation code =====");
//   Serial.print("float ax_bias = "); Serial.print(ax_bias, 6); Serial.println(";");
//   Serial.print("float ay_bias = "); Serial.print(ay_bias, 6); Serial.println(";");
//   Serial.print("float az_bias = "); Serial.print(az_bias, 6); Serial.println(";");

//   Serial.print("float gx_bias = "); Serial.print(gx_bias, 6); Serial.println(";");
//   Serial.print("float gy_bias = "); Serial.print(gy_bias, 6); Serial.println(";");
//   Serial.print("float gz_bias = "); Serial.print(gz_bias, 6); Serial.println(";");

//   Serial.println();
//   Serial.println("Calibration finished.");
// }

// void loop() {
//   // nothing
// }