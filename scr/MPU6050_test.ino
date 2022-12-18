#include <Wire.h>
#include <MPU6050.h>

MPU6050 gy_521;

int16_t ax, ay, az;
int16_t gx, gy, gz;

void setup()
{
  Wire.begin( );
  Serial.begin (9600);
  Serial.println ("Initializing MPU and testing connections");

  gy_521.initialize ( );

  Serial.println(gy_521.testConnection( ) ? "Successfully Connected" : "Connection failed");
  delay(1000);
  Serial.println("Reading Values");
  delay(1000);
}

void loop ( )
{
  gy_521.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  ax = map(ax, -17000, 17000, -125, 125);

//  Serial.print("Accelerometer: ");
//  Serial.print("X = "); Serial.print(ax);
//  Serial.print(" | Y = "); Serial.print(ay);
//  Serial.print(" | Z = "); Serial.print(az);
//  Serial.println(" ");

  Serial.print("Gyroscope: ");
  Serial.print("X = "); Serial.print(gx);
//  Serial.print(" | Y = "); Serial.print(gy);
//  Serial.print(" | Z = "); Serial.print(gz);
  Serial.println(" ");

  delay(100);
}
