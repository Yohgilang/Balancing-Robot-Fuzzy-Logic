#include <Fuzzy.h>
#include <math.h>
#include <MPU6050_light.h>
#include <Wire.h>
#include <SimpleKalmanFilter.h>
#include "WEMOS_Motor.h"

MPU6050 gy_521(Wire);

// GY521 initialization
int16_t ax, ay, az;
int16_t gx, gy, gz;

// Kalman Filter
SimpleKalmanFilter kf(2, 2, 0.1);

// Driver motor initialization
Motor M1(0x30, _MOTOR_A, 1000); //Motor A
Motor M2(0x30, _MOTOR_B, 1000); //Motor B

int mot_speed;
float error = 0;
float error_sebelum = 0;
float derror = 0;

// ------------------------------------------------------------------

// Fuzzy object initialization
Fuzzy *fuzzy = new Fuzzy();

//input error
FuzzySet *neg = new FuzzySet(-30, -30, -20, 0);
FuzzySet *zero = new FuzzySet(-15, 0, 0, 15);
FuzzySet *pos = new FuzzySet(0, 20, 30, 30);

//input derror
FuzzySet *dneg = new FuzzySet(-30, -30, -20, 0);
FuzzySet *dzero = new FuzzySet(-15, 0, 0, 15);
FuzzySet *dpos = new FuzzySet(0, 20, 30, 30);

//output pwm
FuzzySet *apelan = new FuzzySet(90, 90, 90, 90);
FuzzySet *pelan = new FuzzySet(90, 90, 90, 90);
FuzzySet *sedang = new FuzzySet(120, 120, 120, 120);
FuzzySet *asedang = new FuzzySet(120, 120, 120, 120);
FuzzySet *cepat = new FuzzySet(150, 150, 150, 150);

float pitch = 0;

void setup()
{
  Wire.begin();
  Serial.begin(9600);
  gy_521.begin();

  gy_521.calcGyroOffsets();

  //FuzzyInput1-error
  FuzzyInput *error = new FuzzyInput(1);
  error->addFuzzySet(neg);
  error->addFuzzySet(zero);
  error->addFuzzySet(pos);
  fuzzy->addFuzzyInput(error);

  //FuzzyInput2-derror
  FuzzyInput *derror = new FuzzyInput(2);
  derror->addFuzzySet(dneg);
  derror->addFuzzySet(dzero);
  derror->addFuzzySet(dpos);
  fuzzy->addFuzzyInput(derror);

  //FuzzyOutput-pwm
  FuzzyOutput *pwm = new FuzzyOutput(1);
  pwm->addFuzzySet(apelan);
  pwm->addFuzzySet(pelan);
  pwm->addFuzzySet(sedang);
  pwm->addFuzzySet(asedang);
  pwm->addFuzzySet(cepat);
  fuzzy->addFuzzyOutput(pwm);

  //-------------------------------------------------------------------------------------------------------------

  //Build Rule Base
  // 1 -- pwm = pelan IF er = neg && der = dzero OR er = zero && der = dneg (2 RULE BASE)
  FuzzyRuleAntecedent *errorNegAndderrorDzero = new FuzzyRuleAntecedent();
  errorNegAndderrorDzero->joinWithAND(neg, dzero);
  FuzzyRuleAntecedent *errorZeroAndderrorDneg = new FuzzyRuleAntecedent();
  errorZeroAndderrorDneg->joinWithAND(zero, dneg);

  FuzzyRuleAntecedent *iferrorNegAndderrorDzeroOrerrorZeroAndderrorDneg = new FuzzyRuleAntecedent();
  iferrorNegAndderrorDzeroOrerrorZeroAndderrorDneg->joinWithOR(errorNegAndderrorDzero, errorZeroAndderrorDneg);
  FuzzyRuleConsequent *thenpwmPelan = new FuzzyRuleConsequent();
  thenpwmPelan->addOutput(pelan);

  FuzzyRule *fuzzyRule1 = new FuzzyRule(1, iferrorNegAndderrorDzeroOrerrorZeroAndderrorDneg, thenpwmPelan);
  fuzzy->addFuzzyRule(fuzzyRule1);

  // 2 -- pwm = apelan IF er = zero && der = dzero (1 RULE BASE)
  FuzzyRuleAntecedent *iferrorZeroAndderrorDzero = new FuzzyRuleAntecedent();
  iferrorZeroAndderrorDzero->joinWithAND(zero, dzero);
  FuzzyRuleConsequent *thenpwmApelan = new FuzzyRuleConsequent();
  thenpwmApelan->addOutput(apelan);

  FuzzyRule *fuzzyRule2 = new FuzzyRule(2, iferrorZeroAndderrorDzero, thenpwmApelan);
  fuzzy->addFuzzyRule(fuzzyRule2);

  // 3 -- pwm = sedang IF er = neg && der = dpos OR er = zero && der = dpos  (2 RULE BASE)
  FuzzyRuleAntecedent *errorNegAndderrorDpos = new FuzzyRuleAntecedent();
  errorNegAndderrorDpos->joinWithAND(neg, dpos);
  FuzzyRuleAntecedent *errorZeroAndderrorDpos = new FuzzyRuleAntecedent();
  errorZeroAndderrorDpos->joinWithAND(zero, dpos);

  FuzzyRuleAntecedent *iferrorNegAndderrorDposOrerrorZeroAndderrorDpos = new FuzzyRuleAntecedent();
  iferrorNegAndderrorDposOrerrorZeroAndderrorDpos->joinWithOR(errorNegAndderrorDpos, errorZeroAndderrorDpos);
  FuzzyRuleConsequent *thenpwmSedang = new FuzzyRuleConsequent();
  thenpwmSedang->addOutput(sedang);

  FuzzyRule *fuzzyRule3 = new FuzzyRule(3, iferrorNegAndderrorDposOrerrorZeroAndderrorDpos, thenpwmSedang);
  fuzzy->addFuzzyRule(fuzzyRule3);

  // 4 -- pwm = asedang IF er = pos && der = dneg OR er = pos && der = dzero (2 RULE BASE)
  FuzzyRuleAntecedent *errorPosAndderrorDneg = new FuzzyRuleAntecedent();
  errorPosAndderrorDneg->joinWithAND(pos, dneg);
  FuzzyRuleAntecedent *errorPosAndderrorDzero = new FuzzyRuleAntecedent();
  errorPosAndderrorDzero->joinWithAND(pos, dzero);

  FuzzyRuleAntecedent *iferrorPosAndderrorDnegOrerrorPosAndderrorDzero = new FuzzyRuleAntecedent();
  iferrorPosAndderrorDnegOrerrorPosAndderrorDzero->joinWithOR(errorPosAndderrorDneg, errorPosAndderrorDzero);
  FuzzyRuleConsequent *thenpwmAsedang = new FuzzyRuleConsequent();
  thenpwmAsedang->addOutput(asedang);

  FuzzyRule *fuzzyRule4 = new FuzzyRule(4, iferrorPosAndderrorDnegOrerrorPosAndderrorDzero, thenpwmAsedang);
  fuzzy->addFuzzyRule(fuzzyRule4);

  // 5 -- pwm = cepat IF er = neg && der = dneg OR er = pos && der = dpos (2 RULE BASE)
  FuzzyRuleAntecedent *errorNegAndderrorDneg = new FuzzyRuleAntecedent();
  errorNegAndderrorDneg->joinWithAND(neg, dneg);
  FuzzyRuleAntecedent *errorPosAndderrorDpos = new FuzzyRuleAntecedent();
  errorPosAndderrorDpos->joinWithAND(pos, dpos);

  FuzzyRuleAntecedent *iferrorNegAndderrorDnegOrerrorPosAndderrorDpos = new FuzzyRuleAntecedent();
  iferrorNegAndderrorDnegOrerrorPosAndderrorDpos->joinWithOR(errorNegAndderrorDneg, errorPosAndderrorDpos);
  FuzzyRuleConsequent *thenpwmCepat = new FuzzyRuleConsequent();
  thenpwmCepat->addOutput(cepat);

  FuzzyRule *fuzzyRule5 = new FuzzyRule(5, iferrorNegAndderrorDnegOrerrorPosAndderrorDpos, thenpwmCepat);
  fuzzy->addFuzzyRule(fuzzyRule5);

  delay(10000);
}

void loop()
{
  //Sensor update
  gy_521.update();

  //Filter sensor with Kalman Filter
  pitch = kf.updateEstimate(gy_521.getAngleX());

  //Set error dan derror
  error = 0 - pitch;
  derror = error - error_sebelum;
  error_sebelum = error;

  //Set error and derror value as an input
  fuzzy->setInput(1, error);
  fuzzy->setInput(2, derror);

  //Running the Fuzzification
  fuzzy->fuzzify();

  //Running the Defuzzification
  float output = fuzzy->defuzzify(1);

  //Printing error
  Serial.print("Error:");
  Serial.print(error);
  Serial.print(",");

  //Printing derror
  Serial.print("Derror:");
  Serial.print(derror);
  Serial.print(",");

  //Printing output sensor
  Serial.print("Gyroscope:");
  Serial.print(pitch);
  Serial.print(",");

  //Printing output PWM
  Serial.print("PWM:");
  Serial.println(output);

  //Set motor value to absolute
  mot_speed = abs(output);

  //Set rotation motor
  if (error < 0 && derror > 0 || error > 0 && derror > 0) {
    M1.setmotor(_CCW, mot_speed);
    M2.setmotor(_CCW, mot_speed);
  }
  else {
    M1.setmotor(_CW, mot_speed);
    M2.setmotor(_CW, mot_speed);
  }
  delay(50);
}
