#include <Ultrasonic.h>
#include <Servo.h>

class PID {
public:
    double input, output, setpoint;
    double error_sum, last_error;
    double _kp, _ki, _kd;
    double time_diff;

    bool isPActive, isIActive, isDActive;

    void compute() {
        double error = input - setpoint;
        error_sum += (error * time_diff);
        double diff_error = (error - last_error);

        double P = _kp * error;
        double I = _ki * error_sum;
        double D = _kd * diff_error;

        output = P*isPActive + I*isIActive + D*isDActive;

        last_error = error;
    }

    void set_tunings(double kp, double ki, double kd) {
        _kp = kp;
        _ki = ki;
        _kd = kd;
    }
};

#define pTRIG 12
#define pECHO 13
#define pKP A1
#define pKI A2
#define pKD A3
#define pSERVO 9

int const sampleRate = 200;

PID pid;
Ultrasonic ultrasonic(pTRIG, pECHO);
Servo servo;

double modifiedMap(double x, double in_min, double in_max, double out_min, double out_max) {
  double temp = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  temp = (int) (4*temp + .5);
  return (double) temp/4;
}

void setup() {
    Serial.begin(9600);

    pinMode(pKP, INPUT);
    pinMode(pKI, INPUT);
    pinMode(pKD, INPUT);
  
    servo.attach(pSERVO);
    ultrasonic.setTimeout(4000UL);

    pid.isPActive = true;
    pid.isIActive = false;
    pid.isDActive = true;
    pid.setpoint = 30.0;
}

unsigned long last_time;
int numberOfSamples;
double kp, ki, kd;
double avgDistance;

void loop() {
  unsigned long now = millis();
  double distance = ultrasonic.read(CM);
  bool onlySetup = false;

  if ((distance < 50 || distance < 0) && !onlySetup) {
    double time_diff = (double)(now - last_time);

    avgDistance += distance;
    kp += modifiedMap(analogRead(pKP), 0, 1023, 0, 10);
    ki += 0.0001;
    kd += modifiedMap(analogRead(pKD), 0, 1023, 0, 10);
    numberOfSamples++;
    
    if (time_diff >= sampleRate) {
      pid._kp = kp/numberOfSamples;
      pid._ki = ki/numberOfSamples;
      pid._kd = kd/numberOfSamples;
      pid.input = avgDistance/numberOfSamples;
      pid.time_diff = time_diff;
      
      pid.compute();
      double servoValue = modifiedMap(pid.output, -250, 250, 10, 170);
      servo.write(servoValue);

      numberOfSamples = 0;
      kp = 0;
      ki = 0;
      kd = 0;
      avgDistance = 0;
      last_time = now;

      Serial.print("input = ");
      Serial.print(pid.input);
      Serial.print("\t");
      Serial.print("Kp = ");
      Serial.print(pid._kp);
      Serial.print("\t");
      Serial.print("Kd = ");
      Serial.print(pid._kd);
      Serial.print("\t");
      Serial.print("PID = ");
      Serial.println(pid.output);
    }
  }
}
