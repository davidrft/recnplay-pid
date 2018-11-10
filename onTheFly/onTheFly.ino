
#include <Ultrasonic.h>
#include <Servo.h>

#define pKP A1
#define pKI A2
#define pKD A3

double erro, setpoint, input; // distancia US
double kp, ki, kd; // constantes
double output; // grau do motor

double ultimoErro, diffErro, erroDeriv;
long ultimaAtualizacao, diffTempo;

double acc;

Ultrasonic ultrassonic(12, 13);
Servo servo;

void setup() {
  Serial.begin(9600);
	servo.attach(9);
	// kp = 1;
	// ki = 0;
	// kd = 1;
	setpoint = 25;
	ultimaAtualizacao = millis();
}

double modifiedMap(double x, double in_min, double in_max, double out_min, double out_max) {
	double temp = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
	temp = (int) (4*temp + .5);
	return (double) temp/4;
}

void loop() {
	//input = ultrassonic.read(CM); // 0-50
	//Serial.println(input);
	input = ultrassonic.read(CM); // 0-50
	kp = modifiedMap(analogRead(pKP), 0, 1023, 0, 10);
	ki = modifiedMap(analogRead(pKI), 0, 1023, 0, 0);
	kd = modifiedMap(analogRead(pKD), 0, 1023, 0, 5);
	if (input < 50 && input > 0) {
		// Proporcional
		erro = input - setpoint; // -25 - 25
		// Derivativo
		diffErro = erro - ultimoErro;
		diffTempo = millis() - ultimaAtualizacao;
		erroDeriv = diffErro / diffTempo;
		// Integrativa
		acc = acc + (erro*diffTempo);

		// Controle
		output = erro*kp + acc*ki + erroDeriv*kd; // -250 - 250
		// 10 - 170
		output = map(output, -25, 25, 10, 170);
		servo.write(output);

		ultimoErro = erro;
		ultimaAtualizacao = millis();
	}
	Serial.print("KP: ");
	Serial.print(kp);
	Serial.print("\tKI: ");
	Serial.print(ki);
	Serial.print("\tKD: ");
	Serial.print(kd);
	Serial.print("\toutput:");
	Serial.println(output);
	delay(50);
}
