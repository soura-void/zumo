#include <Wire.h>
#include <Zumo32U4.h>
#include <math.h>
#include "EEPROM.h"

#define LL	countsArray[0]
#define LR	countsArray[1]
#define FL	countsArray[2]
#define FR	countsArray[3]
#define RL	countsArray[4]
#define RR	countsArray[5]
#define BLEVELS			30
#define MAXBRIGHTNESS	30
#define LEFTSPEED		300
#define RIGHTSPEED		300


Zumo32U4LCD lcd;
Zumo32U4ButtonA A;
Zumo32U4ButtonB B;
Zumo32U4ButtonC C;
Zumo32U4Motors m;
Zumo32U4ProximitySensors prox;
Zumo32U4Buzzer buzz;

int MAP = 0;
static float Kp=0.0, Ki=0.0, Kd=0.0;
static char pBuffer[100];
int countsArray[6], error, lastError, sumError, control, setPoint;

enum button {
	BUTTONA, BUTTONB, BUTTONC, INVALID
};

button state = INVALID;

void lcdPrint(char *a, char *b)
{
	lcd.clear();
	lcd.print(a);
	lcd.gotoXY(0, 1);
	lcd.print(b);
}

void lcdPrint(String a, String b)
{
	lcd.clear();
	lcd.print(a);
	lcd.gotoXY(0, 1);
	lcd.print(b);
}

void lcdPrint(char *a, int b)
{
	lcd.clear();
	lcd.print(a);
	lcd.gotoXY(0, 1);
	lcd.print(b);
}

void getInputButton()
{
	state = INVALID;
	while (state==INVALID)
	{
		if (A.getSingleDebouncedPress())
			state = BUTTONA;
		else if (B.getSingleDebouncedPress())
			state = BUTTONB;
		else if (C.getSingleDebouncedPress())
			state = BUTTONC;
	}
}

void change_param()
{
	bool flag = true;
	lcdPrint("A B C", "P I D");
	getInputButton();
	switch (state)
	{
	case BUTTONA:
		do
		{
			lcd.clear();
			lcd.print("Change P");
			lcd.gotoXY(0, 1);
			lcd.print(String(Kp));
			getInputButton();
			switch (state)
			{
			case BUTTONB:
				Kp = Kp + 0.1;
				break;
			case BUTTONC:
				Kp = Kp - 0.1;
				break;
			}
		} while (state != BUTTONA);
		break;

	case BUTTONB:
		do
		{
			lcd.clear();
			lcd.print("Change I");
			lcd.gotoXY(0, 1);
			lcd.print(String(Ki));
			getInputButton();
			switch (state)
			{
			case BUTTONB:
				Ki = Ki + 0.1;
				break;
			case BUTTONC:
				Ki = Ki - 0.1;
				break;
			}
		} while (state != BUTTONA);
		break;

	case BUTTONC:
		do
		{
			lcd.clear();
			lcd.print("Change D");
			lcd.gotoXY(0, 1);
			lcd.print(String(Kd));
			getInputButton();
			switch (state)
			{
			case BUTTONB:
				Kd = Kd + 0.1;
				break;
			case BUTTONC:
				Kd = Kd - 0.1;
				break;
			}
		} while (state != BUTTONA);
		break;

	}

	int address = 0;
	EEPROM.put(0, Kp);
	address = address + sizeof(float);
	EEPROM.put(address, Ki);
	address = address + sizeof(float);
	EEPROM.put(address, Kd);
}

void startMusic()
{
	lcd.clear();
	lcdPrint("---3---", "To Start");
	buzz.playFrequency(800, 500, 15);
	delay(1000);
	lcd.clear();
	lcdPrint("---2---", "To Start");
	buzz.playFrequency(800, 500, 15);
	delay(1000);
	lcd.clear();
	lcdPrint("---1---", "To Start");
	buzz.playFrequency(800, 500, 15);
	delay(1000);
	buzz.playFrequency(2000, 1000, 15);
}

void welcome()
{
	lcdPrint("~~The~~", "Beng Bot");
	delay(2000);
	lcdPrint("~~Wall~~", "~Follow~");
	delay(2000);
	lcdPrint("Kp", "Ki  Kd");
	delay(1000);
}

void setup()
{
	welcome();
	int address = 0;
	EEPROM.get(0, Kp);
	address = address + sizeof(float);
	EEPROM.get(address, Ki);
	address = address + sizeof(float);
	EEPROM.get(address, Kd);

	Serial.begin(9600);
	prox.initThreeSensors();
	uint16_t *bLevels = new uint16_t[2*BLEVELS];
	for (int i = 0, k=0; i < BLEVELS; i++)
	{
		bLevels[k] = i * (MAXBRIGHTNESS / BLEVELS);
		k++;
		bLevels[k] = i * (MAXBRIGHTNESS / BLEVELS);
		k++;
	}
	prox.setBrightnessLevels(bLevels, 2*BLEVELS);
	
	String paramValues = String(Ki, 1) + " " + String(Kd, 1);
	lcd.clear();
	lcdPrint(String(Kp, 1), paramValues);
	A.waitForButton();
	lcdPrint("B - Run", "C - Para");
	while (1)
	{
		if (B.getSingleDebouncedPress())
		{
			startMusic();
			break;
		}
		else if (C.getSingleDebouncedPress())
		{
			change_param();
			break;
		}
	}
	setPoint = 30; //calc_set_point();

}


void printReadingsToSerial()
{

	sprintf(pBuffer, "%d %d %d %d %d %d\n",
		prox.countsLeftWithLeftLeds(),
		prox.countsLeftWithRightLeds(),
		prox.countsFrontWithLeftLeds(),
		prox.countsFrontWithRightLeds(),
		prox.countsRightWithLeftLeds(),
		prox.countsRightWithRightLeds()
		);
	Serial.print(pBuffer);
}

void read_sensors()
{
	prox.read();
	countsArray[0] = prox.countsLeftWithLeftLeds();
	countsArray[1] = prox.countsLeftWithRightLeds();
	countsArray[2] = prox.countsFrontWithLeftLeds();
	countsArray[3] = prox.countsFrontWithRightLeds();
	countsArray[4] = prox.countsRightWithLeftLeds();
	countsArray[5] = prox.countsRightWithRightLeds();
}

void test()
{
	long int val = (long int)(exp(FR*FR - RR*RR) * 100000) % 100000;
	memset(pBuffer, '\0', sizeof(pBuffer));
	sprintf(pBuffer, "\t %5d", val);
	Serial.println(pBuffer);
}

int func(int x, int y)
{
	return x + 2 * y + FL;
}

int calc_set_point()
{
	int val = 0;
	lcd.clear();
	lcd.write("Press A");
	A.waitForButton();
	lcd.clear();
	lcd.write("Calibert");
	for (int i = 0; i < 10; i++)
	{
		read_sensors();
		val += RR;
	}
	val = val / 10;
	lcd.clear();
	lcd.print("Press A");
	lcd.gotoXY(0, 1);
	lcd.print(val);
	A.waitForButton();
	return val;
}

void PID(int val, int set)
{
	error = val - set;
	control = Kp*error + Kd*lastError + Ki*sumError;
	lastError = error;
	if (error = 0)
		sumError = 0;
	else
		sumError = sumError + error;
	lcd.clear();
	lcd.print(control);
}

void follow_right_wall()
{
	PID(func(RR,FR), setPoint);
	run();
}

void checkRange(int *l, int *r)
{
	if (*l > 400)
		*l = 400;
	else if (*l < -400)
		*l = -400;

	if (*r > 400)
		*r = 400;
	else if (*r < -400)
		*r = -400;
}

void run()
{
	lcd.clear();
	if (control > 0)
	{
		int l, r;
		l = LEFTSPEED - control;
		r = RIGHTSPEED + control;
		checkRange(&l, &r);
		m.setLeftSpeed(l);
		m.setRightSpeed(r);
		lcd.print(l);
		lcd.gotoXY(0, 1);
		lcd.print(r);
	}
	else if (control == 0)
	{
		
		m.setLeftSpeed(LEFTSPEED);
		m.setRightSpeed(RIGHTSPEED);
		lcd.print("Front");
	}
	else
	{
		control = control*-1;
		int l, r;
		r = RIGHTSPEED - control;
		l = LEFTSPEED + control;
		checkRange(&l, &r);
		m.setRightSpeed(r);
		m.setLeftSpeed(l);
		lcd.print(l);
		lcd.gotoXY(0, 1);
		lcd.print(r);
	}

}

void motor_test()
{
	m.setLeftSpeed(150);
	m.setRightSpeed(50);
}

void loop()
{
	/*
	* Populate the countsArray
	* Get an equation that normalizes the object position to a particular area
	*/

	read_sensors();
	//printReadingsToSerial();
	follow_right_wall();
	//motor_test();
}