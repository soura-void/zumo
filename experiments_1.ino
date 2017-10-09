#include <Wire.h>
#include <Zumo32U4.h>
#include <math.h>

#define LL	countsArray[0]
#define LR	countsArray[1]
#define FL	countsArray[2]
#define FR	countsArray[3]
#define RL	countsArray[4]
#define RR	countsArray[5]
#define C	2

Zumo32U4LCD lcd;
Zumo32U4ProximitySensors prox;

int countsArray[6];

void setup()
{
	Serial.begin(9600);
	prox.initThreeSensors();
}


void printReadingsToSerial()
{
	static char buffer[80];
	sprintf(buffer, "%d %d %d %d %d %d",
		prox.countsLeftWithLeftLeds(),
		prox.countsLeftWithRightLeds(),
		prox.countsFrontWithLeftLeds(),
		prox.countsFrontWithRightLeds(),
		prox.countsRightWithLeftLeds(),
		prox.countsRightWithRightLeds()
		);
	Serial.print(buffer);
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
	double rad = (FR*FR - C - RR*RR) / (2 * C*RR);
	Serial.print("\t");
	Serial.println(rad);
}

void loop()
{
	/*
	* Populate the countsArray
	* Get an equation that normalizes the object position to a particular area
	*/
	read_sensors();
	printReadingsToSerial();
	test();
	delay(100);
}