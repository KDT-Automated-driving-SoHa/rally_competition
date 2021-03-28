#include "wiring_private.h"

#define MAX_DISTANCE 140*58.2 
#define PING_INTERVAL 33*5

#define EA 5

int trig[EA] = {4, 5, 8, 10, 12};
int echo[EA] = {3, 6, 7,  9, 11};

unsigned long pingTimer;
long around_time[EA];
long distance_cm[EA];
int num;

unsigned long pulseIn(uint8_t pin, uint8_t state, unsigned long timeout)
{
	uint8_t bit = digitalPinToBitMask(pin);
	uint8_t port = digitalPinToPort(pin);
	uint8_t stateMask = (state ? bit : 0);

	unsigned long maxloops = microsecondsToClockCycles(timeout)/16;

	unsigned long width = countPulseASM(portInputRegister(port), bit, stateMask, maxloops);

	if (width)
		return clockCyclesToMicroseconds(width * 16 + 16);
	else
		return MAX_DISTANCE;
}

void setup() 
{
  Serial.begin(38400);

  for (uint8_t i=0; i<EA; i++)
  {
    pinMode(trig[i], OUTPUT);
    pinMode(echo[i], INPUT);
  }

  pingTimer = millis();
}

void loop() 
{
  if (millis() >= pingTimer) 
  {   
    pingTimer += PING_INTERVAL;
    around_time[num] = trig_ultra(trig[num], echo[num]);
    num++;      
  }
  if (num>(EA-1)) 
  { 
    num=0; 
	  oneSensorCycle();   
	  pingTimer = millis();
  }
}

long trig_ultra(int TRIG,int ECHO)
{
  long receive_time_ms;

  digitalWrite(TRIG, LOW); 
  delayMicroseconds(2); 
  digitalWrite(TRIG, HIGH); 
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);

  receive_time_ms = pulseIn(ECHO, HIGH, MAX_DISTANCE);

  return(receive_time_ms);
}

void oneSensorCycle() 
{ 
  for (uint8_t i=0; i<EA; i++) 
  {
      distance_cm[i] = around_time[i] / 58.2;
      Serial.print(distance_cm[i]);
      Serial.print(i+1);
      Serial.println();
  }
}
