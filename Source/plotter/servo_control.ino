#include "MachineDefs.h"

#include "MachineDefs.h"
#include <Servo.h>  
Servo myservo;
Servo myservo2;

bool downarr[ARRAY_SIZE+1];
bool fastarr[ARRAY_SIZE+1];
//disable to preserve space
#define USE_SMOOTH_SERVO

static int oldPos = 0;

void setupServo() 
{
  myservo.attach(SERVO_PIN);
  myservo2.attach(SERVO2_PIN);
  movePenInternal(false, true);
  for (int i = 0; i < ARRAY_SIZE; i++) {
	  downarr[i] = fastarr[i] = 0;
  }
  makePenNoise(2);
}

void makePenNoise(int n)
{
  int p = oldPos;  
  for(int i=0; i<n; i++) {
    myservo.write(int(p+5));    
	myservo2.write(180-int(p + 5));

    delay(100); 
    myservo.write(int(p));    
	myservo2.write(180-int(p));
    delay(100); 
  }  
}

void testPen()
{
    movePenInternal(true, false);
    delay(1000); 
    movePenInternal(false, false);
}
void movePen(bool down, bool fast, bool first16) {
	if (first16) {
		for (int i = ARRAY_SIZE - 1; i > 0; i--) {
			downarr[i] = downarr[i - 1];
			fastarr[i] = fastarr[i - 1];
		}
		downarr[0] = down;
		fastarr[0] = fast;
	}
	else {

		for (int i = ARRAY_SIZE - 1; i > 0; i--) {
			downarr[i] = downarr[i - 1];
			fastarr[i] = fastarr[i - 1];
		}
		downarr[0] = down;
		fastarr[0] = fast;
		movePenInternal(downarr[ARRAY_SIZE - 2], fastarr[ARRAY_SIZE - 2]);
	}
}
void movePenInternal(boolean down, boolean fast){
	UniPen = down;
	//Serial.print(down);
	//Serial.println("down");
	int pos = down ? PEN_DOWN : PEN_UP;
	if (pos != oldPos) {
		
#ifdef USE_SMOOTH_SERVO    
		if (fast || !down) {
			myservo.write(int(pos));
			myservo2.write(180 - int(pos));
			delay(20);
			lastSpeedAdjust = micros();
		}
		else {
			for (float i = 0; i <= 1.0; i += 0.02) {
				float i2 = (1 - cos(i*3.14)) / 2;
				float tmpPos = (oldPos*(1.0 - i2) + (pos*i2));
				myservo.write(int(tmpPos));
				myservo2.write(180 - int(tmpPos));
				delay(4);
			}
		}
#else //USE_SMOOTH_SERVO
		myservo.write((int)pos);
#endif //USE_SMOOTH_SERVO

		oldPos = pos;
		lastSpeedAdjust = micros();

	}
}

