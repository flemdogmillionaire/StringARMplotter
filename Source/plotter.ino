#include <ArduinoBlue.h>
#include <SD.h>
bool UniPen = false;
#include "MachineDefs.h"
int callcount = 0;
//circumference of spool
#define spoolCirc 32.0 

//steps per full rotation 
#define stepsPerRotation 1600

//number of steps for each full rotation
#define stepsPerMM (stepsPerRotation/spoolCirc)

//longest allowed line segment before splitting
#define maxSegmentLength 11
bool arrayFillTime = true;
long state = 0;
long state2 = 0;
long stoppedAt = 0; //save of where we stopped, to allow resuming
int currentPlot = 0;
bool resumePlot = false;

bool firstStep = false;
unsigned long lastSpeedAdjust = 0;

//these numbers are left from early versions where setup was done by having exactly 1m of string reeled out on both left and right and a fix disparity of 1m between anchors. This is now set up by user instead.
long disparity = 1000;  //distance between anchor points 
long currentLeftSteps  = 1000*stepsPerMM; 
long currentRightSteps = 1000*stepsPerMM;
float centerX = 500; //starting x pos
float centerY = 866; //starting x pos

bool stopPressed = false;

int program = 0; //0 is start program, responding to IR 
double fsdist;
//manual control
int manualLeft = 0, manualRight = 0;
float printSize = 1.0;
bool continousManualDrive = false;

//bookkeeping for sub segmenting lines
static float prevX = 0;
static float prevY = 0;
static int currentSubSegment = 0;
int subSegments;
//void storePositionInEEPROM();
static int lastReadPoint = -1;

void setup()
{  
#ifdef USE_DATA_FROM_DISK
  //store constant values into mock eeprom, wait 1sec first to allow storage
  delayMicroseconds(1000000);  
  storePositionInEEPROM();
  printf("[disparity, currentLeftSteps, currentRightSteps, centerX, centerY]\n");
  printf("%ld, %ld, %ld, %f, %f\n", disparity, currentLeftSteps, currentRightSteps, centerX, centerY);  
#endif
  
#ifdef SERIAL_DEBUG
  //Initialize serial and wait for port to open:
    Serial.begin(256000); 
    SER_PRINTLN("Setup");
#endif
  //initialize SD card
  setupData();

  //initialize IR (actually TFT/CTP)  
  setupIR();

  //initialize steppers
  setupStep();

  //initialize servo
  setupServo();
  

 #ifdef SERIAL_DEBUG
  Serial.print("Disparity=");
  Serial.println(disparity);
  Serial.print("FullSpeedDist=");
  Serial.println(fsdist);
#endif

#ifdef NO_REMOTE
   //fake start a print since we dont have IR control
   printSize = 1;
   program = 1; //start print
   currentPlot = 4; 
#endif  
}

unsigned long lastEEPromStore = 0;


void setOrigo() {
    float currentLeft  = currentLeftSteps / stepsPerMM;
    float currentRight = currentRightSteps / stepsPerMM;
    float tmp1 = (currentRight*currentRight-disparity*disparity-currentLeft*currentLeft);
    float tmp2 = (-2*currentLeft*disparity);
    float a = acos(tmp1/tmp2);    
    centerX = currentLeft*cos(a);
    centerY = currentLeft*sin(a);
}

static bool prevPen=0;

extern unsigned long lastBatteryLog;
//int prepenchanges = 0;
void loop()
{

	float tmpX, tmpY;
	bool tmpPen;

	
	
	if (program == 0) {
		readIR(true);
		float left = (manualLeft / spoolCirc) * 360.0;
		float right = (manualRight / spoolCirc) * 360.0;

		if (manualLeft != 0 || manualRight != 0) {
			currentLeftSteps += manualLeft * stepsPerMM;
			currentRightSteps += manualRight * stepsPerMM;
			
			stepCalc(manualLeft*stepsPerMM, manualRight*stepsPerMM, false, false, true, false);
			setOrigo();
			
		}

		if (stopPressed || (!continousManualDrive)) {
			manualLeft = manualRight = 0;
			stopPressed = false;
		}
	}
	else {
		readIR(false);
		//stepRoutine();
		callcount++;
		if (!getData(currentPlot, state, &tmpX, &tmpY, &tmpPen) || stopPressed) {
			//stepRoutine();
			callcount++;
			stoppedAt = stopPressed ? state : 0;

			//reached the end, go back to manual mode
			state = 0;
			state2 = 0;
			program = 0;
			resumePlot = false; //make sure to not end up in loop if plot cannot be resumed due to missing file or corrupt data        

			movePen(false, false, false);
			stepCalc(0, 0, prevPen != false, false, false, false); //flush out last line segment
			if (!stopPressed) {
				for (int i = 0; i < (ARRAY_SIZE - 1); i++) {
					movePen(false, false, false);
					stepCalc(0, 0, false, false, false, false);//finish last 16 points
				}
			}
			//stop with pen up        
			movePenInternal(false, false);

			SER_PRINTLN("Plot done");

			//store current position in eeprom 
			//storePositionInEEPROM();
		}
		else {
			
			//stepRoutine();
			//if(resumePlot) {
			if (resumePlot && false) {
				//just skip points until we are at the point where we stopped
				state++;
				prevX = tmpX * printSize;
				prevY = tmpY * printSize;
				//resumePlot = false;
				Serial.print("res");
			}
			else {
				resumePlot = false;
				float nextX, nextY;
				if (tmpX != 0.0f && tmpY != 0.0f) {
					nextX = tmpX * printSize;
					nextY = tmpY * printSize;
				}
				else {

				}
				boolean nextPen = tmpPen;
				boolean advancePoint = true;

				if (state > 0) { //don't try to split first segment         
					float dx = nextX - prevX;
					float dy = nextY - prevY;
					float len = sqrt(dx * dx + dy * dy);
					//stepRoutine();
					callcount++;
					if (len > maxSegmentLength) {
						//split segment
						subSegments = 1 + (int)(len / maxSegmentLength);
						if (currentSubSegment == subSegments) {
							//last segment
							currentSubSegment = 0; //reset                                                                        
						}
						else {
							advancePoint = false; //stay on same point        
							currentSubSegment++;

							nextX = prevX + dx * currentSubSegment / subSegments;
							nextY = prevY + dy * currentSubSegment / subSegments;
						}
					}
					
				}
				/*else {
					step(1, 1, true, true, false);

				}*/

				if (advancePoint) {
					state = state + 1; //next point
					callcount++;
					//Serial.println("adv");
					arrayFillTime = state < (ARRAY_SIZE - 1); //extra -1 needed for that zero step a few lines up
					prevX = nextX;
					prevY = nextY;
				}

				float xL = nextX + centerX;
				float xR = nextX + centerX - disparity;
				float y = nextY + centerY;

				long newLeft = sqrt(xL*xL + y * y)*stepsPerMM;
				//stepRoutine();
				callcount++;
				long newRight = sqrt(xR*xR + y * y)*stepsPerMM;
				//stepRoutine();
				callcount++;

				long dLeft = (newLeft - currentLeftSteps);
				long dRight = (newRight - currentRightSteps);

				currentLeftSteps = newLeft;
				currentRightSteps = newRight;
				// Serial.print(dLeft); Serial.print("  "); Serial.println(dRight);
				if (((dLeft == 0) && (dRight == 0))) {
					//Serial.println("nomove");
					//no move, ignore
					callcount = 0;
				}
				else {
					state2 = state2 + 1;
					arrayFillTime = state2 < (ARRAY_SIZE - 1);
					// if (prevPen != nextPen) {
					   //  movePen(prevPen, false, first16); //adjust pen as necessary 
						// if (first16) prepenchanges++;
						 //first16 = state < 15 - prepenchanges;
						 //step(0, 0, true, first16, false); //don't move steppers
					 //}
					movePen(prevPen, false, arrayFillTime); //adjust pen as necessary (could be same as a few lines before, but whatever) 
					stepCalc(dLeft, dRight, prevPen != nextPen, arrayFillTime, false, subSegments > 1); //move steppers
					prevPen = nextPen;
					//Serial.print("STEP");
				}
			}
		}
	}

	//check and disable steppers if idle
   // checkDisableSteppers();
}
