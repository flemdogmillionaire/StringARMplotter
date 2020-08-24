#include "MachineDefs.h"
//#define StepDirDrivers
//current stepper position in step sequence (not global position)
long currLeftPos = 0;
long currRightPos = 0;
unsigned long clockmicros = 0;
//byte stepSequence[8] = {B1000, B1100, B0100, B0110, B0010, B0011, B0001, B1001};
byte stepSequence[8] = {B1001, B1100, B1100, B0110, B0110, B0011, B0011, B1001};
int microstepsL = 1;
int microstepsR = 1;
byte leftPins[4]  = {LEFT_STEP_PIN_1,  LEFT_STEP_PIN_2,  LEFT_STEP_PIN_3,  LEFT_STEP_PIN_4};
byte rightPins[4] = {RIGHT_STEP_PIN_1, RIGHT_STEP_PIN_2, RIGHT_STEP_PIN_3, RIGHT_STEP_PIN_4};

//AccelStepper left(AccelStepper::DRIVER, leftPins[0], leftPins[1]);
//AccelStepper right(AccelStepper::DRIVER, rightPins[0], rightPins[1]);
//MultiStepper steppers;

long leftarr[ARRAY_SIZE+1];
long rightarr[ARRAY_SIZE+1];
bool FSarr[ARRAY_SIZE+1];
bool SSarr[ARRAY_SIZE+1];
double DirArr[ARRAY_SIZE + 1];
long NSarr[ARRAY_SIZE + 1];
long TSarr[ARRAY_SIZE + 1];
double DFarr[ARRAY_SIZE + 1];
float leftPerStep = 0;
float rightPerStep = 0;
long currentLeftRealSteps = 1000 * stepsPerMM;
long currentRightRealSteps = 1000 * stepsPerMM;

bool stepped = false;
bool accStopped = false;
bool accStopped2 = false;
//shortest reliable delay seems to be around 600 (1220 used previously) also these are only for non-lib speeds
#define MIN_DELAY 132
#define MAN_DELAY 500 
#define THR_DELAY 330
// microstep threshold delay
#define MAX_DELAY 2048
unsigned long periodicPrintSpdCtr = 0;
#define SPEED_PRINT_PERIOD 20000 //how many speed adjustments before speed is printed
//Allowed speed in steps/us
#define MAX_SPEED (1.0/MIN_DELAY)
#define MIN_SPEED (1.0/MAX_DELAY)
#define MAN_SPEED (1.0/MAN_DELAY)
#define THR_SPEED (1.0/THR_DELAY)

//Current speed in steps / us
float currentSpeed = MAN_SPEED; 

long numSteps3 = 0;

// time (us) until full speed is reached
#define FULL_SPEED_TIME 330000
//period (us) after which speed is adjusted
#define SPADJ_PERIOD 55
//acceleration in steps/us/us.
#define D_SPEED ((MAX_SPEED-MIN_SPEED)/(FULL_SPEED_TIME))\

bool forceStopComing = false;

//legacy compatibility
#define FULL_SPEED_DIST fsdist

//disable steppers after half a second of inactivity
#define DISABLE_TIMEOUT 500000
unsigned long lastStepChangeL = 0;
unsigned long lastStepChangeR = 0;


bool Ldir = 0;
bool Rdir = 1;

void setupStep()
{
	fsdist = (MAX_SPEED * MAX_SPEED - MIN_SPEED * MIN_SPEED)/2/D_SPEED / stepsPerMM;
	for (int pin = 0; pin < 4; pin++) {
		pinMode(leftPins[pin], OUTPUT);
		pinMode(rightPins[pin], OUTPUT);
	}
	for (int i = 0; i < ARRAY_SIZE; i++) {
		leftarr[i] = rightarr[i] = FSarr[i] = 0;
	}
	//left.setMaxSpeed(30* stepsPerMM);
	
	
	//right.setMaxSpeed(30 * stepsPerMM);
	
	//steppers.addStepper(left);
	//steppers.addStepper(right);
}
int AccStIdx = 0;
bool DecremAccIdx = false;
static float prevDir;
static long leftSteps=0;
static long rightSteps=0;
//static bool accStop=true; //did we break the last segment

#define ACC_DIR_THRESHOLD PI/4
//#define DIR_CHANGE_WAIT 1000

bool farEnoughDontStop = false;
bool accStop = false;
float diff = 0;
//float diffOut = 0;
double startDir = 0;
long StepsTilStop = 0;
long oldStepsTilStop = 0;
long oldNumSteps = 0;
void stepCalc(long nextLeftSteps, long nextRightSteps, boolean forceStop, bool first16, bool man, bool subseg)
{
	//Serial.print(" scall ");
#ifdef USE_MOCKED_STEPPERS
 //   printf("step %3ld %3ld\n", nextLeftSteps, nextRightSteps);
#endif
	float cumudiff = 0;
	
	
	StepsTilStop = 0;
	forceStopComing = false;
	farEnoughDontStop = false;
	accStop = false;
	if (first16) {
		for (int i = ARRAY_SIZE - 1; i > 0; i--) {
			leftarr[i] = leftarr[i - 1];
			rightarr[i] = rightarr[i - 1];
			FSarr[i] = FSarr[i - 1];
			SSarr[i] = SSarr[i - 1];
			DirArr[i] = DirArr[i - 1];
			NSarr[i] = NSarr[i - 1];
			TSarr[i] = TSarr[i - 1];
			DFarr[i] = DFarr[i - 1];
		}
		leftarr[0] = nextLeftSteps;
		rightarr[0] = nextRightSteps;
		FSarr[0] = forceStop;
		SSarr[0] = subseg;
		TSarr[0] = 0;
		if (leftarr[0] != 0 || rightarr[0] != 0) DirArr[0] = myAtan2(leftarr[0], rightarr[0]);
		NSarr[0] = max(abs(leftarr[0]), abs(rightarr[0]));
		firstStep = true;
	}
	else {
		if (!man) {
			for (int i = ARRAY_SIZE - 1; i > 0; i--) {
				leftarr[i] = leftarr[i - 1];
				rightarr[i] = rightarr[i - 1];
				FSarr[i] = FSarr[i - 1];
				SSarr[i] = SSarr[i - 1];
				DirArr[i] = DirArr[i - 1];
				NSarr[i] = NSarr[i - 1];
				TSarr[i] = TSarr[i - 1];
				DFarr[i] = DFarr[i - 1];
			}
			leftarr[0] = nextLeftSteps;
			rightarr[0] = nextRightSteps;
			FSarr[0] = forceStop;
			SSarr[0] = subseg;
			TSarr[0] = 0;
			if (leftarr[0] != 0 || rightarr[0] != 0) DirArr[0] = myAtan2(leftarr[0], rightarr[0]);

			NSarr[0] = max(abs(leftarr[0]), abs(rightarr[0]));
			for (int i = 0; i < ARRAY_SIZE - 1; i++) {
				DFarr[i] = abs(DirArr[i] - DirArr[i + 1]);
			}
			
			for (int i = 1; i < ARRAY_SIZE && !farEnoughDontStop && !accStop; i++) {

				//number of steps for this line segment (reminder: everything in this file is post-splitting)
				long numSteps = NSarr[ARRAY_SIZE - i];
				oldNumSteps = numSteps;
				StepsTilStop += numSteps;
				if (StepsTilStop > stepsPerMM*(FULL_SPEED_DIST+1)) {
					farEnoughDontStop = true;
				}
				

				double nextDir = DirArr[ARRAY_SIZE - 1 - i];//atan2(leftarr[ARRAY_SIZE - i], rightarr[ARRAY_SIZE - i]);
				//if (i == 1) startDir = prevDir;
				diff = DFarr[ARRAY_SIZE - 1 - i];

				if (diff > PI) {
					diff = 2 * PI - diff;
				}

				cumudiff = cumudiff + diff;
				//if (i == 1 && stepped) {
				//	stepped = false;
				//	//Serial.println("");
				//   // Serial.print("          DIFF:   "); Serial.print(diff); Serial.print("   "); Serial.print(cumudiff);
				//}
				//check if we need to brake towards end of line segment
				if ((cumudiff > ACC_DIR_THRESHOLD*0.6 || FSarr[ARRAY_SIZE - 1 - i])&&!farEnoughDontStop) { //&& !SSarr[ARRAY_SIZE -1 - i] // || diff > ACC_DIR_THRESHOLD // || (AccStIdx > i - 1 && DecremAccIdx
					//sharp turn, brake
					accStop = true;
					if (FSarr[ARRAY_SIZE - 1 - i]) { 
						forceStopComing = true; 
					}
					farEnoughDontStop = false;
					
				}
				
				
				
				oldStepsTilStop = StepsTilStop;
				TSarr[ARRAY_SIZE - i] = StepsTilStop;

				AccStIdx = i;
				
				//prevDir = nextDir;


			}
			StepsTilStop = 0;
			for (int i = 1; i < AccStIdx; i++) {
				StepsTilStop += NSarr[ARRAY_SIZE - i];
			}

		}
		stepAndSpeedCalc(nextLeftSteps, nextRightSteps, man);
	}
}

bool decel = false;

#define SPADJER (firstStep? SPADJ_PERIOD : spadj_lastcall)
void stepAndSpeedCalc(long nextLeftSteps, long nextRightSteps, bool man){

	

	leftSteps += man ? nextLeftSteps : leftarr[ARRAY_SIZE - 1];
	rightSteps += man ? nextRightSteps : rightarr[ARRAY_SIZE - 1];
	//currentLeftRealSteps += man ? nextLeftSteps : leftarr[ARRAY_SIZE - 1];
	//currentRightRealSteps = man ? nextRightSteps : rightarr[ARRAY_SIZE - 1];
	//long posses[2];
	//posses[0] = currentLeftRealSteps;
	//posses[1] = currentRightRealSteps;
		long numSteps2 = man ? max(abs(nextLeftSteps), abs(nextRightSteps)) : NSarr[ARRAY_SIZE - 1];
		//left.setMaxSpeed(man ? 30 * stepsPerMM : 200 * stepsPerMM);
		//right.setMaxSpeed(man ? 30 * stepsPerMM : 200*stepsPerMM);
		if (numSteps2 > 0) {
			//current logic is to step the fastest moving stepper every iteration of the loop while stepping the slower moving one as the "fraction" is accumulated to more than 1.
			//A better logic calculating timings and running the steppers more asynch from one another might result in a more efficient drive of the slow steppper? In some distant future... the fututre is now old man
			leftPerStep = abs(leftSteps) / (float)numSteps2;
			rightPerStep = abs(rightSteps) / (float)numSteps2;
		
			StepsTilStop = TSarr[ARRAY_SIZE - AccStIdx];

			Ldir = (leftSteps < 0);
			Rdir = (rightSteps < 0);

			//Serial.print(".");
			while (((rightSteps != 0) && (rightSteps < 0) == Rdir || (leftSteps != 0) && (leftSteps < 0) == Ldir)) { 
				
				unsigned long spadj_lastcall = 0;
				if (!man) {
					numSteps3 = max(leftSteps*(Ldir?-1:1),rightSteps*(Ldir ? -1 : 1));
					StepsTilStop -= (numSteps2 - numSteps3);
					numSteps2 = numSteps3;
					if (firstStep) {
						currentSpeed = MIN_SPEED;
						
						farEnoughDontStop = false;
					}
				}
				
				if ((micros() - lastSpeedAdjust) >= SPADJ_PERIOD) {
					spadj_lastcall = micros() - lastSpeedAdjust;
					lastSpeedAdjust = micros();
					if (!man && (!farEnoughDontStop) && currentSpeed*currentSpeed-MIN_SPEED*MIN_SPEED> StepsTilStop*D_SPEED*2) { // || decel
						accStopped = true;
						decel = true;
						//start braking
						if (currentSpeed > MIN_SPEED) {
							currentSpeed = (currentSpeed - D_SPEED * SPADJER);
							//if (AccStIdx == 1) currentSpeed = MIN_SPEED;
						}
						else {
							currentSpeed = min(currentSpeed + D_SPEED * SPADJER, MAX_SPEED);
							//decel = false;
						}
					}
					else if (man) {
						currentSpeed = MAN_SPEED;
						accStopped = false;
					}
					else {
						decel = false;
						currentSpeed = min(currentSpeed + D_SPEED * SPADJER, MAX_SPEED);
						accStopped = false;

					}
					//left.setMaxSpeed(currentSpeed*1000000.0);
					//right.setMaxSpeed(currentSpeed*1000000.0);
					//if (micros()-periodicPrintSpdCtr > SPEED_PRINT_PERIOD) {
					//	periodicPrintSpdCtr = micros();
					//	Serial.print(" Sp ");
					//	Serial.print(currentSpeed * 1000000UL / stepsPerMM);
					//	Serial.print(" St ");
					//	Serial.print(StepsTilStop / stepsPerMM * 10);
					//	Serial.print(" St ");
					//	Serial.print(numSteps3 / stepsPerMM * 10);
					//	//Serial.print(" dA ");
					//	//Serial.print(spadj_lastcall);
					//	Serial.print(" Dir ");
					//	Serial.print(DirArr[ARRAY_SIZE - 1] * 50.0F);
					//	Serial.print(" FHalt ");
					//	Serial.print(forceStopComing ? "110" : "210");
					//	//Serial.print(" FSD ");
					//	//Serial.print(fsdist);
					//	Serial.print(" Pen ");
					//	Serial.println(UniPen ? "010" : "090");

					//	//Serial.print(" ");
					//}
					

					if (firstStep) firstStep = false;
				}
				//while (micros() - clockmicros < ulong(1.0 / currentSpeed)) {
				//	delayMicroseconds(1);
				//}
				//clockmicros = micros();
			
				
				stepRoutine();
				
				//Serial.println(1.0 / currentSpeed);
				//Serial.println(max(abs(leftSteps), abs(rightSteps)));
			}
			//Serial.println(callcount);
			callcount = 0;
			decel = false;
		}
		

	}


//void checkDisableSteppers() { 
//
//   if((micros()-lastStepChange) > DISABLE_TIMEOUT) {     
//     //disable steppers
//     for(int pin=0 ; pin<4 ; pin++) {
//       digitalWrite(leftPins[pin], 0); 
//       digitalWrite(rightPins[pin], 0); 
//     }
//   }
//
//}

bool stepDir(long *steps, long *currPos, byte *pins, int *microsteps, float perStep, bool dir) {
	digitalWrite(pins[0], 0);

	bool step = 0;
	
	if ((*steps == 0 )|| (*steps > 0) == dir) {
		
		return true;
	}

	else {
		if ((currentSpeed * perStep > THR_SPEED)) {
			if ((currentSpeed * perStep > THR_SPEED * 2)) {
				if ((currentSpeed * perStep > THR_SPEED * 4)) {// whole step (a little risky, but really fast feedrates)
					*microsteps = 8;
					digitalWrite(pins[2], 0);
					digitalWrite(pins[3], 0);
					step = 1;

				}
				else { // 1/2 step
					*microsteps = 4;
					digitalWrite(pins[2], 1);
					digitalWrite(pins[3], 0);
					step = 1;

				}
			}
			else { // 1/4 step
				*microsteps = 2;
				digitalWrite(pins[2], 0);
				digitalWrite(pins[3], 1);
				step = 1;

			}
		}
		else { // 1/8th step
			*microsteps = 1;
			digitalWrite(pins[2], 1);
			digitalWrite(pins[3], 1);
			step = 1;
		}



		

			if (*steps > 0) {
				(*steps)= *steps -(*microsteps);

				(*currPos)= *currPos +(*microsteps);
				digitalWrite(pins[1], 1);
			}
			else if (*steps < 0) {
				(*steps)= *steps + (*microsteps);

				(*currPos)= *currPos - (*microsteps);
				digitalWrite(pins[1], 0);
			}


			delayMicroseconds(3);
			digitalWrite(pins[0], 1);
			delayMicroseconds(5);
			digitalWrite(pins[0], 0);
			

		
	}
	return false;

}


bool stepRoutine() {
	bool endExit = 0;
	while (!endExit) {
		
		if (micros() - lastStepChangeL >= ulong(1.0 / (currentSpeed*leftPerStep))* microstepsL ) {
			stepDir(&leftSteps, &currLeftPos, leftPins, &microstepsL, leftPerStep, Ldir);
			lastStepChangeL = micros();
			endExit = 1;
			//Serial.println(leftSteps);
		}
		if (micros() - lastStepChangeR >= ulong(1.0 / (currentSpeed*rightPerStep))* microstepsR ) {
			stepDir(&rightSteps, &currRightPos, rightPins, &microstepsR, rightPerStep, Rdir);
			lastStepChangeR = micros();
			endExit = 1;
			//Serial.println(rightSteps);
		}
	}
	//left.run();
	//right.run();
	return true;
}

double myAtan2(double x, double y) {
	double speed = max(abs(x), abs(y));
	double trail = min(abs(x), abs(y));
	double ratio = trail / speed;

	if (x > 0) {
		if (y > 0) {
			if (x > y) {
				return ((PI / 2)*(1 - ratio/2));
			}
			else {
				return ((PI / 2)*(ratio / 2));
			}

		}
		else if (y == 0) {
			return PI / 2;
		}
		else {
			if (x > -y) {
				return ((PI/2)+(PI / 2)*(1 - ratio / 2));
			}
			else {
				return ((PI/2)+(PI / 2)*(ratio / 2));
			}
		}
	}
	else if (x == 0) {
		if (y > 0) {
			return 0;
		}
		else {
			return PI;
		}
	}
	else if (x < 0) {
		if (y > 0) {
			if (-x > y) {
				return -((PI / 2)*(1 - ratio / 2));
			}
			else {
				return -((PI / 2)*(ratio / 2));
			}

		}
		else if (y == 0) {
			return -PI / 2;
		}
		else {
			if (-x > -y) {
				return -((PI / 2) + (PI / 2)*(1 - ratio / 2));
			}
			else {
				return -((PI / 2) + (PI / 2)*(ratio / 2));
			}
		}
	}

}