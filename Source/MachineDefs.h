
#define SERIAL_DEBUG

#ifdef SERIAL_DEBUG
#define SER_PRINT(X) Serial.print((X))
#define SER_PRINT2(X,Y) Serial.print((X),(Y))
#define SER_PRINTLN(X) Serial.println((X))
#define SER_PRINTLN2(X,Y) Serial.println((X),(Y))
#else
#define SER_PRINT(X)
#define SER_PRINT2(X,Y)
#define SER_PRINTLN(X)
#define SER_PRINTLN2(X,Y)
#endif

//define this if plotter has a battery voltage feedback connected
//#define HAS_BATTERY_MEASUREMENT 
 
//which remote control to use
//#define PIONEER_DVD_REMOTE
//#define NO_REMOTE

//servo pin
#define SERVO_PIN 38
#define SERVO2_PIN 39

//servo values for pen up/down
#define PEN_UP 110
#define PEN_DOWN 60



//IR-receiver pin
#define RECV_PIN  A0
 
//stepper motor pins
#define LEFT_STEP_PIN_1 23 //also step for stepDir
#define LEFT_STEP_PIN_2 25 //also dir for stepDir
#define LEFT_STEP_PIN_3 31 // MS1
#define LEFT_STEP_PIN_4 33 // MS2
#define RIGHT_STEP_PIN_1 27 //also step for stepDir
#define RIGHT_STEP_PIN_2 29 //also dir for stepDir
#define RIGHT_STEP_PIN_3 35 // MS1
#define RIGHT_STEP_PIN_4 37 // MS2

#include <SD.h>
//#include <AccelStepper.h>
//#include <MultiStepper.h>

#define StepDirDrivers
#define ARRAY_SIZE 48