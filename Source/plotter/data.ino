#include "MachineDefs.h"
 
#ifndef USE_DATA_FROM_DISK

#include <avr/pgmspace.h>
#include <SPI.h>


#define SD_CS SDCARD_SS_PIN
#endif
bool relGNPS = false;
bool hor = false;
bool ver = false;

bool getDataInternal(int plotNo, int point, float *x, float* y, int* pen);
bool getSvgData(int plotNo, int point, float *x, float* y, int* pen);
//bool getMemoryData(int plotNo, int point, int *x, int* y, int* pen);

static int currentlySelectedPlot = -1;

static File svgFile;

static int reachedTheEndAt = -10;

bool getData(int plotNo, int point, float *x, float* y, bool* pen)
{
  if(point == (reachedTheEndAt+1)) {
    reachedTheEndAt = -10; 
    return false;
  }
  else {
    if(!getDataInternal(plotNo, point, x, y, pen)) {
      //return to origo on end
		*x = *y = 0.0;
      *pen = 0;
      reachedTheEndAt = point; 
    }    
    return true;
  }
}

bool getDataInternal(int plotNo, int point, float *x, float* y, bool* pen)
{  
  if(currentlySelectedPlot != plotNo) {
    //first call, check if we have plot stored on SD
    
    if(svgFile) {
      svgFile.close();
      svgFile = SD.open("dummy_fail_this_open", FILE_READ);;      
    }
    
    char* svgName = (char*)"1.ngc";
    //svgName[0] = '0'+plotNo;    
    svgFile = SD.open(svgName, FILE_READ);
    if(svgFile) {
        //found svg
      SER_PRINT("Found: ");      
      SER_PRINTLN(svgName);      
    }
    else {
      makePenNoise(3);
      SER_PRINT("No such file: ");      
      SER_PRINTLN(svgName);      
      return false;
    }

    currentlySelectedPlot = plotNo;
  }
  
  if(svgFile) {
    return getSvgData(plotNo, point, x, y, pen);    
  }
  else
  {
    return false;
  }
}

#ifdef HAS_BATTERY_MEASUREMENT

static File batteryFile;
unsigned long lastBatteryLog = 0;

static float batteryAverage=800; //just start with something above threshold
#define BATTERY_THRESHOLD 650

void logBattery(int secsSinceStart) {
#ifndef USE_DATA_FROM_DISK  
  int batt = analogRead(0);
  batteryFile.print(secsSinceStart);
  batteryFile.print(' ');
  batteryFile.println(batt);
  batteryFile.flush();
  
  batteryAverage = 0.99*batteryAverage + 0.01*batt;
  if(batteryAverage < BATTERY_THRESHOLD) {
    batteryFile.println("LOW BATTERY!");
    batteryFile.flush();
    stopPressed = true; //stop plot and persist state to eeprom to allow resume after battery has been changed
  }  
#endif
}

#endif //HAS_BATTERY_MEASUREMENT

void setupData()
{
  // make sure that the default chip select pin is set to
  // output, even if you don't use it:
	
  pinMode(SD_CS, OUTPUT);

  // see if the card is present and can be initialized:
  if (!SD.begin(SPI_QUARTER_SPEED, SD_CS)) {
    SER_PRINTLN("SD fail");
    // don't do anything more:
    return;
  }  
#ifdef HAS_BATTERY_MEASUREMENT  
#ifndef USE_DATA_FROM_DISK
  batteryFile = SD.open("battery.log", FILE_WRITE);;      
#endif
#endif //HAS_BATTERY_MEASUREMENT
}

// **************** Svg path ***************************


bool
seekTo(char* pattern)
{
  //yes, this is flawed string matching
  char* tmp = pattern;

  while(*tmp != '\0') {
      char c = svgFile.read();
      if(c < 10 || c == '$') {
         return false;
	
      }
      if(c == *tmp) {
        tmp++;
      }
      else {
        tmp = pattern;        
      }      
  }
  return true;  
}

bool
seekToPathStart(bool rewindFile) {
  if(rewindFile) {
    svgFile.seek(0);  //rewind (we could have paused)
	//Serial.println("REWIND");
  }
    if(!seekTo((char*)"G00")) {
//      SER_PRINTLN("No <path> tag");    
      return false;
    }
    
	svgFile.seek(svgFile.position() - 3);
	
  return true;  
}

//terribly sorry about this, but didn't have program space for using atof
//HA! I'm using a Due!
bool
readFloat(float* ret) {
	char tmp[20];
	float f = 0;
	bool pastPoint = false;
	bool exp = 0;
	bool is_0 = false;
	float div = 1;
	int i = 0;
	while (true) {
		for (i = 0; i < 20; i++) {
			tmp[i] = svgFile.read();

			if (tmp[i] < 10 || tmp[i] == 26 || tmp[i] == '$') {
				return false;
			}

			else if ((tmp[i] >= '0') && (tmp[i] <= '9')) {
				if (exp > 0) {
					for (int e = (tmp[i] - '0'); e > 0; e--) {
						div = div / 10;
					}
				}
				if (exp < 0) {
					for (int e = (tmp[i] - '0'); e > 0; e--) {
						div = div * 10;
					}
				}
				else if (div < 10000) {
					f = f * 10 + (tmp[i] - '0');
					if (pastPoint) {
						div = div * 10;
					}
				}
				else {
					//only care for four decimals
				}
			}
			else if (tmp[i] == '.') {
				pastPoint = true;
			}
			else if (tmp[i] == '-') {
				if (exp != 0) {
					exp = -1;
				}
				else {
					div = -1;
				}
			}
			else if (tmp[i] == 'e') {
				exp = 1;
			}
			else {
				break;
			}
		}
		//Serial.println(tmp);
		//*ret = strtod(tmp, NULL);
		*ret = atof(tmp);
		if (f == 0) {				//|| tmp[0, 1, 2, 3, 4, 5] == '0','.','0','0','0','0')'
			is_0 = true;
			//Serial.print(" is_0 ");
		}
		if (*ret == 0.0f) {
			
			if (is_0) return true;
			Serial.println(tmp);
			Serial.print(" NaN_exit ");
			return false;

		}
		if (*ret > 10000 || *ret < -10000) {
			//retry line parse
			Serial.println("WARNING_OVF");
			Serial.println(tmp);
			svgFile.seek(svgFile.position() - i);

		}
		else {
			break;
		}
		//*ret = f;
		//Serial.println(*ret);
	}
  return true;
}
float holder = 0;
float holderY = 0;
bool
getNextPathSegment(float *x, float *y, bool *line, bool first)
{
	char c;
	char gnum;
	static float lastX, lastY, lastZ;
	float UselessValue;
	bool abetween = false;
	bool Xbet = false;
	bool Ybet = false;
	bool Zbreak = false;
	while (true) {
		if (first) {
			lastX = lastY = lastZ = UselessValue = 0.0;
		}

		c = svgFile.read();
		//    fputc(c,stdout);
		//Serial.print(c);
		if (c == '$') {
			return false;
		}
		if (c == 'G') {

			if (svgFile.read() == '0') {
				gnum = svgFile.read();
				if (gnum == '0') {
					//eat a digit byte
					*line = 0;
				}
				else if (gnum == '1') {
					*line = 1;
				}
				//svgFile.read(); //eat a space byte

			}
			else {
				seekToPathStart(false);
				//Serial.println("G-seek");
			}
		}
		else if (c == 'X') {
			svgFile.read(); //eat a space byte
			if (!readFloat(&holder)) {
				return false;
			}
			if (!( holder < 0.01f)) {
				*x = holder;
				lastX = holder;
			}
			else {
				*x = lastX;
			}
			abetween = false;
			Xbet = true;
		}
		else if (c == 'Y') {
			svgFile.read(); //eat a space byte
			if (!readFloat(&holderY)) {
				return false;
			}
			//Serial.print(holderY);
			if (!(-0.01f < holderY)) {
				*y = holderY;
				lastY = holderY;
			}
			else {
				*y = lastY;
			}
			abetween = false;
			Ybet = true;
		}
		else if (c == 'Z') {
			svgFile.read(); //eat a space byte
			if (!readFloat(&lastZ)) {
				return false;
			}
			if (lastZ < 0) {
				if (!abetween) {
					*line = 1;
				}
			}
			else {
				*line = 0;
			}
			abetween = false;
			if (Xbet && Ybet) Zbreak = true;
			Xbet = Ybet = false;
			if (Zbreak) break;
		}
		else if (c == 'F' || c == 'A') {
			if (c == 'A') {
				abetween = true;
			}
			svgFile.read(); //eat a space byte
			if (!readFloat(&UselessValue)) {
				return false;
			}
		}
		else if (((c >= '0') && (c <= '9')) || c == '-' || c == ' ') {
			if (c == ' ') {
				//svgFile.read();
			}
			else {
				//svgFile.seek(svgFile.position() - 2);
				continue;
			}
		}
		else if (c == 10 || c == 13) {
			svgFile.read(); //eat a linefeed byte
			//Serial.println(" CRLF");
			//if (svgFile.read() == 'G') {
				//svgFile.seek(svgFile.position() - 1);
				//break;
			//}
			//else {
			//	svgFile.seek(svgFile.position() - 1);
			//	continue;
			//}

		}
		else if (c == '(' || c == 'P') {
			if (!seekTo((char*)"G")) {
				return false;
			}
			//break;
		}
		else {
			//reached end, look for another G00
			if (seekToPathStart(false)) {
				first = true;
				continue;
			}
			else {
				return false;
			}
		}
		//break;
	}




	//lastX = *x;
	//lastY = *y;
	//Serial.print(*x);
	//Serial.print("   ");
	//Serial.println(*y);
	//rewind one byte that was eaten by last float read  
	svgFile.seek(svgFile.position() - 1);

	return true;
}

static float min_x = 100000000.0;
static float min_y = 100000000.0;
static float scaleFactor = 1.0;

static float lastX,lastY;
static int lastPen;

bool getSvgData(int plotNo, int point, float *x, float* y, bool* pen)
{
  if(point == 0) {
    long pathPosition;
    long segments = 0;
    float max_x, max_y;
    
    lastReadPoint = -1;
    
    //first read, get dimensions
    if(!seekToPathStart(true)) {      
      SER_PRINTLN("No path found!");    
      return false;
    }    
    SER_PRINTLN("Found G00!");    
    pathPosition = svgFile.position();

    min_y = min_x = 100000000.0;
    max_y = max_x = -100000000.0;

    while(true) {
      if(getNextPathSegment(x, y, pen,segments==0)) {
        segments++; 
		Serial.print(".");
        min_x = min(min_x, *x);
        max_x = max(max_x, *x);
        min_y = min(min_y, *y);
        max_y = max(max_y, *y);
		
      }
      else {
		  //Serial.println("derpo");
        break;
      }
    }
    //scaleFactor = (disparity*0.4) / (max_x-min_x); //fill 40% of disparity as default
    
    SER_PRINT("Segments=");    
    SER_PRINTLN(segments);    

    SER_PRINT("Scale=");    
    SER_PRINTLN(scaleFactor);    

#ifdef USE_MOCKED_STEPPERS
      fprintf(stderr,"segments=%3ld scale=%2.2f x=[%2.2f , %2.2f] y=[%2.2f , %2.2f] disparity=%ld\n", segments, scaleFactor, min_x,max_x, min_y, max_y, disparity);
#endif        

    svgFile.seek(pathPosition);    
  }

  if(point != lastReadPoint) {
	
    lastReadPoint = point;

    if(getNextPathSegment(x, y, pen,point==0)) {
		//stepRoutine();
     // *x = (*x-min_x)*scaleFactor;   
     // *y = (*y-min_y)*scaleFactor;
		if (*x != 0.0f && *y != 0.0f) {
			lastX = *x;
			lastY = *y;
		}
      
      lastPen = *pen;   
    }
    else {
		callcount++;
		//stepRoutine();
      lastX = 1;
      lastY = -1;
      lastPen = 0;
      // rewind the file:
      svgFile.seek(0);    
	  //Serial.println("END_REWIND");
      return false;
    }    
  }
  else {
	  if (lastX != 0.0f && lastY != 0.0f) {
		  *x = lastX;
		  *y = lastY;
	  }
    *pen = lastPen;    
  }

  //SER_PRINT(*pen ? "L " : "M ");    
  //Serial.print(*x); Serial.print(" "); Serial.println(*y);
  return true;
}

