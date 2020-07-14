//#include <IRremote.h>

#define ADAFRUIT_LIBS

#include "MachineDefs.h"
#include <SPI.h>       // this is needed for display
#ifndef ADAFRUIT_LIBS // comment this if usoing bootleg libs
#include <GFX.h>    // Core graphics library
#include <ILI9341.h>
#include <FT6206.h>
#else
#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ILI9341.h>
#include <Adafruit_FT6206.h>

#endif

//FT6206 ctp = FT6206();  //uncomment this if using bootleg libs
Adafruit_FT6206 ctp = Adafruit_FT6206();




#define TFT_CS 9
#define TFT_DC 7
//ILI9341 tft = ILI9341(TFT_CS, TFT_DC); //uncomment this if using bootleg libs
Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC);
int buttonCode;


ArduinoBlue phone(Serial1);




#define DISP_INCHES 23.22

#define DEADZONE 20







int throttle, steering, sliderVal, button, sliderId;

int prevThrottle = 49;
int prevSteering = 49;

//IRrecv irrecv(RECV_PIN);
#define NONAME_WHITE_REMOTE ;
#ifdef PIONEER_DVD_REMOTE
/******************** PIONEER_DVD_REMOTE **********************/
#define CODE_BIG_PRINT 0xF50A4FB0    //up
#define CODE_SMALL_PRINT 0xF50ACF30  //downd
#define CODE_LEFT_REEL_IN 0xf50a1de2   //step
#define CODE_LEFT_REEL_OUT 0xf50aed12  //slow
#define CODE_RIGHT_REEL_IN 0xc53ad926   //rev
#define CODE_RIGHT_REEL_OUT 0xc53a59a6  //fwd
#define CODE_LEFT_CALIBRATION 0xC53AB946  //prev
#define CODE_RIGHT_CALIBRATION 0xC53A39C6 //next
#define CODE_ENABLE_CONT_DRIVE 0xf50a2df0  //return
#define CODE_DISABLE_CONT_DRIVE 0xf50af708 //enter
#define CODE_STOP 0xC53A19E6 //pause
#define CODE_RESUME 0xc53a7986 //play
#define CODE_1 0xf50a857a
#define CODE_2 0xf50a45ba
#define CODE_3 0xf50ac53a
#define CODE_4 0xF50A25DA
#define CODE_5 0xF50AA55A
#define CODE_6 0xF50A659A
#define CODE_7 0xF50AE51A
#define CODE_8 0xF50A15EA
#define CODE_9 0xF50A956A
#define CODE_0 0xF50A05FA

#elif defined NONAME_WHITE_REMOTE
/******************** NONAME_WHITE_REMOTE **********************/
#define CODE_BIG_PRINT 0xff02fd //+
#define CODE_SMALL_PRINT 0xff9867 //-
#define CODE_LEFT_REEL_IN 0xff22dd //test
#define CODE_LEFT_REEL_OUT 0xffe01f //left
#define CODE_RIGHT_REEL_IN 0xffc23d //back
#define CODE_RIGHT_REEL_OUT 0xff906f //right
#define CODE_LEFT_CALIBRATION 0xffa25d //power
#define CODE_RIGHT_CALIBRATION 0xffe21d //menu
#define CODE_ENABLE_CONT_DRIVE 0xffb04f //c
#define CODE_DISABLE_CONT_DRIVE 0xBADC0DE //not button for this shit 
#define CODE_STOP 0xff6897 //0
#define CODE_RESUME 0xffa857 //play
#define CODE_1 0xff30cf
#define CODE_2 0xff18e7
#define CODE_3 0xff7a85
#define CODE_4 0xff10ef
#define CODE_5 0xff38c7
#define CODE_6 0xff5aa5
#define CODE_7 0xff42bd
#define CODE_8 0xff4ab5
#define CODE_9 0xff52ad

#else
#error What remote?
#endif 
#define BUFFPIXEL 20
void bmpDraw(char *filename, uint8_t x, uint16_t y) {

	File     bmpFile;
	int      bmpWidth, bmpHeight;   // W+H in pixels
	uint8_t  bmpDepth;              // Bit depth (currently must be 24)
	uint32_t bmpImageoffset;        // Start of image data in file
	uint32_t rowSize;               // Not always = bmpWidth; may have padding
	uint8_t  sdbuffer[3 * BUFFPIXEL]; // pixel buffer (R+G+B per pixel)
	uint8_t  buffidx = sizeof(sdbuffer); // Current position in sdbuffer
	boolean  goodBmp = false;       // Set to true on valid header parse
	boolean  flip = true;        // BMP is stored bottom-to-top
	int      w, h, row, col;
	uint8_t  r, g, b;
	uint32_t pos = 0, startTime = millis();

	if ((x >= tft.width()) || (y >= tft.height())) return;

	Serial.println();
	Serial.print(F("Loading image '"));
	Serial.print(filename);
	Serial.println('\'');

	// Open requested file on SD card
	if ((bmpFile = SD.open(filename)) == NULL) {
		Serial.print(F("File not found"));
		return;
	}

	// Parse BMP header
	if (read16(bmpFile) == 0x4D42) { // BMP signature
		Serial.print(F("File size: ")); Serial.println(read32(bmpFile));
		(void)read32(bmpFile); // Read & ignore creator bytes
		bmpImageoffset = read32(bmpFile); // Start of image data
		Serial.print(F("Image Offset: ")); Serial.println(bmpImageoffset, DEC);
		// Read DIB header
		Serial.print(F("Header size: ")); Serial.println(read32(bmpFile));
		bmpWidth = read32(bmpFile);
		bmpHeight = read32(bmpFile);
		if (read16(bmpFile) == 1) { // # planes -- must be '1'
			bmpDepth = read16(bmpFile); // bits per pixel
			Serial.print(F("Bit Depth: ")); Serial.println(bmpDepth);
			if ((bmpDepth == 24) && (read32(bmpFile) == 0)) { // 0 = uncompressed

				goodBmp = true; // Supported BMP format -- proceed!
				Serial.print(F("Image size: "));
				Serial.print(bmpWidth);
				Serial.print('x');
				Serial.println(bmpHeight);

				// BMP rows are padded (if needed) to 4-byte boundary
				rowSize = (bmpWidth * 3 + 3) & ~3;

				// If bmpHeight is negative, image is in top-down order.
				// This is not canon but has been observed in the wild.
				if (bmpHeight < 0) {
					bmpHeight = -bmpHeight;
					flip = false;
				}

				// Crop area to be loaded
				w = bmpWidth;
				h = bmpHeight;
				if ((x + w - 1) >= tft.width())  w = tft.width() - x;
				if ((y + h - 1) >= tft.height()) h = tft.height() - y;

				// Set TFT address window to clipped image bounds
				tft.setAddrWindow(x, y, x + w - 1, y + h - 1);

				for (row = 0; row < h; row++) { // For each scanline...

				  // Seek to start of scan line.  It might seem labor-
				  // intensive to be doing this on every line, but this
				  // method covers a lot of gritty details like cropping
				  // and scanline padding.  Also, the seek only takes
				  // place if the file position actually needs to change
				  // (avoids a lot of cluster math in SD library).
					if (flip) // Bitmap is stored bottom-to-top order (normal BMP)
						pos = bmpImageoffset + (bmpHeight - 1 - row) * rowSize;
					else     // Bitmap is stored top-to-bottom
						pos = bmpImageoffset + row * rowSize;
					if (bmpFile.position() != pos) { // Need seek?
						bmpFile.seek(pos);
						buffidx = sizeof(sdbuffer); // Force buffer reload
					}

					for (col = 0; col < w; col++) { // For each pixel...
					  // Time to read more pixel data?
						if (buffidx >= sizeof(sdbuffer)) { // Indeed
							bmpFile.read(sdbuffer, sizeof(sdbuffer));
							buffidx = 0; // Set index to beginning
						}

						// Convert pixel from BMP to TFT format, push to display
						b = sdbuffer[buffidx++];
						g = sdbuffer[buffidx++];
						r = sdbuffer[buffidx++];
						tft.pushColor(tft.color565(r, g, b));
					} // end pixel
				} // end scanline
				Serial.print(F("Loaded in "));
				Serial.print(millis() - startTime);
				Serial.println(" ms");
			} // end goodBmp
		}
	}

	bmpFile.close();
	if (!goodBmp) Serial.println(F("BMP format not recognized."));
}
uint16_t read16(File &f) {
	uint16_t result;
	((uint8_t *)&result)[0] = f.read(); // LSB
	((uint8_t *)&result)[1] = f.read(); // MSB
	return result;
}

uint32_t read32(File &f) {
	uint32_t result;
	((uint8_t *)&result)[0] = f.read(); // LSB
	((uint8_t *)&result)[1] = f.read();
	((uint8_t *)&result)[2] = f.read();
	((uint8_t *)&result)[3] = f.read(); // MSB
	return result;
}

void setupIR() 
{
  //irrecv.enableIRIn(); // Start the receiver  
	Serial1.begin(9600);
	tft.begin();
	ctp.begin(45);
	tft.setRotation(3);
	tft.fillScreen(ILI9341_BLACK);
	bmpDraw("scr.bmp", 0, 0);
	Serial.println();


}

void readIR(bool BTEnable) {
	float lDist;
	bool fail = false;
	if (BTEnable) {
		button = phone.getButton();
		throttle = phone.getThrottle();
		steering = phone.getSteering();
	}
	if (ctp.touched()) {
		TS_Point p = ctp.getPoint();
		p.x = map(p.x, 0, 240, 240, 0);
		p.y = map(p.y, 0, 320, 320, 0);
		//p.x = 240 - p.x;
		//p.y = 320 - p.y;

		//Serial.print(p.x);
		//Serial.println(p.y);

		if (p.y < 100) {//left side
			if (p.x > 175) {
				//top
				buttonCode = CODE_LEFT_REEL_OUT;
			}
			else if (p.x < 75) {
				//bottom
				buttonCode = CODE_LEFT_CALIBRATION;
			}
			else {
				//middle
				buttonCode = CODE_LEFT_REEL_IN;
			}
		}
		else if (p.y > 220) { //right side
			if (p.x > 175) {
				//top
				buttonCode = CODE_RIGHT_REEL_OUT;
			}
			else if (p.x < 75) {
				//bottom
				buttonCode = CODE_RIGHT_CALIBRATION;
			}
			else {
				//middle
				buttonCode = CODE_RIGHT_REEL_IN;
			}
		}
		else {//center column
			if (p.x > 175) {
				//top
				buttonCode = CODE_ENABLE_CONT_DRIVE;
			}
			else if (p.x < 75) {
				//bottom
				buttonCode = CODE_RESUME;
				SER_PRINTLN("startbutton");
			}
			else {
				//middle
				buttonCode = CODE_STOP;
				SER_PRINTLN("stopbutton");
			}
		}
		switch (buttonCode) {
			//#ifndef NO_REMOTE      


		case CODE_LEFT_REEL_IN: //left -
			manualLeft = -1;
			break;
		case CODE_LEFT_REEL_OUT: //left +
			manualLeft = 1;
			break;
		case CODE_RIGHT_REEL_IN: //right -
			manualRight = -1;
			break;
		case CODE_RIGHT_REEL_OUT: //right +
			manualRight = 1;
			break;
		case CODE_LEFT_CALIBRATION:  //prev - calibrate 200 mm from left
			currentLeftSteps = 762 * stepsPerMM;
			//currentLeftRealSteps = 762 * stepsPerMM;
			//testPen();
			break;
		case CODE_RIGHT_CALIBRATION:  //next - calibrate 200 mm from right
			currentRightSteps = 762 * stepsPerMM;
			//currentRightRealSteps = 762 * stepsPerMM;

			lDist = currentLeftSteps / stepsPerMM;
			disparity = (long)sqrt(lDist*lDist - 762L * 762L);
			//disparity = DISP_INCHES *25.4;

			SER_PRINT("Cal: lDist)");
			SER_PRINTLN(lDist);

			SER_PRINT("Cal: Disp=");
			SER_PRINTLN(disparity);

			break;
		case CODE_DISABLE_CONT_DRIVE:
			continousManualDrive = false;
			break;
		case CODE_ENABLE_CONT_DRIVE:
			continousManualDrive = true;
			break;
		case CODE_STOP:
			stopPressed = true;
#if  CODE_DISABLE_CONT_DRIVE == 0xBADC0DE
			//just disable continous drive when pressing stop. Re-enable with CODE_ENABLE_CONT_DRIVE again
			continousManualDrive = false;
#endif
			break;
		case CODE_RESUME:
			//resume print, or start new
			program = 1; //start print
			currentPlot = 1;
			resumePlot = true;
			break;

		default:
			fail = true;
			//#endif //NO_REMOTE         
		}

	}
	else if (button != -1 && BTEnable) {
		switch (button) {
		case 0: buttonCode = CODE_LEFT_REEL_IN;
			break;
		case 1: buttonCode = CODE_LEFT_REEL_OUT;
			break;
		case 2: buttonCode = CODE_RIGHT_REEL_IN;
			break;
		case 3: buttonCode = CODE_RIGHT_REEL_OUT;
			break;
		case 4: buttonCode = CODE_STOP;
			break;
		case 5: buttonCode = CODE_ENABLE_CONT_DRIVE;
			break;
		case 6: buttonCode = CODE_LEFT_CALIBRATION;
			break;
		case 7: buttonCode = CODE_RIGHT_CALIBRATION;
			break;
		case 8: buttonCode = CODE_RESUME;
			break;
		default: fail = true;
		}
		switch (buttonCode) {
			//#ifndef NO_REMOTE      


		case CODE_LEFT_REEL_IN: //left -
			manualLeft = -1;
			break;
		case CODE_LEFT_REEL_OUT: //left +
			manualLeft = 1;
			break;
		case CODE_RIGHT_REEL_IN: //right -
			manualRight = -1;
			break;
		case CODE_RIGHT_REEL_OUT: //right +
			manualRight = 1;
			break;
		case CODE_LEFT_CALIBRATION:  //prev - calibrate 200 mm from left
			currentLeftSteps = 762 * stepsPerMM;
			//currentLeftRealSteps = 762 * stepsPerMM;
			//testPen();
			break;
		case CODE_RIGHT_CALIBRATION:  //next - calibrate 200 mm from right
			currentRightSteps = 762 * stepsPerMM;
			//currentRightRealSteps = 762 * stepsPerMM;

			lDist = currentLeftSteps / stepsPerMM;
			disparity = (long)sqrt(lDist*lDist - 762L * 762L);
			//disparity = DISP_INCHES *25.4;

			SER_PRINT("Cal: lDist)");
			SER_PRINTLN(lDist);

			SER_PRINT("Cal: Disp=");
			SER_PRINTLN(disparity);

			break;
		case CODE_DISABLE_CONT_DRIVE:
			continousManualDrive = false;
			break;
		case CODE_ENABLE_CONT_DRIVE:
			continousManualDrive = true;
			break;
		case CODE_STOP:
			stopPressed = true;
			SER_PRINTLN("stopbutton");
#if  CODE_DISABLE_CONT_DRIVE == 0xBADC0DE
			//just disable continous drive when pressing stop. Re-enable with CODE_ENABLE_CONT_DRIVE again
			continousManualDrive = false;
#endif
			break;
		case CODE_RESUME:
			SER_PRINTLN("startbutton");
			//resume print, or start new
			program = 1; //start print
			currentPlot = 1;
			resumePlot = true;
			break;

		default: fail = true;
			//#endif //NO_REMOTE         
		}
	}
	else if ((throttle != 49 || steering != 49) && BTEnable) { // no buttons pressed on touch or BT, check BT joystick
		if ((throttle > 49 + DEADZONE || throttle < 49 - DEADZONE) || (steering > 49 + DEADZONE || steering < 49 - DEADZONE)) {
			if (throttle > 49) {
				throttle = 49 + (sqrt(abs((throttle - 49)*(throttle - 49) - (steering - 49)*(steering - 49))));
			}
			else {
				throttle = 49 - (sqrt(abs((throttle - 49)*(throttle - 49) - (steering - 49)*(steering - 49))));
			}
			//Serial.print(throttle); Serial.print(" , "); Serial.println(steering);
			manualRight = float(-throttle) - float(steering) + 98;
			manualLeft = float(steering - 49) - float(throttle - 49);
			float highest = max(abs(manualLeft), abs(manualRight));
			manualRight = manualRight / highest;
			manualLeft = manualLeft / highest;

		}
		else fail = true;
	}
	else fail = true;





















	if (fail) {
		if (stopPressed) {

		}
		// SER_PRINT("???: ");
	}

	//else {
	//   makePenNoise(1);
	//}
	//SER_PRINTLN2(results.value, HEX);      

	//irrecv.resume(); // Receive the next value

}

