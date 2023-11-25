#include "DEV_Config.h"
#include "EPD.h"
#include "GUI_Paint.h"
#include "ImageData.h"
#include <stdlib.h>
#include <Arduino.h>


#define LEDC_TIMER_8_BIT  8
#define LEDC_BASE_FREQ    22000
#define motorForwardPin    23
#define motorReversePin    22
#define buttonUpPin 16
#define buttonDownPin 18
#define buttonStopPin 5
#define hallSensorPin 21

/*
#include <GxEPD2_BW.h>
#include <Fonts/FreeMonoBold9pt7b.h>


#define USE_HSPI_FOR_EDP
#define ENABLE_GxEPD2_GFX 0
#define GxEPD2_DISPLAY_CLASS GxEPD2_BW
#define GxEPD2_DRIVER_CLASS GxEPD2_370_TC1 // Waveshare 3.7"



//--------------------------------------------------------------
// somehow there should be an easier way to do this
#define GxEPD2_BW_IS_GxEPD2_BW true
#define GxEPD2_3C_IS_GxEPD2_3C true
#define GxEPD2_7C_IS_GxEPD2_7C true
#define GxEPD2_1248_IS_GxEPD2_1248 true
#define IS_GxEPD(c, x) (c##x)
#define IS_GxEPD2_BW(x) IS_GxEPD(GxEPD2_BW_IS_, x)
#define IS_GxEPD2_3C(x) IS_GxEPD(GxEPD2_3C_IS_, x)
#define IS_GxEPD2_7C(x) IS_GxEPD(GxEPD2_7C_IS_, x)
#define IS_GxEPD2_1248(x) IS_GxEPD(GxEPD2_1248_IS_, x)

#if defined(ESP32)
#define MAX_DISPLAY_BUFFER_SIZE 65536ul // e.g.
#if IS_GxEPD2_BW(GxEPD2_DISPLAY_CLASS)
#define MAX_HEIGHT(EPD) (EPD::HEIGHT <= MAX_DISPLAY_BUFFER_SIZE / (EPD::WIDTH / 8) ? EPD::HEIGHT : MAX_DISPLAY_BUFFER_SIZE / (EPD::WIDTH / 8))
#elif IS_GxEPD2_3C(GxEPD2_DISPLAY_CLASS)
#define MAX_HEIGHT(EPD) (EPD::HEIGHT <= (MAX_DISPLAY_BUFFER_SIZE / 2) / (EPD::WIDTH / 8) ? EPD::HEIGHT : (MAX_DISPLAY_BUFFER_SIZE / 2) / (EPD::WIDTH / 8))
#elif IS_GxEPD2_7C(GxEPD2_DISPLAY_CLASS)
#define MAX_HEIGHT(EPD) (EPD::HEIGHT <= (MAX_DISPLAY_BUFFER_SIZE) / (EPD::WIDTH / 2) ? EPD::HEIGHT : (MAX_DISPLAY_BUFFER_SIZE) / (EPD::WIDTH / 2))
#endif
GxEPD2_DISPLAY_CLASS<GxEPD2_DRIVER_CLASS, MAX_HEIGHT(GxEPD2_DRIVER_CLASS)> display(GxEPD2_DRIVER_CLASS( 15,  27, 26,  25));
#endif

// comment out unused bitmaps to reduce code space used
#include "bitmaps/Bitmaps200x200.h" // 1.54" b/w
#include "bitmaps/Bitmaps104x212.h" // 2.13" b/w flexible GDEW0213I5F
#include "bitmaps/Bitmaps128x250.h" // 2.13" b/w
#include "bitmaps/Bitmaps128x296.h" // 2.9"  b/w
#include "bitmaps/Bitmaps176x264.h" // 2.7"  b/w
#include "bitmaps/Bitmaps400x300.h" // 4.2"  b/w
#include "bitmaps/Bitmaps640x384.h" // 7.5"  b/w
// 3-color
#include "bitmaps/Bitmaps3c200x200.h" // 1.54" b/w/r
#include "bitmaps/Bitmaps3c104x212.h" // 2.13" b/w/r
#include "bitmaps/Bitmaps3c128x296.h" // 2.9"  b/w/r
#include "bitmaps/Bitmaps3c176x264.h" // 2.7"  b/w/r
#include "bitmaps/Bitmaps3c400x300.h" // 4.2"  b/w/r

#if defined(ESP32) && defined(USE_HSPI_FOR_EPD)
SPIClass hspi(HSPI);
#endif


const char HelloWorld[] = "Hello World!";

//--------------------------------------------------------------
*/


enum MotorState {
  STOPPED,
  RUNNING_FORWARD,
  RUNNING_REVERSE,
  SLOWING_FORWARD,
  SLOWING_REVERSE,
  ACCEL_FORWARD,
  ACCEL_REVERSE

};

MotorState currentState = STOPPED;
int pulseCount = 0;
int currentDepth = 0;

/*
void displayTask(void * parameter) {
    // Initialize the e-paper display
    display.init();

   drawBitMaps(BOAT);

    // Wait for 3 seconds
    vTaskDelay(3000 / portTICK_PERIOD_MS);



    // Delete the task after completion
    vTaskDelete(NULL);
}
*/


void setup() {


  // Display manufacturers example code
    printf("EPD_3IN7_test Demo\r\n");
    DEV_Module_Init();

    printf("e-Paper Init and Clear...\r\n");
    EPD_3IN7_4Gray_Init();
    EPD_3IN7_4Gray_Clear();
    DEV_Delay_ms(500);

    //Create a new image cache
    UBYTE *BlackImage;

    // allocating memory for image
    UWORD Imagesize = ((EPD_3IN7_WIDTH % 4 == 0)? (EPD_3IN7_WIDTH / 4 ): (EPD_3IN7_WIDTH / 4 + 1)) * EPD_3IN7_HEIGHT;
    if((BlackImage = (UBYTE *)malloc(Imagesize)) == NULL) {
        printf("Failed to apply for black memory...\r\n");
        while(1);
    }

    printf("Paint_NewImage\r\n");
    Paint_NewImage(BlackImage, EPD_3IN7_WIDTH, EPD_3IN7_HEIGHT, 270, WHITE);
    Paint_SetScale(4);
    Paint_Clear(WHITE);

#if 1   //show image for array    
    printf("show image for array\r\n");

    EPD_3IN7_4Gray_Display(BOAT);
    DEV_Delay_ms(4000);
#endif
  
    printf("Paint_NewImage\r\n");
    Paint_NewImage(BlackImage, EPD_3IN7_WIDTH, EPD_3IN7_HEIGHT, 270, WHITE);
    Paint_SetScale(4);
    Paint_Clear(WHITE);

#if 1   //show image for array    
    printf("show image for array\r\n");
    Paint_DrawBitMap(BOAT);
    DEV_Delay_ms(4000);
#endif
  


#if 1 // Drawing on the image, partial display
    //1.Select Image
    printf("SelectImage:BlackImage\r\n");
    Paint_SelectImage(BlackImage);
    Paint_SetScale(4);
    Paint_Clear(WHITE);

    // 2.Drawing on the image
    printf("Drawing:BlackImage\r\n");
    Paint_DrawPoint(10, 80, BLACK, DOT_PIXEL_1X1, DOT_STYLE_DFT);
    Paint_DrawPoint(10, 90, BLACK, DOT_PIXEL_2X2, DOT_STYLE_DFT);
    Paint_DrawPoint(10, 100, BLACK, DOT_PIXEL_3X3, DOT_STYLE_DFT);
    Paint_DrawLine(20, 70, 70, 120, BLACK, DOT_PIXEL_1X1, LINE_STYLE_SOLID);
    Paint_DrawLine(70, 70, 20, 120, BLACK, DOT_PIXEL_1X1, LINE_STYLE_SOLID);
    Paint_DrawRectangle(20, 70, 70, 120, BLACK, DOT_PIXEL_1X1, DRAW_FILL_EMPTY);
    Paint_DrawRectangle(80, 70, 130, 120, BLACK, DOT_PIXEL_1X1, DRAW_FILL_FULL);
    Paint_DrawCircle(45, 95, 20, BLACK, DOT_PIXEL_1X1, DRAW_FILL_EMPTY);
    Paint_DrawCircle(105, 95, 20, WHITE, DOT_PIXEL_1X1, DRAW_FILL_FULL);
    Paint_DrawLine(85, 95, 125, 95, BLACK, DOT_PIXEL_1X1, LINE_STYLE_DOTTED);
    Paint_DrawLine(105, 75, 105, 115, BLACK, DOT_PIXEL_1X1, LINE_STYLE_DOTTED);
    Paint_DrawString_EN(10, 0, "waveshare", &Font16, BLACK, WHITE);
    Paint_DrawString_EN(10, 20, "hello world", &Font12, WHITE, BLACK);
    Paint_DrawNum(10, 33, 123456789, &Font12, BLACK, WHITE);
    Paint_DrawNum(10, 50, 987654321, &Font16, WHITE, BLACK);
    Paint_DrawString_EN(10, 150, "GRAY1 with black background", &Font24, BLACK, GRAY1);
    Paint_DrawString_EN(10, 175, "GRAY2 with white background", &Font24, WHITE, GRAY2);
    Paint_DrawString_EN(10, 200, "GRAY3 with white background", &Font24, WHITE, GRAY3);
    Paint_DrawString_EN(10, 225, "GRAY4 with white background", &Font24, WHITE, GRAY4);
    printf("EPD_Display\r\n");
    EPD_3IN7_4Gray_Display(BlackImage);
    DEV_Delay_ms(4000);
#endif

#if 1 // partial update, just 1 Gray mode
    EPD_3IN7_1Gray_Init();       //init 1 Gray mode
    EPD_3IN7_1Gray_Clear();
    Paint_SelectImage(BlackImage);
    Paint_SetScale(2);
    Paint_Clear(WHITE);
    printf("show time, partial update, just 1 Gary mode\r\n");
    PAINT_TIME sPaint_time;
    sPaint_time.Hour = 12;
    sPaint_time.Min = 34;
    sPaint_time.Sec = 56;
    UBYTE num = 10;
    for (;;) {
        sPaint_time.Sec = sPaint_time.Sec + 1;
        if (sPaint_time.Sec == 60) {
            sPaint_time.Min = sPaint_time.Min + 1;
            sPaint_time.Sec = 0;
            if (sPaint_time.Min == 60) {
                sPaint_time.Hour =  sPaint_time.Hour + 1;
                sPaint_time.Min = 0;
                if (sPaint_time.Hour == 24) {
                    sPaint_time.Hour = 0;
                    sPaint_time.Min = 0;
                    sPaint_time.Sec = 0;
                }
            }
        }
        Paint_ClearWindows(300, 0, 479, 80, WHITE);
        Paint_DrawTime(300, 20, &sPaint_time, &Font20, WHITE, BLACK);

        num = num - 1;
        if(num == 0) {
            break;
        }

        printf("Part refresh...\r\n");
        EPD_3IN7_1Gray_Display(BlackImage);
        // EPD_3IN7_1Gray_Display_Part(BlackImage, 0, 0, 279, 180);
        DEV_Delay_ms(500);
    }

#endif


    EPD_3IN7_4Gray_Init();
    printf("Clear...\r\n");
    EPD_3IN7_4Gray_Clear();
    
    // Sleep & close 5V
    printf("Goto Sleep...\r\n");
    EPD_3IN7_Sleep();

    free(BlackImage);
    BlackImage = NULL;

    printf("close 5V, Module enters 0 power consumption ...\r\n");  
  // End of manufacturers Setup code




  // Setup timer and attach timer to motor control pins
  ledcAttach(motorForwardPin, LEDC_BASE_FREQ, LEDC_TIMER_8_BIT);
  ledcAttach(motorReversePin, LEDC_BASE_FREQ, LEDC_TIMER_8_BIT);
  pinMode(buttonUpPin, INPUT_PULLUP);
  pinMode(buttonStopPin, INPUT_PULLUP);
  pinMode(buttonDownPin, INPUT_PULLUP);
  pinMode(hallSensorPin, INPUT);
}

void loop() {
  bool buttonUp = digitalRead(buttonUpPin) == LOW;
  bool buttonDown = digitalRead(buttonDownPin) == LOW;
  bool buttonStop = digitalRead(buttonStopPin) == LOW;

  switch (currentState) {
    case STOPPED:

      if (buttonUp) {
        softStart(motorForwardPin);
      } 
      else if (buttonDown) {
        softStart(motorReversePin);
      }
      break;
    
    case RUNNING_FORWARD:
    case RUNNING_REVERSE:
    case SLOWING_REVERSE: // Include SLOWING_REVERSE in the conditions
      if (buttonStop) {
        // If the stop button is pressed, stop the motor from any state
        softStop(currentState == RUNNING_FORWARD ? motorForwardPin : motorReversePin);
      } else if ((currentState == RUNNING_FORWARD && buttonDown) || (currentState == RUNNING_REVERSE && buttonUp)) {
        // If the opposite direction button is pressed, stop the motor
        softStop(currentState == RUNNING_FORWARD ? motorForwardPin : motorReversePin);
      }
      break;
  }


  // Read the Hall effect sensor
  static bool lastHallState = LOW;
  bool currentHallState = digitalRead(hallSensorPin);
  if (currentHallState != lastHallState) {
    // Count every rising edge of the Hall effect sensor
    

      if (currentState == RUNNING_FORWARD || currentState == STOPPED ) {
        pulseCount++;
      } 
      else if (currentState == RUNNING_REVERSE || currentState == SLOWING_REVERSE) {
        pulseCount--;
      }
    

  }
  lastHallState = currentHallState;

    // Print the current depth and pulse count for debugging
  currentDepth = pulseCount / 6.5;
  Serial.print("Depth: ");
  Serial.print(currentDepth);
  Serial.print(", Pulses: ");
  Serial.println(pulseCount);


    // Handle automatic slowing and stopping in reverse
  if (currentState == RUNNING_REVERSE) {
    if (pulseCount == 22 && currentState != SLOWING_REVERSE) {
      // Slow the motor to 30% speed
      setMotorSpeed(motorReversePin, 50); // 30% of full speed
      currentState = SLOWING_REVERSE;
    } else if (pulseCount == 2) {
      // Stop the motor softly
      softStop(motorReversePin);
    }
  }

    // Handle automatic slowing and stopping in reverse
  if (currentState == SLOWING_REVERSE) {
    if (currentDepth == 0) {

      softStop(motorReversePin);
    }
  }


  delay(10); // for debouncing
}

void setMotorSpeed(int pin, int targetPercent) {
  int targetDutyCycle = map(targetPercent, 0, 100, 0, 255); // Convert target percentage to duty cycle
  int currentDutyCycle = ledcRead(pin); // Get the current duty cycle
  int stepTime = 5; // Time in milliseconds for each step of the fade
  int totalSteps = 500 / stepTime; // Total number of steps for the fade

  // Calculate the difference per step
  int stepDifference = (targetDutyCycle - currentDutyCycle) / totalSteps;

  // Gradually change the duty cycle from current to target
  for (int i = 0; i < totalSteps; i++) {
    currentDutyCycle += stepDifference;
    ledcWrite(pin, currentDutyCycle);
    delay(stepTime);
  }

  // Ensure the final value is exactly the target duty cycle
  ledcWrite(pin, targetDutyCycle);
}



void softStart(int pin) {
  if(pin == motorForwardPin){        
    currentState = RUNNING_FORWARD;
  }
  else
    currentState = RUNNING_REVERSE;

  for (int dutyCycle = 0; dutyCycle <= 255; dutyCycle++) {
    ledcWrite(pin, dutyCycle);
    delay(7);
  }
}


void softStop(int pin) {
  // Get the current duty cycle of the pin
  int currentDutyCycle = ledcRead(pin);

  // Ramp down from the current duty cycle to 0
  for (int dutyCycle = currentDutyCycle; dutyCycle >= 0; dutyCycle--) {
    ledcWrite(pin, dutyCycle);
    delay(7); // Adjust delay to control the ramp-down speed
  }

  // Update the state to STOPPED after the motor is completely stopped
  currentState = STOPPED;
}


/*
void helloWorld()
{
  //Serial.println("helloWorld");
  display.setRotation(1);
  display.setFont(&FreeMonoBold9pt7b);
  display.setTextColor(GxEPD_BLACK);
  int16_t tbx, tby; uint16_t tbw, tbh;
  display.getTextBounds(HelloWorld, 0, 0, &tbx, &tby, &tbw, &tbh);
  // center bounding box by transposition of origin:
  uint16_t x = ((display.width() - tbw) / 2) - tbx;
  uint16_t y = ((display.height() - tbh) / 2) - tby;
  display.setFullWindow();
  display.firstPage();
  do
  {
    display.fillScreen(GxEPD_WHITE);
    display.setCursor(x, y);
    display.print(HelloWorld);
  }
  while (display.nextPage());
  //Serial.println("helloWorld done");
}*/

/*
void drawBitMaps(const unsigned char *bitmap)
{
  display.setRotation(90);
  display.setFullWindow();
  display.firstPage();
  do{
    display.fillScreen(GxEPD_WHITE);
    display.drawInvertedBitmap(0, 0, bitmap, display.epd2.WIDTH, display.epd2.HEIGHT, GxEPD_BLACK);
  } while(display.nextPage());
}

*/