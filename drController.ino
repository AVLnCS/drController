#include "DEV_Config.h"
#include "EPD.h"
#include "GUI_Paint.h"
#include "ImageData.h"
#include <stdlib.h>
#include <Arduino.h>
#include <esp_task_wdt.h>

#define LEDC_TIMER_8_BIT  8
#define LEDC_BASE_FREQ    22000
#define motorForwardPin    23
#define motorReversePin    22
#define buttonUpPin 16
#define buttonDownPin 18
#define buttonStopPin 5
#define buttonPlusPin 19
#define buttonMinusPin 17
#define buttonUpdatePin 4
#define hallSensorPin 21


UBYTE *BlackImage;
UBYTE *GUI;

bool updateGUI = false;

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
int userDepth = 80;

// Function Prototypes
void updateDisplay(uint8_t * image);



void setup() {

  // Create a configuration structure
  esp_task_wdt_config_t wdt_config = {
      .timeout_ms = 10000,  // Timeout in milliseconds
      .idle_core_mask = 0,  // Apply to both cores (0 for ESP32)
      .trigger_panic = true // Trigger panic handler on timeout
  };

  // Initialize the watchdog timer
  esp_task_wdt_init(&wdt_config);

  // Subscribe the current task to the watchdog timer
  esp_task_wdt_add(NULL);


  // Create a task for display setup
  xTaskCreatePinnedToCore(
      DisplaySetupTask, /* Task function */
      "DisplaySetup",   /* Name of the task */
      10000,            /* Stack size of task */
      NULL,             /* Parameter of the task */
      1,                /* Priority of the task */
      NULL,
      0);            /* Task handle to keep track of created task */

  // Setup timer and attach timer to motor control pins
  ledcAttach(motorForwardPin, LEDC_BASE_FREQ, LEDC_TIMER_8_BIT);
  ledcAttach(motorReversePin, LEDC_BASE_FREQ, LEDC_TIMER_8_BIT);
  pinMode(buttonUpPin, INPUT_PULLUP);
  pinMode(buttonStopPin, INPUT_PULLUP);
  pinMode(buttonDownPin, INPUT_PULLUP);
  pinMode(buttonPlusPin, INPUT_PULLUP);
  pinMode(buttonMinusPin, INPUT_PULLUP);
  pinMode(buttonUpdatePin, INPUT_PULLUP);
  pinMode(hallSensorPin, INPUT);
}

void loop() {
  bool buttonUp = digitalRead(buttonUpPin) == LOW;
  bool buttonDown = digitalRead(buttonDownPin) == LOW;
  bool buttonStop = digitalRead(buttonStopPin) == LOW;
  bool buttonPlus = digitalRead(buttonPlusPin) == LOW;
  bool buttonMinus = digitalRead(buttonMinusPin) == LOW;
  bool buttonUpdate = digitalRead(buttonUpdatePin) == LOW;

  if (buttonPlus && userDepth < 250) {
          userDepth += 5;
          if (userDepth > 250){
            userDepth = 250;
          }
          createPartialUpdateTask();
          //testUpdateUserDepth() ;
  }

  if (buttonMinus && userDepth > 0) {
      userDepth -= 5;
      if (userDepth < 0){
        userDepth = 0;
      }
      createPartialUpdateTask();
      //testUpdateUserDepth() ;


  }

  if (buttonUpdate) {
      createPartialUpdateTask();
  }


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

    // Reset the watchdog timer
  esp_task_wdt_reset();
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


void updateUserDepth(){
    #if 0 // partial update, just 1 Gray mode
    EPD_3IN7_1Gray_Init();       //init 1 Gray mode
    EPD_3IN7_1Gray_Clear();
    Paint_SelectImage(GUI);
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
        Paint_ClearWindows(0, 0, 479, 80, WHITE);
        Paint_DrawTime(0, 20, &sPaint_time, &Font20, WHITE, BLACK);

        num = num - 1;
        if(num == 0) {
            break;
        }

        printf("Part refresh...\r\n");
        EPD_3IN7_1Gray_Display(GUI);
        // EPD_3IN7_1Gray_Display_Part(BlackImage, 0, 0, 279, 180);
        DEV_Delay_ms(500);
    }

#endif
    
    #if 1//My code
    EPD_3IN7_1Gray_Init(); // Initialize 1 Gray mode for partial update

    // Ensure the correct buffer is selected for drawing
    Paint_SelectImage(GUI);
    Paint_SetScale(2);

    // Clear the area where userDepth is displayed to avoid overlap
    // Adjust coordinates (x1, y1, x2, y2) as per the actual GUI layout
    Paint_ClearWindows(120, 240, 145, 265, WHITE);

    // Draw the updated userDepth value at the correct position
    Paint_DrawNum(120, 240, userDepth, &Font24, BLACK, WHITE);

    // Perform a partial update to refresh only the part of the screen with userDepth
    // The parameters should be the same as used in Paint_ClearWindows
    EPD_3IN7_1Gray_Display_Part(GUI, 120, 240, 145, 265);
    #endif
}


void DisplaySetupTask(void *pvParameters) {
    // Your display setup code here
    printf("EPD_3IN7_test Demo\r\n");
    DEV_Module_Init();
    
    printf("e-Paper Init and Clear...\r\n");
    EPD_3IN7_4Gray_Init();
    EPD_3IN7_4Gray_Clear();
    DEV_Delay_ms(500);

    //Create a new image cache
    //UBYTE *BlackImage;
    // allocating memory for image
    UWORD Imagesize = ((EPD_3IN7_WIDTH % 4 == 0)? (EPD_3IN7_WIDTH / 4 ): (EPD_3IN7_WIDTH / 4 + 1)) * EPD_3IN7_HEIGHT;
    /*if((BlackImage = (UBYTE *)malloc(Imagesize)) == NULL) {
        printf("Failed to apply for black memory...\r\n");
        //while(1);
    }*/
    if((GUI = (UBYTE *)malloc(Imagesize)) == NULL) {
        printf("Failed to apply for gui memory...\r\n");
        //while(1);
    }

    printf("Paint_NewImage\r\n");
    Paint_NewImage(GUI, EPD_3IN7_WIDTH, EPD_3IN7_HEIGHT, 270, WHITE);
    Paint_SetScale(4);
    //Paint_Clear(WHITE);

  #if 1   //LOGO  
      printf("show image for array\r\n");

      EPD_3IN7_4Gray_Display(BOAT);
      DEV_Delay_ms(1000);
  #endif
    /*
    printf("Paint_NewImage\r\n");
    Paint_NewImage(BlackImage, EPD_3IN7_WIDTH, EPD_3IN7_HEIGHT, 270, WHITE);
    Paint_SetScale(4);
    Paint_Clear(WHITE);
    */
    //fullUpdateGUI();
    createFullUpdateTask();   

    // Delete the task after its execution
    vTaskDelete(NULL);
}

void createFullUpdateTask() {
    xTaskCreatePinnedToCore(
        fullUpdateGUI,       // Task function
        "FullUpdateGUI",     // Name of the task
        30000,               // Stack size of task (adjust as needed)
        NULL,                // Parameter of the task (not used)
        1,                   // Priority of the task
        NULL,
        0);               // Task handle (not used)
}

void fullUpdateGUI(void *pvParameters) {
    // Initialize the e-Paper display for full update
    EPD_3IN7_4Gray_Init();
    //EPD_3IN7_4Gray_Clear(); // Ensure this clears the display

    // Reset the watchdog timer
    esp_task_wdt_reset();

    // Create a new image for the GUI
    Paint_NewImage(GUI, EPD_3IN7_WIDTH, EPD_3IN7_HEIGHT, 180, WHITE);
    Paint_SelectImage(GUI);
    Paint_SetScale(4);
    Paint_Clear(WHITE); // Make sure this clears the previous content
    // Reset the watchdog timer (to prevent reset)
    esp_task_wdt_reset();
    // Define GUI layout parameters
    int row1_y = 395;
    int row2_y = 445;
    int col1_x = 10;
    int col2_x = 107;
    int col3_x = 229;

    // Draw GUI components
    Paint_DrawRectangle(0, 380, 280, 480, BLACK, DOT_PIXEL_1X1, DRAW_FILL_FULL);
    Paint_DrawString_EN(10, 20, "CURRENT DEPTH:", &Font24, WHITE, GRAY4);
    Paint_DrawString_EN(10, 175, "USER DEPTH:", &Font24, WHITE, GRAY4);
    //Paint_DrawNum(120, 240, userDepth, &Font24, GRAY4, WHITE);
    //Paint_DrawNum(120, 85, currentDepth, &Font24, GRAY4, WHITE);

    // Button Labels
    Paint_DrawString_EN(col1_x, row1_y, "+", &Font24, BLACK, GRAY1);
    Paint_DrawString_EN(col1_x, row2_y, "-", &Font24, BLACK, GRAY1);
    Paint_DrawString_EN(col2_x, row1_y, "AUTO UP", &Font16, BLACK, GRAY1);
    Paint_DrawString_EN(col2_x, row2_y, "Down", &Font16, BLACK, GRAY1);
    Paint_DrawString_EN(col3_x, row1_y, "STOP", &Font16, BLACK, GRAY1);
    Paint_DrawString_EN(col3_x, row2_y, "Zero", &Font16, BLACK, GRAY1);

      // Reset the watchdog timer
    esp_task_wdt_reset();

    // Refresh the entire display with the updated GUI
    EPD_3IN7_4Gray_Display(GUI);

    vTaskDelete(NULL); // Delete the task after completion



}

void partialUpdateGUI(void *pvParameters) {
    // Define the coordinates for currentDepth and userDepth display areas
    // These coordinates should match with those used in the full GUI update function
    const UWORD currentDepthX = 120, currentDepthY = 85;
    const UWORD currentDepthWidth = 145 - 120, currentDepthHeight = 265 - 240; // Width and height for currentDepth
    const UWORD userDepthX = 120, userDepthY = 240;
    const UWORD userDepthWidth = 145 - 120, userDepthHeight = 265 - 240; // Width and height for userDepth

    // Clear the areas where currentDepth and userDepth are displayed
    Paint_ClearWindows(currentDepthX, currentDepthY, currentDepthX + currentDepthWidth, currentDepthY + currentDepthHeight, WHITE);
    Paint_ClearWindows(userDepthX, userDepthY, userDepthX + userDepthWidth, userDepthY + userDepthHeight, WHITE);

    // Draw the updated currentDepth and userDepth values
    Paint_DrawNum(currentDepthX, currentDepthY, currentDepth, &Font24, BLACK, WHITE);
    Paint_DrawNum(userDepthX, userDepthY, userDepth, &Font24, BLACK, WHITE);

    // Update the specific parts of the display
    EPD_3IN7_1Gray_Display_Part(GUI, currentDepthX, currentDepthY, currentDepthX + currentDepthWidth, currentDepthY + currentDepthHeight);
    EPD_3IN7_1Gray_Display_Part(GUI, userDepthX, userDepthY, userDepthX + userDepthWidth, userDepthY + userDepthHeight);

    vTaskDelete(NULL); // Delete the task after completion
}


void createPartialUpdateTask() {
    xTaskCreatePinnedToCore(
        partialUpdateGUI,   // Task function
        "PartialUpdateGUI", // Name of the task
        30000,              // Stack size of task (adjust as needed)
        NULL,               // Parameter of the task (not used)
        1,                  // Priority of the task
        NULL,
        0);              // Task handle (not used)
}

