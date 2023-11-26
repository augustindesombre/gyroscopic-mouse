/* ============================================
  I2Cdev device library code is placed under the MIT license
  Copyright (c) 2021 Homer Creutz
  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:
  The above copyright notice and this permission notice shall be included in
  all copies or substantial portions of the Software.
  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
  THE SOFTWARE.
  ===============================================
*/

//#include "MPU_ReadMacros.h"
//#include "MPU_WriteMacros.h"
#include "Simple_MPU6050.h"
#include "AbsMouse.h"

#define MPU6050_ADDRESS_AD0_LOW     0x68 // address pin low (GND), default for InvenSense evaluation board
#define MPU6050_ADDRESS_AD0_HIGH    0x69 // address pin high (VCC)
#define MPU6050_DEFAULT_ADDRESS     MPU6050_ADDRESS_AD0_LOW

Simple_MPU6050 mpu;
/*             _________________________________________________________*/
//               X Accel  Y Accel  Z Accel   X Gyro   Y Gyro   Z Gyro
#define OFFSETS  -1238,    4040,     684,      92,       36,      -8  // Best Offsets for given card, needs to be recomputed i using a new chip 

//***************************************************************************************
//*****************              Define user-set variables           ********************
//***************************************************************************************

#define XRESOLUTION 1920          // Largeur de ton écran en pixels
#define YRESOLUTION 1080          // Hauteur de ton écran en pixels

#define X_SENSITIVITY 2.5         // Sensibilité de la souris sur le plan horizontal (2.5)
#define Y_SENSITIVITY 2.5         // Sensibilité de la souris sur le plan vertical (2.5)

#define THRESHOLD_X 1             // Distance en pixels, par laquelle la souris se déconnecte si tu arrêtes de bouger (horizontal)
#define THRESHOLD_Y 1             // Distance en pixels, par laquelle la souris se déconnecte si tu arrêtes de bouger (vertical)

#define FLIP_X false              // Permet de renverser le plan horizontal
#define FLIP_Y true               // Permet de renverser le plan vertical
#define TILT_SENSITIVITY 0.3     // Sensibilité du mouvement de tête pour pencher (0.5)

#define CLICK_TIME_SENSITIVITY 500  // Temps en ms, avant de déclencher un clic
#define CLICK_SPACE_SENSITIVITY 10  // Distance en pixels, qui permet de relancer le timer du click
#define TIME_BETWEEN_CLICKS 1000     // Temps en ms entre les clics successifs

#define BUZZ true                // définit si le boitier fait du bruit

#define GLOBAL_MOUSE_ATTACH true  // développement : définit si la souris a une action sur l'écran
#define SERIAL_OUTPUT false       // développement : définit si il y aura une sortie sur l'écran

//***************************************************************************************
//*****************              Define rotation                     ********************
//***************************************************************************************

// Quaternion OFFSET_ROTATION(1,0,0,0); // décommenter cette ligne si tu veux le mettre avec l'usb à l'horizontal
Quaternion OFFSET_ROTATION(sqrt(0.5),0,0,sqrt(0.5)); // décommenter cette ligne si tu veux le port USB vers le bas.

//***************************************************************************************
//*****************              Define main variables               ********************
//***************************************************************************************

float XSensitivity = XRESOLUTION * X_SENSITIVITY;
float YSensitivity = YRESOLUTION * Y_SENSITIVITY;

int X = XRESOLUTION/2;
int Y = YRESOLUTION/2;

float XOffset = X*1.0;
float YOffset = Y*1.0;



float tiltOffset = 0.0;

bool mouseAttached;
bool clickable;

float lastX = -100;
float lastY = -100;

int angleOffset = - PI/2;

//***************************************************************************************
//*****************              Define time variables               ********************
//***************************************************************************************

float timeOfClick = -10;


//***************************************************************************************
//**********               Orientation computation Functions                *************
//***************************************************************************************

#define spamtimer(t) for (static uint32_t SpamTimer; (uint32_t)(millis() - SpamTimer) >= (t); SpamTimer = millis()) // (BLACK BOX) Ya, don't complain that I used "for(;;){}" instead of "if(){}" for my Blink Without Delay Timer macro. It works nicely!!!

/* printfloatx() is a helper Macro used with the Serial class to simplify my code and provide enhanced viewing of Float and interger values:
   usage: printfloatx(Name,Variable,Spaces,Precision,EndTxt);
   Name and EndTxt are just char arrays
   Variable is any numerical value byte, int, long and float
   Spaces is the number of spaces the floating point number could possibly take up including +- and decimal point.
   Percision is the number of digits after the decimal point set to zero for intergers
*/

void orientation_from_Quaternion(int16_t *gyro, int16_t *accel, int32_t *quat, uint32_t *timestamp, uint16_t SpamDelay = 100){
  Quaternion q;
  VectorFloat orientation(1,0,0);
  
  spamtimer(SpamDelay) {// non blocking delay before printing again. This skips the following code when delay time (ms) hasn't been met
    mpu.GetQuaternion(&q, quat);
    orientation.rotate(&OFFSET_ROTATION);
    orientation.rotate(&q);
    attach_detach_mouse(&q);
    compute_mouse_position(orientation);
  }
}

//***************************************************************************************
//******************              Mouse mouse around                 ********************
//***************************************************************************************


void attach_detach_mouse(Quaternion *q){
  VectorFloat vectorialProduct(0,0,1);
  vectorialProduct.rotate(&OFFSET_ROTATION);
  vectorialProduct.rotate(q);
  float tilt = vectorialProduct.z + tiltOffset;
  if (tilt > TILT_SENSITIVITY && !mouseAttached){
    mouseAttached = true ;
    buzzGood();
    if (SERIAL_OUTPUT){
      Serial.println("Mouse attached");
    }
  }  
  else if (tilt < - TILT_SENSITIVITY && mouseAttached){
    mouseAttached = false;
    buzzNeutral();
    if (SERIAL_OUTPUT){
      Serial.println("Mouse detached");
    }
  }
}

void compute_mouse_position(VectorFloat orientation){
    int oldX = X; 
    int oldY = Y;
    float x = asin((orientation.x*cos(angleOffset) + orientation.y*sin(angleOffset))/sqrt(pow(orientation.x,2) + pow(orientation.y,2))) ;//*orientation.y/abs(orientation.y); // to get sign right
    float y = asin(orientation.z);

    if (FLIP_X){
      x = - x;
    }
    if (FLIP_Y){
      y = - y;
    }
    if (x > 1.5){
      angleOffset -= PI/2;
      return;
    }
    if (x < -1.5){
      angleOffset += PI/2;
    }
    
    X = XOffset + x*XSensitivity;
    Y = YOffset + y*YSensitivity;

    if (X < 0){
      XOffset -= X;
      X = 0;
    }
    else if (X >= XRESOLUTION){
      XOffset -= X - XRESOLUTION;
      X = XRESOLUTION - 1 ;
    }
    if (Y < 0){
      YOffset -= Y;
      Y = 0;
    }
    else if (Y >= YRESOLUTION){
      YOffset -= Y - YRESOLUTION;
      Y= YRESOLUTION - 1;
    }
    if (abs(X-oldX) > THRESHOLD_X ||  abs(Y-oldY) > THRESHOLD_Y){
       moveMouse(X,Y);
    }
    shouldClick(X,Y);
   
}

void shouldClick(int X, int Y){
  if (mouseAttached && GLOBAL_MOUSE_ATTACH){
    float dist = sqrt(pow(lastX - X,2)+pow(lastY - Y, 2));
    if (dist > CLICK_SPACE_SENSITIVITY || X== XRESOLUTION || X == 0 || Y == YRESOLUTION || Y==0){
      lastX = X;
      lastY = Y;
      timeOfClick = millis();
    }
    else{
      if (millis()-timeOfClick > CLICK_TIME_SENSITIVITY && clickable ){
        AbsMouse.press(MOUSE_LEFT);
        AbsMouse.release(MOUSE_LEFT);
        timeOfClick = millis()+TIME_BETWEEN_CLICKS;
        buzzClick();
        
      }
    }
  }
}

void moveMouse(float x, float y){
  if(mouseAttached && GLOBAL_MOUSE_ATTACH){
    if (SERIAL_OUTPUT){
      Serial.print(X);
      Serial.print(",");
      Serial.println(Y);
    }
    AbsMouse.move(X,Y);
  } 
}

//***************************************************************************************
//******************              Callback Function                **********************
//***************************************************************************************

void callback_function (int16_t *gyro, int16_t *accel, int32_t *quat, uint32_t *timestamp) {
  uint8_t Spam_Delay = 10; // Built in Blink without delay timer preventing Serial.print SPAM
  orientation_from_Quaternion(gyro, accel, quat, timestamp, Spam_Delay);
}


//***************************************************************************************
//*******************              Buzzer Function                ***********************
//***************************************************************************************

void buzzGood(){
  if (BUZZ) {
    tone(5, 880, 200);
    delay(300);
  }
}

void buzzBad(){
  if (BUZZ){
  tone(5, 880, 2500);
  delay(3000);
  }
}

void buzzNeutral(){
  if (BUZZ){
  tone(5, 440, 300);
  delay(400);
  }
}

void buzzClick(){
  if (BUZZ){
  tone(5, 440, 5);
  delay(5);
  }
}

//***************************************************************************************
//******************                Setup and Loop                 **********************
//***************************************************************************************

void setup() {
  buzzGood();
  uint8_t val;
  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
#ifdef __AVR__  
  Wire.setWireTimeout(3000, true); //timeout value in uSec
#endif
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif
  if (SERIAL_OUTPUT){
    // initialize serial communication
    Serial.begin(9600);
    Serial.println(F("Start:"));
  }
    mpu.Set_DMP_Output_Rate_Hz(50);           // Set the DMP output rate from 200Hz to 5 Minutes.
  //mpu.Set_DMP_Output_Rate_Seconds(10);   // Set the DMP output rate in Seconds
  //mpu.Set_DMP_Output_Rate_Minutes(5);    // Set the DMP output rate in Minute

  if (SERIAL_OUTPUT){
    Serial.println(F("Using Offsets"));
  }
  mpu.SetAddress(MPU6050_DEFAULT_ADDRESS).load_DMP_Image(OFFSETS); // Does it all for you : calibrates automatically
  
  mpu.on_FIFO(callback_function); // defines callback function

  AbsMouse.init(XRESOLUTION, YRESOLUTION);
  
  mouseAttached = true;
  clickable = true;
  moveMouse(X, Y);
  delay(2000);
  buzzGood();
}



void loop() {
  mpu.dmp_read_fifo();// Must be in loop
}
