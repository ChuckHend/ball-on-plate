//main controller for ball on plate system
//reads touch screen, runs PID, controls servos
//sends data via serial

#include <Servo.h>
#include "Adafruit_STMPE610.h"
#include <PID_v1.h>
#include <EEPROM.h>

#define dt 80     //cycle time in milliseconds
#define xPin 10   //x axis to pin 10
#define yPin 9    //y axis to pin 9

Adafruit_STMPE610 touchPanel = Adafruit_STMPE610();

//declare servos
Servo servoX, servoY;

boolean inActive = true; //ball off plate by default

double OUTPUT_MIN = -45,
       OUTPUT_MAX = 45,
       setPointX     = 500, //center of X plane
       setPointY     = 500, //center of Y plane
       
       currX         = 0,   //current X
       currY         = 0,   //current Y

       Kp = 0.025,
       Ki = 0.020,
       Kd = 0.0018,

       outputX,               //output from xPID class
       outputY;               //output from yPID class
       
unsigned long t =         0,  //for fixing loop time
              touched =   0;  //counting time last touched
     
 int servoNeutralX  = EEPROM.read(0),//40,    //x axis, increase for CW rot.[40]
     servoNeutralY  = EEPROM.read(1),//57,    //y axis. increase for CW rot.[57]
     servoRequest_x = 0,     //x servo, sum of PIDD responses
     servoRequest_y = 0,     //y servo, sum of PIDD responses

     errX,
     errY,
     x_last = 0,
     y_last = 0;
     
uint16_t x, y;              // for touchscreen variables
uint8_t z;                  // touchscreen variables


PID PIDx(&currX, &outputX, &setPointX, Kp, Ki, Kd, DIRECT);//MKI direct, MKII reverse
PID PIDy(&currY, &outputY, &setPointY, Kp, Ki, Kd, REVERSE);//MKI direct, MKII reverse

void setup() {
  Serial.begin(115200);      //baud rate

    //init touchscreen
  Serial.flush();
  if (! touchPanel.begin()) {
    Serial.println("ERROR: STMPE controller not found");
    while (1);
  }
  
  //init servos
  //pulse width default is 544-2400
  //hitec is  750-2250μsec per datasheet
  servoX.attach(xPin, 750, 2250);
  servoY.attach(yPin, 750, 2250);

  //set both servos to horizontal plate positions
  servoX.write(servoNeutralX);
  servoY.write(servoNeutralY);

  //set some parameters for PID class controller
  PIDx.SetMode(AUTOMATIC);
  PIDy.SetMode(AUTOMATIC);
  PIDx.SetOutputLimits(OUTPUT_MIN, OUTPUT_MAX);
  PIDy.SetOutputLimits(OUTPUT_MIN, OUTPUT_MAX);
  PIDx.SetSampleTime(10);
  PIDy.SetSampleTime(10);
  
}

void loop() {
  t = millis();

  getTouchPanel();
  
  PIDx.Compute();
  PIDy.Compute();
  
  servoRequest_x = servoNeutralX + outputX;
  servoRequest_y = servoNeutralY + outputY;

  write_servos();
  
  pyPrint();
  
  while ((millis() - t) < dt) { 
    // Making sure the cycle time is equal to dt
    //do nothing
  }

  check_stable();
  
  x_last = currX;
  y_last = currY;
}

void check_stable(){
  errX = abs(setPointX - x_last);
  errY = abs(setPointY - y_last);

  if (errX < 10){
    EEPROM.write(0, servoRequest_x);
    Serial.println("Saving X: ");
    Serial.println(servoRequest_x);
  }

  if (errY < 10){
    EEPROM.write(1, servoRequest_y);
    Serial.println("Saving Y: ");
    Serial.println(servoRequest_y);
  }

}


void write_servos(){
  
  if(millis() - touched > 500){
    inActive = true;
    currX = setPointX;
    currY = setPointY;
  }
  else {
    inActive = false;
    }

  if(inActive){
    //if inactive, set neutral positions
    servoX.write(servoNeutralX);
    servoY.write(servoNeutralY);
    }
  else if (!inActive){
    servoX.write(servoRequest_x);
    servoY.write(servoRequest_y);
  }
}

void pyPrint(){
  //printing format for visualization
  Serial.print(setPointX);
  Serial.print(",");
  Serial.print(currX);
  Serial.print(",");
  Serial.print(outputX);
  Serial.print(",");
  Serial.print(setPointY);
  Serial.print(",");
  Serial.print(currY);
  Serial.print(",");
  Serial.print(servoRequest_x);
  Serial.print(",");
  Serial.print(servoRequest_y);
  Serial.print(",");
  Serial.println(t);
}

void getTouchPanel(){
  if (touchPanel.touched()) {
    while (! touchPanel.bufferEmpty()) {
      //Serial.print(touchPanel.bufferSize());
      touchPanel.readData(&x, &y, &z);
      currX = map(x, 0, 3950, 0, 1000);//map positions 0 to 1000
      currY = map(y, 0, 3900, 0, 1000);
    }
    touchPanel.writeRegister8(STMPE_INT_STA, 0xFF); // reset all ints
    touched = millis();
  }//end touchpanel acquisition

}//end getTouchPanel

bool stable(){
  
}

