//////// ESP task stuff //////
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
hw_timer_t * timer = NULL;
hw_timer_t * timer2 = NULL;
TaskHandle_t displaytaskhandle;

//////// graphics //////
//sharp LCD
#include <Adafruit_SharpMem.h>
#include "ovlogo.h"
#define SHARP_SCK  14
#define SHARP_MOSI 15
#define SHARP_SS   32
Adafruit_SharpMem sharp(SHARP_SCK, SHARP_MOSI, SHARP_SS, 144, 168);

// wavefront eink
#include <GxEPD.h>
#include <GxGDEP015OC1/GxGDEP015OC1.cpp>    // 1.54" b/w
#include <GxIO/GxIO_SPI/GxIO_SPI.cpp>
#include <GxIO/GxIO.cpp>
GxIO_Class io(SPI, 21, 23, 22);
GxEPD_Class waveshare(io, 22, 17);

// common gfx
#include <Adafruit_GFX.h>
#include <Fonts/FreeSans24pt7b.h>
#include <Fonts/FreeSans18pt7b.h>
#include <Fonts/FreeSans12pt7b.h>
#include <Fonts/font.h>
int BLACK = GxEPD_BLACK;
int WHITE = GxEPD_WHITE;


/////////// Stepper motor ////////////
#include "SwitecX12.h"
const unsigned int STEPS = 315 * 12;
const unsigned int A_STEP =  33;//A0 was 22 ;
const unsigned int A_DIR = 27; //A1 was 23;
const unsigned int RESET = 12; // was 21 ;
SwitecX12 motor1 (STEPS, A_STEP, A_DIR);
int target = 0; //stepper target

////////// encoder
#include <SPI.h>
#include <ClickEncoder.h>
#define ENCODER_PINA     22//33
#define ENCODER_PINB     23//27
#define ENCODER_BTN      21//12
#define ENCODER_STEPS_PER_NOTCH    4
ClickEncoder encoder = ClickEncoder(ENCODER_PINA, ENCODER_PINB, ENCODER_BTN, ENCODER_STEPS_PER_NOTCH);
int16_t last, value;

//timers
unsigned long varioTimer;
unsigned long stepperTimer;
unsigned long currentMillis;

// vario stuff
volatile float vario, varioDelta, avg, alt;
volatile int sVario, iVario, previousiVario, varioDeltaV, stf;
int dir = 1;
boolean noData=1;

// paint stuff
int x, y;
float data[40] = {0}; //for condor*/

 
////////////////// ISR /////////////////////////

void encoder_isr() {
  encoder.service();
  // value += encoder.getValue();
  if (value != last) {
    last = value;
    //Serial.print("Encoder Value: ");
    //Serial.println(value);
  }
}

void stepper_isr() {

  //long int currentMicros=micros();
  long int tmp = (micros() - 1000 * varioTimer) / 1000;
  sVario = previousiVario + varioDeltaV * tmp / 1000;
  sVario = constrain(sVario, -5000, 5000);
  target =  2210 + sVario * 0.3; //
  motor1.setPosition(target);
  motor1.update();
  //stepperTimer = currentMillis;
}

//////////////////// Arduino //////////////////////////////////////

void setup() {
  Serial.begin(115200);
  encoder.setButtonHeldEnabled(true);
  encoder.setDoubleClickEnabled(false);
  encoder.setButtonOnPinZeroEnabled(true);

  // init vars
  
  avg = 0;
  varioTimer = millis();
  stepperTimer = millis();
  value = 50;
  vario = 4.8;
  iVario = int(vario * 1000);
  splash();

  // Stepper setup
  pinMode(A_STEP, OUTPUT);
  pinMode(A_DIR, OUTPUT);
  pinMode(RESET, OUTPUT);
  digitalWrite(RESET, HIGH);
  motor1.zero();
  motor1.stepTo(STEPS);
  delay(1500);
  motor1.stepTo(3780);  //-5
  delay(500);
  motor1.stepTo(600);  //+5
  delay(500);

  // encoder ISR setup
  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &encoder_isr, true);
  timerAlarmWrite(timer, 1000, true); //1000
  timerAlarmEnable(timer);

  // stepper ISR setup
  timer2 = timerBegin(1, 80, true);
  timerAttachInterrupt(timer2, &stepper_isr, true);
  timerAlarmWrite(timer2, 50, true); //1000
  timerAlarmEnable(timer2);


  // RTOS task setup
  xTaskCreatePinnedToCore(
    einkTask,           /* Task function. */
    "update e_ink display",        /* name of task. */
    2000,                    /* Stack size of task */
    NULL,                     /* parameter of the task */
    50,                        /* priority of the task */
    NULL,     /* Task handle to keep track of created task */
    1);

  xTaskCreatePinnedToCore(
    lcdTask,           /* Task function. */
    "update Sharp LCD",        /* name of task. */
    2000,                    /* Stack size of task */
    NULL,                     /* parameter of the task */
    55,                        /* priority of the task */
    &displaytaskhandle,     /* Task handle to keep track of created task */
    0);

  xTaskCreatePinnedToCore(
    varioTask,           /* Task function. */
    "produce vario test data",        /* name of task. */
    1000,                    /* Stack size of task */
    NULL,                     /* parameter of the task */
    20,                        /* priority of the task */
    NULL,
    0);     /* Task handle to keep track of created task */

  xTaskCreatePinnedToCore(
    serialTask,           /* Task function. */
    "check for serial data",        /* name of task. */
    5000,                    /* Stack size of task */
    NULL,                     /* parameter of the task */
    10,                        /* priority of the task */
    NULL,
    1);     /* Task handle to keep track of created task */
}



/////////////////////// RTOS Tasks ///////////////////

void einkTask(void *pvParameters) {
  UBaseType_t uxHighWaterMark;
  uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
  // Serial.println(uxHighWaterMark);
  while (1) {
    // paintwaveshare();
    uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
    //  Serial.println(uxHighWaterMark);
    delay(100);
  }
}

void lcdTask(void *pvParameters) {
  UBaseType_t uxHighWaterMark;
  uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
  // Serial.println(uxHighWaterMark);
  while (1) {
    paintsharp();
    uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
    //  Serial.println(uxHighWaterMark);
    delay(50);
  }
}

void varioTask(void *pvParameters) {
  while (1) {
    if (noData)  variotest();
    //condor();
    delay(100);
  }
}

void serialTask(void *pvParameters) {
  while (1) {
    checkserial();
    delay(1);
  }
}

////////////////////////////////////////////////////////////////////////

void loop() {
   if (millis() -varioTimer > 5000) {
    noData=1;    
  }
  delay(1000);
}

////////////////////////////////////////////////////////////////////////

void splash() {
  waveshare.init();
  waveshare.setRotation(1);
  waveshare.fillRect(0, 0, 200, 200, WHITE);
  waveshare.drawBitmap(0, 0, wavesharelogo, 200, 200, BLACK);
  waveshare.update();

  sharp.begin();
  sharp.setRotation(2);
  sharp.clearDisplay();
  sharp.drawBitmap(0, 0, sharplogo, 144, 168, GxEPD_BLACK); // always black
  sharp.refresh();
  delay(1000);
  sharp.setRotation(3);
}

void projectborder (int rc, int ixres, int iyres, float alpha, float edgedistance) {
  // calculate  X and Y coordinates on curved rectangle for given angle and distance
  float beta = 0; // angle relative to corner circles.
  float xres = float(ixres);
  float yres = float(iyres);
  float refangle1 = atan((yres / 2.0 - rc) / (xres / 2.0)); // angle between middle screen and edge of round segments
  float refangle2 = atan(PI / 2 - (xres / 2.0 - rc) / (yres / 2.0));
  float A = xres / 2.0; // distance between middle of screen and edge.  its recalculated for the round segments
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////
  if (alpha < -refangle1 && alpha > -refangle2) {
    beta = alpha + PI / 2; // lower left corner
    //      Serial.println ("lower left corner");
  }
  else if (alpha > refangle1 && alpha < refangle2) {
    beta = alpha; // upper left corner
    //      Serial.println ("upper left corner");
  }
  else if (alpha > refangle1 + PI / 2 ) {
    beta = alpha - PI / 2; // upper right corner
    //       Serial.println ("upper right corner");
  }
  else if (alpha < -refangle1 - PI / 2) {
    beta = alpha + PI; //lower right corner
    //     Serial.println ("lower right corner");
  }

  if (beta != 0) {
    // round segment
    float c1 = beta - atan(float(yres) / float(xres));
    float O = rc / sin(c1);
    float tmp1 = pow((xres / 2.0 - rc), 2);
    float tmp2 = pow((yres / 2.0 - rc), 2);
    float B = sqrt(tmp1 + tmp2);
    float b1 = asin(B / O);
    //A=rc/sin(beta-PI/4) * sin (PI - asin(B*sin(c')/rc) - c'); see math solution Tim.
    float a1 = PI - b1 - c1;
    A = O * sin(PI - b1 - c1) - edgedistance ;
    x = xres / 2.0 - A * cos(alpha);
    y = yres / 2.0 - A * sin(alpha);
  } else {
    // straight segment
    x = xres / 2.0 - (A - edgedistance) * cos(alpha);
    y = yres / 2.0 - (A - edgedistance) * sin(alpha);
    if (abs(alpha) <= refangle1) x = edgedistance; //left vertical
    else if (alpha >= PI / 2 - refangle2) y = edgedistance; // top horizontal
    else y = yres - edgedistance; //bottom horizontal
  }
}


void smoothvario() {
  // do floating point math here, so we can avoid it in ISR.

  // simulate altitude & STF & average as long as OV doesnt provide this data
  alt += vario * (  millis() - varioTimer ) / 1000.0;
  //Serial.println(vario *(  millis() - varioTimer ) /1000.0);
  alt = max(0, alt);
  float fact = 0.005;
  avg = vario * fact + (1 - fact) * avg;
  if (avg > 0) stf = 120 - vario;
  else stf = 120 - 20 * avg;


  // interpoloation  
  currentMillis = millis();
  iVario = vario * 1000;
  float tmp1 =  (previousiVario - iVario);
  long int tmp2 =  varioTimer - currentMillis;
  float tmp3 =  tmp1 / tmp2;
  varioDeltaV = int(tmp3 * 1000);
  //Serial.println(tmp2 );
  varioTimer = millis();
  
}

void variotest() {
  // generate some test data when no data is received via OV serial port
  boolean switched;
  float varioDelta;
  previousiVario = iVario;
  currentMillis = millis();
  if ((vario <= -5  || vario >= 5) && !switched)  {
    dir *= -1; // switch direction
    //BLACK = !BLACK;
    //WHITE = !WHITE;
    switched = 1;
    //delay(500);
  }
  if (vario > 1)  switched = 0;
  value += random(0, 10) - 5;
  value = constrain(value, 25, 100);

  varioDelta = dir * abs(value) / 2000.0 + dir * value * abs(vario) / 10000.0;
  vario += varioDelta * (millis() - varioTimer) / 10;
  if (random(0, 50) == 0) vario *= -1;
  if (random(0, 100) == 0) dir *= -1;
  if (random(0, 1000) == 0) vario = 0;
  if (random(0, 100) == 0) vario += 1;

  //vario+=random(-100,100)/50;
  vario = constrain(vario, -5, 5);
 
  smoothvario();
}


char* string2char(String command) {
  if (command.length() != 0) {
    char *p = const_cast<char*>(command.c_str());
    return p;
  }
}


void checkserial() {
  static char buffer[80]; // get data from openvario com port
  String received = "", msgType, varioT;
  int ind1, ind2, ind3, ind4, ind5;
// Serial.println("checking serial");
  if (readline(Serial.read(), buffer, 80) > 0) {
    received = buffer;
    if (received.substring(0, 4) == "$POV") {
   //   Serial.print("got vario:> ");
      ind1 = received.indexOf(',');  //finds location of first ,
      ind2 = received.indexOf(',', ind1 + 1 ); //Position of P or E or V
      msgType = received.substring(ind1 + 1, ind2); //P or E or V
      // Serial.println(msgtype);
      if (msgType == "E") { // vario message
        ind3 = received.indexOf(',', ind2 + 1 ); //value
        varioT = received.substring(ind2 + 1, received.length()); //IAS
        ind4 = varioT.indexOf('*'); //position of checksum data at end
        varioT = varioT.substring(0, ind4); //strip checksumdata
        vario = varioT.toFloat();
        vario= constrain(vario,-5,5);
        noData = 0;
        previousiVario = iVario;
        iVario = vario * 1000;
        smoothvario();
        /// quick setting of target, normally overriden by interpolation in stepper ISR
          target =  2210 + iVario * 0.3; //
          motor1.setPosition(target);
        ///
        
      }
    } else if (received.substring(0, 1) == "z") {
      noData = 0;
    }
  }
}

int readline(int readch, char *buffer, int len) {
  static int pos = 0;
  int rpos;
  if (readch > 0) {
    switch (readch) {
      case '\n': // Ignore new-lines
        break;
      case '\r': // Return on CR
        rpos = pos;
        pos = 0;  // Reset position index ready for next time
        return rpos;
      default:
        if (pos < len - 1) {
          buffer[pos++] = readch;
          buffer[pos] = 0;
        }
    }
  }
  // No end of line has been found, so return -1.
  return -1;
}

