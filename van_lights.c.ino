#include <LowPower.h>
#include <AceButton.h>
using namespace ace_button;

//outputs
int dcon = 3; // General Mosfet switch
int lamp1 = 9;
int lamp2 = 10;
int lamp3 = 5;
//inputs
int sw1 = 4;
int sw2 = 6;
int sw3 = 7;
int isrpin = 2;
//variables
int brightness = 128;
int fadeAmount = 1;
int onfade = 0;
int L1ON = 0;
int L2ON = 0;
int L3ON = 0;
static int sleeping = 1;
int iclicks = 0;
unsigned long start;
//constants
#define min_pwm 5
#define max_pwm 255
ButtonConfig* buttonConfig;
AceButton bt1(sw1);
AceButton bt2(sw2);
AceButton bt3(sw3);
void handleEvent(AceButton*, uint8_t, uint8_t);

int toggle_pin(unsigned char pin, int activated){
  if (activated==1)
  {
    analogWrite(pin, 0);
    return 0;
  }
  else
  {
    analogWrite(pin, brightness);
    return 1;
  }
}

void maxout(){
  L1ON = 1;
  L2ON = 1;
  L3ON = 1;
  brightness = max_pwm;
  analogWrite(lamp1, brightness);
  analogWrite(lamp2, brightness);
  analogWrite(lamp3, brightness);
}

void all_off(){
  L1ON = 0;
  L2ON = 0;
  L3ON = 0;
  //brightness = max_pwm;
  analogWrite(lamp1, 0);
  analogWrite(lamp2, 0);
  analogWrite(lamp3, 0);
}

void isrbutton(){
}

void setup() {
  pinMode(sw1, INPUT_PULLUP);
  pinMode(sw2, INPUT_PULLUP);
  pinMode(sw3, INPUT_PULLUP);
  pinMode(isrpin, INPUT_PULLUP);
  pinMode(lamp1, OUTPUT);
  pinMode(lamp2, OUTPUT);
  pinMode(lamp3, OUTPUT);
  pinMode(dcon, OUTPUT);
  digitalWrite(dcon, LOW);
  analogWrite(lamp1, 0);
  analogWrite(lamp2, 0);
  analogWrite(lamp3, 0);  
  //Serial.begin(115200);
  attachInterrupt( digitalPinToInterrupt (isrpin), isrbutton, FALLING);
  buttonConfig = ButtonConfig::getSystemButtonConfig();
  buttonConfig->setEventHandler(handleEvent);
  buttonConfig->setFeature(ButtonConfig::kFeatureClick);
  buttonConfig->clearFeature(ButtonConfig::kFeatureDoubleClick);
  buttonConfig->clearFeature(ButtonConfig::kFeatureLongPress);
  buttonConfig->setClickDelay(500);
  buttonConfig->setDoubleClickDelay(600);  
}

void handleEvent(AceButton* button, uint8_t eventType,
    uint8_t buttonState) {
  switch (eventType) {
    case AceButton::kEventClicked:
      //Serial.println("click");
      iclicks++;
      if (onfade==1)
        onfade++;
      else
      {        
        if (button->getPin() == sw1) {
         L1ON = toggle_pin(lamp1,L1ON);
        }
        else if (button->getPin() == sw2) {
          L2ON = toggle_pin(lamp2,L2ON);
        }
        else if (button->getPin() == sw3) {
          L3ON = toggle_pin(lamp3,L3ON);
        }
      }
      break;
    case AceButton::kEventDoubleClicked:
      //Serial.println("double click");
      if (iclicks<=1)
        maxout();
      else
        all_off();
      break;       
    case AceButton::kEventLongPressed:
      //Serial.println("long press");
      /*if (button->getPin() == sw2) {
        onfade = 1;
      }*/
      onfade = 1;
      break; 
  }
  //Serial.flush();  
}

void fade() {
  if ((L1ON>0)||(L2ON>0)||(L3ON>0)){
    //Serial.println("fading...");
    fadeAmount = -abs(fadeAmount);
    while(onfade==1) {
      if (L1ON>0) analogWrite(lamp1, brightness);
      if (L2ON>0) analogWrite(lamp2, brightness);
      if (L3ON>0) analogWrite(lamp3, brightness);
      brightness = brightness + fadeAmount;
      if (brightness>max_pwm) brightness = max_pwm;
      else if (brightness<min_pwm) brightness = min_pwm;
      if (brightness <= min_pwm || brightness >= max_pwm) fadeAmount = -fadeAmount;
      delay(10);
      bt1.check();
      bt2.check();
      bt3.check();
    }
    //Serial.println("exit fading...");
  }
  else{
    onfade = 0;
  }
}

void loop() {
  start = millis();
  while (sleeping == 1)
  {
    bt1.check();
    bt2.check();
    bt3.check();
    if ((L1ON>0)||(L2ON>0)||(L3ON>0)||((millis() - start) > 5000))
      sleeping = 0;
  }
  bt1.check();
  bt2.check();
  bt3.check();

  if (onfade==1)
  {
    fade();
  }
  if ((L1ON>0)||(L2ON>0)||(L3ON>0))
  {
    digitalWrite(dcon, HIGH);
    buttonConfig->setFeature(ButtonConfig::kFeatureDoubleClick);
    buttonConfig->setFeature(ButtonConfig::kFeatureLongPress);
  }
  else 
  {
    sleeping = 1;
    digitalWrite(dcon, LOW);
    //Serial.println("Down");
    //Serial.flush();
    buttonConfig->clearFeature(ButtonConfig::kFeatureDoubleClick);
    buttonConfig->clearFeature(ButtonConfig::kFeatureLongPress);
    iclicks = 0;
    LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);
    //Serial.println("Up");
    //Serial.flush();    
  }
}
