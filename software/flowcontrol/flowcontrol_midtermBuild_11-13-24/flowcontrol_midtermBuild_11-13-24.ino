/* ECET-430 Midterm - Flow Control Prototype
  We want this to exhibit the basics of what we're doing.
  Eventually we'll evolve this to two solenoids, but three and four is a pipe dream unless we can expand the IO pins.
  Each solenoid will be backed with a master switch
   
*/

//================//
// PIN DEFINITION //
//================//

// Define rotary encoder pins
#define ENC_A 2
#define ENC_B 3
#define ENC_SW 1
// MAIN - Main decision switch. Solenoid will not flow or count down if not receiving 5v. (off)
#define SOL1_MAIN 4
// OUT - Output valve control. Linked to the green LED.
#define SOL1_OUT 5
// WAIT - Hold visual. Linked to the red LED.
#define SOL1_WAIT 6
// SW_SEL - Pushbutton to select solenoid for configuration. Push once for Delay, push twice for Duration.
#define SOL1_SW_SEL 7
// SW_FORCE - Pushbutton to forcefully enable SOL1_OUT. This doesn't even need to exist and can be linked to a 5V.
//#define SOL1_SW_FORCE 11
#define SOL2_MAIN 8
#define SOL2_OUT 9
#define SOL2_WAIT 10
#define SOL2_SW_SEL 11
// It's impossible to expand to the fourth solenoid with the current microcontroller unless there's a way to expand the pins.
// We COULD squeeze one, pins 14, 15, 16, 17 for SOL3. Not SOL4 though. Three might as well be our absolute maximum.
// There are ways. An 8-pin expansion board would be ideal for this matter, but ends up relying on I2C communications, SDA, and SCL.
#define SOL3_MAIN 14
#define SOL3_OUT 15
#define SOL3_WAIT 16
#define SOL3_SW_SEL 17

#define HOUR_CONVERT 60

#include "HT16K33.h"

unsigned long _lastIncReadTime = micros(); 
unsigned long _lastDecReadTime = micros(); 
int _pauseLength = 25000;
int _fastIncrement = 10;

//======================//
// VARIABLE DEFINITIONS //
//======================//

// Solenoids based on the manual switch. MUST default to false if the switch is damaged.
bool solenoidEnabled[] = {false, false, false};
// Solenoids with individual duration counters, in seconds.
int solenoidDuration[] = {0, 0, 0};
// Solenoids with individual delay counters, in minutes.
// Neither the delay or duration counters can go below 0.
int solenoidDelay[] = {0, 0, 0};
// Current solenoid. Ranges from 1 to 3.
int solenoidSelect = 1;
// Editing what part of that solenoid. 1 for delay, 2 for duration.
int solenoidEditing = 1;

struct HrAndMin {
  int16_t hours;
  int16_t minutes;
};

// Constraints - if the counters ever go past these numbers, the code will force the value to the cap values.
#define SOL_MAX 1800
#define SOL_MIN 0

// This counter will be useful. Basically when you select a solenoid...
//  LOAD the DELAY to the COUNTER and UPDATE the value.
//  Make it display DELAY (DEly) for a second, then the actual counter itself.
//  Adjust the COUNTER to what you desire.
//  Press the encoder switch to SAVE the COUNTER to the DELAY.
//  Make the LCD say SET or something.
// Same goes if you press the select button again to select the solenoid's duration.

volatile int16_t counter = 0;
HT16K33 seg(0x70);

// Setup is to set all the variables and pins.
void setup();
// Loop is to consistently check and adjust values.
void loop();
// Switch is to identify what solenoid we're going to adjust.
void solenoid_switch(int ID);
// Duration can never go below 0.
int solenoid_flow(int ID, int duration);
// Included for the encoder module.
void read_encoder();
// Not included for the encoder module. Summarizes all things in the loop to this.
void dial_encoder(int newValue);
// Not included for the encoder module, but we'll want it to perform a saving action.
void press_encoder();
// We'll translate the raw counter numbers into hours, minutes, and seconds.
// This one function is double-purposed since if the value is in seconds, it'll be MMSS anyway.
int16_t base10ToHHMM();
// Displays a custom text line, like SET, Dura, DEly,
void displayText(uint8_t first, uint8_t second, uint8_t third, uint8_t fourth, int duration);
void display10ToHHMM(int value);

void setup() {

  // Set encoder pins and attach interrupts
  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);
  pinMode(ENC_SW, INPUT);
  pinMode(SOL1_OUT, OUTPUT);
  pinMode(SOL1_WAIT, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(ENC_A), read_encoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B), read_encoder, CHANGE);

  // Start the serial monitor to show output
  Serial.begin(115200);

  // Set up counter
  Wire.begin();
  Wire.setClock(100000);
  seg.begin();

  seg.displayOn();
  seg.setBrightness(2);
  seg.displayClear();
  seg.setBlink(0);
  seg.setDigits(1);
  delay(1000);
}

void loop() {
  static int lastCounter = 0;

  //HrAndMin value = {23, 55};
  //Serial.println(value.hours);
  //Serial.println(value.minutes);
  // If count has changed print the new value to serial
  if(counter != lastCounter)
  {
    lastCounter = counter;
    dial_encoder(counter);
  }
  /*
  if (digitalRead(SOL1_SW_SEL) == HIGH || digitalRead(SOL2_SW_SEL) == HIGH || digitalRead(SOL3_SW_SEL) == HIGH)
    solenoid_switch(digitalRead(SOL1_SW_SEL) == HIGH ? 1 : (digitalRead(SOL2_SW_SEL) == HIGH ? 2 : 3));
  */
  if (digitalRead(ENC_SW) == LOW)
  {
    press_encoder();
    // Midterm exclusive temporary measure.
    solenoid_flow(1, counter);
  }


  /*
  static uint32_t last = 0;
  uint32_t now = millis();
  if (now != last)
  {
    last = now;
    uint32_t s = now / 1000;
    uint32_t t = (now - s * 1000) / 10;
    Serial.read(t)
    if (s >= 1000)
    {
      m++;
      if(m > any minute given)
        h++;
    }
    
    //digitalWrite(SOL1_WAIT, LOW);
    // it'll probably blink every minute if it's far, but blink every second if it's close (< 5 minutes)

    s = s % 100;
    // seg.displayTime(s, t);
    //seg.displayTime(s, t, true, false);  // do not display leading zero.
    //seg.displayColon(1);
  }
  //delay(10);
  */
  
  
}

int solenoid_flow(int ID, int duration)
{
  switch(ID)
  {
    case 1: digitalWrite(SOL1_OUT, HIGH); break;
    case 2: digitalWrite(SOL2_OUT, HIGH); break;
    case 3: digitalWrite(SOL3_OUT, HIGH); break;
  }
  delay(duration * 1000);
  digitalWrite(SOL1_OUT, LOW);
  digitalWrite(SOL2_OUT, LOW);
  digitalWrite(SOL3_OUT, LOW);
  return;
}

void press_encoder(){
  //Serial.println(counter);
  switch(solenoidEditing)
  {
    case 1: solenoidDelay[solenoidSelect - 1] = counter; break;
    case 2: solenoidDuration[solenoidSelect - 1] = counter; break;
    default: displayText(SEG_A, SEG_A, SEG_A, SEG_NONE, 2.0f); return;
  }
  // print SEt for 2.5s
  displayText(SEG_A | SEG_F | SEG_G | SEG_C | SEG_D, SEG_A | SEG_D | SEG_E | SEG_F | SEG_G, SEG_F | SEG_E | SEG_G | SEG_D, SEG_NONE, 2.0f);
}

void dial_encoder(int newValue) {
    //counter = newValue;
    //lastCounter = counter;
    display10ToHHMM(counter);
    //Serial.println(counter);
}

void solenoid_switch(int ID){
  // If it's the same switch, switch between delay and duration. The second press leads to the selected solenoid's duration.
  if(solenoidSelect == ID)
    solenoidEditing = (solenoidEditing == 1 ? 2 : 1);
  else
  {
    solenoidSelect = ID;
    solenoidEditing = 1;
  }
  // If 1: display dELy
  //  SEG_B | SEG_C | SEG_D | SEG_E | SEG_G
  //  SEG_A | SEG_D | SEG_E | SEG_F | SEG_G
  //  SEG_D | SEG_E | SEG_F
  //  SEG_B | SEG_C | SEG_D | SEG_F | SEG_G
  // If 2: display durA
  //  SEG_B | SEG_C | SEG_D | SEG_E | SEG_G
  //  SEG_C | SEG_D | SEG_E
  //  SEG_E | SEG_G
  //  SEG_A | SEG_B | SEG_C | SEG_E | SEG_F | SEG_G
  if(solenoidEditing == 1)
    displayText(SEG_B | SEG_C | SEG_D | SEG_E | SEG_G, SEG_A | SEG_D | SEG_E | SEG_F | SEG_G, SEG_D | SEG_E | SEG_F, SEG_B | SEG_C | SEG_D | SEG_F | SEG_G, 2.0f);
  else
    displayText(SEG_B | SEG_C | SEG_D | SEG_E | SEG_G, SEG_C | SEG_D | SEG_E, SEG_E | SEG_G, SEG_A | SEG_B | SEG_C | SEG_E | SEG_F | SEG_G, 2.0f);
}

void read_encoder() {
  // Encoder interrupt routine for both pins. Updates counter
  // if they are valid and have rotated a full indent
 
  static uint8_t old_AB = 3;  // Lookup table index
  static int8_t encval = 0;   // Encoder value  
  static const int8_t enc_states[]  = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0}; // Lookup table

  old_AB <<=2;  // Remember previous state

  if (digitalRead(ENC_A)) old_AB |= 0x02; // Add current state of pin A
  if (digitalRead(ENC_B)) old_AB |= 0x01; // Add current state of pin B
  
  encval += enc_states[( old_AB & 0x0f )];

  // Update counter if encoder has rotated a full indent, that is at least 4 steps
  if( encval > 3 ) {        // Four steps forward
    int changevalue = 1;
    if((micros() - _lastIncReadTime) < _pauseLength) {
      changevalue = _fastIncrement * changevalue; 
    }
    _lastIncReadTime = micros();
    counter = counter + changevalue;              // Update counter
    encval = 0;
  }
  else if( encval < -3 ) {        // Four steps backward
    int changevalue = -1;
    if((micros() - _lastDecReadTime) < _pauseLength) {
      changevalue = _fastIncrement * changevalue; 
    }
    _lastDecReadTime = micros();
    counter = counter + changevalue;              // Update counter
    encval = 0;
  }
  if (counter < 0)
    counter = 0;
} 

// Base 10 to Hours:Minutes, take the value that's interpreted in minutes, and return it in a HH:MM format.
//  We can just put a bool of this and if it's true- or just not put it at all because the variables will just adjust itself.
HrAndMin base10ToHHMM(int16_t input)
{
  // Can be minutes if using a value that has seconds.
  //int hours = input / HOUR_CONVERT;
  // Can be seconds if using a value that has seconds.
  //int minutes = input % HOUR_CONVERT;
  //return {hours, minutes};
  return {input / HOUR_CONVERT, input % HOUR_CONVERT};
}

void displayText(uint8_t first, uint8_t second, uint8_t third, uint8_t fourth, float duration)
{
  // Suggestion: Prematurely cancel the delay if the knob is turned.
  uint8_t segment_data[] = {first, second, third, fourth};
  seg.displayClear();
  seg.displayColon(0);
  delay(100);
  seg.displayRaw(segment_data);
  delay(int(duration * 1000));
  seg.displayClear();
  delay(100);
  seg.displayInt(counter);
}

void display10ToHHMM(int value)
{
  seg.displayColon(1);
  HrAndMin toDisplay = base10ToHHMM(value);
  //int16_t toHours = toDisplay.hours;
  //int16_t toMinutes = toDisplay.minutes;
  seg.displayTime(toDisplay.hours, toDisplay.minutes, true, false);
}