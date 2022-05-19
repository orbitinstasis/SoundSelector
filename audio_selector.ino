/* Example program for from IRLib – an Arduino library for infrared encoding and decoding
   Version 1.5  June 2014
   Copyright 2014 by Chris Young http://cyborg5.com
   Based on original example sketch for IRremote library
   Version 0.11 September, 2009
   Copyright 2009 Ken Shirriff
   http://www.righto.com/
*/

/*
   IRrecord: records IR signals
   An IR detector/demodulator must be connected to the input IR_IN.
   Record a value by pointing your remote at the device
*/

/**
   Retrofitted IR Sleep code
*/

/**
   ############################################ INCLUDES
*/
#include <avr/sleep.h>
#include <avr/power.h>
#include <IRLibDecodeBase.h>  //We need both the coding and
#include <IRLib_P01_NEC.h>    //Lowest numbered protocol 1st
#include <IRLibCombo.h>       // After all protocols, include this
#include <IRLibRecv.h>
#include <Bounce2.h>
#include <EEPROM.h>

/**
   ############################################ DEFINES
*/
//buttons
#define SELECTOR_BTN 7
#define POWER_SWITCH 12

#define AUTO_VOLUME A1

//outputs
#define POWER_LED 8
#define SLEEP_LED 10
#define SEL_4 13
#define SEL_3 6
#define SEL_2 5
#define SEL_1 4
#define MOTOR_REPLACE1 3
#define MOTOR_REPLACE2 A0
#define HIGH_POWER_SWITCH A2

#define IR_IN 11
#define NA_1 9

#define EEPROM_ADDR 0
#define ALL_SELECTIONS_OFF 255
#define BTN_DEBOUNCE_TIME 25
#define BAUD_RATE 115200
#define NO_ACTIVITY_TIMEOUT 2000  // milliseconds

// IR Codes
#define IR_POWER 0xBD807F
#define IR_INPUT 0xBD10EF
#define IR_MUTE 0xBD20DF
#define IR_VOL_POS 0xBDD02F
#define IR_VOL_NEG 0xBDF00F
#define IR_BRIGHT_POS 0xBD52AD
#define IR_BRIGHT_NEG 0xBD926D
#define IR_REPEAT 0xFFFFFFFF
#define IR_DISPLAY 0xBD40BF
#define IR_MODE 0xBDC03F
#define IR_SIZE 0xBD28D7
#define IR_MENU 0xBD50AF

#define SERIAL_ENABLED  true


// All of the above automatically creates a universal decoder
// class called "IRdecode" and a universal sender class "IRsend"
// containing only the protocols you want.
// Now declare instances of the decoder and the sender.

IRrecv  myReceiver(IR_IN);
IRdecode myDecoder;
unsigned long lastCode = IR_REPEAT;

unsigned long previousSleep = 0;
unsigned long previousVolCapture = 0;
unsigned long currentMillis = 0;
static unsigned long lastActivity;  // when we last woke

boolean is_hold = false;
boolean ir_power_pressed = false;
boolean ir_selector_pressed = false;
boolean ir_mute_pressed = false;
boolean is_on = false;
boolean am_moving = false; 

Bounce debouncer_selector = Bounce();
Bounce debouncer_power = Bounce();

uint8_t intput_pulled_up[] = {POWER_SWITCH, AUTO_VOLUME, SELECTOR_BTN};
uint8_t outputs[] = {POWER_LED, SLEEP_LED, SEL_4, SEL_3, SEL_2, SEL_1, MOTOR_REPLACE1, MOTOR_REPLACE2, HIGH_POWER_SWITCH};
uint8_t selection_value = 0;
uint8_t selections[] = {SEL_1, SEL_2, SEL_3, SEL_4};

/*
   Because this version of the library separated the receiver from the decoder,
   technically you would not need to "store" the code outside the decoder object
   for this overly simple example. All of the details would remain in the object.
   However we are going to go ahead and store them just to show you how.
*/
// Storage for the recorded code
uint8_t codeProtocol;          // The type of code
uint32_t codeValue;   // The data bits if type is not raw
uint8_t codeBits;              // The length of the code in bits

// added by Nick Gammon
EMPTY_INTERRUPT (PCINT0_vect);


/**
   ############################################ ############################################ SETUP
*/
void setup()
{
  power_adc_disable(); // ADC converter
    if (SERIAL_ENABLED)
    power_usart0_enable();// Serial (USART)
  else
    power_usart0_disable();// Serial (USART)
  
  power_timer1_disable(); // Timer 1
  power_twi_disable(); // TWI (I2C)
  codeProtocol = UNKNOWN;
  codeValue = 0;
  if (SERIAL_ENABLED)
    Serial.begin(BAUD_RATE);
  for (int i = 0; i < 3; i++)
  {
    pinMode(intput_pulled_up[i], INPUT_PULLUP);
  }
  pinMode(IR_IN, INPUT); //CHECK IF THIS NEEDS TO BE PULLED UP

  for (int i = 0; i < 9; i++)
  {
    pinMode(outputs[i], OUTPUT);
    digitalWrite(outputs[i], LOW);
  }
  digitalWrite(SLEEP_LED, HIGH);
  digitalWrite(POWER_LED, LOW);

  debouncer_selector.attach(SELECTOR_BTN);
  debouncer_selector.interval(BTN_DEBOUNCE_TIME);

  debouncer_power.attach(POWER_SWITCH);
  debouncer_power.interval(BTN_DEBOUNCE_TIME);

  uint8_t eeprom_read = EEPROM.read(EEPROM_ADDR);
  delay(10);
  if ((eeprom_read >= 0) || (eeprom_read <= 3))
    selection_value = eeprom_read;
  else
    selection_value = 0;
  myReceiver.enableIRIn();
  currentMillis = millis();
  previousVolCapture = millis();
  previousSleep = millis();
  myReceiver.enableIRIn(); // Start the receiver
}

/**
   ################################################################################################ MAIN LOOP
*/
void loop() {

  // sleep if no activity for a while
  if (!am_moving && (millis () - lastActivity >= NO_ACTIVITY_TIMEOUT))
  {
    Serial.println("GOing to sleep");
    Serial.flush ();  // wait for Serial to finish outputting
    Serial.end ();    // shut down Serial

    noInterrupts ();  // timed sequence coming up

    // pin change interrupt for D11
    PCMSK0 |= bit (PCINT3);  // want pin 11
    PCIFR  |= bit (PCIF0);   // clear any outstanding interrupts
    PCICR  |= bit (PCIE0);   // enable pin change interrupts for D8 to D13

    set_sleep_mode (SLEEP_MODE_STANDBY);
    sleep_enable();

    byte old_ADCSRA = ADCSRA;
    // disable ADC and SPI to save power
    
    ADCSRA = 0;
    SPCR = 0; //disable SPI
    power_spi_disable() ;
    power_timer0_disable();
    power_timer2_disable();

    
    
    interrupts ();
    
    sleep_cpu ();
    sleep_disable();
    
    SPCR = 1; //enable SPI
    power_spi_enable() ;
    power_timer0_enable();
    power_timer2_enable();
    ADCSRA = old_ADCSRA;   // re-enable ADC conversion
    


    if (SERIAL_ENABLED)
      Serial.begin(BAUD_RATE);
    lastActivity = millis ();
  }  // end of no activity for a couple of seconds

    if (am_moving && currentMillis - previousVolCapture > 250)
    {
      Serial.println("Stopping motor");
      stop_motor();
      am_moving = false; 
      previousVolCapture = currentMillis;
      lastActivity = millis ();
    }

  if (myReceiver.getResults()) {
    myDecoder.decode();
    processIrData();
    delay(50);

    currentMillis = millis();


    debouncer_power.update();
    debouncer_selector.update();

    if (debouncer_power.rose() || ir_power_pressed)
    {
      ir_power_pressed = false;
      if (!is_on)
      { // POWER ON SLEEP OFF
        is_on = true;
        selector(selection_value);
        digitalWrite(SLEEP_LED, LOW);
        digitalWrite(HIGH_POWER_SWITCH, HIGH);
      }
      else
      { //SLEEP ON POWER OFF
        is_on = false;
        selector(ALL_SELECTIONS_OFF);
        stop_motor();
        digitalWrite(SLEEP_LED, HIGH);
        digitalWrite(HIGH_POWER_SWITCH, LOW);
      }
    }

    if (debouncer_selector.rose() || ir_selector_pressed)
    {
      ir_selector_pressed = false;
      if (is_on)
      {
        if (selection_value >= 3)
        {
          selection_value = 0;
        }
        else
          selection_value++;
        EEPROM.write(EEPROM_ADDR, selection_value);
        selector(selection_value);
      }
    }

    myReceiver.enableIRIn();
    lastActivity = millis ();

  }

}

/**
   ############################################ FUNCTIONS
*/
void processIrData(void) {

  is_hold = false;
  // codeProtocol = myDecoder.protocolNum;
    codeValue = myDecoder.value;
    // codeBits = myDecoder.bits;
  if (codeValue == REPEAT_CODE) {
    is_hold = true;
    codeValue = lastCode;      // replace IR_REPEAT with last good code

    // lastActivity = millis ();
  }

    if (codeValue == IR_POWER && !is_hold)
    {
      lastCode = codeValue;
      ir_power_pressed = true;
    }

    if (codeValue == IR_MUTE && is_on && !is_hold)
    {
      lastCode = codeValue;
      ir_mute_pressed = !ir_mute_pressed;
      if (ir_mute_pressed)
      {
        digitalWrite(POWER_LED, HIGH);
        selector(ALL_SELECTIONS_OFF);
      }
      else
      {
        digitalWrite(POWER_LED, LOW);
        selector(selection_value);
      }
    }

    if (codeValue == IR_INPUT && is_on && !is_hold)
    {
      lastCode = codeValue;
      ir_selector_pressed = true;

    }

    if (codeValue == IR_BRIGHT_POS && is_on && !is_hold)
    {
      lastCode = codeValue;
      move_motor(true, 2000);
    }

    if (codeValue == IR_BRIGHT_NEG && is_on && !is_hold)
    {
      lastCode = codeValue;
      move_motor(false, 2000);
    }

    if (codeValue == IR_VOL_POS && is_on )
    {
      lastCode = codeValue;
      move_motor(true, 0);
      am_moving = true;
      previousVolCapture = millis();
      Serial.println("Am moving vol");
    }

    if (codeValue == IR_VOL_NEG && is_on )
    {
      lastCode = codeValue;
      move_motor(false, 0);
      am_moving = true;
      previousVolCapture = millis();
      Serial.println("Am moving vol");
    }
  
}

void stop_motor()
{
  digitalWrite(MOTOR_REPLACE1, LOW);
  digitalWrite(MOTOR_REPLACE2, LOW);
}

void selector(uint8_t in)
{
  if (in == ALL_SELECTIONS_OFF)
  {
    for (int i = 0; i < 4; i++)
    {
      digitalWrite(selections[i], LOW);
    }
  }
  else
  {
    for (int i = 0; i < 4; i++)
    {
      if (i == in)
        digitalWrite(selections[i], HIGH);
      else
        digitalWrite(selections[i], LOW);
    }
  }
}

void move_motor(boolean isClockwise, int _time) // time in seconds
{
  if (isClockwise)
  {
    digitalWrite(MOTOR_REPLACE1, LOW);
    digitalWrite(MOTOR_REPLACE2, HIGH);
  }
  else
  {
    digitalWrite(MOTOR_REPLACE1, HIGH);
    digitalWrite(MOTOR_REPLACE2, LOW);
  }
  if (_time > 0)
  {
    delay(_time);
    stop_motor();
  }
}
