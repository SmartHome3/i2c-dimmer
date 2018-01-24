#include <Arduino.h>
#include <Wire.h>
#include <EnableInterrupt.h>

// Uses Timer1 to delay pulsing a TRIAC by an amount of time relative to
// detecting a zero crossing of the A/C signal. Increasing the delay decreases
// the amount of time current flows through the lamp and dims it.
// Amount of time to delay is passed in as a brightness level via i2c
// and translated to the required delay.

// Timer1 Setup
//
// Assume CLK = 8MHz
// TCCR1 |= (1 << CS13) | (1 << CS11) == CLK/512 == 15625Hz
// Zerocrossing for 50Hz AC is 100Hz
//   -- giving 156.25 ticks beetween Zerocrossing

#define I2C_SLAVE_ADDRESS 0x4

#define DETECT PB4  //zerocrossing detect pin
#define GATE PB3    //TRIAC gate pin

#define PULSE 2  //TRIAC trigger pulse width (in timer counts)

#define MIN   40  //Min phase cut delay before keeping the traic switched ON
#define MAX  150  //Max phase cut delay before keeping the triac switched OFF

// Brightness is on the scale from 0 thru 100
// 0 = COMPLETLY OFF, 100 = COMPLETLY ON
uint8_t brightness = 0;

//zero cross detect interrupt hander
void zeroCrossingInterrupt() {

  int delay = bri2dim(brightness);

  //Should we just leave the triac competely ON or OFF
  if (delay <= MIN)
  {
    digitalWrite(GATE, HIGH); //Leave On
  }
  else if (delay >= MAX)
  {
    digitalWrite(GATE, LOW); //Leave Off
  }
  else
  {
    OCR1A = delay;
    TCCR1 |= (1 << CTC1);
    TCCR1 |= (1 << CS13) | (1 << CS11); // Timer1/512
  }
}

// Time OFF delay after Zerocrossing is now up, switch TRIAC ON, and set
// the timer start value to overflow once the pulse to triac has been sent
ISR(TIMER1_COMPA_vect) {
  TCCR1 = 0x00;             //Stop Timer & Reset it
  TCCR1 |= (1 << CTC1);
  TCCR1 |= (1 << CS13) | (1 << CS11);
  digitalWrite(GATE, HIGH); //set TRIAC gate to high (ON)
  TCNT1 = 256 - PULSE;      //set timer to trigger overflow after TRIAC pulse width time
}

// Pluse has bee sent, now switch off the gate of the
// TRIAC, which will switch off emmitter at next zerocrossing.
ISR(TIMER1_OVF_vect) {
  TCCR1 = 0x00;
  digitalWrite(GATE, LOW);
}

// Convert brightness level to an amount of time in ticks that the TRIAC
// remains OFF after zerocrossing is detected
// 0  = off
// 100  = max brightness (ON)
int bri2dim(uint8_t brightness)
{
  float scale = (MAX - MIN) / 101;
  return MAX - (uint8_t)(scale * brightness);
}

// Receive an i2c instruction from Master. An instruction is just a uint8_t
// The byte indicates the brightness on a scale from 0 thru 100,
// with 100 being completly ON, and 0 being completly OFF
void setBrightnessLevel(uint8_t byte_count)
{
  if (!Wire.available()) return;

  // If we have more than one byte we dont know what to do
  // so ingore for now. It should be corrected by the calling system
  // if the current brightness dosent match the required brightness
  int i = 0;
  if (byte_count > 1)
  {
    while (i < byte_count)
    {
      Wire.read();
      i++;
    }
  }

  // expect input values 0 thru 100
  brightness = Wire.read();

  // Clamp to 100 as max
  if(brightness>100)
  {
    brightness=100;
  }

}


// Return the current brightness via i2c
void sendBrightnessResponse()
{
  Wire.write(brightness);
}

void setup() {

  // Setup i2c
  Wire.begin(I2C_SLAVE_ADDRESS);  // join i2c bus

  // Recieve a command from the master (set input brightness)
  Wire.onReceive(setBrightnessLevel);

  // Service a request from the master (to get current brightness)
  Wire.onRequest(sendBrightnessResponse);

  pinMode(DETECT, INPUT_PULLUP);  //Zerocrossing Input
  pinMode(GATE, OUTPUT);          //TRIAC gate control

  //Enable Timer1 comparator & overflow functions
  TIMSK |= (1 << OCIE1A) | (1 << TOIE1);

  // Attach the interrupt handler to the pin that detects Zerocrossing
  enableInterrupt(DETECT, zeroCrossingInterrupt, RISING);

}

void loop() {
  delay(50);
}
