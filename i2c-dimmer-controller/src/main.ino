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
//   -- giving 152.5 ticks beetween Zerocrossing

#define I2C_SLAVE_ADDRESS 0x4

#define DETECT PB4  //zero cross detect pin
#define GATE PB3    //TRIAC gate pin

#define PULSE 2  //TRIAC trigger pulse width (in timer counts)

#define MIN   40  //Min phase cut delay before keeping the traic switched ON
#define MAX  150  //Max phase cut delay before keeping the triac switched OFF

// Dim level is the amount of time in ticks that the TRIAC
// remains OFF after zercross is detected
int dim_level = MAX; // default dim level to max (OFF)

// Brightness is on the scale from 0 thru 100
// 0 = COMPLETLY OFF, 100 = COMPLETLY ON
uint8_t brightness;

//zero cross detect interrupt hander
void zeroCrossingInterrupt() {

  //Should we just leave the triac competely ON or OFF
  if (dim_level <= MIN)
  {
    digitalWrite(GATE, HIGH); //Leave On
  }
  else if (dim_level >= MAX)
  {
    digitalWrite(GATE, LOW); //Leave Off
  }
  else
  {
    OCR1A = dim_level;
    TCCR1 |= (1 << CTC1);
    TCCR1 |= (1 << CS13) | (1 << CS11); // Timer1/512
  }
}

// Time OFF delay after Zerocrossing is up, switch TRIAC ON, and set up
// the timer start value to overflow once the pulse to triac has been sent
ISR(TIMER1_COMPA_vect) {
  TCCR1 = 0x00;             //Stop Timer
  TCCR1 |= (1 << CTC1);
  TCCR1 |= (1 << CS13) | (1 << CS11);  // Timer1/512  (15625 Hz based on 8MHz Clock)
  digitalWrite(GATE, HIGH); //set TRIAC gate to high
  TCNT1 = 256 - PULSE;      //trigger overflow for TRIAC pulse width
}

// Pluse has bee sent, now turn off the TRIAC
ISR(TIMER1_OVF_vect) {
  TCCR1 = 0x00;
  digitalWrite(GATE, LOW);
}

// Set brightness based on 0 thru 100
// 0  = off
// 100  = max brightness
void bri2dim()
{
  float scale = (MAX - MIN) / 100;
  dim_level = MAX - (uint8_t)(scale * brightness);
}

// Receive an i2c instruction from Master. An instruction is just a uint8_t
// The byte indicates the brightness on a scale from 0 theu 100,
// with 100 being completly off, and 0 being completly on
void setBrightness(uint8_t byte_count)
{
  //first byte is command, next 2 bytes is data for command. Total must equal 3 bytes.
  if (!Wire.available()) return;
  int i = 0;
  if (byte_count > 1)
  {
    while (i < byte_count)
    {
      Wire.read();
      i++;
    }
  }

  // input = 0 thru 100
  brightness = Wire.read();

  // Clamp to 100
  if(brightness>100)
  {
    brightness=100;
  }

  // Turn the Brightness into a dim_level
  bri2dim();
}


// Return the current brightness via i2c
void getBrightness()
{
  Wire.write(brightness);
}

void setup() {

  // Setup i2C
  Wire.begin(I2C_SLAVE_ADDRESS);  // join i2c network
  Wire.onReceive(setBrightness);  // Recieve a command from the master (set input brightness)
  Wire.onRequest(getBrightness);  // Service a request from the master (get input brightness)

  pinMode(DETECT, INPUT_PULLUP);  //Zerocrossing Input
  pinMode(GATE, OUTPUT);          //TRIAC gate control

  //Enable Timer1 comparator & overflow
  TIMSK |= (1 << OCIE1A) | (1 << TOIE1);

  // Attach the interrupt handler to the pin that detects Zerocrossing
  enableInterrupt(DETECT, zeroCrossingInterrupt, RISING);

}

void loop() {
  delay(50);
}
