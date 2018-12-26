// Basic sketch for A4988 stepper motor driver
// This sketch uses interrupts to step the motor

// MIT licence applies
// Created by Bodmer 20/4/2016

// Microsteps per step   :     1      8     16   (Set with A4988 mode pins)
// RPM at 5kHz interrupt :   1500    187    93   (for a motor with 200 steps per rev)
// Approx. minimum RPM   :     74     10     5   (for a motor with 200 steps per rev)

// Use timer prescaler to allow lower speeds, e.g. divide clock by 8
// Use fewer microsteps per step to raise speeds

// Motor settings
#define MICROSTEPS_PER_STEP  16  // A4988 can be set to 1, 8 or 16
#define STEPS_PER_REV       200  // Test motor has 200 steps per rev
#define MAX_RPM             100  // RPM limit
#define AUTO_RAMP_DOWN           // Auto ramp down of speed at end of a movement

// Timer settings
#define CPU_MHZ        16000000L // 16 MHz oscillator
#define PRESCALER           1    // Timer 1 prescaler (no prescaler = 1, 8 for slower speeds)
#define TIMER_1_MIN         1000 // Stop speed for end of movement ramp down
#define TIMER_1_MAX         0xFFFF - ( CPU_MHZ / PRESCALER / (STEPS_PER_REV * MICROSTEPS_PER_STEP * MAX_RPM / 60L) )

#define STEP_ENA 7               // A4988 enable pin
#define STEP_CLK 8               // A4988 clock pin
#define STEP_DIR 9               // A4988 direction pin

unsigned int timer1_counter = 1;
volatile unsigned int new_counter = 1;

volatile int  steps = 0;    // Changes outside of interrupt handler, so make it volatile
volatile bool done  = true; // Used outside interrupt handler, so make it volatile

int motorSpeed = 20;

bool step_forward = true;

void setup()
{
  Serial.begin(115200);

  pinMode(LED_BUILTIN, OUTPUT);

  pinMode(STEP_ENA, OUTPUT);
  pinMode(STEP_CLK, OUTPUT);
  pinMode(STEP_DIR, OUTPUT);

  digitalWrite(STEP_ENA, HIGH); // Disable the driver

  startTimer1();                // Start the timer 1 interrupt

  digitalWrite(STEP_ENA, LOW);  // Enable the driver
}

void loop()
{
  // Choose a random speed on each loop
  motorRPM( random(MAX_RPM) );

  // If movement is complete then change direction
  if (done)
  {
    done = false;                       // Set done = false before a new movement
    if (step_forward) stepMotor(+6400); // Positive for forward movement  400 steps * 16 microsteps
    else              stepMotor(-6400); // Negative for backward movement
    step_forward = !step_forward;
  }

  // Blink LED
  digitalWrite(LED_BUILTIN, HIGH);
  delay(500); 
  digitalWrite(LED_BUILTIN, LOW);
  delay(500);
}

/***************************************************************************************
** Function name:           stepMotor
** Description:             Step motor "us" microsteps, +ve for forward, -ve for reverse
***************************************************************************************/
void stepMotor(long us)
{
  if ( us < 0 )
  {
    digitalWrite(STEP_DIR, LOW);
    us = -us;
  }
  else digitalWrite(STEP_DIR, HIGH);

  // Updating steps variable is non-atomic so turn off interrupts before changing it
  noInterrupts();
  steps = us;
  interrupts();
}

/***************************************************************************************
** Function name:           motorRPM
** Description:             Set the motor RPM
***************************************************************************************/
void motorRPM(long rpm)
{
  if ( rpm > MAX_RPM) rpm = MAX_RPM; // Interrup rate limit may lower this again

  Serial.print("Set rpm = "); Serial.print(rpm);

  long tc = 0xFFFF - ( CPU_MHZ / PRESCALER / (STEPS_PER_REV * MICROSTEPS_PER_STEP * rpm / 60L) );

  if (tc < 1) tc = 1;                           // This is the minimum speed value for the counter

  // To keep the interrupt handler burden acceptable a rate of 5000 Hz maximum is set (this also limits the max RPM)
  if ( (PRESCALER ==  1) && tc > 62336) tc = 62336;
  if ( (PRESCALER ==  8) && tc > 65136) tc = 65136;
  if ( (PRESCALER == 64) && tc > 65486) tc = 65486;

  Serial.print(", tc = "); Serial.println(tc);

  // Updating counter variable is non-atomic so turn off interrupts before changing it
  noInterrupts();
  timer1_counter = tc;
  interrupts();
}

/***************************************************************************************
** Function name:           startTimer1
** Description:             Start the timer interrupt firing
***************************************************************************************/
/*
  CS12 CS11 CS10  Mode Description
  0    0    0    Stop Timer/Counter 1
  0    0    1    No Prescaler (Timer Clock = System Clock)
  0    1    0    divide clock by 8
  0    1    1    divide clock by 64
  1    0    0    divide clock by 256
  1    0    1    divide clock by 1024
  1    1    0    increment timer 1 on T1 (pin 5) falling edge
  1    1    1    increment timer 1 on T1 (pin 5) rising edge
*/
void startTimer1(void)
{
  // Initialise timer 1
  noInterrupts();                      // disable interrupts
  TCCR1A = 0;                          // stop timer
  TCCR1B = 0;

  timer1_counter = TIMER_1_MIN;        // preload timer 65536-16MHz/1/frequency

  TCNT1 = timer1_counter;              // preload timer

  if (PRESCALER == 1) TCCR1B |= (1 << CS10);  // No prescaler
  if (PRESCALER == 8) TCCR1B |= (1 << CS11);  // divide clock by 8

  // Other unused prescaler divisions available
  //if (PRESCALER == 64) TCCR1B |= (1 << CS10) | (1 << CS11);   // divide clock by 64
  //if (PRESCALER == 256) TCCR1B |= (1 << CS12);                // divide clock by 256
  //if (PRESCALER == 1024) TCCR1B |= (1 << CS12) | (1 << CS10); // divide clock by 1024

  TIMSK1 |= (1 << TOIE1);              // enable timer overflow interrupt
  interrupts();                        // enable interrupts
}

/***************************************************************************************
** Function name:           stopTimer1
** Description:             Stop the timer interrupt from firing
***************************************************************************************/
void stopTimer1(void)
{
  noInterrupts();           // disable all interrupts
  TIMSK1 &= !(1 << TOIE1);  // disable timer overflow interrupt
  interrupts();             // enable all interrupts
}

/***************************************************************************************
** Function name:           TIMER1_OVF_vect
** Description:             Called by interrupt when timer 1 overflows
***************************************************************************************/
ISR(TIMER1_OVF_vect)
{
  // First load the timer
  TCNT1 = timer1_counter;

  if ( steps == 0 )
  {
    done = true;
    new_counter = 1;
  }
  // If there are still motor steps left
  else
  {
    // Pulse the clock line to move on one step
    digitalWrite(STEP_CLK, HIGH);
    digitalWrite(STEP_CLK, LOW);

    // Decrease the steps remaining
    steps--;

    // The speed ramp up rate can be set by the controlling funtion

#ifdef AUTO_RAMP_DOWN
    // Motor will stop in 64 microsteps, so ramp down the speed to reduce jerk
    if (steps < 64 && timer1_counter > 1000) timer1_counter -= 1000;
#endif
  }
}
/***************************************************************************************
***************************************************************************************/
