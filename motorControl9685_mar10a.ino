

/*
 * I want to 
 * 1: set up timer -> done
 * 2: get the encoder value -> done
 * 3: get encoder per time -> rev/s angular velocity of the wheel -> done
 * 4: check for angular velocity every TSAMP_ENC ms time -> done
 * 5: set angular velocity target omegaTarget -> done
 *    5.1 Connect the motors properly and have them working from PCA9685 input -> done
 * 6: figure out how to set PWM output to: -> done, with PID library
 *    6.1: accelerate to omegaTarget -> done
 *    6.2: keep omegaTarget -> done
 * 
 * 7: set encoder target (clicks to destination) sClicks
 * 8: figure out how to follow sClicks
 *    8.1: goal: 60 revs in 60 seconds with a mark on the wheel ending up in the correct position
 *    8.2: add PWM, distance, PID params on the screen
 * 9: set up the above for both wheels
 * 10: manage the entire process under one function
 * 11: make turnDegrees function for turning on the spot
 * 12: (optional: learn to drive in arcs)
 * 13: learn to track coordinates
 * 14: drive in a square and end up in the same position
 * the rest is geometry
 * path planning?
 */

 /*
  * DoToo:
  * change PID parameters
  * think about using angular velocity in clicks/s instead of revs/sec, this will reduce calcs
  * implement omegaTarget properly
  * remove motorOn flag
  * 
  * 
  * 
  */

 
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#include <FastPID.h>

#include <Adafruit_GFX.h> // for display
#include <Adafruit_SSD1306.h>

#define OLED_ADDR   0x3C // i2c address for the display


// ----------------------------
// bitwise operations, macro functions that simplify reading the program a bit
#define BV(x)              (1 << x) // get the register bit
#define setBit(P, B)       (P |= BV(B))
#define clearBit(P, B)     (P &= ~BV(B))
#define toggleBit(P, B)    (P ^= BV(B))
// ----------------------------

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

Adafruit_SSD1306 display(-1); // this is fixed, but just in case of reinstall or something
#if (SSD1306_LCDHEIGHT != 64)  // check whether the settings are set up correctly
#error("Height incorrect, please fix Adafruit_SSD1306.h!");
#endif


#define PTIMEFREQ 1000 // desired timer freq, Hz
#define TSAMP_ENC 100 // how often the encoder value should be sampled, ms
#define PSERVOFREQ 60 // good freq for servo, don't forget to disable servo before switching freq
#define PMAXPULSE 4096
#define PHALFPULSE 2048 // this is half width for PWM, max is 4096

// PCA9685 pinount
#define PMOTOR_R1 5
#define PMOTOR_R2 6

#define PMOTOR_L1 7
#define PMOTOR_L2 8

#define PTIMER 3

// Test defines
#define TESTOMTARGET 490*5/10 // 490 (ticks/s) -> 1 rev/0.1s


volatile uint32_t timer_INT = 0; // timer as per interrupt
volatile uint32_t timer_current = 0;
volatile uint32_t timer_prev = 0;


// motor encoders & speed control
volatile uint16_t encR = 0; // if these are connected in opposite, simply switch them in ISR
volatile uint16_t encL = 0;
volatile uint16_t encL_prev = 0;
volatile uint16_t encR_prev = 0;
volatile uint16_t encR_current = 0;
volatile uint16_t encL_current = 0;

volatile uint16_t omegaTargetR = TESTOMTARGET;
volatile uint16_t omegaTargetL = 0;

// PID parameters for motor control****************************
float Kp=30, Ki=0, Kd=0, Hz=10;
float Ki_prev=0, Kd_prev=0; // Kp_prev=15,
int output_bits = 13; // output range is [0, 2^(bits-1)-1]; 4095 = 2^(13-1) -1
bool output_signed = false;


// ************************************************************

volatile bool motorOn = false; // toDo: remove this, it's only for testing

FastPID R_PID(Kp, Ki, Kd, Hz, output_bits, output_signed);


// motor encoder (& IR sensor interrupts)
volatile uint8_t pcint_prevState = 0;   // byte of states
volatile uint8_t pcint_current = 0;
volatile uint8_t pcint_changes = 0;

int16_t pid_R1_out = 0;




void disablePpin(uint8_t pin){ // function to disable a pin on the PCA9685 board
  pwm.setPWM(pin, 0, PMAXPULSE);
}



void setup() {
  // put your setup code here, to run once:
  
  Wire.begin();
  
  setBit(DDRB, PB5);
  toggleBit(PORTB, PB5); // on 1
  _delay_ms(1000);

  cli();

  EIMSK |= (1 << INT0); /* enable INT0 and INT1*/
  EICRA |= (1 << ISC01); 

//  setBit(EICRA, ISC11);
//  setBit(EIMSK, INT1);

  toggleBit(PORTB, PB5); // off 1

  setBit(PCICR, PCIE0); 
  setBit(PCMSK0, PCINT0); 
  setBit(PCMSK0, PCINT1); 
//  setBit(PCMSK0, PCINT2); 
//  setBit(PCMSK0, PCINT3); 
//  setBit(PCMSK0, PCINT4); 
//  setBit(PCMSK0, PCINT5); 

  setBit(PCIFR, PCIF0);

  setBit(PORTB, PB0);
  setBit(PORTB, PB1);

  _delay_ms(1000);

  toggleBit(PORTB, PB5); // on 2


  sei();


  display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
  display.clearDisplay();
  display.display();
  
  pwm.begin();
  pwm.setPWMFreq(PTIMEFREQ);

  display.setTextSize(1);
  display.setTextColor(WHITE, BLACK);
  display.setCursor(0,10);
  display.print("ready");
  display.display();

  _delay_ms(1000);

  display.clearDisplay();
  display.display();
  
  clearBit(PORTB, PB5); // off 2
  clearBit(DDRB, PB5); // set pin as input

  

  _delay_ms(1000);

  pwm.setPWM(PTIMER, 0, PHALFPULSE); // start the timer trigger pin
  
  R_PID.setOutputRange(0, 4095);

}

void loop() {
  // put your main code here, to run repeatedly:

  timer_current = timer_INT;

  if ((timer_current >= 2000)){ //  && (timer_current < 12000) testing time constraint toDo: remove
    motorOn = true;
  } else
  {
    motorOn = false;
  }
  

  if((timer_current - timer_prev) >= TSAMP_ENC){ // non-blocking code, only executes if 100+ ms passed
    encR_current = encR;
    encL_current = encL;

    omegaTargetR = map(analogRead(A0), 0, 1023, 0, TESTOMTARGET);
    Ki = map(analogRead(A1), 0, 1023, 0, 5);
    Kd = map(analogRead(A2), 0, 1023, 0, 5);
    Ki = Ki / 10.0;
    Kd = Kd / 10.0;


    if((Ki != Ki_prev) || (Kd != Kd_prev)){ // reset PID
      R_PID.clear();
      R_PID.setCoefficients(Kp, Ki, Kd, Hz); 
      Ki_prev = Ki; Kd_prev = Kd;
    }

    int16_t speedR = (encR_current - encR_prev); // divided by 0.1 s, or times 10, 
                              // since this is supposed to run every 0.1s
    int16_t speedL = (encL_current - encL_prev); // toDo: figure out how many interrupts per revolution
    // i think my current motor has reduction ratio of 20.4, and there are 12 clicks on the shaft, ~490 click per rev
    // if i make it go for 10-100 loops, it should end up fairly close

    // I've decided to use clicks/s to keep it integer and not float
        
    
    
    if(motorOn){ // run PID // toDo: change the way this is ran, so it doesn't disable motor if it's already disabled
      
      pid_R1_out = R_PID.step(omegaTargetR, speedR);
      if(pid_R1_out>(int16_t)4095) pid_R1_out = (int16_t)4095;else if(pid_R1_out<0) pid_R1_out = 0;
      pwm.setPWM(PMOTOR_R1, 0, pid_R1_out); // toDo: change pid_R1_out to a better name


    } else {
      disablePpin(PMOTOR_R1);
      omegaTargetR = 0;
    }



    encR_prev = encR_current;
    encL_prev = encL_current;
    timer_prev = timer_current;
    
    // display.clearDisplay();
    // display.display();

    display.setCursor(0,10);
    display.print(timer_prev);
    display.setCursor(0,20);
    display.print(omegaTargetR);
    display.setCursor(0,30); // speed R, clicks/s, pwm
    display.print("vR:");display.print(speedR); display.print("   pwm:"); display.print(pid_R1_out);display.print("   ");
    display.setCursor(0,40);
    display.print("enc:"); display.print(encR_current);
    display.setCursor(0,50);
    display.print("Kp:");display.print(Kp);display.print("  Ki:");display.print(Ki);
                  display.print("  Kd:");display.print(Kd);display.print("    ");

    // display.print(speedL); // speed L, clicks/s
    display.display();

    
  }
  
  


  
}




// interrupt routines
ISR(INT0_vect) { 
  timer_INT+=1;
  
}



//
ISR(PCINT0_vect){
  // if this bit is not at its previous state, it has changed
  // 00, 01, 10, 11 -> 4 states, i know that they have to have changed to trigger interrupt
  // 0b000000_00 -> 
  // 0b000000_01
  // XOR will return only changed bit

  // toDo: figure out IR sensors
  pcint_current = PINB; // read the pin state
  pcint_changes = pcint_current ^ pcint_prevState;
  if(pcint_changes & 0b00000001){
    encL++;
  } else if(pcint_changes & 0b00000010) {
    encR++;
  } else if(pcint_changes & 0b00000011){ // hopefully the programs runs fast enough for this to not be triggered
    encL++;
    encR++;
  }
  pcint_prevState = pcint_current;  
  

}


