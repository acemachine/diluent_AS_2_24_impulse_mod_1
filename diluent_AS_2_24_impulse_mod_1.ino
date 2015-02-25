#include <TimerOne.h>
#include <Wire.h>
#include <Time.h>
#include <Adafruit_PWMServoDriver.h>


#define MOTOR_0        204
#define MOTOR_100      408
#define MOTOR_50       ((MOTOR_0+MOTOR_100)/2)
#define MOTOR_25       ((MOTOR_0+MOTOR_50)/2)
#define MOTOR_12       ((MOTOR_0+MOTOR_25)/2)



#define LED_1G    39
#define LED_2A    41
#define LED_3G    43
#define LED_4R    45
#define LED_5R    47
#define LED_6R    49
#define LED_ON(led)    digitalWrite(led, LOW)
#define LED_OFF(led)   digitalWrite(led, HIGH)
#define LED_ALLON()    LED_ON(LED_1G); LED_ON(LED_2A); LED_ON(LED_3G); LED_ON(LED_4R); LED_ON(LED_5R); LED_ON(LED_6R)
#define LED_ALLOFF()   LED_OFF(LED_1G); LED_OFF(LED_2A); LED_OFF(LED_3G); LED_OFF(LED_4R); LED_OFF(LED_5R); LED_OFF(LED_6R)
#define LED_ERROR(led) LED_OFF(LED_1G); LED_OFF(LED_2A); LED_OFF(LED_3G); LED_ON(led)

#define GREEN_TOGGLE         30

#define KNOB1         A11
#define KNOB2         A10
#define KNOB3         A9
#define KNOB4         A8


#define BELT_WAIT               A8
#define BELT_STEP               2
#define BELT_DIRECTION          5
#define BELT_ENCODER            A13

volatile double next_beltpos = 0;
volatile unsigned long belt_watchdog = 0;
volatile int pumpDivCount = 0;
int pumpSpeedDividor =5;


#define STATION_SEAL_MOTOR      14
#define STATION_SEAL_SPEED      A9
#define STATION_SEAL_HALL       28
#define STATION_SEAL_HEAT       25
#define STATION_SEAL_TEMP       A12
#define STATION_SEAL_TEMP_SET   A10
#define STATION_SEAL_TEMP_ERROR() (analogRead(STATION_SEAL_TEMP_SET) - analogRead(STATION_SEAL_TEMP))

volatile enum {
  SEAL_TURN_1,
  SEAL_SLOW_SPEED,
  SEAL_TURN_2,
  SEAL_DONE,
  SEAL_OFF,
} seal_state = SEAL_OFF;

volatile unsigned seal_temperature = 0;


#define STATION_FILL_SERVO      13
#define STATION_FILL_SWITCH     27
#define STATION_FILL_UP         235
#define STATION_FILL_CHECK      /*222*/ 196
#define STATION_FILL_DOWN       135
#define STATION_FILL_STEP       3
#define STATION_FILL_DIRECTION  6
#define STATION_FILL_STEPS      1025
#define STATION_FILL_SENSOR     A14

volatile enum {
  FILL_RESET,
  FILL_CHECK_1,
  FILL_CHECK_2,
  FILL_PUMP,
  FILL_SLOW_UP,
  FILL_DONE,
  FILL_OFF,
} fill_state = FILL_OFF;

volatile int fill_stepsleft = 0;


#define STATION_CUT_MOTOR     15
#define STATION_CUT_HALL      24

volatile enum {
  CUT_TURN_1,
  CUT_TURN_2,
  CUT_DONE,
  CUT_OFF,
} cut_state = CUT_OFF;



Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();


volatile enum {
  INIT,
  WAIT,
  MOVE_BELT,
  STATIONS,
  BELT_ERROR,
  FILL_ERROR,
} global_state = INIT;

#define IS_ERROR()  (global_state == BELT_ERROR || global_state == FILL_ERROR)


int debounceRead(int pin) {
  unsigned value = digitalRead(pin);
  unsigned last_debounce = millis();
  
  while ((millis() - last_debounce) < 50) {
    delay(1);
    int newvalue = digitalRead(pin);
    if (newvalue != value) {
      value = newvalue;
      last_debounce = millis();
    }
  }
  
  return value;
}

void setup()
{
  //Serial.begin(9600);
  
  // Initialize the user interface
  {
    // Set LEDs to out (active low)
    pinMode(LED_1G, OUTPUT);
    pinMode(LED_2A, OUTPUT);
    pinMode(LED_3G, OUTPUT);
    pinMode(LED_4R, OUTPUT);
    pinMode(LED_5R, OUTPUT);
    pinMode(LED_6R, OUTPUT);
    
    // Set up big green button
    pinMode(GREEN_TOGGLE, INPUT_PULLUP);
  }
  
  LED_ALLON();
  
  // Init I2C PWM board to 50Hz
  {
    pwm.begin();
    pwm.setPWMFreq(50);
    
    // Set PWM to OFF, wait a bit for register
    pwm.setPWM(STATION_CUT_MOTOR, 0, MOTOR_0);
    pwm.setPWM(STATION_FILL_SERVO, 0, STATION_FILL_UP);
    pwm.setPWM(STATION_SEAL_MOTOR, 0, MOTOR_0);
    delay(1000);
  }
  
  // Initialize the cutter
  {
    pinMode(STATION_CUT_HALL, INPUT_PULLUP);
    
    // Home cutter motor
    pwm.setPWM(STATION_CUT_MOTOR, 0, MOTOR_50);
    while (digitalRead(STATION_CUT_HALL) == 0);
    while (digitalRead(STATION_CUT_HALL) == 1);
    pwm.setPWM(STATION_CUT_MOTOR, 0, MOTOR_0);
  }
  
  // Initialize the filler
  {
    // Pump stepper
    pinMode(STATION_FILL_STEP, OUTPUT);
    pinMode(STATION_FILL_DIRECTION, OUTPUT);
    digitalWrite(STATION_FILL_DIRECTION, 0);
    
    // Fill check switch
    pinMode(STATION_FILL_SWITCH, INPUT_PULLUP);

    // Put pump servo in upward position
    pwm.setPWM(STATION_FILL_SERVO, 0, STATION_FILL_UP);
    delay(250);
  }
  
  // Initialize sealer
  {
    // Configure heater
    pinMode(STATION_SEAL_HALL, INPUT_PULLUP);
    pinMode(STATION_SEAL_HEAT, OUTPUT);
    //digitalWrite(STATION_SEAL_HEAT, 1);
    //seal_temperature = analogRead(STATION_SEAL_TEMP);
    
    // Home sealer motor
    pwm.setPWM(STATION_SEAL_MOTOR, 0, MOTOR_50);
    while (digitalRead(STATION_SEAL_HALL) == 0);
    while (digitalRead(STATION_SEAL_HALL) == 1);
    pwm.setPWM(STATION_SEAL_MOTOR, 0, MOTOR_0);
  }
  
  // Initialize belt
  {
    // Belt stepper
    pinMode(BELT_STEP, OUTPUT);
    pinMode(BELT_DIRECTION, OUTPUT);
    digitalWrite(BELT_DIRECTION, 0);
    
    // Get encoder value, set belt home position
    //next_beltpos = 697.0;
    next_beltpos = analogRead(BELT_ENCODER);
    next_beltpos += 102.4 - fmod(next_beltpos, 102.4);
  }
  
  // Initialize CNC shield
  {
    // Set CNC shield ENable (active low)
    pinMode(8, OUTPUT);
    digitalWrite(8, 0);
  }

  // Configure stepper timer interrupt
  Timer1.initialize(300);
  Timer1.attachInterrupt(timerint);
  
  // Wait for temperature
  LED_ALLOFF(); LED_ON(LED_2A);
  while ((analogRead(STATION_SEAL_TEMP_SET) - analogRead(STATION_SEAL_TEMP)) > 0) {
    delay(100);
  }
  
  // Wait for green button to depress
  LED_ALLOFF(); LED_ON(LED_3G);
  while (debounceRead(GREEN_TOGGLE) == 0) {
    delay(100);
  }
  
  //global_state = WAIT;
}

void loop()
{
  // Check for errors
  while (IS_ERROR() && debounceRead(GREEN_TOGGLE) == 0) {
    delay(100);
  }
  LED_OFF(LED_4R); LED_OFF(LED_5R); LED_OFF(LED_6R);

handle_pause:

  global_state = WAIT;

  // Handle pause
  while (debounceRead(GREEN_TOGGLE) == 1) {
    LED_OFF(LED_1G); LED_ON(LED_3G);
    delay(100);
  }
  LED_OFF(LED_3G);
  
  // Handle heater
  LED_OFF(LED_1G);
  if (!IS_ERROR() && STATION_SEAL_TEMP_ERROR() > 20) {
    while (!IS_ERROR() && STATION_SEAL_TEMP_ERROR() > 0) {
      delay(100);
    }
    
    LED_ON(LED_3G);
    while (debounceRead(GREEN_TOGGLE) == 0) {
      delay(100);
    }
    
    goto handle_pause;
  }
  LED_ON(LED_1G);
  
  if (global_state == WAIT) {
    double old_beltpos = next_beltpos;
    
    // Advance belt to next position
    next_beltpos = fmod(next_beltpos + 102.4, 1024);
    belt_watchdog = millis();
    global_state = MOVE_BELT;
    
    // Wait for state change
    while (global_state == MOVE_BELT) {
      delay(10);
    }
    
    if (IS_ERROR()) {
      // Revert old belt position on error
      next_beltpos = old_beltpos;
    }
  }
  
  if (global_state == STATIONS) {
    // Cut station
    pwm.setPWM(STATION_CUT_MOTOR, 0, MOTOR_100);
    cut_state = CUT_TURN_1;
    
    // Fill station
    unsigned fill_start = millis();
    unsigned fill_tries = 0;
    unsigned fill_servopos = 0;
    double fill_sensorval = 0;
    pwm.setPWM(STATION_FILL_SERVO, 0, STATION_FILL_CHECK);
    fill_state = FILL_CHECK_1;
    
    // Seal station
   pwm.setPWM(STATION_SEAL_MOTOR, 0, MOTOR_50);
   seal_state = SEAL_TURN_1;
    
    // Wait for state change
    while ((cut_state != CUT_OFF || fill_state != FILL_OFF || seal_state != SEAL_OFF)) {
      if (cut_state == CUT_DONE) {
        pwm.setPWM(STATION_CUT_MOTOR, 0, MOTOR_0);
        cut_state = CUT_OFF;
      }
      
      unsigned now = millis();
      #define FILL_TIME()            (now - fill_start)
      #define FILL_SENSE()           (fill_sensorval = analogRead(STATION_FILL_SENSOR))
      #define FILL_TRANS(state) { fill_start = millis(); fill_state = state; }
      if (fill_state == FILL_RESET && FILL_TIME() > 500) {
        
        // 3 strikes and you're out
        if (fill_tries >= 2) {
          fill_state = FILL_OFF;
          global_state = FILL_ERROR;
          LED_ERROR(LED_5R);
        } else {
          pwm.setPWM(STATION_FILL_SERVO, 0, STATION_FILL_CHECK);
          FILL_TRANS(FILL_CHECK_1);
        }
      } else if (fill_state == FILL_CHECK_1/* && FILL_TIME() > 500*/) {
        
        //pwm.setPWM(STATION_FILL_SERVO, 0, STATION_FILL_DOWN);
        fill_servopos = STATION_FILL_CHECK;
        FILL_TRANS(FILL_CHECK_2);
        
        //delay(100); FILL_SENSE();
        //delay(100); FILL_SENSE();
        //delay(100); FILL_SENSE();
        //delay(100); FILL_SENSE();
        //delay(100); FILL_SENSE();
        //delay(100); FILL_SENSE();
        
      } else if (fill_state == FILL_CHECK_2) {
        //fill torque//
        if (fill_servopos > STATION_FILL_DOWN) {
          if (FILL_SENSE() < 400 && fill_servopos > 160 && fill_servopos < 200) {
            // Fill switch contacted something, bad
            fill_tries += 1;
            pwm.setPWM(STATION_FILL_SERVO, 0, STATION_FILL_UP);
            FILL_TRANS(FILL_RESET);
          } else {
            fill_servopos -= 1;
            pwm.setPWM(STATION_FILL_SERVO, 0, fill_servopos);
            delay(17);
          }
        
        } else {   
          if (digitalRead(STATION_FILL_SWITCH) == 1) {
            // Fill switch contacted nothing, empty
            FILL_TRANS(FILL_DONE);
          } else {
            // Fill switch engaged
            fill_servopos = STATION_FILL_DOWN;
            fill_stepsleft = STATION_FILL_STEPS;
            FILL_TRANS(FILL_PUMP);
          }
        }
      } else if (fill_state == FILL_SLOW_UP) {
        
        if(fill_servopos < STATION_FILL_UP){
          fill_servopos += 1;
          pwm.setPWM(STATION_FILL_SERVO, 0, fill_servopos);
          delay(13);
          
        } else {
          FILL_TRANS(FILL_OFF);
        }
        
      } else if (fill_state == FILL_DONE) {
        pwm.setPWM(STATION_FILL_SERVO, 0, STATION_FILL_UP);
        fill_state = FILL_OFF;
        
      }
      
      if (seal_state == SEAL_SLOW_SPEED) {
        pwm.setPWM(STATION_SEAL_MOTOR, 0, map(analogRead(STATION_SEAL_SPEED), 0, 1023, map(10, 0, 100, MOTOR_0, MOTOR_100), map(25, 0, 100, MOTOR_0, MOTOR_100)));
        seal_state = SEAL_TURN_2;
      } else if (seal_state == SEAL_DONE) {
        pwm.setPWM(STATION_SEAL_MOTOR, 0, MOTOR_0);
        seal_state = SEAL_OFF;
      }
      
      delay(1);
    }
    
    // Reset all pwms (in case error happened)
    pwm.setPWM(STATION_CUT_MOTOR, 0, MOTOR_0);
    pwm.setPWM(STATION_FILL_SERVO, 0, STATION_FILL_UP);
    pwm.setPWM(STATION_SEAL_MOTOR, 0, MOTOR_0);
    cut_state = CUT_OFF; fill_state = FILL_OFF; seal_state = SEAL_OFF;
  }
  
  // Handle knobbed pause
  delay(map(analogRead(BELT_WAIT), 0, 1023, 0, 5000));
}

volatile unsigned count = 0;
void timerint()
{
  if (global_state == MOVE_BELT || global_state == INIT) {
    if(fabs((double)analogRead(BELT_ENCODER) - next_beltpos) > 10.0) {
      digitalWrite(BELT_STEP, digitalRead(BELT_STEP) ^ 1);
    } else if (global_state == MOVE_BELT) {
      // Next step
      global_state = STATIONS;
    }
  }
  
  if (global_state == WAIT && digitalRead(GREEN_TOGGLE) == 1 && analogRead(KNOB1) > 512 && analogRead(KNOB1) < 1000) {
    //->
    if ( pumpDivCount >= pumpSpeedDividor) {
        digitalWrite(STATION_FILL_STEP, digitalRead(STATION_FILL_STEP) ^ 1);
        pumpDivCount = 0;
      } 
      else {
        pumpDivCount++;
      }
    //<- digitalWrite(STATION_FILL_STEP, digitalRead(STATION_FILL_STEP) ^ 1);
  }
  
  /*if (global_state == STATIONS)*/ {
    // Check cutter station
    if (cut_state == CUT_TURN_1 || cut_state == CUT_TURN_2) {
      unsigned hall = digitalRead(STATION_CUT_HALL);
      if (cut_state == CUT_TURN_1 && hall == 1) {
        cut_state = CUT_TURN_2;
      } else if (cut_state == CUT_TURN_2 && hall == 0) {
        cut_state = CUT_DONE;
      }
    }
    
    // Check fill station
    if (fill_state == FILL_PUMP) {
      if (fill_stepsleft > 0 && pumpDivCount >= pumpSpeedDividor) {
        digitalWrite(STATION_FILL_STEP, digitalRead(STATION_FILL_STEP) ^ 1);
        fill_stepsleft -= 1;
        pumpDivCount = 0;
      } 
      else if (fill_stepsleft > 0) {
        pumpDivCount++;
      }
      else {
        fill_state = FILL_SLOW_UP;
      }
    }
    
    // Check seal station
   if (seal_state == SEAL_TURN_1 || seal_state == SEAL_TURN_2) {
      unsigned hall = digitalRead(STATION_SEAL_HALL);
      if (seal_state == SEAL_TURN_1 && hall == 1) {
        seal_state = SEAL_SLOW_SPEED;
      } else if (seal_state == SEAL_TURN_2 && hall == 0) {
        seal_state = SEAL_DONE;
      }
    } 
  }
  
  if ((count % 1000) == 0) {
    int err = STATION_SEAL_TEMP_ERROR();
    
    if(!IS_ERROR() && err > 0) {
      digitalWrite(STATION_SEAL_HEAT, 0);
    } else{
      digitalWrite(STATION_SEAL_HEAT, 1);
    }
    
    if (!IS_ERROR() && err > 20) {
      LED_ON(LED_2A);
    } else {
      LED_OFF(LED_2A);
    }
  }
  
  // Check watchdogs
  if (global_state == MOVE_BELT && (millis() - belt_watchdog) > 1000) {
    global_state = BELT_ERROR;
    LED_ERROR(LED_4R);
  }
  
  count += 1;
}

