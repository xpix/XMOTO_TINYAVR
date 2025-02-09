/* 

i have made this code for the LMD18245 motor controller, 
i have merged the pid code of  Josh Kopel 
whith the code of makerbot servo-controller board,
you can use this code on the some board changing some values.
Daniele Poddighe

https://github.com/sebleedelisle/freeservo/tree/master/ServoStrap-master

Changes from Frank Herrmann for XMOTO and XMOTOD20

*/ 

#include <PID_v2.h>
#include <EEPROM.h>

#if defined(MILLIS_USE_TIMERA0)||defined(__AVR_ATtinyxy2__)
  #error "This sketch takes over TCA0, don't use for millis here.  Pin mappings on 8-pin parts are different"
#endif

// Type of Motordriver as is, with just an PWM Interface = true
// DRV8803
#define PWMCONTROL        1
#define PWMMIN           90 // minimum value for PWM to turn the motor
#define PWMMAX          200 // maximum value for PWM

#define EEPIDADDR         1 // Adress for saved PID Values

// Connect to Hall Sensor PCB
#define encoder0PinA  	2 // PA6
#define encoder0PinB  	3 // PA7

// If u need an revert, please change here the pins to motorcontroller
#if PWMCONTROL
  #define MotorIN1        1 // PA4
  #define MotorIN2        0 // PA5
#else
  #define MotorDIR    	  0 // PA4
  #define MotorPWM 		    1 // PA5
#endif

//from ramps 1.4 stepper driver
#define STEP_PIN	      12 // PC2
#define DIR_PIN         11 // PC1
#define EnableLED       15 // PB6


volatile long encoder0Pos = 0;

long GetSteps = 0;

// PID default values 
struct MyObject {
  double kp;
  double ki;
  double kd;
};
MyObject PIDvals = {
  1.35, 0.00, 0.01 // defaut values for PID
};

double input = 0, output = 0, setpoint = 0, error = 0;
PID myPID(&input, &output, &setpoint, PIDvals.kp, PIDvals.ki, PIDvals.kd, DIRECT);  

long LastInterval = 0;
long previousMillis = 0;        // will store last time LED was updated
bool dir     = false;

// ---------------------- SETUP -----------------------
void setup() { 
	Serial.begin(9600);

  pinMode(EnableLED, OUTPUT);
  digitalWrite (EnableLED, HIGH);

  // get Saved (EEPROM) or default PID Values
  MyObject PIDv;
  EEPROM.get(EEPIDADDR, PIDv);
  if(not (PIDv.kp > 0.01)){
     PIDv = {
        PIDvals.kp, PIDvals.ki, PIDvals.kd // defaut values
     };
  }
  myPID.SetTunings(PIDv.kp, PIDv.ki, PIDv.kd);
  Serial.println("Saved (EEPROM) or default PID Values:");
  Serial.println(PIDv.kp);
  Serial.println(PIDv.ki);
  Serial.println(PIDv.kd);

  myPID.SetMode(AUTOMATIC);     //set PID in Auto mode
  myPID.SetSampleTime(2);     // refresh rate of PID controller
  myPID.SetOutputLimits(-255, 255); // this is the MAX PWM value to move motor

	// Set Pins 
	pinMode(encoder0PinA, INPUT); 
	pinMode(encoder0PinB, INPUT);  

#if PWMCONTROL
  pinMode(MotorIN1, OUTPUT); 
  pinMode(MotorIN2, OUTPUT);
#else
	pinMode(MotorDIR, OUTPUT); 
	pinMode(MotorPWM, OUTPUT);
#endif


	//motor control Pins
	pinMode(STEP_PIN, INPUT_PULLUP);
	pinMode(DIR_PIN, INPUT);

	// Interupts for position and target steps 
	attachInterrupt(digitalPinToInterrupt(encoder0PinA), doEncoderMotor0, CHANGE);  // encoderA pin on interrupt
	attachInterrupt(digitalPinToInterrupt(STEP_PIN), 	   countStep, 	    RISING);  // interrupt to count steppules

	Serial.println("start");
	Serial.println("Position ToPosition MotorPWM RealPWM");

  delay(2000);
  digitalWrite (EnableLED, LOW);
} 

// ---------------------- LOOP -----------------------
void loop(){

  byte n = Serial.available();
  if (n != 0) {
    MyObject PIDv;
    PIDv.kp = Serial.parseFloat();
    PIDv.ki = Serial.parseFloat();
    PIDv.kd = Serial.parseFloat();

    // write only PID values in eeprom if not rubbish
    if( PIDv.kp > 0.00 && PIDv.ki > 0.00 && PIDv.kd > 0.00){
      MyObject PIDvals = {
        PIDv.kp, PIDv.ki, PIDv.kd // defaut values
      };
      Serial.println("Input PID values saved in eeprom:");
      EEPROM.put(EEPIDADDR, PIDvals);
  
      myPID.SetTunings(PIDv.kp, PIDv.ki, PIDv.kd);
    }

    // control PID values    
    Serial.println(PIDv.kp);
    Serial.println(PIDv.ki);
    Serial.println(PIDv.kd);
  }

	setpoint = GetSteps; // setpoint to Steps get from controller
	input = encoder0Pos; // real Position
  error = abs(setpoint - input);
  myPID.Compute();   // compute correct value
  int realpwm=0;
  if(error){
    realpwm = pwmOut(output);    // get PWM Value from PID calculated
  }
  else {
    realpwm = pwmOut(0);    // break
  }

  // Diganostic via serialplotter from arduino ide
  // Serial.println("Position ToPosition MotorPWM DIR");
  if(millis() - LastInterval > (abs(error) > 10 ? 100 : 1000)){

    if(output < PWMMIN){
      digitalWrite (EnableLED, HIGH); // signal that mcu is running
    }

    Serial.print(input);
    Serial.print(" ");
    Serial.print(setpoint);
    Serial.print(" ");
    Serial.print(output);
    Serial.print(" ");
    Serial.print(realpwm);
    Serial.println("");

    if(output < PWMMIN){
      digitalWrite (EnableLED, LOW);
    }
    LastInterval=millis();
  }
}

// ---------------------- SUBS -----------------------
#if PWMCONTROL
int pwmOut(int out) {
  int pwm = 0;
  if (out > 0) {
    // forward
    pwm = map(out, 0, 255, PWMMIN, PWMMAX );
    analogWrite ( MotorIN1, pwm );       
    digitalWrite ( MotorIN2, LOW );
  }
  else if (out < 0) {
    // reverse
    pwm = map(abs(out), 0, 255, PWMMIN, PWMMAX );
    digitalWrite ( MotorIN1, LOW );    // if REV < encoderValue motor move in reverse direction.   
    analogWrite ( MotorIN2, pwm );                      
  }
  else {
    // break
    digitalWrite ( MotorIN1, LOW );
    digitalWrite ( MotorIN2, LOW );
  }
  return pwm;
}
#else
void pwmOut(int out) {                               
	if (out > 0) {
		analogWrite( MotorPWM, out );      	
		digitalWrite ( MotorDIR ,LOW );
	}
	else {
		analogWrite( MotorPWM, abs(out) );                      
		digitalWrite ( MotorDIR, HIGH );		// if REV < encoderValue motor move in reverse direction.   
  }
}
#endif

void doEncoderMotor0(){
	if (digitalRead(encoder0PinA) == HIGH) {   // found a low-to-high on channel A
		if (digitalRead(encoder0PinB) == LOW) {  // check channel B to see which way
												 // encoder is turning
			encoder0Pos = encoder0Pos - 1;         // CCW
		} 
		else {
			encoder0Pos = encoder0Pos + 1;         // CW
		}
	}
	else                                        // found a high-to-low on channel A
	{ 
		if (digitalRead(encoder0PinB) == LOW) {   // check channel B to see which way
												  // encoder is turning  
			encoder0Pos = encoder0Pos + 1;          // CW
		} 
		else {
			encoder0Pos = encoder0Pos - 1;          // CCW
		}
	}
}

void countStep(){
	dir = digitalRead(DIR_PIN);
	if(dir == HIGH){
	  GetSteps++;
	}
	else{
	  GetSteps--;
	}
}
