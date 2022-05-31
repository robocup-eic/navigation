
#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Arduino.h>
#include <math.h>

//Setting the timer used for interrupts
#define USE_TIMER_1     false 
#define USE_TIMER_2     true
#define USE_TIMER_3     false
#define USE_TIMER_4     false
#define USE_TIMER_5     false

#include <TimerInterrupt.h>
#define TIMER_INTERVAL_MS 50L

//Declare PID variables
extern "C"
{
#include <PID.h>
}

#define PID_KP_LEFT 0.4f
#define PID_KI_LEFT 1.0f
#define PID_KD_LEFT 0.0f

#define PID_KP_RIGHT 0.2f
#define PID_KI_RIGHT 1.2f
#define PID_KD_RIGHT 0.01f

#define PID_TAU 0.05f

#define PID_LIM_MIN -255.0f
#define PID_LIM_MAX 255.0f

#define PID_LIM_MIN_INT -125.0f
#define PID_LIM_MAX_INT 125.0f

#define SAMPLE_TIME_S 0.05f

//Import encoder library and servo for motor control
#include <Encoder.h>

//Declare pin for encoders
#define ENCA1 2
#define ENCA2 3
#define ENCB1 18
#define ENCB2 19

//Declare pin for motors

#define PWM_L 9
#define INA_L 10
#define INB_L 11

#define PWM_R 5
#define INA_R 6
#define INB_R 7


// TimerInterrupt ITimer;

Encoder enc1(ENCA1, ENCA2), enc2(ENCB1, ENCB2);
PIDController pid_left = {PID_KP_LEFT,
                     PID_KI_LEFT, PID_KD_LEFT,
                     PID_TAU,
                     PID_LIM_MIN, PID_LIM_MAX,
                     PID_LIM_MIN_INT, PID_LIM_MAX_INT,
                     SAMPLE_TIME_S};

PIDController pid_right = {PID_KP_RIGHT,
                     PID_KI_RIGHT, PID_KD_RIGHT,
                     PID_TAU,
                     PID_LIM_MIN, PID_LIM_MAX,
                     PID_LIM_MIN_INT, PID_LIM_MAX_INT,
                     SAMPLE_TIME_S};

long posPrev[2] = {0, 0};
float setRPM = 50.0;
float velocity[2] = {0.0,0.0};
long pos[2] = {0, 0};

unsigned long currT = 0;
unsigned long prevT = 0;

void setMotor(int pwr, int digiPin1, int digiPin2, int analogPin)
{
  if(pwr<0){
    digitalWrite(digiPin1,HIGH);
    digitalWrite(digiPin2,LOW);
    analogWrite(analogPin,-pwr);
  }
  else{
    digitalWrite(digiPin1,LOW);
    digitalWrite(digiPin2,HIGH);
    analogWrite(analogPin,pwr);
  }
}

void controlMotor()
{
  // Read Encoder
  velocity[0] = (pos[0]-posPrev[0]) * 1e3 / ((float)TIMER_INTERVAL_MS);
  velocity[1] = (posPrev[1]-pos[1]) * 1e3 / ((float)TIMER_INTERVAL_MS);
  velocity[0] = (velocity[0] / 1600.0)*60.0;
  velocity[1] = (velocity[1] / 1600.0)*60.0;

  posPrev[0] = pos[0];
  posPrev[1] = pos[1];

  //Compute PID
  int pwr_left = PIDController_Update(&pid_left, setRPM, velocity[0]);
  int pwr_right = PIDController_Update(&pid_right, setRPM, velocity[1]);

  // Serial.println(millis());
  // if (millis()>8000){
  //   // Serial.println('HI');
  //   pwr_left = 0;
  //   pwr_right = 0;
  // }
  
  // setMotor(pwr_left, INA_L, INB_L, PWM_L);
  //setMotor(pwr_right, INA_R, INB_R, PWM_R);

  int verbose = 2;

  if (verbose == 2){

  Serial.println("-------------------------------------------------");
  Serial.print("SETRPM: ");
  Serial.println(setRPM);
  Serial.println("**LEFT**");
  Serial.println(static_cast<String>("1: position: ") + pos[0]);
  Serial.println(static_cast<String>("2: velocity: ") + velocity[0]);
  Serial.println(static_cast<String>("3: commanded voltage: ") + pwr_left);
  Serial.println("**RIGHT**");

  Serial.println(static_cast<String>("1: position: ") + pos[1]);
  Serial.println(static_cast<String>("2: velocity: ") + velocity[1]);
  Serial.println(static_cast<String>("3: commanded voltage: ") + pwr_right);

  // Serial.println("**PID**");

  // Serial.println(static_cast<String>("1: proportional: ") + pid_left.proportional);
  // Serial.println(static_cast<String>("2: integral: ") + pid_left.integral);
  // Serial.println(static_cast<String>("3: deriviative: ") + pid_left.differentiator);

  // Serial.println(static_cast<String>("3: propotional: ") + pid_left.proportional);
  // Serial.println(static_cast<String>("3: integral: ") + pid_left.integral);
  // Serial.println(static_cast<String>("3: derivative: ") + pid_left.differentiator);

  Serial.println("-------------------------------------------------");
  }else if(verbose == 1){
    Serial.print(velocity[0]);
    Serial.print(" , ");
    Serial.println(velocity[1]);
  }else{
    
  }

  // if(currT-prevT>5000){
  //   setRPM += 1;
  //   prevT = currT;
  // }

}

void print_debug(){
  
}

void setup()
{
  Serial.begin(115200);

  pinMode(INA_L, OUTPUT);
  pinMode(INA_R, OUTPUT);
  pinMode(INB_L, OUTPUT);
  pinMode(INB_R, OUTPUT);

  pinMode(PWM_L, OUTPUT);
  pinMode(PWM_R, OUTPUT);

  digitalWrite(INA_R, HIGH);
  digitalWrite(INB_R, LOW);
  analogWrite(PWM_R, 100);

  digitalWrite(INA_L, HIGH);
  digitalWrite(INB_L, LOW);
  analogWrite(PWM_L, 100);
 

  while (!Serial)
  {
    ; // wait for serial port to connect. Needed for native USB
  }
  //Init PID
  PIDController_Init(&pid_left);
  PIDController_Init(&pid_right);
  // Init timer ITimer1
  ITimer2.init();
  if (ITimer2.attachInterruptInterval(TIMER_INTERVAL_MS, controlMotor))
  {
    Serial.print(F("Starting  ITimer3 OK, millis() = "));
    Serial.println(millis());
  }
  else
    Serial.println(F("Can't set ITimer3. Select another freq. or timer"));
}

void loop()
{
  currT = millis();
  pos[0] = enc1.read();
  pos[1] = enc2.read();

 
  // Serial.println(M_PI);
  // setRPM = (int) Serial.read();
  // Serial.println("HELLO");
  // put your main code here, to run repeatedly:
  // Serial.println(pos);
  delay(1);
}