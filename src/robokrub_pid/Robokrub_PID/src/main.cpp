#define ENCODER_OPTIMIZE_INTERRUPTS
#include <math.h>
extern "C"
{
#include <PID.h>
}
#include <Encoder.h>
#define PID_KP_LEFT 3.0f
#define PID_KI_LEFT 2.0f
#define PID_KD_LEFT 0.0f
#define PID_KP_RIGHT 3.0f
#define PID_KI_RIGHT 2.0f
#define PID_KD_RIGHT 0.0f
#define PID_TAU 0.05f
#define PID_LIM_MIN -255.0f
#define PID_LIM_MAX 255.0f
#define PID_LIM_MIN_INT -125.0f
#define PID_LIM_MAX_INT 125.0f
#define SAMPLE_TIME_S 0.05f
//Declare pin for encoders
#define ENCA1 2
#define ENCA2 3
#define ENCB1 17
#define ENCB2 18
//Declare pin for motors
#define PWM_L 8
#define INA_L 12
#define INB_L 13
#define PWM_R 5
#define INA_R 6
#define INB_R 7
#define TICKS_PER_REV 1600
#define WHEEL_RAD 0.125
Encoder encLeft(ENCA1,ENCA2), encRight(ENCB1,ENCB2);
PIDController pidLeft = {PID_KP_LEFT,
                     PID_KI_LEFT, PID_KD_LEFT,
                     PID_TAU,
                     PID_LIM_MIN, PID_LIM_MAX,
                     PID_LIM_MIN_INT, PID_LIM_MAX_INT,
                     SAMPLE_TIME_S};
PIDController pidRight = {PID_KP_RIGHT,
                     PID_KI_RIGHT, PID_KD_RIGHT,
                     PID_TAU,
                     PID_LIM_MIN, PID_LIM_MAX,
                     PID_LIM_MIN_INT, PID_LIM_MAX_INT,
                     SAMPLE_TIME_S};
long posPrev[2] = {0, 0};
float setRPM = 25.0;
float velocity[2] = {0.0,0.0};
long pos[2] = {0, 0};
int pwr[2] = {0, 0};
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
void setup(){
  Serial.begin(115200);
  pinMode(INA_L, OUTPUT);
  pinMode(INA_R, OUTPUT);
  pinMode(INB_L, OUTPUT);
  pinMode(INB_R, OUTPUT);
  pinMode(PWM_L, OUTPUT);
  pinMode(PWM_R, OUTPUT);
  PIDController_Init(&pidLeft);
  PIDController_Init(&pidRight);
}
void loop(){
  setMotor(pwr[0], INA_L, INB_L, PWM_L);
  setMotor(pwr[1], INA_R, INB_R, PWM_R);
  currT = millis();
  pos[0] = encLeft.read();
  pos[1] = encRight.read();
  
  if(currT-prevT>50){
    velocity[0] = (pos[0]-posPrev[0])*1e3/((float) (currT-prevT));
    velocity[1] = (posPrev[1]-pos[1])*1e3/((float) (currT-prevT));
    velocity[0] = (velocity[0] / TICKS_PER_REV)*60;
    velocity[1] = (velocity[1] / TICKS_PER_REV)*60;

    Serial.print("left velocity: ");
    Serial.print(velocity[0]);

    Serial.print(" right velocity: ");
    Serial.println(velocity[1]);

    posPrev[0] = pos[0];
    posPrev[1] = pos[1];
    //Calculate PID of each wheel
    pwr[0] = PIDController_Update(&pidLeft, setRPM, velocity[0]);
    pwr[1] = PIDController_Update(&pidRight, setRPM, velocity[1]);

    prevT = currT;
   }
}





