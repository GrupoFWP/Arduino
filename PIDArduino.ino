#include <PID_v1.h>
#define MOTOR_A 6
#define MOTOR_B 5
#define WHEEL_HOLES 20 //estos son los huecos de la ruedita de acrílico negra
#define WHEEL_ENCODER_INTERRUPT_PIN 3
#define PWM_MAX 255
#define PWM_MIN 30
#define VEL_MAX 100
#define VEL_MIN -100
#define R_in_mm 16 //16 es el radio en mm de la ruedita de acrílico negra
#define ANGULAR_VEL_MIN (-16 * PI) 
#define ANGULAR_VEL_MAX (16 * PI)
#define LINEAR_VEL_MIN 100 //milimetros por segundo
#define LINEAR_VEL_MAX 100
#define CONST_PART_ROTATIONAL_SPEED ((2 * PI)/WHEEL_HOLES)
#define KP 2
#define KI 0.1
#define KD 0.25

volatile unsigned long wheel_pulses = 0;
//double wheel_angular_velocity = 0.0;
double wheel_linear_velocity = 0.0;
volatile unsigned long wheel_last_time = 0, wheel_actual_time = 0;

/* Gets the current angular velocity based on the encoders pulses in "rpm" */
/*
double angular_velocity(unsigned long pulses, double measurement_time) {
  return (2 * PI * pulses)/(WHEEL_HOLES * measurement_time);
}
*/

double linear_velocity(unsigned long pulses, double measurement_time) {
  return ((2 * PI * R_in_mm *(pulses/(WHEEL_HOLES)))/ measurement_time);
}

/*
void angular_velocity_measurement() {
   //Gets the angular velocity for the wheels.The 0.3 seconds is the update time.
  wheel_angular_velocity = angular_velocity(wheel_pulses, 0.3);
  wheel_pulses = 0; // Reset the measurements
}
*/

void linear_velocity_measurement() {
   //Gets the angular velocity for the wheels.The 0.3 seconds is the update time.
  wheel_linear_velocity = linear_velocity(wheel_pulses, 0.3);
  wheel_pulses = 0; // Reset the measurements


//Mas abajo hay un interrupt, que cuando detecta rising edge llama a esta funcion (el interrupt basicamente hace q el micro deje de hacer lo que estaba haceiendo y ejecute esta funcion). Esta funcion es un contador, que le suman a la función "wheel_pulses" uno cada vez que es llamada. Esta va por otro lado del PID, tal que al PID lo llamas cada X tiempo, y al finalizar el loop resetea el valor de "rwheel_pulses"
void count_pulse(volatile unsigned long &pulses, volatile unsigned long &last_time) {
  // Trying to get rid of wrong counts.
  unsigned long actual_time = millis();
  if(last_time + 10 < actual_time)
    ++pulses;
  last_time = actual_time;
}

/* Wheel's interrupt function for the encoder pulses */
void wheel_encoder_pulse() {
  count_pulse(wheel_pulses, wheel_last_time);
}



/* Custom map is needed to output a double */
double custom_map(long x, double out_min, double out_max, long in_min, long in_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


/*
double compute_angular_velocity_setpoint(int original_velocity) {
  return abs(custom_map(original_velocity, ANGULAR_VEL_MIN, ANGULAR_VEL_MAX, VEL_MIN, VEL_MAX));
}
*/


double compute_linear_velocity_setpoint(int original_velocity) {
  return abs(custom_map(original_velocity, LINEAR_VEL_MIN, LINEAR_VEL_MAX, VEL_MIN, VEL_MAX));
}



/* Outputs power to the motors */
void power_motor(int pin_a, int pin_b, int original_velocity, int PWM) {
  if(original_velocity == 0)
    return;
  analogWrite(pin_a, original_velocity > 0 ? PWM_MIN + PWM : 0);
  analogWrite(pin_b, original_velocity > 0 ? 0 : PWM_MIN + PWM);
}

/* Performs a software delay */
void software_delay(unsigned long milliseconds) {
  unsigned long t0 = millis();
  while(millis() < t0 + milliseconds);
}



/* Moves the robot */
/*
  Input:
    motorL_vel and motorR_vel: motor velocity (speed or power), from -100 to 100.
    durarion: time in seconds that the motor should be on.
*/

/*
void motor(int motorR_vel, float duration){
  //Maps the provided velocity to angular velocity
  double setpoint, wheel_PWM = 0.0;
  double setpoint_angular_vel = compute_angular_velocity_setpoint(motorR_vel);
  unsigned long motor_duration = duration * 1000;

  wheel_pulses = 0;

  // Creates and configures all PID's
  PID WheelPID(&wheel_angular_velocity, &wheel_PWM, &setpoint_angular_vel, KP, KI, KD, DIRECT);
  WheelPID.SetMode(AUTOMATIC);
  
  // Initializes and configures interruptions 
  attachInterrupt(digitalPinToInterrupt(WHEEL_ENCODER_INTERRUPT_PIN), wheel_encoder_pulse, RISING);
 
 //Movement loop - moves until duration is archieved
  unsigned long t0 = millis(); // Tiempo del comienzo del movimiento
  while(millis() < (t0 + motor_duration)) {
    angular_velocity_measurement();
    WheelPID.Compute();
    //Write PWM values, taking into consideration if the original velocity was negative or positive
    power_motor(MOTOR_A, MOTOR_B, motorR_vel, wheel_PWM);
    //Software delay to allow interruptions
    software_delay(300);
  }
  
  //Removes all interrupts and stops the motors
  noInterrupts();
  detachInterrupt(digitalPinToInterrupt(WHEEL_ENCODER_INTERRUPT_PIN));
  analogWrite(MOTOR_A, 0);
  analogWrite(MOTOR_B, 0);
  interrupts();
}
*/


void motor(int motorR_vel, float duration){
  //Maps the provided velocity to linear velocity
  double setpoint, wheel_PWM = 0.0;
  double setpoint_linear_vel = compute_linear_velocity_setpoint(motorR_vel);
  unsigned long motor_duration = duration * 1000;

  wheel_pulses = 0;

  // Creates and configures all PID's
  PID WheelPID(&wheel_linear_velocity, &wheel_PWM, &setpoint_linear_vel, KP, KI, KD, DIRECT);
  WheelPID.SetMode(AUTOMATIC);
  
  // Initializes and configures interruptions 
  attachInterrupt(digitalPinToInterrupt(WHEEL_ENCODER_INTERRUPT_PIN), wheel_encoder_pulse, RISING);
 
 //Movement loop - moves until duration is archieved
  unsigned long t0 = millis(); // Tiempo del comienzo del movimiento
  while(millis() < (t0 + motor_duration)) {
    linear_velocity_measurement();
    WheelPID.Compute();
    //Write PWM values, taking into consideration if the original velocity was negative or positive
    power_motor(MOTOR_A, MOTOR_B, motorR_vel, wheel_PWM);
    //Software delay to allow interruptions
    software_delay(300);
  }
  
  //Removes all interrupts and stops the motors
  noInterrupts();
  detachInterrupt(digitalPinToInterrupt(WHEEL_ENCODER_INTERRUPT_PIN));
  analogWrite(MOTOR_A, 0);
  analogWrite(MOTOR_B, 0);
  interrupts();
}






void setup() {
  Serial.begin(9600);
  /* ================ motor setup ================ */
  pinMode(MOTOR_A, OUTPUT);
  pinMode(MOTOR_B, OUTPUT);
  analogWrite(MOTOR_A, 0);
  analogWrite(MOTOR_B, 0);
}

void loop() {
  
  motor(10, 1);

  while(true)
    delay(1000);
}