#include "bsp.h"
#include "Romi.h"
#include "pidLib.h"
#include "motor.hpp"
#include "kinematics.hpp"
HardwareSerial& bthc05(Serial1);
/* we have created with default pins */

#define KP_VAL 1    // increase response time, but it'll increase oscillation also
#define KI_VAL 0.3  // minimise the total error 
#define KD_VAL 1    // if there is a huge changing

#define M_SPEED 20

#define SQUARE_DISTANCE 300//mm

#define TURN(n) (n > 0 ? (n - 5.2*n/90.0) : (n - 7.2*n/90.0))//calibration

/* Motor instances */
myMotor<uint8_t> leftMotorInstance(L_DIR_PIN, L_PWM_PIN);
myMotor<uint8_t> rightMotorInstance(R_DIR_PIN, R_PWM_PIN);
/* Line sensor instance */

//PID instances for right and left motor
PID pidForLeft(KP_VAL, KI_VAL, KD_VAL);
PID pidForRight(KP_VAL, KI_VAL, KD_VAL);

//These variables for handling the finite state machine
uint8_t current_state;
uint8_t next_state = IDLE_STATE;
uint32_t blocking_time {0};

int left_motor_speed {M_SPEED};
int right_motor_speed {M_SPEED};
int turning_angle {0};

float home_distance = {0};

//Start Romi from origin point
kinematics knm(0, 0);// x-y coordinate

void motorHandleTask(void);
taskInsert motorHandleTaskIns(motorHandleTask, 15);

//This task will be called in a periodic time.
void motorHandleTask() {
  //Update the motor speeds by using PID feedback control to let them drive in a stable speed.
  leftMotorInstance.motorControl(pidForLeft.updateValue(left_motor_speed,
                                 leftMotorInstance.readMotorSpeed(&count_e0)));

  rightMotorInstance.motorControl(pidForRight.updateValue(right_motor_speed,
                                  rightMotorInstance.readMotorSpeed(&count_e1)));
}

void bluetoothSendingTask(void) {
  bthc05.print(knm.get_Yaxis()); bthc05.print("\t"); 
  bthc05.println(knm.get_Xaxis());
}

bool taskHandler() {
  static uint8_t countTask{ 0 };

  if (countTask == 0) {
    if (knm.get_Xaxis() <= SQUARE_DISTANCE) {
      return false;
    }
    else {
      countTask++;
      return true;
    }
  } else if (countTask == 1) {
    if (knm.get_Yaxis() <= SQUARE_DISTANCE) {
      return false;
    }
    else {
      countTask++;
      return true;
    }
  } else if (countTask == 2) {
    if (knm.get_Xaxis() >= 0) {
      return false;
    }
    else {
      countTask++;
      return true;
    }
  } else if (countTask == 3) {
    if (knm.get_Yaxis() >= 0) {
      return false;
    }
    else {
      countTask++;
      return true;
    }
  }
}


void setup() {
  //Start board support packege which is spesific for our MCU
  bsp_ctor();
  //Reset all PID variables
  pidForRight.reset();
  pidForLeft.reset();
  bthc05.begin(9600);
  GO_HANDLE(IDLE_STATE); // start with handling IDLE state
  delay(7000);
}

uint32_t blueTime = 0;

void loop() {
  /* This Class spesific static function will be called for updating
     the time. It will be the same for all instances. */
  blueTime++;
  if (blueTime > 25) {
    bluetoothSendingTask();
    blueTime = 0;
  }
  taskInsert::executeTasks();
  // Update the kinematic variables using the encoder counting rates.
  knm.kinematicupdate(count_e0, count_e1);
  switch (current_state) {

    case IDLE_STATE: {
        static bool initial = true;
        /*Start the state machine with the idle state (do nothing here)
          GO_HANDLE macro and the others are defined in the Romi.h file.
          They are used to navigate the flow of control. */
        knm.printVals();

        left_motor_speed = M_SPEED;
        right_motor_speed = M_SPEED;
        //knm.resetDistanceFrom();
        GO_HANDLE(PATH_TRACING);
        break;
      }

    case PATH_TRACING: {

        knm.printVals();
        if (!taskHandler()) {
          knm.printVals();
          motorHandleTaskIns.callMyTask();
          GO_HANDLE(PATH_TRACING);
        } else {
          turning_angle = TURN(90);
          GO_HANDLE(TURN_ROMI_STATE);
        }
        /*if (knm.getDistanceFrom() < SQUARE_DISTANCE) {
          knm.printVals();
          motorHandleTaskIns.callMyTask();
          GO_HANDLE(PATH_TRACING);
          } else {
          knm.resetDistanceFrom();
          turning_angle = TURN(-90);
          GO_HANDLE(TURN_ROMI_STATE);
          }*/
        break;
      }

    case TURN_ROMI_STATE: {
        /*this sate will handle the romi turning process according to
          value of turning_angle, if it is 90 romi will turn 90 degree right
          if it is -180, romi will turn left 180 degree.*/
        static bool isEnteredFirst {true};
        static float last_angle{0};
        float system_angle = knm.get_angle();

        knm.printVals();
        if (blueTime > 500) {
          bluetoothSendingTask();
          blueTime = 0;
        }
        if (isEnteredFirst) {
          //if entered first, get the target angle
          isEnteredFirst = false;
          last_angle = system_angle + turning_angle;
        }

        if ( turning_angle >= 0) {
          if ( system_angle < last_angle) {
            left_motor_speed = M_SPEED;
            right_motor_speed = -M_SPEED;
          } else {
            isEnteredFirst = true;
            BREAK_AND_GO(MOTOR_STOP);
          }
        } else if ( turning_angle < 0) {
          if ( system_angle > last_angle) {
            left_motor_speed = -M_SPEED;
            right_motor_speed = M_SPEED;
          } else {
            isEnteredFirst = true;
            BREAK_AND_GO(MOTOR_STOP);
          }
        }
        //call motor task
        motorHandleTaskIns.callMyTask();

        GO_HANDLE(TURN_ROMI_STATE);
        break;
      }

    case MOTOR_STOP: {
        leftMotorInstance.motorControl(0);
        rightMotorInstance.motorControl(0);
        GO_HANDLE(IDLE_STATE);
        break;
      }

    case STOP_SYSTEM: {
        leftMotorInstance.motorControl(0);
        rightMotorInstance.motorControl(0);
        GO_HANDLE(STOP_SYSTEM);
        break;
      }

    default : {
        GO_HANDLE(IDLE_STATE);
        break;
      }
  }
}

