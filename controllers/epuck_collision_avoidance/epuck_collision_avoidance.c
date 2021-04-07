#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>

#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>
#include <webots/receiver.h>
#include <webots/emitter.h>
#include <webots/led.h>

/* Motor device */
static WbDeviceTag left_motor, right_motor;

/* E-puck angular speed in rad/s */
#define MAX_SPEED 6.28

/* distance sensor */
#define NUMBER_OF_DISTANCE_SENSORS 8
static WbDeviceTag distance_sensors[NUMBER_OF_DISTANCE_SENSORS];
static const char *distance_sensors_names[NUMBER_OF_DISTANCE_SENSORS] = {"ps0", "ps1", "ps2", "ps3", "ps4", "ps5", "ps6", "ps7"};

/* Central communication */
WbDeviceTag receiver;
WbDeviceTag emitter;

WbDeviceTag rgbLed;

/*
 * The sensor has noise
 * So even there is no obstacle, sensor values is not zero
 * So to detect obstacle, we must use this threshold
 * Based on my experiment, the good threshold value is 140
 * Obstacle detected condition is true if the sensor values is larger then this threshold value
 * */
// #define SENSOR_VALUE_DETECTION_THRESHOLD 140
#define SENSOR_VALUE_DETECTION_THRESHOLD 50

/* speed of robot to spinning in place (in degrees per second) */
// #define ROBOT_ANGULAR_SPEED_IN_DEGREES 283.588111888
#define ROBOT_ANGULAR_SPEED_IN_DEGREES 85.0

static int TIME_STEP_CONTROLLER;

/* function to init robot controller stuff */
void robot_controller_init(int time_step)
{
  TIME_STEP_CONTROLLER = time_step;
  /* get a handler to the motors and set target position to infinity (speed control) */
  left_motor = wb_robot_get_device("left wheel motor");
  right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);
  
  /* get a handler to the sensors */
  for (int i = 0; i < NUMBER_OF_DISTANCE_SENSORS; i++) {
    distance_sensors[i] = wb_robot_get_device(distance_sensors_names[i]);
    wb_distance_sensor_enable(distance_sensors[i], TIME_STEP_CONTROLLER);
  }
  
  receiver = wb_robot_get_device("receiver");
  wb_receiver_enable(receiver, time_step);
  emitter = wb_robot_get_device("emitter"); 
  
  rgbLed = wb_robot_get_device("ledrgb"); 
}

static float calculate_rotation_time(float degrees)
{
  return abs(degrees) / ROBOT_ANGULAR_SPEED_IN_DEGREES;
}

/* function to stop the motor (set motor velocity to zero) */
void motor_stop() {
  wb_motor_set_velocity(left_motor, 0);
  wb_motor_set_velocity(right_motor, 0);
}

/* function to set motor velocity to move forward */
void motor_move_forward() {
  wb_motor_set_velocity(left_motor, MAX_SPEED);
  wb_motor_set_velocity(right_motor, MAX_SPEED);
}

/* function to set motor velocity to move backward */
void motor_move_backward() {
  wb_motor_set_velocity(left_motor, -MAX_SPEED);
  wb_motor_set_velocity(right_motor, -MAX_SPEED);
}

/* function to set motor velocity to rotate right in place*/
void motor_rotate_right() {
  wb_motor_set_velocity(left_motor, MAX_SPEED);
  wb_motor_set_velocity(right_motor, -MAX_SPEED);
}

/* function to set motor velocity to rotate left in place*/
void motor_rotate_left() {
  wb_motor_set_velocity(left_motor, -MAX_SPEED);
  wb_motor_set_velocity(right_motor, MAX_SPEED);
}

void motor_rotate_left_in_degrees(float degrees) {
  motor_rotate_left();
	
  float duration = calculate_rotation_time(degrees);
  // printf("Duration Left Turn: %0.4f\n", duration);
  float start_time = wb_robot_get_time();
  do
  {
    wb_robot_step(TIME_STEP_CONTROLLER);
  } while (wb_robot_get_time() < start_time + duration);
	
  motor_stop();
}

void motor_rotate_right_in_degrees(float degrees) {
  motor_rotate_right();
	
  float duration = calculate_rotation_time(degrees);
  // printf("Duration Right Turn: %0.2f\n", duration);
  float start_time = wb_robot_get_time();
  do
  {
    wb_robot_step(TIME_STEP_CONTROLLER);
  } while (wb_robot_get_time() < start_time + duration);
	
  motor_stop();
}

/* function to get sensors condition 
 * if sensor detect obstacle, then the condition is true
 * */
bool * get_sensors_condition()
{
  static bool sensors_condition[NUMBER_OF_DISTANCE_SENSORS] = {false};
	
  for (int i = 0; i < NUMBER_OF_DISTANCE_SENSORS ; i++) {
    /*
    * Obstacle detected condition is true if the sensor values is larger then the threshold value
    * */
    if (wb_distance_sensor_get_value(distance_sensors[i]) > SENSOR_VALUE_DETECTION_THRESHOLD) {
      sensors_condition[i] = true;
      } else {
      sensors_condition[i] = false;
		}
      }
    return sensors_condition;
}

/* function to print sensors values
 * */
void print_sensor_values() {
  printf("%s sensor values: ", wb_robot_get_name());
	
  for (int i = 0; i < NUMBER_OF_DISTANCE_SENSORS ; i++) {
    printf("%d:%.3f ", i, wb_distance_sensor_get_value(distance_sensors[i]));
  }
	
  printf("\n");
}


#include <stdio.h>

#include <webots/robot.h>

int TIME_STEP;

/* function to init robot stuff */
static void init_robot() {
  /* necessary to initialize webots stuff */
  wb_robot_init();
  
  /* get simulator time step */
  TIME_STEP = (int)wb_robot_get_basic_time_step();
	
  /* init the controller */
  robot_controller_init(TIME_STEP);	
}

/* main function */
int main(int argc, char **argv) {
  init_robot();
  
  int state = -1;
  int mode = 0;
  int contr_rot_deg =0;
  int contr_rot_mode =0;
  int contr_trans_steps =0;
  int contr_trans_mode =0;
  bool contr_rot = false;
  bool contr_trans = false;
  
  long int currentColor = 0;
  
  /* main loop
   * Perform simulation steps of TIME_STEP milliseconds
   * and leave the loop when the simulation is over
   */
  
  while (wb_robot_step(TIME_STEP) != -1) {		
    if(wb_receiver_get_queue_length(receiver) > 0) {    
      printf("Received Message\n");  
      const char *msg = wb_receiver_get_data(receiver);
      wb_receiver_next_packet(receiver); 
      
      state = msg[1];
      mode = msg[2];
      
      // printf("State: %d\n", state);
      // printf("Mode: %d\n", mode);
      
      contr_rot_deg = msg[3];
      contr_rot_mode = msg[4];
      contr_trans_mode = msg[5];
      
      char s1[20];
      char s2[20];
      char s3[20];
      char s4[20];
      
      // Convert both the integers to string
      sprintf(s1, "%d", msg[6]);
      sprintf(s2, "%d", msg[7]);
      sprintf(s3, "%d", msg[8]);
      sprintf(s4, "%d", msg[9]);
      
      // Concatenate both strings
      strcat(s1, s2);
      strcat(s1, s3);
      strcat(s1, s4);
      
      // Convert the concatenated string to integer
      contr_trans_steps = atoi(s1);
      
      contr_rot = true;
      contr_trans = true;
    } 
    

    
    if (state == 1){
      if (mode == 1) {
        currentColor = 0*65536 + 100*256 + 0;
        wb_led_set(rgbLed, currentColor); // 0x0000ff);
      } else {
        currentColor = 100*65536 + 0*256 + 0;
        wb_led_set(rgbLed, currentColor); // 0x0000ff);
      }
    } else if (state == 0){
      currentColor = 0*65536 + 0*256 + 100;
      wb_led_set(rgbLed, currentColor); // 0x0000ff);
    } else {
      currentColor = 0*65536 + 0*256 + 0;
      wb_led_set(rgbLed, currentColor); // 0x0000ff);
    }

    
      
    bool *is_sensors_active = get_sensors_condition();
    // print_sensor_values();
    /*COLLISION AVOIDANCE */
    
    if (state == 1){
    
      if (contr_trans_mode && ((is_sensors_active[0] && is_sensors_active[1]) || is_sensors_active[1])) {
        // print_sensor_values();
        motor_rotate_left();
      } else if (contr_trans_mode && ((is_sensors_active[0] && is_sensors_active[7]) || is_sensors_active[7])) {
        // print_sensor_values();
        motor_rotate_right();
      } else if (contr_trans_mode && is_sensors_active[0]) {
        // print_sensor_values();
        motor_rotate_left_in_degrees(180);
      } else if (!contr_trans_mode && ((is_sensors_active[4] && is_sensors_active[5]) || is_sensors_active[5])) {
        // print_sensor_values();
        motor_rotate_left();
      } else if (!contr_trans_mode && ((is_sensors_active[4] && is_sensors_active[3]) || is_sensors_active[3])) {
        // print_sensor_values();
        motor_rotate_right();
      } else if (!contr_trans_mode && is_sensors_active[4]) {
        // print_sensor_values();
        motor_rotate_left_in_degrees(180);
      } else {
        /*CONTROLLED ROTATION */
        if (contr_rot) {
          if (contr_rot_mode) {
          motor_rotate_left_in_degrees(contr_rot_deg);
          } else {
          motor_rotate_right_in_degrees(contr_rot_deg);
          }
          contr_rot = false;
         } else if (contr_trans) {
        /*CONTROLLED TRANSLATION */ 
        if (contr_trans_mode) { 
          motor_move_forward();
          } else {
          motor_move_backward();
        }
        
        if (contr_trans_steps > 0) {
          contr_trans_steps -= 1;
          // printf("%d\n", contr_trans_steps);
          } else {
          contr_trans = false;
          }
        } else {
          /*action if not controlled nor avoiding */    	
          motor_stop();
          // motor_move_forward();
        }
        }
    } else {
    motor_stop();
    }
  };

  /* Enter your cleanup code here */

  /* This is necessary to cleanup webots resources */
  wb_robot_cleanup();

  return 0;
}



