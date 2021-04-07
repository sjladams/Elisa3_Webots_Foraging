#include <webots/robot.h>
#include <webots/differential_wheels.h>
#include <webots/distance_sensor.h>
#include <webots/led.h>
#include <webots/accelerometer.h>
#include <webots/light_sensor.h>
#include <webots/receiver.h>
#include <webots/emitter.h>
#include <stdio.h>
#include <stdlib.h>

#include <webots/motor.h>


#define LEDS_NUMBER 8
static WbDeviceTag leds[LEDS_NUMBER];
static const char *leds_names[LEDS_NUMBER] = {
  "led0", "led1", "led2", "led3",
  "led4", "led5", "led6", "led7",
};

double speed[2] = {0, 0};
double braitenberg_coefficients[8][2] = {{10, 8}, {7, -1.5}, {5, -1}, {-1, -1}, {-1, -1}, {-1, -1}, {-1, 5}, {-1.5, 7}};
#define RANGE (127/2)

WbDeviceTag rgbLed;
WbDeviceTag distance_sensor[8];
WbDeviceTag distance_ambient_sensor[8];
WbDeviceTag ground_sensor[4];
WbDeviceTag ground_ambient_sensor[4];
WbDeviceTag receiver;
WbDeviceTag emitter;
WbDeviceTag accelerometer;
int time_step;

static void step() {
  if (wb_robot_step(time_step) == -1) {
    wb_robot_cleanup();
    exit(EXIT_SUCCESS);
  }
}

static void init_devices() {
  int i;
  char device_name[4];
  char device_name2[6];
 
  for (i=0; i<LEDS_NUMBER; i++) {
    leds[i]=wb_robot_get_device(leds_names[i]);
  }
 
  rgbLed = wb_robot_get_device("ledrgb");
 
  accelerometer = wb_robot_get_device("accelerometer");
  wb_accelerometer_enable(accelerometer, time_step*4);
 
  for (i = 0; i < 8; i++) {
    sprintf(device_name, "ps%d", i);
    distance_sensor[i] = wb_robot_get_device(device_name);
    wb_distance_sensor_enable(distance_sensor[i],time_step);
    sprintf(device_name, "ls%d", i);
    distance_ambient_sensor[i] = wb_robot_get_device(device_name);
    wb_light_sensor_enable(distance_ambient_sensor[i],time_step*4);
        
  }

  for (i = 0; i < 4; i++) {
    sprintf(device_name, "fs%d", i);
    ground_sensor[i] = wb_robot_get_device(device_name);
    wb_distance_sensor_enable(ground_sensor[i],time_step);
    sprintf(device_name2, "lsfs%d", i);
    ground_ambient_sensor[i] = wb_robot_get_device(device_name2);
    wb_light_sensor_enable(ground_ambient_sensor[i],time_step*4);
        
  }
    
  receiver = wb_robot_get_device("receiver");
  wb_receiver_enable(receiver, time_step);
  
  emitter = wb_robot_get_device("emitter");  
      
  step();
  
}



int main(int argc, char *argv[]) {

  /* define variables */
  long int currentColor = 0;
  int i,j;
  char redValue=0, blueValue=0, greenValue=0;
  char flagValue=0;
  // char rSpeedValue=0, lSpeedValue=0;
  double SpeedValue[2] = {0, 0};
  char greenLedsValue=0;
  int prox_value[8];
  int prox_ambient_value[8];
  int ground_value[4];
  int ground_ambient_value[4];
  const double *accValue;
  char ackPayload[81] = {0};
  
  bool check_collision = false;  

  int control_steps=0;
  
  WbDeviceTag left_motor, right_motor;

  /* initialize Webots */
  wb_robot_init();

  time_step = wb_robot_get_basic_time_step();

  /* get and enable devices */
  init_devices();
  
  /* get a handler to the motors and set target position to infinity (speed control). */
  left_motor = wb_robot_get_device("left wheel motor");
  right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);
  
  step(); // to compute distance sensors values

  /* main loop */
  while (true) {    
   
    if(wb_receiver_get_queue_length(receiver) > 0) {    
      printf("Received Message\n");  
      const char *msg = wb_receiver_get_data(receiver);
      
      // get the rgb values: 0 => off, 100 => max power      
      redValue = msg[0];
      if(redValue > 100) {
        redValue = 100;
      }
      blueValue = msg[1];
      if(blueValue > 100) {
        blueValue = 100;
      }      
      greenValue = msg[2];
      if(greenValue > 100) {
        greenValue = 100;
      }   
      currentColor = redValue*65536 + greenValue*256 + blueValue;
      wb_led_set(rgbLed, currentColor); // 0x0000ff);
      
      flagValue = msg[3];
      if((flagValue&0x01)==0x01) {
        // turn on back IR => not implemented
      } else {
        // turn off back IR
      }
      if((flagValue&0x02)==0x02) {
        // turn on front IRs => not implemented
      } else {
        // turn off front IRs
      }      
      if((flagValue&0x04)==0x04) {
        // enable TV IR receiving => not implemented
      } else {
        // disable TV IR receiving
      }
      if((flagValue&0x10)==0x10) {
        // calibrate sensors
        // reset odometry
      }
      if((flagValue&0x40)==0x40) {
        // enable obstacle avoidance
      } else {
        // disable obstacle avoidance
      }
      if((flagValue&0x80)==0x80) {
        // enable cliff avoidance
      } else {
        // disable cliff avoidance
      }
      
      // lSpeedValue = msg[5];
      // rSpeedValue = msg[4];
      SpeedValue[0]= msg[5];
      SpeedValue[1]= msg[4];
      // control_steps = msg[6] * 1000;
      
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
    
      // Convert the concatenated string
      // to integer
      control_steps = atoi(s1);
      
      
      // printf("%d\n", control_steps);
      
      wb_receiver_next_packet(receiver); 
      
      check_collision = false;  
      
    }      
                              
    // get prox values
    for (i = 0; i < 8; i++) {
      prox_value[i] = wb_distance_sensor_get_value(distance_sensor[i]);
      prox_ambient_value[i] = wb_light_sensor_get_value(distance_ambient_sensor[i]);  
    
      if (prox_value[i] > RANGE) {
        check_collision = true;
        printf("Collision avoidance True\n");
        }
    }
    
     printf("prox values = %d %d %d %d %d %d %d %d\n",
     prox_value[0], prox_value[1], prox_value[2], prox_value[3],
     prox_value[4], prox_value[5], prox_value[6], prox_value[7]);
    
    /* compute collision speed values*/
    if (check_collision & control_steps) {
      for (i = 0; i < 2; i++) {
        speed[i] = 0.0;
        for (j = 0; j < 8; j++)
          speed[i] += braitenberg_coefficients[j][i] * (1.0 - (prox_value[j] / RANGE));
      }
      
      if (speed[0] > 35) {
        speed[0] = 35;
      } 
      if (speed[0] < -35) {
        speed[0] = -35;
      } 
      if (speed[1] > 35) {
        speed[1] = 35;
      } 
      if (speed[1] < -35) {
        speed[1] = -35;
      } 
      wb_motor_set_velocity(left_motor, speed[0]);
      wb_motor_set_velocity(right_motor, speed[1]);
      printf("Collision Left speed: %0.2f\n", speed[0]);
      printf("Collision Right speed: %0.2f\n", speed[1]);
      
    } else if (control_steps) {
      // wb_motor_set_velocity(left_motor, lSpeedValue);
      wb_motor_set_velocity(left_motor, SpeedValue[0]);
      // wb_motor_set_velocity(right_motor, rSpeedValue);
      wb_motor_set_velocity(right_motor, SpeedValue[1]);
      printf("Controlled Left speed: %0.2f\n", SpeedValue[0]);
      printf("Controlled Right speed: %0.2f\n", SpeedValue[1]);
    } else {
      wb_motor_set_velocity(left_motor, 0);
      wb_motor_set_velocity(right_motor, 0);      
    }
    
    // get ground values
    for (i = 0; i < 4; i++) {
      ground_value[i] = wb_distance_sensor_get_value(ground_sensor[i]);
      ground_ambient_value[i] = wb_light_sensor_get_value(ground_ambient_sensor[i]);
    }
    // get accelerometer values
    accValue = wb_accelerometer_get_values(accelerometer);
    
    ackPayload[0] = wb_emitter_get_channel(emitter);
    ackPayload[1] = 3; // first packet
    // ackPayload[2] = prox_value[0]&0xFF;
    ackPayload[2] = prox_value[0];
    // ackPayload[3] = prox_value[0]>>8;
    // ackPayload[4] = prox_value[1]&0xFF;
    ackPayload[4] = prox_value[1];
    // ackPayload[5] = prox_value[1]>>8;
    // ackPayload[6] = prox_value[2]&0xFF;
    ackPayload[6] = prox_value[2];
    // ackPayload[7] = prox_value[2]>>8;
    // ackPayload[8] = prox_value[3]&0xFF;
    ackPayload[8] = prox_value[3];
    // ackPayload[9] = prox_value[3]>>8;
    // ackPayload[10] = prox_value[5]&0xFF;
    ackPayload[10] = prox_value[5];
    // ackPayload[11] = prox_value[5]>>8;
    // ackPayload[12] = prox_value[6]&0xFF;
    ackPayload[12] = prox_value[6];
    // ackPayload[13] = prox_value[6]>>8;
    // ackPayload[14] = prox_value[7]&0xFF;
    ackPayload[14] = prox_value[7];
    // ackPayload[15] = prox_value[7]>>8;
    ackPayload[16] = 0;
    ackPayload[17] = 4; // second packet
    // ackPayload[18] = prox_value[4]&0xFF;
    ackPayload[18] = prox_value[4];
    // ackPayload[19] = prox_value[4]>>8;
    ackPayload[20] = ground_value[0]&0xFF;
    ackPayload[21] = ground_value[0]>>8;
    ackPayload[22] = ground_value[1]&0xFF;
    ackPayload[23] = ground_value[1]>>8;
    ackPayload[24] = ground_value[2]&0xFF;
    ackPayload[25] = ground_value[2]>>8;
    ackPayload[26] = ground_value[3]&0xFF;
    ackPayload[27] = ground_value[3]>>8;
    ackPayload[28] = ((int)accValue[0])&0xFF;
    ackPayload[29] = ((int)accValue[0])>>8;
    ackPayload[30] = ((int)accValue[1])&0xFF;
    ackPayload[31] = ((int)accValue[1])>>8;
    ackPayload[32] = 0; // not implemented (TV remote input) 
    ackPayload[33] = 5; // third packet
    ackPayload[34] = prox_ambient_value[0]&0xFF;
    ackPayload[35] = prox_ambient_value[0]>>8;
    ackPayload[36] = prox_ambient_value[1]&0xFF;
    ackPayload[37] = prox_ambient_value[1]>>8;
    ackPayload[38] = prox_ambient_value[2]&0xFF;
    ackPayload[39] = prox_ambient_value[2]>>8;
    ackPayload[40] = prox_ambient_value[3]&0xFF;
    ackPayload[41] = prox_ambient_value[3]>>8;
    ackPayload[42] = prox_ambient_value[5]&0xFF;
    ackPayload[43] = prox_ambient_value[5]>>8;
    ackPayload[44] = prox_ambient_value[6]&0xFF;
    ackPayload[45] = prox_ambient_value[6]>>8;
    ackPayload[46] = prox_ambient_value[7]&0xFF;
    ackPayload[47] = prox_ambient_value[7]>>8;
    ackPayload[48] = 0; // not implemented (selector)
    ackPayload[49] = 6; // fourth packet
    ackPayload[50] = prox_ambient_value[4]&0xFF;
    ackPayload[51] = prox_ambient_value[4]>>8;
    ackPayload[52] = ground_ambient_value[0]&0xFF;
    ackPayload[53] = ground_ambient_value[0]>>8;
    ackPayload[54] = ground_ambient_value[1]&0xFF;
    ackPayload[55] = ground_ambient_value[1]>>8;
    ackPayload[56] = ground_ambient_value[2]&0xFF;
    ackPayload[57] = ground_ambient_value[2]>>8;
    ackPayload[58] = ground_ambient_value[3]&0xFF;
    ackPayload[59] = ground_ambient_value[3]>>8;
    ackPayload[60] = ((int)accValue[2])&0xFF;
    ackPayload[61] = ((int)accValue[2])>>8;
    ackPayload[62] = 0; // not implemented (battery => 2 bytes)
    ackPayload[63] = 0;
    ackPayload[64] = 0; // free
    ackPayload[65] = 7; // fifth packet
    ackPayload[66] = 0; // not implemented (left steps => 4 bytes)
    ackPayload[67] = 0;
    ackPayload[68] = 0;
    ackPayload[69] = 0;
    ackPayload[70] = 0; // not implemented (right steps => 4 bytes)
    ackPayload[71] = 0;
    ackPayload[72] = 0;
    ackPayload[73] = 0;
    ackPayload[74] = 0; // not implemented (theta => 2 bytes)
    ackPayload[75] = 0;
    ackPayload[76] = 0; // not implemented (xpos => 2 bytes)
    ackPayload[77] = 0;
    ackPayload[78] = 0; // not implemented (ypos => 2 bytes)
    ackPayload[79] = 0;
    ackPayload[80] = 0; // free
    
                  
  
    wb_emitter_send(emitter, ackPayload, 81);     
  
    if (control_steps > 0) {
      control_steps -= 1;
      // printf("%d\n", control_steps);
     }
     
    step();
    
  }
  
  wb_robot_cleanup();

  return 0;

//ciao
}
