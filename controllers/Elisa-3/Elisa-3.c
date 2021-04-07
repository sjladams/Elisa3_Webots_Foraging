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


#define LEDS_NUMBER 8
static WbDeviceTag leds[LEDS_NUMBER];
static const char *leds_names[LEDS_NUMBER] = {
  "led0", "led1", "led2", "led3",
  "led4", "led5", "led6", "led7",
};
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
  int i;
  char redValue=0, blueValue=0, greenValue=0;
  char flagValue=0;
  char rSpeedValue=0, lSpeedValue=0;
  char greenLedsValue=0;
  int prox_value[8];
  int prox_ambient_value[8];
  int ground_value[4];
  int ground_ambient_value[4];
  const double *accValue;
  char ackPayload[81] = {0};

  /* initialize Webots */
  wb_robot_init();

  time_step = wb_robot_get_basic_time_step();

  /* get and enable devices */
  init_devices();
  
  step(); // to compute distance sensors values

  /* main loop */
  while (true) {    
   
    if(wb_receiver_get_queue_length(receiver) > 0) {
      //printf("queue length = %d\n", wb_receiver_get_queue_length(receiver));
      //while(wb_receiver_get_queue_length(receiver) > 1) {
      //  wb_receiver_next_packet(receiver); 
      //}
      
      const char *msg = wb_receiver_get_data(receiver);
      //printf("msg is %d, %d, %d, ...\n", msg[0], msg[1], msg[2]);
      
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
      
      // get the speed: the MSbit indicates whether the velocity is positive (bit set) or negative (bit clear)
      rSpeedValue = msg[4]&0x7F;
      lSpeedValue = msg[5]&0x7F;
      if((msg[4]&0x80)==0x00) {
        rSpeedValue = - rSpeedValue;
      }
      if((msg[5]&0x80)==0x00) {
        lSpeedValue = - lSpeedValue;
      }
      wb_differential_wheels_set_speed(lSpeedValue, rSpeedValue);
      
      greenLedsValue = msg[6];
      for (i=0; i<LEDS_NUMBER; i++) { 
        if(greenLedsValue&(i<<0)) {
          wb_led_set(leds[i], true);
        } else {
          wb_led_set(leds[i], false);
        }
      }      
                        
      // get prox values
      for (i = 0; i < 8; i++) {
        prox_value[i] = wb_distance_sensor_get_value(distance_sensor[i]);
        prox_ambient_value[i] = wb_light_sensor_get_value(distance_ambient_sensor[i]);      
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
      ackPayload[2] = prox_value[0]&0xFF;
      ackPayload[3] = prox_value[0]>>8;
      ackPayload[4] = prox_value[1]&0xFF;
      ackPayload[5] = prox_value[1]>>8;
      ackPayload[6] = prox_value[2]&0xFF;
      ackPayload[7] = prox_value[2]>>8;
      ackPayload[8] = prox_value[3]&0xFF;
      ackPayload[9] = prox_value[3]>>8;
      ackPayload[10] = prox_value[5]&0xFF;
      ackPayload[11] = prox_value[5]>>8;
      ackPayload[12] = prox_value[6]&0xFF;
      ackPayload[13] = prox_value[6]>>8;
      ackPayload[14] = prox_value[7]&0xFF;
      ackPayload[15] = prox_value[7]>>8;
      ackPayload[16] = 0;
      ackPayload[17] = 4; // second packet
      ackPayload[18] = prox_value[4]&0xFF;
      ackPayload[19] = prox_value[4]>>8;
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
      
      wb_receiver_next_packet(receiver);                 
    
      wb_emitter_send(emitter, ackPayload, 81); 
    
    }    
  
    step();
    
  }
  
  wb_robot_cleanup();

  return 0;

//ciao
}
