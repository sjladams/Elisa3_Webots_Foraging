/*
 * File:         void.c
 * Description:  This is an empty robot controller, the robot does nothing. 
 * Author:       www.cyberbotics.com
 * Note:         !!! PLEASE DO NOT MODIFY THIS SOURCE FILE !!!
 *               This is a system file that Webots needs to work correctly.
 */

#include <webots/robot.h>
#include <webots/supervisor.h>
#include <webots/receiver.h>
#include <webots/emitter.h>
#include <stdio.h>
#include <stdlib.h>

#define ROBOTS 10
#define WALL_THR 300
#define SPEED 30

char speed(char value) {
    if(value >= 0) {
        return (value|0x80);
    } else {
        return ((-value)&0x7F);
    }
}

int main() {

  int i=0, j=0;
  char txData[7];
  //char rxData[16] = {0};
  const char *robot_name[ROBOTS] = { "ELISA3-1", "ELISA3-2", "ELISA3-3", "ELISA3-4", "ELISA3-5", "ELISA3-6", "ELISA3-7", "ELISA3-8", "ELISA3-9", "ELISA3-10"};
  WbNodeRef robot[ROBOTS];
  WbFieldRef robot_fields[2*ROBOTS];
  WbDeviceTag receiver;
  WbDeviceTag emitter;
  char motionState=0;
  char colorState=0;
  int timeout=0;

  wb_robot_init();
  int time_step = wb_robot_get_basic_time_step();
  //printf("time_step = %d\n", time_step); //32
  
  for(i=0, j=0; i<ROBOTS; i++, j+=2) {
    robot[i] = wb_supervisor_node_get_from_def(robot_name[i]);
    robot_fields[j] = wb_supervisor_node_get_field(robot[i], "emitter_channel");
    robot_fields[j+1] = wb_supervisor_node_get_field(robot[i], "receiver_channel");
    wb_supervisor_field_set_sf_int32(robot_fields[j], i);
    wb_supervisor_field_set_sf_int32(robot_fields[j+1], i);   
  }
  
  receiver = wb_robot_get_device("receiver0");
  wb_receiver_enable(receiver, time_step);
  wb_receiver_set_channel(receiver, -1); // receive on all channels
  emitter = wb_robot_get_device("emitter0");   

  while(1) { 
  
    wb_robot_step(time_step);
  
    switch(motionState) {
      case 0: // go forward
        txData[4] = speed(SPEED);
        txData[5] = speed(SPEED);
        break;
      case 1: // rotate right
        txData[4] = speed(-SPEED);
        txData[5] = speed(SPEED);
        break;
      case 2: // rotate left
        txData[4] = speed(SPEED);
        txData[5] = speed(-SPEED);
        break;
      case 3: // go backward
        txData[4] = speed(-SPEED);
        txData[5] = speed(-SPEED);                       
        break;
    }
        
    switch(colorState) {
      case 0: // all red
        txData[0]=100;
        txData[1]=0;
        txData[2]=0;
        timeout++;
        if(timeout >= 3000/time_step) {
          timeout = 0;
          colorState = 1;
        }
        break;
      case 1: // all green
        txData[0]=0;
        txData[1]=0;
        txData[2]=100;      
        timeout++;
        if(timeout >= 3000/time_step) {
          timeout = 0;
          colorState = 2;
        }  
        break;
      case 2: // all blue
        txData[0]=0;
        txData[1]=100;
        txData[2]=0;      
        timeout++;
        if(timeout >= 3000/time_step) {
          timeout = 0;
          colorState = 3;
        }
        break;
      case 3: // all white
        txData[0]=100;
        txData[1]=100;
        txData[2]=100;      
        timeout++;
        if(timeout >= 3000/time_step) {
          timeout = 0;
          colorState = 0;
        }                       
        break;
    }        
            
    for(j=0; j<ROBOTS; j++) {
      wb_emitter_set_channel(emitter, j);
      wb_emitter_send(emitter, txData, 7);
      if(wb_receiver_get_queue_length(receiver) > 0) {
        const unsigned char *msg = wb_receiver_get_data(receiver);
        //printf("Supervisor (%d): queue length = %d\n", msg[0], wb_receiver_get_queue_length(receiver));
        if(msg[0]==9) {
          //printf("Supervisor: prox0 = %d (%d, %d)\n", msg[2]+msg[3]*255, msg[2], msg[3]);
          int temp = msg[2]+msg[3]*255;
          if(temp >= WALL_THR) {
            //printf("motionState=3\n");
            motionState = 3;
          }
        }
        if(msg[0]==2) {
          //printf("Supervisor: prox4 = %d (%d, %d)\n", msg[18]+msg[19]*255, msg[18], msg[19]);
          int temp = msg[18]+msg[19]*255;
          if(temp >= WALL_THR) {
            //printf("motionState=0\n");
            motionState = 0;
          }
        }      
        wb_receiver_next_packet(receiver); 
      }
      
    }
      
  }
  
}

