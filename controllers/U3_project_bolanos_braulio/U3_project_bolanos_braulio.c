#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>
#include <webots/position_sensor.h>
#include <webots/keyboard.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <prom.h>

#define TIME_STEP 64
#define PI 3.1416


double A_pos_1,A_pos_dif,B_pos_1,B_pos_dif, C_pos_1, C_pos_dif;
double A_pos_2 = 0;
double B_pos_2 = 0;
double C_pos_2 = 0;
double dis_value_left;
double dis_value_right;
double dis_sen_left;
double dis_sen_right;
double ds_1;
double ds_2;
double encoder_left;
double encoder_right;
double value_cm_left;
double value_cm_right;
double noise;
double n1;
int m;
int a;
int key;
double aproach;
double aproach2;
double turn_l;
double turn_r;
float A_angv, B_angv, C_angv, A_linv, B_linv, C_linv, A_rpm, B_rpm, C_rpm;
double dis_sen;
double radio=0.075;

  void goRobot(WbDeviceTag wheel_A,WbDeviceTag wheel_B,WbDeviceTag wheel_C){
  wb_motor_set_velocity(wheel_A, -3);
  wb_motor_set_velocity(wheel_B, 3);
  wb_motor_set_velocity(wheel_C, 0);
  wb_motor_set_position(wheel_C, INFINITY);

  }
  void downRobot(WbDeviceTag wheel_A,WbDeviceTag wheel_B,WbDeviceTag wheel_C){
  wb_motor_set_velocity(wheel_A,3);
  wb_motor_set_velocity(wheel_B,-3);
  wb_motor_set_velocity(wheel_C, 0);
  }
 void leftRobot(WbDeviceTag wheel_A,WbDeviceTag wheel_B,WbDeviceTag wheel_C){
  wb_motor_set_velocity(wheel_A, -3);
  wb_motor_set_velocity(wheel_B, -3);
  wb_motor_set_velocity(wheel_C, 6.1);
  }
 void rightRobot(WbDeviceTag wheel_A,WbDeviceTag wheel_B,WbDeviceTag wheel_C){
  wb_motor_set_velocity(wheel_A, 3);
  wb_motor_set_velocity(wheel_B, 3);
  wb_motor_set_velocity(wheel_C, -6.1);
  }
 void stopRobot(WbDeviceTag wheel_A,WbDeviceTag wheel_B,WbDeviceTag wheel_C){
  wb_motor_set_velocity(wheel_A,0);
  wb_motor_set_velocity(wheel_B,0);
  wb_motor_set_velocity(wheel_C,0);
  }
 void turnRight(WbDeviceTag wheel_A,WbDeviceTag wheel_B,WbDeviceTag wheel_C){
  wb_motor_set_velocity(wheel_A,-3);
  wb_motor_set_velocity(wheel_B,-3);
  wb_motor_set_velocity(wheel_C,-3);
  }
 void turnLeft(WbDeviceTag wheel_A,WbDeviceTag wheel_B,WbDeviceTag wheel_C){
    wb_motor_set_velocity(wheel_A,3);
    wb_motor_set_velocity(wheel_B,3);
    wb_motor_set_velocity(wheel_C,3);
  }
 void goAuto(WbDeviceTag wheel_A,WbDeviceTag wheel_B,WbDeviceTag wheel_C){
    wb_motor_set_velocity(wheel_A,-1);
    wb_motor_set_velocity(wheel_B,1);
    wb_motor_set_velocity(wheel_C,0);
  }
 void turnrAuto(WbDeviceTag wheel_A,WbDeviceTag wheel_B,WbDeviceTag wheel_C){
    wb_motor_set_velocity(wheel_A,1);
    wb_motor_set_velocity(wheel_B,1);
    wb_motor_set_velocity(wheel_C,1);
  }
 void turnlAuto(WbDeviceTag wheel_A,WbDeviceTag wheel_B,WbDeviceTag wheel_C){
    wb_motor_set_velocity(wheel_A,-1);
    wb_motor_set_velocity(wheel_B,-1);
    wb_motor_set_velocity(wheel_C,-1);
  }


int main(int argc, char **argv)
{

  wb_robot_init();
  wb_keyboard_enable(TIME_STEP);


  WbDeviceTag wheel_A = wb_robot_get_device("motor_A");
  WbDeviceTag wheel_B = wb_robot_get_device("motor_B");
  WbDeviceTag wheel_C = wb_robot_get_device("motor_C");

  WbDeviceTag ds_left = wb_robot_get_device("distance_sensor_left");
  WbDeviceTag ds_right = wb_robot_get_device("distance_sensor_left");

  WbDeviceTag A_pos = wb_robot_get_device("position_sensor_A");
  WbDeviceTag B_pos = wb_robot_get_device("position_sensor_B");
  WbDeviceTag C_pos = wb_robot_get_device("position_sensor_C");

  wb_position_sensor_enable(A_pos, TIME_STEP);
  wb_position_sensor_enable(B_pos, TIME_STEP);
  wb_position_sensor_enable(C_pos, TIME_STEP);

  wb_distance_sensor_enable(ds_left, TIME_STEP);
  wb_distance_sensor_enable(ds_right, TIME_STEP);

  void automatic(){
     WbDeviceTag ds_r1 = wb_robot_get_device("distance_sensor1");
     WbDeviceTag ds_r2 = wb_robot_get_device("distance_sensor2");

     wb_distance_sensor_enable(ds_r1, TIME_STEP);
     wb_distance_sensor_enable(ds_r2, TIME_STEP);

     ds_1=(wb_distance_sensor_get_value(ds_left)*0.2)/65535;
     ds_2=(wb_distance_sensor_get_value(ds_right)*0.2)/65535;


     goAuto( wheel_A, wheel_B, wheel_C);

     if (ds_1<= 0.17 && ds_1<ds_2){
       aproach++;
     }
     if (aproach>=1 && aproach<=65){
     turnlAuto( wheel_A, wheel_B, wheel_C);
     aproach++;
     }
     else {
      aproach=0;
     }

     if (ds_2<0.17 && ds_2<ds_1){
       aproach2++;
     }

     if(aproach2 >=1 && aproach2<=65){
     turnrAuto( wheel_A, wheel_B, wheel_C);
     aproach2++;
     }
     else{
     aproach2=0;
     }
   }
  void manual(){

        if(key == WB_KEYBOARD_UP){
          goRobot( wheel_A, wheel_B, wheel_C);


        }
        else if(key == WB_KEYBOARD_DOWN){
          downRobot( wheel_A, wheel_B, wheel_C);

        }
        else if(key == WB_KEYBOARD_LEFT){
          leftRobot(wheel_A,wheel_B,wheel_C);

        }
        else if(key == WB_KEYBOARD_RIGHT){
          rightRobot(wheel_A,wheel_B,wheel_C);

        }
        else if(key == 'S' ){
          noise = n1 + 0.785398; //.75 = 45 degrees to the left
          turn_l = 1;
        }

        else if(turn_l == 1){

          if(n1 <= noise){
          turnLeft(wheel_A,wheel_B,wheel_C);
        }
        else{
          stopRobot(wheel_A,wheel_B,wheel_C);

          turn_l = 0;
        }

      }

        else if(key == 'A' ){
        noise = n1 - 0.785398; // 45 degrees to the right
        turn_r = 1;

        }
         else if(turn_r == 1){

          if(n1 >= noise){
          turnRight(wheel_A,wheel_B,wheel_C);

        }
        else{
          stopRobot(wheel_A,wheel_B,wheel_C);

          turn_r = 0;
        }

        }
        else{
          stopRobot(wheel_A,wheel_B,wheel_C);
        }
   }

  while (wb_robot_step(TIME_STEP) != -1) {



  wb_position_sensor_get_value(A_pos);
  wb_position_sensor_get_value(B_pos);
  wb_position_sensor_get_value(C_pos);

  A_pos_1 = wb_position_sensor_get_value(A_pos);
  A_pos_dif = A_pos_1 - A_pos_2;
  A_pos_2 = A_pos_1;
  A_angv = A_pos_dif/0.064;
  A_linv = A_angv*radio;
  A_rpm = -1*A_angv*(60/(2*PI));

  B_pos_1 = wb_position_sensor_get_value(B_pos);
  B_pos_dif = B_pos_1 - B_pos_2;
  B_pos_2 = B_pos_1;
  B_angv = B_pos_dif/0.064;
  B_linv = B_angv*radio;
  B_rpm = -1*B_angv*(60/(2*PI));

  C_pos_1 = wb_position_sensor_get_value(C_pos);
  C_pos_dif = C_pos_1 - C_pos_2;
  C_pos_2 = C_pos_1;
  C_angv = C_pos_dif/0.064;
  C_linv = C_angv*radio;
  C_rpm = -1*(C_angv*(60/(2*PI)));

  dis_value_left = wb_distance_sensor_get_value(ds_left);
  dis_value_right = wb_distance_sensor_get_value(ds_right);
  value_cm_left = ((dis_value_left *.2)/65535);
  value_cm_right = ((dis_value_right *.2)/65535);

  printf("Distancia left: %.2f  \n", value_cm_left);
  printf("Distancia right: %.2f  \n", value_cm_right);

/////////////////////Shift//////////////////

  key = wb_keyboard_get_key();

  if(key == 'W'){
    m = 1;
    a = 0;
  printf("Manual mode \n");
  }

  if (key == 'G'){
    m = 0;
    a = 1;
    printf("Automatic mode \n");
  }

  if(m == 1){
    manual();
  }
  if(a == 1){
    automatic();
}


};

  wb_robot_cleanup();

  return 0;
}
