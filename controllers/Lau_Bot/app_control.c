#include "app_control.h"

KEYBOARD keyboard;

void remote_keyboard_init()
{
    wb_keyboard_enable(1);

}

void remote_keyboard_control(){
  keyboard.key_get = -1;
  keyboard.key_get = wb_keyboard_get_key();
  keyboard.velocity_forward = 0;
  keyboard.velocity_backward = 0;
  keyboard.velocity_turnright = 0;
  keyboard.velocity_turnleft = 0;
  keyboard.vertical_up = 0;
  keyboard.vertical_down = 0;
  keyboard.jump = 0;


  if (keyboard.key_get != -1) {
    //printf("获得键盘输入\n");
    switch (keyboard.key_get) {
      case 'W'://前进
        keyboard.velocity_forward = 1;
        break;
      case 'S'://后退
        keyboard.velocity_backward = 1;
        break;
      case 'D'://右转
        keyboard.velocity_turnright = 1;
        break;
      case 'A'://左转
        keyboard.velocity_turnleft = 1;
        break;
      case 'H'://抬升
        keyboard.vertical_up = 1;
        break;
      case 'J'://降落
        keyboard.vertical_down = 1;
        break;
      case 'K'://跳跃
        keyboard.jump = 1;
        break;
      default:
        break;
     }
   }
   else{
        keyboard.key_get = wb_keyboard_get_key();
        keyboard.velocity_forward = 0;
        keyboard.velocity_backward = 0;
        keyboard.velocity_turnright = 0;
        keyboard.velocity_turnleft = 0;
        keyboard.vertical_up = 0;
        keyboard.vertical_down = 0;
   }

}; 