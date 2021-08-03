# STM32_Motor_Control_System

基于STM32F103的电机控制系统，任务是控制电机旋转其上的指针到给定的刻度。有键盘和液晶显示屏以输入和显示给定的参数。
这个项目很简陋，是本人在校课程的一部分。希望可以为单片机初学者带来一定的灵感和指导。

A motor control system with STM32F103, drives a pointer connected to the motor to the required degree. A keyboard and an LCD is attached to import and read the degrees.
This is a immature system with very simple functions. It is a part of my curriculum projects. Hope it can provide some knowledge for beginners in MCUs.

使用正点原子的精英版开发板产品，系统基于库函数编写。LCD1602模块的部分驱动来自网上。本系统绝大部分代码在main.c文件中，包括显示屏、键盘、电机模块的驱动程序。

The development board is an ALIENTEK production. Most of the code is in main.c file, including the drive program for the keyboard, LCD and motor module.

### 概述 General description

系统包括一个STM32F103单片机，一个自制键盘，一个LCD1602液晶显示模块，以及一个L298N电机驱动模块。电机与一个指针相连，指针被安装在一个0-360度的表盘上。
系统可以从键盘获得参数，通过LCD显示，并控制电机带动指针旋转至给定的度数。

The system includes an STM32F103 single chip microcomputer, a self-made keyboard, an LCD1602 display screen, and an L298N motor drive module.
The motor is connected with a pointer, which is on a dial plate of 0-360 degrees.
The system acquires parameters from the keyboard, displays them on the LCD, and controls the motor to drive the pointer to the given degree.

Global view:
![global view](https://github.com/jasonall-cauc/STM32_Motor_Control_System/blob/main/Global.png)


