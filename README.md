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

不要在意背景中的音乐盒和各种凌乱的杜邦线……
Please ignore the background and the messy wires...

### 关于指针位置的负反馈 About Negative Feedback of the Pointer Position

负反馈通过在指针上加装47K电位器，并将模拟电位线性转换为位置信息实现。电位器共可转10圈，电压范围在0-5V范围内变化。实际的位置和电平对应关系通过测量获得。
STM32F103单片机的一部分引脚可以接受模拟电平输入。本系统中设定PA1为模拟信号输入引脚。

### 关于电机控制 About Motor Control

电机控制通过向L298N发送PWM波实现。本系统由中PA6和PA7引脚输出0或VCC的数字信号。其中PA6为对照信号，当正转时，PA6为低电平；当反转时，PA6为高电平。
PA7引脚输出经过调制的、占空比一定的数字信号。当正转时，PA7对PA6形成一定占空比的电压差，通过L298N模块驱使电机转动；反转时相反。

### 关于显示 About Display

根据单片机实际使用的引脚不同，LCD1602模块所需要的驱动程序也有不同。对照表：

The drive program for LCD1602 varies according to the actual pins allocated for it. Pin table from STM32 to LCD1602:

PG1 --> EN

PF13 --> RW

PF15 --> RS

PF0 ~ PF7 --> D0 ~ D7
