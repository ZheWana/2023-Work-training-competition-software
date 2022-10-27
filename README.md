# 2023工训赛软件工程

#### 介绍

2023工训赛控制器软件工程，含有内容：
以STM32F401CCUI6为核心的步进电机驱动；
STM32H750VBT6为核心的控制器；

（**TODO**：视觉部分工程 & 硬件工程）

#### 软件架构与外设

步进电机驱动采用简单的裸机架构，使用外设如下：
* **GPIO**：调试用途；
* **硬件UART**：调试用途；
* **硬件SPI**：通过自拟的通讯协议接收控制器的指令；
* **定时器**：使用**PWM+DMA双缓冲区**的方式控制步进脉冲的输出；



控制器预计使用FreeRTOS架构，使用外设如下：

* （TODO：控制器外设）

#### 开发环境

步进电机驱动采用Keil+VScode（EDIE）的方式进行开发；

控制器预计使用Clion进行开发。