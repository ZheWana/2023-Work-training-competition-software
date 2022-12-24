# 2023工训赛软件工程

#### 介绍

2023工训赛控制器软件工程，含有内容：
STM32H750VBT6为核心的控制器；

（**TODO**：视觉部分工程 & 硬件工程）

#### 软件架构与外设

控制器预计使用FreeRTOS架构，使用外设如下：

|        外设        |    功能     |
| :----------------: | :---------: |
| TIM1,3,4,8 (CH1&2) |   编码器    |
|  TIM2 (CH1&2&3&4)  | 4路有刷电机 |
|    TIM16 (CH1)     |  步进电机   |
|       UART8        |  总线舵机   |
|       UART5        |    调试     |
|      UART4&7       |    预留     |
|        SPI4        |    屏幕     |
|        SPI3        |     ICM     |
|        I2C1        |     QMC     |

#### 开发环境

控制器预计使用Clion+Keil进行开发。