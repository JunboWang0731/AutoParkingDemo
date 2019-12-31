# 自动泊车项目——车
> 侯宇轩

## 概述

使用FreeRTOS，调度任务，操作系统运行时，主板上红灯1s闪烁一次。

## 外设及io

| 模块 | 片上外设 | IO | 功能 | 源码文件 |
| ---- | ---- | ---- | ---- | ---- |
| CH340 | USART1 | PA9,PA10 | 下载程序，和电脑通信 | /SYSTEM/usart.c,/USER/printf.c |
| 蓝牙 | USART3 | PB10,PB11 | 发数据给上位机 | /SYSTEM/usart.c,/USER/printf.c |
| 电机编码器 | TIM2,TIM3 | PA0,PA1,PA6,PA7 | 测速，转速闭环 | /HARDWARE/Motor.c,/USER/moter.c |
| 电机pwm | TIM4 | PB6,PB7,PB8,PB9 | 驱动电机 | /HARDWARE/encoder.c,/USER/moter.c |
| 舵机pwm | TIM1 | PA11| 驱动舵机 | /HARDWARE/Motor.c,/USER/moter.c |
| SW调试 | SW | PA13,PA14 | 下载程序，调试 |  |
| OLED显示屏 | GPIO | PB5,PB4,PB3,PA15 | 显示 | /USER/OLED_task.c |
| LED | GPIO | PA4 |指示灯 |/HARDWARE/led.c |
| 蜂鸣器 | GPIO | PA8 |蜂鸣器 |/HARDWARE/beep.c |
| 电压检测 | ADC1 | PB1 | 检测电源电压 | /HARDWARE/adc.c |

## 遥控上位机地址

上位机： [SBHelper](https://github.com/IronSublimate/serialport-bluetooth-helper) 

## 注意事项

1. uart.py为遥控的测试脚本
2. 舵机方向反复切换的时候，蓝牙可能供电不够会重启，就有几秒钟连不上

