# Drone
4/15 save the current code

4/16
移上了JudgeReceive.c和uart4.c，发现有一些数据和函数的定义缺少，发现是main.h中缺乏include "algorithmOfCRC.h"
移上了Graph.c和.h
缺少DataSendTask.c,
缺乏功率控制相关函数，因为无人机功率控制与底盘不同，暂时将其注释解决
出现大量报错：..\MyTask\JudgeReceive_Task.c(161): error:  #167: argument of type "uint16_t *" is incompatible with parameter of type "uint32_t *"
..\MyTask\JudgeReceive_Task.c(161): error:  #167: argument of type "uint16_t *" is incompatible with parameter of type "uint32_t *"
以上两个报错可以通过强制类型转换解决

4/18 合并了裁判系统和原来的老uart文件，现在主要处理JudgeReceive.c
