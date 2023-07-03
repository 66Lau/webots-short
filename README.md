# webots-short
---
## Introduction 

In this project, I have developed a balanced legged robot capable of jumping. The mechanical model of the robot is modified from **ARX-4**, an exemplary open-source design,my primary contribution lies in empowering the robot to accomplish a range of tasks, including jumping, maintaining balance, and fluid locomotion.


---
## Directory
* [1. 功能展示](#1功能展示)
* [2. code](#2code)
* [3. TBD](#2TBD)

---

## 1.功能展示
 <center>


<img src = "image/综合展示3.gif" alt = "综合展示3.gif" width = "200">

 </center>

### 1.1 平衡
 <center>

<!-- ![描述1](image/平衡.png) -->
<img src = "image/平衡.png" alt = "image/平衡.png" width = "200">

 </center>

### 1.2 轮腿解算
 <center>

<!-- ![描述1](image/轮腿倒立摆角度解算.png)
![描述1](image/轮腿倒立摆角度解算2.png) -->
<img src = "image/轮腿倒立摆角度解算.png" alt = "image/轮腿倒立摆角度解算.png" width = "200">  


<img src = "image/轮腿倒立摆角度解算2.png" alt = "image/轮腿倒立摆角度解算2.png" width = "200">

 </center>

### 1.3 高度控制
 <center>

<!-- ![描述1](image/高度控制.png) -->
<img src = "image/高度控制.png" alt = "image/高度控制.png" width = "200">

 </center>

### 1.4 roll轴平衡
 <center>

<!-- ![描述1](image/roll轴平衡.png) -->
<img src = "image/roll轴平衡.png" alt = "image/高度控制.png" width = "200">

 </center>

---
## 2.code

 <center>

|代码    |  功能   |
| ------- | ------ |
|app |软件层
|bsp |硬件层
|math |数学算法层

|功能   |  键盘操控   |
| ------- | ------ |
|前进/后退 |w/s
|左转/右转 |a/d
|跳跃 |k
|升高机体高度|h
降低机体高度|j


 </center>

 **NOTE** ： The basicTimeStep field defines the duration of the simulation step executed by Webots. It is a floating point value expressed in milliseconds where the minimum value is 1. Setting this field to a high value will accelerate the simulation, but will decrease the accuracy and the stability, especially for physics computations and collision detection. It is usually recommended to tune this value in order to find a suitable speed/accuracy trade-off.

 如果出现轮子下陷或者震动过大的情况，请尝试在worldinfo中降低basicTimeStep，具体的数值可能取决于不同的电脑-_-, 过低的basicTimeStep也并不好，可能会运行过慢或者出现一些还没发现的问题，调节basicTimeStep需要一些耐心找到较为平衡的值

---
## 3.TBD
 <center>

|功能与算法      |  状态   |
| ------- | ------ |
|直立平衡环 |已完成
|轮腿解算 |已完成
|键盘控制 |已完成
|前进后退 |已完成
|差速旋转 |已完成
|高度调节 |已完成
|roll轴平衡环 |已完成
|原地跳跃 |已完成
|整车速度环 |已完成
|飞坡 |待更新

**位控功能初步实现后考虑力控实现**



 </center>