# webots-short
---
## Introduction 
The simulation of jump-able legged which modified from the open source model of ARX, the main work is the controller design


---
## Directory
* [1. 功能展示](#1功能展示)
* [2. code](#2code)
* [3. TBD](#2TBD)

---

## 1.功能展示
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

 </center>

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
|原地跳跃 |初步实验已完成，待改进
|整车速度环 |待更新（目前直接控制重心角度）
|飞坡 |待更新

**位控功能初步实现后考虑力控实现**



 </center>