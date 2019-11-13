# LEGO-LOAM代码解析

### utility.h

定义了一些普适的东西，如雷达型号、点云分割参数、PointType数据结构、

第二个是将分割的点云发送到特征提取模块。第三个，激光雷达里程计使用从上一个模块中提取的特征找到在连续扫描过程中机器人位姿的转换，这些信息最终用于激光雷达用点云方式的建图中。第五个模块是融合激光雷达里程测量和建图的姿态估计结果，并输出最终的姿态估计。

### 1、点云分割

目的：将单个扫描的点云投影到一个固定范围的图像上进行分割。

1. 创建点云及初始化一系列参数

2. 订阅topic

3. 自创的rosmsg来表示点云信息：

   ```cloud_msgs::cloud_info segMsg;```

4. 回调函数ImageProjection::cloudHandler

   1. `copyPointCloud(laserCloudMsg); // 点云复制`

   2. `findStartEndAngle();  // 寻找始末角度`

      // 雷达坐标系：右->X,前->Y,上->Z  和右手定则相反

      // 雷达内部旋转扫描方向：Z轴俯视下来，顺时针方向（Z轴右手定则反向）

      // 一圈为2*PI 

      // 找到始末角度差   segMsg.orientationDiff

   3. `projectPointCloud()     //  点云投影` 
      // 
5. 










