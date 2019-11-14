# LEGO-LOAM代码解析

### utility.h

定义了一些普适的东西，如雷达型号、点云分割参数、PointType数据结构、

第二个是将分割的点云发送到特征提取模块。第三个，激光雷达里程计使用从上一个模块中提取的特征找到在连续扫描过程中机器人位姿的转换，这些信息最终用于激光雷达用点云方式的建图中。第五个模块是融合激光雷达里程测量和建图的姿态估计结果，并输出最终的姿态估计。

### 1、点云分割

目的：将单个扫描的点云投影到一个固定范围的图像上进行分割。

1. 创建点云及初始化一系列参数

2. imageProjecion()构造函数的内容如下：

   1. 订阅话题：订阅来自velodyne雷达驱动的topic
      - `"/velodyne_points"`(`sensor_msgs::PointCloud2`)，订阅的subscriber是`subLaserCloud`。
   2. 发布话题，这些topic有：
     - `"/full_cloud_projected"`(`sensor_msgs::PointCloud2`)  fullCloud - > 完整点云
     - `"/full_cloud_info"`(`sensor_msgs::PointCloud2`)  fullInfoCloud -> 完整点云，比上面多一个intensity信息，存的range
     - `"/ground_cloud"`(`sensor_msgs::PointCloud2`)  groundCloud -> 地面点云
     - `"/segmented_cloud"`(`sensor_msgs::PointCloud2`)  segmentedCloud -> 聚类之后的点云，去除了大部分点，如果是地面点,对于列数不为5的倍数的，直接跳过不处理
     - `"/segmented_cloud_pure"`(`sensor_msgs::PointCloud2`) segmentedCloudPure -> 需要选择不是地面点(labelMat[i][j]!=-1)和没被舍弃的点
     - `"/segmented_cloud_info"`(`cloud_msgs::cloud_info`) segMsg -> 包含更多信息的segmentedCloudPure
     - `"/outlier_cloud"`(`sensor_msgs::PointCloud2`)  outlierCloud -> 界面点，当列数为5的倍数，并且行数较大，可以认为非地面点的
3. 自创的rosmsg来表示点云信息：

   ```cloud_msgs::cloud_info segMsg;```

4. 回调函数ImageProjection::cloudHandler

   1. `copyPointCloud(laserCloudMsg)`  点云复制

   2. `findStartEndAngle()`   计算雷达转过的角度

      > 1. 雷达坐标系：右->X,前->Y,上->Z  和右手定则相反
      >
      > 2. 雷达内部旋转扫描方向：Z轴俯视下来，顺时针方向（Z轴右手定则反向）
      >
      > 一圈为2*PI 
      >
      > 3. 找到始末角度差   segMsg.orientationDiff

   3. `projectPointCloud()`     点云投影 

      > 1. 将激光雷达得到的数据看成一个16x1800的点云阵列。然后根据每个点云返回的XYZ数据将他们对应到这个阵列里去。
      >
      > 2. fullInfoCloud->points[index].intensity = range;    //range是点到雷达的距离
      >
      > ​       rangeMat = range
      >
      > 3. 分别给fullCloud以及fullInfoCloud赋值了

    4. `groundRemoval()`  地面检测

       > 1. 由上下两线之间点的XYZ位置得到两线（找1-7之间的线）之间的俯仰角，如果俯仰角在10度以内，则判定(i,j)和(i,j+1)为地面点,`groundMat[i][j]=1`，否则，则不是地面点，进行后续操作；
       >
       > 2. 更新groundMat地面标志位、更新labelMat标志位，找到所有点中的地面点或者距离为FLT_MAX(rangeMat的初始值)的点，并将他们标记为-1
       >
       > 3. 若需要发布地面点，则给groundCloud赋值

     5. `cloudSegmentation()`  点云分割

        > 1. 先聚类：labelComponents(row, col)：采用BFS判断该点是否是这此平面的点
        >
        > 2. 聚类的规则是：
        >    1.如果聚类超过30个点，直接标记为一个可用聚类，labelCount需要递增；
        >    2.如果聚类点数小于30大于等于5，统计竖直方向上的聚类点数
        >    3.竖直方向上超过3个也将它标记为有效聚类
        >    4.标记为999999的是需要舍弃的聚类的点，因为他们的数量小于30个
        >
        >    最终会修改labelMat的值
        >
        > 3. 分类完成后，找到可用的特征点或者地面点(不选择labelMat[i][j]=0的点)，按照它的标签值进行判断，将部分界外点放进`outlierCloud`中
        >
        > 4. 然后将大部分地面点去掉，只选可用特征点或者地面点，剩下的那些点进行信息的拷贝与保存操作。
        >
        > 5. 最后如果有节点订阅`SegmentedCloudPure`，那么把点云数据保存到`segmentedCloudPure`中去。
        >
    5. `publishCloud()` 发布各类点云的内容
    6. `resetParameters()` 重置参数内容
### 2、特征关联

目的：进行特征关联的过程。

FeatureAssociation()构造函数的内容如下：

1. 订阅话题:
   - `"/segmented_cloud"`(`sensor_msgs::PointCloud2`)，数据处理函数[`laserCloudHandler`](https://wykxwyc.github.io/2019/01/24/LeGO-LOAM-code-review-featureAssociation/#laserCloudHandler)
   - `"/segmented_cloud_info"`(`cloud_msgs::cloud_info`)，数据处理函数[`laserCloudInfoHandler`](https://wykxwyc.github.io/2019/01/24/LeGO-LOAM-code-review-featureAssociation/#laserCloudInfoHandler)
   - `"/outlier_cloud"`(`sensor_msgs::PointCloud2`)，数据处理函数[`outlierCloudHandler`](https://wykxwyc.github.io/2019/01/24/LeGO-LOAM-code-review-featureAssociation/#outlierCloudHandler)
   - `imuTopic = "/imu/data"`(`sensor_msgs::Imu`)，数据处理函数[`imuHandler`](https://wykxwyc.github.io/2019/01/24/LeGO-LOAM-code-review-featureAssociation/#imuHandler)

2. 发布话题，这些topic有：
   - `"/laser_cloud_sharp"`(`sensor_msgs::PointCloud2`)
   - `"/laser_cloud_less_sharp"`(`sensor_msgs::PointCloud2`)
   - `"/laser_cloud_flat"`(`sensor_msgs::PointCloud2`)
   - `"/laser_cloud_less_flat"`(`sensor_msgs::PointCloud2`)
   - `"/laser_cloud_corner_last"`(`sensor_msgs::PointCloud2`)
   - `"/laser_cloud_surf_last"`(`cloud_msgs::cloud_info`)
   - `"/outlier_cloud_last"`(`sensor_msgs::PointCloud2`)
   - `"/laser_odom_to_init"`(`nav_msgs::Odometry`)

3. 初始化各类参数

4. > 第一个回调函数`laserCloudHandler`存储segmentedCloud点云数据的时间戳，将点云数据从ROS定义的格式转化到pcl的格式。
   >
   > 第二个回调函数`laserCloudInfoHandler`存储segmentedCloudInfo点云数据时间戳，将点云数据从ROS定义的格式转化到pcl的格式。
   >
   > 第三个回调函数`outlierCloudHandler`存储outlierCloud点云数据时间戳，将点云数据从ROS定义的格式转化到pcl的格式。
   >

5. 第四个回调函数`imuHandler`，与LOAM中的一样，函数实现如下：

   1. 通过接收到的imuIn里面的四元素得到roll,pitch,yaw三个角；
   2. 对加速度进行坐标变换(`关于坐标变换这一块不够清楚`)；
   3. 设置imuPointerLast，范围为[0-199]；
   4. 将欧拉角，加速度，速度保存到循环队列中；
   5. 对速度，角速度，加速度进行积分，得到位移，角度和速度(`AccumulateIMUShiftAndRotation()`)；

6. **runFeatureAssociation**，该类最主要的函数，步骤如下：

   1. 如果有新数据进来则执行，否则不执行任何操作；根据时间差以及标志位

   2. `adjustDistortion()` 主要进行的处理是将点云数据进行坐标变换，进行插补等工作

      >1. 给每个点云找到对应的imu数据（时间对齐）
      >
      >   * 不能进行线性插值情况：最晚的IMU数据都比点云数据早
      >
      >   * 可以进行插值：IMU数据比点云晚
      >
      >2. 更新i=0时刻PRY角，0时刻为该片点云第一个点
      >3. 速度投影到初始i=0时刻，`VeloToStartIMU`没看懂。
      >4. 将点的坐标变换到初始i=0时刻，`TransformToStartIMU`没看懂，更新point点坐标。

   3. `calculateSmoothness`进行光滑性计算，得到一个平滑过的cloudSmoothness，其中的range为该点周边10个点的中和值的平方。

   4. `markOccludedPoints`标记阻塞点，指在点云中可能出现的互相遮挡的情况（过近的点），将距离变化的点cloudNeighborPicked标记为1，并将周围的5个点也标记为1

   5. `extractFeatures`特征抽取，保存到不同的队列是不同类型的点云，进行了标记的工作，这一步中减少了点云数量，使计算量减少。

   6. `publishCloud`发布各种类型的点云。











