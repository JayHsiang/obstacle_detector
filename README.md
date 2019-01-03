# The obstacle_detector package  包obstacle_detector

The `obstacle_detector` package provides utilities to detect and track obstacles from data provided by 2D laser scanners. Detected obstacles come in a form of line segments or circles. The package was designed for a robot equipped with two laser scanners therefore it contains several additional utilities. The working principles of the method are described in an article provided in the `resources` folder.

obstacle_detector包提供了用于检测和跟踪由2D激光扫描仪提供数据的障碍的实用程序。检测到的障碍物以线段或圆圈的形式呈现。该包设计用于配备两个激光扫描仪的机器人，因此它包含几个额外的实用程序。resources文件夹中提供的文章描述了该方法的工作原理。

The package requires [Armadillo C++](http://arma.sourceforge.net) library for compilation and runtime.

该软件包需要Armadillo C++库进行编译和运行。

-----------------------
<p align="center">
  <img src="https://user-images.githubusercontent.com/1482514/27595825-0abe4338-5b5e-11e7-8438-ffdeec4e9cef.png" alt="Visual example of obstacle detector output."/>
  <br/>
  Fig. 1. Visual example of obstacle detector output.
</p>

-----------------------

1. The nodes and nodelets
  1.1 The scans_merger 
  1.2 The obstacle_extractor 
  1.3 The obstacle_tracker 
  1.4 The obstacle_publisher
2. The messages
3. Launch files
4. The displays

## 1. The nodes and nodelets  节点和nodelets

The package provides separate nodes/nodelets to perform separate tasks. In general solution the data is processed in a following manner:

该包提供单独的nodes/nodelets来执行单独的任务。在一般解决方案中，以下列方式处理数据：

`two laser scans` -> `scans merger` -> `merged scan or pcl` -> `obstacle extractor` -> `obstacles` -> `obstacle tracker` -> `refined obstacles`

For some scenarios the pure obstacle extraction directly from a laser scan (without tracking) might be sufficient.

对于某些情况，直接从激光扫描（没有跟踪）的纯障碍物提取可能就足够了。

The nodes are configurable with the use of ROS parameter server. All of the nodes provide a private `params` service, which allows the process to get the latest parameters from the parameter server. 

可以使用ROS参数服务器配置节点。所有节点都提供私有`params`服务，允许进程从参数服务器获取最新参数。

All of the nodes can be in either active or sleep mode, triggered by setting the appropriate variable in the parameter server and calling `params` service. In the sleep mode, any subscribers or publishers are shut down and the node does nothing until activated again.

所有节点都可以处于活动或睡眠模式，通过在参数服务器中设置适当的变量并调用`params`服务来触发。在睡眠模式下，任何订阅者或发布者都会关闭，节点什么都不做直到再次激活。

For the ease of use it is recomended to use appropriate Rviz panels provided for the nodes with the package. The Rviz panels communicate via parameter server and service-client calls, therefore the names of the nodes must be preserved unchanged (cf. launch files for examples).

为了便于使用，建议使用为包中节点提供的合适的Rviz面板。Rviz面板通过参数服务器和服务-客户端调用进行通信，因此必须保持节点的名称不变（参见示例的启动文件）。

### 1.1. The scans_merger node       节点scans_merger

This node converts two messages of type `sensor_msgs/LaserScan` from topics `front_scan` and `rear_scan` into a single laser scan of the same type, published under topic `scan` and/or a point cloud of type `sensor_msgs/PointCloud`, published under topic `pcl`. The difference between both is that the resulting laser scan divides the area into finite number of circular sectors and put one point (or actually one range value) in each section occupied by some measured points, whereas the resulting point cloud simply copies all of the points obtained from sensors.

该节点将两个类型为`sensor_msgs/LaserScan`的消息从主题`front_scan`和`rear_scan`转换为在主题`scan`下发布的相同类型的单个激光扫描，和/或在主题`pcl`下发布的类型为`sensor_msgs/PointCloud`的点云中。两者之间的区别在于，所得到的激光扫描将区域划分为有限数量的圆形扇区，并且在由一些测量点占据的每个区域中放置一个点（或实际上一个范围值），而得到的点云简单地复制从传感器获得的所有点。

The input laser scans are firstly rectified to incorporate the motion of the scanner in time (see `laser_geometry` package). Next, two PCLs obtained from the previous step are synchronized and transformed into the target coordinate frame at the current point in time.

首先对输入激光扫描进行校正，以便及时合并扫描仪的运动（参见`laser_geometry`包）。接下来，将从前一步骤获得的两个PCL同步并转换为当前时间点的目标坐标系。

-----------------------
<p align="center">
  <img src="https://user-images.githubusercontent.com/1482514/27595821-0aa3519a-5b5e-11e7-9ce0-8f48db4592e4.gif" alt="Visual example of scans_merger output"/>
  <br/>
  Fig. 2. Visual example of `scans_merger` output.
</p>

-----------------------

The resulting messages contain geometric data described with respect to a specific coordinate frame (e.g. `robot`). Assuming that the coordinate frames attached to two laser scanners are called `front_scanner` and `rear_scanner`, both transformation from `robot` frame to `front_scanner` frame and from `robot` frame to `rear_scanner` frame must be provided. The node allows to artificially restrict measured points to some rectangular region around the `robot` frame as well as to limit the resulting laser scan range. The points falling behind this region will be discarded.

得到的消息包含关于特定坐标系（例如`robot`）描述的几何数据。假设连接到两个激光扫描仪的坐标系称为`front_scanner`和`rear_scanner`，则必须提供从`robot`系到`front_scanner`系和从`robot`系到`rear_scanner`系的转换。该节点允许人为地将测量点限制在`robot`系周围的某些矩形区域，以及限制所产生的激光扫描范围。在该区域外的点将被丢弃。

Even if only one laser scanner is used, the node can be useful for simple data pre-processing, e.g. rectification, range restriction or recalculation of points to a different coordinate frame. The node uses the following set of local parameters:

即使仅使用一个激光扫描仪，该节点也可用于简单的数据预处理，例如纠正、范围限制或将点重新计算到不同的坐标系。该节点使用以下一组本地参数：

* `~active` (`bool`, default: `true`) - active/sleep mode, 活动/睡眠模式
* `~publish_scan` (`bool`, default: `false`) - publish the merged laser scan message, 发布合并的激光扫描消息
* `~publish_pcl` (`bool`, default: `true`) - publish the merged point cloud message, 发布合并的点云消息
* `~ranges_num` (`int`, default: `1000`) - number of ranges (circular sectors) contained in the 360 deg laser scan message, 360度激光扫描消息中包含的范围数（圆形扇区）
* `~min_scanner_range` (`double`, default: `0.05`) - minimal allowable range value for produced laser scan message, 生成的激光扫描消息的最小允许范围值
* `~max_scanner_range` (`double`, default: `10.0`) - maximal allowable range value for produced laser scan message, 生成的激光扫描消息的最大允许范围值
* `~min_x_range` (`double`, default: `-10.0`) - limitation for points coordinates (points with coordinates behind these limitations will be discarded), 点坐标的限制（这些限制之外的坐标点将被丢弃）
* `~max_x_range` (`double`, default: `10.0`) - as above,
* `~min_y_range` (`double`, default: `-10.0`) - as above,
* `~max_y_range` (`double`, default: `10.0`) - as above,
* `~fixed_frame_id` (`string`, default: `map`) - name of the fixed coordinate frame used for scan rectification in time, 用于扫描纠正的固定坐标系的名称
* `~target_frame_id` (`string`, default: `robot`) - name of the coordinate frame used as the origin for the produced laser scan or point cloud. 用作生成的激光扫描或点云的原点的坐标系的名称

The package comes with Rviz panel for this node.

该程序包随附Rviz面板。

-----------------------
<p align="center">
  <img src="https://user-images.githubusercontent.com/1482514/28522255-97e2c210-7077-11e7-8ca1-556790b043e6.png" alt="Rviz panel for the scans_merger node."/>
  <br/>
  Fig. 3. Rviz panel for the `scans_merger` node.
</p>

-----------------------

### 1.2. The obstacle_extractor node   节点obstacle_extractor

This node converts messages of type `sensor_msgs/LaserScan` from topic `scan` or messages of type `sensor_msgs/PointCloud` from topic `pcl` into obstacles which are published as messages of custom type `obstacles_detector/Obstacles` under topic `raw_obstacles`. The PCL message must be ordered in the angular fashion, because the algorithm exploits the polar nature of laser scanners.

此节点将主题为`scan`的类型为`sensor_msgs/LaserScan`的消息或主题为“pcl”的类型为`sensor_msgs/PointCloud`的消息转换为障碍物，这些障碍物在主题`raw_obstacles`下发布为自定义类型`obstacles_detector/Obstacles`的消息。必须以有角度的方式对PCL消息进行排序，因为该算法利用了激光扫描仪的极性。

-----------------------
<p align="center">
  <img src="https://user-images.githubusercontent.com/1482514/27595822-0aa50ab2-5b5e-11e7-8061-1da4b947b617.gif" alt="Visual example of obstacle_extractor output."/>
  <br/>
  Fig. 4. Visual example of `obstacle_extractor` output.
</p>

-----------------------

The input points are firstly grouped into subsets and marked as visible or not (if a group is in front of neighbouring groups, it is visible. Otherwise it is assumed to be occluded). The algorithm extracts segments from each points subset. Next, the segments are checked for possible merging between each other. The circular obstacles are then extracted from segments and also merged if possible. Resulting set of obstacles can be transformed to a dedicated coordinate frame.

首先将输入点分组为子集并标记为可见或不可见（如果一个组在相邻组的前面，则它是可见的，否则假定它被遮挡）。该算法从每个点子集中提取分段。接下来，检查分段之间可能的合并。然后从分段中提取圆形障碍物并且如果可能的话也进行合并。产生的障碍物集可以转换为专用坐标系。

The node is configurable with the following set of local parameters:

该节点可使用以下一组本地参数进行配置

* `~active` (`bool`, default: `true`) - active/sleep mode, 活动/睡眠模式
* `~use_scan` (`bool`, default: `false`) - use laser scan messages, 使用激光扫描信息
* `~use_pcl` (`bool`, default: `true`) - use point cloud messages (if both scan and pcl are chosen, scans will have priority), 使用点云消息（如果选择了scan和pcl，扫描将具有优先级）
* `~use_split_and_merge` (`bool`, default: `true`) - choose wether to use Iterative End Point Fit (false) or Split And Merge (true) algorithm to detect segments, 选择是使用End Point Fit(false)还是使用Split And Merge (true)算法来检测分段
* `~circles_from_visibles` (`bool`, default: `true`) - detect circular obstacles only from fully visible (not occluded) segments, 仅从完全可见（未遮挡）的分段中检测圆形障碍物
* `~discard_converted_segments` (`bool`, default: `true`) - do not publish segments, from which the circles were spawned, 不发布产生圆形的分段
* `~transform_coordinates` (`bool`, default: `true`) - transform the coordinates of obstacles to a frame described with `frame_id` parameter, 将障碍物的坐标转换为用`frame_id`参数描述的系
* `~min_group_points` (`int`, default: `5`) - minimum number of points comprising a group to be further processed, 构成要进一步处理的组的最小点数
* `~max_group_distance` (`double`, default: `0.1`) - if the distance between two points is greater than this value, start a new group, 如果两点之间的距离大于此值，则开启一个新组
* `~distance_proportion` (`double`, default: `0.00628`) - enlarge the allowable distance between points proportionally to the range of point (use scan angle increment in radians), 根据点的范围成比例地扩大点之间的允许距离（使用弧度的扫描角度增量）
* `~max_split_distance` (`double`, default: `0.2`) - if a point in group lays further from a leading line than this value, split the group, 如果组中的一个点比该值的前导线更远，则拆分该组
* `~max_merge_separation` (`double`, default: `0.2`) - if distance between obstacles is smaller than this value, consider merging them, 如果障碍物之间的距离小于此值，则考虑合并它们
* `~max_merge_spread` (`double`, default: `0.2`) - merge two segments if all of their extreme points lay closer to the leading line than this value, 如果它们的所有极值点都比这个值更接近引导线，则合并两个分段
* `~max_circle_radius` (`double`, default: `0.6`) - if a circle would have greater radius than this value, skip it, 如果圆的半径大于此值，则跳过它
* `~radius_enlargement` (`double`, default: `0.25`) - artificially enlarge the circles radius by this value, 通过此值人为地扩大圆半径
* `~frame_id` (`string`, default: `map`) - name of the coordinate frame used as origin for produced obstacles (used only if `transform_coordinates` flag is set to true). 用作生成障碍物原点的坐标系的名称（仅当`transform_coordinates`标志设置为true时使用）

The package comes with Rviz panel for this node.

该程序包随附Rviz面板。

-----------------------
<p align="center">
  <img src="https://user-images.githubusercontent.com/1482514/28522256-97e4d0be-7077-11e7-81fa-c2fcaae944be.png" alt="Rviz panel for the obstacle_extractor node."/>
  <br/>
  Fig. 5. Rviz panel for the `obstacle_extractor` node.
</p>

-----------------------

### 1.3. The obstacle_tracker node  节点obstacle_tracker

This node tracks and filters the circular obstacles with the use of Kalman filter. It subscribes to messages of custom type `obstacle_detector/Obstacles` from topic `raw_obstacles` and publishes messages of the same type under topic `tracked_obstacles`. The tracking algorithm is applied only to the circular obstacles, hence the segments list in the published message is simply a copy of the original segments. The tracked obstacles are supplemented with additional information on their velocity.

该节点使用卡尔曼滤波器跟踪和过滤圆形障碍物。它从主题`raw_obstacles`订阅自定义类型`obstacle_detector/Obstacles`的消息，并在主题`tracked_obstacles`下发布相同类型的消息。跟踪算法仅应用于圆形障碍物，因此发布消息中的片段列表只是原始片段的副本。跟踪的障碍物补充了关于其速度的附加信息。

-----------------------
<p align="center">
  <img src="https://user-images.githubusercontent.com/1482514/27595819-0aa1bc86-5b5e-11e7-80ed-90c9ea53f69f.gif" alt="Visual example of obstacle_tracker output."/>
  <br/>
  Fig. 6. Visual example of `obstacle_tracker` output.
</p>

-----------------------

The node works in a synchronous manner with the default rate of 100 Hz. If detected obstacles are published less often, the tracker will supersample them and smoothen their position and radius. The following set of local parameters can be used to tune the node:

节点以同步方式工作，默认速率为100Hz。如果较少发布检测到的障碍物，则跟踪器将对其进行超级采样并使其位置和半径平滑。下面的一组本地参数可用于调整节点：

* `~active` (`bool`, default: `true`) - active/sleep mode, 活动/睡眠模式
* `~copy_segments` (`bool`, default: `true`) - copy detected segments into tracked obstacles message, 将检测到的段复制到跟踪的障碍物消息中
* `~loop_rate` (`double`, default: `100.0`) - the main loop rate in Hz, 主循环速率，单位为Hz
* `~tracking_duration` (`double`, default: `2.0`) - the duration of obstacle tracking in the case of lack of incomming data (after this time from the last corresponding measurement the tracked obstacle will be removed from the list), 在缺少输入数据的情况下跟踪障碍物的持续时间（在此时间从最后一次相应的测量之后，跟踪的障碍物将从列表中移除）
* `~min_correspondence_cost` (`double`, default `0.3`) - a threshold for correspondence test, 通讯测试的阈值
* `~std_correspondence_dev` (`double`, default `0.15`) - (experimental) standard deviation of the position ellipse in the correspondence test, （实验）通讯测试中位置椭圆的标准偏差
* `~process_variance` (`double`, default `0.01`) - variance of obstacles position and radius (parameter of Kalman Filter)， 障碍物位置和半径的方差（卡尔曼滤波器的参数）
* `~process_rate_variance` (`double`, default `0.1`) - variance of rate of change of obstacles values (parameter of Kalman Filter), 障碍物值变化率的方差（卡尔曼滤波器参数）
* `~measurement_variance` (`double`, default `1.0`) - variance of measured obstacles values (parameter of Kalman Filter), 测量障碍物值的方差（卡尔曼滤波器参数）
* `~frame_id` (`string`, default: `map`) - name of the coordinate frame in which the obstacles are described, 描述障碍物的坐标系的名称

The package comes with Rviz panel for this node.

该程序包随附Rviz面板。

-----------------------
<p align="center">
  <img src="https://user-images.githubusercontent.com/1482514/27595823-0aa835a2-5b5e-11e7-9657-8cd129a50aa2.png" alt="Rviz panel for the obstacle_tracker node."/>
  <br/>
  Fig. 7. Rviz panel for the `obstacle_tracker` node.
</p>

-----------------------

### 1.4. The obstacle_publisher node  节点obstacle_publisher

The auxiliary node which allows to publish a set of virtual, circular obstacles in the form of message of type `obstacles_detector/Obstacles` under topic `obstacles`. The node is mostly used for off-line tests. The following parameters are used to configure the node:

辅助节点，允许以`obstacles`主题下的`obstacles_detector/Obstacles`类型的消息形式发布一组虚拟的圆形障碍物。该节点主要用于离线测试。以下参数用于配置节点：

* `~active` (`bool`, default: `true`) - active/sleep mode, 活动/睡眠模式
* `~reset` (`bool`, default: `false`) - reset time for obstacles motion calculation (used by dedicated Rviz panel), 重置障碍物运动计算的时间（由专用的Rviz面板使用）
* `~fusion_example` (`bool`, default: `false`) - produce obstacles showing fusion, 产生障碍物显示融合
* `~fission_example` (`bool`, default: `false`) - produce obstacles showing fission, 产生显示裂变的障碍物
* `~radius_margin` (`double`, default: `0.25`) - artificially enlarge the circles radius by this value in meters, 以米为单位人为地将圆半径放大
* `~loop_rate` (`double`, default: `10.0`) - the main loop rate in Hz, 主循环速率，单位为Hz
* `~frame_id` (`string`, default: `map`) - name of the coordinate frame in which the obstacles are described. 描述障碍物的坐标系的名称

The following parameters are used to provide the node with a set of obstacles:

以下参数用于为节点提供一组障碍物：

* `~x_vector` (`std::vector<double>`, default: `[]`) - the array of x-coordinates of obstacles center points, 障碍物中心点的x坐标数组
* `~y_vector` (`std::vector<double>`, default: `[]`) - the array of y-coordinates of obstacles center points, 障碍物中心点的y坐标数组
* `~r_vector` (`std::vector<double>`, default: `[]`) - the array of obstacles radii, 障碍物半径数组
* `~x_vector` (`std::vector<double>`, default: `[]`) - the array of velocities of obstacles center points in x direction, 障碍物中心点在x方向上的速度数组
* `~y_vector` (`std::vector<double>`, default: `[]`) - the array of velocities of obstacles center points in y direction.障碍物中心点在y方向上的速度数组

The package comes with Rviz panel for this node.

该程序包随附Rviz面板。

-----------------------
<p align="center">
  <img src="https://user-images.githubusercontent.com/1482514/28582779-f8de6fca-7166-11e7-8a52-e147cea89f87.png" alt="Rviz panel for the obstacle_publisher node."/>
  <br/>
  Fig. 8. Rviz panel for the `obstacle_publisher` node.
</p>

-----------------------

## 2. The messages   消息

The package provides three custom message types. All of their numerical values are provided in SI units.

该包提供三种自定义消息类型。它们的所有数值均以SI为单位提供。

* `CircleObstacle`
    - `geometry_msgs/Point center` - center of circular obstacle, 圆形障碍物的中心
    - `geometry_msgs/Vector3 velocity` - linear velocity of circular obstacle, 圆形障碍物的线速度
    - `float64 radius` - radius of circular obstacle with added safety margin, 增加安全边缘的圆形障碍物的半径
    - `float64 true_radius` - measured radius of obstacle without the safety margin. 没有安全边缘的障碍物测量半径
* `SegmentObstacle`
    - `geometry_msgs/Point first_point` - first point of the segment (in counter-clockwise direction), 分段的第一个点（逆时针方向）
    - `geometry_msgs/Point last_point` - end point of the segment. 分段的终点
* `Obstacles`
    - `Header header`
    - `obstacle_detector/SegmentObstacle[] segments`
    - `obstacle_detector/CircleObstacle[] circles`

## 3. The launch files  启动文件

Provided launch files are good examples of how to use `obstacle_detector` package. They give a full list of parameters used by each of provided nodes.

提供的启动文件是如何使用`obstacle_detector`包的很好的例子。它们提供了每个提供的节点使用的参数的完整列表。
* `demo.launch` - Plays a rosbag with recorded scans and starts all of the nodes with Rviz configured with appropriate panels. 运行带有记录扫描的rosbag，并使用适当的面板配置Rviz启动所有节点。
* `nodes_example.launch` - Runs all of the nodes with their parameters set to default values. 运行所有节点，并将其参数设置为默认值。
* `nodelets_example.launch` - Runs all of the nodelets with their parameters set to default values. 运行所有nodelets，并将其参数设置为默认值。

## 4. The displays  显示

For better visual effects, appropriate Rviz display for `Obstacles` messages was prepared. Via its properties, one can change the colors of the obstacles.

为了获得更好的视觉效果，准备了适用于“障碍物”消息的Rviz显示。通过它的属性，人们可以改变障碍物的颜色。

Author:
_Mateusz Przybyla_

