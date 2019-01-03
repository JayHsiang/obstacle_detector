# 包obstacle_detector

   obstacle_detector包提供了用于检测和跟踪由2D激光扫描仪提供数据的障碍的实用程序。检测到的障碍物以线段或圆圈的形式呈现。该包设计用于配备两个激光扫描仪的机器人，因此它包含几个额外的实用程序。resources文件夹中提供的文章描述了该方法的工作原理。

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

   该包提供单独的nodes/nodelets来执行单独的任务。在一般解决方案中，以下列方式处理数据：

`two laser scans` -> `scans merger` -> `merged scan or pcl` -> `obstacle extractor` -> `obstacles` -> `obstacle tracker` -> `refined obstacles`

   对于某些情况，直接从激光扫描（没有跟踪）的纯障碍物提取可能就足够了。

   可以使用ROS参数服务器配置节点。所有节点都提供私有`params`服务，允许进程从参数服务器获取最新参数。

   所有节点都可以处于活动或睡眠模式，通过在参数服务器中设置适当的变量并调用`params`服务来触发。在睡眠模式下，任何订阅者或发布者都会关闭，节点什么都不做直到再次激活。

   为了便于使用，建议使用为包中节点提供的合适的Rviz面板。Rviz面板通过参数服务器和服务-客户端调用进行通信，因此必须保持节点的名称不变（参见示例的启动文件）。

### 1.1. The scans_merger node       节点scans_merger

   该节点将两个类型为`sensor_msgs/LaserScan`的消息从主题`front_scan`和`rear_scan`转换为在主题`scan`下发布的相同类型的单个激光扫描，和/或在主题`pcl`下发布的类型为`sensor_msgs/PointCloud`的点云中。两者之间的区别在于，所得到的激光扫描将区域划分为有限数量的圆形扇区，并且在由一些测量点占据的每个区域中放置一个点（或实际上一个范围值），而得到的点云简单地复制从传感器获得的所有点。

   首先对输入激光扫描进行校正，以便及时合并扫描仪的运动（参见`laser_geometry`包）。接下来，将从前一步骤获得的两个PCL同步并转换为当前时间点的目标坐标系。

-----------------------
<p align="center">
  <img src="https://user-images.githubusercontent.com/1482514/27595821-0aa3519a-5b5e-11e7-9ce0-8f48db4592e4.gif" alt="Visual example of scans_merger output"/>
  <br/>
  Fig. 2. Visual example of `scans_merger` output.
</p>

-----------------------

   得到的消息包含关于特定坐标系（例如`robot`）描述的几何数据。假设连接到两个激光扫描仪的坐标系称为`front_scanner`和`rear_scanner`，则必须提供从`robot`系到`front_scanner`系和从`robot`系到`rear_scanner`系的转换。该节点允许人为地将测量点限制在`robot`系周围的某些矩形区域，以及限制所产生的激光扫描范围。在该区域外的点将被丢弃。

   即使仅使用一个激光扫描仪，该节点也可用于简单的数据预处理，例如纠正、范围限制或将点重新计算到不同的坐标系。该节点使用以下一组本地参数：

* `~active` (`bool`, default: `true`) - 活动/睡眠模式，
* `~publish_scan` (`bool`, default: `false`) - 发布合并的激光扫描消息，
* `~publish_pcl` (`bool`, default: `true`) - 发布合并的点云消息，
* `~ranges_num` (`int`, default: `1000`) - 360度激光扫描消息中包含的范围数（圆形扇区），
* `~min_scanner_range` (`double`, default: `0.05`) - 生成的激光扫描消息的最小允许范围值，
* `~max_scanner_range` (`double`, default: `10.0`) - 生成的激光扫描消息的最大允许范围值，
* `~min_x_range` (`double`, default: `-10.0`) - 点坐标的限制（这些限制之外的坐标点将被丢弃），
* `~max_x_range` (`double`, default: `10.0`) - as above,
* `~min_y_range` (`double`, default: `-10.0`) - as above,
* `~max_y_range` (`double`, default: `10.0`) - as above,
* `~fixed_frame_id` (`string`, default: `map`) - 用于扫描纠正的固定坐标系的名称，
* `~target_frame_id` (`string`, default: `robot`) - 用作生成的激光扫描或点云的原点的坐标系的名称。

   该程序包随附Rviz面板。

-----------------------
<p align="center">
  <img src="https://user-images.githubusercontent.com/1482514/28522255-97e2c210-7077-11e7-8ca1-556790b043e6.png" alt="Rviz panel for the scans_merger node."/>
  <br/>
  Fig. 3. Rviz panel for the `scans_merger` node.
</p>

-----------------------

### 1.2. The obstacle_extractor node   节点obstacle_extractor

   此节点将主题为`scan`的类型为`sensor_msgs/LaserScan`的消息或主题为“pcl”的类型为`sensor_msgs/PointCloud`的消息转换为障碍物，这些障碍物在主题`raw_obstacles`下发布为自定义类型`obstacles_detector/Obstacles`的消息。必须以有角度的方式对PCL消息进行排序，因为该算法利用了激光扫描仪的极性。

-----------------------
<p align="center">
  <img src="https://user-images.githubusercontent.com/1482514/27595822-0aa50ab2-5b5e-11e7-8061-1da4b947b617.gif" alt="Visual example of obstacle_extractor output."/>
  <br/>
  Fig. 4. Visual example of `obstacle_extractor` output.
</p>

-----------------------

   首先将输入点分组为子集并标记为可见或不可见（如果一个组在相邻组的前面，则它是可见的，否则假定它被遮挡）。该算法从每个点子集中提取分段。接下来，检查分段之间可能的合并。然后从分段中提取圆形障碍物并且如果可能的话也进行合并。产生的障碍物集可以转换为专用坐标系。

   该节点可使用以下一组本地参数进行配置：

* `~active` (`bool`, default: `true`) - 活动/睡眠模式
* `~use_scan` (`bool`, default: `false`) - 使用激光扫描信息
* `~use_pcl` (`bool`, default: `true`) - 使用点云消息（如果选择了scan和pcl，扫描将具有优先级）
* `~use_split_and_merge` (`bool`, default: `true`) - 选择是使用End Point Fit(false)还是使用Split And Merge (true)算法来检测分段
* `~circles_from_visibles` (`bool`, default: `true`) - 仅从完全可见（未遮挡）的分段中检测圆形障碍物
* `~discard_converted_segments` (`bool`, default: `true`) - 不发布产生圆形的分段
* `~transform_coordinates` (`bool`, default: `true`) - 将障碍物的坐标转换为用`frame_id`参数描述的系
* `~min_group_points` (`int`, default: `5`) - 构成要进一步处理的组的最小点数
* `~max_group_distance` (`double`, default: `0.1`) - 如果两点之间的距离大于此值，则开启一个新组
* `~distance_proportion` (`double`, default: `0.00628`) - 根据点的范围成比例地扩大点之间的允许距离（使用弧度的扫描角度增量）
* `~max_split_distance` (`double`, default: `0.2`) - 如果组中的一个点比该值的前导线更远，则拆分该组
* `~max_merge_separation` (`double`, default: `0.2`) - 如果障碍物之间的距离小于此值，则考虑合并它们
* `~max_merge_spread` (`double`, default: `0.2`) - 如果它们的所有极值点都比这个值更接近引导线，则合并两个分段
* `~max_circle_radius` (`double`, default: `0.6`) - 如果圆的半径大于此值，则跳过它
* `~radius_enlargement` (`double`, default: `0.25`) - 通过此值人为地扩大圆半径
* `~frame_id` (`string`, default: `map`) - 用作生成障碍物原点的坐标系的名称（仅当`transform_coordinates`标志设置为true时使用）

   该程序包随附Rviz面板。

-----------------------
<p align="center">
  <img src="https://user-images.githubusercontent.com/1482514/28522256-97e4d0be-7077-11e7-81fa-c2fcaae944be.png" alt="Rviz panel for the obstacle_extractor node."/>
  <br/>
  Fig. 5. Rviz panel for the `obstacle_extractor` node.
</p>

-----------------------

### 1.3. The obstacle_tracker node  节点obstacle_tracker

   该节点使用卡尔曼滤波器跟踪和过滤圆形障碍物。它从主题`raw_obstacles`订阅自定义类型`obstacle_detector/Obstacles`的消息，并在主题`tracked_obstacles`下发布相同类型的消息。跟踪算法仅应用于圆形障碍物，因此发布消息中的片段列表只是原始片段的副本。跟踪的障碍物补充了关于其速度的附加信息。

-----------------------
<p align="center">
  <img src="https://user-images.githubusercontent.com/1482514/27595819-0aa1bc86-5b5e-11e7-80ed-90c9ea53f69f.gif" alt="Visual example of obstacle_tracker output."/>
  <br/>
  Fig. 6. Visual example of `obstacle_tracker` output.
</p>

-----------------------

  节点以同步方式工作，默认速率为100Hz。如果较少发布检测到的障碍物，则跟踪器将对其进行超级采样并使其位置和半径平滑。下面的一组本地参数可用于调整节点：

* `~active` (`bool`, default: `true`) - 活动/睡眠模式
* `~copy_segments` (`bool`, default: `true`) - 将检测到的段复制到跟踪的障碍物消息中
* `~loop_rate` (`double`, default: `100.0`) - 主循环速率，单位为Hz
* `~tracking_duration` (`double`, default: `2.0`) - 在缺少输入数据的情况下跟踪障碍物的持续时间（在此时间从最后一次相应的测量之后，跟踪的障碍物将从列表中移除）
* `~min_correspondence_cost` (`double`, default `0.3`) - 通讯测试的阈值
* `~std_correspondence_dev` (`double`, default `0.15`) - （实验）通讯测试中位置椭圆的标准偏差
* `~process_variance` (`double`, default `0.01`) - 障碍物位置和半径的方差（卡尔曼滤波器的参数）
* `~process_rate_variance` (`double`, default `0.1`) - 障碍物值变化率的方差（卡尔曼滤波器参数）
* `~measurement_variance` (`double`, default `1.0`) - 测量障碍物值的方差（卡尔曼滤波器参数）
* `~frame_id` (`string`, default: `map`) - 描述障碍物的坐标系的名称

   该程序包随附Rviz面板。

-----------------------
<p align="center">
  <img src="https://user-images.githubusercontent.com/1482514/27595823-0aa835a2-5b5e-11e7-9657-8cd129a50aa2.png" alt="Rviz panel for the obstacle_tracker node."/>
  <br/>
  Fig. 7. Rviz panel for the `obstacle_tracker` node.
</p>

-----------------------

### 1.4. The obstacle_publisher node  节点obstacle_publisher

   辅助节点，允许以`obstacles`主题下的`obstacles_detector/Obstacles`类型的消息形式发布一组虚拟的圆形障碍物。该节点主要用于离线测试。以下参数用于配置节点：

* `~active` (`bool`, default: `true`) - 活动/睡眠模式
* `~reset` (`bool`, default: `false`) - 重置障碍物运动计算的时间（由专用的Rviz面板使用）
* `~fusion_example` (`bool`, default: `false`) - 产生障碍物显示融合
* `~fission_example` (`bool`, default: `false`) - 产生显示裂变的障碍物
* `~radius_margin` (`double`, default: `0.25`) - 以米为单位人为地将圆半径放大
* `~loop_rate` (`double`, default: `10.0`) - 主循环速率，单位为Hz
* `~frame_id` (`string`, default: `map`) - 描述障碍物的坐标系的名称

   以下参数用于为节点提供一组障碍物：

* `~x_vector` (`std::vector<double>`, default: `[]`) - 障碍物中心点的x坐标数组
* `~y_vector` (`std::vector<double>`, default: `[]`) - 障碍物中心点的y坐标数组
* `~r_vector` (`std::vector<double>`, default: `[]`) - 障碍物半径数组
* `~x_vector` (`std::vector<double>`, default: `[]`) - 障碍物中心点在x方向上的速度数组
* `~y_vector` (`std::vector<double>`, default: `[]`) - 障碍物中心点在y方向上的速度数组

  该程序包随附Rviz面板。

-----------------------
<p align="center">
  <img src="https://user-images.githubusercontent.com/1482514/28582779-f8de6fca-7166-11e7-8a52-e147cea89f87.png" alt="Rviz panel for the obstacle_publisher node."/>
  <br/>
  Fig. 8. Rviz panel for the `obstacle_publisher` node.
</p>

-----------------------

## 2. The messages   消息

   该包提供三种自定义消息类型。它们的所有数值均以SI为单位提供。

* `CircleObstacle`
    - `geometry_msgs/Point center` - 圆形障碍物的中心
    - `geometry_msgs/Vector3 velocity` - 圆形障碍物的线速度
    - `float64 radius` - 增加安全边缘的圆形障碍物的半径
    - `float64 true_radius` - 没有安全边缘的障碍物测量半径
* `SegmentObstacle`
    - `geometry_msgs/Point first_point` - 分段的第一个点（逆时针方向）
    - `geometry_msgs/Point last_point` - 分段的终点
* `Obstacles`
    - `Header header`
    - `obstacle_detector/SegmentObstacle[] segments`
    - `obstacle_detector/CircleObstacle[] circles`

## 3. The launch files  启动文件

   提供的启动文件是如何使用`obstacle_detector`包的很好的例子。它们提供了每个提供的节点使用的参数的完整列表。
* `demo.launch` - 运行带有记录扫描的rosbag，并使用适当的面板配置Rviz启动所有节点。
* `nodes_example.launch` - 运行所有节点，并将其参数设置为默认值。
* `nodelets_example.launch` - 运行所有nodelets，并将其参数设置为默认值。

## 4. The displays  显示

   为了获得更好的视觉效果，准备了适用于“障碍物”消息的Rviz显示。通过它的属性，人们可以改变障碍物的颜色。

Author:
_Mateusz Przybyla_

