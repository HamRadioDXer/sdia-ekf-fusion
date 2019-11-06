# robot_localization目录
## 1. 状态估计节点
### 1.1 ekf定位节点
ekf_localization_node是一个扩展的卡尔曼滤波器的实现。该方法利用全向运动模型及时预测状态，并利用感知到的传感器数据对预测结果进行修正。
### 1.2 ukf定位节点
ukf_localization_node是一个无迹卡尔曼滤波器的实现。它使用一组精心挑选的西格玛点，通过在EKF中使用的相同的运动模型来投射状态，然后使用这些投射的西格玛点来恢复状态估计和协方差。这消除了雅可比矩阵的使用，使滤波器更加稳定。但是，它在计算上也比ekf_localization_node更费力。
### 1.3 参数
ekf_localization_node和ukf_localization_node共享它们的大部分参数，因为大多数参数控制数据在与核心过滤器融合之前的处理方式。
 
状态估计节点可用的参数相对较多，因此启动和配置文件是启动任何节点的首选方法。该包包含模板启动和配置文件，以帮助用户启动。
#### （1）ekf_localization_node和ukf_localization_node的共享参数
>  [**标准参数**] 
-  **频率**

滤波器产生状态估计的实值频率，单位为Hz。

注意：直到从输入之一接收到至少一条消息，过滤器才会开始计算。
- **传感器超时**

实值周期（以秒为单位），之后我们认为任何传感器都已超时。在这种情况下，我们将对EKF进行预测周期，而无需对其进行校正。可以将此参数视为滤波器将生成新输出的最小频率的倒数。

- **two_d_mode**
 
如果您的机器人在平面环境中运行，并且您愿意忽略地面的细微变化（如IMU所报告），则将其设置为true。它将为所有3D变量（Z，侧倾，俯仰以及它们各自的速度和加速度）融合0值。这样可以确保这些值的协方差不会爆炸，同时确保机器人的状态估算值仍固定在XY平面上。

- **坐标系frame**

具体参数

- [地图坐标系] 
- [里程计坐标系]
- [机器人本体base_link坐标系]
- [机器人本体输出坐标系]
- [世界坐标系]

这些参数定义了机器人定位``robot_localization``的操作“模式”。REP-105指定三个主要坐标系：地图，里程计算和base_link。base_link是固定在机器人上的坐标系。机器人在odom框架中的位置会随着时间而漂移，但是在短期内是准确的，应该是连续的。的地图帧，像奥多姆帧，是世界固定坐标帧，虽然它包含用于你的机器人全球最准确的位置估计，它是受离散跳跃，例如由于GPS数据的融合。这是使用这些参数的方法：

1. 设置map_frame，odom_frame和base_link_frame参数去对应您的系统中相应的坐标框架名称。        


==注意==： 如果您的系统没有map_frame，则将其删除，并确保world_frame将其设置为的值odom_frame。

==注意==：如果您正在运行多个EKF实例，并且想“覆盖”输出转换和消息以使其具有此框架child_frame_id，则可以进行设置。该base_link_output_frame是可选的，默认为base_link_frame。当运行多个EKF实例时，这有助于启用断开连接的TF树。当计算最终状态时，我们“覆盖”输出转换和消息以使其具有该帧child_frame_id。

2. 如果你只融合如车轮编码器测距，视觉里程，或IMU数据集的连续位置数据world_frame到你的odom_frame价值。这是中状态估计节点的默认行为robot_localization，也是最常见的用法。
3. 如果你是全球融合的绝对位置数据，是受离散跳跃（例如，GPS或具有里程碑意义的观察位置更新），则：
- [1] 设定您world_frame的map_frame的值
- [2] 确保其他东西正在生成odom->base_link转换。这甚至可以是robot_localization状态估计节点的另一个实例。然而，该实例应该不是保险丝全局数据。

map_frame、odom_frame和base_link_frame的默认值分别是map、odom和base_link。base_link_output_frame参数默认为base_link_frame的值。world_frame参数默认为odom_frame的值。

- **transform_time_offset**

有些包要求您的转换以一个小的时间偏移量为未来日期。该参数的值将被添加到robot_localization中状态估计节点生成的map->odom或odom->base_link变换的时间戳中。

- **transform_timeout**

robot_localization包使用tf2的lookupTransform方法来请求转换。此参数指定如果转换还不可用，我们希望等待多长时间。如果没有设置，默认值为0。值0意味着我们只获得最新可用的(参见tf2实现)转换，因此我们不会阻塞过滤器。指定一个非零的transform_timeout会影响过滤器的计时，因为它会等待transform_timeout的最大值以使转换可用。这直接意味着在大多数情况下，指定的期望输出速率没有得到满足，因为更新时过滤器必须等待转换。

- **传感器**

对于每个传感器，用户需要根据消息类型定义此参数。例如，如果我们定义一个Imu消息源和两个测程消息源，配置将如下所示:
```
<param name="imu0" value="robot/imu/data"/>
<param name="odom0" value="wheel_encoder/odometry"/>
<param name="odom1" value="visual_odometry/odometry"/>
```
每个参数名的索引都是基于0的(例如，odom0、odom1等)，并且必须按顺序定义(例如，如果没有定义pose1，就不要使用pose0和pose2)。每个参数的值都是该传感器的主题名称。

- **传感器配置**

具体参数：
- [odomN_config] 
- [twistN_config] 
- [imuN_config]  
- [poseN_config]  

对于上述定义的每个传感器消息，用户必须指定应该将这些消息的哪些变量融合到最终状态估计中。里程测量配置的一个例子可能是这样的:


```
<rosparam param="odom0_config">[true,  true,  false,
                                false, false, true,
                                true,  false, false,
                                false, false, true,
                                false, false, false]</rosparam>
```

这些布尔值的顺序是X、Y、Z、roll、pitch、yaw、X˙、Y˙、Z˙、roll˙、pitch˙、yaw˙、X¨、Y¨、Z¨。在这个例子中，我们融合了X和Y的位置，偏航角，X˙，和偏航角˙。

==注意==：规范是在传感器的frame_id中完成的，而不是在world_frame或base_link_frame中。有关更多信息，请参阅配置教程。
- **[sensor]_queue_size**

具体参数：

```
- [~odomN_config] 
- [~twistN_config]
- [~imuN_config]
- [~poseN_config]
```


- **[sensor]_differential**

具体参数：

```
- [~odomN_differential] 
- [~imuN_differential]
- [~poseN_differential]
```

对于上面定义的包含姿态信息的每个传感器消息，用户可以指定是否应该对姿态变量进行不同的集成。如果给定的值设置为true，那么对于有问题的传感器在t时刻的测量，我们首先减去t−1时刻的测量，然后将结果值转换为速度。如果您的机器人有两个绝对姿态信息来源，例如来自里程计的偏航测量和IMU，则此设置特别有用。在这种情况下，如果输入源上的方差配置不正确，这些测量值可能彼此不同步，并在过滤器中引起振荡，但是通过对其中一个或两个进行不同的积分，我们可以避免这种情况。

用户在使用这个参数进行方向数据时应该小心，因为转换为速度意味着方向状态变量的协方差将会无限制地增长(除非另一个绝对方向数据的来源正在被融合)。如果你只是想让所有的姿态变量都从0开始，那么请使用_relative参数。

==注意==:如果您通过navsat_transform_node或utm_transform_node融合GPS信息，您应该确保_差分设置为假。

- **[sensor]_relative**

具体参数：

```
- [~odomN_relative] 
- [~imuN_relative]
- [~poseN_relative]
```

如果该参数设置为true，则来自该传感器的任何测量值都将与从该传感器接收到的第一个测量值相融合。如果您希望状态估计总是从(0,0,0)开始，并且滚、俯仰和偏航值都是(0,0,0)，那么这是非常有用的。它类似于_微分参数，但是我们总是在0时刻去掉测量值，而不是在t−1时刻去掉测量值，而且测量值没有转换为速度。

- **imuN_remove_gravitational_acceleration（imu去除重力加速度）**

如果融合来自IMUs的加速度计数据，该参数将决定在融合之前是否将重力加速度从加速度测量中移除。

==注意==：这里假设提供加速度数据的IMU也产生一个绝对方向。正确消除重力加速度需要方向数据。

- **gravitational_acceleration**

如果``imun_remove_gravity/al_acceleration``设置为true，则该参数决定从IMU的线性加速度数据中删除的重力加速度Z。是9默认9.80665 (m/s^2)
- **初始状态**

以指定的状态启动筛选器。状态是一个15维的双精度向量，其顺序与传感器配置相同。例如，要在(5.0,4.0,3.0)的位置启动机器人，偏航为1.57，线速度为(0.1,0.2,0.3)，您将使用:


```
<rosparam param="initial_state">[5.0,  4.0,  3.0,
                                 0.0,  0.0,  1.57,
                                 0.1,  0.2,  0.3,
                                 0.0,  0.0,  0.0,
                                 0.0,  0.0,  0.0]</rosparam>
```
- **发布tf**

如果为真，状态估计节点将把转换从world_frame参数指定的帧发布到base_link_frame参数指定的帧。默认值为true。
- **发布加速度**

如果为真，状态估计节点将发布线性加速状态。默认值为false。
- **print_diagnostics**

如果为真，状态估计节点将向/diagnostics主题发布诊断消息。这对于调试配置和传感器数据非常有用。

> **高级参数**
- use_control

如果为真，则状态估计节点将侦听geometry_msgs/Twist消息的cmd_vel主题，并使用该主题生成一个加速项。这一项将用于机器人的状态预测。这在某些情况下特别有用，即使在给定状态变量的收敛上有很小的延迟，也会导致应用程序出现问题(例如，在旋转过程中激光雷达发生移位)。默认值为false。

==注意==：来自IMU的线性加速度数据的存在和包含将当前“覆盖”预测的线性加速度值。
- **stamped_control **

如果为true，use_control并且也为true，则查找geometry_msgs / TwistStamped消息，而不是geometry_msgs / Twist消息。

- **control_timeout **

如果use_control设置为true且在此时间内没有收到控制命令（以秒为单位），则基于控制的加速项将停止应用。


- **control_config**

控制cmd_vel消息中的哪些变量用于状态预测。这些值的顺序是X˙、Y˙、Z˙、横滚˙、俯仰˙、偏航˙。仅在use_control设置为true时使用。


```
<rosparam param="control_config">[true,  false, false,
                                  false, false, true]</rosparam>
```

- **acceleration_limits**

你的机器人在每个维度上的加速速度。匹配control_config中的参数顺序。仅在use_control设置为true时使用。

<
```
rosparam param="acceleration_limits">[1.3, 0.0, 0.0,
                                       0.0, 0.0, 3.2]</rosparam>
```

- **deceleration_limits**

您的机器人在每个尺寸上减速的速度。匹配中的参数顺序control_config。仅当use_control设置为true时使用。

- **acceleration_gains**

如果您的机器人无法立即达到其加速度极限，则可以通过这些增益来控制允许的变化。仅当use_control设置为true时使用。


```
<rosparam  param = “ acceleration_limits” > [ 0.8，0.0，0.0，0.0，0.0，0.9 
                                       ] </ rosparam>
```
- **deceleration_gains**

如果您的机器人无法立即达到其减速极限，则可以通过这些增益来控制允许的变化。仅当use_control设置为true时使用。

> **smooth_lagged_data** 

如果您的任何传感器产生的时间戳数据都比最新的过滤器更新早（更明确地说，如果您有滞后的传感器数据源），则将此参数设置为true将在接收到滞后的数据后启用过滤器恢复到滞后测量之前的最后状态，然后处理所有测量直到当前时间。这对于来自需要大量CPU使用量以生成姿态估计值的节点（例如，激光扫描匹配器）进行的测量特别有用，因为它们经常滞后于当前时间。

- **HISTORY_LENGTH**
如果smooth_lagged_data设置为true，则此参数指定过滤器将保留其状态和测量历史记录的秒数。该值应至少等于滞后测量值与当前时间之间的时间增量。

- **[sensor]_nodelay**

具体参数：
- [~odomN_nodelay] 
- [~twistN_nodelay] 
- [~imuN_nodelay] 
- [~poseN_nodelay] 

如果为true，则设置tcpNoDelay 传输提示。有证据表明，Nagle的算法与及时接收大消息类型（例如nav_msgs / Odometry消息）有关。将输入设置为true会禁用该订户的Nagle算法。默认为false。

- **[sensor]_threshold（传感器阈值）**

具体参数：

- [~odomN_pose_rejection_threshold] 
- [odomN_twist_rejection_threshold] 
- [poseN_rejection_threshold] 
- [twistN_rejection_threshold]
- [imuN_pose_rejection_threshold] 
- [imuN_angular_velocity_rejection_threshold] 
- [imuN_linear_acceleration_rejection_threshold]

如果您的数据存在异常值，请使用这些阈值设置（表示为马哈拉诺比斯距离）来控制允许传感器测量距当前车辆状态多远。numeric_limits<double>::max()如果未指定，则每个默认为。

- **debug(调试)**

布尔标志，指定是否在调试模式下运行。警告：将其设置为true将生成大量数据。数据被写入debug_out_file参数的值。默认为false。

- **debug_out_file**

如果debug为true，则将调试输出写入其中的文件。

- **process_noise_covariance**

过程噪声协方差（通常表示为Q）用于对滤波算法预测阶段的不确定性建模。调整可能很困难，并且已作为参数公开以方便自定义。可以单独保留此参数，但是通过调整它可以取得更好的结果。通常，相对于输入消息中给定变量的方差，Q值越大，滤波器将收敛到测量值的速度就越快。

- **dynamic_process_noise_covariance** 

如果为true，将process_noise_covariance根据机器人的速度动态缩放。这很有用，例如，当您希望在机器人静止时机器人的估计误差协方差停止增长时。默认为false。

- **initial_estimate_covariance** 

[ 0 ，0 ]X想要将绝对姿态变量的初始协方差值设置为大数。这是因为那些误差将无限制地增长（由于缺少绝对姿态测量来减小误差），并且以大的值开始将不会使状态估计受益。

- **reset_on_time_jump**

如果设置为true且ros::Time::isSimTime()为true，则当在主题上检测到时间跳回时，过滤器将重置为其未初始化状态。这在处理bag数据时很有用，因为可以在不重新启动节点的情况下重新启动bag。

- **predict_to_current_time**

如果设置为真，过滤预测并纠正了最新的测量（默认）的时间，但现在也可以预测到当前时间步长。

- **disabled_at_startup**

如果设置为true，则不会在启动时运行过滤器。

> **节点的具体参数**

标准和高级参数对于中的所有状态估计节点都是通用的robot_localization。本节详细介绍了各自状态估计节点所特有的参数。

- **ukf_localization_node** 

这些参数ukf_localization_node遵循原始论文和Wiki文章的命名法。

1. 〜alpha -控制sigma点的传播。除非你是熟悉无味卡尔曼滤波器，它可能是最适合这种设置保留其默认值（0.001）。

1. 〜kappa -也控制的西格玛点的价差。除非您熟悉无味的卡尔曼滤波器，否则最好将此设置保持为默认值（0）。

1. 〜beta -涉及所述状态向量的分布。默认值为2表示分布是高斯分布。像其他参数一样，除非用户熟悉无味的卡尔曼滤波器，否则该参数应保持不变。

- **发布主题**

odometry/filtered（nav_msgs / Odometry）

accel/filtered（geometry_msgs/AccelWithCovarianceStamped）（如果启用）

- **发布的转换**

如果用户的world_frame参数设置为的值，则将odom_frame转换从odom_frame参数给定的帧发布到参数给定的帧base_link_frame。

如果用户的world_frame参数设置为的值，则将map_frame转换从map_frame参数给定的帧发布到参数给定的帧odom_frame。

注意：此模式假定另一个节点正在广播从odom_frame参数给定的帧到参数给定的帧的转换base_link_frame。这可以是robot_localization状态估计节点的另一个实例。

- **服务**

set_pose-通过向主题发出geometry_msgs / PoseWithCovarianceStamped消息set_pose，用户可以手动设置过滤器的状态。这对于在测试过程中重置过滤器很有用，并允许与进行交互rviz。可选地，状态估计节点发布SetPose服务，其类型为robot_localization / SetPose。

---
#  2. navsat转换节点
navsat_transform_node输入nav_msgs / Odometry消息（通常是ekf_localization_node或的输出ukf_localization_node），包含对机器人航向的准确估计的sensor_msgs / Imu以及包含GPS数据的sensor_msgs / NavSatFix消息作为输入。它会在与您的机器人世界框架一致的坐标中生成测距消息。该值可以直接融合到您的状态估计中。

==注意==：如果将此节点的输出与中的任何状态估计节点融合robot_localization，则应确保该输入的odomN_differential设置为false。
## 2.1 参数

- **〜频率**

设置为true时，以Hz为单位的实值频率，用于navsat_transform_node检查新的sensor_msgs / NavSatFix消息，并发布经过过滤的sensor_msgs / NavSatFix。publish_filtered_gps

- **〜延迟**

计算从GPS坐标到机器人的世界坐标的转换之前要等待的时间（以秒为单位）。

- **〜magnetic_declination_radians** 

输入您所在位置的磁偏角。如果您不知道，请访问http://www.ngdc.noaa.gov/geomag-web（确保将值转换为弧度）。如果您的IMU相对于磁北偏向，则需要此参数。

- **〜yaw_offset** 

当您向东时，IMU的偏航读数应为0。如果不是，请在此处输入偏移量（desired_value =偏移量+ sensor_raw_value）。例如，如果您的IMU朝北时报告0，就像大多数情况一样，则此参数为pi/2（〜1.5707963）。此参数的版本已更改2.2.1。以前，navsat_transform_node假设IMU面向北时读为0，所以相应地使用yaw_offset。

- **zero_altitude**

如果为true，则此节点生成的nav_msgs / Odometry消息的姿态Z值设置为0。

- **publish_filtered_gps**

如果为true，navsat_transform_node还将把您的机器人的世界框架（例如map）位置转换回GPS坐标，并在该主题上发布sensor_msgs / NavSatFix消息/gps/filtered。

- **broadcast_utm_transform**

如果为true，navsat_transform_node则将广播UTM网格和输入里程表数据的帧之间的转换。有关更多信息，请参见下面的发布的转换。

- **use_odometry_yaw**

如果为true，navsat_transform_node则不会从IMU数据获取标题，而是从输入的里程表消息获取标题。如果您的里程表消息具有在以地球为参考的框架中指定的方向数据（例如，由磁力计生成的数据），则用户应注意仅将其设置为true。另外，如果测距法源是其中的状态估计节点之一robot_localization，则用户应至少将一个绝对方位数据源馈入该节点，并且将_differential和_relative参数设置为false。

-**wait_for_datum**

如果为true，navsat_transform_node将等待从以下任一位置获取数据：

- [该datum参数] 
- [该set_datum服务]

- **broadcast_utm_transform_as_parent_frame**

如果为true，navsat_transform_node将发布utm-> world_frame转换，而不是world_frame-> utm转换。请注意，要发布的转换broadcast_utm_transform还必须设置为true。

- **transform_timeout**

此参数指定如果尚不可用转换我们要等待多长时间。如果未设置，则默认为0。值0表示我们只是获取了最新的可用（请参阅tf2实现）转换。
## 2.2 订阅的话题

- imu/data甲sensor_msgs / IMU与方位数据消息
- odometry/filtered一个nav_msgs/里程计的机器人的当前位置的信息。如果您的机器人已经达到一些非零的姿态之后，第一次读取GPS，则需要这样做。
- gps/fix一个sensor_msgs / NavSatFix包含您的机器人的GPS坐标信息
## 2.3 发布的话题
- odometry/gps甲nav_msgs /里程计坐标框架含有你的机器人的GPS坐标信息，变换成它的世界。该消息可以直接融合到robot_localization的状态估计节点中。
- gps/filtered（可选）sensor_msgs / NavSatFix消息，其中包含机器人的世界框架位置，已转换为GPS坐标
## 2.4 发布的转换
- world_frame->utm（可选）-如果broadcast_utm_transform参数设置为 true，则navsat_transform_node计算从 utm框架到frame_id输入里程表数据的转换。默认情况下，使用逆变换将utm框架发布为里程表框架的子代。使用该broadcast_utm_transform_as_parent_frame参数，utm框架将作为里程表框架的父项发布。如果一棵TF树中有多个机械手，这将很有用。
# 3.准备与robot_localization一起使用的数据
在开始使用robot_localization中的状态估计节点之前，用户必须确保他们的传感器数据格式良好。每种类型的传感器数据都有不同的注意事项，建议用户在尝试使用robot_localization之前完整阅读本教程。

如需更多信息，欢迎用户观看ROSCon 2015的演示。

## 3.1 遵守ros标准
要考虑的两个最重要的ROS REP是：

- REP-103（标准计量单位和坐标约定）
- REP-105（坐标框架约定）。

鼓励不熟悉ROS或状态估计的用户阅读两个REP，因为它几乎肯定会帮助您准备传感器数据。robot_localization尝试尽可能地遵守这些标准。

此外，它可能会有益于用户查看每种受支持的ROS消息类型的规范：

- nav_msgs /里程表
- geometry_msgs / PoseWithCovarianceStamped
- geometry_msgs / TwistWithCovarianceStamped
- sensor_msgs / Imu
## 3.2 坐标系和转换传感器数据

REP-105指定了四个主要的坐标帧:base_link、odom、map和earth。base_link框架被刚性地附加到机器人上。map和odom帧是世界固定的帧，它们的起点通常与机器人的起始位置一致。地球框架用于为多个地图框架(例如，为分布在大范围内的机器人)提供公共参考框架。地球框架与本教程无关。

robot_localization的状态估计节点产生状态估计，其位姿在map或odom帧中给出，其速度在base_link帧中给出。所有输入的数据在与状态融合之前都被转换成这些坐标系中的一个。每个消息类型中的数据转换如下:

- nav_msgs/Odometry—所有姿态数据(位置和方向)都从消息头的frame_id转换为world_frame参数指定的坐标系(通常是map或odom)。在消息本身中，这特别指的是包含在pose属性中的所有内容。所有twist数据(线速度和角速度)都从消息的child_frame_id转换为base_link_frame参数(通常是base_link)指定的坐标帧。
- geometry_msgs/PoseWithCovarianceStamped-处理方式与测程信息中的姿态数据相同。
- geometry_msgs/TwistWithCovarianceStamped-处理方式与里程数信息中的twist数据相同。
- sensor_msgs/Imu -IMU的讯息目前有一些含糊不清的地方，不过ROS社区正在解决这个问题。大多数IMUs在一个固定的坐标系中报告方向数据，该坐标系的X轴和Z轴分别由指向地磁北极和地球中心的矢量定义，Y轴向东(与地磁北极矢量90度偏移)。这个框架通常被称为NED(北，东，下)。但是，REP-103为户外导航指定了一个ENU(东、北、上)坐标系。在撰写本文时，robot_localization假设所有IMU数据都有ENU帧，并且不使用NED帧数据。这在将来可能会改变，但是现在，用户应该确保在将数据与robot_localization中的任何节点一起使用之前，将其转换为ENU帧。


IMU也可以定位在机器人的“中立”位置以外的位置。例如，用户可以将IMU安装在它的一侧，或者旋转它，使它朝向机器人前方以外的方向。这个偏移量通常由从base_link_frame参数到IMU消息的frame_id的静态转换指定。robot_localization中的状态估计节点会自动纠正传感器的方向，使其数据与base_link_frame参数指定的帧对齐。
## 3.3 处理tf_prefix
随着从ROS Indigo 到tf2的迁移，robot_localization仍然允许使用该tf_prefix参数，但是根据tf2，所有frame_id值都将去除任何前导“ /”。
## 3.4 每种传感器消息类型的注意事项
### 3.4.1 里程计
许多机器人平台都配备了提供瞬时平移和旋转速度的车轮编码器。许多人还内部整合了这些速度以生成位置估计。如果您对此数据负责或可以对其进行编辑，请记住以下几点：
1. 速度/姿势： robot_localization可以整合速度或绝对姿势信息。通常，最佳做法是：

- 如果里程表同时提供位置和线速度，请融合线速度。
- 如果里程表同时提供方向和角速度，请融合方向。

==注意==：如果您有两个方向数据来源，那么您要格外小心。如果两者都产生具有精确协方差矩阵的方向，则可以安全地融合方向。但是，如果其中一个或两个都未报告其协方差，则应仅融合来自更精确传感器的方向数据。对于另一个传感器，使用角速度（如果已提供），或继续融合绝对方向数据，但是为该传感器打开_differential模式。
2. frame_id：请参见上面有关坐标帧和变换的部分。
3. robot_localizationžrobot_pose_ekf1个Ë 3robot_localization。例外情况是您有第二个输入源测量有问题的变量，在这种情况下，协方差将起作用。

注意：有关更多信息，请参见配置robot_localization和从robot_pose_ekf迁移。
4. 标志：遵守REP-103意味着您需要确保数据的标志正确。例如，如果您有一个地面机器人，然后逆时针旋转它，则其偏航角应增加，并且其偏航速度应为正。如果将其向前推动，则其X位置应增加，并且其X速度应为正。
5. 变换：在广播奥多姆 - > * base_link *变换。当world_frame参数设置为odom_frame配置文件中参数的值时，robot_localization的状态估计节点既输出nav_msgs / Odometry消息中的位置估计，也输出从其odom_frame参数指定的帧到其base_link_frame参数的转换。但是，某些机器人驱动程序也会将此测距信息与里程计信息一起广播。如果用户要robot_localization负责此转换，则需要禁用其机器人驱动程序对该广播的广播。这通常作为参数公开。
### 3.4.2 IMU
除以下内容外，请务必阅读上述有关坐标系和IMU数据转换的部分。

1. 遵守规范：与里程表一样，请确保您的数据符合REP-103和sensor_msgs / Imu规范。仔细检查数据符号，并确保frame_id值正确。
1. 协方差：遵循里程计的建议，请确保您的协方差有意义。不要使用较大的值来使过滤器忽略给定的变量。将您要忽略的变量的配置设置为false。
1. 加速：注意加速数据。中的状态估计节点robot_localization假定将IMU放置在平坦表面上的其中立的右侧向上位置中，它将：
- Z轴是9.81米每秒的平方。
- 如果传感器滚动+90度(左侧向上)，Y轴的加速度应该是+9.81米/秒的平方。
- 如果传感器倾斜+90度(正面朝下)，X轴的读数应该是-9.81米/秒方。
### 3.4.3 PoseWithCovarianceStamped¶
请参阅里程计
### 3.4.4 TwistWithCovarianceStamped
请参阅里程计
## 3.5 常见错误
- 输入数据不符合REP-103。确保所有的值(特别是方向角)在正确的方向上增加和减少。
- 不正确的frame_id值。速度数据应该在base_link_frame参数给出的框架中报告，或者在速度数据的frame_id和base_link_frame之间存在转换。
- 膨胀的协方差。在度量中忽略变量的首选方法是通过odomN_config参数。
- 失踪的协方差。如果您已经配置了一个给定的传感器来将一个给定的变量融合到状态估计节点中，那么该值的方差(即， (i,i)处的协方差矩阵值，其中i为该变量的下标，不应为0。如果遇到一个被融合的变量的方差为0，状态估计节点将在该值上增加一个小的epsilon值(1e−6)。更好的解决方案是让用户适当地设置协方差。
# 4. 配置robot_localization
将传感器数据合并到任何robot_localization状态估计节点的位置估计中时，重要的是要提取尽可能多的信息。本教程详细介绍了传感器集成的最佳实践。

有关更多信息，建议用户观看ROSCon 2015的演示文稿。
## 4.1 传感器配置
即使所讨论的消息类型在配置向量中不包含某些变量（例如，<< MsgLink（geometry_msgs / TwistWithCovarianceStamped）>>缺少任何姿势数据），所有传感器的配置向量格式也相同。配置向量仍然具有姿势变量的值）。未使用的变量将被忽略。

==注意==:配置向量是在输入消息的frame_id中给出的。例如，考虑一个速度传感器，它生成一个geometry_msgs/ twistwithcovarimessage，其frame_id为velocity_sensor_frame。在这个例子中，我们假设存在一个从velocity_sensor_frame到您的机器人的base_link_frame(例如，base_link)的转换，并且这个转换会将velocity_sensor_frame中的X速度转换为base_link_frame中的Z速度。为了将来自传感器的X速度数据包含到滤波器中，配置向量应该将X速度值设为true，而不是设为Z˙速度值:

```
<rosparam param="twist0_config">[false, false, false,
                                 false, false, false,
                                 true,  false, false,
                                 false, false, false,
                                 false, false, false]</rosparam>
```
注意布尔值的顺序是(X,Y,Z,roll,pitch,yaw,X˙，Y˙，Z˙，roll˙，pitch˙，yaw˙，X¨，Y¨，Z¨)。
## 4.2 以2D操作？
在配置传感器时要做的第一个决定是，您的机器人是否在平面环境中工作，并且您可以忽略地平面变化的细微影响，就像IMU报告的那样。如果是，请将two_d_mode参数设置为true。这有效地将每个测量中的三维位姿变量归零，并迫使它们在状态估计中融合。
## 4.3 融合不可测量的变量
让我们从一个例子开始。假设你有一个在平面环境中工作的轮式非完整机器人。你的机器人有一些轮子编码器，用来估计瞬时X速度和绝对姿态信息。此信息在nav_msgs/里程计消息中报告。此外，你的机器人有一个IMU，可以测量转速、车辆姿态和线性加速度。它的数据在sensor_msgs/Imu消息中报告。由于我们是在平面环境中操作的，所以我们将two_d_mode参数设置为true。这将自动归零所有三维变量，如Z，滚，俯仰，它们各自的速度，和Z加速度。我们从这个配置开始:

```
<rosparam param="odom0_config">[true, true, false,
                                false, false, true,
                                true, false, false,
                                false, false, true,
                                false, false, false]</rosparam>

<rosparam param="imu0_config">[false, false, false,
                               false, false, true,
                               false, false, false,
                               false, false, true,
                               true, false, false]</rosparam>
```
首先，平面机器人只需要考虑X, Y, X˙，Y˙，X, Y, Y, yaw, yaw˙。然而，这里有几点需要注意。
1. 对于odom0，我们包括X和Y(在世界坐标系中报告)，偏航，X˙(在体坐标系中报告)，以及偏航˙。但是，除非您的机器人内部使用IMU，否则它很可能只是使用车轮编码器数据来生成其测量值。因此，它的速度、航向和位置数据都来自同一来源。在这种情况下，我们不希望使用所有的值，因为您将重复的信息输入到过滤器中。相反，最好使用速度:

```
<rosparam param="odom0_config">[false, false, false,
                                false, false, false,
                                true, false, false,
                                false, false, true,
                                false, false, false]</rosparam>

<rosparam param="imu0_config">[false, false, false,
                               false, false, true,
                               false, false, false,
                               false, false, true,
                               true, false, false]</rosparam>
```

2. 接下来，我们注意到我们没有把Y˙融合。乍一看，这是正确的选择，因为我们的机器人不能立即横向移动。但是，如果nav_msgs/Odometry message报告了一个Y˙的0值(并且Y˙的协方差没有被夸大到很大的值)，那么最好将这个值反馈给过滤器。在本例中，测量值为0，表示机器人不可能向该方向移动，因此，测量值为0是完全有效的:

```
<rosparam param="odom0_config">[false, false, false,
                                false, false, false,
                                true, true, false,
                                false, false, true,
                                false, false, false]</rosparam>

<rosparam param="imu0_config">[false, false, false,
                               false, false, true,
                               false, false, false,
                               false, false, true,
                               true, false, false]</rosparam>
```
你可能会觉得奇怪，为什么我们不把Z˙的速度融合在一起呢?答案是，当我们将two_d_mode设置为false时，确实是这样。如果我们不这样做，我们可以将Z˙速度的0度量融合到这个滤波器中。
3. 最后，我们来看IMU。你可能已经注意到我们把Y''设置为False了。这是因为许多系统，包括我们在这里讨论的假设系统，不会经历瞬时Y加速度。然而，IMU可能会报告Y加速度的非零、噪声值，这可能导致您的估计值迅速漂移。
## 4.4 微分和相对参数
“robot_localization”中的状态估计节点允许用户融合任意数量的传感器。这允许用户使用多个源来测量某些状态向量变量——特别是姿态变量。例如，您的机器人可能从多个imu获取绝对方向信息，或者它可能有多个数据源来估计其绝对位置。在这种情况下，用户有两个选择:
1. 按原样融合所有绝对位置/方向数据，例如:

```
<rosparam param="imu0_config">[false, false, false,
                               true,  true,  true,
                               false, false, false,
                               false, false, false,
                               false, false, false]</rosparam>

<rosparam param="imu1_config">[false, false, false,
                               true,  true,  true,
                               false, false, false,
                               false, false, false,
                               false, false, false]</rosparam>
```
在这种情况下，用户应该**非常**小心，确保每个测量的方向变量的协方差设置正确。如果每个IMU的偏航方差，例如:math: ' 0.1 '，而IMUs的偏航测量值之间的差值是:math: ' > 0.1 '，那么滤波器的输出将在每个传感器提供的值之间来回振荡。用户应确保每个测量周围的噪声分布重叠。
2. 当然，用户可以使用_differentiation参数。对于给定的传感器，将其设置为true，通过计算两个连续时间步长的测量值的变化，将所有位姿(位置和方向)数据转换为速度。然后数据被融合成一个速度。但是，用户应该再次注意:当度量完全融合时(特别是imu)，如果度量对于给定的变量有一个静态或非递增的方差，那么估计的协方差矩阵中的方差将是有界的。如果将该信息转换为速度，那么在每个时间步长时，估计值将获得一些小的误差，所讨论的变量的方差将无限制地增长。对于位置(X,Y,Z)信息，这不是问题，但对于方向数据，这是一个问题。例如，一个机器人在它的环境中移动，一段时间后在X方向上积累1.5米的误差是可以接受的。如果同一机器人在左右移动时积累了1.5弧度的偏航误差，那么当机器人下一次向前行驶时，它的位置误差将会爆炸。

微分参数的一般经验法则是，如果给定的机器人只有一个方向数据源，那么微分参数应该设置为false。如果有N个源，用户可以将其中N - 1个源的_微分参数设置为true，或者简单地确保协方差值足够大以消除振荡。
# 5. 基于robot_localizaion_ekf移植
从robot_pose_ekf迁移非常简单。此页面旨在突出包之间的相关差异，以促进快速转换。
## 5.1 源消息中的协方差
对于robot_pose_ekf，让过滤器忽略测量值的一种常见方法是给它一个极大的膨胀协方差，通常是10^3的数量级。然而，robot_localization中的状态估计节点允许用户指定测量中的哪些变量应该与当前状态融合。如果你的传感器报告零对于一个给定的变量和你不想融合值过滤,或者如果知道传感器产生糟糕的数据字段,然后简单地设置其xxxx_config参数值为false的变量问题的主要页面(见这个参数的描述)。

但是，用户应该注意:有时候平台约束会创建隐式的变量度量。例如，一个不能立即向Y方向移动的差动驱动机器人可以用一个小的协方差值安全地融合Y˙的一个0测量。
## 5.2 差分参数
默认情况下，robot_pose_ekf将在t时刻进行姿态测量，确定它与t−1时刻的测量之间的差异，将该差异转换为当前帧，然后对该测量进行集成。这巧妙地帮助了两个传感器测量相同的位姿变量的情况:随着时间的推移，每个传感器报告的值将开始出现分歧。如果这些测量值中至少有一个的协方差没有恰当地增长，滤波器最终将开始在测量值之间振荡。通过微分积分，避免了这种情况，测量值与当前状态一致。
1. 如果将两个不同的源用于相同的位姿数据(例如，两个不同的传感器测量Z位置)，确保这些源准确地报告它们的协方差。如果两个源开始出现分歧，那么它们的协方差应该反映至少一个源中出现的增长误差。
2. 如果可用，融合速度数据，而不是位姿数据。如果您有两个测量相同变量的独立数据源，则将最精确的数据源合并为姿态数据和速度数据。
3. 作为(2)的替代方案，如果对于给定的姿态测量无法获得速度数据，则为其中一个传感器启用_差分参数。这将导致它作为一个速度被区分和融合。
使用robot_localization状态估计节点的三种不同方法可以避免这种情况:
# 6. 集成GPS数据
GPS数据集成是用户的普遍要求。robot_localization包含一个节点navsat_transform_node，它将GPS数据转换为一个帧，该帧与机器人在其世界框架中的起始姿态(位置和方向)一致。这大大简化了GPS数据的融合。本教程解释了如何使用navsat_transform_node，并深入研究了它背后的一些数学原理。

如需更多信息，欢迎用户观看ROSCon 2015的演示。
## 6.1 关于融合GPS数据的注意事项
在开始本教程之前，用户应确保自己熟悉REP-105。对于用户而言，重要的是要意识到，由于GPS数据易受离散的不连续性（“跳跃”）的影响，因此使用包含GPS数据的位置估计可能不适合导航模块使用。如果要将来自GPS的数据融合到位置估计中，一种可能的解决方案是执行以下操作：

- 运行robot_localization状态估计节点的一个实例，该实例仅融合连续数据，例如里程表和IMU数据。world_frame将此实例的参数设置为与odom_frame参数相同的值。在此框架中执行本地路径计划和动作。

- 运行robot_localization状态估计节点的另一个实例，该实例融合所有数据源，包括GPS。world_frame将此实例的参数设置为与map_frame参数相同的值。

但是，这只是一个建议，用户可以自由地将GPS数据融合到robot_localization状态估计节点的单个实例中。
## 6.2 使用navsat_transform_node
### 6.2.1 所需输入
navsat_transform_node需要三个信息源:机器人当前在其世界框架中的姿态估计、一个与地球相关的标题，以及一个以纬度/经度对表示的地理坐标(可选高度)。



这些数据可以通过三种不同的方式获得:
1. 这些数据可以完全来自机器人的传感器和姿态估计软件。要启用此模式，请确保wait_for_datum参数设置为false(其默认值)。所需讯息包括:

- 带有原始GPS坐标的sensor_msgs/NavSatFix消息。
- 带有绝对(地球参考)标题的sensor_msgs/Imu消息。
- 一个nav_msgs/测程信息，包含机器人在其起始位置指定的帧中的当前位置估计(通常是robot_localization状态估计节点的输出)。

2.可以通过datum参数指定基准（全局框架原点）。

注意 为了使用此模式，wait_for_datum必须将参数设置为true。

该datum参数采用以下形式：


```
<rosparam param="datum">[55.944904, -3.186693, 0.0, map, base_link]</rosparam>
```
参数顺序是纬度(以十进制度表示)、经度(以十进制度表示)、标题(以弧度表示)。，为robot_localization状态估计节点中的world_frame参数的值)，为机器人身体帧的frame_id(即，为robot_本地化状态估计节点中的base_link_frame参数的值)。当使用此模式时，机器人将假定您的机器人的世界框架原点位于指定的纬度和经度，且标题为0(东)。
3. 可以通过set_datum服务和使用robot_localization/SetDatum服务消息手动设置数据。
> **GPS数据**

请==注意==：navsat_transform_node的所有开发都是使用Garmin 18x GPS单元完成的，因此可能有其他单元生成的复杂数据需要处理。

优秀的nmea_navsat_driver包提供了所需的sensor_msgs/NavSatFix消息。这是我们将在本教程中使用的nmea_navsat_driver启动文件:


```
<node pkg="nmea_navsat_driver" type="nmea_serial_driver" name="navsat" respawn="true">
  <param name="port" value="/dev/ttyUSB0"/>
  <param name="baud" value="19200"/>
</node>
```
只有当用户没有通过datum参数或set_datum服务手动指定原点时，此信息才有意义。
> **IMU数据**

注意:自2.2.1版以来，navsat_transform_node已经转移到一个标准，其中所有的航向数据都假设从其零点朝东开始。如果您的IMU不符合这个标准，而是报告0时，面向北，您仍然可以使用yaw_offset参数来纠正这个错误。在这种情况下,yaw_offset将π/ 2的值(约1.5707963)。

用户应确保其imu符合REP-105。特别是，检查你的方向角的符号在正确的方向增加。此外，用户应该查找机器人工作区域的磁偏角，将其转换为弧度，然后将该值用于magnetic_declination_radians参数。

只有当用户没有通过datum参数或set_datum服务手动指定原点时，此信息才有意义。

> **里程计数据**

这应该是用于融合GPS数据的robot_localization状态估计节点实例的输出。
## 6.2.2 配置navsat_transform_node
下面是我们将在本教程中使用的navsat_transform_node启动文件:


```
<launch>

  <node pkg="robot_localization" type="navsat_transform_node" name="navsat_transform_node" respawn="true">

    <param name="magnetic_declination_radians" value="0"/>

    <param name="yaw_offset" value="0"/>

    <remap from="/imu/data" to="/your/imu/topic" />
    <remap from="/gps/fix" to="/your/gps/fix/topic" />
    <remap from="/odometry/filtered" to="/your/robot_localization/output/topic" />

  </node>

</launch>
```
这些参数将在主页上讨论。

## 6.2.3 配置robot_localization
此时，与robot_localization的集成非常简单。简单地添加这个块到您的状态估计节点启动文件:


```
<rosparam param="odomN_config">[true,  true,  false,
                                false, false, false,
                                false, false, false,
                                false, false, false,
                                false, false, false]</rosparam>
<param name="odomN_differential" value="false"/>
```
确保将odomN更改为您的odometry输入值(例如，odom1、odom2等)。此外，如果希望包含海拔数据，请将odomN_config的第三个值设置为true。

注意，如果你是在2D操作没有任何传感器测量Z或Z速度，你可以:

- 将navsat_transform_node的zero_altitude参数设置为true，然后将odomN_config的第三个值设置为true

- 在robot_本地化状态估计节点中将two_d_mode设置为true

您应该不需要修改状态估计节点中的_差分设置。GPS是一个绝对位置传感器，启用差分集成违背了使用它的目的。
## 6.2.4 细节
我们从一张图片开始。考虑一个从某个经度和纬度开始并带有某个标题的机器人。在本教程中，我们假设这个标题来自一个面向东方的IMU，它的读数为0，并根据ROS规范(即逆时针方向)。本教程的其余部分将参考图1:
![](https://ftp.bmp.ovh/imgs/2019/11/f6c23132b4c84c8e.png)

REP-105建议使用四种坐标系:base_link、odom、map和earth。base_link是固定在车辆上的坐标系。奥多姆和地图帧是世界固定的帧，它们通常起源于车辆的起始位置和方向。地球帧被用作多个地图帧的公共参考帧，目前还不被navsat_transform_node支持。注意，在图1中，机器人刚刚启动(t = 0)，因此它的base_link、odom和map帧是对齐的。我们还可以为UTM网格定义一个坐标系，我们称之为UTM。出于本教程的目的，我们将把UTM网格坐标系称为UTM。因此，我们要做的是创建一个utm->*map*转换。

参考图1，这些想法(希望如此)是清楚的。UTM原点是与机器人的GPS定位相关的UTM区域的(0UTM,0UTM)点。机器人开始于UTM区域内的某个位置(xUTM,yUTM)。上面的一些机器人的初始取向角θUTM网格的轴。因此我们变换将要求我们知道xUTM, yUTM和θ。

现在需要将纬度和经度转换为UTM坐标。UTM网格假设x轴朝东，y轴朝北，z轴指向地面。这符合repo -105所规定的右手坐标系。代表还说，偏航角为0意味着我们正对着x轴向下，偏航是逆时针增加的。navsat_transform_node假设你的标题数据符合这个标准。但是，有两个因素需要考虑:

- IMU驱动程序可能不允许用户应用磁偏角校正因子
- IMU驱动程序朝北时可能报错0，朝东时可能报错0(即使它的标题正确地增加和减少)。幸运的是，navsat_transform_node提供了两个参数来弥补IMU数据中可能存在的缺陷:magnetic_declination_radians和yaw_offset。参考图1，对于当前正在测量imu_yaw的偏航值的IMU，

    yawimu=−ω−offsetyaw+θ

    θ=yawimu+ω+offsetyaw
    
我们现在有一个翻译(xUTM yUTM)和旋转θ,我们可以使用它来创建所需的utm - >映射变换。我们使用转换将所有未来的GPS位置转换为机器人的局部坐标系。如果broadcast_utm_transform参数设置为true，则navsat_transform_node也将广播此转换。

如果您对本教程有任何问题，请在answers.ros.org上随意提问。
# 7. 软件包robot_localizaion的变更日志

> **2.6.5 (2019-08-08)**

- 修正:当use_sim_time为真时使用的墙壁时间

- 创建转换为/从lat long的服务

- 修正tf_prefix的错误

- 向doc添加新的贡献

- 添加未注册的参数

- 更新维基位置

- 贡献者:Andrew Grindstaff, Axel Mousset, Charles Brian Quinn, Oswin So, Tom Moore
> **2.6.4 (2019-02-15)**
- 子午线收敛调整添加到navsat_transform。

- 文档修改

- 添加broadcast_utm_transform_as_parent_frame
- 
如果没有配置构建类型，则启用构建优化。

- 贡献者:G.A. vd. Hoorn, Pavlo Kolomiiets, diasdm
> **2.6.3 (2019-01-14)**
- 将odomBaseLinkTrans重命名为baseLinkOdomTrans遵守命名约定Trans用于worldBaseLinkTrans和mapOdomTrans。

- 通过值捕获多态类型' class tf2::TransformException '，通过0x3fU捕获ZoneNumber来防止错误:指令输出可能被截断，将1到11个字节写入大小为4的区域

- 允许用户覆盖输出child_frame_id

- 修正欧拉体到世界的转换

- 空格

- 在旋律上固定没有数据服务

- 贡献者:Alexis schad, Matthew Jones, Tom Moore, thallerod
> **2.6.2 (2018-10-25)**

- 固定测试
- 贡献者：汤姆·摩尔
> **2.6.1（2018-10-25**）
- 为测量历史记录失败添加更多输出
- 添加过滤器处理切换服务
- 在启动navsat_transform_node之前等待有效的ROS时间
- 贡献者：汤姆·摩尔，史蒂夫·马肯斯基
> **2.6.0（2018-07-27）**
- 转到C ++ 14，添加错误标志，并修复所有警告
- 贡献者：汤姆·摩尔
> **2.5.2（2018-04-11）**
- 将已发布的accel主题添加到文档中
- 在可逆矩阵中为Nans添加日志语句
- 解决潜在的段错误
- 贡献者：Oleg Kalachev，Tom Moore，stevemacenski
> **2.5.1（2018-01-03**）
- 修复CMakeLists
- 贡献者：汤姆·摩尔
> **2.5.0（2017-12-15**）
- 固定原点精度
- 固定时间变量
- 修复状态历史记录还原
- 通过动态过程噪声协方差修复关键错误
- 修正阅读Mahalanobis阈值时的错字。
- GPS中的零旋转到base_link转换
- 更新xmlrpcpp包括对Indigo的支持
- 删除lastUpdateTime
- 在map-> odom转换中修复时间戳
- 简化enabledAtStartup逻辑
- 添加std_srvs依赖项
- 添加启用服务
- 确保所有原始传感器输入方向均已标准化，即使未显示消息
- 安装params目录。
- 添加机器人本地化估算器
- 添加Nodelet支持
> **2.4.0（2017-06-12**）
- 更新的文档
- 添加了reset_on_time_jump选项
- 添加了功能，以选择在navsat_transform_node中以父级形式发布utm框架
- 移动全局回调队列重置
- 添加了initial_state参数和文档
- 固定交流/减速增益默认逻辑
- 增加了重力参数
- 如果tf查找失败，则添加了延迟和限制
- 修复UKF IMUTwistBasicIO测试
- 添加了transform_timeout参数
- 在tf2 lookuptransform之前设置gps_odom时间戳
- 删除了不可移植的sincos呼叫
- 简化逻辑以解决相关错误
- 添加了动态过程噪声协方差计算
- 修复了catkin_package特征警告
- 添加了可选的加速状态发布
- 贡献者：Brian Gerkey，Enrique Fernandez，Jochen Sprickerhof，Rein Appeldoorn，Simon Gene Gottlieb，Tom Moore
> **2.3.1（2016-10-27）**
- 添加gitignore
- 添加剩余的维基页面
- 添加配置和准备页面
- 添加navsat_transform_node文档
- use_odometry_yaw修复n_t_n
- 历史记录不为空时解决手动姿势重置的问题
- 查找机器人姿势时进行逆变换。
- Sphinx文档
- 从navsat_transform输入主题中删除模板启动文件的正斜杠
- 使用navsat_transform_node为两级EKF安装添加示例启动文件和参数文件
- 为navsat_transform_node添加yaml文件，并将参数文档移至该文件。
- 使用用法注释更新EKF和UKF参数模板
- 撰稿人：Tom Moore，asimay
> **2.3.0（2016-07-28**）
- 修复了基准用法和frame_ids的问题
- 修复了wait_for_datum的注释
- 解决非零导航卫星传感器方向偏移的问题
- 解决了base_link-> gps变换破坏了“真” UTM位置计算的问题
- 对过滤的GPS使用正确的协方差
- 修复了统一里程表协方差错误
- 添加了过滤器历史记录和测量队列行为
- 更改输出时间戳以更准确地使用最近处理的测量的时间戳
- 添加了TcpNoDelay（）
- 添加了参数以使变换发布成为可选
- 修复了姿势数据的差异处理，以使其不关心消息的frame_id
- 更新了UKF配置并启动
- 添加了用于时间戳诊断的测试用例
- 通过诊断增加了不良时间戳的报告
- 更新了测试以匹配新方法签名
- 添加控制项
- 增加了用于延迟测量的平滑功能
- 使navsat_transform中的变量符合ROS编码标准
- 贡献者：Adel Fakih，Ivor Wanders，Marc Essinger，Tobias Tueylue，Tom Moore
> **2.2.3（2016-04-24**）
- 清理回调数据结构和回调并更新标头中的doxygen注释
- 删除MessageFilters
- 删除不推荐使用的参数
- 增加了处理GPS相对于车辆原点的偏移的功能
- 清理navsat_transform.h
- 使navsat_transform中的变量符合ROS编码标准
> **2.2.2（2016-02-04）**
- 更新Trig函数以使用sincos来提高效率
- 更新许可信息并添加仅Eigen MPL标志
- 将状态添加到Imu帧转换
- 如果缺少imu方向，则使用状态方向
- 手动添加第二个自旋以用于里程表和传递到消息过滤器的IMU数据
- 减少测量接收与滤波器输出之间的延迟
- 当设置零高度参数时，初始转换中的零高度也为零
- 固定回归并转换回GPS坐标
- 在接种中切换方向数据的裁剪使用mahalanobis检查子集以防止排除方向大于或小于±PI的测量
- 为EKF修复Jacobian。
- 仅测量速度时消除关于方向变量的警告
- 在IMU协方差中检查-1并忽略相关消息数据
- roslint和catkin_lint已应用
- 将base_link添加到基准规范中，并在指定基准时按测量处理顺序修复错误。还添加了检查以确保IMU数据在使用前是可转换的。
- 贡献者：Adnan Ademovic，Jit Ray Chowdhury，Philipp Tscholl，Tom Moore，ayrton04，kphil
> **2.2.1（2015-05-27**）
- 固定处理IMU数据的差模和相对模
> **2.2.0（2015-05-22）**
- 添加了对tf2友好的tf_prefix附加
- 更正了navsat_transform中的IMU方向
- 解决了乱序测量和位姿重置的问题
- 现在，节点采用偏航数据的ENU标准
- 删除了gps_common依赖项
- 向navsat_transform_node添加选项，从而可以使用里程表消息中的标题而不是IMU。
- 将setPoseCallback中使用的frame_id更改为world_frame
- 优化的本征算法可显着提高性能
- 迁移到tf2
- 代码重构和重组
- 从navsat_transform计算中删除了滚动和俯仰
- 固定的IMU数据转换，以更好地支持非标准方向上安装IMU
- 向navsat_transform_node添加了功能，从而可以将过滤的里程表数据覆盖回navsat数据
- 添加了一个参数，以允许将来约会world_frame-> base_link_frame转换。
- 删除了过时的差异设置处理程序
- 增加了相对模式
- 更新和改进的测试
- 修复姿势数据处理中的源frame_id
- 添加了初始协方差参数
- 修复了协方差复制中的错误
- 添加了主题队列大小的参数
- 当机器人具有非零的滚动和俯仰角时，改进的运动模型对角速度的处理
- 改变了差分测量的处理方式
- 添加了诊断
> **2.1.7（2015-01-05**）
- 添加了一些检查以消除不必要的回调
- 更新的启动文件模板
- 增加了测量离群值剔除
- 为tf消息过滤器添加了故障回调
- 为navsat_transform_node添加可选的world_frame->utm转换广播
- 修正了差模和2D模式下Z加速度的处理
- **2.1.6（2014年11月6日）**
- 加入无迹卡尔曼滤波器（UKF）定位节点
- 固定地图-> Odom TF计算
- IMU的加速度数据现在用于计算状态估计
- 添加了2D模式
> **2.1.5（2014年10月7日）**
- 将初始估计误差协方差更改为小得多
- 修复了一些调试输出
- 添加了测试套件
- 更好地遵守REP-105
- 固定差分测量处理
- 实施邮件过滤器
- 添加了navsat_transform_node
> **2.1.4（2014-08-22）**
- 添加utm_transform_node以安装目标
> **2.1.3（2014-06-22）**
- 进行一些更改以简化GPS集成
- 增加姿态数据的差分积分
- 一些文档清理
- 添加了UTM转换节点和启动文件
- Bug修复
> **2.1.2（2014-04-11）**
- 将协方差校正公式更新为“约瑟夫形式”，以提高滤波器的稳定性。
- 实施了新的版本控制方案。
> **2.1.1（2014-04-11**）
- 添加了对Eigen支持的cmake_modules依赖关系，并添加了include，以使来自tf include的boost :: signals警告保持沉默
# 8. 用户输入的教程
下面是用户提供的robot_localization教程列表!
## 8.1 教程
> **ros-sensor-fusion-tutorial（ros传感器融合教程**）

一个全面的端到端的教程，为传感器融合设置robot_localization，以及运行必要的概念。(包括一个使用超声波信标设置它的实际示例!)
> **Kapernikov: ROS robot_localization软件包**

一旦您了解了robot_localization包的工作方式，它的文档就非常清楚了。然而，它缺少一个实际操作的教程来帮助您完成第一步。有一些关于如何设置robot_localization包的很好的例子，但是它们需要好的工作硬件。本教程将使用turtlesim包作为虚拟机器人，试图弥补这一不足。
