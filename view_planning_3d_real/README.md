# 2月14日备注使用方法
+ 启动gazebo和moveit
    source ws_gazebo_moveit/devel/setup.bash   &&  roslaunch wam_arm_moveit wam_kinectv1_bringup_moveit_onlyrobot.launch 
    source ws_gazebo_moveit/devel/setup.bash   &&  roslaunch wam_arm_moveit wam_kinectv1_bringup_moveit_bedroom.launch 
source ws_gazebo_moveit/devel/setup.bash   &&  roslaunch wam_arm_moveit wam_kinectv1_bringup_moveit_livingroom.launch 
+ 可以接受Aslam物体的系统，可以接受ros发送来的物体信息，敲下回车后，会对离自己最近的物体开始规划，
rosrun view_planning QF-ASLAM 0 
+ 发布物体信息
        +  rosrun view_planning pub_object_debug 1    3 0 0.75 0 0 0     2.6 3 1.5
        +  rosrun view_planning pub_object_debug 2    3 0 0.75 0 0 0  2.6 3 1.5            0 10 0.75 0 0 0  2 5 3.5

        +  rosrun view_planning pub_object_debug 1    3.5 0 0.75 0 0 0     2.6 3 1.5       #大床
        + bedroom:  rosrun view_planning pub_object_debug 2    3.5 0 0.75 0 0 0  2.6 3 1.5       3.7 -4 0.6 0 0 0  1.2 2.3 1.2
        + bedroom 实机演示:  rosrun view_planning pub_object_debug 2    3.5 0 0.7 0 0 0  2.6 3 1.5       3.7 -4 0.6 0 0 0  1 2.3 1.2
        rosrun view_planning pub_object_debug 2      3.4 0 0.5 0 0 0  1.8 2  1.2     3.5 0 0.75 0 0 0  2.6 3 1.5
        rosrun view_planning pub_object_debug 1      3.4 0 0.5 0 0 0  1.8 2.5  1.2
        rosrun view_planning pub_object_debug 1      3.5 -4 0.4 0 0 0  3.4 3 1.2

        + living room:  rosrun view_planning pub_object_debug 3    2 0 0.6 0 0 0  1.4 1.4 1.4       1.5 -2.5 0.6 0 0 0  1.4 1.4 1.4    4 -2.5 0.6 0 0 0  1 2 1.3
        + living room 无花瓶:  rosrun view_planning pub_object_debug 3    2 0 0.6 0 0 0  1.4 1.4 1.4       1.5 -2.5 0.6 0 0 0  1.4 1.4 1.4    4 -2.5 0.25 0 0 0  1 2 0.5
        + SUV:  rosrun view_planning pub_object_debug 1    5 0 1.2 0 0 0  3 6 2.4 


+ 实机环境中五个物体群
黄桌子：    rosrun view_planning pub_object_debug 1      1.5, 1.95, 0.472255, 0, 0, 0, 1.4, 1, 1.1
电视桌：    rosrun view_planning pub_object_debug 1      4.0, 2.1, 0.8, 0, 0, 0, 1.5, 0.6, 0.8
圆桌子：    rosrun view_planning pub_object_debug 1      5.55, 1.430089, 0.58, 0, 0, 0, 1.2, 1.6, 1.2
沙发：     rosrun view_planning pub_object_debug 1      4.2, -0.5, 0.530371, 0, 0, 0, 1.9, 1.2, 1.15
床：       rosrun view_planning pub_object_debug 1      -5.1, -2.5, 0.520094, 0, 0, 0, 1.8, 1.9, 1.4

+ 获取相机坐标系在world中的真值
    rosrun tf tf_echo /world /camera_rgb_optical_frame



# 常用指令
+ 启动gazebo和moveit
    source ws_gazebo_moveit/devel/setup.bash   &&  roslaunch wam_arm_moveit wam_kinectv1_bringup_moveit_onlyrobot.launch 
+ 视点规划
    rosrun view_planning  gpmp_wam_getfromik 
+ rosrun tf tf_echo /wam/wrist_palm_link /camera_rgb_optical_frame
    At time 0.000
    - Translation: [0.020, -0.013, 0.130]
    - Rotation: in Quaternion [-0.500, 0.500, -0.500, 0.500]
                in RPY (radian) [-1.571, -0.000, -1.571]
                in RPY (degree) [-90.000, -0.000, -90.000]
        旋转矩阵 [0 0 1 0.02; -1 0 0 -0.013; 0 -1 0 0.13; 0 0 0 1]
        GTSAM:
        Rot3 R = Rot3::RzRyRx(-1.571,  -0.000, -1.571);
        Point3 t(0.020, -0.013, 0.130);
        Pose3 pose_camera_to_endlink(R, t);  // 第一个位姿 (T1)
+ 获取关节角度
  rostopic echo /joint_states
  position: [2.5282849221837402, 1.2592057759932551, 2.5160061763920627, -0.004919911137748478, -0.2684379731147075, -1.32252597005819, -0.06669228928161086]
  start_conf = (Vector(7) << 2.43078, -0.627707, 0.13426, 1.70191, -0.10452, -1.27042, 2.40398).finished();
  2.5282849221837402, 1.2592057759932551, 2.5160061763920627, -0.004919911137748478, -0.2684379731147075, -1.32252597005819, 1.5

+ 发布物体
rosrun view_planning pub_object_debug 1    3 0 0.75 0 0 0     2.6 3 1.5

+ 可以接受slam物体的系统
rosrun view_planning QF-ASLAM

# 利用椭圆+圆+moveit ik，实现环绕观测 commit 72c828759ba615f3a0c04c02b9d2e436e54966b6
+ gpmp_wam_Reproduce_Matlab
    cpp复现matlab机械臂手臂朝上
+ wam_getfromik_debug
    完成了利用椭圆和圆（GenerateCandidates_ellipse_by_circle）实现候选点，并通过moveit ik 计算关节角度。


# 准备融合g2o和gtsam     commit 7410dc96d6aaab6a2c1b2b8d82a7846a85f4583b
+ 因为wam机械臂会摔倒，因此引入了link wam/base_link的重量为1000
+ 添加Converter.cc  MapObject.cpp


# 第二次准备融合g2o和gtsam     commit 18cb4a9540ca6c993d42a0e3c0928603b907bc1d
+ 添加到

# 通过三角面可视化相机视场，并实现了点面距离的计算  commit b9a5d4d88226eb853f41507cc2122e989dc5ffd2
+ 删除了Thirdparty中的g2o，但是Ellipsoid.h还是到导入两个g2o头文件，但是我已经删除了啊？ 为什么还能找到并编译。
    #include "Thirdparty/g2o/g2o/core/base_multi_edge.h"
    #include "Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"
  姑且可以认为融合了g2o和gtsam
+ 实现了一点和相机视场平面（四个包围面）的距离计算；
+ 通过三角面，在rviz中可视化相机视场


# 计算点和视场包络面的距离，并构建误差因子  commit 211e71a33b8799756cd2af9a3b4b33c44e274f12
+ 建立专门的可视化线程
+ 实现了计算椭球体近端点与视场四个平面距离（四个平面的normal与相机z轴相同）的计算
+ 创建 BboxPlaneArmLink.h; ERROR为最小阈值距离减去各关节点与''视场下平面''的距离； ERROR导数与平面的normal相同；
    + 待： 构建视场下平面；


# 自制本体视场避遮挡gtsam因子, 可视化避障球 commit 375b6b6c578e97281db4b3c7431a8d30b6683b93
+ 验证BboxPlaneArmLink的Error基本正确
+ 修改WAM机械臂的DH参数和去掉了抓持爪子的避障球
+ 构建新类Visualize_Arm_Tools，可视化机械臂的避障球和各关节坐标   

# 自制本体视场和椭球体的gtsam因子, 失败  commit c177660de487b4f09f0f226999b70c40c0fb4c1e
+ 生成BboxPlaneEllipsoidFactor因子，
+ 在TestBboxPlaneArmLink.cpp中验证了bbox平面的准确性
+ 构建evaluateError。
+ 在Visualize_Arm_Tools，可视化bbox平面？？ 
+ 待解决 误差和偏导·+问题：引入lzw的椭球。椭球会不会和平面的距离计算函数（拉格朗日）会不会和gtsam冲突？

# 移植改造验证官方椭球体的gtsam因子，构建了BboxEllipsoidFactor commit 5aceced2d56ed9d8d39bb1e829a769bd30a4908c
+ 通过cv::imshow验证了ERROR的准确性
+ 验证了偏导(bbox_Error/Pose3)是4x6的矩阵
 *H1 = db_dC * dC_dx;
    + db_dC 为4x9, 误差向量 对 圆锥二次曲面参数（dual conic parameters） 的导数：
    + dC_dx 为9x6,  dual conic 参数 相对于 相机位姿（pose）参数 的导数：
+ 构建了BoundingBoxFactor因子，需要输入“机器人位姿”和“物体位姿”

# 联合优化失败 commit 0738b0e6f479bd8a0ce210bd615b428435f32a15
+ 编写wam_gpmp将我的三个创新点因子，联合优化，但有问题。
+ 问题分析：
    + BboxEllipsoidFactor： 雅可比链式法则太长
    + BboxPlaneArmLinkFactor： 雅可比推导失败

# 单独对相机轨迹进行优化 commit 532dc5e9ec65c29b3dd1657c74a37a9d3a0ebc23
+ 构建BboxCameraFactor
+ 构建HeightCameraFactor，限制相机高度

# 完全实现了对相机位姿的优化，可以用于论文展示  commit 6748adf34ca383a13cb42392cd2ea9527bd5281e
+ 缩小了measured_bbox到150
+ 修改BboxCameraFactor中的偏导，只优化pyr，对xyz的偏导设置为0
    H1->block<1, 6>(2, 0) /= 10000.0;
    H1->block<1, 6>(3, 0) /= 10000.0;
+ 更新到一个专门的分支“for-paper”

# 实现了椭球体相机位姿+机械臂min关节移动的two-step优化，可以用于论文展示
+ 在wam_bboxcamera.cpp中添加了机械臂min关节移动的第二步优化
+ 

#  一步优化，自动求导，用于论文展示  commit 19c726136bcd006f8eef3b7af9d2ee9d2509b7ac
+ wam_numericalDerivative.cpp
  + BboxEllipsoidFactor
  + BboxPlaneArmLinkFactor
  + CameraXaxisHorizontal
  + GaussianProcessPriorLinear


# rotate通过固定的步骤实现  commit 46db35feccbb19fb98a71fe007bde66c98851919
    + rotate通过固定的步骤实现
    + 修改start_conf和end_conf，从而远离机械臂活动范围的边缘

# 多线程，多物体，键盘控制   commit c7a76882c361706afc25f054bf900f652ad6ab78
    + 将地图、可视化工具、 候选点初值生成、 gpmp视点规划器，各封装成一个Class 
    + 构建一个发布object topic的程序  
        +  rosrun view_planning pub_object_debug 1    3 0 0.75 0 0 0     2.6 3 1.5
        +  rosrun view_planning pub_object_debug 2    3 0 0.75 0 0 0  2.6 3 1.5            0 10 0.75 0 0 0  2 5 3.5
    + 发布环视的起点
    + 键盘wafx控制底盘。s保存相机真值。两步回车确定开始planning、抵达起点
    + 根据距离，判断是否是新物体，从而添加进map
    + 根据距离，从map中选择用于View planning的物体对象
    + 检查物体的长宽比是否小于1.25，如果是的话，则修改小的一方
    
    + 待：移动完之后，更新键盘中的机器人位置

# 修正Visualize_Arm_Tools中最佳视场无法使用API控制的错误  commit 730d4a5cba923c0501959fd76f591d60cd990719 
    + 已完成

# 实现绕物体逆时针和MOVEIT_planning  commit f81225a1ff4db7eb9d074aef25c1a402818f0434
    + 已完成

# 对比对象改用direct方法
    + 相机始终平视


# 修改
    + 待：sdf的可视化
    + 待：去掉各因子项中计算误差时的冗余部分、

# 针对suv场景，调参
    + SUV场景下，视场角缩小了50.  
    + 修改底盘圆轨迹的scale:  
        planning:  scale = 1;
        planning_for_direct:   scale = 1.05
    + 设定direct模式下机械臂为{direct_yaws[i], 0, 0, 0, 0, 0, 0};
    + 【重要】：以上参数要记得改回去

# `实现保存底盘轨迹`
    + 







+ 待：重构BboxPlaneArmLink的雅可比
+ 待：在BboxCameraFactor中添加通过config计算bbox，并可视化
+ 待：修改bbox的error，改为让椭球投影到目标bbox内部
+ 
z
# 三维二次曲面 Class ConstrainedDualQuadric
+ 

# 平面二次曲线 Class DualConic
+ gtsam::Matrix33 dC_;  ///< 3x3 matrix of the quadratic equation
+ 

# 二次曲面的投影工具  Class QuadricCamera
+ QuadricCamera::project(quadric, camera_pose, calibration_, &dC_dq, dC_dx);
+ 其中 Eigen::Matrix<double, 9, 6> dC_dx 代表二次曲面对相机位姿的偏导？？？ 
+ 

#
+ 待：验证gpmp能否输入（GenerateCandidates_ellipse_by_circle）的结果，从而使的规划结果更顺滑。
+ 待：暂时，在底盘的结果上加上扰动，代表底盘的运动。验证效果之后，再改回来

# 安装的依赖项
sudo apt-get install ros-noetic-rviz-visual-tools  ros-noetic-moveit-visual-tools


# 待：删掉plane.h中对g2o库的调用