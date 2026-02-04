
#include "Viewer.h"


void Visualize_Tools::Run()
{   
    ros::Rate r(500);

    while(1){
        auto MapObjects = mpMap->getMapObjects();
        for(int i=0; i<MapObjects.size(); i++ ){
            visualize_ellipsoid( MapObjects[i], "world", i);
        }

        // for(int i=0; i<BboxPlanesTrianglePointsInWorld.size(); i++ ){
        //     visualize_plane_triangle_bypoint(BboxPlanesTrianglePointsInWorld[i], i, default_frame_);
        //     // std::cout << "Publishing triangle marker..." << std::endl;
        // }

        auto MapPlaneNormals = mpMap->getMapPlaneNormals();
        for(int i=0; i<MapPlaneNormals.size(); i++) {
            visualize_plane_rectangle(MapPlaneNormals[i], i, default_frame_);
        }

        r.sleep();
    }
}


void Visualize_Tools::visualize_point(Eigen::Vector3d& p , std::string frame_id, double id, double lifetime){
        visualization_msgs::Marker marker;
        marker.header.frame_id = frame_id;
        marker.header.stamp = ros::Time::now();
        marker.ns = "points";
        marker.id = id;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;

        // Set the pose of the marker
        marker.pose.position.x = p.x();
        marker.pose.position.y = p.y();
        marker.pose.position.z = p.z();
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        // Set the scale of the marker (1x1x1 here means 1m on each side)
        marker.scale.x = 0.2;
        marker.scale.y = 0.2;
        marker.scale.z = 0.2;

        // Set the color -- be sure to set alpha to something non-zero!
        marker.color.r = 1.0f;
        marker.color.g = 0.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0;

        // marker.lifetime = ros::Duration();
        marker.lifetime = ros::Duration(lifetime);

        point_pub.publish(marker);
}



// void Visualize_Tools::visualize_geometry_pose(geometry_msgs::Pose pose, std::string frame_id = "world",  double id_num = 1,  std::string name = "no-name", bool output = false){
void Visualize_Tools::visualize_geometry_pose(geometry_msgs::Pose pose, std::string frame_id,  double id_num,  std::string name, bool output){
    
    // ros::Publisher candidate_quaterniond_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("output", 1);
    
    

    // visualization_msgs::Marker marker;
    // marker.header.frame_id = frame_id;  // 假设您的世界坐标系为"world"
    // marker.id = id_num;
    // marker.type = visualization_msgs::Marker::ARROW;
    // marker.action = visualization_msgs::Marker::ADD;
    // marker.scale.x = 0.03;  // 箭头的尺寸
    // marker.scale.y = 0.03;
    // // marker.scale.z = 0.02;

    // // 设置箭头的起点和终点
    // geometry_msgs::Point start = eigen_to_point(candidates[i].start);
    // marker.points.push_back(start);
    // Eigen::Vector3d end_new = (candidates[i].end - candidates[i].start).normalized() * 0.2 + candidates[i].start;
    // geometry_msgs::Point end = eigen_to_point(end_new);
    // marker.points.push_back(end);

    // // 设置箭头的颜色
    // marker.color.a = 1.0;  // 不透明度
    // marker.color.r = 0.0;  // 红色分量
    // marker.color.g = 0.0;  // 绿色分量
    // marker.color.b = 1.0;  // 蓝色分量
    
    // candidate_pub.publish(marker);
    
    geometry_msgs::PoseWithCovarianceStamped target;
    target.header.frame_id = frame_id;	//设置了消息的头部信息
    //通过将四元数的分量（x、y、z、w）设置为transDockPos变量中存储的旋转四元数分量。这些分量描述了箭头方向的旋转。
    target.pose.pose.orientation.x = pose.orientation.x;
    target.pose.pose.orientation.y = pose.orientation.y;
    target.pose.pose.orientation.z = pose.orientation.z;
    target.pose.pose.orientation.w = pose.orientation.w;
    target.pose.pose.position.x = pose.position.x;
    target.pose.pose.position.y = pose.position.y;
    target.pose.pose.position.z = pose.position.z;

    if(output){
        ROS_INFO("Pose of [%s]: position(x: %f, y: %f, z: %f), orientation(x: %f, y: %f, z: %f, w: %f)", 
            name.c_str(),
            target.pose.pose.position.x, target.pose.pose.position.y, target.pose.pose.position.z,
            target.pose.pose.orientation.x, target.pose.pose.orientation.y, 
            target.pose.pose.orientation.z, target.pose.pose.orientation.w);
    }
    
    candidate_quaterniond_pub.publish(target);
}


// 函数：根据平面方程生成平面上的一个点
geometry_msgs::Point generatePointOnPlane(double a, double b, double c, double d, double x, double y) {
    geometry_msgs::Point point;
    point.x = x;
    point.y = y;
    
    // 根据平面方程 ax + by + cz + d = 0 计算 z
    if (c != 0) {
        point.z = -(a * x + b * y + d) / c;
    } else {
        point.z = 0.0;  // 如果 c == 0，默认为 z = 0
    }

    return point;
}

// 以三角的形式显示平面
void publish_plane_triangle(ros::Publisher& marker_pub, Eigen::Vector4d plane_param, int id, std::string frame_id = "world") {
    double a = plane_param[0];
    double b = plane_param[1];
    double c = plane_param[2];
    double d = plane_param[3];

    // 创建一个Marker消息
    // 创建一个 Marker 消息
    visualization_msgs::Marker marker;
    marker.header.frame_id = frame_id;  // 使用世界坐标系
    marker.header.stamp = ros::Time::now();
    marker.ns = "triangle";
    marker.id = id;
    marker.type = visualization_msgs::Marker::TRIANGLE_LIST;  // 使用三角形列表
    marker.action = visualization_msgs::Marker::ADD;

    // 设置颜色和透明度
    marker.color.a = 0.2;  // 不透明
    marker.color.r = 1.0;  // 红色
    marker.color.g = 0.0;
    marker.color.b = 0.0;

    // 设置比例
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;


    // 生成平面上的点
    std::vector<geometry_msgs::Point> points;
    points.push_back(generatePointOnPlane(a, b, c, d, 0, 0));
    points.push_back(generatePointOnPlane(a, b, c, d, 3.0, 0));
    points.push_back(generatePointOnPlane(a, b, c, d, 0, 3.0));
    marker.points = points;

    // 发布Marker消息
    marker_pub.publish(marker);
}

// 以三角的形式显示平面
void  Visualize_Tools::visualize_plane_rectangle(Eigen::Vector4d plane_param, int id, std::string frame_id) {
    double a = plane_param[0];
    double b = plane_param[1];
    double c = plane_param[2];
    double d = plane_param[3];

    // 创建一个Marker消息
    // 创建一个 Marker 消息
    visualization_msgs::Marker marker;
    marker.header.frame_id = frame_id;  // 使用世界坐标系
    marker.header.stamp = ros::Time::now();
    marker.ns = "rectangle";
    marker.id = id;
    marker.type = visualization_msgs::Marker::TRIANGLE_LIST;  // 使用三角形列表
    marker.action = visualization_msgs::Marker::ADD;

    // 设置颜色和透明度
    marker.color.a = 0.2;  // 不透明
    marker.color.r = 1.0;  // 红色
    marker.color.g = 0.0;
    marker.color.b = 0.0;

    // 设置比例
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;

    double size = 2.0;
    geometry_msgs::Point p1 = generatePointOnPlane(a, b, c, d, size, size);
    geometry_msgs::Point p2 = generatePointOnPlane(a, b, c, d, size, -1*size);
    geometry_msgs::Point p3 = generatePointOnPlane(a, b, c, d, -1*size, size);
    geometry_msgs::Point p4 = generatePointOnPlane(a, b, c, d, -1*size, -1*size);
    // 生成平面上的点
    std::vector<geometry_msgs::Point> points;
    points.push_back(p1);
    points.push_back(p2);
    points.push_back(p3);

    points.push_back(p4);
    points.push_back(p2);
    points.push_back(p3);

    marker.points = points;

    // 发布Marker消息
    normal_plane_pub.publish(marker);
}


// 以三角的形式显示平面
void Visualize_Tools::visualize_plane_triangle_bypoint(std::vector<geometry_msgs::Point>& points, int id, std::string frame_id) {
    
    // 创建一个 Marker 消息
    visualization_msgs::Marker marker;
    marker.header.frame_id = frame_id;  // 使用世界坐标系
    marker.header.stamp = ros::Time::now();
    marker.ns = "triangle";
    marker.id = id;
    marker.type = visualization_msgs::Marker::TRIANGLE_LIST;  // 使用三角形列表
    marker.action = visualization_msgs::Marker::ADD;

    // 设置颜色和透明度
    marker.color.a = 0.12;  // 不透明
    marker.color.r = 1.0;  // 红色
    marker.color.g = 0.0;
    marker.color.b = 0.0;

    // 设置比例
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;


    // 生成平面上的点
    points.push_back(points[0]);
    points.push_back(points[1]);
    points.push_back(points[2]);
    marker.points = points;

    // 发布Marker消息
    bbox_plane_pub.publish(marker);
}


void Visualize_Tools::visualize_ellipsoid(MapObject* ob, std::string frame_id, double id){
    // 创建 Marker 消息
    visualization_msgs::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = ros::Time::now();
    marker.ns = "ellipsoid_visualization";
    marker.id = id;  // 唯一的 ID
    marker.type = visualization_msgs::Marker::SPHERE;  // 使用 SPHERE 类型表示椭球体
    marker.action = visualization_msgs::Marker::ADD;

    // 设置椭球体的位置
    marker.pose.position.x = ob->mCuboid3D.cuboidCenter[0];
    marker.pose.position.y = ob->mCuboid3D.cuboidCenter[1];
    marker.pose.position.z = ob->mCuboid3D.cuboidCenter[2];

    // 椭球体的比例尺寸，a, b, c 分别为 x, y, z 方向的轴长度
    marker.scale.x = ob->mCuboid3D.lenth;  // x 轴长度
    marker.scale.y = ob->mCuboid3D.width;  // y 轴长度
    marker.scale.z = ob->mCuboid3D.height;  // z 轴长度

    // 设置颜色 (RGBA)
    if(!ob->explored){
        marker.color.r = 0.0f;
        marker.color.g = 1.0f;  // 绿色
        marker.color.b = 0.0f;
        marker.color.a = 0.5f;  // 透明度
    }else{
        marker.color.r = 1.0f;
        marker.color.g = 1.0f;  // 黄色
        marker.color.b = 0.0f;
        marker.color.a = 0.5f;  // 透明度
    }

    

    // 创建四元数并设置旋转（Roll, Pitch, Yaw）
    tf2::Quaternion quat;
    quat.setRPY(0, 0, 0);  // 设置旋转角度 (弧度)

    // 将四元数转换为 geometry_msgs::Quaternion 并设置到 marker 中
    marker.pose.orientation = tf2::toMsg(quat);
    // 将 Marker 发布到 ROS 主题
    publisher_ellipsoid.publish(marker);
}

void Visualize_Tools::visualize_ellipsoid(double x, double y, double z, double a, double b, double c, double roll, double pitch, double yaw, std::string frame_id, double id) {

    // 创建 Marker 消息
    visualization_msgs::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = ros::Time::now();
    marker.ns = "ellipsoid_visualization";
    marker.id = id;  // 唯一的 ID
    marker.type = visualization_msgs::Marker::SPHERE;  // 使用 SPHERE 类型表示椭球体
    marker.action = visualization_msgs::Marker::ADD;

    // 设置椭球体的位置
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = z;

    // 椭球体的比例尺寸，a, b, c 分别为 x, y, z 方向的轴长度
    marker.scale.x = a;  // x 轴长度
    marker.scale.y = b;  // y 轴长度
    marker.scale.z = c;  // z 轴长度

    // 设置颜色 (RGBA)
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;  // 绿色
    marker.color.b = 0.0f;
    marker.color.a = 0.8f;  // 透明度

    // 创建四元数并设置旋转（Roll, Pitch, Yaw）
    tf2::Quaternion quat;
    quat.setRPY(roll, pitch, yaw);  // 设置旋转角度 (弧度)

    // 将四元数转换为 geometry_msgs::Quaternion 并设置到 marker 中
    marker.pose.orientation = tf2::toMsg(quat);
    // 将 Marker 发布到 ROS 主题
    publisher_ellipsoid.publish(marker);
}
//






































Visualize_Arm_Tools::Visualize_Arm_Tools(ros::NodeHandle& nh, const Robot& robot, moveit::planning_interface::MoveGroupInterface& move_group, int width, int height, Eigen::Matrix3d calib, std::string default_frame )
       : default_frame_(default_frame), move_group_(move_group), robot_(robot),
                miImageCols(width),
                miImageRows(height),
                mCalib(calib)
{
    // collision_spheres_pub = nh.advertise<visualization_msgs::Marker>("collision_spheres", 1);
    arm_link_spheres_pub = nh.advertise<visualization_msgs::Marker>("arm_spheres", 1);
    // bbox_plane_pub = nh.advertise<visualization_msgs::Marker>("bbox_plane", 1);
    collision_spheres_pub = nh.advertise<visualization_msgs::MarkerArray>("collision_spheres", 10);
    bbox_plane_pub = nh.advertise<visualization_msgs::MarkerArray>("endpoint_bbox_plane", 10);

    mRobotPose = Eigen::Matrix4f::Identity();
}



void Visualize_Arm_Tools::Run() {
    //         ros::AsyncSpinner spinner(1);
    //         spinner.start();
    //         moveit::planning_interface::MoveGroupInterface move_group("arm");
    //         std::string end_effector_link=move_group.getEndEffectorLink();
    //
    //         std::string pose_reference_frame="/wam/base_link";
    //         // std::string pose_reference_frame="world";
    //         move_group.setPoseReferenceFrame(pose_reference_frame);

    // geometry_msgs::PoseStamped end_pose = move_group.getCurrentPose();
    //         const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup("arm");

    ros::Rate r(100);

    while (1) {
        std::vector<double> joint_angles = move_group_.getCurrentJointValues();
        Eigen::Matrix<double, 6, 1> config;
        config << joint_angles[0], -1*joint_angles[1], -1*joint_angles[2], joint_angles[3], -1*joint_angles[4], joint_angles[5];

        std::cout<< "   [zhjd-debug] Current joint angles: " << config.transpose() <<std::endl;

        publish_collision_spheres(config);
        publish_arm_link_spheres(config);
        visualize_plane_triangle_bypoint(config);
        r.sleep();
    }
}


//发布机械臂的球体

// void Visualize_Arm_Tools::publish_collision_spheres(const Eigen::Matrix<double, 6, 1> &conf) {
//     // run forward kinematics of this configuration
//     std::vector<Point3> sph_centers;
//     std::vector<gtsam::Matrix> J_px_jp;
//     robot_.sphereCenters(conf, sph_centers);

//     // for each point on arm stick, get error
//     for (int sph_idx = 0; sph_idx < robot_.nr_body_spheres(); sph_idx++) {
//         // 避障球 半径
//         const double total_eps = robot_.sphere_radius(sph_idx);
//         // 避障球 中心坐标
//         const gtsam::Point3 &point = sph_centers[sph_idx];

//         visualization_msgs::Marker marker;
//         marker.header.frame_id = default_frame_; // Assuming the frame_id is "base_link"
//         marker.header.stamp = ros::Time::now();
//         marker.ns = "collision_spheres";
//         marker.id = sph_idx;
//         marker.type = visualization_msgs::Marker::SPHERE;
//         marker.action = visualization_msgs::Marker::ADD;

//         // Set position
//         marker.pose.position.x = point.x();
//         marker.pose.position.y = point.y();
//         marker.pose.position.z = point.z();

//         // Set orientation
//         marker.pose.orientation.x = 0.0;
//         marker.pose.orientation.y = 0.0;
//         marker.pose.orientation.z = 0.0;
//         marker.pose.orientation.w = 1.0;

//         // Set scale (diameter)
//         marker.scale.x = total_eps * 2.0;
//         marker.scale.y = total_eps * 2.0;
//         marker.scale.z = total_eps * 2.0;

//         // Set color (red)
//         marker.color.a = 0.7;
//         marker.color.r = 1.0;
//         marker.color.g = 1.0;
//         marker.color.b = 0.0;

//         marker.lifetime = ros::Duration(); // Persistent marker

//         collision_spheres_pub.publish(marker);
//     }
// }

// 修改发布函数
void Visualize_Arm_Tools::publish_collision_spheres(const Eigen::Matrix<double, 6, 1> &conf) {
    // run forward kinematics of this configuration
    std::vector<Point3> sph_centers;
    robot_.sphereCenters(conf, sph_centers);

    // 创建 MarkerArray
    visualization_msgs::MarkerArray marker_array;

    // for each point on arm stick, get error
    for (int sph_idx = 0; sph_idx < robot_.nr_body_spheres(); sph_idx++) {
        // 避障球 半径
        const double total_eps = robot_.sphere_radius(sph_idx);
        // 避障球 中心坐标
        const gtsam::Point3 &point = sph_centers[sph_idx];

        visualization_msgs::Marker marker;
        marker.header.frame_id = default_frame_;
        marker.header.stamp = ros::Time::now();
        marker.ns = "collision_spheres";
        marker.id = sph_idx;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;

        // Set position
        marker.pose.position.x = point.x();
        marker.pose.position.y = point.y();
        marker.pose.position.z = point.z();

        // Set orientation
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        // Set scale (diameter)
        marker.scale.x = total_eps * 2.0;
        marker.scale.y = total_eps * 2.0;
        marker.scale.z = total_eps * 2.0;

        // Set color (yellow)
        marker.color.a = 0.7;
        marker.color.r = 1.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;

        marker.lifetime = ros::Duration(); // Persistent marker

        // 添加到数组
        marker_array.markers.push_back(marker);
    }

    // 一次性发布所有 marker
    collision_spheres_pub.publish(marker_array);
}


void Visualize_Arm_Tools::publish_arm_link_spheres(const Eigen::Matrix<double, 6, 1> &conf) {
    // run forward kinematics of this configuration
    // 机械臂endlink的位姿
    std::vector<Pose3> joint_pos; //  link poses in 3D work space
    std::vector<gtsam::Matrix> J_jpx_jp; //  et al. optional Jacobians
    robot_.fk_model().forwardKinematics(conf, {}, joint_pos);

    // for each point on arm stick, get error
    for (int sph_idx = 0; sph_idx < joint_pos.size(); sph_idx++) {
        // 各关节末端 中心坐标
        double x = joint_pos[sph_idx].x();
        double y = joint_pos[sph_idx].y();
        double z = joint_pos[sph_idx].z();

        double total_eps = 0.08;

        visualization_msgs::Marker marker;
        marker.header.frame_id = default_frame_; // Assuming the frame_id is "base_link"
        marker.header.stamp = ros::Time::now();
        marker.ns = "arm_link_spheres";
        marker.id = sph_idx;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;

        // Set position
        marker.pose.position.x = x;
        marker.pose.position.y = y;
        marker.pose.position.z = z;

        // Set orientation
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        // Set scale (diameter)
        marker.scale.x = total_eps * 2.0;
        marker.scale.y = total_eps * 2.0;
        marker.scale.z = total_eps * 2.0;

        // Set color (red)
        marker.color.a = 0.7;
        marker.color.r = .0;
        marker.color.g = .0;
        marker.color.b = 1.0;

        marker.lifetime = ros::Duration(); // Persistent marker

        arm_link_spheres_pub.publish(marker);
    }
}


//发布相机视场

geometry_msgs::Point Visualize_Arm_Tools::pixelToCamera(const Eigen::Vector2d &pixel) {
    // 获取相机内参矩阵的元素
    double fx = mCalib(0, 0); // 焦距 fx
    double fy = mCalib(1, 1); // 焦距 fy
    double cx = mCalib(0, 2); // 光心 cx
    double cy = mCalib(1, 2); // 光心 cy

    // 提取像素坐标 u, v
    double u = pixel(0);
    double v = pixel(1);

    // 将像素坐标转换为相机坐标系下的三维坐标
    double x = (u - cx) * mDepth / fx;
    double y = (v - cy) * mDepth / fy;
    double z = mDepth;

    // 相机坐标系下的三维点
    geometry_msgs::Point p;
    p.x = x;
    p.y = y;
    p.z = z;

    // geometry_msgs::Point p2 = transformPoint(camera_pose, p);

    return p;
}

geometry_msgs::Point Visualize_Arm_Tools::transformPointToWorld(const Eigen::Matrix4f &T_world_camera,
                                           const geometry_msgs::Point &point_camera) {
    // 将相机坐标点转换为 4D 齐次坐标形式 (x, y, z, 1)
    Eigen::Vector4f point_camera_homo(point_camera.x, point_camera.y, point_camera.z, 1.0);

    // 使用变换矩阵将相机坐标系中的点转换为世界坐标系
    Eigen::Vector4f point_world_homo = T_world_camera * point_camera_homo;

    // 将结果转换为 geometry_msgs::Point，并返回
    geometry_msgs::Point point_world;
    point_world.x = point_world_homo(0);
    point_world.y = point_world_homo(1);
    point_world.z = point_world_homo(2);

    return point_world;
}

std::vector<std::vector<geometry_msgs::Point> > Visualize_Arm_Tools::GenerateBboxPlanesTrianglePoints(const typename Robot::Pose &conf) {
    // 1. ROBOT pose
    // 相机相对于endlink的位姿
    Eigen::Matrix4f T_endlink_to_c;
    T_endlink_to_c <<   0, 0, -1, 0.02,
                        1, 0, 0, -0.013,
                        0, -1, 0, 0.07, //实际为0.13，改为0.07
                        0, 0, 0, 1;
    // 机械臂endlink的位姿
    std::vector<Pose3> joint_pos; //  link poses in 3D work space
    std::vector<gtsam::Matrix> J_jpx_jp; //  et al. optional Jacobians
    robot_.fk_model().forwardKinematics(conf, {}, joint_pos);
    Pose3 pose_end_link = joint_pos[joint_pos.size() - 1];
    // 将 gtsam::Pose3 转换为 Eigen::Matrix4f
    Eigen::Matrix4f T_baselink_endlink = Eigen::Matrix4f::Identity(); // 创建 4x4 单位矩阵
    // 获取 gtsam::Pose3 的 3x3 旋转矩阵并赋值到 eigenMatrix 的左上角
    T_baselink_endlink.block<3, 3>(0, 0) = pose_end_link.rotation().matrix().cast<float>();
    // 获取 gtsam::Pose3 的 3x1 平移向量并赋值到 eigenMatrix 的右侧
    T_baselink_endlink.block<3, 1>(0, 3) << pose_end_link.x(), pose_end_link.y(), pose_end_link.z();
    Eigen::Matrix4f T_world_2_c = mRobotPose * T_baselink_endlink * T_endlink_to_c;


    // 2. 生成相机坐标系下的三维点
    double s = mFOV_decrease;
    // std::cout<<"[debug] mFOV_decrease: "<<mFOV_decrease<<std::endl;
    Eigen::Vector4d temp_bbox(0 + s, 0 + s * miImageRows / miImageCols, miImageCols - s,
                              miImageRows - s * miImageRows / miImageCols);

    Eigen::Vector4d bbox = temp_bbox;

    geometry_msgs::Point centor_3d;
    centor_3d.x = T_world_2_c(0, 3);
    centor_3d.y = T_world_2_c(1, 3);
    centor_3d.z = T_world_2_c(2, 3);

    double x1 = bbox(0);
    double y1 = bbox(1);
    double x2 = bbox(2);
    double y2 = bbox(3);

    // 1----2
    // |    |
    // 3----4

    Eigen::Vector2d corner1(x1, y1);
    Eigen::Vector2d corner2(x2, y1);
    Eigen::Vector2d corner3(x1, y2);
    Eigen::Vector2d corner4(x2, y2);

    // 相机坐标系下的三维点
    auto corner1_3d = transformPointToWorld(T_world_2_c, pixelToCamera(corner1));
    auto corner2_3d = transformPointToWorld(T_world_2_c, pixelToCamera(corner2));
    auto corner3_3d = transformPointToWorld(T_world_2_c, pixelToCamera(corner3));
    auto corner4_3d = transformPointToWorld(T_world_2_c, pixelToCamera(corner4));
    // std::cout<<"[debug] corner1_3d: "<<corner1_3d<<std::endl;
    // std::cout<<"[debug] corner2_3d: "<<corner2_3d<<std::endl;
    // std::cout<<"[debug] corner3_3d: "<<corner3_3d<<std::endl;
    // std::cout<<"[debug] corner4_3d: "<<corner4_3d<<std::endl;

    std::vector<std::vector<geometry_msgs::Point> > planes(4, std::vector<geometry_msgs::Point>(3));
    planes[0].push_back(centor_3d);
    planes[0].push_back(corner1_3d);
    planes[0].push_back(corner3_3d);
    planes[1].push_back(centor_3d);
    planes[1].push_back(corner1_3d);
    planes[1].push_back(corner2_3d);
    planes[2].push_back(centor_3d);
    planes[2].push_back(corner2_3d);
    planes[2].push_back(corner4_3d);
    planes[3].push_back(centor_3d);
    planes[3].push_back(corner4_3d);
    planes[3].push_back(corner3_3d);

    return planes;
}


// 以三角的形式显示平面
void Visualize_Arm_Tools::visualize_plane_triangle_bypoint(const typename Robot::Pose &conf) {

    std::vector<std::vector<geometry_msgs::Point>> BboxPlanesTrianglePointsInWorld =
            GenerateBboxPlanesTrianglePoints(conf);

    // 创建 MarkerArray
    visualization_msgs::MarkerArray marker_array;

    for (int i = 0; i < BboxPlanesTrianglePointsInWorld.size(); i++) {
        auto points = BboxPlanesTrianglePointsInWorld[i];

        visualization_msgs::Marker marker;
        marker.header.frame_id = default_frame_;
        marker.header.stamp = ros::Time::now();
        marker.ns = "endpoint_bbox_plane";
        marker.id = i;
        marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
        marker.action = visualization_msgs::Marker::ADD;

        // 设置颜色和透明度
        marker.color.a = 0.2;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;

        // 设置比例
        marker.scale.x = 1.0;
        marker.scale.y = 1.0;
        marker.scale.z = 1.0;

        marker.points = points;

        // 添加到数组，而不是直接发布
        marker_array.markers.push_back(marker);
    }

    // 一次性发布所有平面
    bbox_plane_pub.publish(marker_array);
}