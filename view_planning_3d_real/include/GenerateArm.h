#include <gpmp2/kinematics/Arm.h>
#include <gpmp2/kinematics/RobotModel.h>
#include <gpmp2/kinematics/ArmModel.h>

using namespace std;
using namespace gtsam;
using namespace gpmp2;

ArmModel* generateArm(string arm_type){
    ArmModel* arm_model;
    if(arm_type=="realman"){    //rm_75
        //1. 机械臂DH模型
        gtsam::Vector alpha(7);  alpha << -M_PI/2, M_PI/2, -M_PI/2, M_PI/2, -M_PI/2, M_PI/2, 0;
        gtsam::Vector a(7);      a << 0, 0, 0, 0, 0, 0, 0;
        gtsam::Vector d(7);      d << 0.2405, 0, 0.256, 0, 0.210, 0, 0.144;
        gtsam::Vector theta(7);  theta << 0, 0, 0, 0, 0, 0, 0;
        
        // base_pose = Pose3(Rot3(eye(3)), Point3([0,0,0]'));
        gtsam::Pose3 base_pose = gtsam::Pose3(gtsam::Rot3::RzRyRx(0, 0, 0), gtsam::Point3(0, 0, 0));
        
        // Arm(size_t dof, const gtsam::Vector& a, const gtsam::Vector& alpha, const gtsam::Vector& d, const gtsam::Pose3& base_pose, const gtsam::Vector& theta_bias);
        Arm abs_arm(7, a, alpha, d, base_pose, theta);

        // 2.用于碰撞检测的sphere
        std::vector<std::vector<double>> spheres_data = {
                        // 1  0.2405（0坐标系到1坐标系的距离）
                        {0, 0.0, 0.2405, 0.0, 0.15/2},  //基座，所以体积最大
                        {0, 0.0, 0.16, 0.0, 0.06/2},
                        {0, 0.0, 0.08, 0.0, 0.06/2},
                        {0, 0.0, 0.0, 0.0, 0.06/2},
                        // 2 0  
                        // 3 0.256
                        {2, 0.0, 0.0, 0.0, 0.06/2},
                        {2, 0.0, 0.05, 0.0, 0.06/2},
                        {2, 0.0, 0.1, 0.0, 0.06/2},
                        {2, 0.0, 0.15, 0.0, 0.06/2},
                        {2, 0.0, 0.2, 0.0, 0.06/2},
                        // 4 0
                        // 5 0.210  
                        {4, 0.0, 0.0, 0.0, 0.06/2},
                        {4, 0.0, 0.05, 0.0, 0.06/2},
                        {4, 0.0, 0.1, 0.0, 0.06/2},
                        {4, 0.0, 0.15, 0.0, 0.06/2},
                        // 6 0
                        // 7 0.144
                    };

        BodySphereVector body_spheres;
        body_spheres.clear();
        for(int i=0; i<spheres_data.size(); i++){
            BodySphere sphere(spheres_data[i][0], spheres_data[i][4], Point3(spheres_data[i][1], spheres_data[i][2], spheres_data[i][3]));
            body_spheres.push_back(sphere);
        }

        // 3.生成机械臂模型
        // ArmModel arm = ArmModel(Arm(2, a, alpha, d), BodySphereVector());
        arm_model = new ArmModel(abs_arm, body_spheres);
        std::cout<<"构建realman机械臂模型成功"<<std::endl;
    }
    else if(arm_type=="WAMArm-origin"){ //WAM
        //1. 机械臂DH模型
        gtsam::Vector alpha(7);  alpha << -M_PI/2, M_PI/2, -M_PI/2, M_PI/2, -M_PI/2, M_PI/2, 0;
        gtsam::Vector a(7);      a << 0, 0, 0.045, -0.045, 0, 0, 0;
        gtsam::Vector d(7);      d << 0,0,0.55,0,0.3,0,0.06;
        gtsam::Vector theta(7);  theta << 0, 0, 0, 0, 0, 0, 0;
        
        gtsam::Pose3 base_pose = gtsam::Pose3(gtsam::Rot3::RzRyRx(0, 0, 0), gtsam::Point3(0, 0, 0));
        
        // Arm(size_t dof, const gtsam::Vector& a, const gtsam::Vector& alpha, const gtsam::Vector& d, const gtsam::Pose3& base_pose, const gtsam::Vector& theta_bias);
        Arm abs_arm(7, a, alpha, d, base_pose, theta);

        // 2.用于碰撞检测的sphere
        std::vector<std::vector<double>> spheres_data = {
                        // 1  0.2405（0坐标系到1坐标系的距离）
                        {0, 0.0,  0.0,  0.0, 0.15},  //基座，所以体积最大
                        // 2 0  
                        {1, 0.0,  0.0,  0.2, 0.06},
                        {1, 0.0,  0.0,  0.3, 0.06},
                        {1, 0.0,  0.0,  0.4, 0.06},
                        {1, 0.0,  0.0,  0.5, 0.06},
                        // 3 0.256
                        {2, 0.0, 0.0, 0.0, 0.06},
                        // 4 0
                        {3, 0.0, 0.0, 0.1, 0.06},
                        {3, 0.0, 0.0, 0.2, 0.06},
                        {3, 0.0, 0.0, 0.3, 0.06},
                        // 5 0.210  
                        // 6 0
                        {5, 0.0, 0.0, 0.1, 0.01},
                        // 7 0.144
                        {6, 0.1,  -0.025, 0.08, 0.04},
                        {6, 0.1,  0.025, 0.08, 0.04},
                        {6, -0.1,  0, 0.08, 0.04},
                        {6, 0.15, -0.025, 0.13, 0.04},
                        {6, 0.15,  0.025, 0.13, 0.04},
                        {6, -0.15,  0,     0.13, 0.04}

                    };

        BodySphereVector body_spheres;
        body_spheres.clear();
        for(int i=0; i<spheres_data.size(); i++){
            BodySphere sphere(spheres_data[i][0], spheres_data[i][4], Point3(spheres_data[i][1], spheres_data[i][2], spheres_data[i][3]));
            body_spheres.push_back(sphere);
        }

        // 3.生成机械臂模型
        // ArmModel arm = ArmModel(Arm(2, a, alpha, d), BodySphereVector());
        arm_model = new ArmModel(abs_arm, body_spheres);
        std::cout<<"构建WAMArm机械臂模型成功"<<std::endl;
    }
    else if(arm_type=="WAMArm"){ //WAM
        //1. 机械臂DH模型
        gtsam::Vector alpha(7);  alpha << -M_PI/2, M_PI/2, -M_PI/2, M_PI/2, -M_PI/2, M_PI/2, 0;
        gtsam::Vector a(7);      a << 0, 0, 0.045, -0.045, 0, 0, 0;
        gtsam::Vector d(7);      d << 0.345,0,0.55,0,0.3,0,0.06;
        gtsam::Vector theta(7);  theta << 0, 0, 0, 0, 0, 0, 0;
        
        gtsam::Pose3 base_pose = gtsam::Pose3(gtsam::Rot3::RzRyRx(0, 0, 0), gtsam::Point3(0, 0, 0));
        
        // Arm(size_t dof, const gtsam::Vector& a, const gtsam::Vector& alpha, const gtsam::Vector& d, const gtsam::Pose3& base_pose, const gtsam::Vector& theta_bias);
        Arm abs_arm(7, a, alpha, d, base_pose, theta);

        // 2.用于碰撞检测的sphere
        std::vector<std::vector<double>> spheres_data = {
                        // 1  0.3405（0坐标系到1坐标系的距离）
                        {0, 0.0,  0.0,  0.0, 0.15},  //基座2，所以体积最大
                        // 2 0  
                        {1, 0.0,  0.0,  0.2, 0.06},
                        {1, 0.0,  0.0,  0.3, 0.06},
                        {1, 0.0,  0.0,  0.4, 0.06},
                        {1, 0.0,  0.0,  0.5, 0.06},
                        // 3 0.256
                        {2, 0.0, 0.0, 0.0, 0.06},
                        // 4 0
                        {3, 0.0, 0.0, 0.1, 0.06},
                        {3, 0.0, 0.0, 0.2, 0.06},
                        {3, 0.0, 0.0, 0.3, 0.06},
                        // 5 0.210  
                        // 6 0
                        // {5, 0.0, 0.0, 0.1, 0.06},
                        // 7 0.144
                        // 以下为爪持爪
                        // {6, 0.1,  -0.025, 0.08, 0.14},
                        // {6, 0.1,  0.025, 0.08, 0.14},
                        // {6, -0.1,  0, 0.08, 0.14},
                        // {6, 0.15, -0.025, 0.13, 0.14},
                        // {6, 0.15,  0.025, 0.13, 0.14},
                        // {6, -0.15,  0,     0.13, 0.14}

                    };

        BodySphereVector body_spheres;
        body_spheres.clear();
        for(int i=0; i<spheres_data.size(); i++){
            BodySphere sphere(spheres_data[i][0], spheres_data[i][4], Point3(spheres_data[i][1], spheres_data[i][2], spheres_data[i][3]));
            body_spheres.push_back(sphere);
        }

        // 3.生成机械臂模型
        // ArmModel arm = ArmModel(Arm(2, a, alpha, d), BodySphereVector());
        arm_model = new ArmModel(abs_arm, body_spheres);
        std::cout<<"构建WAMArm机械臂模型成功"<<std::endl;
    }
    else
    {
        std::cout<<"未知的机械臂类型"<<std::endl;
    }
    return arm_model;
}