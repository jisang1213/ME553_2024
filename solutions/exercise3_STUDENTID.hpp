#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>

static double mass[13];
static Eigen::Matrix3d inertia_w[13];
static Eigen::Vector3d pos[13];
static Eigen::MatrixXd massmatrix(rows, cols);

typedef struct{
  Eigen::Vector3d xyz;
  Eigen::Vector3d rpy;
} Origin;

class Joint{
  public:
    const char *name;
    Origin origin;
    Joint *parent = NULL;
    Joint *child = NULL;
    std::vector<Link*> childlinks;
    double axis, angle;
    bool rev = false;
    bool isBase = false;
    int jointID = -1;

    Eigen::Matrix3d getRot(){
      return Rot();
    }

    Joint(){
      origin.rpy.setZero();
      origin.xyz.setZero();
    }

  private:
    Eigen::Matrix3d Rot(){
      Eigen::Matrix3d Rx, Ry, Rz;
      double roll, pitch, yaw;
      if(rev){
        roll=axis*angle; pitch=0.; yaw=0.;
      }
      else{
        roll=rpy[0]; pitch=rpy[1]; yaw=rpy[2];
      }
      Rx << 1, 0, 0,
            0, cos(roll), -sin(roll),
            0, sin(roll), cos(roll);
      Ry << cos(pitch), 0, sin(pitch),
            0, 1, 0,
            -sin(pitch), 0, cos(pitch);
      Rz << cos(yaw), -sin(yaw), 0,
            sin(yaw), cos(yaw), 0,
            0, 0, 1;
    
      return Rz*Ry*Rx;
    }
};

class Link{
  public:
    const char *name;
    Origin origin;
    Joint *parentJoint = NULL;
    bool isleaf = false;
    Eigen::Matrix3d inertia_b, inertia_w;

    Eigen::Matrix3d getRot(){
      return Rot();
    }

    Link(){
      rpy.setZero();
      origin.setZero();
    }

  private:
    Eigen::Matrix3d Rot(){
      Eigen::Matrix3d Rx, Ry, Rz;
      double roll, pitch, yaw;
      if(rev){
        roll=axis*angle; pitch=0.; yaw=0.;
      }
      else{
        roll=rpy[0]; pitch=rpy[1]; yaw=rpy[2];
      }
      Rx << 1, 0, 0,
            0, cos(roll), -sin(roll),
            0, sin(roll), cos(roll);
      Ry << cos(pitch), 0, sin(pitch),
            0, 1, 0,
            -sin(pitch), 0, cos(pitch);
      Rz << cos(yaw), -sin(yaw), 0,
            sin(yaw), cos(yaw), 0,
            0, 0, 1;
    
      return Rz*Ry*Rx;
    }
};

CRBA(Joint* joint){
  //base case:
  if joint->childlink is leaf:



  for child in children:
    //down path:
    CRBA(joint->parent);
    //up path:


}

/// do not change the name of the method
inline Eigen::MatrixXd getMassMatrix (const Eigen::VectorXd& gc) {

  /// !!!!!!!!!! NO RAISIM FUNCTIONS HERE !!!!!!!!!!!!!!!!!

  Joint joint[20];
  Link link[20]

  link[0].name = "base_inertia";
  link[0].parentJoint = & ;
  link[0].origin.xyz << -0.018, -0.002, 0.024;
  link[0].mass = 6.222;
  link[0].inertia_b << 0.017938806, 0.00387963, 0.001500772, 
                        0.0, 0.370887745, 6.8963e-05, 
                        0.0, 0.0, 0.372497653;

  link[1].name = "LF_HAA";
  link[1].parentJoint = & ;
  link[1].origin.xyz << -0.063, 7e-05, 0.00046;
  link[1].mass = 2.04;
  link[1].inertia_b << 0.001053013, 4.527e-05, 8.855e-05, 
                    0.0, 0.001805509, 9.909e-05, 
                    0.0, 0.0, 0.001765827;

  link[2].name = "LF_HIP";
  link[2].parentJoint = & 
  link[2].mass = 0.001;
  link[2].inertia_b << 0.000001, 0.0, 0.0, 
                      0.0, 0.000001, 0.0, 
                      0.0, 0.0, 0.000001

  link[3].name = "LF_hip_fixed";
  link[3].parentJoint = & 
  link[3].origin.xyz << 0.048, 0.008, -0.003;
  link[3].mass = 0.74;
  link[3].inertia_b << 0.001393106, 8.4012e-05, 8.4012e-05, 
                      0.0, 0.003798579, 7.1319e-05, 
                      0.0, 0.0, 0.003897509
  
  link[4].name = "LF_HFE";
  link[4].parentJoint = & 
  link[4].origin.xyz << -0.063, 7e-05, 0.00046;
  link[4].mass = 2.04;
  link[4].inertia_b << 0.001053013, 4.527e-05, 8.855e-05, 
                      0.0, 0.001805509, 9.909e-05, 
                      0.0, 0.0, 0.001765827;

  link[5].name = "LF_THIGH";
  link[5].parentJoint = & ;
  link[5].mass = 0.001;
  link[5].inertia_b << 0.000001, 0.0, 0.0,
                      0.0, 0.000001, 0.0,
                      0.0, 0.0, 0.000001;

  link[6].name = "LF_thigh_fixed";
  link[6].parentJoint = & ;
  link[6].origin.xyz = 0.0, 0.018, -0.169;
  link[6].mass = 1.03;
  link[6].inertia_b << 0.018644469, 5.2e-08, 1.0157e-05, 
                      0.0, 0.019312599, 0.002520077,
                      0.0, 0.0, 0.002838361;

  link[7].name = "LF_KFE";
  link[7].parentJoint = & ;
  link[7].origin.xyz = -0.063, 7e-05, 0.00046;
  link[7].mass = 2.04;
  link[7].inertia_b << 0.001053013, 4.527e-05, 8.855e-05, 
                      0.0, 0.001805509, 0.0, 9.909e-05, 
                      0.0, 0.0, 0.001765827;

  link[8].name = "LF_SHANK";
  link[8].parentJoint = & ;
  link[8].mass = 0.001;
  link[8].inertia_b << 0.000001, 0.0, 0.0, 
                        0.0, 0.000001, 0.0, 
                        0.0, 0.0, 0.000001;


  link[9].name = "LF_shank_fixed";
  link[9].parentJoint = & ;
  link[9].origin.xyz << 0.03463, 0.00688, 0.0009;
  link[9].mass = 0.33742;
  link[9].inertia_b << 0.00032748005, 2.142561e-05, 1.33942e-05, 
  0.0, 0.00110974122, 7.601e-08, 
  0.0, 0.0, 0.00089388521;

  base.name = "base";
  base.isBase = true;

  joint1.name = "base_LH_HAA";
  joint1.rpy << -2.61799387799, 0, -3.14159265359;
  joint1.origin << -0.2999, 0.104, 0.0;
  joint1.parent = &base;

  joint2.name = "LH_HAA";
  joint2.rev = true;
  joint2.axis = -1;
  joint2.angle = gc[13];
  joint2.parent = &joint1;

  joint3.name = "LH_HIP_LH_hip_fixed";
  joint3.rpy << -2.61799387799, 0, -3.14159265359;
  joint3.parent= &joint2;

  joint4.name = "LH_hip_fixed_LH_HFE";
  joint4.rpy << 0, 0, 1.57079632679;
  joint4.origin << -0.0599, 0.08381, 0.0;
  joint4.parent = &joint3;

  joint5.name = "LH_HFE";
  joint5.rev = true;
  joint5.axis = 1;
  joint5.angle = gc[14];
  joint5.parent = &joint4;

  joint6.name = "LH_THIGH_LH_thigh_fixed";
  joint6.rpy << 0, 0, -1.57079632679;
  joint6.parent = &joint5;

  joint7.name = "LH_thigh_fixed_LH_KFE";
  joint7.rpy << 0, 0, 1.57079632679;
  joint7.origin << -0.0, 0.1003, -0.285;
  joint7.parent = &joint6;

  joint8.name = "LH_KFE";
  joint8.rev = true;
  joint8.axis = 1;
  joint8.angle = gc[15];
  joint8.parent = &joint7;

  joint9.name = "LH_shank_LH_shank_fixed";
  joint9.rpy << 0, 0, -1.57079632679;
  joint9.parent = &joint8;

  EndEffector.name = "LH_shank_fixed_LH_FOOT";
  EndEffector.origin << -0.08795, 0.01305, -0.33797;
  EndEffector.parent = &joint9;

  Eigen::Quaterniond q(gc[3], gc[4], gc[5], gc[6]);
  Eigen::Matrix3d orien = q.toRotationMatrix();


  return Eigen::MatrixXd::Ones(18,18);
}