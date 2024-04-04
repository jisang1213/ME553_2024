//
// Created by Jemin Hwangbo on 2022/03/17.
//

#ifndef ME553_2022_SOLUTIONS_EXERCISE1_STUDENTID_HPP_
#define ME553_2022_SOLUTIONS_EXERCISE1_STUDENTID_HPP_

#include <Eigen/Core>
#include <Eigen/Geometry>

class Joint{
  public:
    const char *name;
    Eigen::Vector3d rpy, xyz;
    Joint *child = NULL;
    double axis;
    double angle;
    bool rev = false;
    bool isEndEffector = false;

    Eigen::Matrix3d getRot(){
      return Rot();
    }

    Joint(){
      rpy.setZero();
      xyz.setZero();
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

//Recursive function to get end-effector position relative to baselink in robot frame
Eigen::Vector3d getPos(Joint *joint){
  if(joint->isEndEffector){
    return joint->xyz;
  }
  return joint->xyz + joint->getRot() * getPos(joint->child);
}

/// do not change the name of the method
inline Eigen::Vector3d getEndEffectorPosition (const Eigen::VectorXd& gc) {

  Joint joint1, joint2, joint3, joint4, joint5, joint6, joint7, joint8, joint9, EndEffector; //create joint objects on the stack

  joint1.name = "base_LH_HAA";
  joint1.rpy << -2.61799387799, 0, -3.14159265359;
  joint1.xyz << -0.2999, 0.104, 0.0;
  joint1.child = &joint2;

  joint2.name = "LH_HAA";
  joint2.rev = true;
  joint2.axis = -1;
  joint2.angle = gc[13];
  joint2.child = &joint3;

  joint3.name = "LH_HIP_LH_hip_fixed";
  joint3.rpy << -2.61799387799, 0, -3.14159265359;
  joint3.child = &joint4;

  joint4.name = "LH_hip_fixed_LH_HFE";
  joint4.rpy << 0, 0, 1.57079632679;
  joint4.xyz << -0.0599, 0.08381, 0.0;
  joint4.child = &joint5;

  joint5.name = "LH_HFE";
  joint5.rev = true;
  joint5.axis = 1;
  joint5.angle = gc[14];
  joint5.child = &joint6;

  joint6.name = "LH_THIGH_LH_thigh_fixed";
  joint6.rpy << 0, 0, -1.57079632679;
  joint6.child = &joint7;

  joint7.name = "LH_thigh_fixed_LH_KFE";
  joint7.rpy << 0, 0, 1.57079632679;
  joint7.xyz << -0.0, 0.1003, -0.285;
  joint7.child = &joint8;

  joint8.name = "LH_KFE";
  joint8.rev = true;
  joint8.axis = 1;
  joint8.angle = gc[15];
  joint8.child = &joint9;

  joint9.name = "LH_shank_LH_shank_fixed";
  joint9.rpy << 0, 0, -1.57079632679;
  joint9.child = &EndEffector;

  EndEffector.name = "LH_shank_fixed_LH_FOOT";
  EndEffector.xyz << -0.08795, 0.01305, -0.33797;
  EndEffector.isEndEffector = true;

  Eigen::Quaterniond q(gc[3], gc[4], gc[5], gc[6]);
  Eigen::Matrix3d orien = q.toRotationMatrix();

  return gc.head(3) + orien * getPos(&joint1);
}

#endif // ME553_2022_SOLUTIONS_EXERCISE1_STUDENTID_HPP_
