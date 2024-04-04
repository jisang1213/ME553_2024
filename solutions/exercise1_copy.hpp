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
    Joint *parent = NULL;
    Joint *child = NULL;
    double axis;
    double angle;
    bool rev = false;
    bool isBase = false;

    Eigen::Matrix3d getRot(){
      return Rot();
    }

    Joint(){
      rpy.setZero();
      xyz.setZero();
    }

    Eigen::Vector3d getPos(){
      return parent->getPos_helper(xyz);
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

    Eigen::Vector3d getPos_helper(Eigen::Vector3d pos){
      if(isBase){
        return pos;
      }
      return parent->getPos_helper(xyz + getRot()*pos);
    }

};

// //Recursive function to get end-effector position relative to baselink in robot frame
// Eigen::Vector3d getPos(Joint *joint, Eigen::Vector3d pos){
//   if(joint->isBase){
//     return pos;
//   }
//   return getPos(joint->parent, joint->xyz + joint->getRot() * pos);
// }


/// do not change the name of the method
inline Eigen::Vector3d getEndEffectorPosition (const Eigen::VectorXd& gc) {

  Joint base, joint1, joint2, joint3, joint4, joint5, joint6, joint7, joint8, joint9, EndEffector; //create joint objects on the stack

  base.name = "base";
  base.isBase = true;

  joint1.name = "base_LH_HAA";
  joint1.rpy << -2.61799387799, 0, -3.14159265359;
  joint1.xyz << -0.2999, 0.104, 0.0;
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
  joint4.xyz << -0.0599, 0.08381, 0.0;
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
  joint7.xyz << -0.0, 0.1003, -0.285;
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
  EndEffector.xyz << -0.08795, 0.01305, -0.33797;
  EndEffector.parent = &joint9;

  Eigen::Quaterniond q(gc[3], gc[4], gc[5], gc[6]);
  Eigen::Matrix3d orien = q.toRotationMatrix();

  Eigen::Vector3d pos;
  pos.setZero();

  return gc.head(3) + orien * EndEffector.getPos();
}

#endif // ME553_2022_SOLUTIONS_EXERCISE1_STUDENTID_HPP_
