#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

class Joint{
  public:
    const char *name;
    Eigen::Vector3d rpy, xyz;
    Eigen::Quaterniond base_orien;     //this has to be passed to the base joint
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

    //function to get the positional Jacobian of the current joint
    Eigen::MatrixXd getJacob_p(){
      Eigen::MatrixXd jacob(3, 0);

      jacob = parent->fill_columns_p(jacob, xyz);

      //shift right 3 columns to make room for I
      jacob.conservativeResize(3, jacob.cols() + 3);
      Eigen::MatrixXd temp = jacob.block(0, 0, 3, jacob.cols() - 3);
      jacob.block(0, 3, 3, jacob.cols() - 3) = temp;
      jacob.block(0, 0, 3, 3) = Eigen::Matrix3d::Identity();
    
      return jacob;
    }

    //function to get the angular Jacobian of the current joint
    Eigen::MatrixXd getJacob_a(){
      Eigen::MatrixXd jacob(3, 0);

      jacob = parent->fill_columns_a(jacob);

      //shift right 3 columns to make room for I
      jacob.conservativeResize(3, jacob.cols() + 3);
      Eigen::MatrixXd temp = jacob.block(0, 0, 3, jacob.cols() - 3);
      jacob.block(0, 3, 3, jacob.cols() - 3) = temp;
      jacob.block(0, 0, 3, 3) = Eigen::Matrix3d::Identity();

      //shift right 3 columns to make room for 03x3
      jacob.conservativeResize(3, jacob.cols() + 3);
      temp = jacob.block(0, 0, 3, jacob.cols() - 3);
      jacob.block(0, 3, 3, jacob.cols() - 3) = temp;
      jacob.block(0, 0, 3, 3) = Eigen::Matrix3d::Zero();
    
      return jacob;
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

    //this fills the joint dependent columns of the positional Jacobian (everything except identity)
    Eigen::MatrixXd fill_columns_p(Eigen::MatrixXd jac, Eigen::Vector3d r){  //pos is the position of the end effector in the current joint frame
      Eigen::MatrixXd jacob = jac;
      if(isBase){
        Eigen::Matrix3d base_rot = base_orien.toRotationMatrix(); 
        jacob = base_rot*jacob;

        Eigen::Vector3d r_w = base_rot*r; //this transforms r(vector from base to end) into the world frame
        Eigen::Matrix3d neg_rskew;
        neg_rskew <<  0,      r_w.z(), -r_w.y(),
                      -r_w.z(), 0,     r_w.x(), 
                      r_w.y(), -r_w.x(), 0; 
        //this is -rx where rx is the skew symmetric matrix of r

        //now append this to the left of the Jacobian
        jacob.conservativeResize(3, jacob.cols() + 3);
        Eigen::MatrixXd temp = jacob.block(0, 0, 3, jacob.cols() - 3);
        jacob.block(0, 3, 3, jacob.cols() - 3) = temp;
        jacob.block(0, 0, 3, 3) = neg_rskew;

        return jacob;
      }

      if(rev){
        Eigen::Vector3d p(axis, 0.0, 0.0);
        Eigen::Vector3d V = p.cross(r);

        // Resize the matrix to have one more column to the left to make room for V
        jacob.conservativeResize(3, jacob.cols() + 1);
        Eigen::MatrixXd temp = jacob.block(0, 0, 3, jacob.cols() - 1);
        jacob.block(0, 1, 3, jacob.cols() - 1) = temp;
        // Insert V at the leftmost column
        jacob.block(0, 0, 3, 1) = V;
      }

      //Update frame of previous V vectors to parent frame of current joint
      jacob = getRot()*jacob;

      return parent->fill_columns_p(jacob, xyz + getRot()*r);
    }

  //this fills the joint dependent columns of the angular Jacobian (everything except identity and 03x3)
  Eigen::MatrixXd fill_columns_a(Eigen::MatrixXd jac){  //pos is the position of the end effector in the current joint frame
    Eigen::MatrixXd jacob = jac;
    if(isBase){
      Eigen::Matrix3d base_rot = base_orien.toRotationMatrix(); 
      jacob = base_rot*jacob;

      return jacob;
    }

    if(rev){
      Eigen::Vector3d p(axis, 0.0, 0.0);

      // Resize the matrix to have one more column to the left to make room for V
      jacob.conservativeResize(3, jacob.cols() + 1);
      Eigen::MatrixXd temp = jacob.block(0, 0, 3, jacob.cols() - 1);
      jacob.block(0, 1, 3, jacob.cols() - 1) = temp;
      // Insert p at the leftmost column
      jacob.block(0, 0, 3, 1) = p;
    }

    //Update frame of previous V vectors to parent frame of current joint
    jacob = getRot()*jacob;

    return parent->fill_columns_a(jacob);
  }
};


/// do not change the name of the method
inline Eigen::Vector3d getFootLinearVelocity (const Eigen::VectorXd& gc, const Eigen::VectorXd& gv) {

  Joint base, joint1, joint2, joint3, joint4, joint5, joint6, joint7, joint8, joint9, EndEffector; //create joint objects on the stack
  Eigen::VectorXd genv(9);
  Eigen::Vector3d vel;

  base.name = "base";
  base.isBase = true;
  base.base_orien = Eigen::Quaterniond(gc[3], gc[4], gc[5], gc[6]);

  joint1.name = "base_LH_HAA";
  joint1.rpy << -2.61799387799, 0, -3.14159265359;
  joint1.xyz << -0.2999, 0.104, 0.0;
  joint1.parent = &base;

  joint2.name = "LH_HAA";
  joint2.rev = true;
  joint2.axis = -1.;
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
  joint5.axis = 1.;
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
  joint8.axis = 1.;
  joint8.angle = gc[15];
  joint8.parent = &joint7;

  joint9.name = "LH_shank_LH_shank_fixed";
  joint9.rpy << 0, 0, -1.57079632679;
  joint9.parent = &joint8;

  EndEffector.name = "LH_shank_fixed_LH_FOOT";
  EndEffector.xyz << -0.08795, 0.01305, -0.33797;
  EndEffector.parent = &joint9;

  genv << gv.head(6), gv.segment(12,3);

  vel = EndEffector.getJacob_p()*genv;

  return vel;/// replace this
}


/// do not change the name of the method
inline Eigen::Vector3d getFootAngularVelocity (const Eigen::VectorXd& gc, const Eigen::VectorXd& gv) {

  Joint base, joint1, joint2, joint3, joint4, joint5, joint6, joint7, joint8, joint9, EndEffector; //create joint objects on the stack
  Eigen::VectorXd genv(9);
  Eigen::Vector3d angvel;

  base.name = "base";
  base.isBase = true;
  base.base_orien = Eigen::Quaterniond(gc[3], gc[4], gc[5], gc[6]);

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

  genv << gv.head(6), gv.segment(12,3);

  angvel = EndEffector.getJacob_a()*genv;

  return angvel; /// replace this
}