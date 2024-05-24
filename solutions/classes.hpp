class Kinematic_tree{
  public:
    Origin origin;
    bool isrev = false;
    double axis, angle;
    Joint *parentJoint = NULL;
    //this method returns the rotation matrix of the current joint
    Eigen::Matrix3d getRot(){
      return Rot();
    }
    void setRot(Eigen::Matrix3d rot){
      origin.orien = rot;
      userotmat = true;
    }
    Kinematic_tree(){
      origin.rpy.setZero();
      origin.xyz.setZero();
      origin.orien.setZero();
    }
  
  private:
    Eigen::Matrix3d Rot(){
      Eigen::Matrix3d Rx, Ry, Rz;
      double roll, pitch, yaw;
      if(userotmat){
        return origin.orien;
      }
      if(isrev){
        roll=axis*angle; pitch=0.; yaw=0.;
      }
      else{
        roll=origin.rpy[0]; pitch=origin.rpy[1]; yaw=origin.rpy[2];
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
    bool userotmat = false; //toggle true if origin.orien is assigned
};

class Link;
class Joint : public Kinematic_tree{
  public:
    Eigen::Vector3d w_dir, w_dir_dot;   //axis direction vector in world frame
    Eigen::Vector3d w_pos;  //position of joint in world frame
    Eigen::Vector3d w_lin_vel, w_ang_vel;
    Eigen::VectorXd accel;
    std::vector<Joint*> childjoints;
    std::vector<Link*> childlinks;
    bool isBase = false;
    int jointID = -1;

    struct Inertial_body{
        Eigen::MatrixXd M = Eigen::MatrixXd::Zero(6,6);
        Eigen::VectorXd b = Eigen::VectorXd::Zero(6);
    } RB, AB; //stores M and b for rigid body and articulated body


    //this method returns the 6D motion subspace vector of the joint
    Eigen::VectorXd S(){
      Eigen::VectorXd subspace(6);
      subspace << Eigen::VectorXd::Zero(3), w_dir;
      return subspace;
    }
    //this method returns the time derivative of the 6D motion subspace vector of the joint
    Eigen::VectorXd Sdot(){
      Eigen::VectorXd sdot(6);
      sdot << Eigen::VectorXd::Zero(3), w_dir_dot;
      return sdot;
    }
    //this method returns the twist of the joint
    Eigen::VectorXd getTwist(){
      Eigen::VectorXd twist(6);
      twist << w_lin_vel, w_ang_vel;
      return twist;
    }

    Joint(){
      w_dir.setZero();
      w_dir_dot.setZero();
      w_pos.setZero();
      w_lin_vel.setZero();
      w_ang_vel.setZero();
      accel.resize(6);
      accel.setZero();
    }
};

class Link : public Kinematic_tree{
  public:
    bool isleaf = false;
    Eigen::Matrix3d inertia_b, inertia_w;
    Eigen::Vector3d COM; //com in world frame
    double mass;

    rigidbody RB(){
      return rigidbody{inertia_w, COM, mass};
    }

    Link(){
      inertia_b.setZero();
      inertia_w.setZero();
      COM.setZero();
    }
};