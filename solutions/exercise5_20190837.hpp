#pragma once


#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>
#include <iostream>

typedef struct{
  Eigen::Matrix3d orien;
  Eigen::Vector3d xyz;
  Eigen::Vector3d rpy;
} Origin; //stores the origin from the URDF file

typedef struct{
  Eigen::Matrix3d I;
  Eigen::Vector3d COM;
  double m;
} rigidbody; //composite body inertia, COM, and mass

class Link;
class Joint{
  public:
    Origin origin;
    Eigen::Vector3d w_dir, w_dir_dot;   //axis direction vector in world frame
    Eigen::Vector3d w_pos;  //position of joint in world frame
    Eigen::Vector3d w_lin_vel, w_ang_vel;
    Joint *parentJoint = NULL;
    std::vector<Joint*> childjoints;
    std::vector<Link*> childlinks;
    double axis, angle;
    bool isrev = false;
    bool isBase = false;
    int jointID = -1;

    struct acceleration{
      Eigen::Vector3d lin = Eigen::Vector3d::Zero();
      Eigen::Vector3d ang = Eigen::Vector3d::Zero();
      Eigen::VectorXd get6D(){
        Eigen::VectorXd concat(6);
        concat << lin, ang;
        return concat;
      }
    } accel; //linear and angular force and acceleration
    //consider putting twist here too

    struct Inertial_body{
        Eigen::MatrixXd M = Eigen::MatrixXd::Zero(6,6);
        Eigen::VectorXd b = Eigen::VectorXd::Zero(6);
    } RB, AB; //stores M and b for rigid body and articulated body

    //this method returns the rotation matrix of the current joint
    Eigen::Matrix3d getRot(){
      return Rot();
    }
    void setRot(Eigen::Matrix3d rot){
      origin.orien = rot;
      userotmat = true;
    }
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
      origin.rpy.setZero();
      origin.xyz.setZero();
      origin.orien.setZero();
      w_dir.setZero();
      w_dir_dot.setZero();
      w_pos.setZero();
      w_lin_vel.setZero();
      w_ang_vel.setZero();
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

class Link{
  public:
    Origin origin;
    Joint *parentJoint = NULL;
    bool isleaf = false;
    Eigen::Matrix3d inertia_b, inertia_w;
    Eigen::Vector3d COM; //com in world frame
    double mass;

    Eigen::Matrix3d getRot(){
      return Rot();
    }
    void setRot(Eigen::Matrix3d rot){
      origin.orien = rot;
      userotmat = true;
    }
    rigidbody RB(){
      return rigidbody{inertia_w, COM, mass};
    }

    Link(){
      origin.rpy.setZero();
      origin.xyz.setZero();
      origin.orien.setZero();
      inertia_b.setZero();
      inertia_w.setZero();
      COM.setZero();
    }

  private:
    Eigen::Matrix3d Rot(){
      Eigen::Matrix3d Rx, Ry, Rz;
      double roll, pitch, yaw;
      if(userotmat){
        return origin.orien;
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
    bool userotmat = false;
};

Eigen::Matrix3d skew(Eigen::Vector3d vec){
  Eigen::Matrix3d skew;
  skew << 0.0, -vec.z(), vec.y(),
            vec.z(), 0.0, -vec.x(),
            -vec.y(), vec.x(), 0.0;
  return skew;
}

Eigen::Matrix3d inertia_tensor(double ixx, double ixy, double ixz, double iyy, double iyz, double izz){
  Eigen::Matrix3d tensor;
  tensor << ixx, ixy, ixz,
            ixy, iyy, iyz,
            ixz, iyz, izz;
  return tensor;
}

Eigen::Vector3d newCOM(double m1, Eigen::Vector3d r1, double m2, Eigen::Vector3d r2){
  return (m1*r1 + m2*r2)/(m1+m2);
}

rigidbody joinbodies(rigidbody B1, rigidbody B2){
  Eigen::Vector3d COM = newCOM(B1.m, B1.COM, B2.m, B2.COM);
  Eigen::Vector3d R1 = B1.COM - COM;
  Eigen::Vector3d R2 = B2.COM - COM;
  Eigen::Matrix3d combined_inertia = (B1.I + B2.I) - (B1.m * skew(R1)*skew(R1)) - (B2.m * skew(R2)*skew(R2));
  return rigidbody{combined_inertia, COM, B1.m + B2.m};
}

Eigen::MatrixXd getX(Eigen::Vector3d r){
    Eigen::MatrixXd Xbp = Eigen::MatrixXd::Identity(6, 6);
    Xbp.bottomLeftCorner(3,3) = skew(r);
    return Xbp;
}

//function to eliminate all fixed joints
void eliminate_fixed_joints(Joint* curjoint){
  //down path
  //process all child nodes first
  std::vector<Joint*> childjoints_copy = curjoint->childjoints;
  for(Joint* childjoint : childjoints_copy){
    eliminate_fixed_joints(childjoint);
  }
  //up path
  //if current joint is fixed, move all child joint and links to parent
  //transform position of child links and joints from j' to j frame
  //also update their parent child relationship
  if(!curjoint->isrev){
    //remove curjoint from parentJoint->childjoints
    Joint* parent = curjoint->parentJoint;
    parent->childjoints.erase(std::remove(parent->childjoints.begin(), parent->childjoints.end(), curjoint), parent->childjoints.end());
    
    //move all child joints
    for(Joint* childjoint : curjoint->childjoints){
      //find position and orientation of child joint in parent frame
      childjoint->origin.xyz = curjoint->origin.xyz + curjoint->getRot()*childjoint->origin.xyz;
      childjoint->setRot(curjoint->getRot()*childjoint->getRot());

      childjoint->parentJoint = parent;
      parent->childjoints.push_back(childjoint);
    }
    //move all child links
    for(Link* childlink : curjoint->childlinks){
      //find position and orientation of childlink in parent frame
      childlink->origin.xyz = curjoint->origin.xyz + curjoint->getRot()*childlink->origin.xyz;
      childlink->setRot(curjoint->getRot()*childlink->getRot());

      childlink->parentJoint = parent;
      parent->childlinks.push_back(childlink);
    }
  }
}

//first part of ABA
Joint::Inertial_body ABA1(Joint* curjoint, const Eigen::VectorXd &gv, const Eigen::VectorXd &tau, Eigen::Matrix3d rot){
  //down path (compute acceleration)
  rot = rot*curjoint->getRot(); //get the orientaion of the current joint relative to the world frame

  //For each body(link), find the COM and inertia in the world frame
  //Store these in Eigen::Matrix3d inertia_w and Eigen::Vector3d COM of the link object (unnecessary now, but keep for simplicity)
  //Combine the links of the current joint into one composite body
  rigidbody composite{Eigen::Matrix3d::Zero(), Eigen::Vector3d::Zero(), 0.0};
  for(Link* childlink : curjoint->childlinks){
    Eigen::Matrix3d R = rot*childlink->getRot(); //get the orientaion of the child links relative to the world frame
    childlink->inertia_w = R*(childlink->inertia_b)*R.transpose(); //find inertia in world frame for each link
    childlink->COM = curjoint->w_pos + rot*childlink->origin.xyz;
    composite = joinbodies(composite, childlink->RB());
  }

  std::cout << "checkpoint3" << std::endl;

  //Compute spatial inertia matrix about the current joint
  Eigen::MatrixXd spatialInertial(6, 6);
  Eigen::Vector3d r_ac = composite.COM - curjoint->w_pos;
  spatialInertial.block(0, 0, 3, 3) = composite.m * Eigen::Matrix3d::Identity();
  spatialInertial.block(0, 3, 3, 3) = -composite.m * skew(r_ac);
  spatialInertial.block(3, 0, 3, 3) = composite.m * skew(r_ac);
  spatialInertial.block(3, 3, 3, 3) = composite.I - (composite.m * skew(r_ac) * skew(r_ac));
  curjoint->RB.M = spatialInertial;

  //Compute fictious force vector
  Eigen::VectorXd fictious_forces(6);
  fictious_forces.head(3) << composite.m * skew(curjoint->w_ang_vel) * skew(curjoint->w_ang_vel) * r_ac;
  fictious_forces.tail(3) << skew(curjoint->w_ang_vel) * (composite.I - (composite.m * skew(r_ac) * skew(r_ac))) * curjoint->w_ang_vel;
  curjoint->RB.b = fictious_forces;

  std::cout << "checkpoint4" << std::endl;
  
  //For each revolute child joint, find the position, axis direction, and angular & linear velocity in the world frame
  //Store these in Eigen::Vector3d w_dir, Eigen::Vector3d w_pos, and Eigen::Vector3d w_ang_vel of the Joint class
  for(Joint* childjoint : curjoint->childjoints){
    Eigen::Matrix3d R = rot*childjoint->getRot();    //axis is defined in rotating joint frame (the j' frame)
    Eigen::Vector3d axis(childjoint->axis, 0.0, 0.0);
    childjoint->w_dir = R*axis;
    childjoint->w_dir_dot = curjoint->w_ang_vel.cross(childjoint->w_dir);
    childjoint->w_pos = curjoint->w_pos + rot*childjoint->origin.xyz;
    childjoint->w_ang_vel = curjoint->w_ang_vel + gv(childjoint->jointID) * childjoint->w_dir; //angular velocity of rotating child frame
    childjoint->w_lin_vel = curjoint->w_lin_vel + curjoint->w_ang_vel.cross(childjoint->w_pos - curjoint->w_pos);
    //compute the linear acceleration of the child joint
    childjoint->accel.lin = curjoint->accel.lin + skew(curjoint->accel.ang)*(childjoint->w_pos - curjoint->w_pos)
                            + skew(curjoint->w_ang_vel)*skew(curjoint->w_ang_vel)*(childjoint->w_pos - curjoint->w_pos);
    //compute the angular acceleration of the child joint
    childjoint->accel.ang = curjoint->accel.ang + childjoint->w_dir_dot*gv(childjoint->jointID);
  }

  std::cout << "checkpoint5" << std::endl;
  
  //initialize articulated body properties to rigid body properties
  curjoint->AB.M = curjoint->RB.M;
  curjoint->AB.b = curjoint->RB.b;

  //Compute articulated body inertia and bias for current joint
  for (Joint* childjoint : curjoint->childjoints) {
    //compute matrix Xbp and Xbp_dot
    Eigen::Vector3d r_pb = childjoint->w_pos - curjoint->w_pos;
    Eigen::MatrixXd Xbp = getX(r_pb);
    Eigen::MatrixXd Xbp_dot = getX(curjoint->w_ang_vel.cross(r_pb));

    ///RECURSION CALLED HERE///
    Joint::Inertial_body childAB = ABA1(childjoint, gv, tau, rot);
    //Shortform symbols for terms of AB inertia formula
    Eigen::MatrixXd M = childAB.M;
    Eigen::VectorXd b = childAB.b;
    Eigen::MatrixXd XbpT = Xbp.transpose();
    Eigen::MatrixXd Xbp_dotT = Xbp_dot.transpose();
    Eigen::VectorXd S = childjoint->S();
    Eigen::VectorXd ST = S.transpose();
    Eigen::VectorXd Sdot = childjoint->Sdot();
    Eigen::VectorXd W = curjoint->getTwist();
    size_t j = childjoint->jointID;

    curjoint->AB.M += Xbp * M * (S*(-(ST*M*S).inverse()*(ST*M*XbpT) + XbpT));

    std::cout << "here" << std::endl;

    std::vector<int> vec_initialized(100, 42);
    //Eigen::MatrixXd identity_matrix = Eigen::MatrixXd::Zero(2, 2);

    std::cout << "matrix created" << std::endl;

    Eigen::VectorXd temp = Xbp * (M*(S*(ST*M*S).inverse()*(tau.segment(j,1) - ST*M*(Sdot*gv.segment(j,1) + Xbp_dotT*W) - ST*b) + Sdot*gv.segment(j,1) + Xbp_dotT*W) + b);

    std::cout << "Fixed. Size:  " << temp.size() <<std::endl;
    
    curjoint->AB.b += Xbp * (M*(S*(ST*M*S).inverse()*(tau.segment(j,1) - ST*M*(Sdot*gv.segment(j,1) + Xbp_dotT*W) - ST*b) + Sdot*gv.segment(j,1) + Xbp_dotT*W) + b);
  }

  return curjoint->AB;
}

//second part of ABA - only go down
void ABA2(Joint* curjoint, const Eigen::VectorXd &gv, const Eigen::VectorXd &tau, Eigen::VectorXd& udot){

  //Recursively compute udot
  if(curjoint->isBase){
    udot.head(6) << curjoint->AB.M.inverse()*(tau.head(6) - curjoint->AB.b);    //can I use inverse here?
  }

  ///***Does tau include gravity?///

  for (Joint* childjoint : curjoint->childjoints) {

    //compute matrix Xbp and Xbp_dot
    Eigen::Vector3d r_pb = childjoint->w_pos - curjoint->w_pos;
    Eigen::MatrixXd Xbp = getX(r_pb);
    Eigen::MatrixXd Xbp_dot = getX(curjoint->w_ang_vel.cross(r_pb));

    //Shortform symbols for terms of equation for udot
    Eigen::MatrixXd M = childjoint->AB.M;
    Eigen::VectorXd b = childjoint->AB.b;
    Eigen::MatrixXd XbpT = Xbp.transpose();
    Eigen::MatrixXd Xbp_dotT = Xbp_dot.transpose();
    Eigen::VectorXd S = childjoint->S();
    Eigen::VectorXd ST = S.transpose();
    Eigen::VectorXd Sdot = childjoint->Sdot();
    Eigen::VectorXd W = curjoint->getTwist();
    Eigen::VectorXd Wdot = curjoint->accel.get6D();
    size_t j = childjoint->jointID;

    udot[j] = ((ST*M*S).inverse() * (tau.segment(j,1) - ST*M*(Sdot*gv(j) + Xbp_dotT*W + XbpT*Wdot) - ST*b)).value();   //CHECK

    ABA2(childjoint, gv, tau, udot);
  }
}

Eigen::VectorXd ABA(Joint* base, const Eigen::VectorXd &gv, const Eigen::VectorXd &tau){
    //compute kinematics and dynamics down the tree needed to complete NE of each body except the force
    //Compute M^A and b^A up the tree
    Eigen::Matrix3d rot = Eigen::Matrix3d::Identity();
    std::cout << "checkpoint2" << std::endl;
    ABA1(base, gv, tau, rot);

    //from the root to the leaves, compute udot and w
    Eigen::VectorXd udot = Eigen::VectorXd::Zero(18);
    ABA2(base, gv, tau, udot);
    return udot;
}



/// do not change the name of the method
inline Eigen::VectorXd computeGeneralizedAcceleration (const Eigen::VectorXd& gc, const Eigen::VectorXd& gv, const Eigen::VectorXd& gf) {

  /// !!!!!!!!!! NO RAISIM FUNCTIONS HERE !!!!!!!!!!!!!!!!!

//URDF Parsing:
  Joint base, base_face_front, base_face_rear, base_to_docking_hatch_cover, base_to_lidar_cage, lidar_cage_to_lidar;
  Link base_inertia, hatch, face_front, face_rear, battery, docking_hatch_cover, lidar;

//LF
  Joint base_LF_HAA, LF_HAA_rev, LF_HIP_LF_hip_fixed, LF_hip_fixed_LF_HFE, LF_HFE_rev, LF_THIGH_LF_thigh_fixed, 
        LF_thigh_fixed_LF_KFE, LF_KFE_rev, LF_shank_LF_shank_fixed, LF_shank_fixed_LF_FOOT;
  Link LF_HAA, LF_HIP, LF_hip_fixed, LF_HFE, LF_THIGH, LF_thigh_fixed, LF_KFE, LF_SHANK, LF_shank_fixed, LF_FOOT;

//RF
  Joint base_RF_HAA, RF_HAA_rev, RF_HIP_RF_hip_fixed, RF_hip_fixed_RF_HFE, RF_HFE_rev, RF_THIGH_RF_thigh_fixed, 
        RF_thigh_fixed_RF_KFE, RF_KFE_rev, RF_shank_RF_shank_fixed, RF_shank_fixed_RF_FOOT;
  Link RF_HAA, RF_HIP, RF_hip_fixed, RF_HFE, RF_THIGH, RF_thigh_fixed, RF_KFE, RF_SHANK, RF_shank_fixed, RF_FOOT;

//LH
  Joint base_LH_HAA, LH_HAA_rev, LH_HIP_LH_hip_fixed, LH_hip_fixed_LH_HFE, LH_HFE_rev, LH_THIGH_LH_thigh_fixed, 
        LH_thigh_fixed_LH_KFE, LH_KFE_rev, LH_shank_LH_shank_fixed, LH_shank_fixed_LH_FOOT;
  Link LH_HAA, LH_HIP, LH_hip_fixed, LH_HFE, LH_THIGH, LH_thigh_fixed, LH_KFE, LH_SHANK, LH_shank_fixed, LH_FOOT;

//RH
  Joint base_RH_HAA, RH_HAA_rev, RH_HIP_RH_hip_fixed, RH_hip_fixed_RH_HFE, RH_HFE_rev, RH_THIGH_RH_thigh_fixed, 
        RH_thigh_fixed_RH_KFE, RH_KFE_rev, RH_shank_RH_shank_fixed, RH_shank_fixed_RH_FOOT;
  Link RH_HAA, RH_HIP, RH_hip_fixed, RH_HFE, RH_THIGH, RH_thigh_fixed, RH_KFE, RH_SHANK, RH_shank_fixed, RH_FOOT;

  //get base orientation
  Eigen::Quaterniond q(gc[3], gc[4], gc[5], gc[6]);
  Eigen::Matrix3d orientation = q.toRotationMatrix();

  //set base properties
  base.accel.lin << 0, 0, 9.81;             ///////////Check this///////////
  base.w_lin_vel = gv.segment(0,3);
  base.w_ang_vel = gv.segment(3,3);
  base.setRot(orientation);
  base.isBase = true;
  base.isrev = true; //for simplication, set isrev to true since it is not fixed

  //links
  base.childlinks.push_back(&base_inertia);
  base.childlinks.push_back(&hatch);
  base.childlinks.push_back(&battery);
  //accessories
  base.childjoints.push_back(&base_face_front);
  base.childjoints.push_back(&base_face_rear);
  base.childjoints.push_back(&base_to_docking_hatch_cover);
  base.childjoints.push_back(&base_to_lidar_cage);
  //Legs
  base.childjoints.push_back(&base_LF_HAA);
  base.childjoints.push_back(&base_LH_HAA);
  base.childjoints.push_back(&base_RF_HAA);
  base.childjoints.push_back(&base_RH_HAA);

  //Link base_inertia;
  base_inertia.isleaf = true;
  base_inertia.parentJoint = &base;
  base_inertia.origin.xyz << -0.018, -0.002, 0.024;
  base_inertia.mass = 6.222;
  base_inertia.inertia_b = inertia_tensor(0.017938806, 0.00387963, 0.001500772, 0.370887745, 6.8963e-05, 0.372497653);

  //Link hatch;
  hatch.isleaf = true;
  hatch.parentJoint = &base;
  hatch.origin.xyz << 0.116, 0.0, 0.0758;
  hatch.mass = 0.142;
  hatch.inertia_b = inertia_tensor(0.001, 0.001, 0.001, 0.001, 0.001, 0.001);

  //Joint base_face_front;
  base_face_front.parentJoint = &base;
  base_face_front.childlinks.push_back(&face_front);
  base_face_front.origin.xyz << 0.4145, 0, 0;

  //Link face_front;
  face_front.isleaf = true;
  face_front.parentJoint = &base_face_front;
  face_front.origin.xyz << 0.042, -0.001, 0.004;
  face_front.mass = 0.73;
  face_front.inertia_b = inertia_tensor(0.005238611, 1.7609e-05, 7.2167e-05, 0.002643098, 1.9548e-05, 0.004325938);

  //Joint base_face_rear;
  base_face_rear.parentJoint = &base;
  base_face_rear.childlinks.push_back(&face_rear);
  base_face_rear.origin.rpy << 0, 0, 3.14159265359;
  base_face_rear.origin.xyz << -0.4145, 0, 0;

  //Link face_rear;
  face_rear.isleaf = true;
  face_rear.parentJoint = &base_face_rear;
  face_rear.origin.xyz << 0.042, -0.001, 0.004;
  face_rear.mass = 0.73;
  face_rear.inertia_b = inertia_tensor(0.005238611, 1.7609e-05, 7.2167e-05, 0.002643098, 1.9548e-05, 0.004325938);

  //Link battery;
  battery.isleaf = true;
  battery.parentJoint = &base;
  battery.origin.xyz << -0.00067, -0.00023, -0.03362;
  battery.mass = 5.53425;
  battery.inertia_b = inertia_tensor(0.00749474794, 0.00016686282, 7.82763e-05, 0.0722338913, 1.42902e-06, 0.07482717535);

  //Joint base_to_docking_hatch_cover;
  base_to_docking_hatch_cover.parentJoint = &base;
  base_to_docking_hatch_cover.childlinks.push_back(&docking_hatch_cover);
  base_to_docking_hatch_cover.origin.xyz << 0.343, 0.0, -0.07;

  //Link docking_hatch_cover;
  docking_hatch_cover.isleaf = true;
  docking_hatch_cover.parentJoint = &base_to_docking_hatch_cover;
  docking_hatch_cover.origin.xyz << -0.003, 0.0, 0.005;
  docking_hatch_cover.mass = 0.065;
  docking_hatch_cover.inertia_b = inertia_tensor(0.00063283, 0.0, 3.45e-07, 0.00110971, 0.0, 0.00171883);

  //Joint base_to_lidar_cage;
  base_to_lidar_cage.parentJoint = &base;
  base_to_lidar_cage.childjoints.push_back(&lidar_cage_to_lidar);
  base_to_lidar_cage.origin.xyz << -0.364, 0.0, 0.0735;

  //Joint lidar_cage_to_lidar;
  lidar_cage_to_lidar.parentJoint = &base_to_lidar_cage;
  lidar_cage_to_lidar.childlinks.push_back(&lidar);
  lidar_cage_to_lidar.origin.xyz << 0.0, 0.0, 0.0687;
  lidar_cage_to_lidar.origin.rpy << 0.0, 0.0, -1.57079632679;

  //Link lidar;
  lidar.isleaf = true;
  lidar.parentJoint = &lidar_cage_to_lidar;
  lidar.origin.xyz << -0.012, 0.001, -0.008;
  lidar.mass = 0.695;
  lidar.inertia_b = inertia_tensor(0.000846765, 6.9565e-05, 0.00027111, 0.001367583, 5.8984e-05, 0.001363673);

//LF
  //Joint base_LF_HAA;
  base_LF_HAA.parentJoint = &base;
  base_LF_HAA.childlinks.push_back(&LF_HAA);
  base_LF_HAA.childjoints.push_back(&LF_HAA_rev);
  base_LF_HAA.origin.rpy << 2.61799387799, 0.0, 0.0;
  base_LF_HAA.origin.xyz << 0.2999, 0.104, 0.0;

  //Link LF_HAA;
  LF_HAA.parentJoint = &base_LF_HAA;
  LF_HAA.origin.xyz << -0.063, 7e-05, 0.00046;
  LF_HAA.mass = 2.04;
  LF_HAA.inertia_b = inertia_tensor(0.001053013, 4.527e-05, 8.855e-05, 0.001805509, 9.909e-05, 0.001765827);

  //Joint LF_HAA_rev;
  LF_HAA_rev.jointID = 6;
  LF_HAA_rev.isrev = true;
  LF_HAA_rev.axis = 1;
  LF_HAA_rev.angle = gc[7]; //TOOD
  LF_HAA_rev.parentJoint = &base_LF_HAA;
  LF_HAA_rev.childlinks.push_back(&LF_HIP);
  LF_HAA_rev.childjoints.push_back(&LF_HIP_LF_hip_fixed);

  //Link LF_HIP;
  LF_HIP.parentJoint = &LF_HAA_rev;
  LF_HIP.mass = 0.001;
  LF_HIP.inertia_b = inertia_tensor(0.000001, 0.0, 0.0, 0.000001, 0.0, 0.000001);

  //Joint LF_HIP_LF_hip_fixed;
  LF_HIP_LF_hip_fixed.parentJoint = &LF_HAA_rev;
  LF_HIP_LF_hip_fixed.childlinks.push_back(&LF_hip_fixed);
  LF_HIP_LF_hip_fixed.childjoints.push_back(&LF_hip_fixed_LF_HFE);
  LF_HIP_LF_hip_fixed.origin.rpy << -2.61799387799, 0.0, 0.0;

  //Link LF_hip_fixed;
  LF_hip_fixed.parentJoint = &LF_HIP_LF_hip_fixed;
  LF_hip_fixed.origin.xyz << 0.048, 0.008, -0.003;
  LF_hip_fixed.mass = 0.74;
  LF_hip_fixed.inertia_b = inertia_tensor(0.001393106, 8.4012e-05, 2.3378e-05, 0.003798579, 7.1319e-05, 0.003897509);

  //Joint LF_hip_fixed_LF_HFE;
  LF_hip_fixed_LF_HFE.parentJoint = &LF_HIP_LF_hip_fixed;
  LF_hip_fixed_LF_HFE.childlinks.push_back(&LF_HFE);
  LF_hip_fixed_LF_HFE.childjoints.push_back(&LF_HFE_rev);
  LF_hip_fixed_LF_HFE.origin.rpy << 0, 0, 1.57079632679;
  LF_hip_fixed_LF_HFE.origin.xyz << 0.0599, 0.08381, 0.0;

  //Link LF_HFE;
  LF_HFE.parentJoint = &LF_hip_fixed_LF_HFE;
  LF_HFE.origin.xyz << -0.063, 7e-05, 0.00046;
  LF_HFE.mass = 2.04;
  LF_HFE.inertia_b = inertia_tensor(0.001053013, 4.527e-05, 8.855e-05, 0.001805509, 9.909e-05, 0.001765827);

  //Joint LF_HFE_rev;
  LF_HFE_rev.jointID = 7;
  LF_HFE_rev.isrev = true;
  LF_HFE_rev.axis = 1;
  LF_HFE_rev.angle = gc[8]; //TODO
  LF_HFE_rev.parentJoint = &LF_hip_fixed_LF_HFE;
  LF_HFE_rev.childlinks.push_back(&LF_THIGH);
  LF_HFE_rev.childjoints.push_back(&LF_THIGH_LF_thigh_fixed);

  //Link LF_THIGH;
  LF_THIGH.parentJoint = &LF_HFE_rev;
  LF_THIGH.mass = 0.001;
  LF_THIGH.inertia_b = inertia_tensor(0.000001, 0.0, 0.0, 0.000001, 0.0, 0.000001);

  //Joint LF_THIGH_LF_thigh_fixed;
  LF_THIGH_LF_thigh_fixed.parentJoint = &LF_HFE_rev;
  LF_THIGH_LF_thigh_fixed.childlinks.push_back(&LF_thigh_fixed);
  LF_THIGH_LF_thigh_fixed.childjoints.push_back(&LF_thigh_fixed_LF_KFE);
  LF_THIGH_LF_thigh_fixed.origin.rpy << 0, 0, -1.57079632679;

  //Link LF_thigh_fixed;
  LF_thigh_fixed.parentJoint = &LF_THIGH_LF_thigh_fixed;
  LF_thigh_fixed.origin.xyz << 0.0, 0.018, -0.169;
  LF_thigh_fixed.mass = 1.03;
  LF_thigh_fixed.inertia_b = inertia_tensor(0.018644469, 5.2e-08, 1.0157e-05, 0.019312599, 0.002520077, 0.002838361);

  //Joint LF_thigh_fixed_LF_KFE;
  LF_thigh_fixed_LF_KFE.parentJoint = &LF_THIGH_LF_thigh_fixed;
  LF_thigh_fixed_LF_KFE.childlinks.push_back(&LF_KFE);
  LF_thigh_fixed_LF_KFE.childjoints.push_back(&LF_KFE_rev);
  LF_thigh_fixed_LF_KFE.origin.rpy << 0, 0, 1.57079632679;
  LF_thigh_fixed_LF_KFE.origin.xyz << 0.0, 0.1003, -0.285;

  //Link LF_KFE;
  LF_KFE.parentJoint = &LF_thigh_fixed_LF_KFE;
  LF_KFE.origin.xyz << -0.063, 7e-05, 0.00046;
  LF_KFE.mass = 2.04;
  LF_KFE.inertia_b = inertia_tensor(0.001053013, 4.527e-05, 8.855e-05, 0.001805509, 9.909e-05, 0.001765827);

  //Joint LF_KFE_rev;
  LF_KFE_rev.jointID = 8;
  LF_KFE_rev.isrev = true;
  LF_KFE_rev.axis = 1;
  LF_KFE_rev.angle = gc[9]; //TODO
  LF_KFE_rev.parentJoint = &LF_thigh_fixed_LF_KFE;
  LF_KFE_rev.childlinks.push_back(&LF_SHANK);
  LF_KFE_rev.childjoints.push_back(&LF_shank_LF_shank_fixed);

  //Link LF_SHANK;
  LF_SHANK.parentJoint = &LF_KFE_rev;
  LF_SHANK.mass = 0.001;
  LF_SHANK.inertia_b = inertia_tensor(0.000001, 0.0, 0.0, 0.000001, 0.0, 0.000001);

  //Joint LF_shank_LF_shank_fixed;
  LF_shank_LF_shank_fixed.parentJoint = &LF_KFE_rev;
  LF_shank_LF_shank_fixed.childlinks.push_back(&LF_shank_fixed);
  LF_shank_LF_shank_fixed.childjoints.push_back(&LF_shank_fixed_LF_FOOT);
  LF_shank_LF_shank_fixed.origin.rpy << 0, 0, -1.57079632679;

  //Link LF_shank_fixed;
  LF_shank_fixed.parentJoint = &LF_shank_LF_shank_fixed;
  LF_shank_fixed.origin.xyz << 0.03463, 0.00688, 0.00098;
  LF_shank_fixed.mass = 0.33742;
  LF_shank_fixed.inertia_b = inertia_tensor(0.00032748005, 2.142561e-05, 1.33942e-05, 0.00110974122, 7.601e-08, 0.00089388521);

  //Joint LF_shank_fixed_LF_FOOT;
  LF_shank_fixed_LF_FOOT.parentJoint = &LF_shank_LF_shank_fixed;
  LF_shank_fixed_LF_FOOT.childlinks.push_back(&LF_FOOT);
  LF_shank_fixed_LF_FOOT.origin.xyz << 0.08795, 0.01305, -0.33797;

  //Link LF_FOOT;
  LF_FOOT.isleaf = true;
  LF_FOOT.parentJoint = &LF_shank_fixed_LF_FOOT;
  LF_FOOT.origin.xyz << 0.00948, -0.00948, 0.1468;
  LF_FOOT.mass = 0.25;
  LF_FOOT.inertia_b = inertia_tensor(0.00317174097, 2.63048e-06, 6.815581e-05, 0.00317174092, 6.815583e-05, 8.319196e-05);
//END OF LF

//RF
  //Joint base_RF_HAA;
  base_RF_HAA.parentJoint = &base;
  base_RF_HAA.childlinks.push_back(&RF_HAA);
  base_RF_HAA.childjoints.push_back(&RF_HAA_rev);
  base_RF_HAA.origin.rpy << -2.61799387799, 0.0, 0.0;
  base_RF_HAA.origin.xyz << 0.2999, -0.104, 0.0;

  //Link RF_HAA;
  RF_HAA.parentJoint = &base_RF_HAA;
  RF_HAA.origin.xyz << -0.063, 7e-05, 0.00046;
  RF_HAA.mass = 2.04;
  RF_HAA.inertia_b = inertia_tensor(0.001053013, 4.527e-05, 8.855e-05, 0.001805509, 9.909e-05, 0.001765827);

  //Joint RF_HAA_rev;
  RF_HAA_rev.jointID = 9;
  RF_HAA_rev.isrev = true;
  RF_HAA_rev.axis = 1;
  RF_HAA_rev.angle = gc[10]; //TOOD
  RF_HAA_rev.parentJoint = &base_RF_HAA;
  RF_HAA_rev.childlinks.push_back(&RF_HIP);
  RF_HAA_rev.childjoints.push_back(&RF_HIP_RF_hip_fixed);

  //Link RF_HIP;
  RF_HIP.parentJoint = &RF_HAA_rev;
  RF_HIP.mass = 0.001;
  RF_HIP.inertia_b = inertia_tensor(0.000001, 0.0, 0.0, 0.000001, 0.0, 0.000001);

  //Joint RF_HIP_RF_hip_fixed;
  RF_HIP_RF_hip_fixed.parentJoint = &RF_HAA_rev;
  RF_HIP_RF_hip_fixed.childlinks.push_back(&RF_hip_fixed);
  RF_HIP_RF_hip_fixed.childjoints.push_back(&RF_hip_fixed_RF_HFE);
  RF_HIP_RF_hip_fixed.origin.rpy << 2.61799387799, 0.0, 0.0;

  //Link RF_hip_fixed;
  RF_hip_fixed.parentJoint = &RF_HIP_RF_hip_fixed;
  RF_hip_fixed.origin.xyz << 0.048, -0.008, -0.003;
  RF_hip_fixed.mass = 0.74;
  RF_hip_fixed.inertia_b = inertia_tensor(0.001393106, -8.4012e-05, 2.3378e-05, 0.003798579, -7.1319e-05, 0.003897509);

  //Joint RF_hip_fixed_RF_HFE;
  RF_hip_fixed_RF_HFE.parentJoint = &RF_HIP_RF_hip_fixed;
  RF_hip_fixed_RF_HFE.childlinks.push_back(&RF_HFE);
  RF_hip_fixed_RF_HFE.childjoints.push_back(&RF_HFE_rev);
  RF_hip_fixed_RF_HFE.origin.rpy << 0, 0, -1.57079632679;
  RF_hip_fixed_RF_HFE.origin.xyz << 0.0599, -0.08381, 0.0;

  //Link RF_HFE;
  RF_HFE.parentJoint = &RF_hip_fixed_RF_HFE;
  RF_HFE.origin.xyz << -0.063, 7e-05, 0.00046;
  RF_HFE.mass = 2.04;
  RF_HFE.inertia_b = inertia_tensor(0.001053013, 4.527e-05, 8.855e-05, 0.001805509, 9.909e-05, 0.001765827);

  //Joint RF_HFE_rev;
  RF_HFE_rev.jointID = 10;
  RF_HFE_rev.isrev = true;
  RF_HFE_rev.axis = -1;
  RF_HFE_rev.angle = gc[11]; //TOOD
  RF_HFE_rev.parentJoint = &RF_hip_fixed_RF_HFE;
  RF_HFE_rev.childlinks.push_back(&RF_THIGH);
  RF_HFE_rev.childjoints.push_back(&RF_THIGH_RF_thigh_fixed);

  //Link RF_THIGH;
  RF_THIGH.parentJoint = &RF_HFE_rev;
  RF_THIGH.mass = 0.001;
  RF_THIGH.inertia_b = inertia_tensor(0.000001, 0.0, 0.0, 0.000001, 0.0, 0.000001);

  //Joint RF_THIGH_RF_thigh_fixed;
  RF_THIGH_RF_thigh_fixed.parentJoint = &RF_HFE_rev;
  RF_THIGH_RF_thigh_fixed.childlinks.push_back(&RF_thigh_fixed);
  RF_THIGH_RF_thigh_fixed.childjoints.push_back(&RF_thigh_fixed_RF_KFE);
  RF_THIGH_RF_thigh_fixed.origin.rpy << 0, 0, 1.57079632679;

  //Link RF_thigh_fixed;
  RF_thigh_fixed.parentJoint = &RF_THIGH_RF_thigh_fixed;
  RF_thigh_fixed.origin.xyz << 0.0, -0.018, -0.169;
  RF_thigh_fixed.mass = 1.03;
  RF_thigh_fixed.inertia_b = inertia_tensor(0.018644469, -5.2e-08, 1.0157e-05, 0.019312599, -0.002520077, 0.002838361);

  //Joint RF_thigh_fixed_RF_KFE;
  RF_thigh_fixed_RF_KFE.parentJoint = &RF_THIGH_RF_thigh_fixed;
  RF_thigh_fixed_RF_KFE.childlinks.push_back(&RF_KFE);
  RF_thigh_fixed_RF_KFE.childjoints.push_back(&RF_KFE_rev);
  RF_thigh_fixed_RF_KFE.origin.rpy << 0, 0, -1.57079632679;
  RF_thigh_fixed_RF_KFE.origin.xyz << 0.0, -0.1003, -0.285;

  //Link RF_KFE;
  RF_KFE.parentJoint = &RF_thigh_fixed_RF_KFE;
  RF_KFE.origin.xyz << -0.063, 7e-05, 0.00046;
  RF_KFE.mass = 2.04;
  RF_KFE.inertia_b = inertia_tensor(0.001053013, 4.527e-05, 8.855e-05, 0.001805509, 9.909e-05, 0.001765827);

  //Joint RF_KFE_rev;
  RF_KFE_rev.jointID = 11;
  RF_KFE_rev.isrev = true;
  RF_KFE_rev.axis = -1;
  RF_KFE_rev.angle = gc[12]; //TOOD
  RF_KFE_rev.parentJoint = &RF_thigh_fixed_RF_KFE;
  RF_KFE_rev.childlinks.push_back(&RF_SHANK);
  RF_KFE_rev.childjoints.push_back(&RF_shank_RF_shank_fixed);

  //Link RF_SHANK;
  RF_SHANK.parentJoint = &RF_KFE_rev;
  RF_SHANK.mass = 0.001;
  RF_SHANK.inertia_b = inertia_tensor(0.000001, 0.0, 0.0, 0.000001, 0.0, 0.000001);

  //Joint RF_shank_RF_shank_fixed;
  RF_shank_RF_shank_fixed.parentJoint = &RF_KFE_rev;
  RF_shank_RF_shank_fixed.childlinks.push_back(&RF_shank_fixed);
  RF_shank_RF_shank_fixed.childjoints.push_back(&RF_shank_fixed_RF_FOOT);
  RF_shank_RF_shank_fixed.origin.rpy << 0, 0, 1.57079632679;

  //Link RF_shank_fixed;
  RF_shank_fixed.parentJoint = &RF_shank_RF_shank_fixed;
  RF_shank_fixed.origin.xyz << 0.03463, -0.00688, 0.00098;
  RF_shank_fixed.mass = 0.33742;
  RF_shank_fixed.inertia_b = inertia_tensor(0.00032748005, -2.142561e-05, 1.33942e-05, 0.00110974122, -7.601e-08, 0.00089388521);

  //Joint RF_shank_fixed_RF_FOOT;
  RF_shank_fixed_RF_FOOT.parentJoint = &RF_shank_RF_shank_fixed;
  RF_shank_fixed_RF_FOOT.childlinks.push_back(&RF_FOOT);
  RF_shank_fixed_RF_FOOT.origin.xyz << 0.08795, -0.01305, -0.33797;

  //Link RF_FOOT;
  RF_FOOT.isleaf = true;
  RF_FOOT.parentJoint = &RF_shank_fixed_RF_FOOT;
  RF_FOOT.origin.xyz << 0.00948, 0.00948, 0.1468;
  RF_FOOT.mass = 0.25;
  RF_FOOT.inertia_b = inertia_tensor(0.00317174097, -2.63048e-06, 6.815581e-05, 0.00317174092, -6.815583e-05, 8.319196e-05);
//END OF RF

//LH
  //Joint base_LH_HAA;
  base_LH_HAA.parentJoint = &base;
  base_LH_HAA.childlinks.push_back(&LH_HAA);
  base_LH_HAA.childjoints.push_back(&LH_HAA_rev);
  base_LH_HAA.origin.rpy << -2.61799387799, 0.0, -3.14159265359;
  base_LH_HAA.origin.xyz << -0.2999, 0.104, 0.0;

  //Link LH_HAA;
  LH_HAA.parentJoint = &base_LH_HAA;
  LH_HAA.origin.xyz << -0.063, 7e-05, 0.00046;
  LH_HAA.mass = 2.04;
  LH_HAA.inertia_b = inertia_tensor(0.001053013, 4.527e-05, 8.855e-05, 0.001805509, 9.909e-05, 0.001765827);

  //Joint LH_HAA_rev;
  LH_HAA_rev.jointID = 12;
  LH_HAA_rev.isrev = true;
  LH_HAA_rev.axis = -1;
  LH_HAA_rev.angle = gc[13]; //TOOD
  LH_HAA_rev.parentJoint = &base_LH_HAA;
  LH_HAA_rev.childlinks.push_back(&LH_HIP);
  LH_HAA_rev.childjoints.push_back(&LH_HIP_LH_hip_fixed);

  //Link LH_HIP;
  LH_HIP.parentJoint = &LH_HAA_rev;
  LH_HIP.mass = 0.001;
  LH_HIP.inertia_b = inertia_tensor(0.000001, 0.0, 0.0, 0.000001, 0.0, 0.000001);

  //Joint LH_HIP_LH_hip_fixed;
  LH_HIP_LH_hip_fixed.parentJoint = &LH_HAA_rev;
  LH_HIP_LH_hip_fixed.childlinks.push_back(&LH_hip_fixed);
  LH_HIP_LH_hip_fixed.childjoints.push_back(&LH_hip_fixed_LH_HFE);
  LH_HIP_LH_hip_fixed.origin.rpy << -2.61799387799, 0.0, -3.14159265359;

  //Link LH_hip_fixed;
  LH_hip_fixed.parentJoint = &LH_HIP_LH_hip_fixed;
  LH_hip_fixed.origin.xyz << -0.048, 0.008, -0.003;
  LH_hip_fixed.mass = 0.74;
  LH_hip_fixed.inertia_b = inertia_tensor(0.001393106, -8.4012e-05, -2.3378e-05, 0.003798579, 7.1319e-05, 0.003897509);

  //Joint LH_hip_fixed_LH_HFE;
  LH_hip_fixed_LH_HFE.parentJoint = &LH_HIP_LH_hip_fixed;
  LH_hip_fixed_LH_HFE.childlinks.push_back(&LH_HFE);
  LH_hip_fixed_LH_HFE.childjoints.push_back(&LH_HFE_rev);
  LH_hip_fixed_LH_HFE.origin.rpy << 0, 0, 1.57079632679;
  LH_hip_fixed_LH_HFE.origin.xyz << -0.0599, 0.08381, 0.0;

  //Link LH_HFE;
  LH_HFE.parentJoint = &LH_hip_fixed_LH_HFE;
  LH_HFE.origin.xyz << -0.063, 7e-05, 0.00046;
  LH_HFE.mass = 2.04;
  LH_HFE.inertia_b = inertia_tensor(0.001053013, 4.527e-05, 8.855e-05, 0.001805509, 9.909e-05, 0.001765827);

  //Joint LH_HFE_rev;
  LH_HFE_rev.jointID = 13;
  LH_HFE_rev.isrev = true;
  LH_HFE_rev.axis = 1;
  LH_HFE_rev.angle = gc[14]; //TOOD
  LH_HFE_rev.parentJoint = &LH_hip_fixed_LH_HFE;
  LH_HFE_rev.childlinks.push_back(&LH_THIGH);
  LH_HFE_rev.childjoints.push_back(&LH_THIGH_LH_thigh_fixed);

  //Link LH_THIGH;
  LH_THIGH.parentJoint = &LH_HFE_rev;
  LH_THIGH.mass = 0.001;
  LH_THIGH.inertia_b = inertia_tensor(0.000001, 0.0, 0.0, 0.000001, 0.0, 0.000001);

  //Joint LH_THIGH_LH_thigh_fixed;
  LH_THIGH_LH_thigh_fixed.parentJoint = &LH_HFE_rev;
  LH_THIGH_LH_thigh_fixed.childlinks.push_back(&LH_thigh_fixed);
  LH_THIGH_LH_thigh_fixed.childjoints.push_back(&LH_thigh_fixed_LH_KFE);
  LH_THIGH_LH_thigh_fixed.origin.rpy << 0, 0, -1.57079632679;

  //Link LH_thigh_fixed;
  LH_thigh_fixed.parentJoint = &LH_THIGH_LH_thigh_fixed;
  LH_thigh_fixed.origin.xyz << 0.0, 0.018, -0.169;
  LH_thigh_fixed.mass = 1.03;
  LH_thigh_fixed.inertia_b = inertia_tensor(0.018644469, -5.2e-08, -1.0157e-05, 0.019312599, 0.002520077, 0.002838361);

  //Joint LH_thigh_fixed_LH_KFE;
  LH_thigh_fixed_LH_KFE.parentJoint = &LH_THIGH_LH_thigh_fixed;
  LH_thigh_fixed_LH_KFE.childlinks.push_back(&LH_KFE);
  LH_thigh_fixed_LH_KFE.childjoints.push_back(&LH_KFE_rev);
  LH_thigh_fixed_LH_KFE.origin.rpy << 0, 0, 1.57079632679;
  LH_thigh_fixed_LH_KFE.origin.xyz << 0.0, 0.1003, -0.285;

  //Link LH_KFE;
  LH_KFE.parentJoint = &LH_thigh_fixed_LH_KFE;
  LH_KFE.origin.xyz << -0.063, 7e-05, 0.00046;
  LH_KFE.mass = 2.04;
  LH_KFE.inertia_b = inertia_tensor(0.001053013, 4.527e-05, 8.855e-05, 0.001805509, 9.909e-05, 0.001765827);

  //Joint LH_KFE_rev;
  LH_KFE_rev.jointID = 14;
  LH_KFE_rev.isrev = true;
  LH_KFE_rev.axis = 1;
  LH_KFE_rev.angle = gc[15]; //TOOD
  LH_KFE_rev.parentJoint = &LH_thigh_fixed_LH_KFE;
  LH_KFE_rev.childlinks.push_back(&LH_SHANK);
  LH_KFE_rev.childjoints.push_back(&LH_shank_LH_shank_fixed);

  //Link LH_SHANK;
  LH_SHANK.parentJoint = &LH_KFE_rev;
  LH_SHANK.mass = 0.001;
  LH_SHANK.inertia_b = inertia_tensor(0.000001, 0.0, 0.0, 0.000001, 0.0, 0.000001);

  //Joint LH_shank_LH_shank_fixed;
  LH_shank_LH_shank_fixed.parentJoint = &LH_KFE_rev;
  LH_shank_LH_shank_fixed.childlinks.push_back(&LH_shank_fixed);
  LH_shank_LH_shank_fixed.childjoints.push_back(&LH_shank_fixed_LH_FOOT);
  LH_shank_LH_shank_fixed.origin.rpy << 0, 0, -1.57079632679;

  //Link LH_shank_fixed;
  LH_shank_fixed.parentJoint = &LH_shank_LH_shank_fixed;
  LH_shank_fixed.origin.xyz << -0.03463, 0.00688, 0.00098;
  LH_shank_fixed.mass = 0.33742;
  LH_shank_fixed.inertia_b = inertia_tensor(0.00032748005, -2.142561e-05, -1.33942e-05, 0.00110974122, 7.601e-08, 0.00089388521);

  //Joint LH_shank_fixed_LH_FOOT;
  LH_shank_fixed_LH_FOOT.parentJoint = &LH_shank_LH_shank_fixed;
  LH_shank_fixed_LH_FOOT.childlinks.push_back(&LH_FOOT);
  LH_shank_fixed_LH_FOOT.origin.xyz << -0.08795, 0.01305, -0.33797;

  //Link LH_FOOT;
  LH_FOOT.isleaf = true;
  LH_FOOT.parentJoint = &LH_shank_fixed_LH_FOOT;
  LH_FOOT.origin.xyz << -0.00948, -0.00948, 0.1468;
  LH_FOOT.mass = 0.25;
  LH_FOOT.inertia_b = inertia_tensor(0.00317174097, -2.63048e-06, -6.815581e-05, 0.00317174092, 6.815583e-05, 8.319196e-05);
//END OF LH

//RH
  //Joint base_RH_HAA;
  base_RH_HAA.parentJoint = &base;
  base_RH_HAA.childlinks.push_back(&RH_HAA);
  base_RH_HAA.childjoints.push_back(&RH_HAA_rev);
  base_RH_HAA.origin.rpy << 2.61799387799, 0.0, -3.14159265359;
  base_RH_HAA.origin.xyz << -0.2999, -0.104, 0.0;

  //Link RH_HAA;
  RH_HAA.parentJoint = &base_RH_HAA;
  RH_HAA.origin.xyz << -0.063, 7e-05, 0.00046;
  RH_HAA.mass = 2.04;
  RH_HAA.inertia_b = inertia_tensor(0.001053013, 4.527e-05, 8.855e-05, 0.001805509, 9.909e-05, 0.001765827);

  //Joint RH_HAA_rev;
  RH_HAA_rev.jointID = 15;
  RH_HAA_rev.isrev = true;
  RH_HAA_rev.axis = -1;
  RH_HAA_rev.angle = gc[16]; //TOOD
  RH_HAA_rev.parentJoint = &base_RH_HAA;
  RH_HAA_rev.childlinks.push_back(&RH_HIP);
  RH_HAA_rev.childjoints.push_back(&RH_HIP_RH_hip_fixed);

  //Link RH_HIP;
  RH_HIP.parentJoint = &RH_HAA_rev;
  RH_HIP.mass = 0.001;
  RH_HIP.inertia_b = inertia_tensor(0.000001, 0.0, 0.0, 0.000001, 0.0, 0.000001);

  //Joint RH_HIP_RH_hip_fixed;
  RH_HIP_RH_hip_fixed.parentJoint = &RH_HAA_rev;
  RH_HIP_RH_hip_fixed.childlinks.push_back(&RH_hip_fixed);
  RH_HIP_RH_hip_fixed.childjoints.push_back(&RH_hip_fixed_RH_HFE);
  RH_HIP_RH_hip_fixed.origin.rpy << 2.61799387799, 0.0, -3.14159265359;

  //Link RH_hip_fixed;
  RH_hip_fixed.parentJoint = &RH_HIP_RH_hip_fixed;
  RH_hip_fixed.origin.xyz << -0.048, -0.008, -0.003;
  RH_hip_fixed.mass = 0.74;
  RH_hip_fixed.inertia_b = inertia_tensor(0.001393106, 8.4012e-05, -2.3378e-05, 0.003798579, -7.1319e-05, 0.003897509);

  //Joint RH_hip_fixed_RH_HFE;
  RH_hip_fixed_RH_HFE.parentJoint = &RH_HIP_RH_hip_fixed;
  RH_hip_fixed_RH_HFE.childlinks.push_back(&RH_HFE);
  RH_hip_fixed_RH_HFE.childjoints.push_back(&RH_HFE_rev);
  RH_hip_fixed_RH_HFE.origin.rpy << 0, 0, -1.57079632679;
  RH_hip_fixed_RH_HFE.origin.xyz << -0.0599, -0.08381, 0.0;

  //Link RH_HFE;
  RH_HFE.parentJoint = &RH_hip_fixed_RH_HFE;
  RH_HFE.origin.xyz << -0.063, 7e-05, 0.00046;
  RH_HFE.mass = 2.04;
  RH_HFE.inertia_b = inertia_tensor(0.001053013, 4.527e-05, 8.855e-05, 0.001805509, 9.909e-05, 0.001765827);

  //Joint RH_HFE_rev;
  RH_HFE_rev.jointID = 16;
  RH_HFE_rev.isrev = true;
  RH_HFE_rev.axis = -1;
  RH_HFE_rev.angle = gc[17]; //TOOD
  RH_HFE_rev.parentJoint = &RH_hip_fixed_RH_HFE;
  RH_HFE_rev.childlinks.push_back(&RH_THIGH);
  RH_HFE_rev.childjoints.push_back(&RH_THIGH_RH_thigh_fixed);

  //Link RH_THIGH;
  RH_THIGH.parentJoint = &RH_HFE_rev;
  RH_THIGH.mass = 0.001;
  RH_THIGH.inertia_b = inertia_tensor(0.000001, 0.0, 0.0, 0.000001, 0.0, 0.000001);

  //Joint RH_THIGH_RH_thigh_fixed;
  RH_THIGH_RH_thigh_fixed.parentJoint = &RH_HFE_rev;
  RH_THIGH_RH_thigh_fixed.childlinks.push_back(&RH_thigh_fixed);
  RH_THIGH_RH_thigh_fixed.childjoints.push_back(&RH_thigh_fixed_RH_KFE);
  RH_THIGH_RH_thigh_fixed.origin.rpy << 0, 0, 1.57079632679;

  //Link RH_thigh_fixed;
  RH_thigh_fixed.parentJoint = &RH_THIGH_RH_thigh_fixed;
  RH_thigh_fixed.origin.xyz << 0.0, -0.018, -0.169;
  RH_thigh_fixed.mass = 1.03;
  RH_thigh_fixed.inertia_b = inertia_tensor(0.018644469, 5.2e-08, -1.0157e-05, 0.019312599, -0.002520077, 0.002838361);

  //Joint RH_thigh_fixed_RH_KFE;
  RH_thigh_fixed_RH_KFE.parentJoint = &RH_THIGH_RH_thigh_fixed;
  RH_thigh_fixed_RH_KFE.childlinks.push_back(&RH_KFE);
  RH_thigh_fixed_RH_KFE.childjoints.push_back(&RH_KFE_rev);
  RH_thigh_fixed_RH_KFE.origin.rpy << 0, 0, -1.57079632679;
  RH_thigh_fixed_RH_KFE.origin.xyz << 0.0, -0.1003, -0.285;

  //Link RH_KFE;
  RH_KFE.parentJoint = &RH_thigh_fixed_RH_KFE;
  RH_KFE.origin.xyz << -0.063, 7e-05, 0.00046;
  RH_KFE.mass = 2.04;
  RH_KFE.inertia_b = inertia_tensor(0.001053013, 4.527e-05, 8.855e-05, 0.001805509, 9.909e-05, 0.001765827);

  //Joint RH_KFE_rev;
  RH_KFE_rev.jointID = 17;
  RH_KFE_rev.isrev = true;
  RH_KFE_rev.axis = -1;
  RH_KFE_rev.angle = gc[18]; //TOOD
  RH_KFE_rev.parentJoint = &RH_thigh_fixed_RH_KFE;
  RH_KFE_rev.childlinks.push_back(&RH_SHANK);
  RH_KFE_rev.childjoints.push_back(&RH_shank_RH_shank_fixed);

  //Link RH_SHANK;
  RH_SHANK.parentJoint = &RH_KFE_rev;
  RH_SHANK.mass = 0.001;
  RH_SHANK.inertia_b = inertia_tensor(0.000001, 0.0, 0.0, 0.000001, 0.0, 0.000001);

  //Joint RH_shank_RH_shank_fixed;
  RH_shank_RH_shank_fixed.parentJoint = &RH_KFE_rev;
  RH_shank_RH_shank_fixed.childlinks.push_back(&RH_shank_fixed);
  RH_shank_RH_shank_fixed.childjoints.push_back(&RH_shank_fixed_RH_FOOT);
  RH_shank_RH_shank_fixed.origin.rpy << 0, 0, 1.57079632679;

  //Link RH_shank_fixed;
  RH_shank_fixed.parentJoint = &RH_shank_RH_shank_fixed;
  RH_shank_fixed.origin.xyz << -0.03463, -0.00688, 0.00098;
  RH_shank_fixed.mass = 0.33742;
  RH_shank_fixed.inertia_b = inertia_tensor(0.00032748005, 2.142561e-05, -1.33942e-05, 0.00110974122, -7.601e-08, 0.00089388521);

  //Joint RH_shank_fixed_RH_FOOT;
  RH_shank_fixed_RH_FOOT.parentJoint = &RH_shank_RH_shank_fixed;
  RH_shank_fixed_RH_FOOT.childlinks.push_back(&RH_FOOT);
  RH_shank_fixed_RH_FOOT.origin.xyz << -0.08795, -0.01305, -0.33797;

  //Link RH_FOOT;
  RH_FOOT.isleaf = true;
  RH_FOOT.parentJoint = &RH_shank_fixed_RH_FOOT;
  RH_FOOT.origin.xyz << -0.00948, 0.00948, 0.1468;
  RH_FOOT.mass = 0.25;
  RH_FOOT.inertia_b = inertia_tensor(0.00317174097, 2.63048e-06, -6.815581e-05, 0.00317174092, -6.815583e-05, 8.319196e-05);
//END OF RH

  /////// Start of actual code ///////
  eliminate_fixed_joints(&base);

  std::cout << "checkpoint1" << std::endl;

  //ABA calls two recursive sub functions to traverse the tree twice and compute udot
  Eigen::VectorXd result = ABA(&base, gv, gf);  //Change gf to argument name for generalized force

  //should the z direction acceleration be shifted by 9.81?
  result[2] -= 9.81;

  return result;
}