//
// Created by jemin on 6/15/22.
//
#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>

typedef struct{
  Eigen::MatrixXd massmatrix;
  Eigen::VectorXd nonlinearities;
  Eigen::VectorXd accel;
} Answers;

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

//UTILS
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
  Xbp.block(3,0,3,3) = skew(r);
  return Xbp;
}

Eigen::MatrixXd getXdot(Eigen::Vector3d r){
  Eigen::MatrixXd Xbp = Eigen::MatrixXd::Zero(6, 6);
  Xbp.block(3,0,3,3) = skew(r);
  return Xbp;
}

//CLASSES
class Link;
class Joint;
class Kinematic_tree{
  public:
    Origin origin;
    Joint *parentJoint = NULL;

    Eigen::Matrix3d setRot(Eigen::Matrix3d rot){
      origin.orien = rot;
      userotmat = true;
      return rot;
    }

    //constructor
    Kinematic_tree(){
      origin.rpy.setZero();
      origin.xyz.setZero();
      origin.orien.setZero();
    }
  
  protected:
    Eigen::Matrix3d Rot(double roll, double pitch, double yaw){
      if(userotmat){
        return origin.orien;
      }
      Eigen::Matrix3d Rx, Ry, Rz, R;
      Rx << 1, 0, 0,
            0, cos(roll), -sin(roll),
            0, sin(roll), cos(roll);
      Ry << cos(pitch), 0, sin(pitch),
            0, 1, 0,
            -sin(pitch), 0, cos(pitch);
      Rz << cos(yaw), -sin(yaw), 0,
            sin(yaw), cos(yaw), 0,
            0, 0, 1;
      return setRot(Rz*Ry*Rx);
    }
    bool userotmat = false; //toggle true if origin.orien is assigned
};

class Joint : public Kinematic_tree{
  public:
    bool isRev = false;
    bool isPris = false;
    bool isBase = false;
    bool isFloating = false;
    double theta;
    Eigen::Vector3d axis;
    Eigen::Vector3d w_dir, w_dir_dot;   //axis direction vector in world frame
    Eigen::Vector3d w_pos;  //position of joint in world frame
    Eigen::Vector3d w_lin_vel, w_ang_vel;
    Eigen::VectorXd wrench;
    std::vector<Joint*> childjoints;
    std::vector<Link*> childlinks;
    int jointID = -1;

    //this method returns the rotation matrix of the joint/link
    Eigen::Matrix3d getRot(){
      if(isRev && !userotmat){
        return Eigen::AngleAxisd(theta, axis).toRotationMatrix();
      }
      else if(isPris){
        return Rot(0.0, 0.0, 0.0);
      }
      else{
        return Rot(origin.rpy[0], origin.rpy[1], origin.rpy[2]);
      }
    }

    struct acceleration{
      Eigen::Vector3d lin = Eigen::Vector3d::Zero();
      Eigen::Vector3d ang = Eigen::Vector3d::Zero();
      Eigen::VectorXd get6D(){
        Eigen::VectorXd concat(6);
        concat << lin, ang;
        return concat;
      }
      void set6D(const Eigen::VectorXd &acc){
        lin << acc.head(3);
        ang << acc.tail(3);
      }      
    } accel; //linear and angular force and acceleration

    struct Inertial_body{
        Eigen::MatrixXd M = Eigen::MatrixXd::Zero(6,6);
        Eigen::VectorXd b = Eigen::VectorXd::Zero(6);
    } RB, AB; //stores M and b for rigid body and articulated body

    //this method returns the 6D motion subspace vector of the joint
    Eigen::VectorXd S(){
      return subspace(w_dir);
    }
    //this method returns the time derivative of the 6D motion subspace vector of the joint
    Eigen::VectorXd Sdot(){
      return subspace(w_dir_dot);
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
      wrench.resize(6);
      wrench.setZero();
    }
  private:
    Eigen::VectorXd subspace(Eigen::Vector3d vec3){
      Eigen::VectorXd S(6);
      if(isRev){
        S << Eigen::VectorXd::Zero(3), vec3;
      }
      else if(isPris){
        S << vec3, Eigen::VectorXd::Zero(3);
      }
      return S;
    }
};

class Link : public Kinematic_tree{
  public:
    bool isLeaf = false;
    Eigen::Matrix3d inertia_b, inertia_w;
    Eigen::Vector3d COM; //com in world frame
    double mass;

    Eigen::Matrix3d getRot(){
      return Rot(origin.rpy[0], origin.rpy[1], origin.rpy[2]);
    }
    rigidbody RB(){
      return rigidbody{inertia_w, COM, mass};
    }
    Link(){
      inertia_b.setZero();
      inertia_w.setZero();
      COM.setZero();
    }
};

//function to eliminate all fixed joints
void eliminate_fixed_joints(Joint* curjoint){
  //down path
  //process all child nodes first
  std::vector<Joint*> childjoints_copy = curjoint->childjoints;
  for(Joint* childjoint : childjoints_copy){
    if(childjoint->isPris){
      childjoint->origin.xyz += childjoint->axis*childjoint->theta;
    }
    eliminate_fixed_joints(childjoint);
  }
  //up path
  //if current joint is fixed, move all child joint and links to parent
  //transform position of child links and joints from j' to j frame
  //also update their parent child relationship
  if(!(curjoint->isRev || curjoint->isPris || curjoint->isBase)){
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

rigidbody CRBA(Joint* curjoint, Eigen::Matrix3d rot, Eigen::MatrixXd& massmatrix){
  //down path
  rot = rot*curjoint->getRot(); //get the orientaion of the current joint relative to the world frame

  //For each body(link), find the COM and inertia in the world frame
  //Store these in Eigen::Matrix3d inertia_w and Eigen::Vector3d COM of the Link class
  for(Link* childlink : curjoint->childlinks){
    Eigen::Matrix3d R = rot*childlink->getRot(); //get the orientaion of the child links relative to the world frame
    childlink->inertia_w = R*(childlink->inertia_b)*R.transpose(); //find inertia in world frame for each link
    childlink->COM = curjoint->w_pos + rot*childlink->origin.xyz;
  }

  //For each revolute joint, find the position and axis direction in the world frame
  //Store these in Eigen::Vector3d w_dir and Eigen::Vector3d w_pos of the Joint class
  for(Joint* childjoint : curjoint->childjoints){
    Eigen::Matrix3d R = rot*childjoint->getRot();    //axis is defined in rotating joint frame (the j' frame)
    childjoint->w_dir = R*childjoint->axis;
    childjoint->w_pos = curjoint->w_pos + rot*childjoint->origin.xyz;
  }

  //recursively call CRBA for each child joint and also create new a composite body by combining the
  //links of the current joint and the composite bodies of all child joints
  rigidbody composite{Eigen::Matrix3d::Zero(), Eigen::Vector3d::Zero(), 0.0};
  for (Joint* childjoint : curjoint->childjoints) {
    rigidbody jointRB = CRBA(childjoint, rot, massmatrix);    //CRBA() called here
    composite = joinbodies(composite, jointRB);
  }
  for(Link* childlink : curjoint->childlinks){
    composite = joinbodies(composite, childlink->RB());
  }

  //up path
  Eigen::MatrixXd spatialInertial(6, 6);
  Eigen::Vector3d r_ac = composite.COM - curjoint->w_pos;
  spatialInertial.block(0, 0, 3, 3) = composite.m * Eigen::Matrix3d::Identity();
  spatialInertial.block(0, 3, 3, 3) = -composite.m * skew(r_ac);
  spatialInertial.block(3, 0, 3, 3) = composite.m * skew(r_ac);
  spatialInertial.block(3, 3, 3, 3) = composite.I - (composite.m * skew(r_ac) * skew(r_ac));

  if(curjoint->isBase){
    if(curjoint->isFloating){
      //put the spatial inertial matrix in the top 6x6 lefthand corner of the mass matrix
      massmatrix.block(0, 0, 6, 6) = spatialInertial;
    }
  }
  else{
    //compute and fill Mi,j for all i<=j
    Joint* joint_i = curjoint;    //curjoint is joint j
    Eigen::MatrixXd IR = Eigen::MatrixXd::Identity(6, 6);
    //joint to joint coupling: 
    while(!joint_i->isBase){
      Eigen::Vector3d rji = joint_i->w_pos - curjoint->w_pos;
      IR.block(0, 3, 3, 3) = skew(rji);

      double Mij = curjoint->S().transpose()*spatialInertial*IR*joint_i->S();
      massmatrix(joint_i->jointID, curjoint->jointID) = Mij;

      joint_i = joint_i->parentJoint;
    }

    //quick-fix lambda to check if base is floating:
    auto isFloatingBase = [&curjoint]() -> bool {
      Joint* temp = curjoint;
      while(!temp->isBase){
        temp = temp->parentJoint;
      }
      return temp->isFloating;
    };

    //if floating base, find base to joint coupling:
    if(isFloatingBase()){
      Eigen::Vector3d rji = joint_i->w_pos - curjoint->w_pos;
      IR.block(0, 3, 3, 3) = skew(rji);
      for(int i=0; i<6; i++){
        Eigen::VectorXd S = Eigen::VectorXd::Zero(6);
        S(i) = 1;
        massmatrix(i, curjoint->jointID) = curjoint->S().transpose()*spatialInertial*IR*S;
      }
    }
  }
  return composite;
}

//Recursive newton euler. Returns the wrench at the current joint.
Eigen::VectorXd RNE(Joint* curjoint, Eigen::Matrix3d rot, const Eigen::VectorXd &gv, Eigen::VectorXd& b_vector){
  //down path (compute acceleration)
  rot = rot*curjoint->getRot(); //get the orientaion of the current joint relative to the world frame

  //For each body(link), find the COM and inertia in the world frame
  //Store these in Eigen::Matrix3d inertia_w and Eigen::Vector3d COM of the link object
  for(Link* childlink : curjoint->childlinks){
    Eigen::Matrix3d R = rot*childlink->getRot(); //get the orientaion of the child links relative to the world frame
    childlink->inertia_w = R*(childlink->inertia_b)*R.transpose(); //find inertia in world frame for each link
    childlink->COM = curjoint->w_pos + rot*childlink->origin.xyz;
  }

  //For each joint, find the position, axis direction, and angular velocity in the world frame
  //Store these in Eigen::Vector3d w_dir, Eigen::Vector3d w_pos, and Eigen::Vector3d w_ang_vel of the Joint class
  for(Joint* childjoint : curjoint->childjoints){
    Eigen::Matrix3d R = rot*childjoint->getRot();    //axis is defined in rotating joint frame (the j' frame)
    childjoint->w_dir = R*childjoint->axis;
    childjoint->w_pos = curjoint->w_pos + rot*childjoint->origin.xyz;
    if(childjoint->isRev){
      childjoint->w_ang_vel = curjoint->w_ang_vel + gv(childjoint->jointID) * childjoint->w_dir; //angular velocity of rotating child frame
    }
    else if (childjoint->isPris){
      childjoint->w_ang_vel = curjoint->w_ang_vel;
    }

    //compute the linear acceleration of the child joint
    childjoint->accel.lin = curjoint->accel.lin + skew(curjoint->accel.ang)*(childjoint->w_pos - curjoint->w_pos)
                            + skew(curjoint->w_ang_vel)*skew(curjoint->w_ang_vel)*(childjoint->w_pos - curjoint->w_pos);
    childjoint->accel.ang = curjoint->accel.ang;

    //derivative of axis vector (wxr):
    Eigen::Vector3d w_dir_dot = curjoint->w_ang_vel.cross(childjoint->w_dir);
    if(childjoint->isRev){
      //compute the angular acceleration of the child joint
      childjoint->accel.ang += w_dir_dot*gv(childjoint->jointID);
    }
    else if(childjoint->isPris){
      childjoint->accel.lin += w_dir_dot*gv(childjoint->jointID)
      + skew(curjoint->w_ang_vel)*childjoint->w_dir*gv(childjoint->jointID);
    }
  }

  //recursively call RNE for each child joint to get the net wrench at the current joint due to the reaction at child joints
  Eigen::VectorXd wrench = Eigen::VectorXd::Zero(6);  //net wrench about current joint
  for (Joint* childjoint : curjoint->childjoints) {
    //compute matrix X to move wrench at child joint to wrench at parent joint
    Eigen::MatrixXd X(6, 6);
    Eigen::Vector3d r_pc = childjoint->w_pos - curjoint->w_pos;
    X.block(0, 0, 3, 3) = Eigen::Matrix3d::Identity();
    X.block(0, 3, 3, 3) = Eigen::Matrix3d::Zero();
    X.block(3, 0, 3, 3) = skew(r_pc);
    X.block(3, 3, 3, 3) = Eigen::Matrix3d::Identity();

    wrench = wrench + X * RNE(childjoint, rot, gv, b_vector);    //RNE() called here
  }
  //Combine the links of the current joint into one composite body
  rigidbody composite{Eigen::Matrix3d::Zero(), Eigen::Vector3d::Zero(), 0.0};
  for(Link* childlink : curjoint->childlinks){
    composite = joinbodies(composite, childlink->RB());
  }

  //up path
  //Compute spatial inertia matrix about the current joint
  Eigen::MatrixXd spatialInertial(6, 6);
  Eigen::Vector3d r_ac = composite.COM - curjoint->w_pos;
  spatialInertial.block(0, 0, 3, 3) = composite.m * Eigen::Matrix3d::Identity();
  spatialInertial.block(0, 3, 3, 3) = -composite.m * skew(r_ac);
  spatialInertial.block(3, 0, 3, 3) = composite.m * skew(r_ac);
  spatialInertial.block(3, 3, 3, 3) = composite.I - (composite.m * skew(r_ac) * skew(r_ac));

  //Compute fictious force vector
  Eigen::VectorXd fictious_forces(6);
  fictious_forces.head(3) << composite.m * skew(curjoint->w_ang_vel) * skew(curjoint->w_ang_vel) * r_ac;
  fictious_forces.tail(3) << skew(curjoint->w_ang_vel) * (composite.I - (composite.m * skew(r_ac) * skew(r_ac))) * curjoint->w_ang_vel;

  //compute wrench at joint
  curjoint->wrench = spatialInertial * curjoint->accel.get6D() + fictious_forces + wrench;

  //for floating base,
  if(curjoint->isBase){
    if(curjoint->isFloating){
      b_vector.head(6) << curjoint->wrench;
    }
  }
  //find transmitted generalized force
  else{
    b_vector[curjoint->jointID] = curjoint->S().dot(curjoint->wrench);
  }
  return curjoint->wrench;
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

  //For each joint, find the position, axis direction, and angular velocity in the world frame
  //Store these in Eigen::Vector3d w_dir, Eigen::Vector3d w_pos, and Eigen::Vector3d w_ang_vel of the Joint class
  for(Joint* childjoint : curjoint->childjoints){
    Eigen::Matrix3d R = rot*childjoint->getRot();    //axis is defined in rotating joint frame (the j' frame)
    childjoint->w_dir = R*childjoint->axis;
    childjoint->w_dir_dot = curjoint->w_ang_vel.cross(childjoint->w_dir);
    childjoint->w_pos = curjoint->w_pos + rot*childjoint->origin.xyz;
    if(childjoint->isRev){
      childjoint->w_ang_vel = curjoint->w_ang_vel + gv(childjoint->jointID) * childjoint->w_dir; //angular velocity of rotating child frame
    }
    else if (childjoint->isPris){
      childjoint->w_ang_vel = curjoint->w_ang_vel;
    }
  }

  //initialize articulated body properties to rigid body properties
  curjoint->AB = curjoint->RB;

  //Compute articulated body inertia and bias for current joint
  for (Joint* childjoint : curjoint->childjoints) {
    //compute matrix Xbp and Xbp_dot
    Eigen::Vector3d r_pb = childjoint->w_pos - curjoint->w_pos;
    Eigen::MatrixXd Xbp = getX(r_pb);
    Eigen::MatrixXd Xbp_dot;
    if(childjoint->isRev){
      Xbp_dot = getXdot(curjoint->w_ang_vel.cross(r_pb));
    }
    else if(childjoint->isPris){
      Xbp_dot = getXdot(curjoint->w_ang_vel.cross(r_pb) + childjoint->w_dir*gv(childjoint->jointID));
    }
    
    ///RECURSION CALLED HERE///
    Joint::Inertial_body childAB = ABA1(childjoint, gv, tau, rot);

    //Shortform symbols for terms of AB inertia formula
    Eigen::MatrixXd M = childAB.M;
    Eigen::VectorXd b = childAB.b;
    Eigen::MatrixXd XbpT = Xbp.transpose();
    Eigen::MatrixXd Xbp_dotT = Xbp_dot.transpose();
    Eigen::VectorXd S = childjoint->S();
    Eigen::MatrixXd ST = S.transpose();
    Eigen::VectorXd Sdot = childjoint->Sdot();
    Eigen::VectorXd W = curjoint->getTwist();
    size_t j = childjoint->jointID;

    curjoint->AB.M += Xbp * M * (-S*(ST*M*S).inverse()*(ST*M*XbpT) + XbpT);
    curjoint->AB.b += Xbp * (M*(S*(ST*M*S).inverse()*(tau.segment(j,1) - ST*M*(Sdot*gv.segment(j,1) + Xbp_dotT*W) - ST*b) + Sdot*gv.segment(j,1) + Xbp_dotT*W) + b);
  }

  return curjoint->AB;
}

//second part of ABA - only go down
void ABA2(Joint* curjoint, const Eigen::VectorXd &gv, const Eigen::VectorXd &tau, Eigen::VectorXd& udot){

  //Recursively compute udot
  if(curjoint->isBase){
    curjoint->accel.set6D(curjoint->AB.M.inverse()*(tau.head(6) - curjoint->AB.b));
    udot.head(6) << curjoint->accel.get6D();
  }

  ///Does tau include gravity?/// --no, gravity is not a force here

  for (Joint* childjoint : curjoint->childjoints) {

    //compute matrix Xbp and Xbp_dot
    Eigen::Vector3d r_pb = childjoint->w_pos - curjoint->w_pos;
    Eigen::MatrixXd Xbp = getX(r_pb);
    Eigen::MatrixXd Xbp_dot;
    if(childjoint->isRev){
      Xbp_dot = getXdot(curjoint->w_ang_vel.cross(r_pb));
    }
    else if(childjoint->isPris){
      Xbp_dot = getXdot(curjoint->w_ang_vel.cross(r_pb) + childjoint->w_dir*gv(childjoint->jointID));
    }

    //Shortform symbols for terms of equation for udot
    Eigen::MatrixXd M = childjoint->AB.M;
    Eigen::VectorXd b = childjoint->AB.b;
    Eigen::MatrixXd XbpT = Xbp.transpose();
    Eigen::MatrixXd Xbp_dotT = Xbp_dot.transpose();
    Eigen::VectorXd S = childjoint->S();
    Eigen::MatrixXd ST = S.transpose();
    Eigen::VectorXd Sdot = childjoint->Sdot();
    Eigen::VectorXd W = curjoint->getTwist();
    Eigen::VectorXd Wdot = curjoint->accel.get6D();
    size_t j = childjoint->jointID;

    udot[j] = ((ST*M*S).inverse() * (tau.segment(j,1) - ST*M*(Sdot*gv(j) + Xbp_dotT*W + XbpT*Wdot) - ST*b)).value();

    childjoint->accel.set6D(S*udot(j) + Sdot*gv(j) + Xbp_dotT*W + XbpT*Wdot);

    ABA2(childjoint, gv, tau, udot);
  }
}

Eigen::VectorXd ABA(Joint* base, const Eigen::VectorXd &gv, const Eigen::VectorXd &tau){
    //compute kinematics and dynamics down the tree needed to complete NE of each body except the force
    //Compute M^A and b^A up the tree
    Eigen::Matrix3d rot = Eigen::Matrix3d::Identity();
    ABA1(base, gv, tau, rot);

    //from the root to the leaves, compute udot and w
    Eigen::VectorXd udot = Eigen::VectorXd::Zero(18);
    ABA2(base, gv, tau, udot);
    return udot;
}

inline Answers computeSolution (const Eigen::VectorXd& gc, const Eigen::VectorXd& gv, const Eigen::VectorXd& gf) {
  //PARAMETERS TO SET:
    //For Joints: origin(xyz,rpy), isBase, isFloating, isRev, isPris, parentJoint, childlinks, childjoints, jointID, axis, theta
    //**jointID is gv index, not gc index
    //jointID = gc index for fixed base, gc_index-1 for floating base
    //For floating base, gc and gv for joints start from indices 7 and 6, respectively
    //For Links: origin, isLeaf, parentJoint, mass, inertia_b


  Joint base, slider, rod_revolute, rod_revolute2;
  Link sliderBar, slider_link, rod, rod2;
  
  //first fixed joint is the base joint
  base.isBase = true;
  base.isFloating = false;
  base.childjoints.push_back(&slider);

  slider.parentJoint = &base;
  slider.isPris = true;
  slider.axis << 1,0,0;
  slider.theta = gc[0];
  slider.childlinks.push_back(&slider_link);
  slider.childjoints.push_back(&rod_revolute);
  slider.childjoints.push_back(&rod_revolute2);
  slider.jointID = 0;

  slider_link.parentJoint = &slider;
  slider_link.mass = 2;
  slider_link.inertia_b = inertia_tensor(2,0,0,1,0,2);

  rod_revolute.parentJoint = &slider;
  rod_revolute.isRev = true;
  rod_revolute.axis << 0,1,0;
  rod_revolute.theta = gc[1];
  rod_revolute.childlinks.push_back(&rod);
  rod_revolute.jointID = 1;

  rod.parentJoint = &rod_revolute;
  rod.origin.xyz << 0,0,0.5;
  rod.mass = 5;
  rod.inertia_b = inertia_tensor(1,0,0,1,0,1);
  rod.isLeaf = true;

  rod_revolute2.parentJoint = &slider;
  rod_revolute2.isRev = true;
  rod_revolute2.axis << 0,1,0;
  rod_revolute2.theta = gc[2];
  rod_revolute2.childlinks.push_back(&rod2);
  rod_revolute2.jointID = 2;

  rod2.parentJoint = &rod_revolute2;
  rod2.origin.xyz << 0, 0, 0.5;
  rod2.mass = 5;
  rod2.inertia_b = inertia_tensor(1,0,0,1,0,1);
  rod2.isLeaf = true;

///END OF URDF

  //find the dof
  int dof;
  if(base.isFloating) dof = gc.size()-1; //dim of gv is 1 less than dim of gc for floating base
  else dof = gc.size(); //gc and gv have same dim for fixed base
  //Create container for answers
  Answers answer;
  //First, eliminate all fixed joint
  eliminate_fixed_joints(&base);

  //CRBA
  Eigen::MatrixXd massMatrix = Eigen::MatrixXd::Zero(dof, dof);
  CRBA(&base, Eigen::Matrix3d::Identity(), massMatrix);
  //fill in lower triangle of the mass matrix
  for (int i = 0; i < dof; i++) {
    for (int j = 0; j < i; j++) {
        massMatrix(i, j) = massMatrix(j, i);
    }
  }
  answer.massmatrix = massMatrix;

  //RNE
  base.accel.lin << 0, 0, 9.81;
  Eigen::VectorXd b_vector = Eigen::VectorXd::Zero(dof);
  RNE(&base, Eigen::Matrix3d::Identity(), gv, b_vector);
  answer.nonlinearities = b_vector;

  //ABA
  Eigen::VectorXd result = ABA(&base, gv, gf);
  result[2] -= 9.81;
  answer.accel = result;

  return answer;
}

//Set the size of placeholder vectors to DOF
inline Eigen::MatrixXd getMassMatrix (const Eigen::VectorXd& gc) {
  Answers answer = computeSolution(gc, Eigen::VectorXd::Zero(99), Eigen::VectorXd::Zero(99));
  return answer.massmatrix;
}

inline Eigen::VectorXd getNonlinearities (const Eigen::VectorXd& gc, const Eigen::VectorXd& gv) {
  Answers answer = computeSolution(gc, gv, Eigen::VectorXd::Zero(99));
  return answer.nonlinearities;
}

inline Eigen::VectorXd computeGeneralizedAcceleration (const Eigen::VectorXd& gc, const Eigen::VectorXd& gv, const Eigen::VectorXd& gf) {
  Answers answer = computeSolution(gc, gv, gf);
  return answer.accel;
}