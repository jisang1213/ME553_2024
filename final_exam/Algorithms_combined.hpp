#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>
#include <string>
#include <iostream>

/*CODE SUMMARY

The Kinematic_Tree base class stores information common to both joint and links like their position and orientation.
The Joint class stores information specific to joints like axis, type of joint, and kinematic/dynamics at the joint frame.
The Link class stores information specific to links like the mass and inertia of a link.

The ArticulatedSystems class takes the joints and links and links them into a tree, eliminates any fixed joints, 
and determines the properties of the AS such as the DOF and whether it is a fixed bse or floating base. It is also 
responsible for all of the algorithms (CRBA, RNE, and ABA).

*/

typedef struct{
  Eigen::MatrixXd massmatrix;
  Eigen::VectorXd nonlinearities;
  Eigen::VectorXd accel;
} Answers; //container to store the answers

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

enum class JointType {
    FIXED,
    REVOLUTE,
    PRISMATIC
};

//Utils
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

class Link;
class Joint;

class Kinematic_Tree{
  public:
    Origin origin;
    Joint *parentJoint = NULL;
    std::string name;

    //constructor
    Kinematic_Tree(){
      origin.rpy.setZero();
      origin.xyz.setZero();
      origin.orien.setZero();
    }

    //this method sets the rotation matrix of the joint/link
    Eigen::Matrix3d setRot(Eigen::Matrix3d rot){
      origin.orien = rot;
      userotmat = true;
      return rot;
    }
    //this method returns the rotation matrix of the joint/link
    Eigen::Matrix3d getRot(){
      if(userotmat){
        return origin.orien;
      }
      double roll = origin.rpy[0], pitch = origin.rpy[1], yaw = origin.rpy[2];
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

  private:
    bool userotmat = false; //toggle true if origin.orien is assigned
};

class Joint : public Kinematic_Tree{
  public:
    JointType type = JointType::FIXED;
    std::string childlink;
    std::string parentlink;
    bool isBase = false;
    bool isFloating = false;
    Eigen::Vector3d axis;
    Eigen::Vector3d w_dir, w_dir_dot;   //axis direction vector in world frame
    Eigen::Vector3d w_pos;  //position of joint in world frame
    Eigen::Vector3d w_lin_vel, w_ang_vel;
    Eigen::VectorXd wrench;
    std::vector<Joint*> childjoints;
    std::vector<Link*> childlinks;
    int jointID = -1;
    int gc_idx= -1;

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
      switch(type){
        case JointType::REVOLUTE:
          S << Eigen::VectorXd::Zero(3), vec3;
          break;
        case JointType::PRISMATIC:
          S << vec3, Eigen::VectorXd::Zero(3);
          break;
        case JointType::FIXED:
          break;
      }
      return S;
    }
};

class Link : public Kinematic_Tree{
  public:
    Eigen::Matrix3d inertia_b, inertia_w;
    Eigen::Vector3d COM; //com in world frame
    double mass=0;
    bool virtual_link = false;

    rigidbody RB(){
      return rigidbody{inertia_w, COM, mass};
    }
    Link(){
      inertia_b.setZero();
      inertia_w.setZero();
      COM.setZero();
    }
};

class ArticulatedSystem{
  private:
    std::vector<Joint*> joints;
    std::vector<Link*> links;
    Joint* root;
    bool floating = false;
    int dof=0, gc_index=0, gv_index=0;
    Eigen::VectorXd gc, gv, gf;

  public:
    void addjoint(Joint* joint){
      if(joint->isBase){
        root = joint;
        floating = joint->isFloating;
        if(floating){
          gc_index+=7; gv_index+=6; dof+=6;
        }
      }
      if(joint->type == JointType::REVOLUTE || joint->type == JointType::PRISMATIC){
        joint->gc_idx = gc_index;
        joint->jointID = gv_index;
        gc_index++;
        gv_index++;
        dof++;
      }
      joints.push_back(joint);
    }
    void addlink(Link* link){
      links.push_back(link);
    }
    void setState(Eigen::VectorXd generalizedCoordinate, Eigen::VectorXd generalizedVelocity){
      gc = generalizedCoordinate;
      gv = generalizedVelocity;

      if(floating){
        //Set floating base orientation
        Eigen::Quaterniond q(gc[3], gc[4], gc[5], gc[6]);
        Eigen::Matrix3d orientation = q.normalized().toRotationMatrix();
        root->w_lin_vel = gv.segment(0,3);
        root->w_ang_vel = gv.segment(3,3);
        root->setRot(orientation);
      }
      makeTree();
      eliminate_fixed_joints(root);
    }
    void setGeneralizedForce(Eigen::VectorXd generalizedForce){
      gf = generalizedForce;
    }

  //Main dynamics functions:
  public:
    Eigen::MatrixXd getMassMatrix() const{
      //CRBA
      Eigen::MatrixXd massMatrix = Eigen::MatrixXd::Zero(dof, dof);
      CRBA(root, Eigen::Matrix3d::Identity(), massMatrix);
      //fill in lower triangle of the mass matrix
      for (int i = 0; i < dof; i++) {
        for (int j = 0; j < i; j++) {
            massMatrix(i, j) = massMatrix(j, i);
        }
      }
      return massMatrix;
    }
    Eigen::VectorXd getNonlinearities() const{
      //RNE
      root->accel.lin << 0, 0, 9.81;
      Eigen::VectorXd b_vector = Eigen::VectorXd::Zero(dof);
      RNE(root, Eigen::Matrix3d::Identity(), b_vector);
      return b_vector;
    }
    Eigen::VectorXd computeGeneralizedAcceleration() const{
      //ABA
      root->accel.lin << 0, 0, 9.81;    //DOUBLE CHECK
      Eigen::VectorXd acceleration = ABA(root);
      if(floating){
        acceleration[2] -= 9.81;
      }
      return acceleration;
    }

  private:
    void makeTree(){
      //connect all links to joints and joints to links
      for(Joint* joint : joints){
        joint->childjoints.clear();
        joint->childlinks.clear();
        for(Link* link : links){
          if(joint->childlink == link->name){
            if(!link->virtual_link) joint->childlinks.push_back(link);
            link->parentJoint = joint;
          }
        }
      }
      //connect all child and parent joints
      for(Joint* joint : joints){
        for(Link* link : links){
          if(joint->parentlink == link->name){
            joint->parentJoint = link->parentJoint;
            joint->parentJoint->childjoints.push_back(joint);
          }
        }
      }
    }
    //function to eliminate all fixed joints
    void eliminate_fixed_joints(Joint* curjoint){
      //down path
      //process all child nodes first
      std::vector<Joint*> childjoints_copy = curjoint->childjoints; //as childjoints can be changed inside the loop
      for(Joint* childjoint : childjoints_copy){
        
        switch(childjoint->type){
          case JointType::PRISMATIC:
            childjoint->origin.xyz += childjoint->axis*gc(childjoint->gc_idx);
            break;
          case JointType::REVOLUTE:
            childjoint->setRot(childjoint->getRot() * Eigen::AngleAxisd(gc(childjoint->gc_idx), childjoint->axis).toRotationMatrix());
            break;
          case JointType::FIXED:
            break;
        }
          
        eliminate_fixed_joints(childjoint);
      }
      //up path
      //if current joint is fixed, move all child joint and links to parent
      //transform position of child links and joints from j' to j frame
      //also update their parent child relationship
      if(curjoint->type == JointType::FIXED && !curjoint->isBase){
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

    //ALGORITHMS
    //CRBA
    rigidbody CRBA(Joint* curjoint, Eigen::Matrix3d rot, Eigen::MatrixXd& massmatrix) const{
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
        if(jointRB.m != 0) composite = joinbodies(composite, jointRB);
      }
      for(Link* childlink : curjoint->childlinks){
        if(childlink->mass != 0) composite = joinbodies(composite, childlink->RB());
      }

      //up path
      Eigen::MatrixXd spatialInertial(6, 6);
      Eigen::Vector3d r_ac = composite.COM - curjoint->w_pos;
      spatialInertial.block(0, 0, 3, 3) = composite.m * Eigen::Matrix3d::Identity();
      spatialInertial.block(0, 3, 3, 3) = -composite.m * skew(r_ac);
      spatialInertial.block(3, 0, 3, 3) = composite.m * skew(r_ac);
      spatialInertial.block(3, 3, 3, 3) = composite.I - (composite.m * skew(r_ac) * skew(r_ac));

      if(curjoint->isFloating){
        //put the spatial inertial matrix in the top 6x6 lefthand corner of the mass matrix
        massmatrix.block(0, 0, 6, 6) = spatialInertial;
      }
      else{
        //compute and fill Mi,j for all i<=j
        Joint* joint_i = curjoint;    //curjoint is joint j
        Eigen::MatrixXd IR = Eigen::MatrixXd::Identity(6, 6);
        //joint to joint coupling: 
        while(!joint_i->isBase){
          Eigen::Vector3d rji = joint_i->w_pos - curjoint->w_pos;
          IR.block(0, 3, 3, 3) = skew(rji);
          massmatrix(joint_i->jointID, curjoint->jointID) = curjoint->S().transpose()*spatialInertial*IR*joint_i->S();
          joint_i = joint_i->parentJoint;
        }

        //if floating base, find base to joint coupling:
        if(floating){
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

    //RNE - Returns the wrench at the current joint.
    Eigen::VectorXd RNE(Joint* curjoint, Eigen::Matrix3d rot, Eigen::VectorXd& b_vector) const{
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

        switch(childjoint->type){
          case JointType::PRISMATIC:
            childjoint->w_ang_vel = curjoint->w_ang_vel;
            break;
          case JointType::REVOLUTE:
            childjoint->w_ang_vel = curjoint->w_ang_vel + gv(childjoint->jointID) * childjoint->w_dir; //angular velocity of rotating child frame
            break;
          case JointType::FIXED:
            break;
        }

        //compute the linear acceleration of the child joint
        childjoint->accel.lin = curjoint->accel.lin + skew(curjoint->accel.ang)*(childjoint->w_pos - curjoint->w_pos)
                                + skew(curjoint->w_ang_vel)*skew(curjoint->w_ang_vel)*(childjoint->w_pos - curjoint->w_pos);
        childjoint->accel.ang = curjoint->accel.ang;

        //derivative of axis vector (wxr):
        Eigen::Vector3d w_dir_dot = curjoint->w_ang_vel.cross(childjoint->w_dir);

        switch(childjoint->type){
          case JointType::PRISMATIC:
            childjoint->accel.lin += w_dir_dot*gv(childjoint->jointID) + skew(curjoint->w_ang_vel)*childjoint->w_dir*gv(childjoint->jointID);
            break;
          case JointType::REVOLUTE:
            //compute the angular acceleration of the child joint
            childjoint->accel.ang += w_dir_dot*gv(childjoint->jointID);
            break;
          case JointType::FIXED:
            break;
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

        wrench = wrench + X * RNE(childjoint, rot, b_vector);    //RNE() called here
      }
      //Combine the links of the current joint into one composite body
      rigidbody composite{Eigen::Matrix3d::Zero(), Eigen::Vector3d::Zero(), 0.0};
      for(Link* childlink : curjoint->childlinks){
        if(childlink->mass != 0) composite = joinbodies(composite, childlink->RB());
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

      if(curjoint->isBase){
        if(curjoint->isFloating){
          //for floating base
          b_vector.head(6) << curjoint->wrench;
        }
      }
      else{
        //find transmitted generalized force
        b_vector[curjoint->jointID] = curjoint->S().dot(curjoint->wrench);
      }
      return curjoint->wrench;
    }

    //first part of ABA - traverse up and down recursively
    Joint::Inertial_body ABA1(Joint* curjoint, Eigen::Matrix3d rot) const{
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
        if(childlink->mass != 0) composite = joinbodies(composite, childlink->RB());
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

        switch(childjoint->type){
          case JointType::PRISMATIC:
            childjoint->w_ang_vel = curjoint->w_ang_vel;
            break;
          case JointType::REVOLUTE:
            childjoint->w_ang_vel = curjoint->w_ang_vel + gv(childjoint->jointID) * childjoint->w_dir; //angular velocity of rotating child frame
            break;
          case JointType::FIXED:
            break;
        }

        childjoint->w_lin_vel = curjoint->w_lin_vel + curjoint->w_ang_vel.cross(childjoint->w_pos - curjoint->w_pos);
      }

      //initialize articulated body properties to rigid body properties
      curjoint->AB = curjoint->RB;

      //Compute articulated body inertia and bias for current joint
      for (Joint* childjoint : curjoint->childjoints) {
        //compute matrix Xbp and Xbp_dot
        Eigen::Vector3d r_pb = childjoint->w_pos - curjoint->w_pos;
        Eigen::MatrixXd Xbp = getX(r_pb);
        Eigen::MatrixXd Xbp_dot;

        switch(childjoint->type){
          case JointType::PRISMATIC:
            Xbp_dot = getXdot(curjoint->w_ang_vel.cross(r_pb) + childjoint->w_dir*gv(childjoint->jointID));
            break;
          case JointType::REVOLUTE:
            Xbp_dot = getXdot(curjoint->w_ang_vel.cross(r_pb));
            break;
          case JointType::FIXED:
            break;
        }
        
        ///RECURSION CALLED HERE///
        Joint::Inertial_body childAB = ABA1(childjoint, rot);

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
        curjoint->AB.b += Xbp * (M*(S*(ST*M*S).inverse()*(gf.segment(j,1) - ST*M*(Sdot*gv.segment(j,1) + Xbp_dotT*W) - ST*b) + Sdot*gv.segment(j,1) + Xbp_dotT*W) + b);
      
      }

      return curjoint->AB;
    }

    //second part of ABA - compute the generalized acceleration from root to leaf
    void ABA2(Joint* curjoint, Eigen::VectorXd& udot) const{

      //Recursively compute udot
      if(curjoint->isFloating){
        curjoint->accel.set6D(curjoint->AB.M.inverse()*(gf.head(6) - curjoint->AB.b));
        udot.head(6) << curjoint->accel.get6D();
      }

      for (Joint* childjoint : curjoint->childjoints) {
        //compute matrix Xbp and Xbp_dot
        Eigen::Vector3d r_pb = childjoint->w_pos - curjoint->w_pos;
        Eigen::MatrixXd Xbp = getX(r_pb);
        Eigen::MatrixXd Xbp_dot;

        switch(childjoint->type){
          case JointType::PRISMATIC:
            Xbp_dot = getXdot(curjoint->w_ang_vel.cross(r_pb) + childjoint->w_dir*gv(childjoint->jointID));
            break;
          case JointType::REVOLUTE:
            Xbp_dot = getXdot(curjoint->w_ang_vel.cross(r_pb));
            break;
          case JointType::FIXED:
            break;
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

        udot[j] = ((ST*M*S).inverse() * (gf.segment(j,1) - ST*M*(Sdot*gv(j) + Xbp_dotT*W + XbpT*Wdot) - ST*b)).value();
        childjoint->accel.set6D(S*udot(j) + Sdot*gv(j) + Xbp_dotT*W + XbpT*Wdot);

        ABA2(childjoint, udot);
      }
    }

    Eigen::VectorXd ABA(Joint* base) const{
      //compute kinematics and dynamics down the tree needed to complete NE of each body except the force
      //Compute M^A and b^A up the tree
      Eigen::Matrix3d rot = Eigen::Matrix3d::Identity();
      ABA1(base, rot);

      //from the root to the leaves, compute udot and w
      Eigen::VectorXd udot = Eigen::VectorXd::Zero(dof);
      ABA2(base, udot);
      return udot;
    }

  //every joint and link has a parent joint except the base joint
  //each joint has a child joint and child link
  //each joint knows its parent and child links
};

//tree traversal for debugging
void traverse(Joint* joint){
  std::cout << "current joint:" << joint->jointID << std::endl;
  for(Joint* child : joint->childjoints){
    traverse(child);
  }
};

inline Answers computeSolution (const Eigen::VectorXd& gc, const Eigen::VectorXd& gv, const Eigen::VectorXd& gf) {
  //PARAMETERS TO SET:
    //For Joints: origin(xyz,rpy), isBase, isFloating, JointType, axis
    //For Links: name, origin, mass, inertia_b, virtual

  Answers answer;

  ArticulatedSystem anymal, arm;
  Joint j1, j2, j3, J1, J2, J3, J4;
  Link l1, l2, l3, L1, L2, L3, L4;
  
  //first fixed joint is the base joint - it has no parent link
  //joints and links must be added in order in URDF, starting with base joint

  j1.isBase = true;
  j1.isFloating = true;
  j1.childlink = "base";

  l1.name = "base";
  l1.mass = 2;
  l1.inertia_b = inertia_tensor(0.001,0,0,0.001,0,0.001);

  j2.type = JointType::REVOLUTE;
  j2.axis << 1,0,0;
  j2.parentlink = "base";
  j2.childlink = "LF_HAA";
  j2.origin.rpy << 2.61799387799, 0, 0.0;
  j2.origin.xyz << 0.2999, 0.104, 0.0;

  l2.name = "LF_HAA";
  l2.origin.xyz << -0.063, 7e-05, 0.00046;
  l2.mass = 2.04;
  l2.inertia_b = inertia_tensor(0.001,0,0,0.001,0,0.001);

  j3.type = JointType::REVOLUTE;
  j3.parentlink = "LF_HAA";
  j3.childlink = "LF_HIP";
  j3.axis << 1,0,0;

  l3.name = "LF_HIP";
  l3.mass = 0.01;
  l3.inertia_b = inertia_tensor(0.001,0,0,0.001,0,0.001);

  anymal.addjoint(&j1);
  anymal.addlink(&l1);
  anymal.addjoint(&j2);
  anymal.addlink(&l2);
  anymal.addjoint(&j3);
  anymal.addlink(&l3);
    
  //URDF FOR /2DrobotArm/robot_3D.urdf
  J1.isBase = true;
  J1.childlink = "link1";
  arm.addjoint(&J1);

  L1.name = "link1";
  arm.addlink(&L1);

  J2.parentlink = "link1";
  J2.childlink = "link2";
  J2.origin.xyz << 0.,0.,0.3;
  J2.type = JointType::REVOLUTE;
  J2.axis << 1,0,0;
  arm.addjoint(&J2);

  L2.name = "link2";
  L2.origin.xyz << 0,0,0.2;
  L2.mass = 1;
  L2.inertia_b = inertia_tensor(0.001, 0, 0, 0.001, 0, 0.001);
  arm.addlink(&L2);

  J3.parentlink = "link2";
  J3.childlink = "link3";
  J3.origin.xyz << 0,0,0.3;
  J3.type = JointType::REVOLUTE;
  J3.axis << 1,0,0;
  arm.addjoint(&J3);

  L3.name = "link3";
  L3.origin.xyz << 0,0,0.2;
  L3.mass = 1;
  L3.inertia_b = inertia_tensor(0.001, 0, 0, 0.001, 0, 0.001);
  arm.addlink(&L3);

  J4.parentlink = "link3";
  J4.childlink = "link4";
  J4.origin.xyz << 0,0,0.3;
  J4.type = JointType::PRISMATIC;
  J4.axis << 0,1,0;
  arm.addjoint(&J4);

  L4.name = "link4";
  L4.origin.xyz << 0.,0.,0.2;
  L4.mass = 1;
  L4.inertia_b = inertia_tensor(0.001, 0, 0, 0.001, 0, 0.001);
  arm.addlink(&L4);

  //set gc,gv,gf of anymal
  anymal.setState(gc, gv);
  anymal.setGeneralizedForce(gf);
  answer.massmatrix = anymal.getMassMatrix();
  answer.nonlinearities = anymal.getNonlinearities();
  answer.accel = anymal.computeGeneralizedAcceleration();

  // //set gc,gv,gf of arm
  // arm.setState(gc, gv);
  // arm.setGeneralizedForce(gf);
  // answer.massmatrix = arm.getMassMatrix();
  // answer.nonlinearities = arm.getNonlinearities();
  // answer.accel = arm.computeGeneralizedAcceleration();

  return answer;
}

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