#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>
#include <string>
#include <iostream>

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

//Forward declaration of derived classes
class Link;
class Joint;

//Base class of Link and Joint
class Kinematic_tree{
  public:
    Origin origin;
    Joint *parentJoint = NULL;
    std::string name;

    //constructor
    Kinematic_tree(){
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

class Joint : public Kinematic_tree{
  public:
    std::string childlink;
    std::string parentlink;
    bool isRev = false;
    bool isPris = false;
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

//for debugging
void traverse(Joint* joint){
  std::cout << "current joint:" << joint->jointID << std::endl;
  for(Joint* child : joint->childjoints){
    traverse(child);
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
      if(joint->isRev || joint->isPris){
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
      // //initialize gc,gv,gf
      // if(floating)  gc = Eigen::VectorXd::Zero(dof+1);
      // else  gc = Eigen::VectorXd::Zero(dof);
      // gv = Eigen::VectorXd::Zero(dof);
      // gf = Eigen::VectorXd::Zero(dof);
    }
    //function to eliminate all fixed joints
    void eliminate_fixed_joints(Joint* curjoint){
      //down path
      //process all child nodes first
      std::vector<Joint*> childjoints_copy = curjoint->childjoints; //as childjoints can be changed inside the loop
      for(Joint* childjoint : childjoints_copy){
        if(childjoint->isPris){
          childjoint->origin.xyz += childjoint->axis*gc(childjoint->gc_idx);
        }
        if(childjoint->isRev){
          childjoint->setRot(childjoint->getRot() * Eigen::AngleAxisd(gc(childjoint->gc_idx), childjoint->axis).toRotationMatrix());
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
        if(childjoint->isRev){
          childjoint->w_ang_vel = curjoint->w_ang_vel + gv(childjoint->jointID) * childjoint->w_dir; //angular velocity of rotating child frame
        }
        else if (childjoint->isPris){
          childjoint->w_ang_vel = curjoint->w_ang_vel;
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
        if(childjoint->isRev){
          Xbp_dot = getXdot(curjoint->w_ang_vel.cross(r_pb));
        }
        else if(childjoint->isPris){
          Xbp_dot = getXdot(curjoint->w_ang_vel.cross(r_pb) + childjoint->w_dir*gv(childjoint->jointID));
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

inline Answers computeSolution (const Eigen::VectorXd& gc, const Eigen::VectorXd& gv, const Eigen::VectorXd& gf) {
  //PARAMETERS TO SET:
    //For Joints: origin(xyz,rpy), isBase, isFloating, isRev, isPris, axis
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

  j2.isRev = true;
  j2.axis << 1,0,0;
  j2.parentlink = "base";
  j2.childlink = "LF_HAA";
  j2.origin.rpy << 2.61799387799, 0, 0.0;
  j2.origin.xyz << 0.2999, 0.104, 0.0;

  l2.name = "LF_HAA";
  l2.origin.xyz << -0.063, 7e-05, 0.00046;
  l2.mass = 2.04;
  l2.inertia_b = inertia_tensor(0.001,0,0,0.001,0,0.001);

  j3.isRev = true;
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
  J2.isRev = true;
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
  J3.isRev = true;
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
  J4.isPris = true;
  J4.axis << 0,1,0;
  arm.addjoint(&J4);

  L4.name = "link4";
  L4.origin.xyz << 0.,0.,0.2;
  L4.mass = 1;
  L4.inertia_b = inertia_tensor(0.001, 0, 0, 0.001, 0, 0.001);
  arm.addlink(&L4);

  // //set gc,gv,gf of anymal
  // anymal.setState(gc, gv);
  // anymal.setGeneralizedForce(gf);
  // answer.massmatrix = anymal.getMassMatrix();
  // answer.nonlinearities = anymal.getNonlinearities();
  // answer.accel = anymal.computeGeneralizedAcceleration();

  //set gc,gv,gf of arm
  arm.setState(gc, gv);
  arm.setGeneralizedForce(gf);
  answer.massmatrix = arm.getMassMatrix();
  answer.nonlinearities = arm.getNonlinearities();
  answer.accel = arm.computeGeneralizedAcceleration();

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







//   Joint base, base_face_front, base_face_rear, base_to_docking_hatch_cover, base_to_lidar_cage, lidar_cage_to_lidar;
//   Link base_inertia, hatch, face_front, face_rear, battery, docking_hatch_cover, lidar;

// //LF
//   Joint base_LF_HAA, LF_HAA_rev, LF_HIP_LF_hip_fixed, LF_hip_fixed_LF_HFE, LF_HFE_rev, LF_THIGH_LF_thigh_fixed, 
//         LF_thigh_fixed_LF_KFE, LF_KFE_rev, LF_shank_LF_shank_fixed, LF_shank_fixed_LF_FOOT;
//   Link LF_HAA, LF_HIP, LF_hip_fixed, LF_HFE, LF_THIGH, LF_thigh_fixed, LF_KFE, LF_SHANK, LF_shank_fixed, LF_FOOT;

// //RF
//   Joint base_RF_HAA, RF_HAA_rev, RF_HIP_RF_hip_fixed, RF_hip_fixed_RF_HFE, RF_HFE_rev, RF_THIGH_RF_thigh_fixed, 
//         RF_thigh_fixed_RF_KFE, RF_KFE_rev, RF_shank_RF_shank_fixed, RF_shank_fixed_RF_FOOT;
//   Link RF_HAA, RF_HIP, RF_hip_fixed, RF_HFE, RF_THIGH, RF_thigh_fixed, RF_KFE, RF_SHANK, RF_shank_fixed, RF_FOOT;

// //LH
//   Joint base_LH_HAA, LH_HAA_rev, LH_HIP_LH_hip_fixed, LH_hip_fixed_LH_HFE, LH_HFE_rev, LH_THIGH_LH_thigh_fixed, 
//         LH_thigh_fixed_LH_KFE, LH_KFE_rev, LH_shank_LH_shank_fixed, LH_shank_fixed_LH_FOOT;
//   Link LH_HAA, LH_HIP, LH_hip_fixed, LH_HFE, LH_THIGH, LH_thigh_fixed, LH_KFE, LH_SHANK, LH_shank_fixed, LH_FOOT;

// //RH
//   Joint base_RH_HAA, RH_HAA_rev, RH_HIP_RH_hip_fixed, RH_hip_fixed_RH_HFE, RH_HFE_rev, RH_THIGH_RH_thigh_fixed, 
//         RH_thigh_fixed_RH_KFE, RH_KFE_rev, RH_shank_RH_shank_fixed, RH_shank_fixed_RH_FOOT;
//   Link RH_HAA, RH_HIP, RH_hip_fixed, RH_HFE, RH_THIGH, RH_thigh_fixed, RH_KFE, RH_SHANK, RH_shank_fixed, RH_FOOT;

//   //Base
//   base.isBase = true;
//   base.isFloating = true;

//   //Set floating base orientation
//   Eigen::Quaterniond q(gc[3], gc[4], gc[5], gc[6]);
//   Eigen::Matrix3d orientation = q.normalized().toRotationMatrix();
//   base.w_lin_vel = gv.segment(0,3);
//   base.w_ang_vel = gv.segment(3,3);
//   base.setRot(orientation);

//   //links
//   base.childlinks.push_back(&base_inertia);
//   base.childlinks.push_back(&hatch);
//   base.childlinks.push_back(&battery);
//   //accessories
//   base.childjoints.push_back(&base_face_front);
//   base.childjoints.push_back(&base_face_rear);
//   base.childjoints.push_back(&base_to_docking_hatch_cover);
//   base.childjoints.push_back(&base_to_lidar_cage);
//   //Legs
//   base.childjoints.push_back(&base_LF_HAA);
//   base.childjoints.push_back(&base_LH_HAA);
//   base.childjoints.push_back(&base_RF_HAA);
//   base.childjoints.push_back(&base_RH_HAA);

//   //Link base_inertia;
//   base_inertia.isLeaf = true;
//   base_inertia.parentJoint = &base;
//   base_inertia.origin.xyz << -0.018, -0.002, 0.024;
//   base_inertia.mass = 6.222;
//   base_inertia.inertia_b = inertia_tensor(0.017938806, 0.00387963, 0.001500772, 0.370887745, 6.8963e-05, 0.372497653);

//   //Link hatch;
//   hatch.isLeaf = true;
//   hatch.parentJoint = &base;
//   hatch.origin.xyz << 0.116, 0.0, 0.0758;
//   hatch.mass = 0.142;
//   hatch.inertia_b = inertia_tensor(0.001, 0.001, 0.001, 0.001, 0.001, 0.001);

//   //Joint base_face_front;
//   base_face_front.parentJoint = &base;
//   base_face_front.childlinks.push_back(&face_front);
//   base_face_front.origin.xyz << 0.4145, 0, 0;

//   //Link face_front;
//   face_front.isLeaf = true;
//   face_front.parentJoint = &base_face_front;
//   face_front.origin.xyz << 0.042, -0.001, 0.004;
//   face_front.mass = 0.73;
//   face_front.inertia_b = inertia_tensor(0.005238611, 1.7609e-05, 7.2167e-05, 0.002643098, 1.9548e-05, 0.004325938);

//   //Joint base_face_rear;
//   base_face_rear.parentJoint = &base;
//   base_face_rear.childlinks.push_back(&face_rear);
//   base_face_rear.origin.rpy << 0, 0, 3.14159265359;
//   base_face_rear.origin.xyz << -0.4145, 0, 0;

//   //Link face_rear;
//   face_rear.isLeaf = true;
//   face_rear.parentJoint = &base_face_rear;
//   face_rear.origin.xyz << 0.042, -0.001, 0.004;
//   face_rear.mass = 0.73;
//   face_rear.inertia_b = inertia_tensor(0.005238611, 1.7609e-05, 7.2167e-05, 0.002643098, 1.9548e-05, 0.004325938);

//   //Link battery;
//   battery.isLeaf = true;
//   battery.parentJoint = &base;
//   battery.origin.xyz << -0.00067, -0.00023, -0.03362;
//   battery.mass = 5.53425;
//   battery.inertia_b = inertia_tensor(0.00749474794, 0.00016686282, 7.82763e-05, 0.0722338913, 1.42902e-06, 0.07482717535);

//   //Joint base_to_docking_hatch_cover;
//   base_to_docking_hatch_cover.parentJoint = &base;
//   base_to_docking_hatch_cover.childlinks.push_back(&docking_hatch_cover);
//   base_to_docking_hatch_cover.origin.xyz << 0.343, 0.0, -0.07;

//   //Link docking_hatch_cover;
//   docking_hatch_cover.isLeaf = true;
//   docking_hatch_cover.parentJoint = &base_to_docking_hatch_cover;
//   docking_hatch_cover.origin.xyz << -0.003, 0.0, 0.005;
//   docking_hatch_cover.mass = 0.065;
//   docking_hatch_cover.inertia_b = inertia_tensor(0.00063283, 0.0, 3.45e-07, 0.00110971, 0.0, 0.00171883);

//   //Joint base_to_lidar_cage;
//   base_to_lidar_cage.parentJoint = &base;
//   base_to_lidar_cage.childjoints.push_back(&lidar_cage_to_lidar);
//   base_to_lidar_cage.origin.xyz << -0.364, 0.0, 0.0735;

//   //Joint lidar_cage_to_lidar;
//   lidar_cage_to_lidar.parentJoint = &base_to_lidar_cage;
//   lidar_cage_to_lidar.childlinks.push_back(&lidar);
//   lidar_cage_to_lidar.origin.xyz << 0.0, 0.0, 0.0687;
//   lidar_cage_to_lidar.origin.rpy << 0.0, 0.0, -1.57079632679;

//   //Link lidar;
//   lidar.isLeaf = true;
//   lidar.parentJoint = &lidar_cage_to_lidar;
//   lidar.origin.xyz << -0.012, 0.001, -0.008;
//   lidar.mass = 0.695;
//   lidar.inertia_b = inertia_tensor(0.000846765, 6.9565e-05, 0.00027111, 0.001367583, 5.8984e-05, 0.001363673);

// //LF
//   //Joint base_LF_HAA;
//   base_LF_HAA.parentJoint = &base;
//   base_LF_HAA.childlinks.push_back(&LF_HAA);
//   base_LF_HAA.childjoints.push_back(&LF_HAA_rev);
//   base_LF_HAA.origin.rpy << 2.61799387799, 0.0, 0.0;
//   base_LF_HAA.origin.xyz << 0.2999, 0.104, 0.0;

//   //Link LF_HAA;
//   LF_HAA.parentJoint = &base_LF_HAA;
//   LF_HAA.origin.xyz << -0.063, 7e-05, 0.00046;
//   LF_HAA.mass = 2.04;
//   LF_HAA.inertia_b = inertia_tensor(0.001053013, 4.527e-05, 8.855e-05, 0.001805509, 9.909e-05, 0.001765827);

//   //Joint LF_HAA_rev;
//   LF_HAA_rev.jointID = 6;
//   LF_HAA_rev.isRev = true;
//   LF_HAA_rev.axis << 1,0,0;
//   LF_HAA_rev.theta = gc[7]; //TOOD
//   LF_HAA_rev.parentJoint = &base_LF_HAA;
//   LF_HAA_rev.childlinks.push_back(&LF_HIP);
//   LF_HAA_rev.childjoints.push_back(&LF_HIP_LF_hip_fixed);

//   //Link LF_HIP;
//   LF_HIP.parentJoint = &LF_HAA_rev;
//   LF_HIP.mass = 0.001;
//   LF_HIP.inertia_b = inertia_tensor(0.000001, 0.0, 0.0, 0.000001, 0.0, 0.000001);

//   //Joint LF_HIP_LF_hip_fixed;
//   LF_HIP_LF_hip_fixed.parentJoint = &LF_HAA_rev;
//   LF_HIP_LF_hip_fixed.childlinks.push_back(&LF_hip_fixed);
//   LF_HIP_LF_hip_fixed.childjoints.push_back(&LF_hip_fixed_LF_HFE);
//   LF_HIP_LF_hip_fixed.origin.rpy << -2.61799387799, 0.0, 0.0;

//   //Link LF_hip_fixed;
//   LF_hip_fixed.parentJoint = &LF_HIP_LF_hip_fixed;
//   LF_hip_fixed.origin.xyz << 0.048, 0.008, -0.003;
//   LF_hip_fixed.mass = 0.74;
//   LF_hip_fixed.inertia_b = inertia_tensor(0.001393106, 8.4012e-05, 2.3378e-05, 0.003798579, 7.1319e-05, 0.003897509);

//   //Joint LF_hip_fixed_LF_HFE;
//   LF_hip_fixed_LF_HFE.parentJoint = &LF_HIP_LF_hip_fixed;
//   LF_hip_fixed_LF_HFE.childlinks.push_back(&LF_HFE);
//   LF_hip_fixed_LF_HFE.childjoints.push_back(&LF_HFE_rev);
//   LF_hip_fixed_LF_HFE.origin.rpy << 0, 0, 1.57079632679;
//   LF_hip_fixed_LF_HFE.origin.xyz << 0.0599, 0.08381, 0.0;

//   //Link LF_HFE;
//   LF_HFE.parentJoint = &LF_hip_fixed_LF_HFE;
//   LF_HFE.origin.xyz << -0.063, 7e-05, 0.00046;
//   LF_HFE.mass = 2.04;
//   LF_HFE.inertia_b = inertia_tensor(0.001053013, 4.527e-05, 8.855e-05, 0.001805509, 9.909e-05, 0.001765827);

//   //Joint LF_HFE_rev;
//   LF_HFE_rev.jointID = 7;
//   LF_HFE_rev.isPris = true;
//   LF_HFE_rev.axis << 1,0,0;
//   LF_HFE_rev.theta = gc[8]; //TODO
//   LF_HFE_rev.parentJoint = &LF_hip_fixed_LF_HFE;
//   LF_HFE_rev.childlinks.push_back(&LF_THIGH);
//   LF_HFE_rev.childjoints.push_back(&LF_THIGH_LF_thigh_fixed);

//   //Link LF_THIGH;
//   LF_THIGH.parentJoint = &LF_HFE_rev;
//   LF_THIGH.mass = 0.001;
//   LF_THIGH.inertia_b = inertia_tensor(0.000001, 0.0, 0.0, 0.000001, 0.0, 0.000001);

//   //Joint LF_THIGH_LF_thigh_fixed;
//   LF_THIGH_LF_thigh_fixed.parentJoint = &LF_HFE_rev;
//   LF_THIGH_LF_thigh_fixed.childlinks.push_back(&LF_thigh_fixed);
//   LF_THIGH_LF_thigh_fixed.childjoints.push_back(&LF_thigh_fixed_LF_KFE);
//   LF_THIGH_LF_thigh_fixed.origin.rpy << 0, 0, -1.57079632679;

//   //Link LF_thigh_fixed;
//   LF_thigh_fixed.parentJoint = &LF_THIGH_LF_thigh_fixed;
//   LF_thigh_fixed.origin.xyz << 0.0, 0.018, -0.169;
//   LF_thigh_fixed.mass = 1.03;
//   LF_thigh_fixed.inertia_b = inertia_tensor(0.018644469, 5.2e-08, 1.0157e-05, 0.019312599, 0.002520077, 0.002838361);

//   //Joint LF_thigh_fixed_LF_KFE;
//   LF_thigh_fixed_LF_KFE.parentJoint = &LF_THIGH_LF_thigh_fixed;
//   LF_thigh_fixed_LF_KFE.childlinks.push_back(&LF_KFE);
//   LF_thigh_fixed_LF_KFE.childjoints.push_back(&LF_KFE_rev);
//   LF_thigh_fixed_LF_KFE.origin.rpy << 0, 0, 1.57079632679;
//   LF_thigh_fixed_LF_KFE.origin.xyz << 0.0, 0.1003, -0.285;

//   //Link LF_KFE;
//   LF_KFE.parentJoint = &LF_thigh_fixed_LF_KFE;
//   LF_KFE.origin.xyz << -0.063, 7e-05, 0.00046;
//   LF_KFE.mass = 2.04;
//   LF_KFE.inertia_b = inertia_tensor(0.001053013, 4.527e-05, 8.855e-05, 0.001805509, 9.909e-05, 0.001765827);

//   //Joint LF_KFE_rev;
//   LF_KFE_rev.jointID = 8;
//   LF_KFE_rev.isRev = true;
//   LF_KFE_rev.axis << 1,0,0;
//   LF_KFE_rev.theta = gc[9]; //TODO
//   LF_KFE_rev.parentJoint = &LF_thigh_fixed_LF_KFE;
//   LF_KFE_rev.childlinks.push_back(&LF_SHANK);
//   LF_KFE_rev.childjoints.push_back(&LF_shank_LF_shank_fixed);

//   //Link LF_SHANK;
//   LF_SHANK.parentJoint = &LF_KFE_rev;
//   LF_SHANK.mass = 0.001;
//   LF_SHANK.inertia_b = inertia_tensor(0.000001, 0.0, 0.0, 0.000001, 0.0, 0.000001);

//   //Joint LF_shank_LF_shank_fixed;
//   LF_shank_LF_shank_fixed.parentJoint = &LF_KFE_rev;
//   LF_shank_LF_shank_fixed.childlinks.push_back(&LF_shank_fixed);
//   LF_shank_LF_shank_fixed.childjoints.push_back(&LF_shank_fixed_LF_FOOT);
//   LF_shank_LF_shank_fixed.origin.rpy << 0, 0, -1.57079632679;

//   //Link LF_shank_fixed;
//   LF_shank_fixed.parentJoint = &LF_shank_LF_shank_fixed;
//   LF_shank_fixed.origin.xyz << 0.03463, 0.00688, 0.00098;
//   LF_shank_fixed.mass = 0.33742;
//   LF_shank_fixed.inertia_b = inertia_tensor(0.00032748005, 2.142561e-05, 1.33942e-05, 0.00110974122, 7.601e-08, 0.00089388521);

//   //Joint LF_shank_fixed_LF_FOOT;
//   LF_shank_fixed_LF_FOOT.parentJoint = &LF_shank_LF_shank_fixed;
//   LF_shank_fixed_LF_FOOT.childlinks.push_back(&LF_FOOT);
//   LF_shank_fixed_LF_FOOT.origin.xyz << 0.08795, 0.01305, -0.33797;

//   //Link LF_FOOT;
//   LF_FOOT.isLeaf = true;
//   LF_FOOT.parentJoint = &LF_shank_fixed_LF_FOOT;
//   LF_FOOT.origin.xyz << 0.00948, -0.00948, 0.1468;
//   LF_FOOT.mass = 0.25;
//   LF_FOOT.inertia_b = inertia_tensor(0.00317174097, 2.63048e-06, 6.815581e-05, 0.00317174092, 6.815583e-05, 8.319196e-05);
// //END OF LF

// //RF
//   //Joint base_RF_HAA;
//   base_RF_HAA.parentJoint = &base;
//   base_RF_HAA.childlinks.push_back(&RF_HAA);
//   base_RF_HAA.childjoints.push_back(&RF_HAA_rev);
//   base_RF_HAA.origin.rpy << -2.61799387799, 0.0, 0.0;
//   base_RF_HAA.origin.xyz << 0.2999, -0.104, 0.0;

//   //Link RF_HAA;
//   RF_HAA.parentJoint = &base_RF_HAA;
//   RF_HAA.origin.xyz << -0.063, 7e-05, 0.00046;
//   RF_HAA.mass = 2.04;
//   RF_HAA.inertia_b = inertia_tensor(0.001053013, 4.527e-05, 8.855e-05, 0.001805509, 9.909e-05, 0.001765827);

//   //Joint RF_HAA_rev;
//   RF_HAA_rev.jointID = 9;
//   RF_HAA_rev.isRev = true;
//   RF_HAA_rev.axis << 1,0,0;
//   RF_HAA_rev.theta = gc[10]; //TOOD
//   RF_HAA_rev.parentJoint = &base_RF_HAA;
//   RF_HAA_rev.childlinks.push_back(&RF_HIP);
//   RF_HAA_rev.childjoints.push_back(&RF_HIP_RF_hip_fixed);

//   //Link RF_HIP;
//   RF_HIP.parentJoint = &RF_HAA_rev;
//   RF_HIP.mass = 0.001;
//   RF_HIP.inertia_b = inertia_tensor(0.000001, 0.0, 0.0, 0.000001, 0.0, 0.000001);

//   //Joint RF_HIP_RF_hip_fixed;
//   RF_HIP_RF_hip_fixed.parentJoint = &RF_HAA_rev;
//   RF_HIP_RF_hip_fixed.childlinks.push_back(&RF_hip_fixed);
//   RF_HIP_RF_hip_fixed.childjoints.push_back(&RF_hip_fixed_RF_HFE);
//   RF_HIP_RF_hip_fixed.origin.rpy << 2.61799387799, 0.0, 0.0;

//   //Link RF_hip_fixed;
//   RF_hip_fixed.parentJoint = &RF_HIP_RF_hip_fixed;
//   RF_hip_fixed.origin.xyz << 0.048, -0.008, -0.003;
//   RF_hip_fixed.mass = 0.74;
//   RF_hip_fixed.inertia_b = inertia_tensor(0.001393106, -8.4012e-05, 2.3378e-05, 0.003798579, -7.1319e-05, 0.003897509);

//   //Joint RF_hip_fixed_RF_HFE;
//   RF_hip_fixed_RF_HFE.parentJoint = &RF_HIP_RF_hip_fixed;
//   RF_hip_fixed_RF_HFE.childlinks.push_back(&RF_HFE);
//   RF_hip_fixed_RF_HFE.childjoints.push_back(&RF_HFE_rev);
//   RF_hip_fixed_RF_HFE.origin.rpy << 0, 0, -1.57079632679;
//   RF_hip_fixed_RF_HFE.origin.xyz << 0.0599, -0.08381, 0.0;

//   //Link RF_HFE;
//   RF_HFE.parentJoint = &RF_hip_fixed_RF_HFE;
//   RF_HFE.origin.xyz << -0.063, 7e-05, 0.00046;
//   RF_HFE.mass = 2.04;
//   RF_HFE.inertia_b = inertia_tensor(0.001053013, 4.527e-05, 8.855e-05, 0.001805509, 9.909e-05, 0.001765827);

//   //Joint RF_HFE_rev;
//   RF_HFE_rev.jointID = 10;
//   RF_HFE_rev.isRev = true;
//   RF_HFE_rev.axis << -1,0,0;
//   RF_HFE_rev.theta = gc[11]; //TOOD
//   RF_HFE_rev.parentJoint = &RF_hip_fixed_RF_HFE;
//   RF_HFE_rev.childlinks.push_back(&RF_THIGH);
//   RF_HFE_rev.childjoints.push_back(&RF_THIGH_RF_thigh_fixed);

//   //Link RF_THIGH;
//   RF_THIGH.parentJoint = &RF_HFE_rev;
//   RF_THIGH.mass = 0.001;
//   RF_THIGH.inertia_b = inertia_tensor(0.000001, 0.0, 0.0, 0.000001, 0.0, 0.000001);

//   //Joint RF_THIGH_RF_thigh_fixed;
//   RF_THIGH_RF_thigh_fixed.parentJoint = &RF_HFE_rev;
//   RF_THIGH_RF_thigh_fixed.childlinks.push_back(&RF_thigh_fixed);
//   RF_THIGH_RF_thigh_fixed.childjoints.push_back(&RF_thigh_fixed_RF_KFE);
//   RF_THIGH_RF_thigh_fixed.origin.rpy << 0, 0, 1.57079632679;

//   //Link RF_thigh_fixed;
//   RF_thigh_fixed.parentJoint = &RF_THIGH_RF_thigh_fixed;
//   RF_thigh_fixed.origin.xyz << 0.0, -0.018, -0.169;
//   RF_thigh_fixed.mass = 1.03;
//   RF_thigh_fixed.inertia_b = inertia_tensor(0.018644469, -5.2e-08, 1.0157e-05, 0.019312599, -0.002520077, 0.002838361);

//   //Joint RF_thigh_fixed_RF_KFE;
//   RF_thigh_fixed_RF_KFE.parentJoint = &RF_THIGH_RF_thigh_fixed;
//   RF_thigh_fixed_RF_KFE.childlinks.push_back(&RF_KFE);
//   RF_thigh_fixed_RF_KFE.childjoints.push_back(&RF_KFE_rev);
//   RF_thigh_fixed_RF_KFE.origin.rpy << 0, 0, -1.57079632679;
//   RF_thigh_fixed_RF_KFE.origin.xyz << 0.0, -0.1003, -0.285;

//   //Link RF_KFE;
//   RF_KFE.parentJoint = &RF_thigh_fixed_RF_KFE;
//   RF_KFE.origin.xyz << -0.063, 7e-05, 0.00046;
//   RF_KFE.mass = 2.04;
//   RF_KFE.inertia_b = inertia_tensor(0.001053013, 4.527e-05, 8.855e-05, 0.001805509, 9.909e-05, 0.001765827);

//   //Joint RF_KFE_rev;
//   RF_KFE_rev.jointID = 11;
//   RF_KFE_rev.isRev = true;
//   RF_KFE_rev.axis << -1,0,0;
//   RF_KFE_rev.theta = gc[12]; //TOOD
//   RF_KFE_rev.parentJoint = &RF_thigh_fixed_RF_KFE;
//   RF_KFE_rev.childlinks.push_back(&RF_SHANK);
//   RF_KFE_rev.childjoints.push_back(&RF_shank_RF_shank_fixed);

//   //Link RF_SHANK;
//   RF_SHANK.parentJoint = &RF_KFE_rev;
//   RF_SHANK.mass = 0.001;
//   RF_SHANK.inertia_b = inertia_tensor(0.000001, 0.0, 0.0, 0.000001, 0.0, 0.000001);

//   //Joint RF_shank_RF_shank_fixed;
//   RF_shank_RF_shank_fixed.parentJoint = &RF_KFE_rev;
//   RF_shank_RF_shank_fixed.childlinks.push_back(&RF_shank_fixed);
//   RF_shank_RF_shank_fixed.childjoints.push_back(&RF_shank_fixed_RF_FOOT);
//   RF_shank_RF_shank_fixed.origin.rpy << 0, 0, 1.57079632679;

//   //Link RF_shank_fixed;
//   RF_shank_fixed.parentJoint = &RF_shank_RF_shank_fixed;
//   RF_shank_fixed.origin.xyz << 0.03463, -0.00688, 0.00098;
//   RF_shank_fixed.mass = 0.33742;
//   RF_shank_fixed.inertia_b = inertia_tensor(0.00032748005, -2.142561e-05, 1.33942e-05, 0.00110974122, -7.601e-08, 0.00089388521);

//   //Joint RF_shank_fixed_RF_FOOT;
//   RF_shank_fixed_RF_FOOT.parentJoint = &RF_shank_RF_shank_fixed;
//   RF_shank_fixed_RF_FOOT.childlinks.push_back(&RF_FOOT);
//   RF_shank_fixed_RF_FOOT.origin.xyz << 0.08795, -0.01305, -0.33797;

//   //Link RF_FOOT;
//   RF_FOOT.isLeaf = true;
//   RF_FOOT.parentJoint = &RF_shank_fixed_RF_FOOT;
//   RF_FOOT.origin.xyz << 0.00948, 0.00948, 0.1468;
//   RF_FOOT.mass = 0.25;
//   RF_FOOT.inertia_b = inertia_tensor(0.00317174097, -2.63048e-06, 6.815581e-05, 0.00317174092, -6.815583e-05, 8.319196e-05);
// //END OF RF

// //LH
//   //Joint base_LH_HAA;
//   base_LH_HAA.parentJoint = &base;
//   base_LH_HAA.childlinks.push_back(&LH_HAA);
//   base_LH_HAA.childjoints.push_back(&LH_HAA_rev);
//   base_LH_HAA.origin.rpy << -2.61799387799, 0.0, -3.14159265359;
//   base_LH_HAA.origin.xyz << -0.2999, 0.104, 0.0;

//   //Link LH_HAA;
//   LH_HAA.parentJoint = &base_LH_HAA;
//   LH_HAA.origin.xyz << -0.063, 7e-05, 0.00046;
//   LH_HAA.mass = 2.04;
//   LH_HAA.inertia_b = inertia_tensor(0.001053013, 4.527e-05, 8.855e-05, 0.001805509, 9.909e-05, 0.001765827);

//   //Joint LH_HAA_rev;
//   LH_HAA_rev.jointID = 12;
//   LH_HAA_rev.isRev = true;
//   LH_HAA_rev.axis << -1,0,0;
//   LH_HAA_rev.theta = gc[13]; //TOOD
//   LH_HAA_rev.parentJoint = &base_LH_HAA;
//   LH_HAA_rev.childlinks.push_back(&LH_HIP);
//   LH_HAA_rev.childjoints.push_back(&LH_HIP_LH_hip_fixed);

//   //Link LH_HIP;
//   LH_HIP.parentJoint = &LH_HAA_rev;
//   LH_HIP.mass = 0.001;
//   LH_HIP.inertia_b = inertia_tensor(0.000001, 0.0, 0.0, 0.000001, 0.0, 0.000001);

//   //Joint LH_HIP_LH_hip_fixed;
//   LH_HIP_LH_hip_fixed.parentJoint = &LH_HAA_rev;
//   LH_HIP_LH_hip_fixed.childlinks.push_back(&LH_hip_fixed);
//   LH_HIP_LH_hip_fixed.childjoints.push_back(&LH_hip_fixed_LH_HFE);
//   LH_HIP_LH_hip_fixed.origin.rpy << -2.61799387799, 0.0, -3.14159265359;

//   //Link LH_hip_fixed;
//   LH_hip_fixed.parentJoint = &LH_HIP_LH_hip_fixed;
//   LH_hip_fixed.origin.xyz << -0.048, 0.008, -0.003;
//   LH_hip_fixed.mass = 0.74;
//   LH_hip_fixed.inertia_b = inertia_tensor(0.001393106, -8.4012e-05, -2.3378e-05, 0.003798579, 7.1319e-05, 0.003897509);

//   //Joint LH_hip_fixed_LH_HFE;
//   LH_hip_fixed_LH_HFE.parentJoint = &LH_HIP_LH_hip_fixed;
//   LH_hip_fixed_LH_HFE.childlinks.push_back(&LH_HFE);
//   LH_hip_fixed_LH_HFE.childjoints.push_back(&LH_HFE_rev);
//   LH_hip_fixed_LH_HFE.origin.rpy << 0, 0, 1.57079632679;
//   LH_hip_fixed_LH_HFE.origin.xyz << -0.0599, 0.08381, 0.0;

//   //Link LH_HFE;
//   LH_HFE.parentJoint = &LH_hip_fixed_LH_HFE;
//   LH_HFE.origin.xyz << -0.063, 7e-05, 0.00046;
//   LH_HFE.mass = 2.04;
//   LH_HFE.inertia_b = inertia_tensor(0.001053013, 4.527e-05, 8.855e-05, 0.001805509, 9.909e-05, 0.001765827);

//   //Joint LH_HFE_rev;
//   LH_HFE_rev.jointID = 13;
//   LH_HFE_rev.isRev = true;
//   LH_HFE_rev.axis << 1,0,0;
//   LH_HFE_rev.theta = gc[14]; //TOOD
//   LH_HFE_rev.parentJoint = &LH_hip_fixed_LH_HFE;
//   LH_HFE_rev.childlinks.push_back(&LH_THIGH);
//   LH_HFE_rev.childjoints.push_back(&LH_THIGH_LH_thigh_fixed);

//   //Link LH_THIGH;
//   LH_THIGH.parentJoint = &LH_HFE_rev;
//   LH_THIGH.mass = 0.001;
//   LH_THIGH.inertia_b = inertia_tensor(0.000001, 0.0, 0.0, 0.000001, 0.0, 0.000001);

//   //Joint LH_THIGH_LH_thigh_fixed;
//   LH_THIGH_LH_thigh_fixed.parentJoint = &LH_HFE_rev;
//   LH_THIGH_LH_thigh_fixed.childlinks.push_back(&LH_thigh_fixed);
//   LH_THIGH_LH_thigh_fixed.childjoints.push_back(&LH_thigh_fixed_LH_KFE);
//   LH_THIGH_LH_thigh_fixed.origin.rpy << 0, 0, -1.57079632679;

//   //Link LH_thigh_fixed;
//   LH_thigh_fixed.parentJoint = &LH_THIGH_LH_thigh_fixed;
//   LH_thigh_fixed.origin.xyz << 0.0, 0.018, -0.169;
//   LH_thigh_fixed.mass = 1.03;
//   LH_thigh_fixed.inertia_b = inertia_tensor(0.018644469, -5.2e-08, -1.0157e-05, 0.019312599, 0.002520077, 0.002838361);

//   //Joint LH_thigh_fixed_LH_KFE;
//   LH_thigh_fixed_LH_KFE.parentJoint = &LH_THIGH_LH_thigh_fixed;
//   LH_thigh_fixed_LH_KFE.childlinks.push_back(&LH_KFE);
//   LH_thigh_fixed_LH_KFE.childjoints.push_back(&LH_KFE_rev);
//   LH_thigh_fixed_LH_KFE.origin.rpy << 0, 0, 1.57079632679;
//   LH_thigh_fixed_LH_KFE.origin.xyz << 0.0, 0.1003, -0.285;

//   //Link LH_KFE;
//   LH_KFE.parentJoint = &LH_thigh_fixed_LH_KFE;
//   LH_KFE.origin.xyz << -0.063, 7e-05, 0.00046;
//   LH_KFE.mass = 2.04;
//   LH_KFE.inertia_b = inertia_tensor(0.001053013, 4.527e-05, 8.855e-05, 0.001805509, 9.909e-05, 0.001765827);

//   //Joint LH_KFE_rev;
//   LH_KFE_rev.jointID = 14;
//   LH_KFE_rev.isRev = true;
//   LH_KFE_rev.axis << 1,0,0;
//   LH_KFE_rev.theta = gc[15]; //TOOD
//   LH_KFE_rev.parentJoint = &LH_thigh_fixed_LH_KFE;
//   LH_KFE_rev.childlinks.push_back(&LH_SHANK);
//   LH_KFE_rev.childjoints.push_back(&LH_shank_LH_shank_fixed);

//   //Link LH_SHANK;
//   LH_SHANK.parentJoint = &LH_KFE_rev;
//   LH_SHANK.mass = 0.001;
//   LH_SHANK.inertia_b = inertia_tensor(0.000001, 0.0, 0.0, 0.000001, 0.0, 0.000001);

//   //Joint LH_shank_LH_shank_fixed;
//   LH_shank_LH_shank_fixed.parentJoint = &LH_KFE_rev;
//   LH_shank_LH_shank_fixed.childlinks.push_back(&LH_shank_fixed);
//   LH_shank_LH_shank_fixed.childjoints.push_back(&LH_shank_fixed_LH_FOOT);
//   LH_shank_LH_shank_fixed.origin.rpy << 0, 0, -1.57079632679;

//   //Link LH_shank_fixed;
//   LH_shank_fixed.parentJoint = &LH_shank_LH_shank_fixed;
//   LH_shank_fixed.origin.xyz << -0.03463, 0.00688, 0.00098;
//   LH_shank_fixed.mass = 0.33742;
//   LH_shank_fixed.inertia_b = inertia_tensor(0.00032748005, -2.142561e-05, -1.33942e-05, 0.00110974122, 7.601e-08, 0.00089388521);

//   //Joint LH_shank_fixed_LH_FOOT;
//   LH_shank_fixed_LH_FOOT.parentJoint = &LH_shank_LH_shank_fixed;
//   LH_shank_fixed_LH_FOOT.childlinks.push_back(&LH_FOOT);
//   LH_shank_fixed_LH_FOOT.origin.xyz << -0.08795, 0.01305, -0.33797;

//   //Link LH_FOOT;
//   LH_FOOT.isLeaf = true;
//   LH_FOOT.parentJoint = &LH_shank_fixed_LH_FOOT;
//   LH_FOOT.origin.xyz << -0.00948, -0.00948, 0.1468;
//   LH_FOOT.mass = 0.25;
//   LH_FOOT.inertia_b = inertia_tensor(0.00317174097, -2.63048e-06, -6.815581e-05, 0.00317174092, 6.815583e-05, 8.319196e-05);
// //END OF LH

// //RH
//   //Joint base_RH_HAA;
//   base_RH_HAA.parentJoint = &base;
//   base_RH_HAA.childlinks.push_back(&RH_HAA);
//   base_RH_HAA.childjoints.push_back(&RH_HAA_rev);
//   base_RH_HAA.origin.rpy << 2.61799387799, 0.0, -3.14159265359;
//   base_RH_HAA.origin.xyz << -0.2999, -0.104, 0.0;

//   //Link RH_HAA;
//   RH_HAA.parentJoint = &base_RH_HAA;
//   RH_HAA.origin.xyz << -0.063, 7e-05, 0.00046;
//   RH_HAA.mass = 2.04;
//   RH_HAA.inertia_b = inertia_tensor(0.001053013, 4.527e-05, 8.855e-05, 0.001805509, 9.909e-05, 0.001765827);

//   //Joint RH_HAA_rev;
//   RH_HAA_rev.jointID = 15;
//   RH_HAA_rev.isRev = true;
//   RH_HAA_rev.axis << -1,0,0;
//   RH_HAA_rev.theta = gc[16]; //TOOD
//   RH_HAA_rev.parentJoint = &base_RH_HAA;
//   RH_HAA_rev.childlinks.push_back(&RH_HIP);
//   RH_HAA_rev.childjoints.push_back(&RH_HIP_RH_hip_fixed);

//   //Link RH_HIP;
//   RH_HIP.parentJoint = &RH_HAA_rev;
//   RH_HIP.mass = 0.001;
//   RH_HIP.inertia_b = inertia_tensor(0.000001, 0.0, 0.0, 0.000001, 0.0, 0.000001);

//   //Joint RH_HIP_RH_hip_fixed;
//   RH_HIP_RH_hip_fixed.parentJoint = &RH_HAA_rev;
//   RH_HIP_RH_hip_fixed.childlinks.push_back(&RH_hip_fixed);
//   RH_HIP_RH_hip_fixed.childjoints.push_back(&RH_hip_fixed_RH_HFE);
//   RH_HIP_RH_hip_fixed.origin.rpy << 2.61799387799, 0.0, -3.14159265359;

//   //Link RH_hip_fixed;
//   RH_hip_fixed.parentJoint = &RH_HIP_RH_hip_fixed;
//   RH_hip_fixed.origin.xyz << -0.048, -0.008, -0.003;
//   RH_hip_fixed.mass = 0.74;
//   RH_hip_fixed.inertia_b = inertia_tensor(0.001393106, 8.4012e-05, -2.3378e-05, 0.003798579, -7.1319e-05, 0.003897509);

//   //Joint RH_hip_fixed_RH_HFE;
//   RH_hip_fixed_RH_HFE.parentJoint = &RH_HIP_RH_hip_fixed;
//   RH_hip_fixed_RH_HFE.childlinks.push_back(&RH_HFE);
//   RH_hip_fixed_RH_HFE.childjoints.push_back(&RH_HFE_rev);
//   RH_hip_fixed_RH_HFE.origin.rpy << 0, 0, -1.57079632679;
//   RH_hip_fixed_RH_HFE.origin.xyz << -0.0599, -0.08381, 0.0;

//   //Link RH_HFE;
//   RH_HFE.parentJoint = &RH_hip_fixed_RH_HFE;
//   RH_HFE.origin.xyz << -0.063, 7e-05, 0.00046;
//   RH_HFE.mass = 2.04;
//   RH_HFE.inertia_b = inertia_tensor(0.001053013, 4.527e-05, 8.855e-05, 0.001805509, 9.909e-05, 0.001765827);

//   //Joint RH_HFE_rev;
//   RH_HFE_rev.jointID = 16;
//   RH_HFE_rev.isRev = true;
//   RH_HFE_rev.axis << -1,0,0;
//   RH_HFE_rev.theta = gc[17]; //TOOD
//   RH_HFE_rev.parentJoint = &RH_hip_fixed_RH_HFE;
//   RH_HFE_rev.childlinks.push_back(&RH_THIGH);
//   RH_HFE_rev.childjoints.push_back(&RH_THIGH_RH_thigh_fixed);

//   //Link RH_THIGH;
//   RH_THIGH.parentJoint = &RH_HFE_rev;
//   RH_THIGH.mass = 0.001;
//   RH_THIGH.inertia_b = inertia_tensor(0.000001, 0.0, 0.0, 0.000001, 0.0, 0.000001);

//   //Joint RH_THIGH_RH_thigh_fixed;
//   RH_THIGH_RH_thigh_fixed.parentJoint = &RH_HFE_rev;
//   RH_THIGH_RH_thigh_fixed.childlinks.push_back(&RH_thigh_fixed);
//   RH_THIGH_RH_thigh_fixed.childjoints.push_back(&RH_thigh_fixed_RH_KFE);
//   RH_THIGH_RH_thigh_fixed.origin.rpy << 0, 0, 1.57079632679;

//   //Link RH_thigh_fixed;
//   RH_thigh_fixed.parentJoint = &RH_THIGH_RH_thigh_fixed;
//   RH_thigh_fixed.origin.xyz << 0.0, -0.018, -0.169;
//   RH_thigh_fixed.mass = 1.03;
//   RH_thigh_fixed.inertia_b = inertia_tensor(0.018644469, 5.2e-08, -1.0157e-05, 0.019312599, -0.002520077, 0.002838361);

//   //Joint RH_thigh_fixed_RH_KFE;
//   RH_thigh_fixed_RH_KFE.parentJoint = &RH_THIGH_RH_thigh_fixed;
//   RH_thigh_fixed_RH_KFE.childlinks.push_back(&RH_KFE);
//   RH_thigh_fixed_RH_KFE.childjoints.push_back(&RH_KFE_rev);
//   RH_thigh_fixed_RH_KFE.origin.rpy << 0, 0, -1.57079632679;
//   RH_thigh_fixed_RH_KFE.origin.xyz << 0.0, -0.1003, -0.285;

//   //Link RH_KFE;
//   RH_KFE.parentJoint = &RH_thigh_fixed_RH_KFE;
//   RH_KFE.origin.xyz << -0.063, 7e-05, 0.00046;
//   RH_KFE.mass = 2.04;
//   RH_KFE.inertia_b = inertia_tensor(0.001053013, 4.527e-05, 8.855e-05, 0.001805509, 9.909e-05, 0.001765827);

//   //Joint RH_KFE_rev;
//   RH_KFE_rev.jointID = 17;
//   RH_KFE_rev.isRev = true;
//   RH_KFE_rev.axis << -1,0,0;
//   RH_KFE_rev.theta = gc[18]; //TOOD
//   RH_KFE_rev.parentJoint = &RH_thigh_fixed_RH_KFE;
//   RH_KFE_rev.childlinks.push_back(&RH_SHANK);
//   RH_KFE_rev.childjoints.push_back(&RH_shank_RH_shank_fixed);

//   //Link RH_SHANK;
//   RH_SHANK.parentJoint = &RH_KFE_rev;
//   RH_SHANK.mass = 0.001;
//   RH_SHANK.inertia_b = inertia_tensor(0.000001, 0.0, 0.0, 0.000001, 0.0, 0.000001);

//   //Joint RH_shank_RH_shank_fixed;
//   RH_shank_RH_shank_fixed.parentJoint = &RH_KFE_rev;
//   RH_shank_RH_shank_fixed.childlinks.push_back(&RH_shank_fixed);
//   RH_shank_RH_shank_fixed.childjoints.push_back(&RH_shank_fixed_RH_FOOT);
//   RH_shank_RH_shank_fixed.origin.rpy << 0, 0, 1.57079632679;

//   //Link RH_shank_fixed;
//   RH_shank_fixed.parentJoint = &RH_shank_RH_shank_fixed;
//   RH_shank_fixed.origin.xyz << -0.03463, -0.00688, 0.00098;
//   RH_shank_fixed.mass = 0.33742;
//   RH_shank_fixed.inertia_b = inertia_tensor(0.00032748005, 2.142561e-05, -1.33942e-05, 0.00110974122, -7.601e-08, 0.00089388521);

//   //Joint RH_shank_fixed_RH_FOOT;
//   RH_shank_fixed_RH_FOOT.parentJoint = &RH_shank_RH_shank_fixed;
//   RH_shank_fixed_RH_FOOT.childlinks.push_back(&RH_FOOT);
//   RH_shank_fixed_RH_FOOT.origin.xyz << -0.08795, -0.01305, -0.33797;

//   //Link RH_FOOT;
//   RH_FOOT.isLeaf = true;
//   RH_FOOT.parentJoint = &RH_shank_fixed_RH_FOOT;
//   RH_FOOT.origin.xyz << -0.00948, 0.00948, 0.1468;
//   RH_FOOT.mass = 0.25;
//   RH_FOOT.inertia_b = inertia_tensor(0.00317174097, 2.63048e-06, -6.815581e-05, 0.00317174092, -6.815583e-05, 8.319196e-05);
// //END OF RH