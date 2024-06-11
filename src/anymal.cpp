//
// Created by Jemin Hwangbo on 2024/05/12.
//

#include "raisim/RaisimServer.hpp"
#include "Algorithms_combined.hpp"

#define _MAKE_STR(x) __MAKE_STR(x)
#define __MAKE_STR(x) #x

int main(int argc, char* argv[]) {
  auto binaryPath = raisim::Path::setFromArgv(argv[0]);

  // create raisim world
  raisim::World world; // physics world

  // anymal
  auto anymal = world.addArticulatedSystem(std::string(_MAKE_STR(RESOURCE_DIR)) + "/anymal_c/urdf/anymal_copy.urdf");


  // anymal configuration
  Eigen::VectorXd gc(anymal->getGeneralizedCoordinateDim()), gv(anymal->getDOF()), gf(anymal->getDOF());

  for(int i=0; i<1; i++){
    gc = Eigen::VectorXd::Random(anymal->getGeneralizedCoordinateDim());
    gv = Eigen::VectorXd::Random(anymal->getDOF());
    gf = Eigen::VectorXd::Random(anymal->getDOF());
    
    anymal->setState(gc, gv);
    anymal->setGeneralizedForce(gf);

    /// if you are using an old version of Raisim, you need this line
    world.integrate1();
    anymal->getMassMatrix();

    //precalculate the mass matrix and nonlinearities
    Eigen::VectorXd nonlinearity(anymal->getDOF());
    Eigen::MatrixXd massMatrix(anymal->getDOF(), anymal->getDOF());
    massMatrix = anymal->getMassMatrix().e();
    nonlinearity = anymal->getNonlinearities({0,0,-9.81}).e();

    double error;

    //CRBA TESTER
    error = (getMassMatrix(gc) - massMatrix).norm();
    std::cout<<"error is "<< error<< std::endl;
    if(error < 1e-8)
      std::cout<<"CRBA passed "<<std::endl;
    else
      std::cout<<"CRBA failed "<<std::endl; 

    //RNE TESTER
    error = (getNonlinearities(gc, gv) - nonlinearity).norm();
    std::cout<<"error is "<< error<< std::endl;
    if(error < 1e-8){
      std::cout<<"RNE passed "<<std::endl;
    }
    else{
      std::cout<<"RNE failed "<<std::endl;
    }

    //ABA TESTER
    error = (computeGeneralizedAcceleration(gc, gv, gf) - massMatrix.inverse() * (gf-nonlinearity)).norm();
    std::cout<<"error:  "<<error<< std::endl;
    if(error< 1e-8) 
      std::cout<<"ABA passed "<<std::endl;
    else 
      std::cout<<"ABA failed " <<std::endl;
  }

  return 0;
}
