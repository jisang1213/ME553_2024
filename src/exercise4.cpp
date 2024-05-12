//
// Created by Jemin Hwangbo on 2024/05/12.
//

#include "raisim/RaisimServer.hpp"
#include "exercise4_20190837.hpp"

#define _MAKE_STR(x) __MAKE_STR(x)
#define __MAKE_STR(x) #x

int main(int argc, char* argv[]) {
  auto binaryPath = raisim::Path::setFromArgv(argv[0]);

  // create raisim world
  raisim::World world; // physics world

  // anymal
  auto anymal = world.addArticulatedSystem(std::string(_MAKE_STR(RESOURCE_DIR)) + "/anymal_c/urdf/anymal.urdf");

  // Create a random number generator engine
  std::random_device rd; // Obtain a random seed from the operating system
  std::mt19937 gen(rd()); // Standard Mersenne Twister engine
  std::uniform_real_distribution<> dis(-1.0, 1.0); // Uniform distribution between 0 and 1

  // anymal configuration
  Eigen::VectorXd gc(anymal->getGeneralizedCoordinateDim()), gv(anymal->getDOF());
  // gc << 0, 0, 0.54, 1.0, 0.0, 0.0, 0.0, 0.03, 0.4, -0.8, -0.03, 0.4, -0.8, 0.03, -0.4, 0.8, -0.03, -0.4, 0.8; /// Jemin: I'll randomize the gc, gv when grading
  // gv << 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0, 1.1, 1.2, 1.3, 1.4, 1.5, 1.6, 1.7, 1.8;

  for(int i=0; i<100; i++){
    gc = Eigen::VectorXd::Random(19);
    gv = Eigen::VectorXd::Random(18);
    anymal->setState(gc, gv);

    /// if you are using an old version of Raisim, you need this line
    world.integrate1();
    anymal->getMassMatrix();

    std::cout<<"nonlinearities should be \n"<< anymal->getNonlinearities({0,0,-9.81}).e().transpose()<<std::endl;
    std::cout<<"nonlinearities are \n"<< getNonlinearities(gc, gv).transpose()<<std::endl;

    double error = (getNonlinearities(gc, gv) - anymal->getNonlinearities({0,0,-9.81}).e()).norm();
    std::cout<<"error is "<< error<< std::endl;

    if(error < 1e-8){
      std::cout<<"passed "<<std::endl;
    }
    else{
      std::cout<<"failed "<<std::endl; 
      break;
    }
    
    if(i==99){
      std::cout<<"all passed"<<std::endl;
    }
  }

  return 0;
}
