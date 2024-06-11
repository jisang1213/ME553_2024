#include "raisim/RaisimServer.hpp"

#define _MAKE_STR(x) __MAKE_STR(x)
#define __MAKE_STR(x) #x

int main(int argc, char* argv[]) {
  auto binaryPath = raisim::Path::setFromArgv(argv[0]);

  // create raisim world
  raisim::World world; // physics world

  // anymal
  auto anymal = world.addArticulatedSystem(std::string(_MAKE_STR(RESOURCE_DIR)) + "/2DrobotArm/robot_3D.urdf");
  
  // anymal configuration
  Eigen::VectorXd gc(anymal->getGeneralizedCoordinateDim()), gv(anymal->getDOF()), gf(anymal->getDOF());

  //set gc,gv,gf arbitrarily 
  gc << 0.1, 0.2, 0.3;
  gv << 0.4, 0.5, 0.6;
  gf << 0.7, 0.8, 0.9;

  for(int i=0; i<1; i++){

    //change gc,gv,gf randomly
    gc *= 1.3;
    gv *= 0.8;
    gf *= 0.5;
    
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

    std::cout<<"Raisim Nonlinearities: " << nonlinearity.transpose() << std::endl;
  }

  return 0;
}
