#include "raisim/RaisimServer.hpp"
#include "Algorithms_combined.hpp"

#define _MAKE_STR(x) __MAKE_STR(x)
#define __MAKE_STR(x) #x

int main(int argc, char* argv[]) {
  auto binaryPath = raisim::Path::setFromArgv(argv[0]);

  // create raisim world
  raisim::World world; // physics world
  raisim::RaisimServer server(&world);

  // anymal
  auto anymal = world.addArticulatedSystem(std::string(_MAKE_STR(RESOURCE_DIR)) + "/2DrobotArm/robot_3D.urdf");

  // anymal configuration
  Eigen::VectorXd gc(anymal->getGeneralizedCoordinateDim()), gv(anymal->getDOF()), gf(anymal->getDOF());
  Eigen::VectorXd nonlinearity(anymal->getDOF());
  Eigen::MatrixXd massMatrix(anymal->getDOF(), anymal->getDOF());
  Eigen::VectorXd accel(anymal->getDOF());

  gc = Eigen::VectorXd::Random(anymal->getGeneralizedCoordinateDim());
  gv = Eigen::VectorXd::Random(anymal->getDOF());
  gf = Eigen::VectorXd::Zero(anymal->getDOF());

  double error, dt = 0.001;

  server.launchServer();

  for(int i=0; i<100000; i++){

    anymal->setState(gc, gv);
    anymal->setGeneralizedForce(gf);
    massMatrix = anymal->getMassMatrix().e();
    nonlinearity = anymal->getNonlinearities({0,0,-9.81}).e();

    accel = computeGeneralizedAcceleration(gc, gv, gf);

    error = (accel - massMatrix.inverse() * (gf-nonlinearity)).norm();
    if(error< 1e-8) 
      std::cout<<"passed "<<std::endl;
    else
      std::cout<<"failed "<<std::endl;

    gv += accel * dt;
    gc += gv * dt;

    raisim::MSLEEP(1000);
  }

  while(true) {
    raisim::MSLEEP(4);
  }

  server.killServer();
  
  return 0;
}
