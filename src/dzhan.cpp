#include "raisim/World.hpp"
#include "raisim/RaisimServer.hpp"

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>

#define _MAKE_STR(x) __MAKE_STR(x)
#define __MAKE_STR(x) #x

int main() {

  // Open the CSV file
  std::ifstream file("rot.csv");
  if (!file.is_open()) {
      std::cerr << "Error opening file!" << std::endl;
      return 1;
  }

  // Define a 2D vector to store the CSV data
  std::vector<std::vector<double>> csvData;

  // Read and process each line
  std::string line;
  while (std::getline(file, line)) {
      // Create a stringstream from the line
      std::stringstream ss(line);
      
      // Temporary vector to store values for one row
      std::vector<double> row;
      
      // Read each value from the stringstream and push it into the row vector
      double value;
      char comma; // To store the comma, but not used
      while (ss >> value >> comma) {
          row.push_back(value);
      }

      // Push the row into the 2D vector
      csvData.push_back(row);
  }

  // Close the file
  file.close();

  std::cout << "HERE   " << csvData[1][8] << std::endl;

  raisim::World world;
  world.addGround();

  raisim::RaisimServer server(&world);

  auto monkey = server.addVisualMesh("monkey", std::string(_MAKE_STR(RESOURCE_DIR)) + "/monkey/monkey.obj");
  monkey->setColor(0,0,1,1);
  monkey->setPosition(0,0,2);

  server.launchServer();

  Eigen::Matrix3d rot;

  std::cout << "START" << std::endl;

  while(true){
    for(int step = 0; step<6000; step++){
      rot << csvData[step][0], csvData[step][1], csvData[step][2],
          csvData[step][3], csvData[step][4], csvData[step][5],
          csvData[step][6], csvData[step][7], csvData[step][8];

      raisim::Vec<4> ori;
      raisim::rotMatToQuat(rot, ori);
      monkey->setOrientation(ori.e());
      raisim::MSLEEP(5);
    }
    std::cout << "END" << std::endl;
  }


  while(true) {
    raisim::MSLEEP(4);
  }

  server.killServer();

  return 0;
}
