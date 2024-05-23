#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iostream>

using namespace std;

int main(){
    Eigen::Vector3d vec;
    vec << 1,2,3;
    Eigen::MatrixXd transposed = vec.transpose();
    cout << vec << endl;
    cout << transposed << endl;
    cout << transposed*vec << endl;
    
}