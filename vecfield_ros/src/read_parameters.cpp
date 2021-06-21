// Read Yaml File with Parameters Class (Lib)
/*
Universidade Federal de Minas Gerais (UFMG) - 2020
Laboraorio CORO
Instituto Tecnologico Vale (ITV)
*/

#include "drone_class.h"


#define PI 3.1415926535

using namespace std;
using namespace Eigen;



// Constructor
parameters::parameters(std::string file_name){
  file = file_name;
  // read yaml file and set yaml node
  node = YAML::LoadFile(file);
  n_params = node["Drone_Dynamics"].size();

}

// get parameters
VectorXd parameters::get_parameters(){
    // setting vector of params according with the number of parameters in yaml file
    Eigen::VectorXd parameters_return(n_params);   
    int count = 0;
    // searching for parameters in yaml node
    for(YAML::const_iterator it=node["Drone_Dynamics"].begin();it!=node["Drone_Dynamics"].end();++it) {
      parameters_return[count] = it->second.as<double>();
      count++;
    }

    return parameters_return;
}


// destructor
parameters::~parameters(){
}



