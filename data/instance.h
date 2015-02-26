#ifndef INSTANCE_H
#define INSTANCE_H

#include <string>

/*! \brief This class contains basic info on the instance at hand, such as its name and its data file */
struct instance {
    /*! Instance name */
    std::string name;
    
    /*! File name of the instance data file (JSON) */
    std::string file_name;
    
    /*! Empty constructor */
    instance() {}
    
    /*! Basic constructor */
    instance(std::string name, std::string file_name) : name{name}, file_name{file_name} {}
};

#endif