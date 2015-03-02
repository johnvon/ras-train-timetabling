#ifndef SPEEDS_H
#define SPEEDS_H

#include <boost/property_tree/ptree.hpp>

/*! \brief This class contains info on the speed limits on segments */
struct speeds {
    /*! Speed limit on main track westbound */
    double ew;
    
    /*! Speed limit on main track eastbound */
    double we;
    
    /*! Speed limit on siding */
    double siding;
    
    /*! Speed limit on switches */
    double swi;
    
    /*! Speed limit on cross-over */
    double xover;
    
    /*! Empty constructor */
    speeds() {}
    
    /*! Construct from ptree representing the JSON data file */
    speeds(const boost::property_tree::ptree& pt);
};

#endif