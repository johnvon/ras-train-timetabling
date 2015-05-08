#ifndef TIME_WINDOWS_H
#define TIME_WINDOWS_H

#include <boost/property_tree/ptree.hpp>

/*! This class contains info on the time windows at the arrival terminal and at SA points */
struct time_windows {
    /*! Dimension of the left half-tw, starting from the train's want time at its arrival terminal */
    unsigned int wt_left;
    
    /*! Dimension of the right half-tw, starting from the train's want time at its arrival terminal */
    unsigned int wt_right;
    
    /*! Dimension of the right (and only) half-tw, starting from the train's SA time at its SA points */
    unsigned int sa_right;
    
    /*! Empty constructor */
    time_windows() {}
    
    /*! Construct from ptree representing the JSON data file */
    time_windows(const boost::property_tree::ptree& pt, unsigned int ni);
};

#endif
