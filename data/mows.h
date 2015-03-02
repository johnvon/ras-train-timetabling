#ifndef MOWS_H
#define MOWS_H

#include <data/array.h>
#include <data/segments.h>

/*! \brief This class contains info on the maintenance of way (MOW) on the network */
struct mows {
    /*! Easternmost extremes of the MOWs */
    uint_vector e_ext;
    
    /*! Westernmost extreme of the MOWs */
    uint_vector w_ext;
    
    /*! Start time of the MOWs */
    uint_vector start_time;
    
    /*! End time of the MOWs */
    uint_vector end_time;
    
    /*! Is true for (s,t) iff segment s is interested by a MOW at time t */
    bool_matrix_2d is_mow;
    
    /*! Empty constructor */
    mows() {}
    
    /*! Construct from ptree representing the JSON data file */
    mows(const boost::property_tree::ptree& pt, unsigned int ni, unsigned int ns, const segments& seg);
    
private:
    auto calculate_is_mow(unsigned int ns, const segments& seg) -> void;
};

#endif