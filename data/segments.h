#ifndef SEGMENTS_H
#define SEGMENTS_H

#include <data/array.h>

#include <boost/property_tree/ptree.hpp>

#include <array>

/*! |brief This class contains info about the segments in the network */
struct segments {
    /*! Valid segment types */
    static constexpr std::array<char, 6> valid_types = {'0', '1', '2', 'S', 'X', 'D'};
    
    /*! Easternmost extreme point of the segments */
    uint_vector e_ext;
    
    /*! Westernmost extreme point of the segments */
    uint_vector w_ext;
    
    /*! Minimum distance from the eastern terminal */
    double_vector e_min_dist;
    
    /*! Minimum distance from the western terminal */
    double_vector w_min_dist;
    
    /*! Segments' length */
    double_vector length;
    
    /*! Segments' "original" length.
     *  This is different from the normal length only for sidings, because the normal
     *  length also takes into account the switches. This original length is used for
     *  comparisons against the train's length and to determine the travel time
     */
    double_vector original_length;
    
    /*! Segments' types:
     *      * 0 for bidirectional main track
     *      * 1 for eastbound main track
     *      * 2 for westbound main track
     *      * S for siding
     *      * X for crossover
     *      * D for "dummy", i.e. sigma and tau
     */
    char_vector type;
    
    /*! True iff the segment is eastbound */
    bool_vector is_eastbound;
    
    /*! True iff the segment is westbound */
    bool_vector is_westbound;
    
    /*! Empty constructor */
    segments() {}
    
    /*! Construct from ptree representing the JSON data file */
    segments(const boost::property_tree::ptree& pt);
};

#endif