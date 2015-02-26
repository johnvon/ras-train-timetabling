#ifndef NETWORK_H
#define NETWORK_H

#include <data/array.h>

#include <utility>

/*! This class contains info about segment classes in the network and the relationships among segments and between segments and trains */
struct network {
    /*! List of sidings */
    uint_vector sidings;
    
    /*! List of cross-overs */
    uint_vector xovers;
    
    /*! For (tr, s) it's the minimum time at which train tr can arrive at segment s */
    uint_matrix_2d min_time_to_arrive;
    
    /*! For (tr, s) it's the minimum time train tr needs to occupy segment s */
    uint_matrix_2d min_travel_time;
    
    /*! Indexed over s; if s is a siding, it contains a list of corresponding main segments; otherwise, it contains an empty list */
    uint_matrix_2d main_tracks;
    
    /*! For (tr, s) is true iff segment s is unpreferred for train tr */
    bool_matrix_2d unpreferred;
    
    /*! For (s1, s2) is true iff s1 and s2 are connected, i.e. share a junction */
    bool_matrix_2d connected;
    
    /*! Empty constructor */
    network() {}
    
    /*! Basic constructor */
    network(    uint_vector sidings,
                uint_vector xovers,
                uint_matrix_2d min_time_to_arrive,
                uint_matrix_2d min_travel_time,
                uint_matrix_2d main_tracks,
                bool_matrix_2d unpreferred,
                bool_matrix_2d connected
    ) :         sidings(std::move(sidings)),
                xovers(std::move(xovers)),
                min_time_to_arrive(std::move(min_time_to_arrive)),
                min_travel_time(std::move(min_travel_time)),
                main_tracks(std::move(main_tracks)),
                unpreferred(std::move(unpreferred)),
                connected(std::move(connected)) {}
};

#endif