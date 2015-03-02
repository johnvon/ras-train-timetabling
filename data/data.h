#ifndef DATA_H
#define DATA_H

#include <boost/property_tree/ptree.hpp>

#include <data/array.h>
#include <data/graph.h>
#include <data/instance.h>
#include <data/mows.h>
#include <data/network.h>
#include <data/prices.h>
#include <data/segments.h>
#include <data/speeds.h>
#include <data/time_windows.h>
#include <data/trains.h>
#include <params/params.h>

/*
    SEGMENTS:       0 = sigma
                    1 ... ns = actual segments
                    ns + 1 = tau

    TRAINS:         0 ... nt

    TIME INTERVALS: 0 = reserved for sigma
                    1 ... ni = actual times (sigma and tau can also use them)
                    ni + 1 = reserved for tau
*/

/*! \brief This class contains all the data relative to an instance */
struct data {
    /*! Information on the instance */
    instance ins;
    
    /*! Information about the speeds */
    speeds spd;
    
    /*! Information on width of time windows */
    time_windows tiw;
    
    /*! Information about prices to pay when violating soft contraints */
    prices pri;
    
    /*! Information about the segments */
    segments seg;
    
    /*! Information about the MOWs */
    mows mnt;
    
    /*! Information about the trains */
    trains trn;
    
    /*! Information about the network */
    network net;
    
    /*! Adjacency matrix and other info about the graph */
    graph gr;
    
    /*! NUmber of trains */
    unsigned int nt;
    
    /*! Number of segments */
    unsigned int ns;
    
    /*! Number of time intervals */
    unsigned int ni;

    /*! Headway between two trains occupying the same segment */
    unsigned int headway;
    
    /*! Reference to program params */
    const params& p;
    
    /*! Build data from a JSON data file and the parameters */
    data(const std::string& file_name, const params& p);
    
private:
    auto create_trains(const boost::property_tree::ptree& pt) -> void;
        auto calculate_trains_max_speeds() -> void;
        auto calculate_trains_origin_and_destination_segments() -> void;
        auto calculate_trains_unpreferred_segments() -> void;
        auto calculate_trains_sa_segments() -> void;
        
    auto create_network() -> void;
        auto calculate_times() -> void;
        auto calculate_main_tracks() -> void;
    
    auto create_graphs() -> void;
        auto calculate_deltas() -> void;
        auto calculate_vertices() -> void;
        auto calculate_starting_arcs() -> void;
        auto calculate_ending_arcs() -> void;
        auto calculate_escape_arcs() -> void;
        auto calculate_stop_arcs() -> void;
        auto calculate_movement_arcs() -> void;
        auto cleanup_graph() -> void;
        auto calculate_costs() -> void;
};

#endif