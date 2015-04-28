#ifndef GRAPH_H
#define GRAPH_H

#include <data/array.h>
#include <data/mows.h>
#include <data/network.h>
#include <data/prices.h>
#include <data/segments.h>
#include <data/time_windows.h>
#include <data/trains.h>
#include <params/params.h>

/*! This class contains info on the time-expanded graph */
struct graph {
    /*! Number of nodes in the graph, for each train */
    uint_vector n_nodes;
    
    /*! Number of arcs in the graph, for each train */
    uint_vector n_arcs;
    
    /*! Indexed as (s, t), is true iff the couple (s, t) is a node in some train's arc */
    bool_matrix_2d v_for_someone;
    
    /*! Indexed over (tr, s1) contains the list of segments connected to s1 in tr's running direction (including s1) */
    uint_matrix_3d delta;
    
    /*! Indexed over (tr, s1) contains the list of segments connected to s1 in the direction opposite to tr's running direction (including s1) */
    uint_matrix_3d inverse_delta;
    
    /*! Indexed over (tr, s1) contains the list of segments connected to s1 in tr's running direction (excluding s1) */
    uint_matrix_3d bar_delta;
    
    /*! Indexed over (tr, s1) contains the list of segments connected to s1 in the direction opposite to tr's running direction (excluding s1) */
    uint_matrix_3d bar_inverse_delta;
    
    /*! Indexed over (s, t) contais the list of trains whose graph has the vertex (s,t) */
    uint_matrix_3d trains_for;
    
    /*! Indexed as (tr, s, t), is true iff (s, t) is a vertex in tr's graph */
    bool_matrix_3d v;
    
    /*! Indexed as (tr, s1, t, s2), is true iff there is an arc from (s1, t) to (s2, t + 1) in tr's graph */
    bool_matrix_4d adj;
    
    /*! Indexed as (tr, s, t), is the number of arcs going out from (s, t) in tr's graph */
    uint_matrix_3d n_out;
    
    /*! Indexed as (tr, s, t), is the number of arcs coming in from (s, t) in tr's graph */
    uint_matrix_3d n_in;
    
    /*! Indexed as (tr, s1, t, s2) it is the cost of arc (s1, t) -> (s2, t + 1) in tr's graph */
    double_matrix_4d costs;
    
    /*! Indexed over tr, it is the first time we need tau in tr's graph, i.e. earliest possible arrival time at the destination terminal */
    uint_vector first_time_we_need_tau;
    
    /*! Empty constructor */
    graph() {}
    
    /*! Construct from data already read from the JSON data file */
    graph(unsigned int nt, unsigned int ns, unsigned int ni, const params& p, const trains& trn, const mows& mnt, const segments& seg, const network& net, const time_windows& tiw, const prices& pri);
    
private:
    
    auto calculate_deltas(unsigned int nt, unsigned int ns, const trains& trn, const segments& seg) -> void;
    auto calculate_vertices(unsigned int nt, unsigned int ns, unsigned int ni, const params& p, const trains& trn, const mows& mnt, const segments& seg, const network& net) -> void;
    auto calculate_starting_arcs(unsigned int nt, unsigned int ni, const params& p, const trains& trn, const segments& seg, const network& net) -> void;
    auto calculate_ending_arcs(unsigned int nt, unsigned int ns, unsigned int ni, const params& p, const trains& trn, const network& net) -> void;
    auto calculate_escape_arcs(unsigned int nt, unsigned int ns, unsigned int ni) -> void;
    auto calculate_stop_arcs(unsigned int nt, unsigned int ns, unsigned int ni, const network& net) -> void;
    auto calculate_movement_arcs(unsigned int nt, unsigned int ns, unsigned int ni, const network& net) -> void;
    auto cleanup(unsigned int nt, unsigned int ns, unsigned int ni) -> void;
    auto calculate_costs(unsigned int nt, unsigned int ns, unsigned int ni, const trains& trn, const network& net, const time_windows& tiw, const prices& pri) -> void;
};

#endif