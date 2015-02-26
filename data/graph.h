#ifndef GRAPH_H
#define GRAPH_H

#include <data/array.h>

#include <utility>

/*! This class contains info on the time-expanded graph */
struct graph {
    /*! Number of nodes in the graph */
    unsigned int n_nodes;
    
    /*! Number of arcs in the graph */
    unsigned int n_arcs;
    
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
    
    /*! Indexed as (tr, s1, t, s2), is true iff there is an arc from (s1, t) to (s2, t+1) in tr's graph */
    bool_matrix_4d adj;
    
    /*! Indexed as (tr, s, t), is the number of arcs going out from (s, t) in tr's graph */
    uint_matrix_3d n_out;
    
    /*! Indexed as (tr, s, t), is the number of arcs coming in from (s, t) in tr's graph */
    uint_matrix_3d n_in;
    
    /*! Empty constructor */
    graph() {}
    
    /*! Basic constructor */
    graph(  unsigned int n_nodes,
            unsigned int n_arcs,
            bool_matrix_2d v_for_someone,
            uint_matrix_3d delta,
            uint_matrix_3d inverse_delta,
            uint_matrix_3d bar_delta,
            uint_matrix_3d bar_inverse_delta,
            uint_matrix_3d trains_for,
            bool_matrix_3d v,
            bool_matrix_4d adj,
            uint_matrix_3d n_out,
            uint_matrix_3d n_in)
    :       n_nodes{n_nodes},
            n_arcs{n_arcs},
            v_for_someone(std::move(v_for_someone)),
            delta(std::move(delta)),
            inverse_delta(std::move(inverse_delta)),
            bar_delta(std::move(bar_delta)),
            bar_inverse_delta(std::move(bar_inverse_delta)),
            trains_for(std::move(trains_for)),
            v(std::move(v)),
            adj(std::move(adj)),
            n_out(std::move(n_out)),
            n_in(std::move(n_in)) {}
};

#endif