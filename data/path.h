#ifndef PATH_H
#define PATH_H

#include <data/array.h>
#include <data/data.h>

/*! \brief This class describes the path of a train in the network */
struct path {
    /*! This class simply repreesnts a segment at a certain time interval */
    struct node {
        /*! The segment's id */
        unsigned int seg;
        
        /*! The time interval */
        unsigned int t;
        
        /*! Basic constructor */
        node(unsigned int seg, unsigned int t) : seg{seg}, t{t} {}
    };
    
    /*! The problem data */
    const data& d;
    
    /*! Id of the train */
    unsigned int train;
    
    /*! The variables matrix relative to the train, e.g. coming from the MIP solver */
    uint_matrix_3d x;
    
    /*! The succession of nodes visited by the train */
    bv<node> p;
    
    /*! Constructs the path starting from the variables matrix */
    path(const data& d, unsigned int train, uint_matrix_3d x);
    
    /*! Prints a human-readable summary of the path */
    auto print_summary() const -> void;
};

#endif