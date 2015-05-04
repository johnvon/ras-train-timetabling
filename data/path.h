#ifndef PATH_H
#define PATH_H

#include <data/array.h>
#include <data/data.h>

#include <ostream>

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
    
    /*! A pointer to the problem data */
    const data* d;
    
    /*! Id of the train */
    unsigned int train;
    
    /*! The variables matrix relative to the train, e.g. coming from the MIP solver */
    uint_matrix_3d x;
    
    /*! The succession of nodes visited by the train */
    bv<node> p;
    
    /*! The cost of the path */
    double cost;
    
    /*! Constructs the path starting from the variables matrix */
    path(const data& d, unsigned int train, uint_matrix_3d x, double cost);
    
    /*! Makes a dummy path for the train - It goes from sigma to tau and costs nothing */
    path(const data& d, unsigned int train);
    
    /*! Marks the path as dummy */
    auto make_dummy() -> void;
    
    /*! Marks the path as empty, i.e. the path of a train yet to schedule - this is different fro ma dummy path */
    auto make_empty() -> void;
    
    /*! Prints a human-readable summary of the path */
    auto print_summary(std::ostream& where) const -> void;
    
    /*! Tells wether the path is dummy or not */
    auto is_dummy() const -> bool;
    
    /*! Tells wether the path is empty or not */
    auto is_empty() const -> bool;
};

#endif