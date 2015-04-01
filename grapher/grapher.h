#ifndef GRAPHER_H
#define GRAPHER_H

#include <data/array.h>
#include <data/data.h>

/*! \brief This class takes care of outputting the results in a nice format, in order to create graphs */
struct grapher {
    /*! The problem data */
    const data& d;
    
    /*! The solution, given in terms of x variables */
    const uint_matrix_4d& x;
    
    /*! Basic constructor */
    grapher(const data& d, const uint_matrix_4d& x) : d{d}, x{x} {}
    
    /*! Write data to file */
    auto write_data() -> void;
};

#endif