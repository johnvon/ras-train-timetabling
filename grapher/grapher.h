#ifndef GRAPHER_H
#define GRAPHER_H

#include <data/array.h>
#include <data/data.h>
#include <data/path.h>

#include <utility>
#include <vector>

/*! \brief This class takes care of outputting the results in a nice format, in order to create graphs */
struct grapher {
    /*! The problem data */
    const data& d;
    
    /*! The solution, given in terms of train paths */
    const bv<path>& paths;
    
    /*! Basic constructor */
    grapher(const data& d, const bv<path>& paths) : d{d}, paths{paths} {}
    
    /*! Write data to file */
    auto write_graph() -> void;
    
private:
    using series_data = std::vector<std::pair<double, double>>;
    using points_data = std::vector<series_data>;
    
    auto generate_points() -> points_data;
};

#endif