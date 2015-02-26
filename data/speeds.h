#ifndef SPEEDS_H
#define SPEEDS_H

#include <cassert>

/*! \brief This class contains info on the speed limits on segments */
struct speeds {
    /*! Speed limit on main track westbound */
    double ew;
    
    /*! Speed limit on main track eastbound */
    double we;
    
    /*! Speed limit on siding */
    double siding;
    
    /*! Speed limit on switches */
    double swi;
    
    /*! Speed limit on cross-over */
    double xover;
    
    /*! Empty constructor */
    speeds() {}
    
    /*! Basic constructor */
    speeds(double ew, double we, double siding, double swi, double xover) : ew{ew}, we{we}, siding{siding}, swi{swi}, xover{xover} {}
};

#endif