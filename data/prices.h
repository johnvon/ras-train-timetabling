#ifndef PRICES_H
#define PRICES_H

#include <data/trains.h>

#include <boost/property_tree/ptree.hpp>

#include <unordered_map>

/*! \brief This class contains info about prices and penalties to pay */
struct prices {
    using delay_price_map = std::unordered_map<char, double>;
    
    /*! Cost of arriving one time unit outside the tw at the train's arrival terminal */
    double wt;
    
    /*! Cost of arriving one time unit outside the tw at the train's SA points */
    double sa;
    
    /* Cost of one time unit of usage of an unpreferred track */ 
    double unpreferred;
    
    /* Cost of one time unit of delay on any segment, given for each train class */
    delay_price_map delay;
    
    /*! Empty constructor */
    prices() {}
    
    /*! Construct from ptree representing the JSON data file */
    prices(const boost::property_tree::ptree& pt);
};

#endif