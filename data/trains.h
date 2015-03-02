#ifndef TRAINS_H
#define TRAINS_H

#include <data/array.h>
#include <data/segments.h>
#include <data/speeds.h>

#include <boost/property_tree/ptree.hpp>

/*! \brief This class contains info about trains in the instance */
struct trains {
    /*! Trains with TOB > than this are considered heavy */
    static constexpr unsigned int heavy_weight = 100;
    
    /*! The most prioritised train class */
    static constexpr char first_train_class = 'A';
    
    /*! The least prioritised train class */
    static constexpr char last_train_class = 'F';
    
    /*! Trains' want times */
    uint_vector want_time;
    
    /*! Trains' entry times */
    uint_vector entry_time;
    
    /*! Trains' TOB (ton per operating brake) */
    uint_vector tob;
    
    /*! Trains' speed multipliers */
    double_vector speed_multi;
    
    /*! Trains' maximum achievable speed at any point in the network */ 
    double_vector speed_max;
    
    /*! Trains's lengths */
    double_vector length;
    
    /*! Number of SA points for each train */
    uint_vector sa_num;
    
    /*! Trains' type (ranging from 'A' to 'F') */
    char_vector type;
    
    /*! True iff the train is SA */
    bool_vector is_sa;
    
    /*! True iff the train is heavy */
    bool_vector is_heavy;
    
    /*! True iff the train is eastbound */
    bool_vector is_eastbound;
    
    /*! True iff the train is westbound */
    bool_vector is_westbound;
    
    /*! True iff the train is HAZMAT (transporting HAZardous MATerial) */
    bool_vector is_hazmat;
    
    /*! List of trains' origin extremes */
    uint_vector orig_ext;
    
    /*! List of trains' destination extremes */
    uint_vector dest_ext;
    
    /*! List of origin segments for each train */
    uint_matrix_2d orig_segs;
    
    /*! List of destination segments for each train */
    uint_matrix_2d dest_segs;
    
    /*! Indexed over tr, is the list of unpreferred segments for tr */
    uint_matrix_2d unpreferred_segs;
    
    /*! Indexed over (tr,n) is the time at which train tr should be at its n-th SA point */
    uint_matrix_2d sa_times;
    
    /*! List of SA points' extremes for each train */
    uint_matrix_2d sa_ext;
    
    /*! Indexed over (tr,n) contains the list of segments corresponding to tr's n-th SA point */
    uint_matrix_3d sa_segs;
    
    /*! Empty constructor */
    trains() {}
    
    /*! Construct from ptree representing the JSON data file */
    trains(const boost::property_tree::ptree& pt, unsigned int nt, unsigned int ns, unsigned int ni, const speeds& spd, const segments& seg);
    
private:
    
    auto calculate_max_speeds(unsigned int nt, const speeds& spd) -> void;
    auto calculate_origin_and_destination_segments(unsigned int nt, unsigned int ns, const segments& seg) -> void;
    auto calculate_unpreferred_segments(unsigned int nt, unsigned int ns, const segments& seg) -> void;
    auto calculate_sa_segments(unsigned int nt, unsigned int ns, const segments& seg) -> void;
};

#endif