#ifndef TRAINS_H
#define TRAINS_H

#include <data/array.h>

#include <utility>

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
    
    /*! For (tr,s) is true iff s is unpreferred for tr */
    uint_matrix_2d unpreferred_segs;
    
    /*! Indexed over (tr,n) is the time at which train tr should be at its n-th SA point */
    uint_matrix_2d sa_times;
    
    /*! List of SA points' extremes for each train */
    uint_matrix_2d sa_ext;
    
    /*! Indexed over (tr,n) contains the list of segments corresponding to tr's n-th SA point */
    uint_matrix_3d sa_segs;
    
    /*! First time we need tau, i.e. earliest possible arrival time at the destination terminal */
    uint_vector first_time_we_need_tau;
    
    /*! Empty constructor */
    trains() {}
    
    /*! Basic constructor */
    trains( uint_vector want_time,
            uint_vector entry_time,
            uint_vector tob,
            double_vector speed_multi,
            double_vector speed_max,
            double_vector length,
            uint_vector sa_num,
            char_vector type,
            bool_vector is_sa,
            bool_vector is_heavy,
            bool_vector is_eastbound,
            bool_vector is_westbound,
            bool_vector is_hazmat,
            uint_vector orig_ext,
            uint_vector dest_ext,
            uint_matrix_2d orig_segs,
            uint_matrix_2d dest_segs,
            uint_matrix_2d unpreferred_segs,
            uint_matrix_2d sa_times,
            uint_matrix_2d sa_ext,
            uint_matrix_3d sa_segs,
            uint_vector first_time_we_need_tau
    ) :     want_time(std::move(want_time)),
            entry_time(std::move(entry_time)),
            tob(std::move(tob)),
            speed_multi(std::move(speed_multi)),
            speed_max(std::move(speed_max)),
            length(std::move(length)),
            sa_num(std::move(sa_num)),
            type(std::move(type)),
            is_sa(std::move(is_sa)),
            is_heavy(std::move(is_heavy)),
            is_eastbound(std::move(is_eastbound)),
            is_westbound(std::move(is_westbound)),
            is_hazmat(std::move(is_hazmat)),
            orig_ext(std::move(orig_ext)),
            dest_ext(std::move(dest_ext)),
            orig_segs(std::move(orig_segs)),
            dest_segs(std::move(dest_segs)),
            unpreferred_segs(std::move(unpreferred_segs)),
            sa_times(std::move(sa_times)),
            sa_ext(std::move(sa_ext)),
            sa_segs(std::move(sa_segs)),
            first_time_we_need_tau(std::move(first_time_we_need_tau)) {}
};

#endif