#ifndef MOWS_H
#define MOWS_H

#include <data/array.h>

#include <utility>

/*! \brief This class contains info on the maintenance of way (MOW) on the network */
struct mows {
    /*! Easternmost extremes of the MOWs */
    uint_vector e_ext;
    
    /*! Westernmost extreme of the MOWs */
    uint_vector w_ext;
    
    /*! Start time of the MOWs */
    uint_vector start_time;
    
    /*! End time of the MOWs */
    uint_vector end_time;
    
    /*! Is true for (s,t) iff segment s is interested by a MOW at time t */
    bool_matrix_2d is_mow;
    
    /*! Empty constructor */
    mows() {}
    
    /*! Basic constructor */
    mows(    uint_vector e_ext,
            uint_vector w_ext,
            uint_vector start_time,
            uint_vector end_time,
            bool_matrix_2d is_mow
    ) :     e_ext(std::move(e_ext)),
            w_ext(std::move(w_ext)),
            start_time(std::move(start_time)),
            end_time(std::move(end_time)),
            is_mow(std::move(is_mow)) {}
};

#endif