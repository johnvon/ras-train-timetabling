#ifndef TIME_WINDOWS_H
#define TIME_WINDOWS_H

/*! This class contains info on the time windows at the arrival terminal and at SA points */
struct time_windows {
    /*! Dimension of the left half-tw, starting from the train's want time at its arrival terimnal */
    unsigned int wt_left;
    
    /*! Dimension of the right half-tw, starting from the train's want time at its arrival terimnal */
    unsigned int wt_right;
    
    /*! Dimension of the right (and only) half-tw, starting from the train's SA time at its SA points */
    unsigned int sa_right;
    
    /*! Empty constructor */
    time_windows() {}
    
    /*! Basic constructor */
    time_windows(unsigned int wt_left, unsigned int wt_right, unsigned int sa_right) : wt_left{wt_left}, wt_right{wt_right}, sa_right{sa_right} {}
};

#endif