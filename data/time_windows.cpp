#include <data/time_windows.h>

#include <cassert>

using namespace boost::property_tree;
using namespace boost;

time_windows::time_windows(const ptree& pt, unsigned int ni) {
    wt_left = std::min(ni, pt.get<unsigned int>("want_time_tw_start"));
    wt_right = std::min(ni, pt.get<unsigned int>("want_time_tw_end"));
    sa_right = std::min(ni, pt.get<unsigned int>("schedule_tw_end"));
}