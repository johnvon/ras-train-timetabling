#include <data/speeds.h>

#include <cassert>

using namespace boost::property_tree;
using namespace boost;

speeds::speeds(const ptree& pt) {
    ew = pt.get<double>("speed_ew");
    we = pt.get<double>("speed_we");
    siding = pt.get<double>("speed_siding");
    swi = pt.get<double>("speed_switch");
    xover = pt.get<double>("speed_xover");
        
    assert(ew > 0);
    assert(we > 0);
    assert(siding > 0);
    assert(swi > 0);
    assert(xover > 0);
}