#include <data/prices.h>
#include <data/trains.h>

#include <cassert>

using namespace boost::property_tree;
using namespace boost;

prices::prices(const ptree& pt) {
    for(char cl = trains::first_train_class; cl <= trains::last_train_class; cl++) {
        auto price = pt.get_child("general_delay_price").get<double>(std::string(1,cl));
        
        assert(price > 0);
        
        delay.emplace(cl, price);
    }
    
    wt = pt.get<double>("terminal_delay_price");
    sa = pt.get<double>("schedule_delay_price");
    unpreferred = pt.get<double>("unpreferred_price");
                    
    assert(wt > 0);
    assert(sa > 0);
    assert(unpreferred > 0);
}