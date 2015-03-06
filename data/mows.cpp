#include <data/mows.h>

#include <boost/foreach.hpp>

using namespace boost::property_tree;
using namespace boost;

mows::mows(const ptree& pt, unsigned int ni, unsigned int ns, const segments& seg) {
    BOOST_FOREACH(const ptree::value_type& mow_child, pt.get_child("mow")) {
        e_ext.push_back(mow_child.second.get<unsigned int>("extreme_2"));
        w_ext.push_back(mow_child.second.get<unsigned int>("extreme_1"));
        start_time.push_back(mow_child.second.get<unsigned int>("start_time"));
        end_time.push_back(mow_child.second.get<unsigned int>("end_time"));
    
        assert(start_time.back() < ni);
        assert(end_time.back() < ni);
        assert(end_time.back() - start_time.back() >= 0u);
    }
    
    is_mow = bool_matrix_2d(ns + 2, bool_vector(ni + 2, false));
    calculate_is_mow(ns, seg);
}

auto mows::calculate_is_mow(unsigned int ns, const segments& seg) -> void {
    for(auto m = 0u; m < e_ext.size(); m++) {
        for(auto s = 0u; s <= ns + 1; s++) {
            if(seg.e_ext.at(s) == e_ext.at(m) && seg.w_ext.at(s) == w_ext.at(m)) {
                for(auto t = start_time.at(m); t <= end_time.at(m); t++) {
                    is_mow.at(s).at(t) = true;
                }
            }
        }
    }
}