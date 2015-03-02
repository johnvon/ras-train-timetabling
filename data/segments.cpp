#include <data/segments.h>

#include <boost/foreach.hpp>

using namespace boost::property_tree;
using namespace boost;

constexpr std::array<char, 6> segments::valid_types;

segments::segments(const ptree& pt) {
    static constexpr unsigned int dummy = 999999u;
    
    // Sigma:
    e_ext.push_back(dummy - 1);
    w_ext.push_back(dummy - 2);
    e_min_dist.push_back(dummy - 3);
    w_min_dist.push_back(dummy - 4);
    length.push_back(dummy - 5);
    original_length.push_back(dummy - 6);
    type.push_back('D');
    is_eastbound.push_back(false);
    is_westbound.push_back(false);
    
    BOOST_FOREACH(const ptree::value_type& segment_child, pt.get_child("segments")) {
        e_ext.push_back(segment_child.second.get<unsigned int>("extreme_2"));
        w_ext.push_back(segment_child.second.get<unsigned int>("extreme_1"));
        e_min_dist.push_back(segment_child.second.get<double>("min_distance_from_e"));
        w_min_dist.push_back(segment_child.second.get<double>("min_distance_from_w"));
        type.push_back(segment_child.second.get<char>("type"));
        length.push_back(segment_child.second.get<double>("length"));
        
        if(type.back() == 'S') {
            original_length.push_back(segment_child.second.get<double>("siding_length"));
        } else {
            original_length.push_back(length.back());
        }
        
        is_eastbound.push_back(segment_child.second.get<bool>("eastbound"));
        is_westbound.push_back(segment_child.second.get<bool>("westbound"));
        
        assert(e_min_dist.back() >= 0);
        assert(w_min_dist.back() >= 0);
        assert(length.back() > 0);
        assert(original_length.back() <= length.back());
        assert(std::find(segments::valid_types.begin(), segments::valid_types.end(), type.back()) != segments::valid_types.end());
        assert(is_eastbound.back() || is_westbound.back());
    }
    
    // Tau:
    e_ext.push_back(dummy + 1);
    w_ext.push_back(dummy + 2);
    e_min_dist.push_back(dummy + 3);
    w_min_dist.push_back(dummy + 4);
    length.push_back(dummy + 5);
    original_length.push_back(dummy + 6);
    type.push_back('D');
    is_eastbound.push_back(false);
    is_westbound.push_back(false);
}