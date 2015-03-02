#include <boost/container/vector.hpp>

template<typename T>
using bv = boost::container::vector<T>;

using uint_vector = bv<unsigned int>;
using double_vector = bv<double>;
using char_vector = bv<char>;
using bool_vector = bv<bool>;

using uint_matrix_2d = bv<uint_vector>;
using uint_matrix_3d = bv<uint_matrix_2d>;

using bool_matrix_2d = bv<bool_vector>;
using bool_matrix_3d = bv<bool_matrix_2d>;
using bool_matrix_4d = bv<bool_matrix_3d>;

using double_matrix_2d = bv<double_vector>;
using double_matrix_3d = bv<double_matrix_2d>;
using double_matrix_4d = bv<double_matrix_3d>;