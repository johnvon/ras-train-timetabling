#include <boost/container/vector.hpp>

template<typename T>
using bv = boost::container::vector<T>;

using char_vector = bv<char>;

using uint_vector = bv<unsigned int>;
using uint_matrix_2d = bv<uint_vector>;
using uint_matrix_3d = bv<uint_matrix_2d>;
using uint_matrix_4d = bv<uint_matrix_3d>;

using bool_vector = bv<bool>;
using bool_matrix_2d = bv<bool_vector>;
using bool_matrix_3d = bv<bool_matrix_2d>;
using bool_matrix_4d = bv<bool_matrix_3d>;

using double_vector = bv<double>;
using double_matrix_2d = bv<double_vector>;
using double_matrix_3d = bv<double_matrix_2d>;
using double_matrix_4d = bv<double_matrix_3d>;

#if USE_CPLEX
    #include <ilcplex/ilocplex.h>

    using var_vector = IloNumVarArray;
    using var_matrix_2d = IloArray<var_vector>;
    using var_matrix_3d = IloArray<var_matrix_2d>;
    using var_matrix_4d = IloArray<var_matrix_3d>;

    using cst_vector = IloRangeArray;
    using cst_matrix_2d = IloArray<cst_vector>;
    using cst_matrix_3d = IloArray<cst_matrix_2d>;

    using num_vector = IloNumArray;
    using num_matrix_2d = IloArray<num_vector>;
    using num_matrix_3d = IloArray<num_matrix_2d>;
    using num_matrix_4d = IloArray<num_matrix_3d>;
#endif