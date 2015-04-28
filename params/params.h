#ifndef PARAMS_H
#define PARAMS_H

#include <string>

/*! \brief This class contains a representation of the params that are passed to the programme via a JSON file */
struct params {
    /*! \brief This class contains prams relative to CPLEX */
    struct cplex_params {
        /*! Number of threads CPLEX is allowed to use */
        unsigned int threads;
        
        /*! Hard time limit for the solver */
        unsigned int time_limit;
        
        /*! Empty constructor */
        cplex_params() {}
        
        /*! Basic constructor */
        cplex_params(   unsigned int threads,
                        unsigned int time_limit
        ) :             threads{threads},
                        time_limit{time_limit} {}
    };
    
    /*! \brief This class contains params relative to the heuristics */
    struct heuristics_params {
        /*! \brief This class contains params relative to the constructive heuristics */
        struct mip_constructive_params {
            /*! \brief This class contains params relative to the creation of a corridor */
            struct corridor_params {
                /*! Wether we want to create a corridor or not */
                bool active;
                
                /*! Max number of time intervals the train is allowed to be behind its theoretical fastest schedule at any time */
                unsigned int max_delay_over_fastest_route;
                
                /*! Empty constructor */
                corridor_params() {}
                
                /*! Basic constructor */
                corridor_params(bool active, unsigned int max_delay_over_fastest_route) : active{active}, max_delay_over_fastest_route{max_delay_over_fastest_route} {}
            };
            
            /*! Wether we want to run the MIP constructive heuristic or not */
            bool active;
            
            /*! Fix the starting time for each train as its entry time */
            bool fix_start;
            
            /*! Fix the ending time for each train as its want time */
            bool fix_end;
            
            /*! When a train could enter the scenario on a main track or on a siding, only allow entering on a main track */
            bool only_start_at_main;
            
            /*! Parameters relative to the corridor */
            corridor_params corridor;
            
            /*! Empty constructor */
            mip_constructive_params() {}
            
            /*! Basic constructor */
            mip_constructive_params(    bool active,
                                        bool fix_start,
                                        bool fix_end,
                                        bool only_start_at_main,
                                        corridor_params corridor
            ) :                         active{active},
                                        fix_start{fix_start},
                                        fix_end{fix_end},
                                        only_start_at_main{only_start_at_main},
                                        corridor{std::move(corridor)} {}
        };
        
        /*! Constructive heurstics parameters */
        mip_constructive_params     constructive;
    
        /*! Empty constructor */
        heuristics_params() {}
        
        /*! Default constructor */
        heuristics_params(mip_constructive_params constructive) : constructive{std::move(constructive)} {}
    };
    
    /*! Name of the file wehre to save the results */
    std::string         results_file;
    
    /*! Params relative to CPLEX */
    cplex_params        cplex;
    
    /*! Params relative to the heuristics */
    heuristics_params   heuristics;
    
    /*! Construct the params from the given params file */
    params(std::string file_name);
};

#endif