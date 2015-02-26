cmake_minimum_required(VERSION 2.8.4)
project(tsppddl)

# CPLEX ROOT DIR HINT
if(CMAKE_SYSTEM_NAME STREQUAL Darwin)
    set(CPLEX_ROOT_DIR "/opt/ibm" CACHE FILEPATH "IBM Cplex install dir")
endif()
if(CMAKE_SYSTEM_NAME STREQUAL Linux)
    set(CPLEX_ROOT_DIR "/opt/ibm/ILOG/CPLEX_Studio126" CACHE FILEPATH "IBM Cplex install dir")
endif()

# BOOST ROOT DIR HINT (Linux only)
if(CMAKE_SYSTEM_NAME STREQUAL Linux)
    set(BOOST_ROOT "~/boost-trunk/boost")
endif()

# COMPILER: COMMON FLAGS
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -DIL_STD -m64 -Wall -Werror")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -DDEBUG=true -ggdb")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -DNDEBUG -DDEBUG=false -O3 -flto")

# COMPILER: PLATFORM-DEPENDENT FLAGS
if(CMAKE_SYSTEM_NAME STREQUAL Darwin)
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -save-temps=obj")
endif()

# LOAD MODULES: cplex, boost
set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
find_package(Cplex)
find_package(Boost)

# INCLUDE DIRECTORIES
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(SYSTEM ${CPLEX_INCLUDE_DIRS})
include_directories(SYSTEM ${Boost_INCLUDEDIR})

# BUILD THE EXECUTABLE INSIDE ./build
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/build")

set(SOURCE_FILES
    data/array.h
    data/data.h
    data/data.cpp
    data/graph.h
    data/instance.h
    data/mows.h
    data/network.h
    data/prices.h
    data/segments.h
    data/speeds.h
    data/time_windows.h
    data/trains.h
    params/params.h
    params/params.cpp
    main.cpp)

add_executable(ras ${SOURCE_FILES})
target_link_libraries(ras ${CPLEX_LIBRARIES})