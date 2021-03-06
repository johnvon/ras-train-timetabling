cmake_minimum_required(VERSION 2.8.4)
project(tsppddl)

# === CPLEX
if(NEED_CPLEX)
    # DEFAULT ROOT
    set(CPLEX_ROOT_DIR "/opt/ibm" CACHE FILEPATH "IBM Cplex install dir")

    # CUSTOM ROOT
    if(CMAKE_SYSTEM_NAME STREQUAL Linux)
         if(BUILDING_FOR_CLUSTER)
             set(CPLEX_ROOT_DIR "/opt/ibm/ILOG/CPLEX_Studio126" CACHE FILEPATH "IBM Cplex install dir")
         endif()
         if(BUILDING_FOR_DBG_MACHINE)
             set(CPLEX_ROOT_DIR "~/cplex" CACHE FILEPATH "IBM Cplex install dir")
         endif()
    endif()
    
    set(USE_CPLEX_FLAG "true")
    set(USE_CPLEX_SOURCE_FILES
      solver/solver.h
      solver/solver.cpp
      solver/sequential_solver.h
      solver/sequential_solver.cpp
    )
else()
    set(USE_CPLEX_FLAG "false")
    set(USE_CPLEX_SOURCE_FILES "")
endif()

# === BOOST
if(CMAKE_SYSTEM_NAME STREQUAL Linux)
    if(BUILDING_FOR_CLUSTER)
        set(BOOST_ROOT "~/boost-trunk/boost")
    endif()
    if(BUILDING_FOR_DEI207)
        set(BOOST_ROOT "/opt/boost-trunk/boost")
    endif()
elseif(CMAKE_SYSTEM_NAME STREQUAL Darwin)
    if(BUILDING_FOR_MB_ALBERTO)
      set(Boost_NO_SYSTEM_PATHS TRUE)
      set(BOOST_ROOT "/opt/boost")
    endif()
endif()

# === BOOST COMPILED
if(NEED_BOOST_COMPILED)
  set(BOOST_COMPILED_COMPONENTS filesystem iostreams system)
  set(USE_BOOST_COMPILED_SOURCE_FILES
    grapher/grapher.h
    grapher/grapher.cpp
    grapher/gnuplot-iostream.h
  )
  set(USE_GRAPHER_FLAG "true")
else()
  set(USE_BOOST_COMPILED_SOURCE_FILES "")
  set(USE_GRAPHER_FLAG "false")
endif()

# COMPILER: COMMON FLAGS
if(BUILDING_FOR_DBG_MACHINE)
    set(ADDITIONAL_FLAGS "")
    set(ADDITIONAL_RELEASE_FLAGS "")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -L/usr/lib/i386-linux-gnu")
else()
    set(ADDITIONAL_FLAGS "-m64")
    set(ADDITIONAL_RELEASE_FLAGS "-flto")
endif()
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -DIL_STD -DUSE_CPLEX=${USE_CPLEX_FLAG} -DUSE_GRAPHER=${USE_GRAPHER_FLAG} -Wall -Werror ${ADDITIONAL_FLAGS}")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -DDEBUG=true -ggdb")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -DNDEBUG -DDEBUG=false -O3 ${ADDITIONAL_RELEASE_FLAGS}")

# COMPILER: PLATFORM-DEPENDENT FLAGS
if(CMAKE_SYSTEM_NAME STREQUAL Darwin)
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -save-temps=obj")
endif()

# LOAD MODULES: cplex, boost
set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
if(NEED_CPLEX)
    find_package(Cplex)
endif()
if(NEED_BOOST_COMPILED)
  find_package(Boost COMPONENTS ${BOOST_COMPILED_COMPONENTS} REQUIRED)
else()
  find_package(Boost)
endif()

set(CMAKE_INCLUDE_SYSTEM_FLAG_CXX "-isystem ")

# INCLUDE DIRECTORIES
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
if(NEED_CPLEX)
    include_directories(SYSTEM ${CPLEX_INCLUDE_DIRS})
endif()
include_directories(SYSTEM ${Boost_INCLUDE_DIR})

# BUILD THE EXECUTABLE INSIDE ./build
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/build")

set(SOURCE_FILES
    data/array.h
    data/data.h
    data/data.cpp
    data/graph.h
    data/graph.cpp
    data/instance.h
    data/mows.h
    data/mows.cpp
    data/network.h
    data/network.cpp
    data/path.h
    data/path.cpp
    data/prices.h
    data/prices.cpp
    data/segments.h
    data/segments.cpp
    data/speeds.h
    data/speeds.cpp
    data/time_windows.h
    data/time_windows.cpp
    data/trains.h
    data/trains.cpp
    params/params.h
    params/params.cpp
    ${USE_BOOST_COMPILED_SOURCE_FILES}
    ${USE_CPLEX_SOURCE_FILES}
    main.cpp)

# EXECUTABLE FILE
add_executable(ras ${SOURCE_FILES})

# LINKED LIBRARIES
if(NEED_CPLEX)
    target_link_libraries(ras ${CPLEX_LIBRARIES})
endif()
if(NEED_BOOST_COMPILED)
  target_link_libraries(ras ${Boost_FILESYSTEM_LIBRARY} ${Boost_IOSTREAMS_LIBRARY} ${Boost_SYSTEM_LIBRARY})
endif()
