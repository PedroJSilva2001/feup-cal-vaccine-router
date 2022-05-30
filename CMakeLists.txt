cmake_minimum_required(VERSION 3.17)
project(FEUP_CAL_proj)

set(CMAKE_CXX_STANDARD 20)
set(GCC_COVERAGE_COMPILE_FLAGS "-Wall -static-libgcc -static-libstdc++ -static")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GCC_COVERAGE_COMPILE_FLAGS}" )
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")


include_directories(src/DataStructures)
include_directories(src/Algorithms)
include_directories(src/Algorithms/Connectivity)
include_directories(src/Algorithms/ShortestPath)
include_directories(src/Algorithms/TravellingSalesmenProblem)
include_directories(src/Algorithms/Reachability)
include_directories(src/Algorithms/VehicleRoutingProblem)
include_directories(src)
include_directories(src/Algorithms/Clustering)
include_directories(PortoMaps)
include_directories(src/vaccinerouter)
include_directories(lib)
include_directories(src/File)
include_directories(src/Algorithms/MaxFlowMinCost)
include_directories(src/Menu)
link_libraries(ws2_32 wsock32)

# Build GraphViewerCpp
set(GRAPHVIEWERCPP_PATH lib/GraphViewerCpp)
add_subdirectory(${GRAPHVIEWERCPP_PATH})
include_directories(${GRAPHVIEWERCPP_PATH}/include)

include_directories(include)
file(GLOB SRC_FILES CONFIGURE_DEPENDS
        "src/*.cpp"
        )

add_executable( ${PROJECT_NAME} main.cpp
        src/DataStructures/MutablePriorityQueue.h
        src/Algorithms/ShortestPath/NodeWrapper.h
        src/DataStructures/Graph.h
        src/Algorithms/Connectivity/Tarjan.cpp
        src/Algorithms/Connectivity/Tarjan.h
        src/Algorithms/Connectivity/Kosaraju.cpp
        src/Algorithms/Connectivity/Kosaraju.h
        src/Algorithms/ShortestPath/Dijkstra.cpp
        src/Algorithms/ShortestPath/Dijkstra.h
        src/Algorithms/ShortestPath/Astar.h
        src/Algorithms/ShortestPath/Astar.cpp
        src/Algorithms/ShortestPath/DijkstraBidirectional.cpp
        src/Algorithms/ShortestPath/DijkstraBidirectional.h
        src/Algorithms/TravellingSalesmenProblem/NearestNeighbour.cpp
        src/Algorithms/TravellingSalesmenProblem/NearestNeighbour.h
        src/Algorithms/ShortestPath/ShortestPathAlgorithm.cpp
        src/Algorithms/ShortestPath/ShortestPathAlgorithm.h
        src/Algorithms/Connectivity/SCCAlgorithm.cpp
        src/Algorithms/Connectivity/SCCAlgorithm.h
        src/Algorithms/TravellingSalesmenProblem/TSPAlgorithm.cpp
        src/Algorithms/TravellingSalesmenProblem/TSPAlgorithm.h
        src/Algorithms/Reachability/DepthFirstSearch.cpp
        src/Algorithms/Reachability/DepthFirstSearch.h
        src/vaccinerouter/VaccineRouter.cpp
        src/vaccinerouter/VaccineRouter.h
        src/Algorithms/VehicleRoutingProblem/VRPAlgorithm.h
        src/Algorithms/VehicleRoutingProblem/VRPAlgorithm.cpp
        src/Algorithms/VehicleRoutingProblem/CapacitatedNearestNeighbour.cpp
        src/Algorithms/VehicleRoutingProblem/CapacitatedNearestNeighbour.h
        src/File/FileLoader.cpp
        src/File/FileLoader.h
        src/Algorithms/Clustering/Naive.cpp
        src/Algorithms/Clustering/Naive.h
        src/Algorithms/Clustering/ClusteringAlgorithm.cpp
        src/Algorithms/Clustering/ClusteringAlgorithm.h
        src/Algorithms/Clustering/Kmeans.cpp
        src/Algorithms/Clustering/Kmeans.h
        src/Algorithms/Clustering/Point.cpp
        src/Algorithms/Clustering/Point.h
        src/vaccinerouter/CostFunction.h src/Algorithms/VehicleRoutingProblem/BuildVRP.cpp
        src/Algorithms/VehicleRoutingProblem/BuildVRP.h
        src/Algorithms/VehicleRoutingProblem/ClarkeWright.cpp
        src/Algorithms/VehicleRoutingProblem/ClarkeWright.h
        src/Algorithms/VehicleRoutingProblem/Route.cpp
        src/Algorithms/VehicleRoutingProblem/Route.h
        src/Algorithms/TravellingSalesmenProblem/BellmanHeldKarp.cpp
        src/Algorithms/TravellingSalesmenProblem/BellmanHeldKarp.h
        src/vaccinerouter/TimeAstarHeuristic.h
        src/Algorithms/MaxFlowMinCost/FlowGraph.h
        src/Menu/Menu.cpp
        src/Menu/Menu.h src/DataStructures/Graph.cpp)


target_link_libraries(${PROJECT_NAME} graphviewer)
