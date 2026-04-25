#include "opencv2/imgproc.hpp"
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/SimpleSetup.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;

struct MapData {
    int width, height;
    float resolution;
    std::vector<float> origin; // [x, y, z]
    std::vector<int8_t> data;  // 0=free, 255=occupied, 205=unknown
};

bool isStateValid(const ob::State *state, const MapData& map) {
    const auto *pos = state->as<ob::RealVectorStateSpace::StateType>();
    
    // Convert world (meters) to grid (pixels)
    int u = (pos->values[0] - map.origin[0]) / map.resolution;
    int v = (pos->values[1] - map.origin[1]) / map.resolution;

    // Bounds check
    if (u < 0 || u >= map.width || v < 0 || v >= map.height) return false;

    // Check if cell is occupied (100) or unknown (-1)
    return map.data[v * map.width + u] == 0; 
}

// TODO: correct this to use the array of probe positions and the start pos
void plan(MapData map) {

    auto space(std::make_shared<ob::RealVectorStateSpace>(2));

    // Set bounds based on your map size and resolution
    ob::RealVectorBounds bounds(2);
    bounds.setLow(0, map.origin[0]); 
    bounds.setHigh(0, map.origin[0] + (map.width * map.resolution));
    // ... repeat for Y ...
    space->setBounds(bounds);

    auto si = std::make_shared<ob::SpaceInformation>(space);
    
    // Set the validity checker using a lambda to pass the map data
    si->setStateValidityChecker([map](const ob::State *state) {
        return isStateValid(state, map);
    });

    og::SimpleSetup ss(si);
    
    // Define Start and Goal
    ob::ScopedState<> start(space);
    start[0] = 1.0; start[1] = 1.0; // Example world coords

    ob::ScopedState<> goal(space);
    goal[0] = 5.0; goal[1] = 5.0;

    ss.setStartAndGoalStates(start, goal);

    // Use RRT*
    auto planner = std::make_shared<og::RRTstar>(si);
    ss.setPlanner(planner);

    // Solve
    ob::PlannerStatus solved = ss.solve(1.0); // 1 second timeout

    if (solved) {
        ss.getSolutionPath().print(std::cout);
    }
}

int main() {
    // 1a. import map yaml + pgm
    // 1b. import probe positions (TODO: make exporter also export those)
    // (could be as part of the yaml)
    // 2. pre-process data with openCV (inflate obstacles with dilation to ensure 1m rover clearance)
    // 3. cv::Mat to MapData struct
    // 4. path = plan(obs_map, probes)
    // 5. draw route on obstacle map
    // 6. convert route with drawing to png
    // 7. compile report (latex, using static template)

    return 0;
}