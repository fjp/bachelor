# AID Coding Challenge Solution

This is my solution for the AID Bachelor Coding Challenge. 
For the problem description refer to the AID [Coding Challenge.pdf](../../AID Coding Challenge.pdf). The complete doxygen documentation
can be found in the doc folder.

# Result

Travelling from the Rover (ROVER_X, ROVER_Y) to the Bachelor (BACHELOR_X, BACHELOR_Y) will take
2094.51 island seconds (34.9085 island minutes or 0.581809 island hours) on the shortest path.



Travelling from the Bachelor (BACHELOR_X, BACHELOR_Y) to the Wedding (WEDDING_X, WEDDING_Y) will
take 1283.17 island seconds (21.3862 island minutes or 0.356436 island hours) on the shortes path.

<img src="solution_rover_bachelor_wedding1.jpg" alt="Solution Rover Bachelor Wedding" width="500"/>


# Algorithm Choice

To find the fastest route from a start to a goal location, I considered Gradient Fields, 
Dynamic Programming and Graph Search. I decided for A* graph search algorithm, which uses
Dynamic Programming to find the shortest path. Compared to Dijkstra it utilizes a heuristic 
\f$h(n)\f$ which provides an estimate of the minimum cost from any node n to the goal. 

Looking at the map, I deceided against Gradient because of the non convex constraints that are
imposed by the fjords. 

## Cost Function

The evaluation function \f$f(n) = g(n) + h(n)\f$ describes the total cost of a node. 
It consists of the path cost \f$g(n)\f$, which describes how long it takes the rover to get to node \f$n\f$ from its start location.

The path cost \f$g(n)\f$ is calculated using the parent node's path cost \f$g(parent)\f$ and the step cost \f$c(n)\f$,
which is the cost of getting from the parent to the current node. The step cost is described in the next 
section \ref step-cost.


Because the robot can move in eight directions (straight and diagonal) I use an octile distance heuristic \f$h(n)\f$, 
implemented in cPlanner::UpdateHeuristic(). To get a consistent heuristic, I scaled it using the maximum
gradient analyzing the elevation of the map and taking the slope into account. The consistency \f$ h(n) <= c(n,p) + h(p)\f$ is checked in 
planner::cPlanner::HeuristicCheck(). I exported the calculated heuristic values using planner::cPlanner::GenerateHeuristic()
in the google test TEST_F(cPlannerTest, heuristic) (see, test_system.cpp).


<img src="heuristic.jpg" alt="Heuristic function plotted in Matlab" width="500"/>

## Step Cost Model
\section step-cost

Another task was to model the speed of the rover when driving up or downhill. 
For this I implemented a simple kinematic approach with an inclined plane to model 
the elevation. 

\image html inclined.jpg

<img src="inclined.jpg" alt="Inclinde plane of a free body (Wikipedia)" width="500"/>

The first step is to calculate the descent or ascent using the height difference
between two locations, see planner::cPlanner::UpdateCost(). In this method
the pitch angle is calculated next, followed by computing the x component of the downhill-slope force using
the gravitational force \f$F_G\f$. 

\f{eqnarray*}{
F_H &= F_G \cdot sin\alpha \rightarrow a = g \cdot sin\alpha \\
F_{H,x} &= a
\f}

The rover’s speed is lower uphill. It’s part of the task to model in which way it becomes
slower.
 The rover’s speed gets higher when running downhill. It’s part of the task to model in
which way it becomes faster.

# Time and Space Complexity

I tried to avoid alocating two dimensional vectors of the image size. Instead I am using a std::priority_queue priority_queue.h 
(planner::cPlanner::m_poFrontier) and a std::map.
The priority queue is sorted by the score value of the evaluation function \f$f(n)\f$.


# Software Organization and Architecture
\section software

The advantage of using interfaces is to get different implementations with different behavior but keep the public
interface methods the same. I use two interfaces that reference each other, planner::cRoverInterface and planner::cPlannerInterface.
The planner::cAudiRover implements planner::cRoverInterface and acts as a factory, creating planner::cPlanner in 
planner::cRoverInterface::InitializePlanner(). To start planning the start and goal positions of the rover need to be set,
planner::cRoverInterface::SetStart() and planner::cRoverInterface::SetGoal() passing a location struct of type tLocation.
After the initialization, a call to the method planner::cAudiRover::Summon() invokes the planner::cPlannerInterface::Plan().
Possible parameters to planner::cAudiRover::Summon() are the step size planner::cRoverInterface::m_nStepSize and its velocity
planner::cRoverInterface::m_nVelocity (not tested \ref testing).



# Coding Style

For the variable naming conventions, I follow the [MISRA C](https://en.wikipedia.org/wiki/MISRA_C) standard. 


# Testing
\section testing

\bold Note The tests are not completely finished because of the limited time.

I added google gTest version 1.8.0 to the zip file and created a test fixture cPlannerTest in test_fixture.h.
It is used to initialize the test by loading the evaluation.data and overrides.data files. Furthermore, the
cAudiRover is initialized in the cPlannerTest::SetUp method. 


As explained in \ref software the default values for step size and velocity are set to one for both parameters. I tested with 
different step sizes to reach the goal faster, thereby verifying that the path and island seconds stay the same. 

To get intermediate paths from the planner::cPlanner::AStar() I used the provided visualizer::writeBMP() inside AStar().
This results in the following output:




# References

\li Artificial Intelligence A Modern Approach Third Edition - Stuart Russel, Peter Norvig
\li Head First Design Patterns - Eric Freeman, Elisabeth Robson
\li https://autonomous-driving.org/2018/08/15/so-you-want-to-be-a-self-driving-car-engineer/
\li https://www.linkedin.com/pulse/software-quality-sami-vaaraniemi/
\li https://en.wikipedia.org/wiki/A*_search_algorithm
\li https://en.wikipedia.org/wiki/Consistent_heuristic
\li https://en.wikipedia.org/wiki/Admissible_heuristic
\li https://www.redblobgames.com/pathfinding/a-star/introduction.html
