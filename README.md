# AID Coding Challenge Solution

This is my updated solution for the AID Bachelor Coding Challenge. 
The readme of the previous version can be found [here](README_v0.md).
For the problem description refer to the [AID Coding Challenge.pdf](AID_Coding_Challenge.pdf). 
The complete doxygen documentation can be found in the doc folder, see [index.html](doc/html/index.html)


## Changelog

- Fixes diagonal cost value: uses sqrt(2) for the diagonal cost instead of 1.4f.
- Fixes memory leaks caused by dangling pointers, mainly in the Child() method of planner.cpp.
    - Now using smart pointers for the nodes
    - Uses [shared_ptr](https://en.cppreference.com/w/cpp/memory/shared_ptr), [std::weak_ptr](http://en.cppreference.com/w/cpp/memory/weak_ptr) and 
    [std::enable_shared_from_this](http://en.cppreference.com/w/cpp/memory/enable_shared_from_this) and [shared_from_this()](https://en.cppreference.com/w/cpp/memory/enable_shared_from_this/shared_from_this)
    for the cyclic dependency between rover interface and planner interface.
    - Use of smart pointers reduces memory consumption from approx 2 GB to 570 MB. This high memory is caused by creating
    child nodes while exploring new locations. Each node contains information that increase the memory used each time a new node is created.
- To verify the correctness of the current implementation this version adds two more implementations of A*,
one from [Wikipedia](https://en.wikipedia.org/wiki/A*_search_algorithm#Pseudocode) and
the other from [Red Blob Games](https://www.redblobgames.com/pathfinding/a-star/implementation.html#cpp-astar).
- The new A* implementations use less memory 
- Simplifies step cost model of the rover: instead of the physical model the rover now uses height costs that are percentage values of the step cost.
The percentage step cost (height cost) is added to the current step cost if the rover moves uphill and subtracted if it moves downhill.
The height cost calculation takes place in planner::cPlanner::HeightCost().
- Ensures consistent heuristic with the chosen step cost and height cost model. This is achieved by normalizing the 
calculated octile heuristic value in planner::cPlanner::Heuristic().
- In case of an inconsistent heuristic the boolean flag bConsistentHeuristic in the new member struct planner::tResult of cPlannerInterface is set to false.
- Adds gTests that use a simplified map, including water and elevation transitions. The result struct is used to check the expected results.
- Refactors visualizer
    - Encapsulates the visualizer::writeBMP() function including its lambda 
    - Adds option what to draw
        - path, 
        - locations such as start and goal passed as list of locations
        - NEW: show visited nodes
- C++11 features used
    - smart pointer: std::shared_ptr, std::weak_ptr, std::enable_shared_from_this, shared_from_this()
    - 


<img src="doc/images/solution_rover_bachelor_wedding.bmp" alt="Solution Rover Bachelor Wedding" width="200"/>
