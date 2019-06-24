
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
    [std::enable_shared_from_this](http://en.cppreference.com/w/cpp/memory/enable_shared_from_this) 
    for the cyclic dependency between rover interface and planner interface.


<img src="doc/images/solution_rover_bachelor_wedding.bmp" alt="Solution Rover Bachelor Wedding" width="200"/>