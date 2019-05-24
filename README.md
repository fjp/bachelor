# AID Coding Challenge Solution

This is my solution for the AID Bachelor Coding Challenge. 
For the problem description refer to the AID [Coding Challenge.pdf](Coding Challenge.pdf).

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
Because the robot can move in eight directions (straight and diagonal) I use an octile distance heuristic, 
implemented in cPlanner::UpdateHeuristic. The get consistent heuristic, I scaled it using the maximum
 gradient of the elevation. The consistency \f$ h(n) <= c(n,p) + h(p)\f$ is checked in 

Looking at the map, I deceided against Gradient because of the non convex constraints that are
imposed by the fjords. 

## Cost Function

Consists of the path cost \f$g(n)\f$, which describes how long it takes the rover to get to node
node \f$n\f$ from its start location.

- step cost c(n)
- heuristic h(n)


## Step Cost Model

Another task was to model the speed of the rover when driving up or downhill. 
For this I implemented a simple kinematic approach with an inclined plane to model 
the elevation. The first step is to calculate the descent or ascent using the height difference
between two locations, see planner::cPlanner::UpdateCost(). In this method
the pitch angle is calculated next, followed by taking the x component of the downhill-slope force
gravitational force \f$F_G\f$. The rover’s speed is lower uphill. It’s part of the task to model in which way it becomes
slower.
 The rover’s speed gets higher when running downhill. It’s part of the task to model in
which way it becomes faster.


## Algorithm Steps

## A* Pseudocode



# Time and Space Complexity


# Software Organization and Architecture


To demonstrate that it has advantages to program against an interface instead of an implementation, I used the following interfaces.




# Coding Style

For the variable naming conventions, I tried to use the [MISRA C](https://en.wikipedia.org/wiki/MISRA_C) standard. 
