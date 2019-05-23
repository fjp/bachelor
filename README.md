# AID Coding Challenge Solution

This is my solution for the AID Bachelor Coding Challenge. 
For the problem description refer to the AID [Coding Challenge.pdf](Coding Challenge.pdf).

# Algorithm Choice

To find the fastest route from a start to a goal location, I considered Gradient Fields, 
Dynamic Programming and Graph Search. I decided for A* graph search algorithm, which uses
Dynamic Programming to find the shortest path. Compared to Dijkstra it uses a heuristic 
\f$h(n)\f$ which provides an estimate of the minimum cost from any node n to the goal. Because the robot can move in eight directions (North, East, South, West, North East, South West, North West, South East) I use an octile distance heuristic, as explained here. This heuristic speeds up the time to find a solution because the algorithm starts exploring new nodes in the direction towards the goal.

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
