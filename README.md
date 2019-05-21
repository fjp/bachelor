# AID Coding Challenge Solution

This is my solution for the AID Bachelor Coding Challenge. 
For the problem description refer to the AID [Coding Challenge.pdf](Coding Challenge.pdf).

# Algorithm Choice

An algorithm for finding a path between a start and goal, lead me to choose between Gradient Fields, Dynamic Programming and Graph Search. I decided for A* graph search algorithm because of its heuristic, compared with Dijkstra. Because the robot can move in eight directions (North, East, South, West, North East, South West, North West, South East) I use an octile distance heuristic, as explained here. This heuristic speeds up the time to find a solution because the algorithm starts exploring new nodes in the direction towards the goal.

## Decisions against otehr algorithms

Looking at the map, Gradient Files do not seem to be a good choice because of the non convex constraints that are imposed by the fjords. 

## Cost Function

Consists of three terms

- step cost c(n)
- heuristic h(n)
- cost so far g(n)

## Step Cost Model


## Algorithm Steps

## A* Pseudocode



# Time and Space Complexity


# Software Organization and Architecture


To demonstrate that it has advantages to program against an interface instead of an implementation, I used the following interfaces.




# Coding Style

For the variable naming conventions, I tried to use the [MISRA C](https://en.wikipedia.org/wiki/MISRA_C) standard. 
