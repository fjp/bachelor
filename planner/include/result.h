//
// Created by Franz Pucher on 6/23/19.
//

#ifndef BACHELORTEST_RESULT_H
#define BACHELORTEST_RESULT_H

///\brief Contains result information from the planner
struct tResult {
    ///\brief Total time it will take the rover for the found path
    double fTravellingTime;

    ///\brief True if the goal was found. False otherwise
    bool bFoundGoal;

    ///\brief True if the used heuristic is consistent h(x) <= d(x,y) + h(y) for every node x and its successor y.
    ///\details Will be set to false if h(x) > d(x,y) + h(y)
    bool bConsistentHeuristic;

    ///\brief Stores the number of expanded nodes
    int nNodesExpanded;

    ///\brief Numer of total iterations
    int nIterations;
};

#endif //BACHELORTEST_RESULT_H
