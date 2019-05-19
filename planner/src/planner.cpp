//
// Created by Franz Pucher on 2019-05-18.
//

#include "planner.h"

#include <iostream>

namespace planner {



    void cPlanner::Plan(cGraph &i_oMap, cRoverInterface &i_oRover)
    {


        // Create a closed 2 array filled with 0s and first element 1
        std::vector<std::vector<int> > closed(i_oMap.Height(), std::vector<int>(i_oMap.Width()));
        closed[i_oRover.m_afStart[0]][i_oRover.m_afStart[1]] = 1;

        // Create expand array filled with -1
        std::vector<std::vector<int> > expand(i_oMap.Height(), std::vector<int>(i_oMap.Width(), -1));

        // Create action array filled with -1
        std::vector<std::vector<int> > action(i_oMap.Height(), std::vector<int>(i_oMap.Width(), -1));

        // Defined the quadruplet values
        int x = i_oRover.m_afStart[0];
        int y = i_oRover.m_afStart[1];
        double g = 0;
        double f = g + i_oMap.m_mnHeuristic[x][y];

        // Store the expansions
        std::vector<std::vector<double> > open; // TODO fix mix between int and double
        open.push_back({ f, g, (double)x, (double)y });

        // Flags and Counts
        bool found = false;
        bool resign = false;
        int count = 0;

        int x2;
        int y2;

        // While I am still searching for the goal and the problem is solvable
        while (!found && !resign) {
            // Resign if no values in the open list and you can't expand anymore
            if (open.size() == 0) {
                resign = true;
                std::cout << "Failed to reach a goal" << std::endl;
            }
                // Keep expanding
            else {
                // Remove quadruplets from the open list
                sort(open.begin(), open.end());
                reverse(open.begin(), open.end());
                std::vector<double> next;
                // Stored the poped value into next
                next = open.back();
                open.pop_back();

                x = (int)next[2];
                y = (int)next[3];
                g = next[1];

                // Fill the expand vectors with count
                expand[x][y] = count;
                count += 1;


                // Check if we reached the goal:
                if (x == i_oRover.m_afGoal[0] && y == i_oRover.m_afGoal[1]) {
                    found = true;
                    //cout << "[" << g << ", " << x << ", " << y << "]" << endl;
                }

                    //else expand new elements
                else {
                    for (int i = 0; i < i_oRover.m_mnMovements.size(); i++) {
                        x2 = x + i_oRover.m_mnMovements[i][0];
                        y2 = y + i_oRover.m_mnMovements[i][1];
                        if (x2 >= 0 && x2 < i_oMap.Width() && y2 >= 0 && y2 < i_oMap.Height()) {
                            bool bWater = i_oMap.Water(x2, y2);
                            if (closed[x2][y2] == 0 and not bWater) {
                                double g2 = g + i_oRover.m_fCostStraight;
                                f = g2 + i_oMap.m_mnHeuristic[x2][y2];
                                open.push_back({ f, g2, (double)x2, (double)y2 });
                                closed[x2][y2] = 1;
                                action[x2][y2] = i;
                            }
                        }
                    }
                }
            }
        }

        // Print the expansion List
        //print2DVector(expand);

        // Find the path with robot orientation
        std::vector<std::vector<std::string> > policy(i_oMap.Height(), std::vector<std::string>(i_oMap.Width(), "-"));

        // Going backward
        x = i_oRover.m_afGoal[0];
        y = i_oRover.m_afGoal[1];
        policy[x][y] = '*';

        while (x != i_oRover.m_afStart[0] or y != i_oRover.m_afStart[1]) {
            x2 = x - i_oRover.m_mnMovements[action[x][y]][0];
            y2 = y - i_oRover.m_mnMovements[action[x][y]][1];
            // Store the  Path in a vector
            i_oRover.m_mnPath.push_back({ x2, y2 });
            policy[x2][y2] = i_oRover.m_astrMovementArrows[action[x][y]];
            x = x2;
            y = y2;
        }
    }

    //void cPlanner::AStar(tLocation& i_oStart, tLocation& i_oGoal,
    //                                                std::unordered_map<tLocation, tLocation>& i_oPredecessors,
    //                                                std::unordered_map<tLocation, double>& i_oPathCost) {
    //
    //    /// Initialize the frontier using the initial state of the problem
    //    tFrontier oFrontier;
    //    oFrontier.Put(i_oStart, 0.0);
    //
    //    i_oPredecessors[i_oStart] = i_oStart;
    //
    //    i_oPathCost[i_oStart] = 0.0;
    //
    //    while (!oFrontier.Empty()) {
    //        tLocation oCurrent = oFrontier.Get();
    //
    //        if (oCurrent == i_oGoal) {
    //            break;
    //        }
    //
    //        for (tLocation oNext : m_oGraph.Neighbors(oCurrent)) {
    //            double fNewCost = i_oPredecessors[oCurrent] + m_oGraph.Cost(oCurrent, oNext);
    //            if (i_oPathCost.find(oNext) == i_oPathCost.end()
    //                || fNewCost < i_oPathCost[oNext]) {
    //                i_oPathCost[oNext] = fNewCost;
    //                double priority = fNewCost + heuristic(oNext, i_oGoal);
    //                oFrontier.Put(oNext, priority);
    //                i_oPredecessors[oNext] = oCurrent;
    //            }
    //        }
    //    }
    //}



}