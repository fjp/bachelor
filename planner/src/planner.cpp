//
// Created by Franz Pucher on 2019-05-18.
//

#include "planner.h"

#include <iostream>
#include <algorithm>
#include <fstream>

#include "audi_rover.h"


namespace planner {



    cPlanner::cPlanner(cRoverInterface *i_oRover, cGraph &i_oMap) : cPlannerInterface(static_cast<cAudiRover*>(i_oRover), i_oMap) {

        GenerateHeuristic();
    }

    // Generate a Manhattan Heuristic Vector
    void cPlanner::GenerateHeuristic()
    {
        m_mnHeuristic = std::vector<std::vector<int> >(m_oMap.Height(), std::vector<int>(m_oMap.Width()));
        for (int i = 0; i < m_mnHeuristic.size(); i++) {
            for (int j = 0; j < m_mnHeuristic[0].size(); j++) {
                int nDeltaX = m_oRover->m_afGoal[0] - i;
                int nDeltaY = m_oRover->m_afGoal[1] - j;
                // Manhattan Distance
                int d = std::abs(nDeltaX) + std::abs(nDeltaY);
                // Euclidian Distance
                // double d = sqrt(xd * xd + yd * yd);
                // Chebyshev distance
                // int d = max(abs(xd), abs(yd));
                m_mnHeuristic[i][j] = d;
            }
        }
    }


    void cPlanner::Plan()
    {

        // Defined the quadruplet values
        int x = m_oRover->m_afStart[0];
        int y = m_oRover->m_afStart[1];
        double g = 0;
        double f = g + m_mnHeuristic[x][y]; //TODO use AStar insetead of Dijkstra

        // Create a closed 2 array filled with 0s and first element 1
        std::vector<std::vector<int> > closed(m_oMap.Height(), std::vector<int>(m_oMap.Width()));

        closed[x][y] = 1;

        // Create expand array filled with -1
        std::vector<std::vector<int> > expand(m_oMap.Height(), std::vector<int>(m_oMap.Width(), -1));

        // Create action array filled with -1
        std::vector<std::vector<int> > action(m_oMap.Height(), std::vector<int>(m_oMap.Width(), -1));



        // Store the expansions
        std::vector<std::vector<double> > open; // TODO fix mix between int and double
        open.push_back({ f, g, (double)x, (double)y });

        // Flags and Counts
        bool found = false;
        bool resign = false;
        int count = 0;

        int x2;
        int y2;

        int nStepSize = 6;

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
                //if (x == m_oRover->m_afGoal[0] && y == m_oRover->m_afGoal[1]) {
                if (abs(x - m_oRover->m_afGoal[0]) < (nStepSize+1) && abs(y - m_oRover->m_afGoal[1]) < (nStepSize+1)) {
                    found = true;
                    //cout << "[" << g << ", " << x << ", " << y << "]" << endl;
                }

                    //else expand new elements
                else {
                    for (int i = 0; i < m_oRover->m_mnMovements.size(); i++) {
                        x2 = x + m_oRover->m_mnMovements[i][0] * nStepSize;
                        y2 = y + m_oRover->m_mnMovements[i][1] * nStepSize;
                        if (x2 >= 0 && x2 < m_oMap.Width() && y2 >= 0 && y2 < m_oMap.Height()) {
                            bool bWater = m_oMap.Water(x, y, x2, y2);
                            if (closed[x2][y2] == 0 and not bWater) {
                                double g2 = g + m_oRover->m_fCostStraight;
                                f = g2 + m_mnHeuristic[x2][y2];
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
        std::vector<std::vector<std::string> > policy(m_oMap.Height(), std::vector<std::string>(m_oMap.Width(), "-"));


        // Going backward
        x = m_oRover->m_afGoal[0]-4;
        y = m_oRover->m_afGoal[1]-5;
        policy[x][y] = '*';

        while (x != m_oRover->m_afStart[0] or y != m_oRover->m_afStart[1]) {
            int nAction = action[x][y];
            x2 = x - m_oRover->m_mnMovements[nAction][0] * nStepSize;
            y2 = y - m_oRover->m_mnMovements[nAction][1] * nStepSize;
            // Store the  Path in a vector
            m_oRover->m_mnPath.push_back({ x2, y2 });
            policy[x2][y2] = m_oRover->m_astrMovementArrows[action[x][y]];
            x = x2;
            y = y2;


            /// Store path in overrides
            m_oMap.SetOverrides(x2, y2, 0x01);
        }

        std::ofstream myfile;
        myfile.open ("result.txt");
        // Print the robot path
        //cout << endl;
        for (int i = 0; i < policy.size(); ++i) {
            for (int j = 0; j < policy[0].size(); ++j) {
                myfile << policy[i][j] << ' ';
            }
            myfile << std::endl;
        }


        myfile.close();

        //return policy; // TODO return plan
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