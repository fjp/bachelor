//
// Created by Franz Pucher on 2019-05-18.
//

#include "planner_wiki.h"
#include "simple_node.h"

#include <iostream>
#include <algorithm>
#include <map>

#include <vector>
#include <cmath>

#include "audi_rover.h"

#include "visualizer.h"
#include "constants.h"

#include <map>

namespace planner {



    cPlannerWiki::cPlannerWiki(std::shared_ptr<cRoverInterface<8>> i_poRover, std::shared_ptr<cGraph> i_oMap)
            : cPlanner(i_poRover, i_oMap)
            {

        std::cout << "Constructing cPlannerWiki" << std::endl;
    }



    tResult cPlannerWiki::Plan() {

        return AStar();
    }

    tResult cPlannerWiki::AStar()
    {
        /// The set of nodes already evaluated. Implemented as 2d array filled with 0s and start element set to 1.
        std::vector<std::vector<int> > closed(m_oMap->Height(), std::vector<int>(m_oMap->Width()));
        closed[m_poRover->Start().nX][m_poRover->Start().nY] = 1;

        /// The set of currently discovered nodes that are not evaluated yet.
        /// Initially, only the start node is known.
        PriorityQueue<tSimpleNode, double> oOpenPrioQ;

        /// Defined the simplified start node
        int nX = m_poRover->Start().nX;
        int nY = m_poRover->Start().nY;
        double g = 0;
        double h = Heuristic(m_poRover->Start());
        double f = g + h;
        tSimpleNode sNode{nY * m_oMap->Width() + nX, nX, nY, g, h, f};
        oOpenPrioQ.put(sNode, f);

        /// For each node, which action it can most efficiently be reached from.
        /// If a node can be reached from many nodes, action will eventually contain the
        /// most efficient previous step.
        std::vector<std::vector<int> > action(m_oMap->Height(), std::vector<int>(m_oMap->Width(), -1));


        /// For each node, the cost of getting from the start node to that node.
        std::vector<std::vector<double> > gScore(m_oMap->Height(), std::vector<double>(m_oMap->Width(), std::numeric_limits<double>::max()));

        /// The cost of going from start to start is zero.
        gScore[nX][nY] = 0.f;


        /// Initialize result struct
        m_sResult.bFoundGoal = false;
        bool bResign = false;
        m_sResult.nIterations = 0;

        int nXNext;
        int nYNext;

        /// While the goal is not found the problem is solvable
        while (!m_sResult.bFoundGoal && !bResign) {

            m_sResult.nIterations++;
            if (m_sResult.nIterations % 100000 == 0)
            {
                std::cout << "Iteration " << m_sResult.nIterations
                          << ": Best node location (" << oOpenPrioQ.pop().nX << "," << oOpenPrioQ.pop().nY
                          << "), \n\t Evaluation function f(n): " << oOpenPrioQ.pop().f
                          //<< ", step cost c(n): " << m_oFrontier.pop()->g - m_oFrontier.pop()->psParent->g
                          << ", path cost g(n): " << oOpenPrioQ.pop().g
                          << ", heuristic h(n): " << oOpenPrioQ.pop().h
                          << std::endl;
                //Plot();
            }

            /// Resign if no values in the open list and you can't expand anymore
            if (oOpenPrioQ.empty()) {
                bResign = true;
                std::cout << "Failed to reach goal" << std::endl;
            }
            else {
                /// Remove the node from the open priority queue having the lowest fScore value
                tSimpleNode sCurrent;
                sCurrent = oOpenPrioQ.get();

                nX = sCurrent.nX;
                nY = sCurrent.nY;
                g = sCurrent.g;
                double fparent = sCurrent.f;


                /// Check if the goal is reached
                if (nX == m_poRover->Goal().nX && nY == m_poRover->Goal().nY)
                {
                    m_sResult.bFoundGoal = true;
                }
                /// Otherwise explore new locations
                else {
                    /// Add the current node to the set of nodes already evaluated
                    closed[nX][nY] = 1;

                    /// Perform each possible rover action on the current node
                    for (int i = 0; i < m_poRover->m_asActions.size(); i++) {
                        auto sAction = m_poRover->m_asActions[i];
                        nXNext = nX + sAction.nX;
                        nYNext = nY + sAction.nY;
                        tLocation sNextLocation{nXNext, nYNext};
                        tSimpleNode sNext{nYNext * m_oMap->Width() + nXNext, nXNext, nYNext};


                        /// Check if the location of the next node lies within the map and is not on water.
                        /// Ignore the neighbors which are already evaluated (closed[nXNext][nYNext] == 0).
                        if (WithinMap(sNextLocation)) {
                            if (closed[nXNext][nYNext] == 0 && !m_oMap->Water(nXNext, nYNext)) {

                                tLocation sCurrent{nX, nY};
                                double fHeightCost = HeightCost(sCurrent, sNextLocation, sAction);
                                double g2 = g + sAction.fCost + fHeightCost;

                                /// The distance from start to a neighbor
                                double tentative_gScore = gScore[nX][nY] + sAction.fCost + fHeightCost;

                                if (tentative_gScore >= gScore[nXNext][nYNext]) {
                                    //std::cout << "x2, y2" << std::endl;
                                    continue;
                                }

                                gScore[nXNext][nYNext] = tentative_gScore;

                                h = Heuristic(sNextLocation);
                                f = g2 + h;

                                sNext.g = g2;
                                sNext.h = h;
                                sNext.f = f;

                                oOpenPrioQ.put(sNext, f);
                                action[nXNext][nYNext] = i;

                                /// Mark visited nodes
                                m_oMap->SetOverrides(nXNext, nYNext, 0x02);
                                m_sResult.nNodesExpanded++;

                                /// Check that heuristic never overestimates the true distance:
                                /// Priority of a new node should never be lower than the priority of its parent.
                                if (f < fparent - 1e-8f)
                                {
                                    std::cout << "Heuristic overestimates true distance" << std::endl;
                                }
                            }
                        }
                    }
                }
            }
        }


        ReconstructPath(g, action);
        PrintTravelResult();

        return m_sResult;
    }

    template<typename TCostSoFar, typename TCameFrom>
    void cPlannerWiki::ReconstructPath(TCostSoFar &&i_cost_so_far, TCameFrom &&i_came_from)
    {
        /// Set the cost (time) it takes to get to the goal
        m_sResult.fTravellingTime = i_cost_so_far;

        /// Reconstruct the path by going backward
        int nX = m_poRover->Goal().nX;
        int nY = m_poRover->Goal().nY;

        /// Update cumulative elevation
        m_sResult.nCumulativeElevation += m_oMap->Elevation(nX, nY);
        /// Store path in overrides
        m_oMap->SetOverrides(nX, nY, 0x01);

        /// Check if the current node is the start node
        int nXNext, nYNext;
        while (nX != m_poRover->Start().nX || nY != m_poRover->Start().nY) {
            /// Move towards the start
            nXNext = nX - m_poRover->m_asActions[i_came_from[nX][nY]].nX;
            nYNext = nY - m_poRover->m_asActions[i_came_from[nX][nY]].nY;

            /// Update cumulative elevation
            m_sResult.nCumulativeElevation += m_oMap->Elevation(nXNext, nYNext);
            /// Store path in overrides
            m_oMap->SetOverrides(nXNext, nYNext, 0x01);

            nX = nXNext;
            nY = nYNext;
        }

    }

}
