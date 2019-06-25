//
// Created by Franz Pucher on 2019-05-18.
//



#include "planner_rbg.h"
#include "simple_node.h"

#include <iostream>
#include <algorithm>
#include <map>

#include <vector>
#include <fstream>
#include <cmath>

#include "audi_rover.h"


#include "visualizer.h"
#include "constants.h"

#include <map>

namespace planner {



    cPlannerRBG::cPlannerRBG(std::shared_ptr<cRoverInterface<8>> i_poRover, std::shared_ptr<cGraph> i_oMap)
            : cPlanner(i_poRover, i_oMap)
            {

        //std::cout << "Constructing cPlannerRBG" << std::endl;
    }



    tResult cPlannerRBG::Plan() {

        return AStar();
    }

    tResult cPlannerRBG::AStar()
    {
        std::cout << "Planning optimal path with A* RBG" << std::endl;

        std::map<tSimpleLocation, tSimpleLocation> came_from;
        std::map<tSimpleLocation, double> cost_so_far;

        /// Start
        int nX = m_poRover->Start().nX;
        int nY = m_poRover->Start().nY;
        int nId = nY * m_poMap->Width() + nX;
        tSimpleLocation sStart{nId, nX, nY};

        PriorityQueue<tSimpleLocation, double> oFrontier;
        oFrontier.put(sStart, 0.f);

        came_from[sStart] = sStart;
        cost_so_far[sStart] = 0.f;


        tSimpleLocation sCurrent;

        /// Initialize result struct
        m_sResult.bFoundGoal = false;
        m_sResult.nIterations = 0;

        // While I am still searching for the goal and the problem is solvable
        while (!oFrontier.empty()) {

            m_sResult.nIterations++;
            if (m_sResult.nIterations % 100000 == 0)
            {
                std::cout << "Iteration " << m_sResult.nIterations
                          //<< ": Best node location (" << m_oFrontier.pop()->sLocation.nX << "," << m_oFrontier.pop()->sLocation.nY
                          //<< "), \n\t Evaluation function f(n): " << m_oFrontier.pop()->f
                          //<< ", step cost c(n): " << m_oFrontier.pop()->g - m_oFrontier.pop()->psParent->g
                          //<< ", path cost g(n): " << m_oFrontier.pop()->g
                          //<< ", heuristic h(n): " << m_oFrontier.pop()->h
                          << std::endl;
                //Plot();
            }


            // Remove quadruplets from the open list
            sCurrent = oFrontier.get();

            /// Check if we reached the goal:
            if (sCurrent == m_poRover->Goal()) {
                m_sResult.bFoundGoal = true;
                break;
            }

            for (auto sAction : m_poRover->m_asActions) {

                int nXNext = sCurrent.nX + sAction.nX;
                int nYNext = sCurrent.nY + sAction.nY;
                int nIdNext = nYNext * m_poMap->Width() + nXNext;
                tSimpleLocation sNext{nIdNext, nXNext, nYNext};

                tLocation sLocation{nXNext, nYNext};
                if (WithinMap(sLocation) && !m_poMap->Water(nXNext, nYNext)) {

                    double fHeightCost = HeightCost(sCurrent, sNext, sAction);

                    double new_cost = cost_so_far[sCurrent] + sAction.fCost + fHeightCost;
                    if (cost_so_far.find(sNext) == cost_so_far.end() || new_cost < cost_so_far[sNext])
                    {
                        cost_so_far[sNext] = new_cost;
                        double h = Heuristic(sLocation);
                        double priority = new_cost + h;
                        oFrontier.put(sNext, priority);
                        came_from[sNext] = sCurrent;

                        /// Mark visited nodes
                        m_poMap->SetOverrides(sNext.nX, sNext.nY, 0x02);
                        m_sResult.nNodesExpanded++;

                        /// Check that heuristic never overestimates the true distance:
                        /// Priority of a new node should never be lower than the priority of its parent.
                        //if (f < fparent) {
                        //    std::cout << "Heuristic overestimates true distance" << std::endl;
                        //}
                    }
                }
            }
        }

        if (m_sResult.bFoundGoal) {
            /// Move from the current node back to the start node
            ReconstructPath(cost_so_far, came_from);
        }

        PrintTravelResult();

        return m_sResult;
    }

    template<typename TCostSoFar, typename TCameFrom>
    void cPlannerRBG::ReconstructPath(TCostSoFar&& i_cost_so_far, TCameFrom&& i_came_from)
    {
        /// Reconstruct the path by going backward from the goal location
        int nX = m_poRover->Goal().nX;
        int nY = m_poRover->Goal().nY;
        int nId = nY * m_poMap->Width() + nX;
        tSimpleLocation sGoal{nId, nX, nY};
        tSimpleLocation sCurrent = sGoal;

        /// Set the cost (time) it takes to get to the goal
        m_sResult.fTravellingTime = i_cost_so_far[sGoal];

        /// Update cumulative elevation
        m_sResult.nCumulativeElevation += m_poMap->Elevation(sCurrent.nX, sCurrent.nY);
        /// Store path in overrides
        m_poMap->SetOverrides(sCurrent.nX, sCurrent.nY, 0x01);

        /// Check if the current node is the start node
        while (sCurrent != m_poRover->Start()) {
            /// Move towards the start
            sCurrent = i_came_from[sCurrent];

            /// Update cumulative elevation
            m_sResult.nCumulativeElevation += m_poMap->Elevation(sCurrent.nX, sCurrent.nY);
            /// Store path in overrides
            m_poMap->SetOverrides(sCurrent.nX, sCurrent.nY, 0x01);
        }
    }

}
