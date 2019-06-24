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

        std::cout << "Constructing cPlannerRBG" << std::endl;
    }

   

    double cPlannerRBG::Plan() {

        return AStar();
    }

    double cPlannerRBG::AStar()
    {
        std::map<tSimpleLocation, tSimpleLocation> came_from;
        std::map<tSimpleLocation, double> cost_so_far;

        /// Start
        int nX = m_poRover->Start().nX;
        int nY = m_poRover->Start().nY;
        int nId = nY * m_oMap->Width() + nX;
        tSimpleLocation sStart{nId, nX, nY};

        PriorityQueue<tSimpleLocation, double> oFrontier;
        oFrontier.put(sStart, 0.f);

        came_from[sStart] = sStart;
        cost_so_far[sStart] = 0.f;


        tSimpleLocation sCurrent;

        int nIteration = 0;

        // While I am still searching for the goal and the problem is solvable
        while (!oFrontier.empty()) {

            nIteration++;
            if (nIteration % 100000 == 0)
            {
                std::cout << "Iteration " << nIteration
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
                break;
            }

            for (auto sAction : m_poRover->m_asActions) {

                int nXNext = sCurrent.nX + sAction.nX;
                int nYNext = sCurrent.nY + sAction.nY;
                int nIdNext = nYNext * m_oMap->Width() + nXNext;
                tSimpleLocation sNext{nIdNext, nXNext, nYNext};

                tLocation sLocation{nXNext, nYNext};
                if (WithinMap(sLocation) && !m_oMap->Water(nXNext, nYNext)) {

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
                        m_oMap->SetOverrides(sNext.nX, sNext.nY, 0x02);

                        /// Check that heuristic never overestimates the true distance:
                        /// Priority of a new node should never be lower than the priority of its parent.
                        //if (f < fparent) {
                        //    std::cout << "Heuristic overestimates true distance" << std::endl;
                        //}
                    }
                }
            }
        }

        /// Goal
        nX = m_poRover->Goal().nX;
        nY = m_poRover->Goal().nY;
        nId = nY * m_oMap->Width() + nX;
        tSimpleLocation sGoal{nId, nX, nY};
        sCurrent = sGoal;

        int nElevation = 0;
        while (sCurrent.nX != sStart.nX || sCurrent.nY != sStart.nY) {
            m_oMap->SetOverrides(sCurrent.nX, sCurrent.nY, 0x01);
            sCurrent = came_from[sCurrent];

            nElevation += m_oMap->Elevation(sCurrent.nX, sCurrent.nY);
        }

        std::cout << "Total Elevation: " << nElevation << std::endl;

        double fIslandSeconds = cost_so_far[sGoal];
        std::cout << "Travelling will take " << fIslandSeconds << " island seconds ("
                  << fIslandSeconds/60.f << " island minutes or " << fIslandSeconds/60.f/60.f << " island hours) on the fastest path. " << std::endl;

        return fIslandSeconds;
    }

    void cPlannerRBG::TraversePath(std::shared_ptr<tNode> i_psNode) const
    {
        int nElevation = 0;
        int nX, nY;
        /// Check if the current node is the start node, which has no parent and is therefore set to NULL
        while (nullptr != i_psNode->psParent) {
            nX = i_psNode->sLocation.nX;
            nY = i_psNode->sLocation.nY;

            /// Store path in overrides
            m_oMap->SetOverrides(nX, nY, 0x01);

            nElevation += m_oMap->Elevation(nX, nY);

            /// Move towards the start
            i_psNode = i_psNode->psParent;
        }

        std::cout << "Total Elevation: " << nElevation << std::endl;

    }

}
