//
// Created by Franz Pucher on 2019-05-18.
//

#include "planner.h"

#include <iostream>
#include <algorithm>
#include <map>

#include <vector>
#include <fstream>
#include <cmath>

#include "audi_rover.h"
#include "priority_queue.h"



namespace planner {



    cPlanner::cPlanner(cRoverInterface<8> *i_oRover, cGraph &i_oMap) : cPlannerInterface(static_cast<cAudiRover*>(i_oRover), i_oMap) {

        GenerateHeuristic();


        /// Calculate maximum elevation gradient of the map
        m_nMaxGradient = 0;
        for (int nY = 0; nY < m_oMap.Height(); ++nY)
        {
            for (int nX = 0; nX < m_oMap.Width(); ++nX)
            {
                int nGradX = GradX(nX, nY);
                int nGradY = GradY(nX, nY);
                uint8_t nCurrentMaxGradient = std::max(std::abs(nGradX), std::abs(nGradY));
                if (m_nMaxGradient < nCurrentMaxGradient)
                {
                    m_nMaxGradient = nCurrentMaxGradient;
                }
            }
        }

        std::cout << "Max gradient " << (int)m_nMaxGradient << std::endl;

    }


    const int32_t cPlanner::GradX(uint32_t i_nX, uint32_t i_nY) const {
        if(i_nX == m_oRover->StepSize()) {
            return m_oMap.Elevation(i_nX, i_nY);
        }
        return m_oMap.Elevation(i_nX, i_nY) - m_oMap.Elevation(i_nX - m_oRover->StepSize(), i_nY);
    }


    const int32_t cPlanner::GradY(uint32_t i_nX, uint32_t i_nY) const {
        if(i_nY < m_oRover->StepSize()) {
            return m_oMap.Elevation(i_nX, i_nY);
        }
        return m_oMap.Elevation(i_nX, i_nY) - m_oMap.Elevation(i_nX, i_nY - m_oRover->StepSize());
    }

    // Generate a Manhattan Heuristic Vector
    void cPlanner::GenerateHeuristic()
    {

        int nD1 = m_oRover->StepSize();
        int nD2 = sqrt(2*nD1);

        m_mnHeuristic = std::vector<std::vector<int> >(m_oMap.Height(), std::vector<int>(m_oMap.Width()));
        for (int i = 0; i < m_mnHeuristic.size(); i++) {
            for (int j = 0; j < m_mnHeuristic[0].size(); j++) {
                int nDeltaX = m_oRover->Goal().nX - i;
                int nDeltaY = m_oRover->Goal().nY - j;
                // Octile distance
                int nHeuristicValue = nD1 * (nDeltaX + nDeltaY) + (nD2 - 2*nD1) * std::min(nDeltaX, nDeltaY);
                // Manhattan Distance
                //int nHeuristicValue = std::abs(nDeltaX) + std::abs(nDeltaY);
                // Chebyshev distance
                // int nHeuristicValue = std::max(std::abs(nDeltaX), std::abs(nDeltaY));
                // Euclidian Distance
                // double fHeuristicValue = sqrt(nDeltaX * nDeltaX + nDeltaY * nDeltaY);
                m_mnHeuristic[i][j] = nHeuristicValue;
            }
        }

#ifdef DEBUG_FILES
        std::ofstream myfile;
        myfile.open ("heuristic.txt");
        // Print the robot path
        //cout << endl;
        for (int i = 0; i < m_mnHeuristic.size(); ++i) {
            for (int j = 0; j < m_mnHeuristic[0].size(); ++j) {
                myfile << m_mnHeuristic[i][j] << ' ';
            }
            myfile << std::endl;
        }
        myfile.close();
#endif

    }

    bool cPlanner::GoalTest(const tNode *i_sFirst, const tNode *i_sSecond) const {
        uint nDeltaX = std::abs(i_sFirst->sLocation.nX - i_sSecond->sLocation.nX);
        uint nDeltaY = std::abs(i_sFirst->sLocation.nY - i_sSecond->sLocation.nY);
        return nDeltaX <= m_oRover->StepSize() && nDeltaY <= m_oRover->StepSize();
    }

    tNode* cPlanner::Child(tNode *i_sParent, const tAction &i_sAction) const
    {
        tNode *sNext = new tNode(*i_sParent);
        sNext->psParent = i_sParent;
        sNext->sLocation.nX = i_sParent->sLocation.nX + i_sAction.nX * m_oRover->StepSize();
        sNext->sLocation.nY = i_sParent->sLocation.nY + i_sAction.nY * m_oRover->StepSize();
        sNext->sAction = i_sAction;

        sNext->g = i_sParent->g + i_sAction.fCost * m_oRover->StepSize(); /// TODO gradient cost
        
        /// Calculate hash of node using its location
        sNext->nId = NodeHash(sNext);

        return sNext;
    }


    bool cPlanner::WithinMap(const tLocation &i_sLocation) const {

        uint nX = i_sLocation.nX;
        uint nY = i_sLocation.nY;

        return nX >= 0 && nX < m_oMap.Width() && nY >= 0 && nY < m_oMap.Height();
    }


    uint32_t cPlanner::NodeHash(const tNode *i_sNode) const
    {
        return i_sNode->sLocation.nY * m_oMap.Width() + i_sNode->sLocation.nX;
    }


    bool cPlanner::Plan()
    {
        /// Define start node
        tNode *sStart = new tNode(m_oRover->Start());
        /// Create hash of the node using its position
        sStart->nId = NodeHash(sStart);


        PriorityQueue<tNode*, double> m_oFrontier;


        m_oFrontier.put(sStart, 0);

        /// Get the goal node
        tNode *sGoal = new tNode(m_oRover->Goal());

        /// Create current node
        tNode *sCurrent = new tNode();

        /// Serves as explored (closed) set and cost to reach a node
        std::map<tNode, double> oCost;

        /// Initialize start node with cost of zero
        oCost[*sStart] = 0.0;

        // Flags and Counts
        bool bFound = false;
        bool bResign = false;
        //int count = 0;

        tLocation sNextLocation;

        while (!bFound && !bResign) {

            /// Resign if the frontier is empty, which means there are no nodes to expand and the goal has not been found
            if (m_oFrontier.empty())
            {
                bResign = true;
                break;
            }

            sCurrent = m_oFrontier.get();

            if (GoalTest(sCurrent, sGoal))
            {
                bFound = true;
            }
            else
            {
                for (auto sAction : m_oRover->m_asActions) {
                    tNode *sNext = Child(sCurrent, sAction);

                    sNextLocation.nX = sNext->sLocation.nX; //sCurrent.sLocation.nX + sAction.nX * m_nStepSize;
                    sNextLocation.nY = sNext->sLocation.nY; //sCurrent.sLocation.nY + sAction.nY * m_nStepSize;

                    if (WithinMap(sNextLocation)) {
                        bool bWater = m_oMap.Water(sCurrent->sLocation.nX, sCurrent->sLocation.nY,
                                                   sNextLocation.nX, sNextLocation.nY);
                        if (!bWater) {
                            /// Calculate current gradient in step direction and normalize it
                            double fHeightCost =
                                    (m_oMap.Elevation(sNextLocation.nX, sNextLocation.nY) - m_oMap.Elevation(sCurrent->sLocation.nX, sCurrent->sLocation.nY)) / m_nMaxGradient;
                            //std::cout << fHeightCost << std::endl;
                            sNext->g = sNext->g + fHeightCost; // TODO height cost
                            //nIslandSeconds += g2; // TODO fix island seconds calculation; must be outside of this loop


                            if (oCost.find(*sNext) == oCost.end() || sNext->g < oCost[*sNext]) {
                                oCost[*sNext] = sNext->g;
                                const int nHeuristic = Heuristic(sNextLocation);
                                sNext->f = sNext->g + nHeuristic;
                                m_oFrontier.put(sNext, sNext->f);
                            }
                        }
                    }
                }
            }
        }



        /// Move from the current node back to the start node
        TraversePath(sCurrent);


        if (bResign)
        {
            std::cout << "Goal location not found." << std::endl;
            return false;
        }

        return true;
    }


    void cPlanner::TraversePath(tNode *i_psNode) const
    {
        uint32_t x, y;
        /// Check if the current node is the start node, which has no parent and is therefore set to NULL
        while (nullptr != i_psNode->psParent) {
            x = i_psNode->sLocation.nX;
            y = i_psNode->sLocation.nY;

            /// Store path in overrides
            m_oMap.SetOverrides(x, y, 0x01);

            /// Move towards the start
            i_psNode = i_psNode->psParent;
        }

        int nIslandSeconds = 0; // TODO island seconds calculation
        std::cout << "Travelling will take " << nIslandSeconds << " island seconds on the shortes path." << std::endl;
        std::cout << "Travelling will take " << (double)nIslandSeconds/60.0 << " island mins on the shortes path." << std::endl;
        std::cout << "Travelling will take " << (double)nIslandSeconds/60.0/60.0 << " island hours on the shortes path." << std::endl;
    }





    const int &cPlanner::Heuristic(const uint i_nX, const uint i_nY) const {
        return m_mnHeuristic[i_nX][i_nY];
    }

    const int &cPlanner::Heuristic(const tLocation &i_sLocation) const {
        return m_mnHeuristic[i_sLocation.nX][i_sLocation.nY];
    }



}
