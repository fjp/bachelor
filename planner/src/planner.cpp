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



    cPlanner::cPlanner(cRoverInterface<8> *i_oRover, cGraph &i_oMap, uint8_t i_oStepSize) : cPlannerInterface(static_cast<cAudiRover*>(i_oRover), i_oMap) {

        m_nStepSize = i_oStepSize;
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


    int cPlanner::GradX(int i_nX, int i_nY) {
        if(i_nX == m_nStepSize) {
            return m_oMap.Elevation(i_nX, i_nY);
        }
        return m_oMap.Elevation(i_nX, i_nY) - m_oMap.Elevation(i_nX-m_nStepSize, i_nY);
    }


    int cPlanner::GradY(int i_nX, int i_nY) {
        if(i_nY < m_nStepSize) {
            return m_oMap.Elevation(i_nX, i_nY);
        }
        return m_oMap.Elevation(i_nX, i_nY) - m_oMap.Elevation(i_nX, i_nY-m_nStepSize);
    }

    // Generate a Manhattan Heuristic Vector
    void cPlanner::GenerateHeuristic()
    {

        int nD1 = m_nStepSize;
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

    bool cPlanner::GoalTest(const tNode &i_sFirst, const tNode &i_sSecond) const {
        uint nDeltaX = std::abs(i_sFirst.sLocation.nX - i_sSecond.sLocation.nX);
        uint nDeltaY = std::abs(i_sFirst.sLocation.nY - i_sSecond.sLocation.nY);
        return nDeltaX <= m_nStepSize && nDeltaY <= m_nStepSize;
    }

    tNode cPlanner::Child(tNode &i_sParent, const tAction &i_sAction)
    {
        tNode sNext = i_sParent;
        sNext.psParent = &i_sParent;
        sNext.sLocation.nX = i_sParent.sLocation.nX + i_sAction.nX * m_nStepSize;
        sNext.sLocation.nY = i_sParent.sLocation.nY + i_sAction.nY * m_nStepSize;
        sNext.sAction = i_sAction;

        sNext.g = i_sParent.g + i_sAction.fCost * m_nStepSize;
        
        /// Calculate hash of node 
        sNext.nId = NodeHash(sNext);

        return sNext;
    }


    bool cPlanner::WithinMap(const tLocation &i_sLocation) const {

        uint nX = i_sLocation.nX;
        uint nY = i_sLocation.nY;

        return nX >= 0 && nX < m_oMap.Width() && nY >= 0 && nY < m_oMap.Height();
    }
    
    
    int cPlanner::NodeHash(const tNode &i_sNode) 
    {
        return i_sNode.nY * m_oMap.Width() + i_sNode.nX;
    }


    void cPlanner::Plan()
    {
        /// Define start node
        tNode sStart;
        sStart.psParent = NULL;
        sStart.sLocation = m_oRover->Start();
        sStart.g = 0;
        sStart.f = sStart.g + Heuristic(sStart.sLocation);


        PriorityQueue<tNode, double> m_oFrontier;



        m_oFrontier.put(sStart, 0);

        /// Get the goal node
        tNode sGoal;
        sGoal.sLocation = m_oRover->Goal();


        tNode sCurrent;

        /// Serves as explored (closed) set and cost to reach a node
        std::map<tNode, double> oCost;

        //
        oCost[sStart] = 0.0;

        // Flags and Counts
        bool bFound = false;
        bool bResign = false;
        int count = 0;

        tLocation sNextLocation;

        while (!bFound && !bResign) {

            /// Resign if the frontier is empty, which means there are no nodes to expand and the goal has not been found
            if (m_oFrontier.empty())
            {
                bResign = true;
                std::cout << "Goal location not found." << std::endl;
                return;
            }

            sCurrent = m_oFrontier.get();

            if (GoalTest(sCurrent, sGoal))
            {
                bFound = true;
            }
            else
            {
                for (auto sAction : m_oRover->m_asActions) {
                    tNode sNext = Child(sCurrent, sAction);

                    sNextLocation.nX = sNext.sLocation.nX; //sCurrent.sLocation.nX + sAction.nX * m_nStepSize;
                    sNextLocation.nY = sNext.sLocation.nY; //sCurrent.sLocation.nY + sAction.nY * m_nStepSize;

                    if (WithinMap(sNextLocation)) {
                        bool bWater = m_oMap.Water(sCurrent.sLocation.nX, sCurrent.sLocation.nY,
                                                   sNextLocation.nX, sNextLocation.nY);
                        if (!bWater) {
                            /// Calculate current gradient in step direction and normalize it
                            double fHeightCost =
                                    (m_oMap.Elevation(sNextLocation.nX, sNextLocation.nY) - m_oMap.Elevation(sCurrent.sLocation.nX, sCurrent.sLocation.nY)) / m_nMaxGradient;
                            //std::cout << fHeightCost << std::endl;
                            sNext.g = sNext.g + fHeightCost; // TODO height cost
                            //nIslandSeconds += g2; // TODO fix island seconds calculation; must be outside of this loop


                            if (oCost.find(sNext) == oCost.end() || sNext.g < oCost[sNext]) {
                                oCost[sNext] = sNext.g;

                                sNext.f = sNext.g + Heuristic(sNextLocation);
                                m_oFrontier.put(sNext, sNext.f);
                            }
                        }
                    }
                }
            }
        }

        int x, y;
        /// Check if the current node is the start node, which has no parent and is therefore set to NULL
        while (sCurrent.psParent != NULL) {
            x = sCurrent.sLocation.nX;
            y = sCurrent.sLocation.nY;

            /// Store path in overrides
            m_oMap.SetOverrides(x, y, 0x01);

            /// Move towards the start
            sCurrent = *sCurrent.psParent;
        }
        //

    }


    void cPlanner::PlanClean()
    {

        // Defined the quadruplet values
        int x = m_oRover->Start().nX;
        int y = m_oRover->Start().nY;
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
                // Stored the popped value into next
                next = open.back();
                open.pop_back();

                x = (int)next[2];
                y = (int)next[3];
                g = next[1];

                // Fill the expand vectors with count
                expand[x][y] = count;
                count += 1;


                /// Check if we reached the goal:
                //if (x == m_oRover->m_afGoal[0] && y == m_oRover->m_afGoal[1]) {
                if (std::abs(x - m_oRover->Goal().nX) <= (m_nStepSize) && std::abs(y - m_oRover->Goal().nY) <= (m_nStepSize)) {
                    found = true;
                    //cout << "[" << g << ", " << x << ", " << y << "]" << endl;
                }

                    //else expand new elements
                else {
                    for (int i = 0; i < m_oRover->m_asActions.size(); ++i) {
                        auto direction = m_oRover->m_asActions[i];
                        x2 = x + direction.nX * m_nStepSize;
                        y2 = y + direction.nY * m_nStepSize;
                        if (x2 >= 0 && x2 < m_oMap.Width() && y2 >= 0 && y2 < m_oMap.Height()) {
                            bool bWater = m_oMap.Water(x, y, x2, y2);
                            if (closed[x2][y2] == 0 and not bWater) {
                                /// Calculate current gradient in step direction and normalize it
                                double fHeightCost = (m_oMap.Elevation(x2, y2) - m_oMap.Elevation(x, y)) / m_nMaxGradient;
                                //std::cout << fHeightCost << std::endl;
                                double g2 = g + direction.fCost*m_nStepSize + fHeightCost; // TODO height cost
                                //nIslandSeconds += g2; // TODO fix island seconds calculation; must be outside of this loop
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

        int nIslandSeconds = 0; // TODO island seconds calculation
        std::cout << "Travelling will take " << nIslandSeconds << " island seconds on the shortes path." << std::endl;
        std::cout << "Travelling will take " << (double)nIslandSeconds/60.0 << " island mins on the shortes path." << std::endl;
        std::cout << "Travelling will take " << (double)nIslandSeconds/60.0/60.0 << " island hours on the shortes path." << std::endl;

        // Print the expansion List
        //Print2DVector(expand);

        /// Find the path including the robot direction
        std::vector<std::vector<std::string> > policy(m_oMap.Height(), std::vector<std::string>(m_oMap.Width(), "-"));


        /// Going backward

        /// Find the goal position if the step size is greater than one
        //x = m_oRover->m_afGoal[0];
        //y = m_oRover->m_afGoal[1];

        //x = nOffsetX;//m_oRover->m_afGoal[0]-4;
        //y = nOffsetY;//m_oRover->m_afGoal[1]-5;
        policy[x][y] = '*';

        while (x != m_oRover->Start().nX or y != m_oRover->Start().nY) {
            int nAction = action[x][y];
            x2 = x - m_oRover->m_asActions[nAction].nX * m_nStepSize;
            y2 = y - m_oRover->m_asActions[nAction].nY * m_nStepSize;
            /// Store the  Path in a vector // TODO not really needed
            m_oRover->m_mnPath.push_back({ x2, y2 });
            policy[x2][y2] = m_oRover->m_astrMovementArrows[action[x][y]];
            x = x2;
            y = y2;


            /// Store path in overrides
            m_oMap.SetOverrides(x2, y2, 0x01);
        }

#ifdef DEBUG_FILES
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
#endif // DEBUG_FILES

        //return policy; // TODO return plan
    }

    const int &cPlanner::Heuristic(const uint i_nX, const uint i_nY) const {
        return m_mnHeuristic[i_nX][i_nY];
    }

    const int &cPlanner::Heuristic(const tLocation i_sLocation) const {
        return m_mnHeuristic[i_sLocation.nX][i_sLocation.nY];
    }



}
