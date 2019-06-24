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


#include "visualizer.h"
#include "constants.h"

#include <map>

namespace planner {



    cPlanner::cPlanner(std::shared_ptr<cRoverInterface<8>> i_poRover, std::shared_ptr<cGraph> i_oMap)
            : cPlannerInterface(std::static_pointer_cast<cAudiRover>(i_poRover), i_oMap)
            , m_nMaxGradient(0)
            , m_fConsistencyFactor(0.f) {

        std::cout << "cPlanner" << std::endl;

        // TODO consider check if value is already calculated.
        CalculateConsistencyFactor();

    }

    void cPlanner::CalculateConsistencyFactor()
    {
        /// Calculate maximum elevation gradient of the map and find its maximum elevation
        m_nMaxGradient = 0;
        int nMaxElevation = 0;
        for (int nY = 0; nY < m_oMap->Height(); ++nY)
        {
            for (int nX = 0; nX < m_oMap->Width(); ++nX)
            {
                int nGradX = GradX(nX, nY);
                int nGradY = GradY(nX, nY);
                int nCurrentMaxGradient = std::max(std::abs(nGradX), std::abs(nGradY));
                if (m_nMaxGradient < nCurrentMaxGradient)
                {
                    m_nMaxGradient = nCurrentMaxGradient;
                }
                if (nMaxElevation < m_oMap->Elevation(nX, nY))
                {
                    nMaxElevation = m_oMap->Elevation(nX, nY);
                }
            }
        }

        m_fConsistencyFactor = 1.f - static_cast<double>(m_nMaxGradient) / 255.f;


        //double fAlpha = atan(m_nMaxGradient / static_cast<double>(m_poRover->StepSize()));
        //double fAlphaAbs = fabs(fAlpha);

        //double g = 9.81;
        //double fDen = g * sin(2.f * fAlphaAbs);
        //double fDeltaS = std::max(m_poRover->CostStraight(), m_poRover->CostDiagonal()) * m_poRover->StepSize();
        //m_fConsistencyFactor = m_nMaxGradient + ceil(sqrt(4.f * fDeltaS / fDen));

        std::cout << "Analyzing map: \n"
            << "    Max elevation:      " << static_cast<int>(nMaxElevation) << "\n"
            << "    Max gradient:       " << static_cast<int>(m_nMaxGradient) << "\n"
            << "    Consistency factor: " << m_fConsistencyFactor << std::endl;

    }


    const int cPlanner::GradX(int i_nX, int i_nY) const {
        if(i_nX == m_poRover->StepSize() || i_nX == 0) {
            return m_oMap->Elevation(i_nX, i_nY);
        }
        return m_oMap->Elevation(i_nX, i_nY) - m_oMap->Elevation(i_nX - m_poRover->StepSize(), i_nY);
    }


    const int cPlanner::GradY(int i_nX, int i_nY) const {
        if(i_nY < m_poRover->StepSize()) {
            return m_oMap->Elevation(i_nX, i_nY);
        }
        return m_oMap->Elevation(i_nX, i_nY) - m_oMap->Elevation(i_nX, i_nY - m_poRover->StepSize());
    }

    double cPlanner::UpdateHeuristic(std::shared_ptr<tNode> i_psNode, const tHeuristic i_eHeuristic) const {

        double fHeuristicValue = Heuristic(i_psNode->sLocation, i_eHeuristic);

        i_psNode->h = fHeuristicValue;

        return fHeuristicValue;
    }

    double cPlanner::Heuristic(const tLocation &i_sLocation, const tHeuristic i_eHeuristic) const {
        int fDeltaX = std::abs(m_poRover->Goal().nX - i_sLocation.nX);
        int fDeltaY = std::abs(m_poRover->Goal().nY - i_sLocation.nY);

        double fHeuristicValue = 0.f;
        switch (i_eHeuristic) {
            case MANHATTEN: {
                /// Manhattan Distance
                fHeuristicValue = fDeltaX + fDeltaY;
                break;
            }
            case EUCLIDEAN: {
                /// Euclidian Distance
                fHeuristicValue = sqrt(fDeltaX * fDeltaX + fDeltaY * fDeltaY);
                break;
            }
            case OCTILE: {
                /// Octile distance
                double fD1 = static_cast<double>(m_poRover->StepSize()) / static_cast<double>(m_poRover->Velocity());
                double fD2 = m_poRover->CostDiagonal();
                fHeuristicValue = fD1 * (fDeltaX + fDeltaY) + (fD2 - 2.f * fD1) * std::min(fDeltaX, fDeltaY);
                break;
            }
            case CHEBYSHEV: {
                /// Euclidian Distance
                fHeuristicValue = std::max(fDeltaX, fDeltaY);
                break;
            }
        }

        /// Correct heuristic value to get a consistent heuristic. Required because of moving up or down the hill.
        return fHeuristicValue * m_fConsistencyFactor; //0.8; // / m_fConsistencyFactor;

    }

    void cPlanner::HeuristicCheck(std::shared_ptr<tNode>& i_sNode) {
        double fStepCost = i_sNode->g - i_sNode->psParent->g;
        //if (!(Heuristic(sNext) <= StepCost + Heuristic(sNext->psParent)))
        //if (!(i_sNode->h <= fStepCost + i_sNode->psParent->h))
        double epsilon = std::numeric_limits<double>::epsilon();
        if (!(i_sNode->psParent->h <= fStepCost + i_sNode->h + 1e-8f))//std::numeric_limits<double>::epsilon()))
        {

            double fDeltaHeight =
                    (m_oMap->Elevation(i_sNode->sLocation.nX, i_sNode->sLocation.nY) -
                     m_oMap->Elevation(i_sNode->psParent->sLocation.nX, i_sNode->psParent->sLocation.nY));

            //std::cout << "Heuristic not consistent: " << i_sNode->h << " > " << fStepCost << " + " << i_sNode->psParent->h << " = " << fStepCost + i_sNode->psParent->h << std::endl;
            std::cout << "Heuristic not consistent: "
                << i_sNode->psParent->h << " > " << fStepCost << " + " << i_sNode->h << " = " << fStepCost + i_sNode->h
                << "; " << (fDeltaHeight > 0.f ? "Moving uphill" : "Moving downhill")
                << " at location " << "(" << i_sNode->psParent->sLocation.nX << "," << i_sNode->psParent->sLocation.nY << ")"
                << " -> " << "(" << i_sNode->sLocation.nX << "," << i_sNode->sLocation.nY << ")" << std::endl;


            /// Store this incident in the result struct of the planner interface
            m_sResult.bConsistentHeuristic = false;
        }
    }

    template<typename TLocation>
    double cPlanner::HeightCost(TLocation& i_sCurrent, TLocation& i_sNext, tAction& i_sAction) const
    {
        /// If the rover is going up or down hill, calculate the acceleration on the inclined plane
        /// Calculate current gradient in step direction
        double fDeltaHeight =
                (m_oMap->Elevation(i_sNext.nX, i_sNext.nY) -
                 m_oMap->Elevation(i_sCurrent.nX, i_sCurrent.nY));


        double fHeightCost;// = 0.f;
        //if (fDeltaHeight > 0)
        //{
            fHeightCost = i_sAction.fCost * fDeltaHeight / 255.f; //static_cast<double>(m_nMaxGradient + 1); //10.f;
        //}
        //else if (fDeltaHeight < 0)
        //{
        //    fHeightCost = -i_sAction.fCost * fDeltaHeight / static_cast<double>(m_nMaxGradient); //0.2f;
        //}

        return fHeightCost;

    }

    bool cPlanner::GoalTest(std::shared_ptr<tNode>& i_sFirst, std::shared_ptr<tNode>& i_sSecond) const {

        int nDeltaX = std::abs(i_sFirst->sLocation.nX - i_sSecond->sLocation.nX);
        int nDeltaY = std::abs(i_sFirst->sLocation.nY - i_sSecond->sLocation.nY);
        return nDeltaX < m_poRover->StepSize() && nDeltaY < m_poRover->StepSize();
    }

    std::shared_ptr<tNode> cPlanner::Child(std::shared_ptr<tNode> i_sParent, const tAction &i_sAction) const
    {
        auto sNext = std::make_shared<tNode>(*i_sParent);
        sNext->psParent = i_sParent;
        sNext->sLocation.nX = i_sParent->sLocation.nX + i_sAction.nX * m_poRover->StepSize();
        sNext->sLocation.nY = i_sParent->sLocation.nY + i_sAction.nY * m_poRover->StepSize();
        sNext->sAction = i_sAction;
        
        /// Calculate hash of node using its location
        sNext->nId = NodeHash(sNext);

        return sNext;
    }


    bool cPlanner::WithinMap(const tLocation &i_sLocation) const {

        int nX = i_sLocation.nX;
        int nY = i_sLocation.nY;

        return nX >= 0 && nX < m_oMap->Width() && nY >= 0 && nY < m_oMap->Height();
    }


    int cPlanner::NodeHash(std::shared_ptr<tNode>& i_sNode) const
    {
        return i_sNode->sLocation.nY * m_oMap->Width() + i_sNode->sLocation.nX;
    }

    bool cPlanner::Traversable(std::shared_ptr<tNode> i_sCurrent, std::shared_ptr<tNode> i_sNext) const {
        if (WithinMap(i_sNext->sLocation)) {
            /// Check if the intermediate locations moving from current node to next are on mainland or water
            bool bWater = m_oMap->Water(i_sCurrent->sLocation.nX, i_sCurrent->sLocation.nY,
                                       i_sNext->sLocation.nX, i_sNext->sLocation.nY);
            if (bWater)
            {
                return false;
            }
            return true;
        }
        /// Next location lies outside the map
        return false;
    }


    double cPlanner::Plan() {

        switch (m_eAlgorithm)
        {
            case ASTAR:
                return AStar();

            case ASTAR_OPT:
                return AStarOptimized();

            case ASTAR_CK:
                return AStarCheck();
        }
    }

    double cPlanner::AStarCheck() {
        struct tSimpleLocation {
            int nId;
            int nX, nY;

            bool operator<(const tSimpleLocation &i_rhs) const {
                return nId < i_rhs.nId;
            }
        };
        std::map<tSimpleLocation, tSimpleLocation> came_from;
        std::map<tSimpleLocation, double> cost_so_far;

        /// Start
        int nX = m_poRover->Start().nX;
        int nY = m_poRover->Start().nY;
        int nId = nY * m_oMap->Width() + nX;
        tSimpleLocation sStart{nId, nX, nY};


        PriorityQueue<tSimpleLocation, double> frontier;
        frontier.put(sStart, 0.f);

        came_from[sStart] = sStart;
        cost_so_far[sStart] = 0.f;


        tSimpleLocation sCurrent;

        int nIteration = 0;

        // While I am still searching for the goal and the problem is solvable
        while (!frontier.empty()) {

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
            sCurrent = frontier.get();

            // Check if we reached the goal:
            if (sCurrent.nX == m_poRover->Goal().nX && sCurrent.nY == m_poRover->Goal().nY) {
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
                        frontier.put(sNext, priority);
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

    double cPlanner::AStarOptimized()
    {
        /// Simplified node struct without the a pointer to its parent.
        struct tSimpleNode {
            int nId;
            int nX, nY;
            double g;
            double h;
            double f;

            bool operator<(const tSimpleNode& i_rhs) const
            {
                return nId < i_rhs.nId;
            }
        };

        using namespace std;
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
        vector<vector<int> > action(m_oMap->Height(), vector<int>(m_oMap->Width(), -1));


        /// For each node, the cost of getting from the start node to that node.
        vector<vector<double> > gScore(m_oMap->Height(), vector<double>(m_oMap->Width(), std::numeric_limits<double>::max()));

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

            // Resign if no values in the open list and you can't expand anymore
            if (oOpenPrioQ.empty()) {
                bResign = true;
                std::cout << "Failed to reach goal" << std::endl;
            }
                // Keep expanding
            else {
                /// Remove the node from the open priority queue having the lowest fScore value
                tSimpleNode sCurrent;
                sCurrent = oOpenPrioQ.get();

                nX = sCurrent.nX;
                nY = sCurrent.nY;
                g = sCurrent.g;
                double fparent = sCurrent.f;


                /// Check if the goal is reached
                if (nX == m_poRover->Goal().nX && nY == m_poRover->Goal().nY) {
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
                                //closed[x2][y2] = 1;
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

        /// Reconstruct the path by going backward
        nX = m_poRover->Goal().nX;
        nY = m_poRover->Goal().nY;

        int nElevation = 0;

        while (nX != m_poRover->Start().nX || nY != m_poRover->Start().nY) {
            nXNext = nX - m_poRover->m_asActions[action[nX][nY]].nX;
            nYNext = nY - m_poRover->m_asActions[action[nX][nY]].nY;
            // Store the  Path in a vector
            m_oMap->SetOverrides(nXNext, nYNext, 0x01);

            nElevation += m_oMap->Elevation(nXNext, nYNext);

            nX = nXNext;
            nY = nYNext;
        }

        std::cout << "Total Elevation: " << nElevation << std::endl;

        double fIslandSeconds = g;
        std::cout << "Travelling will take " << fIslandSeconds << " island seconds ("
                  << fIslandSeconds/60.f << " island minutes or " << fIslandSeconds/60.f/60.f << " island hours) on the fastest path. " << std::endl;


        m_sResult.fTravellingTime = fIslandSeconds;

        return fIslandSeconds;
    }


    double cPlanner::AStar()
    {
        /// Define start node
        auto sStart = std::make_shared<tNode>(m_poRover->Start());
        /// Create hash of the node using its position
        sStart->nId = NodeHash(sStart);
        UpdateHeuristic(sStart);

        m_oFrontier.put(sStart, sStart->h);

        /// Get the goal node
        auto sGoal = std::make_shared<tNode>(m_poRover->Goal());

        /// Create current node
        auto sCurrent = std::make_shared<tNode>();

        /// Serves as explored (closed) set and cost to reach a node
        std::map<tNode, double> oPathCost;

        /// Initialize start node with cost of zero because it does not cost anything to go to it
        oPathCost[*sStart] = 0.f;

        // Flags and Counts
        bool bFound = false;
        bool bResign = false;
        int nIteration = 0;


        while (!bFound) {

            nIteration++;
            if (nIteration % 100000 == 0)
            {
                std::cout << "Iteration " << nIteration
                << ": Best node location (" << m_oFrontier.pop()->sLocation.nX << "," << m_oFrontier.pop()->sLocation.nY
                << "), \n\t Evaluation function f(n): " << m_oFrontier.pop()->f
                << ", step cost c(n): " << m_oFrontier.pop()->g - m_oFrontier.pop()->psParent->g
                << ", path cost g(n): " << m_oFrontier.pop()->g
                << ", heuristic h(n): " << m_oFrontier.pop()->h
                << std::endl;
                //Plot();
            }

            /// Resign if the frontier is empty, which means there are no nodes to expand and the goal has not been found
            if (m_oFrontier.empty()) {
                bResign = true;
                break;
            }

            sCurrent = m_oFrontier.get();

            if (GoalTest(sCurrent, sGoal)) {
                bFound = true;
            } else {
                for (auto sAction : m_poRover->m_asActions) {
                    auto sNext = Child(sCurrent, sAction);

                    if (Traversable(sCurrent, sNext)) {

                        //UpdateCost(sNext);

                        double fHeightCost = HeightCost(sCurrent->sLocation, sNext->sLocation, sAction);

                        double fTime = sAction.fCost + fHeightCost;

                        sNext->g = sNext->psParent->g + fTime;

                        /// Check if the node is already explored and if its path cost got smaller (found a better path to it).
                        if (oPathCost.find(*sNext) == oPathCost.end() || sNext->g < oPathCost[*sNext])
                        {
                            oPathCost[*sNext] = sNext->g;
                            UpdateHeuristic(sNext);

                            /// Check if the heuristic of the node is consistent h(n) <= c(p,n) + h(p).
                            HeuristicCheck(sNext);

                            /// Update the evaluation score value and put it on the frontier.
                            sNext->f = sNext->g + sNext->h;

                            m_oFrontier.put(sNext, sNext->f);

                            /// Mark visited nodes
                            m_oMap->SetOverrides(sNext->sLocation.nX, sNext->sLocation.nY, 0x02);

                            /// Check that heuristic never overestimates the true distance:
                            /// Priority of a new node should never be lower than the priority of its parent.
                            if (sNext->psParent && sNext->f < sNext->psParent->f - 1e-8f)
                            {
                                std::cout << "Heuristic overestimates true distance" << std::endl;
                            }
                        }
                    }
                }
            }
        }


        if (bResign)
        {
            std::cout << "Goal location not found." << std::endl;
            return -1;
        }


        /// Move from the current node back to the start node
        TraversePath(sCurrent);

        double fIslandSeconds = sCurrent->g;
        std::cout << "Travelling will take " << fIslandSeconds << " island seconds ("
        << fIslandSeconds/60.f << " island minutes or " << fIslandSeconds/60.f/60.f << " island hours) on the fastest path. " << std::endl;

        /// Free memory
        oPathCost.clear();
        m_oFrontier.clear();

        return fIslandSeconds;
    }


    void cPlanner::Plot()
    {
        auto sNode = m_oFrontier.pop();
        TraversePath(sNode);

        //visualizer::write("pic_intermediate.bmp", &m_oMap->m_oElevation[0], &m_oMap->m_oOverrides[0], IMAGE_DIM);
#if __APPLE__
        //auto res = system("open pic.bmp");
        //(void)res;
#endif
    }


    void cPlanner::TraversePath(std::shared_ptr<tNode> i_psNode) const
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


/// Deprecated methods
    void cPlanner::UpdateCost(std::shared_ptr<tNode> io_psNode) const
    {
        auto psParent = io_psNode->psParent;
        if (nullptr != psParent) {
            /// Rover's normal speed is 1 cell per island second
            //double fV = m_poRover->Velocity();
            //double fDeltaS = io_psNode->sAction.fCost * m_poRover->StepSize();

            /// Add action (step) cost, which is given in island seconds
            double fStepCost = io_psNode->sAction.fCost; //fDeltaS / fV;

            /*
            /// If the rover is going up or down hill, calculate the acceleration on the inclined plane
            /// Calculate current gradient in step direction
            double fDeltaHeight =
                    (m_oMap->Elevation(io_psNode->sLocation.nX, io_psNode->sLocation.nY) -
                     m_oMap->Elevation(psParent->sLocation.nX, psParent->sLocation.nY));



            double fAlpha = atan(static_cast<double>(fDeltaHeight) / static_cast<double>(m_poRover->StepSize()));
            double fAlphaAbs = fabs(fAlpha);

            double fHeightCost = 0.f;

            if (fAlphaAbs > 0.f) {
                double g = 9.81;
                double fDen = g * sin(2.f * fAlphaAbs);
                fHeightCost = sqrt(4.f * fDeltaS / fDen);

                if (fAlpha < 0.f) /// Down hill
                {
                    fHeightCost *= -1.f;
                }

            }
             */

            double fHeightCost = HeightCost(io_psNode->sLocation, psParent->sLocation, io_psNode->sAction);

            double fTime = fStepCost + fHeightCost;

            io_psNode->g = psParent->g + fTime;
        }
    }


    void cPlanner::GenerateHeuristic()
    {
        std::ofstream oFile;
        oFile.open ("heuristic.txt");
        std::vector<std::vector<double> > mfHeuristic(m_oMap->Height(), std::vector<double>(m_oMap->Width()));
        for (int32_t nX = 0; nX < mfHeuristic.size(); nX++) {
            for (int32_t nY = 0; nY < mfHeuristic[0].size(); nY++) {

                /// Output octile distance
                oFile << Heuristic(tLocation{ nX, nY }, OCTILE) << ' ';
            }
            oFile << std::endl;
        }
        oFile.close();
    }

}
