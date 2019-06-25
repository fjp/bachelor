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



    cPlanner::cPlanner(std::shared_ptr<cRoverInterface<8>> i_poRover, std::shared_ptr<cGraph> i_poMap)
            : cPlannerInterface(std::static_pointer_cast<cAudiRover>(i_poRover), i_poMap)
            , m_nMaxGradient(0)
            , m_fConsistencyFactor(0.0) {

        //std::cout << "Constructing cPlanner" << std::endl;

        CalculateConsistencyFactor();
    }

    void cPlanner::CalculateConsistencyFactor()
    {
        /// Calculate maximum elevation gradient of the map and find its maximum elevation
        m_nMaxGradient = 0;
        int nMaxElevation = 0;
        for (int nY = 0; nY < m_poMap->Height(); ++nY)
        {
            for (int nX = 0; nX < m_poMap->Width(); ++nX)
            {
                int nGradX = GradX(nX, nY);
                int nGradY = GradY(nX, nY);
                int nCurrentMaxGradient = std::max(std::abs(nGradX), std::abs(nGradY));
                if (m_nMaxGradient < nCurrentMaxGradient)
                {
                    m_nMaxGradient = nCurrentMaxGradient;
                }
                if (nMaxElevation < m_poMap->Elevation(nX, nY))
                {
                    nMaxElevation = m_poMap->Elevation(nX, nY);
                }
            }
        }

        m_fConsistencyFactor = 1.f - static_cast<double>(m_nMaxGradient) / 255.0;


        //double fAlpha = atan(m_nMaxGradient / static_cast<double>(m_poRover->StepSize()));
        //double fAlphaAbs = fabs(fAlpha);

        //double g = 9.81;
        //double fDen = g * sin(2.f * fAlphaAbs);
        //double fDeltaS = std::max(m_poRover->CostStraight(), m_poRover->CostDiagonal()) * m_poRover->StepSize();
        //m_fConsistencyFactor = m_nMaxGradient + ceil(sqrt(4.f * fDeltaS / fDen));

        std::cout << "Analyzing map: \n"
            << "    Map Size:           " << static_cast<int>(m_poMap->Height()) << "x" << static_cast<int>(m_poMap->Width()) << "\n"
            << "    Max elevation:      " << static_cast<int>(nMaxElevation) << "\n"
            << "    Max gradient:       " << static_cast<int>(m_nMaxGradient) << "\n"
            << "    Consistency factor: " << m_fConsistencyFactor << "\n" << std::endl;

    }


    const int cPlanner::GradX(int i_nX, int i_nY) const {
        if(i_nX == m_poRover->StepSize() || i_nX == 0) {
            return m_poMap->Elevation(i_nX, i_nY);
        }
        return m_poMap->Elevation(i_nX, i_nY) - m_poMap->Elevation(i_nX - m_poRover->StepSize(), i_nY);
    }


    const int cPlanner::GradY(int i_nX, int i_nY) const {
        if(i_nY < m_poRover->StepSize()) {
            return m_poMap->Elevation(i_nX, i_nY);
        }
        return m_poMap->Elevation(i_nX, i_nY) - m_poMap->Elevation(i_nX, i_nY - m_poRover->StepSize());
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
                fHeuristicValue = fD1 * (fDeltaX + fDeltaY) + (fD2 - 2.0 * fD1) * std::min(fDeltaX, fDeltaY);
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
                    (m_poMap->Elevation(i_sNode->sLocation.nX, i_sNode->sLocation.nY) -
                     m_poMap->Elevation(i_sNode->psParent->sLocation.nX, i_sNode->psParent->sLocation.nY));

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


    bool cPlanner::GoalTest(std::shared_ptr<tNode>& i_sFirst, std::shared_ptr<tNode>& i_sSecond) const {

        int nDeltaX = std::abs(i_sFirst->sLocation.nX - i_sSecond->sLocation.nX);
        int nDeltaY = std::abs(i_sFirst->sLocation.nY - i_sSecond->sLocation.nY);
        return nDeltaX < m_poRover->StepSize() && nDeltaY < m_poRover->StepSize();
    }

    std::shared_ptr<tNode> cPlanner::Child(std::shared_ptr<tNode>& i_sParent, const tAction &i_sAction) const
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

        return nX >= 0 && nX < m_poMap->Width() && nY >= 0 && nY < m_poMap->Height();
    }


    int cPlanner::NodeHash(std::shared_ptr<tNode>& i_sNode) const
    {
        return i_sNode->sLocation.nY * m_poMap->Width() + i_sNode->sLocation.nX;
    }

    bool cPlanner::Traversable(std::shared_ptr<tNode> i_sCurrent, std::shared_ptr<tNode> i_sNext) const {
        if (WithinMap(i_sNext->sLocation)) {
            /// Check if the intermediate locations moving from current node to next are on mainland or water
            bool bWater = m_poMap->Water(i_sCurrent->sLocation.nX, i_sCurrent->sLocation.nY,
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


    tResult cPlanner::Plan() {
        return AStar();
    }


    tResult cPlanner::AStar()
    {
        std::cout << "Planning optimal path with A*" << std::endl;

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

        /// Initialize result struct
        m_sResult.bFoundGoal = false;
        m_sResult.nIterations = 0;

        while (!m_sResult.bFoundGoal) {

            m_sResult.nIterations++;
            if (m_sResult.nIterations % 100000 == 0)
            {
                std::cout << "Iteration " << m_sResult.nIterations
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
                m_sResult.bFoundGoal = false;
                break;
            }

            sCurrent = m_oFrontier.get();

            if (GoalTest(sCurrent, sGoal)) {
                m_sResult.bFoundGoal = true;
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
                            m_poMap->SetOverrides(sNext->sLocation.nX, sNext->sLocation.nY, 0x02);
                            m_sResult.nNodesExpanded++;

                            /// Check that heuristic never overestimates the true distance:
                            /// Priority of a new node should never be lower than the priority of its parent.
                            if (sNext->psParent && sNext->f < sNext->psParent->f - 1.0e-8)
                            {
                                m_sResult.bConsistentHeuristic = false;
                                std::cout << "Heuristic overestimates true distance" << std::endl;
                            }
                        }
                    }
                }
            }
        }

        if (m_sResult.bFoundGoal) {
            /// Move from the current node back to the start node
            TraversePath(sCurrent);
        }

        PrintTravelResult();

        /// Free memory
        oPathCost.clear();
        m_oFrontier.clear();


        return m_sResult;
    }


    void cPlanner::Plot()
    {
        auto sNode = m_oFrontier.pop();
        TraversePath(sNode);

        //visualizer::write("pic_intermediate.bmp", &m_poMap->m_oElevation[0], &m_poMap->m_oOverrides[0], IMAGE_DIM, ALL);
#if __APPLE__
        //auto res = system("open pic.bmp");
        //(void)res;
#endif
    }

    void cPlanner::PrintTravelResult()
    {
        if (!m_sResult.bFoundGoal)
        {
            std::cout << "Failed to find goal location!" << std::endl;
        }
        else {

            std::cout << "\nTravel Results:" << std::endl;
            double fIslandSeconds = m_sResult.fTravellingTime;
            std::cout << "Travelling will take " << fIslandSeconds << " island seconds ("
                      << fIslandSeconds / 60.0 << " island minutes or " << fIslandSeconds / 60.0 / 60.0
                      << " island hours) on the fastest path. " << std::endl;
            std::cout << "Cumulative elevation: " << m_sResult.nCumulativeElevation << std::endl;
            std::cout << "Number of expanded nodes: " << m_sResult.nNodesExpanded << std::endl;
            std::cout << "Heuristic is " << (m_sResult.bConsistentHeuristic ? "consistent" : "NOT consistent") << "\n" << std::endl;
        }
    }


    void cPlanner::TraversePath(std::shared_ptr<tNode>& i_psNode)
    {
        /// Set the cost (time) it takes to get to the goal
        m_sResult.fTravellingTime = i_psNode->g;

        /// Reconstruct the path by going backward from the goal location
        int nXCurrent = i_psNode->sLocation.nX;
        int nYCurrent = i_psNode->sLocation.nY;

        /// Update cumulative elevation
        m_sResult.nCumulativeElevation += m_poMap->Elevation(nXCurrent, nYCurrent);
        /// Store path in overrides
        m_poMap->SetOverrides(nXCurrent, nYCurrent, 0x01);

        /// Check if the current node is the start node, which has no parent and is therefore set to NULL
        while (nullptr != i_psNode->psParent) {
            /// Move towards the start
            i_psNode = i_psNode->psParent;
            nXCurrent = i_psNode->sLocation.nX;
            nYCurrent = i_psNode->sLocation.nY;

            /// Update cumulative elevation
            m_sResult.nCumulativeElevation += m_poMap->Elevation(nXCurrent, nYCurrent);
            /// Store path in overrides
            m_poMap->SetOverrides(nXCurrent, nYCurrent, 0x01);
        }
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
                    (m_poMap->Elevation(io_psNode->sLocation.nX, io_psNode->sLocation.nY) -
                     m_poMap->Elevation(psParent->sLocation.nX, psParent->sLocation.nY));



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
        std::vector<std::vector<double> > mfHeuristic(m_poMap->Height(), std::vector<double>(m_poMap->Width()));
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
