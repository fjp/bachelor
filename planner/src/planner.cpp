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



namespace planner {



    cPlanner::cPlanner(std::shared_ptr<cRoverInterface<8>> i_poRover, std::shared_ptr<cGraph> i_oMap)
            : cPlannerInterface(std::static_pointer_cast<cAudiRover>(i_poRover), i_oMap), m_nMaxGradient(0), m_fConsistencyFactor(0.f) {

        // TODO consider check if value is already calculated.
        CalculateConsistencyFactor();

    }

    void cPlanner::CalculateConsistencyFactor()
    {
        /// Calculate maximum elevation gradient of the map
        m_nMaxGradient = 0;
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
            }
        }


        float fAlpha = atan(m_nMaxGradient / static_cast<float>(m_poRover->StepSize()));
        float fAlphaAbs = fabs(fAlpha);

        float g = 9.81;
        float fDen = g * sin(2.f * fAlphaAbs);
        float fDeltaS = std::max(m_poRover->CostStraight(), m_poRover->CostDiagonal()) * m_poRover->StepSize();
        m_fConsistencyFactor = m_nMaxGradient + ceil(sqrt(4.f * fDeltaS / fDen));

        std::cout << "Max gradient: " << (int)m_nMaxGradient << ", Consistency factor: " << m_fConsistencyFactor << std::endl;

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

    float cPlanner::UpdateHeuristic(std::shared_ptr<tNode> i_sNode, const tHeuristic i_eHeuristic) const {

        float fHeuristicValue = UpdateHeuristic(i_sNode->sLocation, i_eHeuristic);


        /// Correct heuristic value to get a consistent heuristic. Required because of moving up or down the hill.
        i_sNode->h = fHeuristicValue / m_fConsistencyFactor;

        return fHeuristicValue;
    }

    float cPlanner::UpdateHeuristic(const tLocation &i_sLocation, const tHeuristic i_eHeuristic) const {
        int fDeltaX = std::abs(m_poRover->Goal().nX - i_sLocation.nX);
        int fDeltaY = std::abs(m_poRover->Goal().nY - i_sLocation.nY);

        float fHeuristicValue = 0.f;
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
                float fD1 = static_cast<float>(m_poRover->StepSize()) / static_cast<float>(m_poRover->Velocity());
                float fD2 = sqrt(2.f); // TODO
                fHeuristicValue = fD1 * (fDeltaX + fDeltaY) + (fD2 - 2.f * fD1) * std::min(fDeltaX, fDeltaY);
                break;
            }
            case CHEBYSHEV: {
                /// Euclidian Distance
                fHeuristicValue = std::max(fDeltaX, fDeltaY);
                break;
            }
        }

        return fHeuristicValue;

    }

    void cPlanner::HeuristicCheck(std::shared_ptr<tNode>& i_sNode) const {
        float fStepCost = i_sNode->g - i_sNode->psParent->g;
        //if (!(Heuristic(sNext) <= StepCost + Heuristic(sNext->psParent)))
        //if (!(i_sNode->h <= fStepCost + i_sNode->psParent->h))
        if (!(i_sNode->psParent->h <= fStepCost + i_sNode->h))
        {
            //std::cout << "Heuristic not consistent: " << i_sNode->h << " > " << fStepCost << " + " << i_sNode->psParent->h << " = " << fStepCost + i_sNode->psParent->h << std::endl;
            std::cout << "Heuristic not consistent: " << i_sNode->psParent->h << " > " << fStepCost << " + " << i_sNode->h << " = " << fStepCost + i_sNode->h << std::endl;
        }
    }


    void cPlanner::UpdateCost(std::shared_ptr<tNode> io_sNode) const
    {
        auto psParent = io_sNode->psParent;
        if (nullptr != psParent) {
            /// Rover's normal speed is 1 cell per island second
            float fV = m_poRover->Velocity();
            float fDeltaS = io_sNode->sAction.fCost * m_poRover->StepSize();

            /// Add action (step) cost, which is given in island seconds
            float fStepCost = io_sNode->sAction.fCost; //fDeltaS / fV;


            /// If the rover is going up or down hill, calculate the acceleration on the inclined plane
            /// Calculate current gradient in step direction
            float fDeltaHeight =
                    (m_oMap->Elevation(io_sNode->sLocation.nX, io_sNode->sLocation.nY) -
                     m_oMap->Elevation(psParent->sLocation.nX, psParent->sLocation.nY));


            /*
            float fAlpha = atan(static_cast<float>(fDeltaHeight) / static_cast<float>(m_poRover->StepSize()));
            float fAlphaAbs = fabs(fAlpha);

            float fHeightCost = 0.f;

            if (fAlphaAbs > 0.f) {
                float g = 9.81;
                float fDen = g * sin(2.f * fAlphaAbs);
                fHeightCost = sqrt(4.f * fDeltaS / fDen);

                if (fAlpha < 0.f) /// Down hill
                {
                    fHeightCost *= -1.f;
                }

            }
             */


            float fHeightCost = 0.f;

            if (fDeltaHeight > 0)
            {
                fHeightCost = io_sNode->sAction.fCost * 1.f; //fDeltaHeight / (float)m_nMaxGradient; //10.f;
            }
            else if (fDeltaHeight < 0)
            {
                fHeightCost = -io_sNode->sAction.fCost * 0.2f; //fDeltaHeight / (float)m_nMaxGradient;//0.2f;
            }
            else
            {
                fHeightCost = 0.f;
            }



            float fTime = fStepCost + fHeightCost;

            io_sNode->g = psParent->g + fTime;

        }

    }


    void cPlanner::GenerateHeuristic()
    {
        std::ofstream oFile;
        oFile.open ("heuristic.txt");
        std::vector<std::vector<float> > mfHeuristic(m_oMap->Height(), std::vector<float>(m_oMap->Width()));
        for (int32_t nX = 0; nX < mfHeuristic.size(); nX++) {
            for (int32_t nY = 0; nY < mfHeuristic[0].size(); nY++) {

                /// Output octile distance
                oFile << UpdateHeuristic(tLocation{ nX, nY }, OCTILE) << ' ';
            }
            oFile << std::endl;
        }
        oFile.close();
    }


    bool cPlanner::GoalTest(std::shared_ptr<tNode>& i_sFirst, std::shared_ptr<tNode>& i_sSecond) const {

        int nDeltaX = std::abs(i_sFirst->sLocation.nX - i_sSecond->sLocation.nX);
        int nDeltaY = std::abs(i_sFirst->sLocation.nY - i_sSecond->sLocation.nY);
        return nDeltaX < m_poRover->StepSize() && nDeltaY < m_poRover->StepSize();
    }

    std::shared_ptr<tNode> cPlanner::Child(std::shared_ptr<tNode> i_sParent, const tAction &i_sAction) const
    {
        auto sNext = std::make_shared<tNode>(tNode(*i_sParent));
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


    uint32_t cPlanner::NodeHash(std::shared_ptr<tNode>& i_sNode) const
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


    float cPlanner::Plan() {
        return AStar();
    }


    float cPlanner::AStar()
    {
        /// Define start node
        auto sStart = std::make_shared<tNode>(tNode(m_poRover->Start()));
        /// Create hash of the node using its position
        sStart->nId = NodeHash(sStart);
        UpdateHeuristic(sStart);

        m_oFrontier.put(sStart, sStart->h);

        /// Get the goal node
        auto sGoal = std::make_shared<tNode>(tNode(m_poRover->Goal()));

        /// Create current node
        auto sCurrent = std::make_shared<tNode>(tNode());

        /// Serves as explored (closed) set and cost to reach a node
        std::map<tNode, double> oPathCost;

        /// Initialize start node with cost of zero because it does not cost anything to go to it
        oPathCost[*sStart] = 0.0;

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

                        UpdateCost(sNext);

                        /// Check if the node is already explored and if its path cost got smaller (found a better path to it).
                        if (oPathCost.find(*sNext) == oPathCost.end() || sNext->g < oPathCost[*sNext]) {
                            oPathCost[*sNext] = sNext->g;
                            UpdateHeuristic(sNext);

                            /// Check if the heuristic of the node is consistent h(n) <= c(p,n) + h(p).
                            HeuristicCheck(sNext);

                            /// Update the evaluation score value and put it on the frontier.
                            sNext->f = sNext->g + sNext->h;
                            m_oFrontier.put(sNext, sNext->f);
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

        float fIslandSeconds = sCurrent->g;
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


        std::ofstream of("pic.bmp", std::ofstream::binary);
        visualizer::writeBMP(
                of,
                &m_oMap->m_oElevation[0],
                IMAGE_DIM,
                IMAGE_DIM,
                [&] (size_t x, size_t y, uint8_t elevation) {

                    // Marks interesting positions on the map
                    if (visualizer::donut(x, y, ROVER_X, ROVER_Y) ||
                            visualizer::donut(x, y, BACHELOR_X, BACHELOR_Y) ||
                            visualizer::donut(x, y, WEDDING_X, WEDDING_Y))
                    {
                        return uint8_t(visualizer::IPV_PATH);
                    }

                    if (visualizer::path(x, y, &m_oMap->m_oOverrides[0]))
                    {
                        return uint8_t(visualizer::IPV_PATH);
                    }

                    // Signifies water
                    if ((m_oMap->m_oOverrides[y * IMAGE_DIM + x] & (OF_WATER_BASIN | OF_RIVER_MARSH)) ||
                        elevation == 0)
                    {
                        return uint8_t(visualizer::IPV_WATER);
                    }

                    // Signifies normal ground color
                    if (elevation < visualizer::IPV_ELEVATION_BEGIN)
                    {
                        elevation = visualizer::IPV_ELEVATION_BEGIN;
                    }
                    return elevation;
                });
        of.flush();
#if __APPLE__
        //auto res = system("open pic.bmp");
        //(void)res;
#endif
    }


    void cPlanner::TraversePath(std::shared_ptr<tNode> i_psNode) const
    {
        uint32_t nX, nY;
        /// Check if the current node is the start node, which has no parent and is therefore set to NULL
        while (nullptr != i_psNode->psParent) {
            nX = i_psNode->sLocation.nX;
            nY = i_psNode->sLocation.nY;

            /// Store path in overrides
            m_oMap->SetOverrides(nX, nY, 0x01);

            /// Move towards the start
            i_psNode = i_psNode->psParent;
        }

    }

}
