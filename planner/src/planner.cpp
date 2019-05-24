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



    cPlanner::cPlanner(cRoverInterface<8> *i_poRover, cGraph &i_oMap)
            : cPlannerInterface(static_cast<cAudiRover*>(i_poRover), i_oMap) {

        /// Calculate maximum elevation gradient of the map
        m_nMaxGradient = 0;
        for (uint32_t nY = 0; nY < m_oMap.Height(); ++nY)
        {
            for (uint32_t nX = 0; nX < m_oMap.Width(); ++nX)
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


        float fAlpha = atan(m_nMaxGradient / static_cast<float>(m_poRover->StepSize()));
        float fAlphaAbs = fabs(fAlpha);

        float g = 9.81;
        float fDen = g * sin(2.f * fAlphaAbs);
        float fDeltaS = std::max(m_poRover->m_fCostStraight, m_poRover->m_fCostDiagonal) * m_poRover->StepSize();
        m_fConsistencyFactor = m_nMaxGradient + ceil(sqrt(4.f * fDeltaS / fDen));

    }


    const int32_t cPlanner::GradX(uint32_t i_nX, uint32_t i_nY) const {
        if(i_nX == m_poRover->StepSize()) {
            return m_oMap.Elevation(i_nX, i_nY);
        }
        return m_oMap.Elevation(i_nX, i_nY) - m_oMap.Elevation(i_nX - m_poRover->StepSize(), i_nY);
    }


    const int32_t cPlanner::GradY(uint32_t i_nX, uint32_t i_nY) const {
        if(i_nY < m_poRover->StepSize()) {
            return m_oMap.Elevation(i_nX, i_nY);
        }
        return m_oMap.Elevation(i_nX, i_nY) - m_oMap.Elevation(i_nX, i_nY - m_poRover->StepSize());
    }

    float cPlanner::UpdateHeuristic(tNode *i_sNode, const tHeuristic i_eHeuristic) const {

        float fHeuristicValue = UpdateHeuristic(i_sNode->sLocation, i_eHeuristic);


        /// Correct heuristic value to get a consistent heuristic. Required because of moving up or down the hill.
        i_sNode->h = fHeuristicValue / m_fConsistencyFactor;

        return fHeuristicValue;
    }

    float cPlanner::UpdateHeuristic(const tLocation &i_sLocation, const tHeuristic i_eHeuristic) const {
        float fDeltaX = std::fabs(m_poRover->Goal().nX - i_sLocation.nX);
        float fDeltaY = std::fabs(m_poRover->Goal().nY - i_sLocation.nY);

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


    void cPlanner::GenerateHeuristic()
    {
        std::ofstream oFile;
        oFile.open ("heuristic.txt");
        std::vector<std::vector<float> > mfHeuristic(m_oMap.Height(), std::vector<float>(m_oMap.Width()));
        for (int32_t nX = 0; nX < mfHeuristic.size(); nX++) {
            for (int32_t nY = 0; nY < mfHeuristic[0].size(); nY++) {

                /// Output octile distance
                oFile << UpdateHeuristic(tLocation{ nX, nY }, OCTILE) << ' ';
            }
            oFile << std::endl;
        }
        oFile.close();
    }


    bool cPlanner::GoalTest(const tNode *i_sFirst, const tNode *i_sSecond) const {

        uint nDeltaX = std::abs(i_sFirst->sLocation.nX - i_sSecond->sLocation.nX);
        uint nDeltaY = std::abs(i_sFirst->sLocation.nY - i_sSecond->sLocation.nY);
        return nDeltaX < m_poRover->StepSize() && nDeltaY < m_poRover->StepSize();
    }

    tNode* cPlanner::Child(tNode *i_sParent, const tAction &i_sAction) const
    {
        tNode* sNext = new tNode(*i_sParent);
        sNext->psParent = i_sParent;
        sNext->sLocation.nX = i_sParent->sLocation.nX + i_sAction.nX * m_poRover->StepSize();
        sNext->sLocation.nY = i_sParent->sLocation.nY + i_sAction.nY * m_poRover->StepSize();
        sNext->sAction = i_sAction;
        
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

    bool cPlanner::Traversable(tNode *i_sCurrent, tNode *i_sNext) const {
        if (WithinMap(i_sNext->sLocation)) {
            /// Check if the intermediate locations moving from current node to next are on mainland or water
            bool bWater = m_oMap.Water(i_sCurrent->sLocation.nX, i_sCurrent->sLocation.nY,
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


    bool cPlanner::Plan() {
        return AStar();
    }


    bool cPlanner::AStar()
    {
        /// Define start node
        tNode *sStart = new tNode(m_poRover->Start());
        /// Create hash of the node using its position
        sStart->nId = NodeHash(sStart);
        UpdateHeuristic(sStart);

        m_oFrontier.put(sStart, sStart->h);

        /// Get the goal node
        tNode *sGoal = new tNode(m_poRover->Goal());

        /// Create current node
        tNode *sCurrent = new tNode();

        /// Serves as explored (closed) set and cost to reach a node
        std::map<tNode, double> oPathCost;

        /// Initialize start node with cost of zero because it does not cost anything to go to it
        oPathCost[*sStart] = 0.0;

        // Flags and Counts
        bool bFound = false;
        bool bResign = false;
        int nIteration = 0;


        while (!bFound && !bResign) {

            nIteration++;
            if (nIteration % 100000 == 0)
            {
                std::cout << "Iteration " << nIteration
                << " Best node location (" << m_oFrontier.pop()->sLocation.nX << "," << m_oFrontier.pop()->sLocation.nY
                << "), \n\t Evaluation function f(n): " << m_oFrontier.pop()->f
                << ", step cost: " << m_oFrontier.pop()->g - m_oFrontier.pop()->psParent->g
                << ", path cost: " << m_oFrontier.pop()->g
                << ", f cost: " << m_oFrontier.pop()->f
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
                    tNode *sNext = Child(sCurrent, sAction);

                    if (Traversable(sCurrent, sNext)) {

                        UpdateCost(sNext);

                        if (oPathCost.find(*sNext) == oPathCost.end() || sNext->g < oPathCost[*sNext]) {
                            oPathCost[*sNext] = sNext->g;
                            UpdateHeuristic(sNext);

                            HeuristicCheck(sNext);

                            sNext->f = sNext->g + sNext->h;
                            m_oFrontier.put(sNext, sNext->f);
                            //std::cout << "Heuristic " << sNext->h << std::endl;
                            //Plot();
                        }
                    }
                }
            }
        }


        if (bResign)
        {
            std::cout << "Goal location not found." << std::endl;
            return false;
        }


        /// Move from the current node back to the start node
        TraversePath(sCurrent);

        float nIslandSeconds = sCurrent->g;
        std::cout << "Travelling will take " << nIslandSeconds << " island seconds ("
        << nIslandSeconds/60.0 << " island minutes or " << nIslandSeconds/60.f/60.f << " island hours) on the shortest path. " << std::endl;

        return true;
    }


    void cPlanner::UpdateCost(tNode *i_sNode) const
    {
        tNode *psParent = i_sNode->psParent;
        if (nullptr != psParent) {
            /// Rover's normal speed is 1 cell per island second
            float fV = m_poRover->Velocity();
            float fDeltaS = i_sNode->sAction.fCost * m_poRover->StepSize();

            /// Add action (step) cost, which is given in island seconds
            float fStepCost = fDeltaS / fV;


            /// If the rover is going up or down hill, calculate the acceleration on the inclined plane
            /// Calculate current gradient in step direction
            float nDeltaHeight =
                    (m_oMap.Elevation(i_sNode->sLocation.nX, i_sNode->sLocation.nY) -
                     m_oMap.Elevation(psParent->sLocation.nX, psParent->sLocation.nY));

            float fAlpha = atan(nDeltaHeight / static_cast<float>(m_poRover->StepSize()));
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


            float fTime = fStepCost + fHeightCost;

            i_sNode->g = psParent->g + fTime;

        }

    }

    void cPlanner::Plot()
    {

        tNode *sNode = m_oFrontier.pop();
        TraversePath(sNode);


        std::ofstream of("pic.bmp", std::ofstream::binary);
        visualizer::writeBMP(
                of,
                &m_oMap.m_poElevation[0],
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

                    if (visualizer::path(x, y, &m_oMap.m_poOverrides[0]))
                    {
                        return uint8_t(visualizer::IPV_PATH);
                    }

                    // Signifies water
                    if ((m_oMap.m_poOverrides[y * IMAGE_DIM + x] & (OF_WATER_BASIN | OF_RIVER_MARSH)) ||
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


    void cPlanner::TraversePath(tNode *i_psNode) const
    {
        //uint32_t nIslandSeconds = 0; // TODO island seconds calculation
        uint32_t x, y;
        /// Check if the current node is the start node, which has no parent and is therefore set to NULL
        while (nullptr != i_psNode->psParent) {
            x = i_psNode->sLocation.nX;
            y = i_psNode->sLocation.nY;


            /// Calculate Island seconds (ds = v0 * t + 1/2 * a * t^2
            //float v0 = i_psNode->sAction.fCost;
            //nIslandSeconds += v0; // TODO fix island seconds calculation; must be outside of this loop

            /// Store path in overrides
            m_oMap.SetOverrides(x, y, 0x01);

            /// Move towards the start
            i_psNode = i_psNode->psParent;
        }

        //return nIslandSeconds;

    }

    cPlanner::~cPlanner() {

    }


}
