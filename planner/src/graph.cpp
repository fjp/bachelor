//
// Created by Franz Pucher on 2019-05-19.
//


#include <algorithm>
#include "graph.h"

#include "constants.h"

namespace planner
{

    cGraph::cGraph(uint8_t* i_oElevation,
                   uint8_t* i_oOverrides,
                   int i_nHeight, int i_nWidth)
                   : m_oElevation(i_oElevation),
                   m_oOverrides(i_oOverrides),
                   m_nHeight(i_nHeight), m_nWidth(i_nWidth)

    {

    }

    int planner::cGraph::Height() const {
        return m_nHeight;
    }

    int planner::cGraph::Width() const {
        return m_nWidth;
    }

    bool planner::cGraph::Water(int i_nX, int i_nY) {
        bool bWater = ((Overrides(i_nX, i_nY) & (OF_WATER_BASIN | OF_RIVER_MARSH)) || Elevation(i_nX, i_nY) == 0);
        return bWater;
    }

    bool cGraph::Water(int i_nX0, int i_nY0, int i_nX1, int i_nY1) {
        int dx = i_nX1 - i_nX0;
        int dy = i_nY1 - i_nY0;

        int dxNorm = dx / std::max(std::abs(dx), 1);
        int dyNorm = dy / std::max(std::abs(dy), 1);

        int nSteps = std::max(std::abs(dx), 1) * std::max(std::abs(dy), 1);

        int nX = i_nX0;
        int nY = i_nY0;

        for (int i = 0; i <= nSteps; ++i)
        {
            if (Water(nX, nY))
            {
                return true;
            }

            nX += dxNorm;
            nY += dyNorm;
        }

        return false;
    }

    uint8_t cGraph::Elevation(int i_nX, int i_nY)
    {
        // TODO check bounds
        uint8_t nElevation = m_oElevation[i_nY * m_nWidth + i_nX];
        return nElevation;
    }

    uint8_t cGraph::Overrides(int i_nX, int i_nY)
    {
        // TODO check bounds
        uint8_t nOverrides = m_oOverrides[i_nY * m_nWidth + i_nX];
        return nOverrides;
    }

    void cGraph::SetOverrides(int i_nX, int i_nY, uint8_t value) {
        m_oOverrides[i_nY * m_nWidth + i_nX] |= value;
    }



}