//
// Created by Franz Pucher on 2019-05-19.
//

#include "graph.h"

#include "definitions.h"

namespace planner
{

    cGraph::cGraph(std::vector<uint8_t> &i_oElevation,
                   std::vector<uint8_t> &i_oOverrides,
                   int i_nHeight, int i_nWidth) : m_oElevation(i_oElevation), m_oOverrides(i_oOverrides)
    {
        m_mnHeuristic = GenerateHeuristic();
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

    // Generate a Manhattan Heuristic Vector
    std::vector<std::vector<int> > cGraph::GenerateHeuristic()
    {
        std::vector<std::vector<int> > heuristic(m_nHeight, std::vector<int>(m_nWidth));
        int goal[2] = { 60, 50 };
        for (int i = 0; i < heuristic.size(); i++) {
            for (int j = 0; j < heuristic[0].size(); j++) {
                int xd = goal[0] - i;
                int yd = goal[1] - j;
                // Manhattan Distance
                int d = abs(xd) + abs(yd);
                // Euclidian Distance
                // double d = sqrt(xd * xd + yd * yd);
                // Chebyshev distance
                // int d = max(abs(xd), abs(yd));
                heuristic[i][j] = d;
            }
        }
        return heuristic;
    }

}