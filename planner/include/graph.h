//
// Created by Franz Pucher on 2019-05-19.
//

#ifndef BACHELOR_GRAPH_H
#define BACHELOR_GRAPH_H

#include <vector>

namespace planner {


    class cGraph {
    public:
        cGraph(std::vector<uint8_t> &i_oElevation,
               std::vector<uint8_t> &i_oOverrides,
               int i_nHeight, int i_nWidth);


        uint8_t Elevation(int i_nX, int i_nY);

        // Constraints
        uint8_t Overrides(int i_nX, int i_nY);

        bool Water(int i_nX, int i_nY);


        std::vector<std::vector<int> > m_mnHeuristic;

        int Height() const;

        int Width() const;

    private:
        std::vector<uint8_t> &m_oElevation;
        std::vector<uint8_t> &m_oOverrides;

        int m_nHeight;
        int m_nWidth;


        // Generate a Manhattan Heuristic Vector
        std::vector<std::vector<int> > GenerateHeuristic();

    };
}


#endif //BACHELOR_GRAPH_H
