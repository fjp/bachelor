//
// Created by Franz Pucher on 2019-05-19.
//

#ifndef BACHELOR_GRAPH_H
#define BACHELOR_GRAPH_H

#include <stdint.h>

namespace planner {


    class cGraph {
    public:
        cGraph(uint8_t* i_oElevation,
               uint8_t* i_oOverrides,
               int i_nHeight, int i_nWidth);


        uint8_t Elevation(int i_nX, int i_nY);

        // Constraints
        uint8_t Overrides(int i_nX, int i_nY);

        bool Water(int i_nX, int i_nY);

        bool Water(int i_nX0, int i_nY0, int i_nX1, int i_nY1);

        int Height() const;

        int Width() const;


        void SetOverrides(int i_nX, int i_nY, uint8_t value);

    private:
    public: // TODO
        uint8_t* m_oElevation;
        uint8_t* m_oOverrides;

        int m_nHeight;
        int m_nWidth;

    };
}


#endif //BACHELOR_GRAPH_H
