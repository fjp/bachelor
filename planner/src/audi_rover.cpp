//
// Created by Franz Pucher on 2019-05-18.
//

#include <cmath>
#include "audi_rover.h"

#include "planner.h"

namespace planner
{

    cAudiRover::cAudiRover(
            uint8_t* i_oElevation,
            uint8_t* i_oOverrides,
            int i_nHeight, int i_nWidth) : cRoverInterface(), m_oElevation(i_oElevation), m_oOverrides(i_oOverrides) {


        m_oMap = new cGraph(m_oElevation, m_oOverrides, i_nHeight, i_nWidth); // TODO make unique and delete


        SetCost(1.0, std::sqrt(2));


        SetStart(0, 0);
        SetGoal(10, 10);
    }


    void cAudiRover::SetStart(int i_nX, int i_nY) {
        m_afStart[0] = i_nX;
        m_afStart[1] = i_nY;

    }

    void cAudiRover::SetGoal(int i_nX, int i_nY) {
        m_afGoal[0] = i_nX;
        m_afGoal[1] = i_nY;
    }

    void cAudiRover::SetCost(double i_nStraight, double i_nDiagonal) {

        // TODO assert input > 0
        m_fCostStraight = i_nStraight;
        m_fCostDiagonal = i_nDiagonal;
    }

    void cAudiRover::Summon() {

        m_oPlanner = new cPlanner(this, *m_oMap); // TODO make unique and delete

        m_oPlanner->Plan();

    }

}


