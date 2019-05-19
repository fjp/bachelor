//
// Created by Franz Pucher on 2019-05-18.
//

#include "audi_rover.h"


namespace planner
{

    cAudiRover::cAudiRover(
            std::vector<uint8_t> &i_oElevation,
            std::vector<uint8_t> &i_oOverrides) : cRoverInterface() {

        m_fCostStraight = 1.0;
        m_fCostDiagonal = 1.4;


        //m_astrMovementArrows[0] = "<";
        //m_astrMovementArrows{ "^", "<", "v", ">" },



        SetCost(1.0, 1.4);

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

    }

}


