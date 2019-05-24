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

        m_oMap = new cMap(m_oElevation, m_oOverrides, i_nHeight, i_nWidth); // TODO make unique and delete


        // TODO get start values
        //SetStart(0, 0);
        //SetGoal(10, 10);

        /// Define direction and cost of different possible actions
        m_asActions[0] = tAction{ 1, 0, m_fCostStraight };   // E
        m_asActions[1] = tAction{ 1, -1, m_fCostDiagonal };  // NE
        m_asActions[2] = tAction{ 0, -1, m_fCostStraight };  // N
        m_asActions[3] = tAction{ -1, -1, m_fCostDiagonal }; // NW
        m_asActions[4] = tAction{ -1, 0, m_fCostStraight };  // W
        m_asActions[5] = tAction{ -1, 1, m_fCostDiagonal };  // SW
        m_asActions[6] = tAction{ 0, 1, m_fCostStraight };   // S
        m_asActions[7] = tAction{ 1, 1, m_fCostDiagonal };   // SE

    }

    void cAudiRover::Summon(const uint8_t i_nStepSize, const uint8_t i_nVelocity) {

        InitializePlanner(i_nStepSize, i_nVelocity);

        m_poPlanner->Plan();

    }

    void cAudiRover::InitializePlanner(const uint8_t &i_nStepSize, const uint8_t &i_nVelocity) {

        SetStepSize(i_nStepSize);

        SetVelocity(i_nVelocity);

        m_poPlanner = new cPlanner(this, *m_oMap); // TODO make unique and delete

    }

}


