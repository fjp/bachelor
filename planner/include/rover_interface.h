//
// Created by Franz Pucher on 2019-05-18.
//

#ifndef BACHELOR_ROVER_INTERFACE_H
#define BACHELOR_ROVER_INTERFACE_H

#include <vector>
#include <array>

#include <string>

#include "location.h"
#include "action.h"

namespace planner {


    /// Forward declaration of planner interface
    template <size_t Directions>
    class cPlannerInterface;


    template <size_t Directions>
    class cRoverInterface  {


    public:
        cRoverInterface() : m_fCostStraight(1.0), m_fCostDiagonal(1.4), m_astrMovementArrows{ "E", "NE", "N", "NW", "W", "SW", "S", "SE" }
        {


        };


        double m_fCostStraight;
        double m_fCostDiagonal;

        std::string m_astrMovementArrows[Directions];

        std::array<tAction, Directions> m_asActions;


        inline tLocation &Start() {
            return m_sStart;
        }

        void SetStart(const tLocation &i_sStart) {
            m_sStart = i_sStart;
        }

        inline tLocation &Goal() {
            return m_sGoal;
        }

        void SetGoal(const tLocation &i_sGoal) {
            m_sGoal = i_sGoal;
        }

        uint8_t StepSize() const {
            return m_nStepSize;
        }

        void SetStepSize(uint8_t i_nStepSize) {
            m_nStepSize = i_nStepSize;
        }

        uint8_t Velocity() const {
            return m_nVelocity;
        }

        void SetVelocity(uint8_t i_nVelocity) {
            m_nVelocity = i_nVelocity;
        }

    private:
        /// Start location (x,y) of the rover
        tLocation m_sStart;

        /// Goal location (x,y) of the rover
        tLocation m_sGoal;

        /// Step size of the rover
        uint8_t m_nStepSize;

        /// Speed of the rover
        uint8_t m_nVelocity;



    };


}


#endif //BACHELOR_ROVER_INTERFACE_H
