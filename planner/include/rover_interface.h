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
        cRoverInterface() : m_fCostStraight(1.0), m_fCostDiagonal(1.4) // TODO cost
        {

        };

        virtual ~cRoverInterface() {
            if (nullptr != m_poPlanner)
                delete m_poPlanner;
            m_poPlanner = nullptr;
        };

        virtual void InitializePlanner(const uint8_t &i_nStepSize, const uint8_t &i_nVelocity) = 0;


        float m_fCostStraight;
        float m_fCostDiagonal;

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

        cPlannerInterface<Directions> *GetPlanner() const {
            return m_poPlanner;
        }

    protected:
        ///\brief Pointer to an abstract planner interface.
        ///\details An implementation of this interface should initialize m_poPlanner
        cPlannerInterface<Directions> *m_poPlanner;


    private:
        ///\brief Start location (x,y) of the rover
        tLocation m_sStart;

        ///\brief Goal location (x,y) of the rover
        tLocation m_sGoal;

        ///\brief Step size of the rover
        uint8_t m_nStepSize;

        ///\brief Speed of the rover
        uint8_t m_nVelocity;



    };


}


#endif //BACHELOR_ROVER_INTERFACE_H
