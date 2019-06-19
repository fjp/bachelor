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
#include <cmath>
#include <memory>

#include <iostream>

namespace planner {


    /// Forward declaration of planner interface
    template <size_t Directions>
    class cPlannerInterface;


    template <size_t Directions>
    class cRoverInterface  {


    public:
        cRoverInterface() : m_fCostStraight(1.f), m_fCostDiagonal(sqrt(2.f)), m_nStepSize(1), m_nVelocity(1)
        {
            std::cout << "cRoverInterface" << std::endl;
        };

        virtual ~cRoverInterface() {
            std::cout << "~cRoverInterface" << std::endl;

            std::cout << "     destructor ~cRoverInterface: m_poPlanner.use_count() == " << m_poPlanner.use_count() << std::endl;
        }


        ///\brief
        virtual std::shared_ptr<cPlannerInterface<Directions>> InitializePlanner(const int &i_nStepSize, const int &i_nVelocity) = 0;

        ///\brief Array that contains the number of actions in which the robot can move. It has a template size parameter Directions.
        std::array<tAction, Directions> m_asActions;


        /// Getter and Setter for the private member varialbes

        float CostStraight() const {
            return m_fCostStraight;
        }

        float CostDiagonal() const {
            return m_fCostDiagonal;
        }

        void SetCostStraight(float i_fCostStraight) {
            m_fCostStraight = i_fCostStraight;
        }

        void SetCostDiagonal(float i_fCostDiagonal) {
            m_fCostDiagonal = i_fCostDiagonal;
        }


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

        int StepSize() const {
            return m_nStepSize;
        }

        void SetStepSize(int i_nStepSize) {
            m_nStepSize = i_nStepSize;
        }

        int Velocity() const {
            return m_nVelocity;
        }

        void SetVelocity(int i_nVelocity) {
            m_nVelocity = i_nVelocity;
        }

        std::shared_ptr<cPlannerInterface<Directions>> GetPlanner() const {
            return m_poPlanner;
        }

    protected:
        ///\brief Pointer to an abstract planner interface.
        ///\details An implementation of this interface should initialize m_poPlanner.
        //std::shared_ptr<cPlannerInterface<Directions>> m_poPlanner;
        std::weak_ptr<cPlannerInterface<Directions>> m_poPlanner;


        ///\brief Start location (x,y) of the rover.
        tLocation m_sStart;

        ///\brief Goal location (x,y) of the rover.
        tLocation m_sGoal;

        ///\brief Step size of the rover, varied to get to the goal quicker.
        int m_nStepSize;

        ///\brief Speed of the rover (always set to one, not tested).
        int m_nVelocity;

        ///\brief Cost to move straight.
        float m_fCostStraight;

        ///\brief Cost to move diagonal.
        float m_fCostDiagonal;



    };


}


#endif //BACHELOR_ROVER_INTERFACE_H
