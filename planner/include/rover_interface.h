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
        cRoverInterface() : m_fCostStraight(1.0), m_fCostDiagonal(1.4), m_astrMovementArrows{ "E", "NE", "N", "NW", "W", "SW", "S", "SE" },
                            m_mnMovements{
                                    { 1, 0 },   // E
                                    { 1, -1 },  // NE
                                    { 0, -1 },  // N
                                    { -1, -1 }, // NW
                                    { -1, 0 },  // W
                                    { -1, 1 },  // SW
                                    { 0, 1 },   // S
                                    { 1, 1 },   // SE
                            }  {


        };


        double m_fCostStraight;
        double m_fCostDiagonal;

        std::string m_astrMovementArrows[Directions];

        std::array<tAction, Directions> m_asActions;


        std::vector<std::vector<int> > m_mnMovements;

        std::vector<std::vector<int> > m_mnPath;


        tLocation &Start() {
            return m_sStart;
        }

        void SetStart(const tLocation &i_sStart) {
            m_sStart = i_sStart;
        }

        tLocation &Goal() {
            return m_sGoal;
        }

        void SetGoal(const tLocation &i_sGoal) {
            m_sGoal = i_sGoal;
        }

    private:
        /// Start location (x,y) of the rover
        tLocation m_sStart;

        /// Goal location (x,y) of the rover
        tLocation m_sGoal;


    };


}


#endif //BACHELOR_ROVER_INTERFACE_H
