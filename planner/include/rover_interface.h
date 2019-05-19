//
// Created by Franz Pucher on 2019-05-18.
//

#ifndef BACHELOR_ROVER_INTERFACE_H
#define BACHELOR_ROVER_INTERFACE_H

#include <vector>
#include <array>

#include <string>

namespace planner {


    /// Forward declaration of planner interface
    template <size_t Directions>
    class cPlannerInterface;


    template <size_t Directions>
    class cRoverInterface  {


    public:
        cRoverInterface() : m_astrMovementArrows{ "E", "NE", "N", "NW", "W", "SW", "S", "SE" },
                            m_mnMovements{
                                    { 1, 0 },   // E
                                    { 1, -1 },  // NE
                                    { 0, -1 },  // N
                                    { -1, -1 }, // NW
                                    { -1, 0 },  // W
                                    { -1, 1 },  // SW
                                    { 0, 1 },   // S
                                    { 1, 1 },   // SE
                            } {

        };



        int m_afStart[2];
        int m_afGoal[2];
        double m_fCostStraight;
        double m_fCostDiagonal;

        std::string m_astrMovementArrows[Directions];


        std::vector<std::vector<int> > m_mnMovements;

        std::vector<std::vector<int> > m_mnPath;


    };


}


#endif //BACHELOR_ROVER_INTERFACE_H
