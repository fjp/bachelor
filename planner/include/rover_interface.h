//
// Created by Franz Pucher on 2019-05-18.
//

#ifndef BACHELOR_ROVER_INTERFACE_H
#define BACHELOR_ROVER_INTERFACE_H

#include <vector>


#include "planner_interface.h"

#include <vector>

#include <string>

namespace planner {


    class cRoverInterface {


    public:
        cRoverInterface();

        //virtual void SetLocation(tLocation i_oLocation) = 0;
        //virtual void SetLocation(uint32_t i_x, uint32_t i_y) = 0;


        int m_afStart[2];
        int m_afGoal[2];
        double m_fCostStraight;
        double m_fCostDiagonal;

        std::string m_astrMovementArrows[4];

        std::vector<std::vector<int> > m_mnMovements;

        std::vector<std::vector<int> > m_mnPath;


    };


}


#endif //BACHELOR_ROVER_INTERFACE_H
