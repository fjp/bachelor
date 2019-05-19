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



        int m_afStart[2];
        int m_afGoal[2];
        double m_fCostStraight;
        double m_fCostDiagonal;

        std::string m_astrMovementArrows[8];

        std::vector<std::vector<int> > m_mnMovements;

        std::vector<std::vector<int> > m_mnPath;


    };


}


#endif //BACHELOR_ROVER_INTERFACE_H
