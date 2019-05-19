//
// Created by Franz Pucher on 2019-05-18.
//

#ifndef BACHELOR_PLANNERINTERFACE_H
#define BACHELOR_PLANNERINTERFACE_H

#include <unordered_map>
#include "priority_queue.h"
#include "structs.h"

namespace planner {

/**
 * \brief cPlannerInterface is an interface which servers other planners as blue print
 */
    class cPlannerInterface {

    public:
        cPlannerInterface();



        //virtual void Plan(tLocation &i_oStart, tLocation &i_oGoal) = 0;

    };


}

#endif //BACHELOR_PLANNERINTERFACE_H
