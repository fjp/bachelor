//
// Created by Franz Pucher on 2019-05-18.
//

#ifndef BACHELOR_PLANNER_H
#define BACHELOR_PLANNER_H


#include "planner_interface.h"
#include <unordered_map>

#include "structs.h"
#include "priority_queue.h"

namespace planner {

    typedef tPriorityQueue<tLocation, double> tFrontier;
    class cPlanner : public cPlannerInterface<tGraph, tFrontier, tLocation> {

    public:
        cPlanner(tGraph &i_oGraph, tFrontier &i_oFrontier);

        void AStar(tLocation &i_oStart, tLocation &i_oGoal, std::unordered_map<tLocation, tLocation> &i_oPredecessors,
                   std::unordered_map<tLocation, double> &i_oPathCost) override;


    };


}

#endif //BACHELOR_PLANNER_H
