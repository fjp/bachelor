//
// Created by Franz Pucher on 2019-05-18.
//

#ifndef BACHELOR_PLANNER_H
#define BACHELOR_PLANNER_H


#include "planner_interface.h"
#include <unordered_map>

#include "graph.h"

#include "rover_interface.h"

namespace planner {


    class cPlanner : public cPlannerInterface {

    public:
        //cPlanner(cGraph *i_oGraph, tPriorityQueue<tLocation, double> *i_oFrontier);


        void Plan(cGraph &i_oMap, cRoverInterface &i_oRover);

        //void AStar(tLocation &i_oStart, tLocation &i_oGoal, std::unordered_map<tLocation, tLocation> &i_oPredecessors,
        //           std::unordered_map<tLocation, double> &i_oPathCost);

    };


}

#endif //BACHELOR_PLANNER_H
