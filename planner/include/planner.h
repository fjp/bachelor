//
// Created by Franz Pucher on 2019-05-18.
//

#ifndef BACHELOR_PLANNER_H
#define BACHELOR_PLANNER_H


#include "planner_interface.h"
#include <unordered_map>

namespace planner {

    template<typename Graph, typename Model, typename Frontier, typename Location>
    class cPlanner : public cPlannerInterface<Graph, Model, Frontier, Location> {

    public:
        cPlanner(Graph &i_oGraph, Model &i_oTransitionModel, Frontier &i_oFrontier, Location &i_oStart, Location &i_oGoal);

        void Plan(Location& i_oStart, Location& i_oGoal,
                  std::unordered_map<Location, Location>& i_oPredecessors,
                  std::unordered_map<Location, double>& i_oPathCost) override;


    };


}

#endif //BACHELOR_PLANNER_H
