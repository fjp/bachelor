//
// Created by Franz Pucher on 2019-05-18.
//

#ifndef BACHELOR_PLANNER_H
#define BACHELOR_PLANNER_H


#include "planner_interface.h"

#include "graph.h"

#include <vector>
#include <iostream> // TODO remove

namespace planner {


    class cPlanner : public cPlannerInterface {
    public:
        cPlanner(cRoverInterface *i_oRover, cGraph &i_oMap);

        void Plan() override;


        // Generate a octile distance heuristic Vector
        void GenerateHeuristic();

        std::vector<std::vector<int> > m_mnHeuristic;


    };





}

#endif //BACHELOR_PLANNER_H
