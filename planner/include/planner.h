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


    class cPlanner : public cPlannerInterface<8> {
    public:
        cPlanner(cRoverInterface<8> *i_oRover, cGraph &i_oMap, uint8_t i_oStepSize = 5);

        void Plan() override;

        std::vector<std::vector<int> > m_mnHeuristic;

        // Generate a octile distance heuristic Vector
        void GenerateHeuristic();

    private:
        uint8_t m_nStepSize;



    };





}

#endif //BACHELOR_PLANNER_H
