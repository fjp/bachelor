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
        cPlanner(cRoverInterface<8> *i_oRover, cGraph &i_oMap, uint8_t i_oStepSize = 3);

        void Plan() override;

        std::vector<std::vector<int> > m_mnHeuristic;

        /// \brief Generate distance heuristic vector member, which is inherited from the cPlannerInterface.
        /// \details This implementation calculates the octile distance heuristic because the robot can move in
        ///          eight directions. Other possible gird map heuristics are Manhatten, Chebyshev and Euclidean.
        void GenerateHeuristic();

    private:
        uint8_t m_nStepSize;
        uint8_t m_nMaxGradient; ///< Maximum gradient of the map elevation.

        int GradX(int i_nX, int i_nY);
        int GradY(int i_nX, int i_nY);

    };





}

#endif //BACHELOR_PLANNER_H
