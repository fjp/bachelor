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
        cPlanner(cRoverInterface<8> *i_oRover, cGraph &i_oMap, uint8_t i_oStepSize = 4);

        void Plan() override;
        void PlanClean();



        /// \brief Generate distance heuristic vector member, which is inherited from the cPlannerInterface.
        /// \details This implementation calculates the octile distance heuristic because the robot can move in
        ///          eight directions. Other possible gird map heuristics are Manhatten, Chebyshev and Euclidean.
        void GenerateHeuristic();

    private:
        uint8_t m_nStepSize;
        uint8_t m_nMaxGradient; ///< Maximum gradient of the map elevation.

        std::vector<std::vector<int> > m_mnHeuristic;
    public:
        const int &Heuristic(const uint i_nX, const uint i_nY) const;
        const int &Heuristic(const tLocation i_sLocation) const;

        //bool GoalTest(const tNode &i_sFirst, const tNode &i_sSecond) const override;
        //tNode Child(tNode &i_sParent, const tAction &i_sAction) override;

        bool WithinMap(const tLocation &i_sLocation) const;

        bool GoalTest(const tNode &i_sFirst, const tNode &i_sSecond) const override;

        tNode Child(tNode &i_sParent, const tAction &i_sAction) override;

    private:

        int GradX(int i_nX, int i_nY);
        int GradY(int i_nX, int i_nY);
        
        int NodeHash(const tNode &i_sNode);

    };





}

#endif //BACHELOR_PLANNER_H
