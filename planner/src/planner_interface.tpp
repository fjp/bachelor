//
// Created by Franz Pucher on 2019-05-18.
//

#include "planner_interface.h"

using namespace planner;

template<typename Graph, typename Model, typename Frontier, typename Location>
cPlannerInterface<Graph, Model, Frontier, Location>::cPlannerInterface(
        Graph &i_oGraph,
        Model &i_oTransitionModel,
        Frontier &i_oFrontier,
        Location &i_oStart, Location &i_oGoal) :
        m_oGraph(i_oGraph),
        m_oTransitionModel(i_oTransitionModel),
        m_oFrontier(i_oFrontier),
        m_oStart(i_oStart),
        m_oGoal(i_oGoal) {

        }
