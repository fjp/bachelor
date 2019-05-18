//
// Created by Franz Pucher on 2019-05-18.
//

#ifndef BACHELOR_PLANNERINTF_H
#define BACHELOR_PLANNERINTF_H

namespace planner {

/**
 * \brief cPlannerInterface is an interface which servers other planners as blue print
 */
    template<typename Graph, typename Model, typename Frontier, typename Location>
    class cPlannerInterface {

    public:
        cPlannerInterface(Graph &i_oGraph,
                Model &i_oTransitionModel,
                Frontier &i_oFrontier,
                Location &i_oStart,
                Location &i_oGoal);


        virtual void Plan(Location &i_oStart, Location &i_oGoal) = 0;

    public:
        Graph &m_oGraph;

        Model &m_oTransitionModel;
        Frontier &m_oFrontier;

        Location m_oStart;
        Location m_oGoal;
    };


}

#endif //BACHELOR_PLANNERINTF_H
