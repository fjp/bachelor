//
// Created by Franz Pucher on 2019-05-18.
//

#ifndef BACHELOR_PLANNERINTF_H
#define BACHELOR_PLANNERINTF_H

#include <unordered_map>

namespace planner {

/**
 * \brief cPlannerInterface is an interface which servers other planners as blue print
 */
    template<typename Graph, typename Frontier, typename Location>
    class cPlannerInterface {

    public:
        cPlannerInterface(Graph &i_oGraph,
                Frontier &i_oFrontier);


        virtual void Plan(Location &i_oStart, Location &i_oGoal) = 0;



        virtual void AStar(Location &i_oStart, Location &i_oGoal,
                           std::unordered_map<Location, Location>& i_oPredecessors,
                           std::unordered_map<Location, double>& i_oPathCost);

    public:
        Graph &m_oGraph;

        Frontier &m_oFrontier;

        //Location m_oStart;
        //Location m_oGoal;
    };



    template<typename Graph, typename Frontier, typename Location>
    cPlannerInterface<Graph, Frontier, Location>::cPlannerInterface(
            Graph &i_oGraph,
            Frontier &i_oFrontier) :
            m_oGraph(i_oGraph),
            m_oFrontier(i_oFrontier) {

    }

    template<typename Graph, typename Frontier, typename Location>
    void cPlannerInterface<Graph, Frontier, Location>::AStar(Location &i_oStart, Location &i_oGoal,
                                                             std::unordered_map<Location, Location> &i_oPredecessors,
                                                             std::unordered_map<Location, double> &i_oPathCost) {

    }


}

#endif //BACHELOR_PLANNERINTF_H
