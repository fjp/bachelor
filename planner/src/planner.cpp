//
// Created by Franz Pucher on 2019-05-18.
//

#include "planner.h"
#include "priority_queue.h"
#include "structs.h"

namespace planner {


    template<typename Graph, typename Model, typename Frontier, typename Location>
    cPlanner<Graph, Model, Frontier, Location>::cPlanner(
            Graph &i_oGraph,
            Model &i_oTransitionModel,
            Frontier &i_oFrontier,
            Location &i_oStart, Location &i_oGoal) : cPlannerInterface<Graph, Location, Model, Frontier>(
            i_oGraph,
            i_oTransitionModel,
            i_oFrontier,
            i_oStart, i_oGoal) {

    }

    template<typename Graph, typename Model, typename Frontier, typename Location>
    void cPlanner<Graph, Model, Frontier, Location>::Plan(Location& i_oStart, Location& i_oGoal,
            std::unordered_map<Location, Location>& i_oPredecessors,
            std::unordered_map<Location, double>& i_oPathCost) {

        /// Initialize the frontier using the initial state of the problem
        Frontier oFrontier;
        oFrontier.put(i_oStart);

        i_oPredecessors[i_oStart] = i_oStart;

        i_oPathCost[i_oStart] = 0.0;

        while (!oFrontier.empty()) {
            Location oCurrent = oFrontier.get();

            if (oCurrent == i_oGoal) {
                break;
            }

            for (Location oNext : this->m_oGraph.Neighbors(oCurrent)) {
                double fNewCost = i_oPredecessors[oCurrent] + this->m_oGraph.Cost(oCurrent, oNext);
                if (i_oPathCost.find(oNext) == i_oPathCost.end()
                    || fNewCost < i_oPathCost[oNext]) {
                    i_oPathCost[oNext] = fNewCost;
                    double priority = fNewCost + heuristic(oNext, i_oGoal);
                    oFrontier.put(oNext, priority);
                    i_oPredecessors[oNext] = oCurrent;
                }
            }
        }
    }


}