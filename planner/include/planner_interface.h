//
// Created by Franz Pucher on 2019-05-18.
//

#ifndef BACHELOR_PLANNERINTERFACE_H
#define BACHELOR_PLANNERINTERFACE_H

#include "graph.h"

#include <vector>

namespace planner {

    template <size_t Directions>
    class cRoverInterface;

/**
 * \brief cPlannerInterface is an interface which servers other planners as blue print
 */
    template <size_t Directions>
    class cPlannerInterface {

    public:

        explicit cPlannerInterface(cRoverInterface<Directions> *i_oRover, cGraph &i_oMap) : m_oRover(i_oRover), m_oMap(i_oMap) {};

        virtual void Plan() = 0;

    protected:
        cRoverInterface<Directions> *m_oRover;
        cGraph &m_oMap;

        std::vector<std::vector<std::string> > *m_oPolicy; // TODO remove or use

    };


}

#endif //BACHELOR_PLANNERINTERFACE_H
