//
// Created by Franz Pucher on 2019-05-18.
//

#ifndef BACHELOR_PLANNERINTERFACE_H
#define BACHELOR_PLANNERINTERFACE_H

#include "graph.h"
#include "node.h"

#include <vector>

namespace planner {

    /// Forward declaration of interface planner::cRoverInterface.
    template <size_t Directions>
    class cRoverInterface;


    ///\brief cPlannerInterface is an abstract interface which can be implemented by concrete planner classes.
    template <size_t Directions>
    class cPlannerInterface {

    public:

        ///\brief The constructor of the interface which initializes its members m_poRover and m_oMap.
        explicit cPlannerInterface(cRoverInterface<Directions> *i_poRover, cGraph &i_oMap) : m_poRover(i_poRover), m_oMap(i_oMap) {};

        ///\brief Virtual abstract method of the base interface, which must be implemented to perform a search algorithm.
        ///\returns The cost to move from start to goal if it was found. Otherwise -1 is returned.
        virtual float Plan() = 0;

        ///\brief Virtual abstract method of the base interface, which must be implemented to output the best found path.
        ///\details The nodes in the best path contain a field tNode::psParent which makes it possible to move back to
        ///         the start node, which has its parent pointer set to nullptr.
        ///         This method is also used in planner::cPlanner::Plot() to output intermediate paths on an output image.
        ///\param[in] The node which on which we walk back. Usually the goal node.
        virtual void TraversePath(tNode *i_psNode) const = 0;


        ///\brief Test if two nodes are the same, which means the goal is reached.
        ///\details Defined pure virtual which means it must be implemented by the subclasses the inherit from this interface.
        virtual bool GoalTest(const tNode *i_sFirst, const tNode *i_sSecond) const = 0;

        ///\brief Given the action i_sAction and the parent node i_sParent a new node of type tNode is created.
        ///\details Defined pure virtual which means it must be implemented by the subclasses the inherit from this interface.
        virtual tNode* Child(tNode *i_sParent, const tAction &i_sAction) const = 0;


        virtual ~cPlannerInterface() {
            if (nullptr != m_poRover)
                delete m_poRover;
            m_poRover = nullptr;
        }


    protected:
        ///\brief Reference pointer to the interface of the rover class planner::cRoverInterface<Directions>.
        cRoverInterface<Directions> *m_poRover;

        ///\brief Provides the subclasses with important map information such as terrain and elevation maps.
        cGraph &m_oMap;


    };


}

#endif //BACHELOR_PLANNERINTERFACE_H
