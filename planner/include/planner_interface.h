//
// Created by Franz Pucher on 2019-05-18.
//

#ifndef BACHELOR_PLANNERINTERFACE_H
#define BACHELOR_PLANNERINTERFACE_H

#include "graph.h"
#include "node.h"
#include "result.h"

#include <vector>

#include <cstddef>
#include <memory>

#include <iostream>

namespace planner {

    enum tAlgorithm {
        ASTAR,
        ASTAR_OPT,
        ASTAR_CK
    };

    /// Forward declaration of interface planner::cRoverInterface.
    template <size_t Directions>
    class cRoverInterface;


    ///\brief cPlannerInterface is an abstract interface which can be implemented by concrete planner classes.
    template <size_t Directions>
    class cPlannerInterface {

    public:

        ///\brief The constructor of the interface which initializes its members m_poRover and m_oMap.
        explicit cPlannerInterface(std::shared_ptr<cRoverInterface<Directions>> i_poRover, std::shared_ptr<cGraph> i_oMap) :
            m_poRover(i_poRover)
            , m_oMap(i_oMap)
            , m_eAlgorithm(ASTAR) {
            std::cout << "cPlannerInterface" << std::endl;
        };

        ///\brief Virtual abstract method of the base interface, which must be implemented to perform a search algorithm.
        ///\returns The cost to move from start to goal if it was found. Otherwise -1 is returned.
        virtual double Plan() = 0;

        ///\brief Virtual abstract method of the base interface, which must be implemented to output the best found path.
        ///\details The nodes in the best path contain a field tNode::psParent which makes it possible to move back to
        ///         the start node, which has its parent pointer set to nullptr.
        ///         This method is also used in planner::cPlanner::Plot() to output intermediate paths on an output image.
        ///\param[in] The node which on which we walk back. Usually the goal node.
        virtual void TraversePath(std::shared_ptr<tNode> i_psNode) const = 0;


        ///\brief Test if two nodes are the same, which means the goal is reached.
        ///\details Defined pure virtual which means it must be implemented by the subclasses the inherit from this interface.
        virtual bool GoalTest(std::shared_ptr<tNode>& i_sFirst, std::shared_ptr<tNode>& i_sSecond) const = 0;

        ///\brief Given the action i_sAction and the parent node i_sParent a new node of type tNode is created.
        ///\details Defined pure virtual which means it must be implemented by the subclasses the inherit from this interface.
        virtual std::shared_ptr<tNode> Child(std::shared_ptr<tNode> i_sParent, const tAction &i_sAction) const = 0;


        virtual ~cPlannerInterface() {
            std::cout << "~cPlannerInterface" << std::endl;
        }

        ///\brief Setter to specify the algorithm that should be used to plan a path.
        void SetAlgorithm(const tAlgorithm i_eAlgorithm = ASTAR) { m_eAlgorithm = i_eAlgorithm; };
        
        ///\brief Getter to obtain the currently set algorithm.
        tAlgorithm Algorithm() { return m_eAlgorithm; };

        ///\brief Getter to obtain result struct member m_sResult that contains information about the found path.
        tResult Result() { return m_sResult; };

    protected:
        ///\brief Select which algorithm should be used to plan a path.
        tAlgorithm m_eAlgorithm;

        ///\brief Information to store planning results such as travelling time and consistency of heuristic.
        tResult m_sResult;

        ///\brief Reference pointer to the interface of the rover class planner::cRoverInterface<Directions>.
        std::shared_ptr<cRoverInterface<Directions>> m_poRover;

        ///\brief Provides the subclasses with important map information such as terrain and elevation maps.
        std::shared_ptr<cGraph> m_oMap;


    };


}

#endif //BACHELOR_PLANNERINTERFACE_H
