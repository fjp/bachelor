//
// Created by Franz Pucher on 2019-05-20.
//

#ifndef BACHELOR_NODE_H
#define BACHELOR_NODE_H

#include <memory>
#include "action.h"
#include "location.h"

namespace planner {


    ///\brief Node gets expanded while searching for the shortest path.
    ///\details A node has an identifier nId which is defined by its location tLocation (see cPlanner::NodeHash()).
    ///         Each node has an associated action sAction of type tAction which describes how this node was reached
    ///         from its parent node psParent.
    struct tNode {

        ///\brief Default constructor which initializes its members to define a start node.
        ///\details A start node has its parent psParent set to nullptr and the zero action associated.
        tNode() : nId(0), f(0.0), g(0.0), h(0.0), sLocation{0, 0}, psParent(nullptr), sAction{0, 0, 0.0} {}

        ///\brief Allows to set a location and calls the default constructor (c++11)
        tNode(const tLocation &i_sLocation)
        : tNode()
        {
            sLocation = i_sLocation;
        }

        ///\brief Assignment operator of the node struct.
        tNode& operator=(const tNode& i_sNode) {
            nId = i_sNode.nId;
            f = i_sNode.f;
            g = i_sNode.g;
            h = i_sNode.h;
            sLocation = i_sNode.sLocation;
            psParent = i_sNode.psParent;
            sAction = i_sNode.sAction;
            return *this;
        }

        ///\brief Copy constructor of the node struct.
        tNode(const tNode& i_sNode) {
            nId = i_sNode.nId;
            f = i_sNode.f;
            g = i_sNode.g;
            h = i_sNode.h;
            sLocation = i_sNode.sLocation;
            psParent = i_sNode.psParent;
            sAction = i_sNode.sAction;
        }

        ///\brief Node identifier describes its location and updated with cPlanner::NodeHash().
        uint32_t nId;
        
        ///\brief Evaluation function \f$f(n) = g(n) + h(n)\f$
        float f;

        ///\brief Cost to reach this node \f$g(n)\f$, also known as path cost.
        float g;

        ///\brief Heuristic value h(n) of this node.
        float h;

        ///\brief (x,y) location of the node in the grid map cMap.
        tLocation sLocation;

        ///\brief Pointer to the parent of the node, which is required to find the best path by traversing back from the goal node.
        std::shared_ptr<tNode> psParent;

        /// The action that lead to this node
        tAction sAction;

        ///\brief Overloads less than operator to provide hasing for this node.
        ///\details The hash of a node is calculated using cPlanner::NodeHash().
        ///         Required for the path cost map std::map<tNode> oPathCost used in cPlanner::AStar().
        bool operator<(const tNode& i_rhs) const
        {
            return nId < i_rhs.nId;
        }

        ///\brief Overloads greater than operator to provide using node identifier nId.
        bool operator>(const tNode& i_lhs) const
        {
            return i_lhs.nId < nId;
        }


    };
}

#endif //BACHELOR_NODE_H
