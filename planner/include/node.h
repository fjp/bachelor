//
// Created by Franz Pucher on 2019-05-20.
//

#ifndef BACHELOR_NODE_H
#define BACHELOR_NODE_H

#include "action.h"
#include "location.h"

namespace planner {



    struct tNode {

        tNode() : nId(0), f(0.0), g(0.0), h(0.0), sLocation{0, 0}, psParent(nullptr), sAction{0, 0, 0.0} {}
        tNode(const tLocation &i_sLocation)
        : tNode()
        {
            sLocation = i_sLocation;
        }

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

        /// Copy constructor
        tNode(const tNode& i_sNode) {
            nId = i_sNode.nId;
            f = i_sNode.f;
            g = i_sNode.g;
            h = i_sNode.h;
            sLocation = i_sNode.sLocation;
            psParent = i_sNode.psParent;
            sAction = i_sNode.sAction;
        }

        /// Node id
        uint32_t nId;
        
        /// Total cost f(n) = g(n) + h(n)
        double f;

        /// Cost to reach this node g(n)
        double g;

        /// Heuristic value of this node h(n)
        double h;

        /// (x,y) location of the node in the grid
        tLocation sLocation;

        /// Pointer to the parent of the node, which is required to find the best path by traversing back from the goal node
        tNode *psParent;

        /// The action that lead to this node
        tAction sAction;

        bool operator<(const tNode& i_rhs) const
        {
            return nId < i_rhs.nId;
        }

        bool operator>(const tNode& i_lhs) const
        {
            return i_lhs.nId < nId;
        }


    };
}

#endif //BACHELOR_NODE_H
