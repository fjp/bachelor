//
// Created by fjp on 6/24/19.
//

#ifndef BACHELORTEST_SIMPLE_NODE_H
#define BACHELORTEST_SIMPLE_NODE_H

#include "location.h"

namespace planner {
    ///\brief Extended location struct with identifier used as hash value.
    struct tSimpleLocation : public tLocation {
        tSimpleLocation(int i_nId = 0, int i_nX = 0, int i_nY = 0) : tLocation{i_nX, i_nY}, nId(i_nId) {};
        int nId;

        bool operator<(const tSimpleLocation& i_rhs) const {
            return nId < i_rhs.nId;
        }

        template<typename TLocation>
        bool operator==(const TLocation& i_rhs) const {
            return nX == i_rhs.nX && nY == i_rhs.nY;
        }
    };


    ///\brief Simplified node struct without the a pointer to its parent as in node struct (node.h).
    struct tSimpleNode : public tSimpleLocation {
        tSimpleNode(int i_nId = 0
                , int i_nX = 0
                , int i_nY = 0
                , double i_g = 0.0
                , double i_h = 0.0
                , double i_f = 0.0) : tSimpleLocation{i_nId, i_nX, i_nY}, g(i_g), h(i_h), f(i_f) {};

        ///\brief Step cost to reach this node
        double g;
        
        ///\brief Heuristic value of this node
        double h;
        
        ///\brief Combined cost of step cost and heuristic \f$f = g + h\f$.
        double f;

    };
}

#endif //BACHELORTEST_SIMPLE_NODE_H
