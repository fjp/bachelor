//
// Created by Franz Pucher on 2019-05-18.
//

#ifndef BACHELOR_STRUCTS_H
#define BACHELOR_STRUCTS_H

#include <unordered_map>
#include <vector>

namespace planner {

    struct tGraph {
        std::unordered_map<int, std::vector<int> > edges; ///< Graph data structure of the map implemented as adjacency list.

        std::vector<int> Neighbors(int i_nId) {
            return edges[i_nId];
        }
    };

    struct tLocation {
        int nX, nY;
    };


}

#endif //BACHELOR_STRUCTS_H
