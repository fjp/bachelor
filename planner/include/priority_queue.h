//
// Created by Franz Pucher on 2019-05-18.
//

#ifndef BACHELOR_PRIORITY_QUEUE_H
#define BACHELOR_PRIORITY_QUEUE_H

#include <queue>

namespace planner
{
    template<typename Node, typename priority_t>
    struct tPriorityQueue {
        typedef std::pair<priority_t, Node> Element;
        std::priority_queue<Element, std::vector<Element>, std::greater<Element>> oNodes;

        inline bool Empty() const {
            return oNodes.empty();
        }

        inline void Put(Node i_oElement, priority_t i_priority) {
            oNodes.emplace(i_priority, i_oElement);
        }

        Node Get() {
            Node oBestNode = oNodes.top().second;
            oNodes.pop();
            return oBestNode;
        }
    };
}

#endif //BACHELOR_PRIORITY_QUEUE_H
