//
// Created by Franz Pucher on 2019-05-21.
//

#ifndef BACHELOR_PRIORITY_QUEUE_H
#define BACHELOR_PRIORITY_QUEUE_H


#include <queue>
#include <tuple>
#include <algorithm>

namespace planner {


    template<typename T, typename priority_t>
    struct PriorityQueue {
        typedef std::pair<priority_t, T> PQElement;
        std::priority_queue<PQElement, std::vector<PQElement>,
                std::greater<PQElement>> elements;

        inline bool empty() const {
            return elements.empty();
        }

        inline void put(T item, priority_t priority) {
            elements.emplace(priority, item);
        }

        T get() {
            T best_item = elements.top().second;
            elements.pop();
            return best_item;
        }
    };
}

#endif //BACHELOR_PRIORITY_QUEUE_H
