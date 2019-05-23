//
// Created by Franz Pucher on 2019-05-19.
//

#ifndef BACHELOR_ACTION_H
#define BACHELOR_ACTION_H


namespace planner
{
    struct tAction
    {
        /// Direction of the action TODO use location struct
        int32_t nX, nY;
        /// Step cost, which can be different depending on the direction
        float fCost;
    };
}


#endif //BACHELOR_ACTION_H
