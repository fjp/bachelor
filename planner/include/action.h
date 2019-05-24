//
// Created by Franz Pucher on 2019-05-19.
//

#ifndef BACHELOR_ACTION_H
#define BACHELOR_ACTION_H


namespace planner
{
    ///\brief Action struct, used to specify the motion direction and the cost it takes to move in that direction.
    struct tAction
    {
        ///\brief Direction of the action
        int32_t nX, nY;
        ///\brief Step cost, which can be different depending on the direction (straight, diagonal)
        float fCost;
    };
}


#endif //BACHELOR_ACTION_H
