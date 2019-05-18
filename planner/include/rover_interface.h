//
// Created by Franz Pucher on 2019-05-18.
//

#ifndef BACHELOR_ROVER_INTERFACE_H
#define BACHELOR_ROVER_INTERFACE_H

#include <vector>

namespace planner {


    template<typename TMap, typename TPlanner, typename TLocation>
    class cRoverInterface {

    public:
        cRoverInterface(TMap i_oElevation, TMap i_oOverrides, TPlanner i_oPlanner);

        //virtual void SetLocation(TLocation i_oLocation);
        virtual void SetLocation(TLocation i_x, TLocation i_y) = 0;

    protected:
        TMap m_oElevation;
        TMap m_oOverrides;
        TPlanner m_oPlanner;


    };

    template<typename TMap, typename TPlanner, typename TLocation>
    cRoverInterface<TMap, TPlanner, TLocation>::cRoverInterface(TMap i_oElevation, TMap i_oOverrides, TPlanner i_oPlanner) :
    m_oElevation(i_oElevation),
    m_oOverrides(i_oOverrides),
    m_oPlanner(i_oPlanner) {}


}

//#include "rover_interface.tpp"


#endif //BACHELOR_ROVER_INTERFACE_H
