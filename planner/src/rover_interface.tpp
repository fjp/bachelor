//
// Created by Franz Pucher on 2019-05-18.
//


namespace planner
{
    template<typename T>
    cRoverInterface<T>::cRoverInterface(T i_oElevation, T i_oOverrides) : m_oElevation(i_oElevation),
                                                                          m_oOverrides(i_oOverrides) {}

}