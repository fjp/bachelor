//
// Created by Franz Pucher on 2019-05-19.
//

#ifndef BACHELOR_GRAPH_H
#define BACHELOR_GRAPH_H

#include <stdint.h>

namespace planner {


    ///\brief The map class cMap has pointers to the elevation.data (m_poElevation) and overrides.data (m_poOverrides).
    ///\details This class is used to check for constraints in the map (e.g. water basins, rivers and marsh,
    ///         which cannot be traversed by the robot. The elevation data is used to model the acceleration of the rover.
    class cMap {
    public:
        ///\brief Initializes the members m_poElevation, m_poOverrides and the maps height m_nHeight and width m_nWidth.
        cMap(uint8_t* i_oElevation,
               uint8_t* i_oOverrides,
               int i_nHeight, int i_nWidth);


        ///\brief Get elevation at provided location
        ///\details From the read elevation.data the 8 bit value is returned.
        ///\param[in] i_nX X location of the elevation.
        ///\param[in] i_nY Y location of the elevation.
        ///\return Elevation at (i_nX, i_nY).
        uint8_t Elevation(const uint32_t i_nX, const uint32_t i_nY);

        ///\brief The overrides map describes the constraints, see \enum OverrideFlags in constants.h.
        ///\details From the read overrides.data the 8 bit value is returned.
        ///         \li 1. Rivers and marsh have bit 4 set.
        ///         \li 2. Water basins have bit 6 set.
        ///\param[in] i_nX X location of the elevation.
        ///\param[in] i_nY Y location of the elevation.
        ///\return Elevation at (i_nX, i_nY).
        uint8_t Overrides(const uint32_t i_nX, const uint32_t i_nY);

        bool Water(const uint32_t i_nX, const uint32_t i_nY);

        bool Water(const uint32_t i_nX0, const uint32_t i_nY0, const uint32_t i_nX1, const uint32_t i_nY1);

        ///\brief Getter for the height of the map, stored in m_nHeight.
        ///\return The total height of the map.
        inline uint32_t Height() const { return m_nHeight; };

        ///\brief Getter for the width of the map, stored in m_nWidth.
        ///\return The total width of the map.
        inline uint32_t Width() const { return m_nWidth; };


        ///\brief Sets the provided overrides m_poOverrides 8 bit value i_nValue at the provided location while keeping the old value (or operation).
        ///\param[in] i_nX X location of the overrides m_poOverrides that should be updated.
        ///\param[in] i_nY Y location of the overrides m_poOverrides that should be updated.
        ///\param[in] i_nValue The or operator is applied to this 8 bit mask value and the current overrides value at (i_nX, i_nY).
        void SetOverrides(const uint32_t i_nX, const uint32_t i_nY, uint8_t i_nValue);

    private:
    public: // TODO
        ///\brief Elevation over the ground (0-255).
        uint8_t* m_poElevation;
        ///\brief Overrides is the terrain flag bit mask which act as constraints see \enum OverrideFlags and planner::cMap::Water().
        uint8_t* m_poOverrides;

        ///\brief Stores the height of the map.
        uint32_t m_nHeight;
        ///\brief Stores the width of the map.
        uint32_t m_nWidth;

    };
}


#endif //BACHELOR_GRAPH_H
