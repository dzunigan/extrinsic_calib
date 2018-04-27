#ifndef Z_PLANE_MATCHING_HPP
#define Z_PLANE_MATCHING_HPP

//STL
#include <vector>
#include <utility>

#include "ObservationPair.hpp"
#include "Plane.hpp"

typedef std::vector< std::pair<Plane, Plane> > PlaneCorrespondences;

class PlaneMatching
{

public:

    //TODO: params
    //dist: maximum perpendicular distance allowed
    //ang: maximum angle between normals allowed
    PlaneMatching();

    //void getCorrespondences(const ObservationPair &obs2, PlaneCorrespondences &c) const;

protected:

    //params

};

#endif
