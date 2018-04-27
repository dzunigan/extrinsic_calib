#ifndef Z_PLANE_HPP
#define Z_PLANE_HPP

//STL
#include <vector>

//Eigen
#include <Eigen/Dense>

struct Plane
{
    Eigen::Vector3f n;
    float d;

    Plane()
        : n(Eigen::Vector3f::Zero()), d(0.f)
    {   }

    Plane(const Eigen::Vector3f &n, float d)
        : n(n), d(d)
    {    }
};

typedef std::vector<Plane> PlanesList;

#endif
