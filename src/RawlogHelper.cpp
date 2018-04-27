#include "RawlogHelper.hpp"

//MRPT redefinition so they work without using mrpt namepase (more general)
#ifndef CLASS_ID_
    #define CLASS_ID_(class_name, space_name) static_cast<const mrpt::utils::TRuntimeClassId*>(&space_name::class_name::class##class_name)
#endif

#ifndef IS_CLASS_
    #define IS_CLASS_( ptrObj, class_name, space_name )  ((ptrObj)->GetRuntimeClass()==CLASS_ID_(class_name, space_name))
#endif

//STL
#include <cmath>

//MRPT
#include <mrpt/system/datetime.h>

//Debug
#include <iostream>

RawlogHelper::RawlogHelper(const ParametersPtr &params)
    : n(0), m(0), last_obs() //last_obs is a null pointer (default constructor)
{
    max_time_diff = params->max_time_diff;
    verbose = params->verbose;

    rawlog.loadFromRawLogFile(params->rawlog_file);
}

bool RawlogHelper::getNextObservation(mrpt::obs::CObservation3DRangeScanPtr &obs)
{
    mrpt::obs::CObservationPtr observation;

    while (!this->hasFinished())
    {
        //1. Get observation from rawlog
        observation = rawlog.getAsObservation(n);
        n++;

        if (verbose)
            std::cout << "RawlogHelper: entry " << n << " (" << rawlog.size()+1 << ")" << std::endl;

        if(!observation || !IS_CLASS_(observation, CObservation3DRangeScan, mrpt::obs))
        {
            if (verbose)
                std::cout << "Skipping rawlog entry " << (n-1) << "... (not valid CObservation3DRangeScan)" << std::endl;

            continue;
        }

        obs = (mrpt::obs::CObservation3DRangeScanPtr) observation;
        last_obs = obs;

        return true;
    }

    return false;
}

bool RawlogHelper::getNextPair(ObservationPair &obs2)
{
    //Handle first call
    if (!last_obs)
        if (!this->getNextObservation(last_obs)) return false;

    do
    {
        obs2.first = last_obs;
        if (!this->getNextObservation(obs2.second)) return false;

        if (verbose)
        {
            std::cout << obs2.first->sensorLabel << ": " << obs2.first->timestamp << std::endl;
            std::cout << obs2.second->sensorLabel << ": " << obs2.second->timestamp << std::endl;
        }

        if (obs2.first->sensorLabel.compare(obs2.second->sensorLabel) != 0)
            m++;

    } while ((obs2.first->sensorLabel.compare(obs2.second->sensorLabel) == 0) ||
             (std::abs(mrpt::system::timeDifference(obs2.first->timestamp, obs2.second->timestamp))) > max_time_diff);
    //abs shouldn't be needed, but it also doesn't harm...

    if (verbose) std::cout << "Synch" << std::endl;

    return true;
}

bool RawlogHelper::hasFinished()
{
    return (n >= rawlog.size());
}
