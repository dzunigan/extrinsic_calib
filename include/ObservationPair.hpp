#ifndef Z_OBSERVATION_PAIR_HPP
#define Z_OBSERVATION_PAIR_HPP

//STL
#include <utility>

//MRPT
#include <mrpt/obs/CObservation3DRangeScan.h>

typedef std::pair<mrpt::obs::CObservation3DRangeScanPtr, mrpt::obs::CObservation3DRangeScanPtr> ObservationPair;

#endif
