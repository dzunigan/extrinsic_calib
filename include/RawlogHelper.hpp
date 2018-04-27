//STL
#include <string>

#include <cstddef>

//MRPT
#include <mrpt/obs/CRawlog.h>
#include <mrpt/obs/CObservation3DRangeScan.h>

#include "ObservationPair.hpp"
#include "Parameters.hpp"

class RawlogHelper
{

public:

    RawlogHelper(const ParametersPtr &params);

    //Retrieves next rawlog observation and updates last_obs accordingly
    //Returns   true iif rawlog contains a new CObservation3DRangeScan;
    //          false when no more observation can be retrieved (i.e. hasFinished = true)
    bool getNextObservation(mrpt::obs::CObservation3DRangeScanPtr &obs);

    //Retrieves next synch observation pair (two CObservation3DRangeScan taken from different sensors and less time difference than dt)
    //Returns   true iif rawlog contains a new synch observation pair (as defined above)
    //          false when no more synch observation pair can be retrieved
    //Note: assumes rawlog observations are timestamp ordered (taken with rawlog-grabber)
    bool getNextPair(ObservationPair &obs2);

    bool hasFinished();

    int m; //TODO: debug only

protected:

    float max_time_diff;

    bool verbose;

    mrpt::obs::CRawlog rawlog;
    std::size_t n;

    mrpt::obs::CObservation3DRangeScanPtr last_obs;

};
