// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "DextraRawControlBoard.hpp"

#include <yarp/os/LogStream.h>

#include "LogComponent.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue DextraRawControlBoard::setPosLimitsRaw(int axis, double min, double max)
#else
bool DextraRawControlBoard::setLimitsRaw(int axis, double min, double max)
#endif
{
    CHECK_JOINT(axis);
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue_error_not_implemented_by_device;
#else
    return false;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue DextraRawControlBoard::getPosLimitsRaw(int axis, double * min, double * max)
#else
bool DextraRawControlBoard::getLimitsRaw(int axis, double * min, double * max)
#endif
{
    CHECK_JOINT(axis);

    const auto & [_min, _max] = Synapse::LIMITS[axis];
    *min = _min;
    *max = _max;

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue_ok;
#else
    return true;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue DextraRawControlBoard::setVelLimitsRaw(int axis, double min, double max)
#else
bool DextraRawControlBoard::setVelLimitsRaw(int axis, double min, double max)
#endif
{
    yCIWarning(DEXTRA, id()) << "setVelLimitsRaw() not supported";
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue_error_not_implemented_by_device;
#else
    return false;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue DextraRawControlBoard::getVelLimitsRaw(int axis, double * min, double * max)
#else
bool DextraRawControlBoard::getVelLimitsRaw(int axis, double * min, double * max)
#endif
{
    yCIWarning(DEXTRA, id()) << "getVelLimitsRaw() not supported";
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue_error_not_implemented_by_device;
#else
    return false;
#endif
}

// -----------------------------------------------------------------------------
