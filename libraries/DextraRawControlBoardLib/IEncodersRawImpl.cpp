// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "DextraRawControlBoard.hpp"

#include <algorithm>

#include <yarp/os/SystemClock.h>

using namespace roboticslab;

// ------------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue DextraRawControlBoard::resetEncoderRaw(int j)
#else
bool DextraRawControlBoard::resetEncoderRaw(int j)
#endif
{
    return setEncoderRaw(j, 0.0);
}

// ------------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue DextraRawControlBoard::resetEncodersRaw()
#else
bool DextraRawControlBoard::resetEncodersRaw()
#endif
{
    Synapse::Setpoints setpoints = {0};
    setSetpoints(setpoints);
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue_ok;
#else
    return true;
#endif
}

// ------------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue DextraRawControlBoard::setEncoderRaw(int j, double val)
#else
bool DextraRawControlBoard::setEncoderRaw(int j, double val)
#endif
{
    CHECK_JOINT(j);
    setSetpoint(j, val);
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue_ok;
#else
    return true;
#endif
}

// ------------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue DextraRawControlBoard::setEncodersRaw(const double * vals)
#else
bool DextraRawControlBoard::setEncodersRaw(const double * vals)
#endif
{
    Synapse::Setpoints setpoints;
    std::copy(vals, vals + Synapse::DATA_POINTS, std::begin(setpoints));
    setSetpoints(setpoints);
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue_ok;
#else
    return true;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue DextraRawControlBoard::getEncoderRaw(int j, double * v)
#else
bool DextraRawControlBoard::getEncoderRaw(int j, double * v)
#endif
{
    CHECK_JOINT(j);
    *v = getSetpoint(j);
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue_ok;
#else
    return true;
#endif
}

// ------------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue DextraRawControlBoard::getEncodersRaw(double * encs)
#else
bool DextraRawControlBoard::getEncodersRaw(double * encs)
#endif
{
    Synapse::Setpoints setpoints;
    getSetpoints(setpoints);
    std::copy(std::cbegin(setpoints), std::cend(setpoints), encs);
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue_ok;
#else
    return true;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue DextraRawControlBoard::getEncoderSpeedRaw(int j, double * sp)
#else
bool DextraRawControlBoard::getEncoderSpeedRaw(int j, double * sp)
#endif
{
    CHECK_JOINT(j);
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue_error_not_implemented_by_device;
#else
    return false;
#endif
}

// ------------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue DextraRawControlBoard::getEncoderSpeedsRaw(double * spds)
#else
bool DextraRawControlBoard::getEncoderSpeedsRaw(double * spds)
#endif
{
    bool ok = true;

    for (int j = 0; j < Synapse::DATA_POINTS; j++)
    {
        ok &= getEncoderSpeedRaw(j, &spds[j]);
    }

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return ok ? yarp::dev::ReturnValue_ok : yarp::dev::ReturnValue_error_method_failed;
#else
    return ok;
#endif
}

// ------------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue DextraRawControlBoard::getEncoderAccelerationRaw(int j, double * accs)
#else
bool DextraRawControlBoard::getEncoderAccelerationRaw(int j, double * accs)
#endif
{
    CHECK_JOINT(j);
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue_error_not_implemented_by_device;
#else
    return false;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue DextraRawControlBoard::getEncoderAccelerationsRaw(double * accs)
#else
bool DextraRawControlBoard::getEncoderAccelerationsRaw(double * accs)
#endif
{
    bool ok = true;

    for (int j = 0; j < Synapse::DATA_POINTS; j++)
    {
        ok &= getEncoderAccelerationRaw(j, &accs[j]);
    }

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return ok ? yarp::dev::ReturnValue_ok : yarp::dev::ReturnValue_error_method_failed;
#else
    return ok;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue DextraRawControlBoard::getEncoderTimedRaw(int j, double * enc, double * time)
#else
bool DextraRawControlBoard::getEncoderTimedRaw(int j, double * enc, double * time)
#endif
{
    CHECK_JOINT(j);
    *time = yarp::os::SystemClock::nowSystem();
    return getEncoderRaw(j, enc);
}

// ------------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue DextraRawControlBoard::getEncodersTimedRaw(double * encs, double * time)
#else
bool DextraRawControlBoard::getEncodersTimedRaw(double * encs, double * time)
#endif
{
    auto now = yarp::os::SystemClock::nowSystem();

    for (int j = 0; j < Synapse::DATA_POINTS; j++)
    {
        time[j] = now;
    }

    return getEncodersRaw(encs);
}

// -----------------------------------------------------------------------------
