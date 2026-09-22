// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "DextraRawControlBoard.hpp"

using namespace roboticslab;

// ----------------------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue DextraRawControlBoard::setPositionRaw(int j, double ref)
#else
bool DextraRawControlBoard::setPositionRaw(int j, double ref)
#endif
{
    return positionMoveRaw(j, ref);
}

// ----------------------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue DextraRawControlBoard::setPositionsRaw(const double * refs)
#else
bool DextraRawControlBoard::setPositionsRaw(const double * refs)
#endif
{
    return positionMoveRaw(refs);
}

// ----------------------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue DextraRawControlBoard::setPositionsRaw(int n_joint, const int * joints, const double * refs)
#else
bool DextraRawControlBoard::setPositionsRaw(int n_joint, const int * joints, const double * refs)
#endif
{
    return positionMoveRaw(n_joint, joints, refs);
}

// ----------------------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue DextraRawControlBoard::getRefPositionRaw(int j, double * ref)
#else
bool DextraRawControlBoard::getRefPositionRaw(int j, double * ref)
#endif
{
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue_error_not_implemented_by_device;
#else
    return false;
#endif
}

// ----------------------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue DextraRawControlBoard::getRefPositionsRaw(double * refs)
#else
bool DextraRawControlBoard::getRefPositionsRaw(double * refs)
#endif
{
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue_error_not_implemented_by_device;
#else
    return false;
#endif
}

// ----------------------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue DextraRawControlBoard::getRefPositionsRaw(int n_joint, const int * joints, double * refs)
#else
bool DextraRawControlBoard::getRefPositionsRaw(int n_joint, const int * joints, double * refs)
#endif
{
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue_error_not_implemented_by_device;
#else
    return false;
#endif
}

// ----------------------------------------------------------------------------------------
