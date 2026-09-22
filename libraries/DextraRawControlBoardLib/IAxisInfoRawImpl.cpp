// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "DextraRawControlBoard.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue DextraRawControlBoard::getAxisNameRaw(int axis, std::string & name)
#else
bool DextraRawControlBoard::getAxisNameRaw(int axis, std::string & name)
#endif
{
    CHECK_JOINT(axis);
    name = axisPrefix + Synapse::LABELS[axis];
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue_ok;
#else
    return true;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue DextraRawControlBoard::getJointTypeRaw(int axis, yarp::dev::JointTypeEnum & type)
#else
bool DextraRawControlBoard::getJointTypeRaw(int axis, yarp::dev::JointTypeEnum & type)
#endif
{
    CHECK_JOINT(axis);
    type = yarp::dev::JointTypeEnum::VOCAB_JOINTTYPE_REVOLUTE;
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue_ok;
#else
    return true;
#endif
}

// -----------------------------------------------------------------------------
