// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "DextraRawControlBoard.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue DextraRawControlBoard::getAvailableControlModesRaw(int j, std::vector<yarp::dev::SelectableControlModeEnum> & avail)
{
    CHECK_JOINT(j);
    avail = {yarp::dev::SelectableControlModeEnum::VOCAB_CM_POSITION_DIRECT};
    return yarp::dev::ReturnValue_ok;
}
#endif

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue DextraRawControlBoard::getControlModeRaw(int j, yarp::dev::ControlModeEnum & mode)
#else
bool DextraRawControlBoard::getControlModeRaw(int j, int * mode)
#endif
{
    CHECK_JOINT(j);
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    mode = yarp::dev::ControlModeEnum::VOCAB_CM_POSITION;
    return yarp::dev::ReturnValue_ok;
#else
    *mode = VOCAB_CM_POSITION;
    return true;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue DextraRawControlBoard::getControlModesRaw(std::vector<yarp::dev::ControlModeEnum> & modes)
#else
bool DextraRawControlBoard::getControlModesRaw(int * modes)
#endif
{
    bool ok = true;

    for (int j = 0; j < Synapse::DATA_POINTS; j++)
    {
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
        ok &= getControlModeRaw(j, modes[j]);
#else
        ok &= getControlModeRaw(j, &modes[j]);
#endif
    }

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return ok ? yarp::dev::ReturnValue_ok : yarp::dev::ReturnValue_error_method_failed;
#else
    return ok;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue DextraRawControlBoard::getControlModesRaw(const std::vector<int> & joints, std::vector<yarp::dev::ControlModeEnum> & modes)
#else
bool DextraRawControlBoard::getControlModesRaw(int n_joint, const int * joints, int * modes)
#endif
{
    bool ok = true;

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    for (int i = 0; i < joints.size(); i++)
#else
    for (int i = 0; i < n_joint; i++)
#endif
    {
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
        ok &= getControlModeRaw(joints[i], modes[i]);
#else
        ok &= getControlModeRaw(joints[i], &modes[i]);
#endif
    }

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return ok ? yarp::dev::ReturnValue_ok : yarp::dev::ReturnValue_error_method_failed;
#else
    return ok;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue DextraRawControlBoard::setControlModeRaw(int j, yarp::dev::SelectableControlModeEnum mode)
#else
bool DextraRawControlBoard::setControlModeRaw(int j, int mode)
#endif
{
    CHECK_JOINT(j);
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue_error_method_failed; // don't allow any control modes other than position direct, for now
#else
    return false; // don't allow any control modes other than position direct, for now
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue DextraRawControlBoard::setControlModesRaw(const std::vector<yarp::dev::SelectableControlModeEnum> & modes)
#else
bool DextraRawControlBoard::setControlModesRaw(int * modes)
#endif
{
    bool ok = true;

    for (int j = 0; j < Synapse::DATA_POINTS; j++)
    {
        ok &= setControlModeRaw(j, modes[j]);
    }

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return ok ? yarp::dev::ReturnValue_ok : yarp::dev::ReturnValue_error_method_failed;
#else
    return ok;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue DextraRawControlBoard::setControlModesRaw(const std::vector<int> & joints, const std::vector<yarp::dev::SelectableControlModeEnum> & modes)
#else
bool DextraRawControlBoard::setControlModesRaw(int n_joint, const int * joints, int * modes)
#endif
{
    bool ok = true;

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    for (int i = 0; i < joints.size(); i++)
#else
    for (int i = 0; i < n_joint; i++)
#endif
    {
        ok &= setControlModeRaw(joints[i], modes[i]);
    }

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return ok ? yarp::dev::ReturnValue_ok : yarp::dev::ReturnValue_error_method_failed;
#else
    return ok;
#endif
}

// -----------------------------------------------------------------------------
