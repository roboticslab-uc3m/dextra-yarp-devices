// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "DextraRawControlBoard.hpp"

#include <cstring>

#include <algorithm>

using namespace roboticslab;

// -----------------------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue DextraRawControlBoard::getAxes(std::size_t & ax)
{
    ax = Synapse::DATA_POINTS;
    return yarp::dev::ReturnValue_ok;
}
#else
bool DextraRawControlBoard::getAxes(int * ax)
{
    *ax = Synapse::DATA_POINTS;
    return true;
}
#endif

// -----------------------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue DextraRawControlBoard::positionMoveRaw(int j, double ref)
#else
bool DextraRawControlBoard::positionMoveRaw(int j, double ref)
#endif
{
    CHECK_JOINT(j);

    Synapse::Setpoints setpoints;
    getSetpoints(setpoints);
    setpoints[j] = ref;

    if (!synapse->writeSetpointList(setpoints))
    {
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
        return yarp::dev::ReturnValue_error_method_failed;
#else
        return false;
#endif
    }

    setSetpoint(j, ref);
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue_ok;
#else
    return true;
#endif
}

// -----------------------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue DextraRawControlBoard::positionMoveRaw(const double * refs)
#else
bool DextraRawControlBoard::positionMoveRaw(const double * refs)
#endif
{
    Synapse::Setpoints setpoints;
    std::copy(refs, refs + Synapse::DATA_POINTS, std::begin(setpoints));

    if (!synapse->writeSetpointList(setpoints))
    {
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
        return yarp::dev::ReturnValue_error_method_failed;
#else
        return false;
#endif
    }

    setSetpoints(setpoints);
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue_ok;
#else
    return true;
#endif
}

// -----------------------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue DextraRawControlBoard::positionMoveRaw(int n_joint, const int * joints, const double * refs)
#else
bool DextraRawControlBoard::positionMoveRaw(int n_joint, const int * joints, const double * refs)
#endif
{
    double encs[Synapse::DATA_POINTS];

    if (!getEncodersRaw(encs))
    {
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
        return yarp::dev::ReturnValue_error_method_failed;
#else
        return false;
#endif
    }

    for (int i = 0; i < n_joint; i++)
    {
        encs[joints[i]] = refs[i];
    }

    return positionMoveRaw(encs);
}

// -----------------------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue DextraRawControlBoard::relativeMoveRaw(int j, double delta)
#else
bool DextraRawControlBoard::relativeMoveRaw(int j, double delta)
#endif
{
    CHECK_JOINT(j);

    double ref;

    if (!getEncoderRaw(j, &ref))
    {
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
        return yarp::dev::ReturnValue_error_method_failed;
#else
        return false;
#endif
    }

    return positionMoveRaw(j, ref + delta);
}

// -----------------------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue DextraRawControlBoard::relativeMoveRaw(const double * deltas)
#else
bool DextraRawControlBoard::relativeMoveRaw(const double * deltas)
#endif
{
    double encs[Synapse::DATA_POINTS];

    if (!getEncodersRaw(encs))
    {
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
        return yarp::dev::ReturnValue_error_method_failed;
#else
        return false;
#endif
    }

    for (int j = 0; j < Synapse::DATA_POINTS; j++)
    {
        encs[j] += deltas[j];
    }

    return positionMoveRaw(encs);
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue DextraRawControlBoard::relativeMoveRaw(int n_joint, const int * joints, const double * deltas)
#else
bool DextraRawControlBoard::relativeMoveRaw(int n_joint, const int * joints, const double * deltas)
#endif
{
    double encs[Synapse::DATA_POINTS];

    if (!getEncodersRaw(encs))
    {
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
        return yarp::dev::ReturnValue_error_method_failed;
#else
        return false;
#endif
    }

    for (int i = 0; i < n_joint; i++)
    {
        encs[joints[i]] += deltas[i];
    }

    return positionMoveRaw(encs);
}

// -----------------------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue DextraRawControlBoard::checkMotionDoneRaw(int j, bool & flag)
#else
bool DextraRawControlBoard::checkMotionDoneRaw(int j, bool * flag)
#endif
{
    CHECK_JOINT(j);
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    flag = true;
    return yarp::dev::ReturnValue_ok;
#else
    *flag = true;
    return true;
#endif
}

// -----------------------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue DextraRawControlBoard::checkMotionDoneRaw(bool & flag)
#else
bool DextraRawControlBoard::checkMotionDoneRaw(bool * flag)
#endif
{
    bool ok = true;

    for (int j = 0; j < Synapse::DATA_POINTS; j++)
    {
        bool localFlag;
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
        ok &= checkMotionDoneRaw(j, localFlag);
        flag &= localFlag;
#else
        ok &= checkMotionDoneRaw(j, &localFlag);
        *flag &= localFlag;
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
yarp::dev::ReturnValue DextraRawControlBoard::checkMotionDoneRaw(const std::vector<int> & joints, bool & flag)
#else
bool DextraRawControlBoard::checkMotionDoneRaw(int n_joint, const int * joints, bool * flag)
#endif
{
    bool ok = true;

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    for (int i = 0; i < joints.size(); i++)
#else
    for (int i = 0; i < n_joint; i++)
#endif
    {
        bool localFlag;
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
        ok &= checkMotionDoneRaw(joints[i], localFlag);
        flag &= localFlag;
#else
        ok &= checkMotionDoneRaw(joints[i], &localFlag);
        *flag &= localFlag;
#endif
    }

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return ok ? yarp::dev::ReturnValue_ok : yarp::dev::ReturnValue_error_method_failed;
#else
    return ok;
#endif
}

// -----------------------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue DextraRawControlBoard::setTrajSpeedRaw(int j, double sp)
#else
bool DextraRawControlBoard::setRefSpeedRaw(int j, double sp)
#endif
{
    CHECK_JOINT(j);
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue_error_not_implemented_by_device;
#else
    return false;
#endif
}

// -----------------------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue DextraRawControlBoard::setTrajSpeedsRaw(const double * spds)
#else
bool DextraRawControlBoard::setRefSpeedsRaw(const double * spds)
#endif
{
    bool ok = true;

    for (int j = 0; j < Synapse::DATA_POINTS; j++)
    {
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
        ok &= setTrajSpeedRaw(j, spds[j]);
#else
        ok &= setRefSpeedRaw(j, spds[j]);
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
yarp::dev::ReturnValue DextraRawControlBoard::setTrajSpeedsRaw(int n_joint, const int * joints, const double * spds)
#else
bool DextraRawControlBoard::setRefSpeedsRaw(int n_joint, const int * joints, const double * spds)
#endif
{
    bool ok = true;

    for (int i = 0; i < n_joint; i++)
    {
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
        ok &= setTrajSpeedRaw(joints[i], spds[i]);
#else
        ok &= setRefSpeedRaw(joints[i], spds[i]);
#endif
    }

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return ok ? yarp::dev::ReturnValue_ok : yarp::dev::ReturnValue_error_method_failed;
#else
    return ok;
#endif
}

// -----------------------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue DextraRawControlBoard::setTrajAccelerationRaw(int j, double acc)
#else
bool DextraRawControlBoard::setRefAccelerationRaw(int j, double acc)
#endif
{
    CHECK_JOINT(j);
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue_error_not_implemented_by_device;
#else
    return false;
#endif
}

// -----------------------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue DextraRawControlBoard::setTrajAccelerationsRaw(const double * accs)
#else
bool DextraRawControlBoard::setRefAccelerationsRaw(const double * accs)
#endif
{
    bool ok = true;

    for (int j = 0; j < Synapse::DATA_POINTS; j++)
    {
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
        ok &= setTrajAccelerationRaw(j, accs[j]);
#else
        ok &= setRefAccelerationRaw(j, accs[j]);
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
yarp::dev::ReturnValue DextraRawControlBoard::setTrajAccelerationsRaw(int n_joint, const int * joints, const double * accs)
#else
bool DextraRawControlBoard::setRefAccelerationsRaw(int n_joint, const int * joints, const double * accs)
#endif
{
    bool ok = true;

    for (int i = 0; i < n_joint; i++)
    {
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
        ok &= setTrajAccelerationRaw(joints[i], accs[i]);
#else
        ok &= setRefAccelerationRaw(joints[i], accs[i]);
#endif
    }

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return ok ? yarp::dev::ReturnValue_ok : yarp::dev::ReturnValue_error_method_failed;
#else
    return ok;
#endif
}

// -----------------------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue DextraRawControlBoard::getTrajSpeedRaw(int j, double * ref)
#else
bool DextraRawControlBoard::getRefSpeedRaw(int j, double * ref)
#endif
{
    CHECK_JOINT(j);
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue_error_not_implemented_by_device;
#else
    return false;
#endif
}

// -----------------------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue DextraRawControlBoard::getTrajSpeedsRaw(double * spds)
#else
bool DextraRawControlBoard::getRefSpeedsRaw(double * spds)
#endif
{
    bool ok = true;

    for (int j = 0; j < Synapse::DATA_POINTS; j++)
    {
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
        ok &= getTrajSpeedRaw(j, &spds[j]);
#else
        ok &= getRefSpeedRaw(j, &spds[j]);
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
yarp::dev::ReturnValue DextraRawControlBoard::getTrajSpeedsRaw(int n_joint, const int * joints, double * spds)
#else
bool DextraRawControlBoard::getRefSpeedsRaw(int n_joint, const int * joints, double * spds)
#endif
{
    bool ok = true;

    for (int i = 0; i < n_joint; i++)
    {
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
        ok &= getTrajSpeedRaw(joints[i], &spds[i]);
#else
        ok &= getRefSpeedRaw(joints[i], &spds[i]);
#endif
    }

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return ok ? yarp::dev::ReturnValue_ok : yarp::dev::ReturnValue_error_method_failed;
#else
    return ok;
#endif
}

// -----------------------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue DextraRawControlBoard::getTrajAccelerationRaw(int j, double * acc)
#else
bool DextraRawControlBoard::getRefAccelerationRaw(int j, double * acc)
#endif
{
    CHECK_JOINT(j);
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue_error_method_failed;
#else
    return false;
#endif
}

// -----------------------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue DextraRawControlBoard::getTrajAccelerationsRaw(double * accs)
#else
bool DextraRawControlBoard::getRefAccelerationsRaw(double * accs)
#endif
{
    bool ok = true;

    for (int j = 0; j < Synapse::DATA_POINTS; j++)
    {
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
        ok &= getTrajAccelerationRaw(j, &accs[j]);
#else
        ok &= getRefAccelerationRaw(j, &accs[j]);
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
yarp::dev::ReturnValue DextraRawControlBoard::getTrajAccelerationsRaw(int n_joint, const int * joints, double * accs)
#else
bool DextraRawControlBoard::getRefAccelerationsRaw(int n_joint, const int * joints, double * accs)
#endif
{
    bool ok = true;

    for (int i = 0; i < n_joint; i++)
    {
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
        ok &= getTrajAccelerationRaw(joints[i], &accs[i]);
#else
        ok &= getRefAccelerationRaw(joints[i], &accs[i]);
#endif
    }

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return ok ? yarp::dev::ReturnValue_ok : yarp::dev::ReturnValue_error_method_failed;
#else
    return ok;
#endif
}

// -----------------------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue DextraRawControlBoard::stopRaw(int j)
#else
bool DextraRawControlBoard::stopRaw(int j)
#endif
{
    CHECK_JOINT(j);
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue_error_not_implemented_by_device;
#else
    return false;
#endif
}

// -----------------------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue DextraRawControlBoard::stopRaw()
#else
bool DextraRawControlBoard::stopRaw()
#endif
{
    bool ok = true;

    for (int j = 0; j < Synapse::DATA_POINTS; j++)
    {
        ok &= stopRaw(j);
    }

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return ok ? yarp::dev::ReturnValue_ok : yarp::dev::ReturnValue_error_method_failed;
#else
    return ok;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue DextraRawControlBoard::stopRaw(int n_joint, const int * joints)
#else
bool DextraRawControlBoard::stopRaw(int n_joint, const int * joints)
#endif
{
    bool ok = true;

    for (int i = 0; i < n_joint; i++)
    {
        ok &= stopRaw(joints[i]);
    }

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return ok ? yarp::dev::ReturnValue_ok : yarp::dev::ReturnValue_error_method_failed;
#else
    return ok;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue DextraRawControlBoard::getTargetPositionRaw(int joint, double * ref)
#else
bool DextraRawControlBoard::getTargetPositionRaw(int joint, double * ref)
#endif
{
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue_error_not_implemented_by_device;
#else
    return false;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue DextraRawControlBoard::getTargetPositionsRaw(double * refs)
#else
bool DextraRawControlBoard::getTargetPositionsRaw(double * refs)
#endif
{
    bool ok = true;

    for (int j = 0; j < Synapse::DATA_POINTS; j++)
    {
        ok &= getTargetPositionRaw(j, &refs[j]);
    }

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return ok ? yarp::dev::ReturnValue_ok : yarp::dev::ReturnValue_error_method_failed;
#else
    return ok;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue DextraRawControlBoard::getTargetPositionsRaw(int n_joint, const int * joints, double * refs)
#else
bool DextraRawControlBoard::getTargetPositionsRaw(int n_joint, const int * joints, double * refs)
#endif
{
    bool ok = true;

    for (int i = 0; i < n_joint; i++)
    {
        ok &= getTargetPositionRaw(joints[i], &refs[i]);
    }

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return ok ? yarp::dev::ReturnValue_ok : yarp::dev::ReturnValue_error_method_failed;
#else
    return ok;
#endif
}

// -----------------------------------------------------------------------------
