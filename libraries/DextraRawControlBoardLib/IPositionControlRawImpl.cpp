// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "DextraRawControlBoard.hpp"

#include <cstring>

#include <algorithm>

using namespace roboticslab;

// -----------------------------------------------------------------------------------------

bool DextraRawControlBoard::getAxes(int *ax)
{
    *ax = Synapse::DATA_POINTS;
    return true;
}

// -----------------------------------------------------------------------------------------

bool DextraRawControlBoard::positionMoveRaw(int j, double ref)
{
    CHECK_JOINT(j);

    Synapse::Setpoints setpoints;
    getSetpoints(setpoints);
    setpoints[j] = ref;

    if (!synapse->writeSetpointList(setpoints))
    {
        return false;
    }

    setSetpoint(j, ref);
    return true;
}

// -----------------------------------------------------------------------------------------

bool DextraRawControlBoard::positionMoveRaw(const double * refs)
{

    Synapse::Setpoints setpoints;
    std::copy(refs, refs + Synapse::DATA_POINTS, std::begin(setpoints));

    if (!synapse->writeSetpointList(setpoints))
    {
        return false;
    }

    setSetpoints(setpoints);
    return true;
}

// -----------------------------------------------------------------------------------------

bool DextraRawControlBoard::positionMoveRaw(int n_joint, const int * joints, const double * refs)
{

    double encs[Synapse::DATA_POINTS];

    if (!getEncodersRaw(encs))
    {
        return false;
    }

    for (int i = 0; i < n_joint; i++)
    {
        encs[joints[i]] = refs[i];
    }

    return positionMoveRaw(encs);
}

// -----------------------------------------------------------------------------------------

bool DextraRawControlBoard::relativeMoveRaw(int j, double delta)
{
    CHECK_JOINT(j);

    double ref;

    if (!getEncoderRaw(j, &ref))
    {
        return false;
    }

    return positionMoveRaw(j, ref + delta);
}

// -----------------------------------------------------------------------------------------

bool DextraRawControlBoard::relativeMoveRaw(const double * deltas)
{

    double encs[Synapse::DATA_POINTS];

    if (!getEncodersRaw(encs))
    {
        return false;
    }

    for (int j = 0; j < Synapse::DATA_POINTS; j++)
    {
        encs[j] += deltas[j];
    }

    return positionMoveRaw(encs);
}

// -----------------------------------------------------------------------------

bool DextraRawControlBoard::relativeMoveRaw(int n_joint, const int * joints, const double * deltas)
{

    double encs[Synapse::DATA_POINTS];

    if (!getEncodersRaw(encs))
    {
        return false;
    }

    for (int i = 0; i < n_joint; i++)
    {
        encs[joints[i]] += deltas[i];
    }

    return positionMoveRaw(encs);
}

// -----------------------------------------------------------------------------------------

bool DextraRawControlBoard::checkMotionDoneRaw(int j, bool * flag)
{
    CHECK_JOINT(j);
    *flag = true;
    return true;
}

// -----------------------------------------------------------------------------------------

bool DextraRawControlBoard::checkMotionDoneRaw(bool * flag)
{

    bool ok = true;

    for (int j = 0; j < Synapse::DATA_POINTS; j++)
    {
        bool localFlag;
        ok &= checkMotionDoneRaw(j, &localFlag);
        *flag &= localFlag;
    }

    return ok;
}

// -----------------------------------------------------------------------------

bool DextraRawControlBoard::checkMotionDoneRaw(int n_joint, const int * joints, bool * flag)
{

    bool ok = true;

    for (int i = 0; i < n_joint; i++)
    {
        bool localFlag;
        ok &= checkMotionDoneRaw(joints[i], &localFlag);
        *flag &= localFlag;
    }

    return ok;
}

// -----------------------------------------------------------------------------------------

bool DextraRawControlBoard::setRefSpeedRaw(int j, double sp)
{
    CHECK_JOINT(j);
    return false;
}

// -----------------------------------------------------------------------------------------

bool DextraRawControlBoard::setRefSpeedsRaw(const double * spds)
{

    bool ok = true;

    for (int j = 0; j < Synapse::DATA_POINTS; j++)
    {
        ok &= setRefSpeedRaw(j, spds[j]);
    }

    return ok;
}

// -----------------------------------------------------------------------------

bool DextraRawControlBoard::setRefSpeedsRaw(int n_joint, const int * joints, const double * spds)
{

    bool ok = true;

    for (int i = 0; i < n_joint; i++)
    {
        ok &= setRefSpeedRaw(joints[i], spds[i]);
    }

    return ok;
}

// -----------------------------------------------------------------------------------------

bool DextraRawControlBoard::setRefAccelerationRaw(int j, double acc)
{
    CHECK_JOINT(j);
    return false;
}

// -----------------------------------------------------------------------------------------

bool DextraRawControlBoard::setRefAccelerationsRaw(const double * accs)
{

    bool ok = true;

    for (int j = 0; j < Synapse::DATA_POINTS; j++)
    {
        ok &= setRefAccelerationRaw(j, accs[j]);
    }

    return ok;
}

// -----------------------------------------------------------------------------

bool DextraRawControlBoard::setRefAccelerationsRaw(int n_joint, const int * joints, const double * accs)
{

    bool ok = true;

    for (int i = 0; i < n_joint; i++)
    {
        ok &= setRefAccelerationRaw(joints[i], accs[i]);
    }

    return ok;
}

// -----------------------------------------------------------------------------------------

bool DextraRawControlBoard::getRefSpeedRaw(int j, double * ref)
{
    CHECK_JOINT(j);
    return false;
}

// -----------------------------------------------------------------------------------------

bool DextraRawControlBoard::getRefSpeedsRaw(double * spds)
{

    bool ok = true;

    for (int j = 0; j < Synapse::DATA_POINTS; j++)
    {
        ok &= getRefSpeedRaw(j, &spds[j]);
    }

    return ok;
}

// -----------------------------------------------------------------------------

bool DextraRawControlBoard::getRefSpeedsRaw(int n_joint, const int * joints, double * spds)
{

    bool ok = true;

    for (int i = 0; i < n_joint; i++)
    {
        ok &= getRefSpeedRaw(joints[i], &spds[i]);
    }

    return ok;
}

// -----------------------------------------------------------------------------------------

bool DextraRawControlBoard::getRefAccelerationRaw(int j, double * acc)
{
    CHECK_JOINT(j);
    return false;
}

// -----------------------------------------------------------------------------------------

bool DextraRawControlBoard::getRefAccelerationsRaw(double * accs)
{

    bool ok = true;

    for (int j = 0; j < Synapse::DATA_POINTS; j++)
    {
        ok &= getRefAccelerationRaw(j, &accs[j]);
    }

    return ok;
}

// -----------------------------------------------------------------------------

bool DextraRawControlBoard::getRefAccelerationsRaw(int n_joint, const int * joints, double * accs)
{

    bool ok = true;

    for (int i = 0; i < n_joint; i++)
    {
        ok &= getRefAccelerationRaw(joints[i], &accs[i]);
    }

    return ok;
}

// -----------------------------------------------------------------------------------------

bool DextraRawControlBoard::stopRaw(int j)
{
    CHECK_JOINT(j);
    return false;
}

// -----------------------------------------------------------------------------------------

bool DextraRawControlBoard::stopRaw()
{

    bool ok = true;

    for (int j = 0; j < Synapse::DATA_POINTS; j++)
    {
        ok &= stopRaw(j);
    }

    return ok;
}

// -----------------------------------------------------------------------------

bool DextraRawControlBoard::stopRaw(int n_joint, const int * joints)
{

    bool ok = true;

    for (int i = 0; i < n_joint; i++)
    {
        ok &= stopRaw(joints[i]);
    }

    return ok;
}

// -----------------------------------------------------------------------------

bool DextraRawControlBoard::getTargetPositionRaw(int joint, double * ref)
{
    return false;
}

// -----------------------------------------------------------------------------

bool DextraRawControlBoard::getTargetPositionsRaw(double * refs)
{

    bool ok = true;

    for (int j = 0; j < Synapse::DATA_POINTS; j++)
    {
        ok &= getTargetPositionRaw(j, &refs[j]);
    }

    return ok;
}

// -----------------------------------------------------------------------------

bool DextraRawControlBoard::getTargetPositionsRaw(int n_joint, const int * joints, double * refs)
{

    bool ok = true;

    for (int i = 0; i < n_joint; i++)
    {
        ok &= getTargetPositionRaw(joints[i], &refs[i]);
    }

    return ok;
}

// -----------------------------------------------------------------------------
