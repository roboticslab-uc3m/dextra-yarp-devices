// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "DextraRawControlBoard.hpp"

using namespace roboticslab;

// ----------------------------------------------------------------------------------------

bool DextraRawControlBoard::setPositionRaw(int j, double ref)
{
    return positionMoveRaw(j, ref);
}

// ----------------------------------------------------------------------------------------

bool DextraRawControlBoard::setPositionsRaw(const double * refs)
{
    return positionMoveRaw(refs);
}

// ----------------------------------------------------------------------------------------

bool DextraRawControlBoard::setPositionsRaw(int n_joint, const int * joints, const double * refs)
{
    return positionMoveRaw(n_joint, joints, refs);
}

// ----------------------------------------------------------------------------------------
