// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __DEXTRA_SERIAL_CONTROL_BOARD_HPP__
#define __DEXTRA_SERIAL_CONTROL_BOARD_HPP__

#include <yarp/conf/version.h>

#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/IAxisInfo.h>
#include <yarp/dev/IControlLimits.h>
#include <yarp/dev/IControlMode.h>
#include <yarp/dev/IEncodersTimed.h>
#include <yarp/dev/IPositionControl.h>
#include <yarp/dev/IPositionDirect.h>

#include <yarp/dev/ISerialDevice.h>
#include <yarp/dev/PolyDriver.h>

#include "DextraRawControlBoard.hpp"
#include "Synapse.hpp"

namespace roboticslab
{

/**
 * @ingroup YarpPlugins
 * @defgroup DextraSerialControlBoard
 * @brief Contains roboticslab::DextraSerialControlBoard.
 */

/**
 * @ingroup DextraSerialControlBoard
 * @brief Synapse interface for a serial bus.
 */
class SerialSynapse : public Synapse
{
public:
    //! Constructor.
    SerialSynapse(yarp::dev::ISerialDevice * iSerialDevice);

protected:
    bool getMessage(unsigned char * msg, char stopByte, int size) override;
    bool sendMessage(unsigned char * msg, int size) override;

private:
    yarp::dev::ISerialDevice * iSerialDevice;
};

/**
 * @ingroup DextraSerialControlBoard
 * @brief Implementation of a serial-based stand-alone control board for a Dextra hand.
 */
class DextraSerialControlBoard : public yarp::dev::DeviceDriver,
                                 public yarp::dev::IAxisInfo,
                                 public yarp::dev::IControlLimits,
                                 public yarp::dev::IControlMode,
                                 public yarp::dev::IEncodersTimed,
                                 public yarp::dev::IPositionControl,
                                 public yarp::dev::IPositionDirect
{
public:
    //  --------- DeviceDriver declarations. Implementation in DeviceDriverImpl.cpp ---------

    bool open(yarp::os::Searchable & config) override;
    bool close() override;

    //  --------- IAxisInfo declarations ---------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    yarp::dev::ReturnValue getAxisName(int axis, std::string & name) override
    { return raw.getAxisNameRaw(axis, name); }
    yarp::dev::ReturnValue getJointType(int axis, yarp::dev::JointTypeEnum & type) override
    { return raw.getJointTypeRaw(axis, type); }
#else
    bool getAxisName(int axis, std::string & name) override
    { return raw.getAxisNameRaw(axis, name); }
    bool getJointType(int axis, yarp::dev::JointTypeEnum & type) override
    { return raw.getJointTypeRaw(axis, type); }
#endif

    //  --------- IControlLimits declarations ---------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    yarp::dev::ReturnValue setPosLimits(int axis, double min, double max) override
    { return raw.setPosLimitsRaw(axis, min, max); }
    yarp::dev::ReturnValue getPosLimits(int axis, double * min, double * max) override
    { return raw.getPosLimitsRaw(axis, min, max); }
    yarp::dev::ReturnValue setVelLimits(int axis, double min, double max) override
    { return raw.setVelLimitsRaw(axis, min, max); }
    yarp::dev::ReturnValue getVelLimits(int axis, double * min, double * max) override
    { return raw.getVelLimitsRaw(axis, min, max); }
#else
    bool setLimits(int axis, double min, double max) override
    { return raw.setLimitsRaw(axis, min, max); }
    bool getLimits(int axis, double * min, double * max) override
    { return raw.getLimitsRaw(axis, min, max); }
    bool setVelLimits(int axis, double min, double max) override
    { return raw.setVelLimitsRaw(axis, min, max); }
    bool getVelLimits(int axis, double * min, double * max) override
    { return raw.getVelLimitsRaw(axis, min, max); }
#endif

    //  --------- IControlMode declarations ---------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    yarp::dev::ReturnValue getAvailableControlModes(int j, std::vector<yarp::dev::SelectableControlModeEnum> & modes) override
    { return raw.getAvailableControlModesRaw(j, modes); }
    yarp::dev::ReturnValue getControlMode(int j, yarp::dev::ControlModeEnum & mode) override
    { return raw.getControlModeRaw(j, mode); }
    yarp::dev::ReturnValue getControlModes(std::vector<yarp::dev::ControlModeEnum> & modes) override
    { return raw.getControlModesRaw(modes); }
    yarp::dev::ReturnValue getControlModes(const std::vector<int> & joints, std::vector<yarp::dev::ControlModeEnum> & modes) override
    { return raw.getControlModesRaw(joints, modes); }
    yarp::dev::ReturnValue setControlMode(int j, yarp::dev::SelectableControlModeEnum mode) override
    { return raw.setControlModeRaw(j, mode); }
    yarp::dev::ReturnValue setControlModes(const std::vector<yarp::dev::SelectableControlModeEnum> & modes) override
    { return raw.setControlModesRaw(modes); }
    yarp::dev::ReturnValue setControlModes(const std::vector<int> & joints, const std::vector<yarp::dev::SelectableControlModeEnum> & modes) override
    { return raw.setControlModesRaw(joints, modes); }
#else
    bool getControlMode(int j, int * mode) override
    { return raw.getControlModeRaw(j, mode); }
    bool getControlModes(int * modes) override
    { return raw.getControlModesRaw(modes); }
    bool getControlModes(int n_joint, const int * joints, int * modes) override
    { return raw.getControlModesRaw(n_joint, joints, modes); }
    bool setControlMode(int j, int mode) override
    { return raw.setControlModeRaw(j, mode); }
    bool setControlModes(int * modes) override
    { return raw.setControlModesRaw(modes); }
    bool setControlModes(int n_joint, const int * joints, int * modes) override
    { return raw.setControlModesRaw(n_joint, joints, modes); }
#endif

    //  ---------- IEncoders declarations ----------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    yarp::dev::ReturnValue resetEncoder(int j) override
    { return raw.resetEncoderRaw(j); }
    yarp::dev::ReturnValue resetEncoders() override
    { return raw.resetEncodersRaw(); }
    yarp::dev::ReturnValue setEncoder(int j, double val) override
    { return raw.setEncoderRaw(j, val); }
    yarp::dev::ReturnValue setEncoders(const double *vals) override
    { return raw.setEncodersRaw(vals); }
    yarp::dev::ReturnValue getEncoder(int j, double * v) override
    { return raw.getEncoderRaw(j, v); }
    yarp::dev::ReturnValue getEncoders(double *encs) override
    { return raw.getEncodersRaw(encs); }
    yarp::dev::ReturnValue getEncoderSpeed(int j, double * sp) override
    { return raw.getEncoderSpeedRaw(j, sp); }
    yarp::dev::ReturnValue getEncoderSpeeds(double * spds) override
    { return raw.getEncoderSpeedsRaw(spds); }
    yarp::dev::ReturnValue getEncoderAcceleration(int j, double * spds) override
    { return raw.getEncoderAccelerationRaw(j, spds); }
    yarp::dev::ReturnValue getEncoderAccelerations(double * accs) override
    { return raw.getEncoderAccelerationsRaw(accs); }
    yarp::dev::ReturnValue getEncoderTimed(int j, double * encs, double * time) override
    { return raw.getEncoderTimedRaw(j, encs, time); }
    yarp::dev::ReturnValue getEncodersTimed(double * encs, double * time) override
    { return raw.getEncodersTimedRaw(encs, time); }
#else
    bool resetEncoder(int j) override
    { return raw.resetEncoderRaw(j); }
    bool resetEncoders() override
    { return raw.resetEncodersRaw(); }
    bool setEncoder(int j, double val) override
    { return raw.setEncoderRaw(j, val); }
    bool setEncoders(const double *vals) override
    { return raw.setEncodersRaw(vals); }
    bool getEncoder(int j, double * v) override
    { return raw.getEncoderRaw(j, v); }
    bool getEncoders(double *encs) override
    { return raw.getEncodersRaw(encs); }
    bool getEncoderSpeed(int j, double * sp) override
    { return raw.getEncoderSpeedRaw(j, sp); }
    bool getEncoderSpeeds(double * spds) override
    { return raw.getEncoderSpeedsRaw(spds); }
    bool getEncoderAcceleration(int j, double * spds) override
    { return raw.getEncoderAccelerationRaw(j, spds); }
    bool getEncoderAccelerations(double * accs) override
    { return raw.getEncoderAccelerationsRaw(accs); }
    bool getEncoderTimed(int j, double * encs, double * time) override
    { return raw.getEncoderTimedRaw(j, encs, time); }
    bool getEncodersTimed(double * encs, double * time) override
    { return raw.getEncodersTimedRaw(encs, time); }
#endif

    // ------- IPositionControl declarations -------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    yarp::dev::ReturnValue getAxes(std::size_t & ax) override
    { return raw.getAxes(ax); }
    yarp::dev::ReturnValue positionMove(int j, double ref) override
    { return raw.positionMoveRaw(j, ref); }
    yarp::dev::ReturnValue positionMove(const double * refs) override
    { return raw.positionMoveRaw(refs); }
    yarp::dev::ReturnValue positionMove(int n_joint, const int * joints, const double * refs) override
    { return raw.positionMoveRaw(n_joint, joints, refs); }
    yarp::dev::ReturnValue relativeMove(int j, double delta) override
    { return raw.relativeMoveRaw(j, delta); }
    yarp::dev::ReturnValue relativeMove(const double * deltas) override
    { return raw.relativeMoveRaw(deltas); }
    yarp::dev::ReturnValue relativeMove(int n_joint, const int * joints, const double * deltas) override
    { return raw.relativeMoveRaw(n_joint, joints, deltas); }
    yarp::dev::ReturnValue checkMotionDone(int j, bool & flag) override
    { return raw.checkMotionDoneRaw(j, flag); }
    yarp::dev::ReturnValue checkMotionDone(bool & flag) override
    { return raw.checkMotionDoneRaw(flag); }
    yarp::dev::ReturnValue checkMotionDone(const std::vector<int> & joints, bool & flag) override
    { return raw.checkMotionDoneRaw(joints, flag); }
    yarp::dev::ReturnValue setTrajSpeed(int j, double sp) override
    { return raw.setTrajSpeedRaw(j, sp); }
    yarp::dev::ReturnValue setTrajSpeeds(const double * spds) override
    { return raw.setTrajSpeedsRaw(spds); }
    yarp::dev::ReturnValue setTrajSpeeds(int n_joint, const int * joints, const double * spds) override
    { return raw.setTrajSpeedsRaw(n_joint, joints, spds); }
    yarp::dev::ReturnValue setTrajAcceleration(int j, double acc) override
    { return raw.setTrajAccelerationRaw(j, acc); }
    yarp::dev::ReturnValue setTrajAccelerations(const double * accs) override
    { return raw.setTrajAccelerationsRaw(accs); }
    yarp::dev::ReturnValue setTrajAccelerations(int n_joint, const int * joints, const double * accs) override
    { return raw.setTrajAccelerationsRaw(n_joint, joints, accs); }
    yarp::dev::ReturnValue getTrajSpeed(int j, double * ref) override
    { return raw.getTrajSpeedRaw(j, ref); }
    yarp::dev::ReturnValue getTrajSpeeds(double * spds) override
    { return raw.getTrajSpeedsRaw(spds); }
    yarp::dev::ReturnValue getTrajSpeeds(int n_joint, const int * joints, double * spds) override
    { return raw.getTrajSpeedsRaw(n_joint, joints, spds); }
    yarp::dev::ReturnValue getTrajAcceleration(int j, double * acc) override
    { return raw.getTrajAccelerationRaw(j, acc); }
    yarp::dev::ReturnValue getTrajAccelerations(double * accs) override
    { return raw.getTrajAccelerationsRaw(accs); }
    yarp::dev::ReturnValue getTrajAccelerations(int n_joint, const int * joints, double * accs) override
    { return raw.getTrajAccelerationsRaw(n_joint, joints, accs); }
    yarp::dev::ReturnValue stop(int j) override
    { return raw.stopRaw(j); }
    yarp::dev::ReturnValue stop() override
    { return raw.stopRaw(); }
    yarp::dev::ReturnValue stop(int n_joint, const int * joints) override
    { return raw.stopRaw(n_joint, joints); }
    yarp::dev::ReturnValue getTargetPosition(int joint, double * ref) override
    { return raw.getTargetPositionsRaw(joint, &joint, ref); }
    yarp::dev::ReturnValue getTargetPositions(double * refs) override
    { return raw.getTargetPositionsRaw(refs); }
    yarp::dev::ReturnValue getTargetPositions(int n_joint, const int * joints, double * refs) override
    { return raw.getTargetPositionsRaw(n_joint, joints, refs); }
#else
    bool getAxes(int * ax) override
    { return raw.getAxes(ax); }
    bool positionMove(int j, double ref) override
    { return raw.positionMoveRaw(j, ref); }
    bool positionMove(const double * refs) override
    { return raw.positionMoveRaw(refs); }
    bool positionMove(int n_joint, const int * joints, const double * refs) override
    { return raw.positionMoveRaw(n_joint, joints, refs); }
    bool relativeMove(int j, double delta) override
    { return raw.relativeMoveRaw(j, delta); }
    bool relativeMove(const double * deltas) override
    { return raw.relativeMoveRaw(deltas); }
    bool relativeMove(int n_joint, const int * joints, const double * deltas) override
    { return raw.relativeMoveRaw(n_joint, joints, deltas); }
    bool checkMotionDone(int j, bool * flag) override
    { return raw.checkMotionDoneRaw(j, flag); }
    bool checkMotionDone(bool * flag) override
    { return raw.checkMotionDoneRaw(flag); }
    bool checkMotionDone(int n_joint, const int * joints, bool * flag) override
    { return raw.checkMotionDoneRaw(n_joint, joints, flag); }
    bool setRefSpeed(int j, double sp) override
    { return raw.setRefSpeedRaw(j, sp); }
    bool setRefSpeeds(const double * spds) override
    { return raw.setRefSpeedsRaw(spds); }
    bool setRefSpeeds(int n_joint, const int * joints, const double * spds) override
    { return raw.setRefSpeedsRaw(n_joint, joints, spds); }
    bool setRefAcceleration(int j, double acc) override
    { return raw.setRefAccelerationRaw(j, acc); }
    bool setRefAccelerations(const double * accs) override
    { return raw.setRefAccelerationsRaw(accs); }
    bool setRefAccelerations(int n_joint, const int * joints, const double * accs) override
    { return raw.setRefAccelerationsRaw(n_joint, joints, accs); }
    bool getRefSpeed(int j, double * ref) override
    { return raw.getRefSpeedRaw(j, ref); }
    bool getRefSpeeds(double * spds) override
    { return raw.getRefSpeedsRaw(spds); }
    bool getRefSpeeds(int n_joint, const int * joints, double * spds) override
    { return raw.getRefSpeedsRaw(n_joint, joints, spds); }
    bool getRefAcceleration(int j, double * acc) override
    { return raw.getRefAccelerationRaw(j, acc); }
    bool getRefAccelerations(double * accs) override
    { return raw.getRefAccelerationsRaw(accs); }
    bool getRefAccelerations(int n_joint, const int * joints, double * accs) override
    { return raw.getRefAccelerationsRaw(n_joint, joints, accs); }
    bool stop(int j) override
    { return raw.stopRaw(j); }
    bool stop() override
    { return raw.stopRaw(); }
    bool stop(int n_joint, const int * joints) override
    { return raw.stopRaw(n_joint, joints); }
    bool getTargetPosition(int joint, double * ref) override
    { return raw.getTargetPositionsRaw(joint, &joint, ref); }
    bool getTargetPositions(double * refs) override
    { return raw.getTargetPositionsRaw(refs); }
    bool getTargetPositions(int n_joint, const int * joints, double * refs) override
    { return raw.getTargetPositionsRaw(n_joint, joints, refs); }
#endif

    // ------- IPositionDirect declarations -------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    yarp::dev::ReturnValue setPosition(int j, double ref) override
    { return raw.setPositionRaw(j, ref); }
    yarp::dev::ReturnValue setPositions(const double * refs) override
    { return raw.setPositionsRaw(refs); }
    yarp::dev::ReturnValue setPositions(int n_joint, const int * joints, const double * refs) override
    { return raw.setPositionsRaw(n_joint, joints, refs); }
    yarp::dev::ReturnValue getRefPosition(int j, double * ref) override
    { return raw.getRefPositionRaw(j, ref); }
    yarp::dev::ReturnValue getRefPositions(double * refs) override
    { return raw.getRefPositionsRaw(refs); }
    yarp::dev::ReturnValue getRefPositions(int n_joint, const int * joints, double * refs) override
    { return raw.getRefPositionsRaw(n_joint, joints, refs); }
#else
    bool setPosition(int j, double ref) override
    { return raw.setPositionRaw(j, ref); }
    bool setPositions(const double * refs) override
    { return raw.setPositionsRaw(refs); }
    bool setPositions(int n_joint, const int * joints, const double * refs) override
    { return raw.setPositionsRaw(n_joint, joints, refs); }
    bool getRefPosition(int j, double * ref) override
    { return raw.getRefPositionRaw(j, ref); }
    bool getRefPositions(double * refs) override
    { return raw.getRefPositionsRaw(refs); }
    bool getRefPositions(int n_joint, const int * joints, double * refs) override
    { return raw.getRefPositionsRaw(n_joint, joints, refs); }
#endif

protected:
    DextraRawControlBoard raw;
    yarp::dev::PolyDriver serialDevice;
};

} // namespace roboticslab

#endif // __DEXTRA_SERIAL_CONTROL_BOARD_HPP__
