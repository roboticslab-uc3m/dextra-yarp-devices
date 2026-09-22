// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __DEXTRA_RAW_CONTROL_BOARD_HPP__
#define __DEXTRA_RAW_CONTROL_BOARD_HPP__

#include <mutex>
#include <string>

#include <yarp/conf/version.h>

#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/IAxisInfo.h>
#include <yarp/dev/IControlLimits.h>
#include <yarp/dev/IControlMode.h>
#include <yarp/dev/IEncodersTimed.h>
#include <yarp/dev/IPositionControl.h>
#include <yarp/dev/IPositionDirect.h>

#include "Synapse.hpp"

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
#define CHECK_JOINT(j) do { if ((j) < 0 || (j) >= Synapse::DATA_POINTS) return yarp::dev::ReturnValue_error_input_out_of_bounds; } while (0)
#else
#define CHECK_JOINT(j) do { if ((j) < 0 || (j) >= Synapse::DATA_POINTS) return false; } while (0)
#endif

namespace roboticslab
{

/**
 * @ingroup yarp_devices_libraries
 * @defgroup DextraRawControlBoard
 * @brief Contains roboticslab::DextraRawControlBoard.
 */

/**
 * @ingroup DextraRawControlBoard
 * @brief Base implementation for the custom UC3M Dextra Hand control board interfaces.
 */
class DextraRawControlBoard : public yarp::dev::DeviceDriver,
                              public yarp::dev::IAxisInfoRaw,
                              public yarp::dev::IControlLimitsRaw,
                              public yarp::dev::IControlModeRaw,
                              public yarp::dev::IEncodersTimedRaw,
                              public yarp::dev::IPositionControlRaw,
                              public yarp::dev::IPositionDirectRaw
{
public:
    ~DextraRawControlBoard() override = default;

    void acquireSynapseHandle(Synapse * synapse);
    void destroySynapse();

    //  --------- DeviceDriver declarations. Implementation in DeviceDriverImpl.cpp ---------

    bool open(yarp::os::Searchable & config) override;

    //  --------- IAxisInfoRaw declarations. Implementation in IAxisInfoRawImpl.cpp ---------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    yarp::dev::ReturnValue getAxisNameRaw(int axis, std::string & name) override;
    yarp::dev::ReturnValue getJointTypeRaw(int axis, yarp::dev::JointTypeEnum & type) override;
#else
    bool getAxisNameRaw(int axis, std::string & name) override;
    bool getJointTypeRaw(int axis, yarp::dev::JointTypeEnum & type) override;
#endif

    //  --------- IControlLimitsRaw declarations. Implementation in IControlLimitsRawImpl.cpp ---------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    yarp::dev::ReturnValue setPosLimitsRaw(int axis, double min, double max) override;
    yarp::dev::ReturnValue getPosLimitsRaw(int axis, double * min, double * max) override;
    yarp::dev::ReturnValue setVelLimitsRaw(int axis, double min, double max) override;
    yarp::dev::ReturnValue getVelLimitsRaw(int axis, double * min, double * max) override;
#else
    bool setLimitsRaw(int axis, double min, double max) override;
    bool getLimitsRaw(int axis, double * min, double * max) override;
    bool setVelLimitsRaw(int axis, double min, double max) override;
    bool getVelLimitsRaw(int axis, double * min, double * max) override;
#endif

    //  --------- IControlModeRaw declarations. Implementation in IControlModeRawImpl.cpp ---------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    yarp::dev::ReturnValue getAvailableControlModesRaw(int j, std::vector<yarp::dev::SelectableControlModeEnum> & modes) override;
    yarp::dev::ReturnValue getControlModeRaw(int j, yarp::dev::ControlModeEnum & mode) override;
    yarp::dev::ReturnValue getControlModesRaw(std::vector<yarp::dev::ControlModeEnum> & modes) override;
    yarp::dev::ReturnValue getControlModesRaw(const std::vector<int> & joints, std::vector<yarp::dev::ControlModeEnum> & modes) override;
    yarp::dev::ReturnValue setControlModeRaw(int j, yarp::dev::SelectableControlModeEnum mode) override;
    yarp::dev::ReturnValue setControlModesRaw(const std::vector<yarp::dev::SelectableControlModeEnum> & modes) override;
    yarp::dev::ReturnValue setControlModesRaw(const std::vector<int> & joints, const std::vector<yarp::dev::SelectableControlModeEnum> & modes) override;
#else
    bool getControlModeRaw(int j, int * mode) override;
    bool getControlModesRaw(int * modes) override;
    bool getControlModesRaw(int n_joint, const int * joints, int * modes) override;
    bool setControlModeRaw(int j, int mode) override;
    bool setControlModesRaw(int * modes) override;
    bool setControlModesRaw(int n_joint, const int * joints, int * modes) override;
#endif

    //  ---------- IEncodersRaw declarations. Implementation in IEncodersRawImpl.cpp ----------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    yarp::dev::ReturnValue getAxes(std::size_t & ax) override;
    yarp::dev::ReturnValue resetEncoderRaw(int j) override;
    yarp::dev::ReturnValue resetEncodersRaw() override;
    yarp::dev::ReturnValue setEncoderRaw(int j, double val) override;
    yarp::dev::ReturnValue setEncodersRaw(const double * vals) override;
    yarp::dev::ReturnValue getEncoderRaw(int j, double * v) override;
    yarp::dev::ReturnValue getEncodersRaw(double * encs) override;
    yarp::dev::ReturnValue getEncoderSpeedRaw(int j, double * sp) override;
    yarp::dev::ReturnValue getEncoderSpeedsRaw(double * spds) override;
    yarp::dev::ReturnValue getEncoderAccelerationRaw(int j, double * spds) override;
    yarp::dev::ReturnValue getEncoderAccelerationsRaw(double * accs) override;
    yarp::dev::ReturnValue getEncoderTimedRaw(int j, double * encs, double * time) override;
    yarp::dev::ReturnValue getEncodersTimedRaw(double * encs, double * time) override;
#else
    bool getAxes(int * ax) override;
    bool resetEncoderRaw(int j) override;
    bool resetEncodersRaw() override;
    bool setEncoderRaw(int j, double val) override;
    bool setEncodersRaw(const double * vals) override;
    bool getEncoderRaw(int j, double * v) override;
    bool getEncodersRaw(double * encs) override;
    bool getEncoderSpeedRaw(int j, double * sp) override;
    bool getEncoderSpeedsRaw(double * spds) override;
    bool getEncoderAccelerationRaw(int j, double * spds) override;
    bool getEncoderAccelerationsRaw(double * accs) override;
    bool getEncoderTimedRaw(int j, double * encs, double * time) override;
    bool getEncodersTimedRaw(double * encs, double * time) override;
#endif

    // ------- IPositionControlRaw declarations. Implementation in IPositionControlRawImpl.cpp -------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    yarp::dev::ReturnValue positionMoveRaw(int j, double ref) override;
    yarp::dev::ReturnValue positionMoveRaw(const double * refs) override;
    yarp::dev::ReturnValue positionMoveRaw(int n_joint, const int * joints, const double * refs) override;
    yarp::dev::ReturnValue relativeMoveRaw(int j, double delta) override;
    yarp::dev::ReturnValue relativeMoveRaw(const double * deltas) override;
    yarp::dev::ReturnValue relativeMoveRaw(int n_joint, const int * joints, const double * deltas) override;
    yarp::dev::ReturnValue checkMotionDoneRaw(int j, bool & flag) override;
    yarp::dev::ReturnValue checkMotionDoneRaw(bool & flag) override;
    yarp::dev::ReturnValue checkMotionDoneRaw(const std::vector<int> & joints, bool & flag) override;
    yarp::dev::ReturnValue setTrajSpeedRaw(int j, double sp) override;
    yarp::dev::ReturnValue setTrajSpeedsRaw(const double * spds) override;
    yarp::dev::ReturnValue setTrajSpeedsRaw(int n_joint, const int * joints, const double * spds) override;
    yarp::dev::ReturnValue setTrajAccelerationRaw(int j, double acc) override;
    yarp::dev::ReturnValue setTrajAccelerationsRaw(const double * accs) override;
    yarp::dev::ReturnValue setTrajAccelerationsRaw(int n_joint, const int * joints, const double * accs) override;
    yarp::dev::ReturnValue getTrajSpeedRaw(int j, double * ref) override;
    yarp::dev::ReturnValue getTrajSpeedsRaw(double * spds) override;
    yarp::dev::ReturnValue getTrajSpeedsRaw(int n_joint, const int * joints, double * spds) override;
    yarp::dev::ReturnValue getTrajAccelerationRaw(int j, double * acc) override;
    yarp::dev::ReturnValue getTrajAccelerationsRaw(double * accs) override;
    yarp::dev::ReturnValue getTrajAccelerationsRaw(int n_joint, const int * joints, double * accs) override;
    yarp::dev::ReturnValue stopRaw(int j) override;
    yarp::dev::ReturnValue stopRaw() override;
    yarp::dev::ReturnValue stopRaw(int n_joint, const int * joints) override;
    yarp::dev::ReturnValue getTargetPositionRaw(int joint, double * ref) override;
    yarp::dev::ReturnValue getTargetPositionsRaw(double * refs) override;
    yarp::dev::ReturnValue getTargetPositionsRaw(int n_joint, const int * joints, double * refs) override;
#else
    bool positionMoveRaw(int j, double ref) override;
    bool positionMoveRaw(const double * refs) override;
    bool positionMoveRaw(int n_joint, const int * joints, const double * refs) override;
    bool relativeMoveRaw(int j, double delta) override;
    bool relativeMoveRaw(const double * deltas) override;
    bool relativeMoveRaw(int n_joint, const int * joints, const double * deltas) override;
    bool checkMotionDoneRaw(int j, bool * flag) override;
    bool checkMotionDoneRaw(bool * flag) override;
    bool checkMotionDoneRaw(int n_joint, const int * joints, bool * flag) override;
    bool setRefSpeedRaw(int j, double sp) override;
    bool setRefSpeedsRaw(const double * spds) override;
    bool setRefSpeedsRaw(int n_joint, const int * joints, const double * spds) override;
    bool setRefAccelerationRaw(int j, double acc) override;
    bool setRefAccelerationsRaw(const double * accs) override;
    bool setRefAccelerationsRaw(int n_joint, const int * joints, const double * accs) override;
    bool getRefSpeedRaw(int j, double * ref) override;
    bool getRefSpeedsRaw(double * spds) override;
    bool getRefSpeedsRaw(int n_joint, const int * joints, double * spds) override;
    bool getRefAccelerationRaw(int j, double * acc) override;
    bool getRefAccelerationsRaw(double * accs) override;
    bool getRefAccelerationsRaw(int n_joint, const int * joints, double * accs) override;
    bool stopRaw(int j) override;
    bool stopRaw() override;
    bool stopRaw(int n_joint, const int * joints) override;
    bool getTargetPositionRaw(int joint, double * ref) override;
    bool getTargetPositionsRaw(double * refs) override;
    bool getTargetPositionsRaw(int n_joint, const int * joints, double * refs) override;
#endif
    // ------- IPositionDirectRaw declarations. Implementation in IPositionDirectRawImpl.cpp -------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    yarp::dev::ReturnValue setPositionRaw(int j, double ref) override;
    yarp::dev::ReturnValue setPositionsRaw(const double * refs) override;
    yarp::dev::ReturnValue setPositionsRaw(int n_joint, const int * joints, const double * refs) override;
    yarp::dev::ReturnValue getRefPositionRaw(int j, double * ref) override;
    yarp::dev::ReturnValue getRefPositionsRaw(double * refs) override;
    yarp::dev::ReturnValue getRefPositionsRaw(int n_joint, const int * joints, double * refs) override;
#else
    bool setPositionRaw(int j, double ref) override;
    bool setPositionsRaw(const double * refs) override;
    bool setPositionsRaw(int n_joint, const int * joints, const double * refs) override;
    bool getRefPositionRaw(int j, double * ref) override;
    bool getRefPositionsRaw(double * refs) override;
    bool getRefPositionsRaw(int n_joint, const int * joints, double * refs) override;
#endif

protected:
    Synapse * synapse {nullptr};
    std::string axisPrefix;

private:
    double getSetpoint(int j);
    void getSetpoints(Synapse::Setpoints & setpoints);
    void setSetpoint(int j, Synapse::setpoint_t setpoint);
    void setSetpoints(const Synapse::Setpoints & setpoints);

    Synapse::Setpoints setpoints;
    mutable std::mutex setpointMutex;
};

} // namespace roboticslab

#endif // __DEXTRA_RAW_CONTROL_BOARD_HPP__
