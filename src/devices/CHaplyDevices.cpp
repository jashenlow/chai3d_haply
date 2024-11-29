//==============================================================================
/*
    Software License Agreement (BSD License)
    Copyright (c) 2003-2024, CHAI3D
    (www.chai3d.org)

    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:

    * Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.

    * Redistributions in binary form must reproduce the above
    copyright notice, this list of conditions and the following
    disclaimer in the documentation and/or other materials provided
    with the distribution.

    * Neither the name of CHAI3D nor the names of its contributors may
    be used to endorse or promote products derived from this software
    without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
    FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
    COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
    BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
    LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
    ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
    POSSIBILITY OF SUCH DAMAGE.

    \author    <http://www.chai3d.org>
    \author    Jashen Low
    \version   3.3.0

*/
//==============================================================================

//------------------------------------------------------------------------------
#include "system/CGlobals.h"
//------------------------------------------------------------------------------
#if defined(C_ENABLE_HAPLY_DEVICE_SUPPORT)
//------------------------------------------------------------------------------

////////////////////////////////////////////////////////////////////////////////
/*
    INSTRUCTION TO IMPLEMENT YOUR OWN CUSTOM DEVICE:

    Please review header file CHaplyDevices.h for some initial
    guidelines about how to implement your own haptic device using this
    template.

    When ready, simply completed the next 11 documented steps described here
    below.
*/
////////////////////////////////////////////////////////////////////////////////

// Haply specific includes

#include <fstream>
#include "devices/CHaplyDevices.h"

#include <cstdlib>
#include <thread>

//------------------------------------------------------------------------------
namespace chai3d
{
    //------------------------------------------------------------------------------
    namespace hAPI = Haply::HardwareAPI;

    static constexpr std::chrono::microseconds HAPLY_HANDLE_PERIOD_US =
        std::chrono::microseconds((uint64_t)((1.0 / (float)HAPLY_HANDLE_HZ) * 1000.0 * 1000.0));

    namespace ports
    {
        static std::vector<std::string> inverse3_ports;
        static std::vector<std::string> handle_ports;
    } // namespace ports

    cHaplyDevices::Handle::Handle(hAPI::IO::SerialStream *stream) : hAPI::Devices::Handle(stream)
    {
    }

    cHaplyDevices::Handle::~Handle()
    {
        m_exit.store(true);
        m_thread.stop();
    }

    // We run the handle in its own thread as the hardware runs more slowly then
    // the inverse3 hardware where making synchronous calls to it would slow
    // down the main thread down to the 50-60hz range.
    void cHaplyDevices::Handle::start()
    {
        m_thread.start(run, CTHREAD_PRIORITY_HAPTICS, this);
    }

    void cHaplyDevices::Handle::run(void *ctx)
    {
        auto handle = reinterpret_cast<cHaplyDevices::Handle *>(ctx);
        std::chrono::high_resolution_clock::time_point exec_start;
        std::chrono::high_resolution_clock::time_point exec_end;

        while (!handle->m_exit.load())
        {
            exec_start = std::chrono::high_resolution_clock::now();

            handle->SendHandleState(handle->handle_info.device_id, 0, nullptr);
            int ret = handle->Receive();
            if (ret < 0) {
                printf("%s: Error reading handle state with code %d.\n",
                    __FUNCTION__,
                    ret);
            }

            exec_end = std::chrono::high_resolution_clock::now();
            auto exec_elapsed =
                std::chrono::duration_cast<std::chrono::microseconds>(exec_end - exec_start);
            auto exec_remaining = std::chrono::microseconds(HAPLY_HANDLE_PERIOD_US) - exec_elapsed;
            if (exec_remaining > std::chrono::microseconds(0)) {
                std::this_thread::sleep_for(exec_remaining);
            }
            // printf("%s: Elapsed time = %.3fms\n", __FUNCTION__, (double)exec_elapsed.count() / 1000.0);
        }
    }

    void cHaplyDevices::Handle::OnReceiveHandleInfo(
        uint8_t handle_data_remaining,
        uint16_t device_id,
        uint8_t device_model_number,
        uint8_t hardware_version,
        uint8_t firmware_version)
    {
        handle_info.device_id = device_id;
        /* [Deprecated]
        handle_info.device_model_number = device_model_number;
        handle_info.hardware_version = hardware_version;
        handle_info.firmware_version = firmware_version;
        */
    }

    void cHaplyDevices::Handle::OnReceiveHandleStatusMessage(
        uint16_t device_id,
        float *quaternion,
        uint8_t error_flag,
        uint8_t hall_effect_sensor_level,
        uint8_t user_data_length,
        uint8_t *user_data)
    {
        std::lock_guard<std::mutex> lock(m_handle_state_lock);

        handle_state.device_id = device_id;
        memcpy(handle_state.quaternion, quaternion, sizeof(float) * hAPI::QUATERNION_SIZE);
        /* [Deprecated]
        handle_state.error_flag = error_flag;
        */
        handle_state.handle_connection_sensor = hall_effect_sensor_level;
        handle_state.user_data_length = user_data_length;
        memcpy(handle_state.user_data, user_data, user_data_length);
    }

    void cHaplyDevices::Handle::OnReceiveHandleErrorResponse(
        uint16_t device_id,
        uint8_t error_code)
    {
        printf("Haply handle error: device=%x, code=%x\n", device_id, error_code);
    }

    bool cHaplyDevices::openLibraries()
    {
        return (C_SUCCESS);
    }

    //==============================================================================
    /*!
        Constructor of cHaplyDevices.
    */
    //==============================================================================
    cHaplyDevices::cHaplyDevices(unsigned int a_deviceNumber)
    {
        // print out a message
        //  the connection to your device has not yet been established.
        m_deviceReady = false;

        // haptic device model (see file "CGenericHapticDevice.h")
        m_specifications.m_model = cHapticDeviceModel::C_HAPTIC_DEVICE_HAPLY_INVERSE3;

        // name of the device manufacturer, research lab, university.
        m_specifications.m_manufacturerName = "Haply Robotics";

        // name of your device
        m_specifications.m_modelName = "Inverse3";

        //--------------------------------------------------------------------------
        // CHARACTERISTICS: (The following values must be positive or equal to zero)
        //--------------------------------------------------------------------------

        // the maximum force [N] the device can produce along the x,y,z axis.
        m_specifications.m_maxLinearForce = 10.0; // [N]

        // the maximum amount of torque your device can provide arround its
        // rotation degrees of freedom.
        m_specifications.m_maxAngularTorque = 0.2; // [N*m]

        // the maximum amount of torque which can be provided by your gripper
        m_specifications.m_maxGripperForce = 3.0; // [N]

        // the maximum closed loop linear stiffness in [N/m] along the x,y,z axis
        m_specifications.m_maxLinearStiffness = 1000.0; // [N/m]

        // the maximum amount of angular stiffness
        m_specifications.m_maxAngularStiffness = 1.0; // [N*m/Rad]

        // the maximum amount of stiffness supported by the gripper
        m_specifications.m_maxGripperLinearStiffness = 1000.0; // [N*m]

        // the radius of the physical workspace of the device (x,y,z axis)
        m_specifications.m_workspaceRadius = 0.15; // [m]

        // the maximum opening angle of the gripper
        m_specifications.m_gripperMaxAngleRad = cDegToRad(30.0);

        ////////////////////////////////////////////////////////////////////////////
        /*
            DAMPING PROPERTIES:

            Start with small values as damping terms can be highly sensitive to
            the quality of your velocity signal and the spatial resolution of your
            device. Try gradually increasing the values by using example "01-devices"
            and by enabling viscosity with key command "2".
        */
        ////////////////////////////////////////////////////////////////////////////

        // Maximum recommended linear damping factor Kv
        m_specifications.m_maxLinearDamping = 6.0; // [N/(m/s)]

        //! Maximum recommended angular damping factor Kv (if actuated torques are available)
        m_specifications.m_maxAngularDamping = 0.0; // [N*m/(Rad/s)]

        //! Maximum recommended angular damping factor Kv for the force gripper. (if actuated gripper is available)
        m_specifications.m_maxGripperAngularDamping = 0.0; // [N*m/(Rad/s)]

        //--------------------------------------------------------------------------
        // CHARACTERISTICS: (The following are of boolean type: (true or false)
        //--------------------------------------------------------------------------

        // does your device provide sensed position (x,y,z axis)?
        m_specifications.m_sensedPosition = true;

        // does your device provide sensed rotations (i.e stylus)?
        m_specifications.m_sensedRotation = true;

        // does your device provide a gripper which can be sensed?
        m_specifications.m_sensedGripper = false;

        // is you device actuated on the translation degrees of freedom?
        m_specifications.m_actuatedPosition = true;

        // is your device actuated on the rotation degrees of freedom?
        m_specifications.m_actuatedRotation = false;

        // is the gripper of your device actuated?
        m_specifications.m_actuatedGripper = false;

        // can the device be used with the left hand?
        m_specifications.m_leftHand = true;

        // can the device be used with the right hand?
        m_specifications.m_rightHand = true;

        m_device_index = a_deviceNumber;
        m_deviceAvailable = true;
    }

    //==============================================================================
    /*!
        Destructor of cHaplyDevices.
    */
    //==============================================================================
    cHaplyDevices::~cHaplyDevices()
    {
        // close connection to device
        if (m_deviceReady)
        {
            close();
        }
    }

    //==============================================================================
    /*!
        This method opens a connection to your device.

        \return __true__ if the operation succeeds, __false__ otherwise.
    */
    //==============================================================================

    bool cHaplyDevices::openInverse()
    {
        if (ports::inverse3_ports.empty() ||
            ports::inverse3_ports.size() <= m_device_index) {
            return C_ERROR;
        }

        const std::string& port_str = ports::inverse3_ports[m_device_index];

        m_inverseStream = std::make_unique<hAPI::IO::SerialStream>(port_str.c_str());
        m_inverse = std::make_unique<hAPI::Devices::Inverse3>(m_inverseStream.get());

        hAPI::Devices::Inverse3::DeviceInfoResponse info =
            m_inverse->DeviceWakeup();

        printf("Initialized Inverse3 device on %s.\n", port_str.c_str());

        return C_SUCCESS;
    }

    bool cHaplyDevices::openHandle()
    {
        if (ports::handle_ports.empty() ||
            ports::handle_ports.size() <= m_device_index) {
            return C_ERROR;
        }

        const std::string& port_str = ports::handle_ports[m_device_index];

        m_handleStream = std::make_unique<hAPI::IO::SerialStream>(port_str.c_str());
        m_handle = std::make_unique<Handle>(m_handleStream.get());

        m_handle->SendDeviceWakeup();
        int ret = m_handle->Receive();
        // return value of > 0 means successful.
        if (ret <= 0) {
            m_handle.reset();
            m_handleStream.reset();
            return C_ERROR;
        }

        m_handle->start();
        printf("Initialized Inverse3 handle on %s.\n", port_str.c_str());

        return C_SUCCESS;
    }

    bool cHaplyDevices::open()
    {
        // check if the system is available or already opened.
        if (!m_deviceAvailable || m_deviceReady)
            return (C_ERROR);

        memset(m_force.force, 0, sizeof(float) * hAPI::VECTOR_SIZE);
        m_linear_velocity.fill(0);

        bool result = openInverse();
        result = result && openHandle();

        m_deviceReady = result;
        return result;
    }

    //==============================================================================
    /*!
        This method closes the connection to your device.

        \return __true__ if the operation succeeds, __false__ otherwise.
    */
    //==============================================================================
    bool cHaplyDevices::close()
    {
        // check if the system has been opened previously
        if (!m_deviceReady)
            return (C_ERROR);

        m_handle.reset();
        m_handleStream.reset();

        m_inverse.reset();
        m_inverseStream.reset();

        m_deviceReady = false;
        
        return C_SUCCESS;
    }

    //==============================================================================
    /*!
        This method calibrates your device.

        \return __true__ if the operation succeeds, __false__ otherwise.
    */
    //==============================================================================
    bool cHaplyDevices::calibrate(bool a_forceCalibration)
    {
        // check if the device is read. See step 3.
        if (!m_deviceReady)
            return (C_ERROR);

        // Nothing to do here for the Haply Inverse3.
        // Calibration is done manually on the device.

        return C_SUCCESS;
    }

    //==============================================================================
    /*!
        This method returns the number of devices available from this class of device.

        \return __true__ if the operation succeeds, __false__ otherwise.
    */
    //==============================================================================
    unsigned int cHaplyDevices::getNumDevices()
    {
        /*
        * Detect Inverse3 devices
        */
        // Use auto port detection here
        ports::inverse3_ports = hAPI::Devices::DeviceDetection::DetectInverse3s();

        if (ports::inverse3_ports.size() > 0) {
            printf("Detected %zd Inverse3 devices on:", ports::inverse3_ports.size());

            for (const auto& port : ports::inverse3_ports) {
                printf(" %s", port.c_str());
            }
        }
        else {
            printf("Detected 0 Inverse3 devices.");
        }     
        printf("\n");

        /*
        * Detect Inverse3 handles
        */
        ports::handle_ports = hAPI::Devices::DeviceDetection::DetectHandles();

        if (ports::handle_ports.size() > 0) {
            printf("Detected %zd Inverse3 handles on:", ports::handle_ports.size());

            for (const auto& port : ports::handle_ports) {
                printf(" %s", port.c_str());
            }
        }
        else {
            printf("Detected 0 Inverse3 handles.");
        }
        printf("\n");

        // Only count the Inverse3 devices.
        return ports::inverse3_ports.size();
    }

    //==============================================================================
    /*!
        This method returns the position of your device. Units are meters [m].

        \param   a_position  Return value.

        \return __true__ if the operation succeeds, __false__ otherwise.
    */
    //==============================================================================
    bool cHaplyDevices::getPosition(cVector3d &a_position)
    {
        // check if the device is read. See step 3.
        if (!m_deviceReady)
            return (C_ERROR);

        /*
            Note:
            For consistency, units must be in meters.
            If your device is located in front of you, the x-axis is pointing
            towards you (the operator). The y-axis points towards your right
            hand side and the z-axis points up towards the sky.
        */
        hAPI::Devices::Inverse3::EndEffectorStateResponse response =
            m_inverse->EndEffectorForce(m_force);
        
        a_position.set(
            -response.position[1] - 0.150f,
            response.position[0],
            response.position[2] - 0.20f); // 0.10f

        memcpy(m_linear_velocity.data(), response.velocity, sizeof(float) * hAPI::VECTOR_SIZE);

        return C_SUCCESS;
    }

    bool cHaplyDevices::getLinearVelocity(cVector3d &a_linearvelocity)
    {
        a_linearvelocity.set(
            -m_linear_velocity[1],
            m_linear_velocity[0],
            m_linear_velocity[2]);

        return C_SUCCESS;
    }

    //==============================================================================
    /*!
        This method returns the orientation frame of your device end-effector

        \param   a_rotation  Return value.

        \return __true__ if the operation succeeds, __false__ otherwise.
    */
    //==============================================================================
    bool cHaplyDevices::getRotation(cMatrix3d &a_rotation)
    {
        // check if the device is read. See step 3.
        if (!m_deviceReady)
            return (C_ERROR);

        // Get handle orientation
        m_rot_matrix.identity();

        if (m_handle.get() != nullptr)
        {
            float Q[4] = { 0, 0, 0, 0 };
            {
                std::lock_guard<std::mutex> lock{ m_handle->m_handle_state_lock };
                memcpy(Q, m_handle->handle_state.quaternion, sizeof(float) * hAPI::QUATERNION_SIZE);
            };
            // TODO: Verify if these 2 lines need to be updated from >= HardwareAPI 0.1.12.
            Q[1] = -Q[1];
            Q[3] = -Q[3];

            m_rot_matrix(0, 0) = 1 - 2 * (Q[1] * Q[1] + Q[3] * Q[3]);
            m_rot_matrix(0, 1) = 2 * (Q[2] * Q[1] + Q[3] * Q[0]);
            m_rot_matrix(0, 2) = 2 * (Q[2] * Q[3] - Q[1] * Q[0]);

            m_rot_matrix(1, 0) = 2 * (Q[2] * Q[1] - Q[3] * Q[0]);
            m_rot_matrix(1, 1) = 1 - 2 * (Q[2] * Q[2] + Q[3] * Q[3]);
            m_rot_matrix(1, 2) = 2 * (Q[1] * Q[3] + Q[2] * Q[0]);

            m_rot_matrix(2, 0) = 2 * (Q[2] * Q[3] + Q[1] * Q[0]);
            m_rot_matrix(2, 1) = 2 * (Q[1] * Q[3] - Q[2] * Q[0]);
            m_rot_matrix(2, 2) = 1 - 2 * (Q[1] * Q[1] + Q[2] * Q[2]);

            // store new rotation matrix
            a_rotation = m_rot_matrix;

            return C_SUCCESS;
        }
        else {
            return C_ERROR;
        }
    }

    bool cHaplyDevices::getAngularVelocity(cVector3d& a_angularVelocity)
    {
        // estimate angular velocity
        estimateAngularVelocity(m_rot_matrix);
        a_angularVelocity = m_angularVelocity;

        return C_SUCCESS;
    }

    //==============================================================================
    /*!
        This method returns the gripper angle in radian.

        \param   a_angle  Return value.

        \return __true__ if the operation succeeds, __false__ otherwise.
    */
    //==============================================================================
    bool cHaplyDevices::getGripperAngleRad(double &a_angle)
    {
        // check if the device is read. See step 3.
        if (!m_deviceReady)
            return (C_ERROR);

        // return gripper angle in radian
        a_angle = 0.0; // a_angle = getGripperAngleInRadianFromMyDevice();

        // estimate gripper velocity
        estimateGripperVelocity(a_angle);

        // exit
        return C_SUCCESS;
    }

    //==============================================================================
    /*!
        This method sends a force [N] and a torque [N*m] and gripper torque [N*m]
        to your haptic device.

        \param   a_force  Force command.
        \param   a_torque  Torque command.
        \param   a_gripperForce  Gripper force command.

        \return __true__ if the operation succeeds, __false__ otherwise.
    */
    //==============================================================================
    bool cHaplyDevices::setForceAndTorqueAndGripperForce(const cVector3d &a_force,
                                                           const cVector3d &a_torque,
                                                           const double a_gripperForce)
    {
        // check if the device is read, and forces are enabled.
        if (!m_deviceReady || !m_enable_forces)
            return (C_ERROR);

        m_force.force[0] = (float)a_force(1);  // y
        m_force.force[1] = -(float)a_force(0);  // x
        m_force.force[2] = (float)a_force(2);  // z

        // store new values.
        m_prevForce = a_force;
        m_prevTorque = a_torque;
        m_prevGripperForce = a_gripperForce;

        return C_SUCCESS;
    }

    bool cHaplyDevices::enableForces(bool a_value)
    {
        if (!a_value) {
            memset(m_force.force, 0, sizeof(float) * hAPI::VECTOR_SIZE);
            m_prevForce.set(m_force.force[0], m_force.force[1], m_force.force[2]);
        }

        m_enable_forces = a_value;

        return C_SUCCESS;
    }

    bool cHaplyDevices::getUserSwitch(int a_switchIndex, bool& a_status)
    {
        /*
        The Inverse3 handle only has 1 user switch,
        but if a_switchIndex > HandleStatusResponse::user_data_length,
        return an error.
        */

        const int max_accept_index = m_handle->handle_state.user_data_length - 1;

        if (!m_deviceReady || a_switchIndex > max_accept_index)
            return (C_ERROR);

        a_status = (m_handle->handle_state.user_data[a_switchIndex] == 0) ? false : true;

        return (C_SUCCESS);
    }

    //==============================================================================
    /*!
        This method returns status of all user switches
        [__true__ = __ON__ / __false__ = __OFF__].

        \param  a_userSwitches  Return the 32-bit binary mask of the device buttons.

        \return __true__ if the operation succeeds, __false__ otherwise.
    */
    //==============================================================================
    bool cHaplyDevices::getUserSwitches(unsigned int& a_userSwitches)
    {
        /*
        The Inverse3 handle only has 1 user switch, so we set a_userSwitches
        to only 1 or 0.
        */

        a_userSwitches = m_handle->handle_state.user_data[0];

        return C_SUCCESS;
    }

    //------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------
#endif // C_ENABLE_CUSTOM_DEVICE_SUPPORT
//------------------------------------------------------------------------------
