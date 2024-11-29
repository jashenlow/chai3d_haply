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
#ifndef CHaplyDevicesH
#define CHaplyDevicesH
//------------------------------------------------------------------------------
#if defined(C_ENABLE_HAPLY_DEVICE_SUPPORT)
//------------------------------------------------------------------------------
#include "devices/CGenericHapticDevice.h"
//------------------------------------------------------------------------------
#include "include/HardwareAPI.h"
#include "system/CThread.h"
#include <atomic>
#include <memory>
#include <array>
#include <mutex>

/*
NOTE:
  - To modify the polling frequencies, simply define HAPLY_HANDLE_HZ
    before including this file.
  - Communication polling rate for the Inverse3 is up to 4kHz.
  - Communication polling rate for the handle should be about 50Hz to 60Hz.
*/
#ifndef HAPLY_HANDLE_HZ
#define HAPLY_HANDLE_HZ 50
#endif // HAPLY_HANDLE_HZ

//------------------------------------------------------------------------------
namespace chai3d
{
    //------------------------------------------------------------------------------

    //==============================================================================
    /*!
        \file       CHaplyDevices.h

        \brief
        Implements support for the Haply Inverse3 haptic device.
    */
    //==============================================================================

    //------------------------------------------------------------------------------
    class cHaplyDevices;
    typedef std::shared_ptr<cHaplyDevices> cHaplyDevicesPtr;
    //------------------------------------------------------------------------------

    //==============================================================================
    /*!
        \class      cHaplyDevices
        \ingroup    devices

        \brief
        This class implements an interface to provide support for Haply Inverse3 devices.

        \details
        This class implements an interface to provide support for Haply Inverse3 devices.
    */
    //==============================================================================
    class cHaplyDevices : public cGenericHapticDevice
    {
        //--------------------------------------------------------------------------
        // CONSTRUCTOR & DESTRUCTOR:
        //--------------------------------------------------------------------------

    public:
        //! Constructor of cHaplyDevices.
        cHaplyDevices(unsigned int a_deviceNumber = 0);

        //! Destructor of cHaplyDevices.
        virtual ~cHaplyDevices();

        //! Shared cHaplyDevices allocator.
        static cHaplyDevicesPtr create(unsigned int a_deviceNumber = 0) { return (std::make_shared<cHaplyDevices>(a_deviceNumber)); }

        //--------------------------------------------------------------------------
        // PUBLIC METHODS:
        //--------------------------------------------------------------------------

    public:
        //! This method opens a connection to the haptic device.
        virtual bool open();

        //! This method closes the connection to the haptic device.
        virtual bool close();

        //! This method calibrates the haptic device.
        virtual bool calibrate(bool a_forceCalibration = false);

        //! This method returns the position of the device.
        /*!
        This MUST be called in order to do the following:
          - Get position values.
          - Get linear velocity values.
          - Set forces.
        */
        virtual bool getPosition(cVector3d &a_position);

        //! This method returns the velocity of the device.
        virtual bool getLinearVelocity(cVector3d &a_linearvelocity);

        //! This method returns the orientation frame of the device end-effector.
        virtual bool getRotation(cMatrix3d &a_rotation);

        //! This method returns the angular velocity of haptic device.
        virtual bool getAngularVelocity(cVector3d& a_angularVelocity);

        //! This method returns the gripper angle in radian [rad].
        virtual bool getGripperAngleRad(double &a_angle);

        //! This method returns the status of the handle's button state (index 0 only)
        virtual bool getUserSwitch(int a_switchIndex, bool& a_status);

        //! This method returns the status of all user switches [__true__ = __ON__ / __false__ = __OFF__].
        virtual bool getUserSwitches(unsigned int& a_userSwitches);
        
        //! This method sends a force [N] and a torque [N*m] and gripper force [N] to the haptic device.
        virtual bool setForceAndTorqueAndGripperForce(const cVector3d &a_force, const cVector3d &a_torque, double a_gripperForce);

        //! This method enables or disables forces.
        virtual bool enableForces(bool a_value);

        //--------------------------------------------------------------------------
        // PUBLIC STATIC METHODS:
        //--------------------------------------------------------------------------

    public:
        //! This method returns the number of devices available from this class of device.
        static unsigned int getNumDevices();

        //--------------------------------------------------------------------------
        // PROTECTED MEMBERS:
        //--------------------------------------------------------------------------

        ////////////////////////////////////////////////////////////////////////////
        /*
            INTERNAL VARIABLES:

            If you need to declare any local variables or methods for your device,
            you may do it here below.
        */
        ////////////////////////////////////////////////////////////////////////////

    protected:
        //! This method opens libraries for this class of devices.
        static bool openLibraries();

        bool openInverse();
        bool openHandle();

        struct Handle : public Haply::HardwareAPI::Devices::Handle
        {
            Handle(Haply::HardwareAPI::IO::SerialStream *stream);
            ~Handle();

            void start();

            Haply::HardwareAPI::Devices::Handle::HandleInfoResponse handle_info;
            Haply::HardwareAPI::Devices::Handle::HandleStatusResponse handle_state;

            std::mutex m_handle_state_lock;

        protected:
            static void run(void *);
            void OnReceiveHandleInfo(
                uint8_t, uint16_t, uint8_t, uint8_t, uint8_t) override;
            void OnReceiveHandleStatusMessage(
                uint16_t, float *, uint8_t, uint8_t, uint8_t, uint8_t *) override;
            void OnReceiveHandleErrorResponse(
                uint16_t, uint8_t) override;

            std::atomic<bool> m_exit{false};
            cThread m_thread{};
        };

        size_t m_device_index{0};

        std::unique_ptr<Haply::HardwareAPI::Devices::Inverse3> m_inverse;
        std::unique_ptr<Haply::HardwareAPI::IO::SerialStream> m_inverseStream;

        std::unique_ptr<Handle> m_handle;
        std::unique_ptr<Haply::HardwareAPI::IO::SerialStream> m_handleStream;

        Haply::HardwareAPI::Devices::Inverse3::EndEffectorForceRequest m_force;
        std::array<float, Haply::HardwareAPI::VECTOR_SIZE> m_linear_velocity;
        cMatrix3d m_rot_matrix;
        bool m_enable_forces{ true };
    };

    //------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------
#endif // C_ENABLE_CUSTOM_DEVICE_SUPPORT
//------------------------------------------------------------------------------
#endif // CHaplyDevicesH
//------------------------------------------------------------------------------
