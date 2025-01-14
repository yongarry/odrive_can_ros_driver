#ifndef ODRIVE_INTERFACE_HPP_
#define ODRIVE_INTERFACE_HPP_

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <can_msgs/Frame.h>
#include <math.h>

#define DEFAULT_UPDATE_RATE 1


namespace odrive {
    // ODrive command IDs
    enum ODriveCommandId {
        CANOPEN_NMT_MESSAGE = 0x000,
        HEARTBEAT_MESSAGE = 0x001,
        ESTOP_MESSAGE = 0x002,
        GETERROR = 0x003,
        SET_AXIS_NODE_ID = 0x006,
        SET_AXIS_REQUESTED_STATE = 0x007,
        SET_AXIS_STARTUP_CONFIG = 0x008,
        GET_ENCODER_ESTIMATE = 0x009,
        GET_ENCODER_COUNT = 0x00A,
        SET_CONTROLLER_MODES = 0x00B,
        SET_INPUT_POS = 0x00C,
        SET_INPUT_VELOCITY = 0x00D,
        SET_INPUT_TORQUE = 0x00E,
        SET_LIMITS = 0x00F,
        START_ANTI_COGGING = 0x010,
        SET_TRAJ_VEL_LIMIT = 0x011,
        SET_TRAJ_ACCEL_LIMITS = 0x012,
        SET_TRAJ_INERTIA = 0x013,
        GET_IQ = 0x014,
        GET_TEMPERATURE = 0x015,
        REBOOT_ODRIVE = 0x016,
        GET_BUS_VOLTAGE_AND_CURRENT = 0x017,
        CLEAR_ERRORS = 0x018,
        SET_ABSOLUTE_POSITION = 0x019,
        SET_POSITION_GAIN = 0x01A,
        SET_VEL_GAINS = 0x01B,
        CANOPEN_HEARTBEAT_MESSAGE = 0x700
    };

    enum ODriveAxisState {
        UNDEFINED = 0x00,
        IDLE = 0x01,
        STARTUP_SEQUENCE = 0x02,
        FULL_CALIBRATION_SEQUENCE = 0x03,
        MOTOR_CALIBRATION = 0x04,
        ENCODER_INDEX_SEARCH = 0x06,
        ENCODER_OFFSET_CALIBRATION = 0x07,
        CLOSED_LOOP_CONTROL = 0x08,
        LOCKIN_SPIN = 0x09,
        ENCODER_DIR_FIND = 0x0A,
        HOMING = 0x0B,
        ENCODER_HALL_POLARITY_CALIBRATION = 0x0C,
        ENCODER_HALL_PHASE_CALIBRATION = 0x0D
    };

    enum ODriveControlMode {
        VOLTAGE_CONTROL = 0x0,
        TORQUE_CONTROL = 0x1,
        VELOCITY_CONTROL = 0x2,
        POSITION_CONTROL = 0x3
    };

    enum ODriveInputMode {
        INACTIVE = 0x0,
        PASSTHROUGH = 0x1,
        VEL_RAMP = 0x2,
        POS_FILTER = 0x3,
        MIX_CHANNELS = 0x4,
        TRAP_TRAJ = 0x5,
        TORQUE_RAMP = 0x6,
        MIRROR= 0x7,
        TUNING= 0x8
    };

    class ODriveAxis {
        public:
            ODriveAxis(ros::NodeHandle *node, std::string axis_name, int axis_can_id, std::string direction);
            double getAxisAngle();
            double getAxisVelocity();
            double getAxisVoltage();
            double getAxisCurrent();
            void engage();
            void disengage();
            
            uint8_t axis_current_state_;
        private:
            ros::Subscriber received_messages_sub_;
            ros::Publisher sent_messages_pub_;

            ros::Subscriber target_position_sub_;
            ros::Subscriber target_velocity_sub_;
            ros::Subscriber target_torque_sub_;
            ros::Publisher output_position_pub_;
            ros::Publisher output_velocity_pub_;
            ros::Publisher output_torque_pub_;
            
            ros::Publisher axis_angle_pub_;
            ros::Publisher axis_velocity_pub_;
            ros::Publisher axis_voltage_pub_;
            ros::Publisher axis_current_pub_;

            ros::Timer axis_update_timer_;
            std::string axis_name_;
            std::string can_rx_topic_;
            std::string can_tx_topic_;
            
            int axis_can_id_;
            int direction_;
            bool engage_on_startup_;
            double update_rate_;
            double axis_min_velocity_;
            
            double axis_angle_;
            double axis_velocity_;
            double axis_voltage_;
            double axis_current_;

            
            void canReceivedMessagesCallback(const can_msgs::Frame::ConstPtr& msg);

            void positionReceivedMessagesCallback(const std_msgs::Float64::ConstPtr& msg);
            void velocityReceivedMessagesCallback(const std_msgs::Float64::ConstPtr& msg);
            void torqueReceivedMessagesCallback(const std_msgs::Float64::ConstPtr& msg);

            void updateTimerCallback(const ros::TimerEvent& event);
            void requestEncoderEstimate();
            void requestBusVoltageAndCurrent();

            void setAxisRequestedState(ODriveAxisState state);
            void setControllerModes(ODriveControlMode control_mode, ODriveInputMode input_mode);

            void setInputPosition(double position);
            void setInputVelocity(double velocity);
            void setInputTorque(double torque);

            uint32_t createCanId(int axis_can_id, int command);
    };
}

#endif // ODRIVE_INTERFACE_HPP_
