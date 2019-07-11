// Copyright (c) 2018, Smart Projects Holdings Ltd
// All rights reserved.
// See LICENSE file for license details.

/**
 * @file mavlink_vehicle.h
 */
#ifndef _MAVLINK_VEHICLE_H_
#define _MAVLINK_VEHICLE_H_

#include <ugcs/vsm/vsm.h>
#include <ugcs/vsm/mavlink.h>
#include <stdint.h>
#include <sstream>

/** Mavlink compatible vehicle class. It contains the functionality which
 * is considered to be "common" for all Mavlink compatible vehicles. Ardupilot
 * and PX4 are the most typical examples. Autopilot/Vehicle specific
 * functionality is expected to be implemented in derived classes by linking
 * together existing and new activities. */
class Mavlink_vehicle: public ugcs::vsm::Vehicle
{
    DEFINE_COMMON_CLASS(Mavlink_vehicle, ugcs::vsm::Vehicle)
public:
    // Built in vendors.
    enum class Vendor {
        CUSTOM = 0,     // Use this when creating mavlink vehicle not in the list.
        EMULATOR = 1,
        ARDUPILOT = 2,
        PX4 = 3
    };

    // Constructor for command processor.
    Mavlink_vehicle(Vendor vendor, const std::string& autopilot_type, ugcs::vsm::proto::Vehicle_type type);

    Mavlink_vehicle(
        ugcs::vsm::Mavlink_demuxer::System_id system_id,
        ugcs::vsm::Mavlink_demuxer::Component_id component_id,
        Vendor vendor,
        ugcs::vsm::mavlink::MAV_TYPE type,
        ugcs::vsm::Mavlink_stream::Ptr stream,
        ugcs::vsm::Optional<std::string> mission_dump_path,
        const std::string& serial,
        const std::string& model,
        ugcs::vsm::Request_processor::Ptr proc,
        ugcs::vsm::Request_completion_context::Ptr comp):
            ugcs::vsm::Vehicle(ugcs::vsm::proto::DEVICE_TYPE_VEHICLE, proc, comp),
            real_system_id(system_id),
            real_component_id(component_id),
            mission_dump_path(mission_dump_path),
            mav_stream(stream),
            vehicle_vendor(vendor),
            common_handlers(*this),
            heartbeat(*this),
            statistics(*this),
            read_parameters(*this),
            read_string_parameters(*this),
            read_version(*this),
            do_commands(*this),
            write_parameters(*this),
            read_waypoints(*this),
            telemetry(*this),
            mission_upload(*this)
    {
        Set_port_name(stream->Get_stream()->Get_name());
        Set_serial_number(serial);
        Set_model_name(model);

        // Deduct vehicle type and frame type from MAV_TYPE.
        switch (type) {
        case ugcs::vsm::mavlink::MAV_TYPE_HEXAROTOR:
            Set_frame_type("generic_hexa_x");
            Set_vehicle_type(ugcs::vsm::proto::VEHICLE_TYPE_MULTICOPTER);
            break;
        case ugcs::vsm::mavlink::MAV_TYPE_OCTOROTOR:
            Set_frame_type("generic_octa_v");
            Set_vehicle_type(ugcs::vsm::proto::VEHICLE_TYPE_MULTICOPTER);
            break;
        case ugcs::vsm::mavlink::MAV_TYPE_QUADROTOR:
            Set_frame_type("generic_quad_x");
            Set_vehicle_type(ugcs::vsm::proto::VEHICLE_TYPE_MULTICOPTER);
            break;
        case ugcs::vsm::mavlink::MAV_TYPE_TRICOPTER:
            Set_frame_type("generic_tri_y");
            Set_vehicle_type(ugcs::vsm::proto::VEHICLE_TYPE_MULTICOPTER);
            break;
        case ugcs::vsm::mavlink::MAV_TYPE_HELICOPTER:
        case ugcs::vsm::mavlink::MAV_TYPE_COAXIAL:
            Set_frame_type("generic_heli");
            Set_vehicle_type(ugcs::vsm::proto::VEHICLE_TYPE_HELICOPTER);
            break;
        case ugcs::vsm::mavlink::MAV_TYPE_FIXED_WING:
        case ugcs::vsm::mavlink::MAV_TYPE_AIRSHIP:
            Set_frame_type("generic_fixed_wing");
            Set_vehicle_type(ugcs::vsm::proto::VEHICLE_TYPE_FIXED_WING);
            break;
        case ugcs::vsm::mavlink::MAV_TYPE_GROUND_ROVER:
            Set_vehicle_type(ugcs::vsm::proto::VEHICLE_TYPE_GROUND);
            break;
        case ugcs::vsm::mavlink::MAV_TYPE_VTOL_DUOROTOR:
            Set_frame_type("generic_vtol_duo");
            Set_vehicle_type(ugcs::vsm::proto::VEHICLE_TYPE_VTOL);
            break;
        case ugcs::vsm::mavlink::MAV_TYPE_VTOL_QUADROTOR:
            Set_frame_type("generic_vtol_quad");
            Set_vehicle_type(ugcs::vsm::proto::VEHICLE_TYPE_VTOL);
            break;
        case ugcs::vsm::mavlink::MAV_TYPE_VTOL_TILTROTOR:
            Set_frame_type("generic_tilt_rotor");
            Set_vehicle_type(ugcs::vsm::proto::VEHICLE_TYPE_VTOL);
            break;
        case ugcs::vsm::mavlink::MAV_TYPE_VTOL_RESERVED2:
        case ugcs::vsm::mavlink::MAV_TYPE_VTOL_RESERVED3:
        case ugcs::vsm::mavlink::MAV_TYPE_VTOL_RESERVED4:
        case ugcs::vsm::mavlink::MAV_TYPE_VTOL_RESERVED5:
            Set_frame_type("generic_vtol");
            Set_vehicle_type(ugcs::vsm::proto::VEHICLE_TYPE_VTOL);
            break;
        default:
            LOG_ERR("Could not deduct vehicle type and frame from mav_type: %d", type);
        }

        VEHICLE_LOG_WRN(
            *this,
            "New mavlink Vehicle type=%d, default frame=%s",
            Get_vehicle_type(),
            Get_frame_type().c_str());

        t_servo_pwm_1 = flight_controller->Add_telemetry("servo_pwm_1", ugcs::vsm::proto::FIELD_SEMANTIC_NUMERIC);
        t_servo_pwm_2 = flight_controller->Add_telemetry("servo_pwm_2", ugcs::vsm::proto::FIELD_SEMANTIC_NUMERIC);
        t_servo_pwm_3 = flight_controller->Add_telemetry("servo_pwm_3", ugcs::vsm::proto::FIELD_SEMANTIC_NUMERIC);
        t_servo_pwm_4 = flight_controller->Add_telemetry("servo_pwm_4", ugcs::vsm::proto::FIELD_SEMANTIC_NUMERIC);
        t_servo_pwm_5 = flight_controller->Add_telemetry("servo_pwm_5", ugcs::vsm::proto::FIELD_SEMANTIC_NUMERIC);
        t_servo_pwm_6 = flight_controller->Add_telemetry("servo_pwm_6", ugcs::vsm::proto::FIELD_SEMANTIC_NUMERIC);
        t_servo_pwm_7 = flight_controller->Add_telemetry("servo_pwm_7", ugcs::vsm::proto::FIELD_SEMANTIC_NUMERIC);
        t_servo_pwm_8 = flight_controller->Add_telemetry("servo_pwm_8", ugcs::vsm::proto::FIELD_SEMANTIC_NUMERIC);
        t_servo_pwm_9 = flight_controller->Add_telemetry("servo_pwm_9", ugcs::vsm::proto::FIELD_SEMANTIC_NUMERIC);
        t_servo_pwm_10 = flight_controller->Add_telemetry("servo_pwm_10", ugcs::vsm::proto::FIELD_SEMANTIC_NUMERIC);
        t_servo_pwm_11 = flight_controller->Add_telemetry("servo_pwm_11", ugcs::vsm::proto::FIELD_SEMANTIC_NUMERIC);
        t_servo_pwm_12 = flight_controller->Add_telemetry("servo_pwm_12", ugcs::vsm::proto::FIELD_SEMANTIC_NUMERIC);
        t_servo_pwm_13 = flight_controller->Add_telemetry("servo_pwm_13", ugcs::vsm::proto::FIELD_SEMANTIC_NUMERIC);
        t_servo_pwm_14 = flight_controller->Add_telemetry("servo_pwm_14", ugcs::vsm::proto::FIELD_SEMANTIC_NUMERIC);
        t_servo_pwm_15 = flight_controller->Add_telemetry("servo_pwm_15", ugcs::vsm::proto::FIELD_SEMANTIC_NUMERIC);
        t_servo_pwm_16 = flight_controller->Add_telemetry("servo_pwm_16", ugcs::vsm::proto::FIELD_SEMANTIC_NUMERIC);

        t_vibration_x = flight_controller->Add_telemetry("vibration_x", ugcs::vsm::proto::FIELD_SEMANTIC_NUMERIC);
        t_vibration_y = flight_controller->Add_telemetry("vibration_y", ugcs::vsm::proto::FIELD_SEMANTIC_NUMERIC);
        t_vibration_z = flight_controller->Add_telemetry("vibration_z", ugcs::vsm::proto::FIELD_SEMANTIC_NUMERIC);
    }

    /** System ID of a VSM itself. */
    ugcs::vsm::Mavlink_demuxer::System_id vsm_system_id = 1;

    /** Component ID of VSM. Use this as source component_id in all messages
     * from vsm to vehicle. Value to be defined by a subclass.*/
    static constexpr ugcs::vsm::Mavlink_demuxer::Component_id VSM_COMPONENT_ID = ugcs::vsm::mavlink::MAV_COMP_ID_MISSIONPLANNER;

    /** Write operations timeout. */
    constexpr static std::chrono::seconds
    WRITE_TIMEOUT = std::chrono::seconds(180);

    bool
    Is_mission_upload_active();

    void
    Inject_message(
        ugcs::vsm::mavlink::MESSAGE_ID_TYPE message_id,
        uint8_t sys,
        uint8_t cmp,
        ugcs::vsm::Io_buffer::Ptr buf);

    // Return true if heartbeat is vrom vehicle.
    // Return false if heartbeat is from gimbal, GCS, etc...
    static bool
    Is_vehicle_heartbeat_valid(ugcs::vsm::mavlink::Message<ugcs::vsm::mavlink::MESSAGE_ID::HEARTBEAT>::Ptr message);

    // Helper to tell vehicle manager to remove from vehicle list.
    bool is_active = true;

protected:
    void
    Send_message(const ugcs::vsm::mavlink::Payload_base& payload);

    void
    Send_message_v1(const ugcs::vsm::mavlink::Payload_base& payload);

    void
    Send_message_v2(const ugcs::vsm::mavlink::Payload_base& payload);

    /** Custom telemetry for mavlink vehicles*/
    ugcs::vsm::Property::Ptr t_servo_pwm_1 = nullptr;
    ugcs::vsm::Property::Ptr t_servo_pwm_2 = nullptr;
    ugcs::vsm::Property::Ptr t_servo_pwm_3 = nullptr;
    ugcs::vsm::Property::Ptr t_servo_pwm_4 = nullptr;
    ugcs::vsm::Property::Ptr t_servo_pwm_5 = nullptr;
    ugcs::vsm::Property::Ptr t_servo_pwm_6 = nullptr;
    ugcs::vsm::Property::Ptr t_servo_pwm_7 = nullptr;
    ugcs::vsm::Property::Ptr t_servo_pwm_8 = nullptr;
    ugcs::vsm::Property::Ptr t_servo_pwm_9 = nullptr;
    ugcs::vsm::Property::Ptr t_servo_pwm_10 = nullptr;
    ugcs::vsm::Property::Ptr t_servo_pwm_11 = nullptr;
    ugcs::vsm::Property::Ptr t_servo_pwm_12 = nullptr;
    ugcs::vsm::Property::Ptr t_servo_pwm_13 = nullptr;
    ugcs::vsm::Property::Ptr t_servo_pwm_14 = nullptr;
    ugcs::vsm::Property::Ptr t_servo_pwm_15 = nullptr;
    ugcs::vsm::Property::Ptr t_servo_pwm_16 = nullptr;

    /** How much reported vehicle clock can differ to drop altitude origin */
    static constexpr std::chrono::milliseconds
    ALTITUDE_ORIGIN_RESET_TRESHOLD = std::chrono::seconds(30);

    /** Saved vehicle boot time from previous telemetry data */
    std::chrono::time_point<std::chrono::steady_clock>
    last_known_vehicle_boot_time;

    /** to differentiate first time we got boot time from vehicle */
    bool last_known_vehicle_boot_time_known = false;

    // Allow vsm to report relative altitude. Used for ardupilot 3.3.1+ to report only amsl altitudes.
    bool report_relative_altitude = true;

    int current_sensor_health = 0;
    int current_sensor_enabled = 0;
    int current_sensors_present = 0;

    int current_autopilot_status = -1;

    // Return names of currenly failed sensors.
    // Return empty string if all sensors are healthy.
    std::string
    Get_failed_sensor_report();

    /** Uses the above variables to detect recent boot and
     * reset altitude origin.
     */
    void
    Update_boot_time(std::chrono::milliseconds);

    /** Enable handler. */
    virtual void
    On_enable() override;

    /** Disable handler. */
    virtual void
    On_disable() override;

    /** Reset state machine to initial state and start vehicle waiting. */
    void
    Wait_for_vehicle();

    /** Get the vendor of vehicle */
    Vendor
    Get_vendor() const;

    /** Read handler for the data recieved from the vehicle. */
    void
    On_read_handler(ugcs::vsm::Io_buffer::Ptr, ugcs::vsm::Io_result);

    /** Schedule next read operation from the vehicle. */
    void
    Schedule_next_read();

    /** Disable all vehicle activities. */
    void
    Disable_activities();

    /** Process heartbeat message by setting system status according to it. */
    virtual void
    Process_heartbeat(
            ugcs::vsm::mavlink::Message<ugcs::vsm::mavlink::MESSAGE_ID::HEARTBEAT>::Ptr);

    /** Vehicle specific telemetry initialization.
     * Default initializer can be used if vehicle has the same telemetry schema as ardupilot 3.3 */
    virtual void
    Initialize_telemetry();

    void
    Set_message_interval(ugcs::vsm::mavlink::MESSAGE_ID_TYPE id, int interval);

    /** Get flight base mode. */
    uint8_t
    Get_base_mode() {
        return base_mode;
    }

    /** Invoked when write operation to the vehicle has timed out.
     */
    void
    Write_to_vehicle_timed_out(
            const ugcs::vsm::Operation_waiter::Ptr&,
            ugcs::vsm::Mavlink_stream::Weak_ptr);

    static const std::string
    Mav_result_to_string(int);

    static const std::string
    Mav_mission_result_to_string(int);

    /** Vehicle can override it to support setting parameters from config file on connect.
     * Perform name and value range checking before setting parameter on the vehicle.
     * @param name Name of parameter as defined by the autopilot.
     * @param value parameter value. Float or integer.
     * @param type [OUT] Type of the parameter to set when sending PARAM_SET.
     *
     * @return @a true if parameter is ok to be set, otherwise @a false.
     *
     */
    virtual bool
    Verify_parameter(const std::string& name, float value, ugcs::vsm::mavlink::MAV_PARAM_TYPE& type);

    /** Read mavlink parameters from properties and set them on the vehicle connect.
     * Calls Verify_parameter on each parameter before sending it to vehicle.
     * @param prefix parameter name prefix. E.g. "vehicle.parameters"
     */
    void
    Set_parameters_from_properties(const std::string& prefix);

    /** Real system id of the vehicle, i.e. the physical vehicle which is
     * available through the Mavlink stream. It has nothing to do with
     * the system id visible to UCS server.
     */
    ugcs::vsm::Mavlink_demuxer::System_id real_system_id;

    /** This is the component which sent hearbeat to VSM.
     * VSM uses this as target component for all mavlink messges
     * it sends to vehicle.
     */
    ugcs::vsm::Mavlink_demuxer::Component_id real_component_id;

    /** Path for mission dumping. */
    ugcs::vsm::Optional<std::string> mission_dump_path;

    /** Mavlink streams towards the vehicle. */
    ugcs::vsm::Mavlink_stream::Ptr mav_stream;

    // Vendors have some differences of mavlink implementation. Use this to handle them.
    Vendor vehicle_vendor;

    uint8_t base_mode = 0;

    /** Telemetry rate in hertz. Multiple messages are sent from APM at
     * every telemetry period.
     * Default is 2 messages per second.
     */
    static constexpr float DEFAULT_TELEMETRY_RATE = 2.0;

    uint8_t telemetry_rate_hz = DEFAULT_TELEMETRY_RATE;

    /** Number of telemetry message types we are expecting to receive per second. */
    float expected_telemetry_rate = 0;

    ugcs::vsm::Vehicle::Command_map current_command_map;

    // Seconds allowed for time since boot to differ before reconnecting the vehicle.
    // If time_boot_ms difference between two consecutive GLOBAL_POSITION_INT messages
    // is more than this then consider that vehicle should be recreated.
    static constexpr float VEHICLE_RESET_TIME_DIFFERENCE = 2.0;

    bool
    Is_armed() {
        return (base_mode & ugcs::vsm::mavlink::MAV_MODE_FLAG::MAV_MODE_FLAG_SAFETY_ARMED);
    }

    static uint32_t
    Get_mission_item_hash(const ugcs::vsm::mavlink::Pld_mission_item& msg);

    // Generate QGC WPL format 110
    static std::string
    Generate_wpl(const ugcs::vsm::mavlink::Payload_list& messages, bool use_crlf);

    // Forward declaration to make sure list is initialized before default_handlers.
    class Activity;
    /** List of activities of the vehicle. */
    std::list<Activity*> activities;

    /** Represents an activity ongoing with a vehicle. This is mostly a
     * convenience class to separate activity-related methods and members.
     * Activity can be enabled or disabled. At any given moment of timer,
     * there could be any number of activities enabled and there could be also
     * activity enabling/disabling dependencies controlled by the vehicle itself.
     */
    class Activity {
    public:
        /** Handler for a next action to execute when activity finishes.
         * Parameter denotes whether activity has finished successfully (true)
         * or not (false). */
        typedef ugcs::vsm::Callback_proxy<void, bool, std::string> Next_action;

        /** Convenience builder for next action callback. Failure by default. */
        DEFINE_CALLBACK_BUILDER(Make_next_action, (bool, std::string), (false, std::string()));

        const int try_count;
        const std::chrono::milliseconds retry_timeout;
        const std::chrono::milliseconds extended_retry_timeout;

        Activity(Mavlink_vehicle& vehicle) :
            try_count(vehicle.command_try_count),
            retry_timeout(vehicle.command_timeout),
            extended_retry_timeout(vehicle.command_timeout * 3),
            vehicle(vehicle)
        {
            /* Vehicle knows about all its activities. */
            vehicle.activities.push_back(this);
        }

        virtual
        ~Activity() {}

        /** Disable the activity. */
        void
        Disable();

        /** Disable the activity. */
        void
        Disable(const std::string& status);

        void
        Disable_success();

        /** Disable event to be overridden by a subclass, if necessary. */
        virtual void
        On_disable() {}

        /** Send next action to execute. */
        void
        Set_next_action(Next_action next_action);

        /** Call next action, if any. Activity is disabled before calling
         * next action handler. */
        void
        Call_next_action(bool success, const std::string& msg = std::string());

        /**
         * Helper for registration of vehicle Mavlink message handlers. System
         * id is taken from the vehicle. Handler is called from the vehicle
         * processing context.
         * @param callable Message handler, i.e. method of the activity which
         * meets Mavlink demuxer message handler requirements.
         * @param pthis Pointer to activity instance.
         * @param args ugcs::vsm::Optional arguments for the Message handler.
         * @param component_id Component id of the sender defaults to COMPONENT_ID_ANY.
         * @param system_id System id of the sender. If not specified, then real
         * system id of the vehicle is used.
         */

        template<ugcs::vsm::mavlink::MESSAGE_ID_TYPE msg_id, class Extention_type = ugcs::vsm::mavlink::Extension,
                class Callable, class This, typename... Args>
        ugcs::vsm::Mavlink_demuxer::Key
        Register_mavlink_handler(Callable &&callable, This *pthis,
                Args&& ...args,
                ugcs::vsm::Mavlink_demuxer::Component_id component_id = ugcs::vsm::Mavlink_demuxer::COMPONENT_ID_ANY,
                ugcs::vsm::Optional<ugcs::vsm::Mavlink_demuxer::System_id> system_id =
                        ugcs::vsm::Optional<ugcs::vsm::Mavlink_demuxer::System_id>()                        )
        {
            auto key =
                vehicle.mav_stream->Get_demuxer().Register_handler<msg_id, Extention_type>(
                    ugcs::vsm::Mavlink_demuxer::Make_handler<msg_id, Extention_type>(
                        std::forward<Callable>(callable),
                        pthis, std::forward<Args>(args)...),
                        system_id ? *system_id : vehicle.real_system_id,
                        component_id);
            registered_handlers.push_back(key);
            return key;
        }

        /**
         * Fill Mavlink target system and component ids. System id is always
         * vehicle real system id.
         */
        template<class Mavlink_payload>
        void
        Fill_target_ids(Mavlink_payload& message, uint8_t comp_id)
        {
            message->target_system = vehicle.real_system_id;
            message->target_component = comp_id;
        }

        /**
         * Fill Mavlink target system and component ids. System id is always
         * vehicle real system id.
         */
        template<class Mavlink_payload>
        void
        Fill_target_ids(Mavlink_payload& message)
        {
            message->target_system = vehicle.real_system_id;
            message->target_component = vehicle.real_component_id;
        }

        /**
         * Fill Mavlink target system id. System id is always vehicle real
         * system id.
         */
        template<class Mavlink_payload>
        void
        Fill_target_system_id(Mavlink_payload& message)
        {
            message->target_system = vehicle.real_system_id;
        }

        /** Send Mavlink message to the vehicle. */
        void
        Send_message(const ugcs::vsm::mavlink::Payload_base& payload)
        {
            vehicle.mav_stream->Send_message(
                    payload,
                    vehicle.vsm_system_id,
                    VSM_COMPONENT_ID,
                    Mavlink_vehicle::WRITE_TIMEOUT,
                    Make_timeout_callback(
                            &Mavlink_vehicle::Write_to_vehicle_timed_out,
                            &vehicle,
                            vehicle.mav_stream),
                    vehicle.Get_completion_ctx());
        }

        /** Managed vehicle. */
        Mavlink_vehicle& vehicle;

        /** new style command request. */
        ugcs::vsm::Ucs_request::Ptr ucs_request = nullptr;

    private:
        /** Handlers registered in demuxer. */
        std::vector<ugcs::vsm::Mavlink_demuxer::Key>
            registered_handlers;

        /** Next action to execute. */
        Next_action next_action;
    } common_handlers;

    /** Heartbeat receiving activity. */
    class Heartbeat: public Activity {
    public:
        using Activity::Activity;

        enum {
            /** Maximum interval for receiving a heartbeat message. If heartbeat
             * message is not received during this interval, vehicle is
             * considered unreachable/disconnected.
             */
            MAX_INTERVAL = 5 /** Seconds. */,
        };

        /** Enable heartbeat receiving. */
        void
        Enable();

        /** Disable heartbeat receiving. */
        virtual void
        On_disable() override;

        void
        On_heartbeat(ugcs::vsm::mavlink::Message<ugcs::vsm::mavlink::MESSAGE_ID::HEARTBEAT>::Ptr);

        bool
        On_timer();

        bool first_ok_received = false;

        int received_count = 0;

        /** Timer for missing heartbeats. */
        ugcs::vsm::Timer_processor::Timer::Ptr timer;
    } heartbeat;

    class Statistics: public Activity {
    public:
        using Activity::Activity;

        /** Handler for status text. */
        typedef ugcs::vsm::Callback_proxy<
                void,
                ugcs::vsm::mavlink::Message<ugcs::vsm::mavlink::MESSAGE_ID::STATUSTEXT>::Ptr>
            Statustext_handler;

        /** Convenience builder. */
        DEFINE_CALLBACK_BUILDER(
                Make_statustext_handler,
                (ugcs::vsm::mavlink::Message<ugcs::vsm::mavlink::MESSAGE_ID::STATUSTEXT>::Ptr),
                (nullptr));

        enum {
            /** Seconds. */
            COLLECTION_INTERVAL = 10,
        };

        /** Enable statistics. */
        void
        Enable();

        /** Disable statistics. */
        virtual void
        On_disable() override;

        /** Timer handler. */
        bool
        On_timer();

        void
        On_status_text(ugcs::vsm::mavlink::Message<ugcs::vsm::mavlink::MESSAGE_ID::STATUSTEXT>::Ptr);

        /** Optional handler. */
        Statustext_handler statustext_handler;

        /** Statistics timer. */
        ugcs::vsm::Timer_processor::Timer::Ptr timer;

        uint64_t num_of_processed_messages = 0;
        uint64_t num_of_csum_errors = 0;
    } statistics;

    class Read_parameters: public Activity {
    public:
        using Activity::Activity;

        /** Handler for status text. */
        typedef ugcs::vsm::Callback_proxy<
                void,
                ugcs::vsm::mavlink::Pld_param_value>
            Parameter_handler;

        /** Convenience builder. */
        DEFINE_CALLBACK_BUILDER(
                Make_parameter_handler,
                (ugcs::vsm::mavlink::Pld_param_value),
                (ugcs::vsm::mavlink::Pld_param_value()));

        /** Start parameters reading.
         * @param names set of parameter names to retrieve.
         * If names not given - read all parameters. */
        void
        Enable(std::unordered_set<std::string> names);

        /** Stop parameters reading. */
        virtual void
        On_disable() override;

        /** Try next parameters read attempt. */
        bool
        Try();

        /** true if activity is currently active */
        bool
        Is_enabled();

        /** Schedule retry timer. */
        void
        Schedule_timer();

        /** Parameter received. */
        void
        On_param_value(ugcs::vsm::mavlink::Message<ugcs::vsm::mavlink::MESSAGE_ID::PARAM_VALUE>::Ptr);

        /** Print parameter value. */
        template<typename Value>
        void
        Print_param(
            const typename ugcs::vsm::mavlink::Message<ugcs::vsm::mavlink::MESSAGE_ID::PARAM_VALUE>::Ptr& message,
            Value value)
        {
            std::stringstream buf;
            buf << value;
            LOG_INFO("PARAM [%s:%d (%d)] = %s",
                    message->payload->param_id.Get_string().c_str(),
                    message->payload->param_index.Get(),
                    message->payload->param_type.Get(),
                    buf.str().c_str());
        }

        /** Retry timer. */
        ugcs::vsm::Timer_processor::Timer::Ptr timer;

        /** Number of read attempts left. */
        size_t attempts_left;

        /** Optional handler. Called on  each retrieved parameter. */
        Parameter_handler parameter_handler;

        std::unordered_set<std::string> param_names;
    } read_parameters;

    class Read_string_parameters: public Activity {
    public:
        using Activity::Activity;

        /** Handler for status text. */
        typedef ugcs::vsm::Callback_proxy<
                void,
                ugcs::vsm::mavlink::sph::Pld_param_str_value>
            Parameter_handler;

        /** Convenience builder. */
        DEFINE_CALLBACK_BUILDER(
                Make_parameter_handler,
                (ugcs::vsm::mavlink::sph::Pld_param_str_value),
                (ugcs::vsm::mavlink::sph::Pld_param_str_value()));

        /** Start parameters reading.
         * @param names set of parameter names to retrieve.
         * If names not given - read all parameters. */
        void
        Enable(std::unordered_set<std::string> names);

        /** Stop parameters reading. */
        virtual void
        On_disable() override;

        /** Try next parameters read attempt. */
        bool
        Try();

        /** Schedule retry timer. */
        void
        Schedule_timer();

        /** Parameter received. */
        void
        On_param_value(ugcs::vsm::mavlink::Message<
                ugcs::vsm::mavlink::sph::MESSAGE_ID::PARAM_STR_VALUE,
                ugcs::vsm::mavlink::sph::Extension>::Ptr);

        /** Retry timer. */
        ugcs::vsm::Timer_processor::Timer::Ptr timer;

        /** Number of read attempts left. */
        size_t attempts_left;

        /** Optional handler. Called on  each retrieved parameter. */
        Parameter_handler parameter_handler;

        std::unordered_set<std::string> param_names;
    } read_string_parameters;

    /** Data related to initial reading of parameters. */
    class Read_version: public Activity {
    public:
        using Activity::Activity;

        enum {
            // Give 1 minute to detect autopilot version.
            ATTEMPTS = 20,
            RETRY_TIMEOUT = 3,
        };

        /** Handler for status text. */
        typedef ugcs::vsm::Callback_proxy<
                void,
                ugcs::vsm::mavlink::Pld_autopilot_version>
            Version_handler;

        /** Convenience builder. */
        DEFINE_CALLBACK_BUILDER(
                Make_version_handler,
                (ugcs::vsm::mavlink::Pld_autopilot_version),
                (ugcs::vsm::mavlink::Pld_autopilot_version()));

        /** Start parameters reading.
         * @param names set of parameter names to retrieve.
         * If names not given - read all parameters. */
        void
        Enable();

        /** Stop parameters reading. */
        virtual void
        On_disable() override;

        /** Try next parameters read attempt. */
        bool
        Try();

        /** Schedule retry timer. */
        void
        Schedule_timer();

        /** Parameter received. */
        void
        On_version(ugcs::vsm::mavlink::Message<ugcs::vsm::mavlink::MESSAGE_ID::AUTOPILOT_VERSION>::Ptr);

        /** Retry timer. */
        ugcs::vsm::Timer_processor::Timer::Ptr timer;

        /** Number of read attempts left. */
        size_t attempts_left;

        /** Optional handler. Called on  each retrieved parameter. */
        Version_handler version_handler;
    } read_version;

    /** Write parameters to the vehicle. */
    class Do_command_long: public Activity {
    public:
        using Activity::Activity;

        /** List of commands. */
        typedef std::vector<ugcs::vsm::mavlink::Pld_command_long> List;

        /** Start parameters writing. */
        void
        Enable(const List& commands);

        /** Stop parameters writing. */
        virtual void
        On_disable() override;

        /** Command ack received. */
        void
        On_command_ack(ugcs::vsm::mavlink::Message<ugcs::vsm::mavlink::MESSAGE_ID::COMMAND_ACK>::Ptr);

        /** Try next parameters write attempt. */
        bool
        Try();

        /** Schedule retry timer. */
        void
        Schedule_timer();

        /** Retry timer. */
        ugcs::vsm::Timer_processor::Timer::Ptr timer;

        /** Number of write attempts left. */
        size_t attempts_left;

        /** commands to write. */
        List commands;
    } do_commands;

    /** Write parameters to the vehicle. */
    class Write_parameters: public Activity {
    public:
        Write_parameters(Mavlink_vehicle& vehicle):
            Activity(vehicle),
            parameters(vehicle.real_system_id, vehicle.real_component_id)
        {}

        /** List of parameters. */
        typedef class _List : public std::vector<ugcs::vsm::mavlink::Pld_param_set> {
        public:
            _List(ugcs::vsm::Mavlink_demuxer::System_id, ugcs::vsm::Mavlink_demuxer::Component_id);

            // Create PARAM_SET command for MAV_PARAM_TYPE_REAL32 type
            void
            Append_float(const std::string& name, float value);

            // Create PARAM_SET command for int32 type.
            // This does a static_cast so that integer value converted to float field of PARAM_SET.
            // Used for Ardupilot.
            void
            Append_int_ardu(
                const std::string& name,
                int32_t value,
                ugcs::vsm::mavlink::MAV_PARAM_TYPE type = ugcs::vsm::mavlink::MAV_PARAM_TYPE_INT32);

            // Create PARAM_SET command for int32 type
            // This does a reinterpret cast so that integer value is
            // put into float field of PARAM_SET unconverted.
            // Used for PX4.
            void
            Append_int_px4(
                const std::string& name,
                int32_t value,
                ugcs::vsm::mavlink::MAV_PARAM_TYPE type = ugcs::vsm::mavlink::MAV_PARAM_TYPE_INT32);

        private:
            ugcs::vsm::Mavlink_demuxer::System_id sysid;
            ugcs::vsm::Mavlink_demuxer::Component_id compid;
        } List;

        /** Start parameters writing. */
        void
        Enable(const List& parameters_to_write);

        /** Stop parameters writing. */
        virtual void
        On_disable() override;

        /** Parameter received for verification. */
        void
        On_param_value(ugcs::vsm::mavlink::Message<ugcs::vsm::mavlink::MESSAGE_ID::PARAM_VALUE>::Ptr);

        /** Try next parameters write attempt. */
        bool
        Try();

        /** Schedule retry timer. */
        void
        Schedule_timer();

        /** Retry timer. */
        ugcs::vsm::Timer_processor::Timer::Ptr timer;

        /** Number of write attempts left. */
        size_t attempts_left = 0;

        /** Parameters to write. */
        List parameters;
    } write_parameters;

    /** Data related to waypoints reading. */
    class Read_waypoints: public Activity {
    public:
        // This is needed to initialize home_location correctly.
        // "using Activity::Activity;" here produces compiler error.
        Read_waypoints(Mavlink_vehicle& vehicle):
            Activity(vehicle) {}

        /** Handler for status text. */
        typedef ugcs::vsm::Callback_proxy<
                void,
                ugcs::vsm::mavlink::Pld_mission_item>
            Mission_item_handler;

        /** Convenience builder. */
        DEFINE_CALLBACK_BUILDER(
                Make_mission_item_handler,
                (ugcs::vsm::mavlink::Pld_mission_item),
                (ugcs::vsm::mavlink::Pld_mission_item()));

        /** Start waypoints reading. */
        void
        Enable();

        /** On ardupilot WP0 is home location, so use this activity to retrieve HL */
        void
        Get_home_location();

        /** Stop waypoints reading. */
        virtual void
        On_disable() override;

        void
        On_count(ugcs::vsm::mavlink::Message<ugcs::vsm::mavlink::MESSAGE_ID::MISSION_COUNT>::Ptr);

        void
        On_mission_ack(ugcs::vsm::mavlink::Message<ugcs::vsm::mavlink::MESSAGE_ID::MISSION_ACK>::Ptr);

        void
        On_item(ugcs::vsm::mavlink::Message<ugcs::vsm::mavlink::MESSAGE_ID::MISSION_ITEM>::Ptr);

        /** Read waypoint count. */
        bool
        Get_count();

        /** Read next waypoint. */
        bool
        Get_next_item();

        bool
        In_progress();

        void
        Cancel_timer();

        /** Number of waypoints remnainign to be read. */
        size_t items_total;

        /** Current waypoint sequence number to read. */
        size_t item_to_read;

        Mission_item_handler item_handler;

        int retries = 0;

        /** Retry timer. */
        ugcs::vsm::Timer_processor::Timer::Ptr timer;
    } read_waypoints;

    class Telemetry: public Activity {
    public:
        using Activity::Activity;

        /** Telemetry configuration parameters. */
        class Config {
        public:
            /** Telemetry is requested again if it was absent during this
             * amount of time. Should be considerably less frequent than
             * telemetry_rate_hz.
             */
            std::chrono::seconds WATCHDOG_INTERVAL = std::chrono::seconds(5);

            /** Telemetry damping factor. Expected telemetry rate is multiplied
             * by this value to eliminate fluctuations around 100% quality.
             */
            double DAMPING_FACTOR = 0.85;
        } config;

        /** Defines how many times slower the link quality estimation should
         * be compared to telemetry_rate_hz.
         */
        static constexpr int ESTIMATION_RATE_MULTIPLIER = 4;

        /** Defines how many timer slower the heartbeat request should be
         * compared to telemetry_rate_hz.
         */
        static constexpr int HEARTBEAT_RATE_MULTIPLIER = 2;

        /** The rolling average quotient of the link quality. Defines how much
         * the newly calculated quality value affects the rolling average value.
         * The less the quotient is, the smoother (slower) is the reaction on
         * immediate quality changes.
         */
        static constexpr double QUALITY_RA_QUOT = 0.2;

        /** Register handler for the regular telemetry message. Arguments are
         * the same as for Register_mavlink_handler. Method is needed to account
         * the number of telemetry message types which in turn needed for link
         * quality estimation.
         */
        template<ugcs::vsm::mavlink::MESSAGE_ID msg_id, typename... Args>
        void
        Register_telemetry_handler(Args&& ...args)
        {
            Register_mavlink_handler<msg_id>(std::forward<Args>(args)...);
        }

        /** Start telemetry. */
        void
        Enable();

        /** Stop telemetry. */
        virtual void
        On_disable() override;

        /** Request telemetry from the vehicle if there were no telemetry during
         * last telemetry watchdog interval.
         */
        bool
        On_telemetry_check();

        /** Send heartbeat towards the vehicle in a hope to get back the
         * 3DR RADIO quality response message.
         */
        bool
        On_heartbeat();

        /** Periodically estimate the quality of the link. */
        bool
        On_estimate_link_quality();

        void
        On_sys_status(ugcs::vsm::mavlink::Message<ugcs::vsm::mavlink::MESSAGE_ID::SYS_STATUS>::Ptr);

        void
        On_global_position_int(ugcs::vsm::mavlink::Message<ugcs::vsm::mavlink::MESSAGE_ID::GLOBAL_POSITION_INT>::Ptr);

        void
        On_attitude(ugcs::vsm::mavlink::Message<ugcs::vsm::mavlink::MESSAGE_ID::ATTITUDE>::Ptr);

        void
        On_vfr_hud(ugcs::vsm::mavlink::Message<ugcs::vsm::mavlink::MESSAGE_ID::VFR_HUD>::Ptr);

        void
        On_vibration(ugcs::vsm::mavlink::Message<ugcs::vsm::mavlink::MESSAGE_ID::VIBRATION>::Ptr);

        void
        On_gps_raw(ugcs::vsm::mavlink::Message<ugcs::vsm::mavlink::MESSAGE_ID::GPS_RAW_INT>::Ptr);

        void
        On_mission_current(ugcs::vsm::mavlink::Message<ugcs::vsm::mavlink::MESSAGE_ID::MISSION_CURRENT>::Ptr);

        void
        On_target_position(
            ugcs::vsm::mavlink::Message<ugcs::vsm::mavlink::MESSAGE_ID::POSITION_TARGET_GLOBAL_INT>::Ptr);

        void
        On_radio(ugcs::vsm::mavlink::Message<ugcs::vsm::mavlink::apm::MESSAGE_ID::RADIO,
                                       ugcs::vsm::mavlink::apm::Extension>::Ptr);

        void
        On_altitude(ugcs::vsm::mavlink::Message<ugcs::vsm::mavlink::MESSAGE_ID::ALTITUDE>::Ptr);

        void
        On_servo_output_raw(ugcs::vsm::mavlink::Message<ugcs::vsm::mavlink::MESSAGE_ID::SERVO_OUTPUT_RAW>::Ptr);

        /** Watchdog timer for telemetry receiving from vehicle. */
        ugcs::vsm::Timer_processor::Timer::Ptr watchdog_timer;

        /** Heartbeat timer. */
        ugcs::vsm::Timer_processor::Timer::Ptr heartbeat_timer;

        /** Link quality estimation timer. */
        ugcs::vsm::Timer_processor::Timer::Ptr estimation_timer;

        /** Indicate the presence of any telemetry message during last telemetry
         * check interval. If false, telemetry is requested again.
         */
        bool telemetry_alive = false;

        /** Number of telemetry messages received during last quality estimation
         * period. */
        size_t telemetry_messages_last = 0;

        /** Statistics from previous measurement. */
        ugcs::vsm::Mavlink_decoder::Stats prev_stats;

        /** Accumulated rx errors which are not yet accounted by link quality
         * algorithm.
         */
        unsigned rx_errors_accum = 0;

        /** End-result of estimated link quality. */
        double link_quality = 0.5;

        /** Previous number of rx errors reported by 3DR RADIO message.
         * Used to control the difference and wrap-around. -1 means no previous
         * value is known.
         */
        int prev_rx_errors_3dr = -1;

        /** Last timestamp in GLOBAL_POSITION_INT message. used to calculate vspeed */
        double prev_time_since_boot = 0;

        /** Last altitude in GLOBAL_POSITION_INT message. used to calculate vspeed */
        double prev_altitude = 0;
    } telemetry;

    /** Mavlink mission upload protocol. */
    class Mission_upload: public Activity {
    public:
        using Activity::Activity;

        /** Handler for status text. */
        typedef ugcs::vsm::Callback_proxy<void, int>
            Mission_request_handler;

        /** Convenience builder. */
        DEFINE_CALLBACK_BUILDER(
                Make_mission_request_handler,
                (int),
                (0));

        /** return true if mission upload is active. */
        bool
        Is_active();

        /** Start mission items uploading stored in mission_items variable. */
        void
        Enable();

        /** Disable handler. */
        virtual void
        On_disable() override;

        /** Try to upload current action to vehicle. */
        bool
        Try();

        /** Mission ack received. */
        void
        On_mission_ack(ugcs::vsm::mavlink::Message<ugcs::vsm::mavlink::MESSAGE_ID::MISSION_ACK>::Ptr);

        /** Mission item request. */
        void
        On_mission_request(ugcs::vsm::mavlink::Message<ugcs::vsm::mavlink::MESSAGE_ID::MISSION_REQUEST>::Ptr);

        /** Schedule timer for retry attempt. */
        void
        Schedule_timer(std::chrono::milliseconds timeout);

        /** Try next attempt and decrement remaining attempts counters.
         * @return @a true if attempts left, otherwise @a false.
         */
        bool
        Next_attempt();

        /** Upload current action to vehicle. */
        void
        Upload_current_action();

        /** Dump mission to disk. Continue on failure. */
        void
        Dump_mission();

        /** Mission items to be uploaded to the vehicle. */
        ugcs::vsm::mavlink::Payload_list mission_items;

        /** Current action being uploaded. */
        ssize_t current_action;

        /** Number of attempts left for current action. */
        size_t attempts_action_left;

        /** true when handler for the final ack is registered. */
        bool final_ack_waiting = false;

        Mission_request_handler item_handler;

        /** Retry timer. */
        ugcs::vsm::Timer_processor::Timer::Ptr timer;
    } mission_upload;

    class Mavlink_route {
    public:
        void
        Reset();

        void
        Add_item(const ugcs::vsm::mavlink::Pld_mission_item&);

        const ugcs::vsm::mavlink::Pld_mission_item*
        Get_item_ref(int idx);

        int
        Get_item_count();

    private:
        std::unordered_map<int, ugcs::vsm::mavlink::Pld_mission_item> items;
    };

    // Return true vehicle is navigating waypoints and current command is cmd.
    bool
    Is_current_command(int cmd);

    // Return true if there is a valid mission and current command is the last one in the mission.
    bool
    Is_current_command_last();

    // Last uploaded/downloaded route items.
    Mavlink_route current_route;

    // Keep current sequence number reported via MISSION_CURRENT.
    int current_mission_item_index = -1;

    // Force use of specific mavlink version.
    ugcs::vsm::Optional<bool> use_mavlink_2;

    /* Somewhat ugly friendship is needed to enable activities from derived
     * classes to access activities from base class.
     */
    friend class Ardupilot_vehicle;
    friend class Emulator_vehicle;
    friend class Px4_vehicle;
    friend class Airmast_vehicle;

private:
    void
    Handle_inject_message(
        ugcs::vsm::mavlink::MESSAGE_ID_TYPE msg,
        uint8_t sys_id,
        uint8_t cmp_id,
        ugcs::vsm::Io_buffer::Ptr buf,
        ugcs::vsm::Request::Ptr request);
};


#endif /* _MAVLINK_VEHICLE_H_ */
