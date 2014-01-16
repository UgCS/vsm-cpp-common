// Copyright (c) 2014, Smart Projects Holdings Ltd
// All rights reserved.
// See LICENSE file for license details.

/**
 * @file mavlink_vehicle.h
 */
#ifndef _MAVLINK_VEHICLE_H_
#define _MAVLINK_VEHICLE_H_

#include <vsm/vsm.h>
#include <sstream>

/** Mavlink compatible vehicle class. It contains the functionality which
 * is considered to be "common" for all Mavlink compatible vehicles. Ardupilot
 * and ArDrone are the most typical examples. Autopilot/Vehicle specific
 * functionality is expected to be implemented in derived classes by linking
 * together existing and new activities. */
class Mavlink_vehicle: public vsm::Vehicle
{
    DEFINE_COMMON_CLASS(Mavlink_vehicle, vsm::Vehicle)
public:
    template<typename... Args>
    Mavlink_vehicle(
            vsm::Mavlink_demuxer::System_id system_id,
            vsm::Mavlink_demuxer::Component_id component_id,
            vsm::mavlink::MAV_TYPE type,
            vsm::mavlink::MAV_AUTOPILOT autopilot,
            vsm::Io_stream::Ref stream,
            size_t forced_max_read,
            Args &&... args) :
    vsm::Vehicle(type, autopilot,
            std::forward<Args>(args)...),
            real_system_id(system_id),
            real_component_id(component_id),
            forced_max_read(forced_max_read),
            mav_stream(vsm::Mavlink_stream::Create(stream, vsm::mavlink::apm::Extension::Get())),
            heartbeat(*this),
            statistics(*this),
            read_parameters(*this),
            write_parameters(*this),
            read_waypoints(*this),
            telemetry(*this),
            clear_all_missions(*this),
            mission_upload(*this)
    {
        ASSERT(real_system_id != vsm::Mavlink_demuxer::SYSTEM_ID_ANY);
    }

protected:

    /**
     * Use these values if ucs did not specify correct acceptance radius.
     */
    constexpr static double ACCEPTANCE_RADIUS_DEFAULT = 3.0;
    constexpr static double ACCEPTANCE_RADIUS_MIN = 1;
    constexpr static double ACCEPTANCE_RADIUS_MAX = 100.0;

    /** System ID of a VSM itself. Thats is the value seen by vehicle. Value
     * to be defined by a subclass. */
    const static vsm::Mavlink_demuxer::System_id VSM_SYSTEM_ID;

    /** Component ID of VSM. Use this as source component_id in all messages
     * from vsm to vehicle. Value to be defined by a subclass.*/
    const static vsm::Mavlink_demuxer::Component_id VSM_COMPONENT_ID;

    /** Write operations timeout. */
    constexpr static std::chrono::seconds WRITE_TIMEOUT =
            std::chrono::seconds(60);

    /** Enable handler. */
    virtual void
    On_enable() override;

    /** Disable handler. */
    virtual void
    On_disable() override;

    /** Enable handler in vehicle context. */
    void
    On_enable_vehicle(vsm::Request::Ptr);

    /** Disable handler in vehicle context. */
    void
    On_disable_vehicle(vsm::Request::Ptr);

    /** Reset state machine to initial state and start vehicle waiting. */
    void
    Wait_for_vehicle();

    bool
    Default_mavlink_handler(vsm::mavlink::MESSAGE_ID_TYPE, vsm::mavlink::System_id, uint8_t);

    /** Read handler for the data recieved from the vehicle. */
    void
    On_read_handler(vsm::Io_buffer::Ptr, vsm::Io_result);

    /** Schedule next read operation from the vehicle. */
    void
    Schedule_next_read();

    /** Disable all vehicle activities. */
    void
    Disable_activities();

    /** Invoked when write operation to the vehicle has timed out.
     */
    void
    Write_to_vehicle_timed_out(
            const vsm::Operation_waiter::Ptr&,
            vsm::Mavlink_stream::Weak_ptr);

    /** Real system id of the vehicle, i.e. the physical vehicle which is
     * available through the Mavlink stream. It has nothing to do with
     * the system id visible to UCS server.
     */
    const vsm::Mavlink_demuxer::System_id real_system_id;

    /** This is the component which sent hearbeat to VSM.
     * VSM uses this as target component for all mavlink messges
     * it sends to vehicle.
     */
    const vsm::Mavlink_demuxer::Component_id real_component_id;

    /** Forced maximum read size for associated stream. Zero if force is
     * no in use. */
    const size_t forced_max_read;

    /** Mavlink streams towards the vehicle. */
    vsm::Mavlink_stream::Ptr mav_stream;

    /** Current Mavlink read operation. */
    vsm::Operation_waiter read_op;

    /** Last connect time of the vehicle. */
    std::chrono::steady_clock::time_point last_connect;

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
        typedef vsm::Callback_proxy<void, bool> Next_action;

        /** Convenience builder for next action callback. Failure by default. */
        DEFINE_CALLBACK_BUILDER(Make_next_action, (bool), (false))

        Activity(Mavlink_vehicle& vehicle) :
            vehicle(vehicle)
        {
            /* Vehicle known about all its activities. */
            vehicle.activities.push_back(this);
        }

        virtual
        ~Activity() {}

        /** Disable the activity. */
        void
        Disable();

        /** Disable event to be overridden by a subclass, if necessary. */
        virtual void
        On_disable() {};

        /** Send next action to execute. */
        void
        Set_next_action(Next_action next_action);

        /** Call next action, if any. Activity is disabled before calling
         * next action handler. */
        void
        Call_next_action(bool success);

        /**
         * Helper for registration of vehicle Mavlink message handlers. System
         * id is taken from the vehicle. Handler is called from the vehicle
         * processing context.
         * @param callable Message handler, i.e. method of the activity which
         * meets Mavlink demuxer message handler requirements.
         * @param pthis Pointer to activity instance.
         * @param component_id Component id of the sender.
         * @param system_id System id of the sender. If not specified, then real
         * system id of the vehicle is used.
         * @param args vsm::Optional arguments for the Message handler.
         */

        template<vsm::mavlink::MESSAGE_ID_TYPE msg_id, class Extention_type = vsm::mavlink::Extension,
                class Callable, class This, typename... Args>
        vsm::Mavlink_demuxer::Key
        Register_mavlink_handler(Callable &&callable, This *pthis,
                vsm::Mavlink_demuxer::Component_id component_id,
                Args&& ...args,
                vsm::Optional<vsm::Mavlink_demuxer::System_id> system_id =
                        vsm::Optional<vsm::Mavlink_demuxer::System_id>()                        )
        {
            auto key =
                vehicle.mav_stream->Get_demuxer().Register_handler<msg_id, Extention_type>(
                        vsm::Make_mavlink_demuxer_handler<msg_id, Extention_type>(
                                std::forward<Callable>(callable),
                                pthis, std::forward<Args>(args)...),
                                system_id ? *system_id : vehicle.real_system_id,
                                component_id);
            registered_handlers.insert(key);
            return key;
        }

        /**
         * Helper for unregistration of vehicle Mavlink message handlers. System
         * id is taken from the vehicle.
         * @param msg_id Mavlink message id for unregistration.
         * @param component_id Component id for unregistration.
         */
        void
        Unregister_mavlink_handler(vsm::Mavlink_demuxer::Key& key)
        {
            registered_handlers.erase(key);
            vehicle.mav_stream->Get_demuxer().Unregister_handler(key);
        }

        /** Unregister all currently registered Mavlink message handlers. */
        void
        Unregister_mavlink_handlers()
        {
            for(auto& key: registered_handlers) {
                auto copy = key;
                vehicle.mav_stream->Get_demuxer().Unregister_handler(copy);
            }
            registered_handlers.clear();
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

        /** Send Mavlink message to the vehicle. */
        void
        Send_message(
                const vsm::mavlink::Payload_base& payload,
                vsm::mavlink::System_id system_id = VSM_SYSTEM_ID,
                uint8_t component_id = VSM_COMPONENT_ID)
        {
            vehicle.mav_stream->Send_message(
                    payload,
                    system_id,
                    component_id,
                    Mavlink_vehicle::WRITE_TIMEOUT,
                    Make_timeout_callback(
                            &Mavlink_vehicle::Write_to_vehicle_timed_out,
                            &vehicle,
                            vehicle.mav_stream),
                    vehicle.Get_completion_ctx());
        }

        /** Managed vehicle. */
        Mavlink_vehicle& vehicle;

    private:

        /** Handlers registered in demuxer. */
        std::unordered_set<vsm::Mavlink_demuxer::Key, vsm::Mavlink_demuxer::Key::Hasher>
            registered_handlers;

        /** Next action to execute. */
        Next_action next_action;
    };

    /** List of activities of the vehicle. */
    std::list<Activity*> activities;

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
        On_heartbeat(vsm::mavlink::Message<vsm::mavlink::MESSAGE_ID::HEARTBEAT>::Ptr);

        bool
        On_timer();

        bool
        Is_system_status_ok(vsm::mavlink::MAV_STATE system_status);

        bool first_ok_received = false;

        int received_count = 0;

        /** Timer for missing heartbeats. */
        vsm::Timer_processor::Timer::Ptr timer;

    } heartbeat;

    class Statistics: public Activity {
    public:
        using Activity::Activity;

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
        On_status_text(vsm::mavlink::Message<vsm::mavlink::MESSAGE_ID::STATUSTEXT>::Ptr);

        /** Statistics timer. */
        vsm::Timer_processor::Timer::Ptr timer;

        uint64_t num_of_processed_messages = 0;
    } statistics;

    /** Data related to initial reading of parameters. */
    class Read_parameters: public Activity {
    public:

        using Activity::Activity;

        enum {
            ATTEMPTS = 3,
            RETRY_TIMEOUT = 1,
        };

        /** Start parameters reading. */
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
        On_param_value(vsm::mavlink::Message<vsm::mavlink::MESSAGE_ID::PARAM_VALUE>::Ptr);

        /** Print parameter value. */
        template<typename Value>
        void
        Print_param(typename vsm::mavlink::Message<vsm::mavlink::MESSAGE_ID::PARAM_VALUE>::Ptr& message,
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
        vsm::Timer_processor::Timer::Ptr timer;

        /** Number of read attempts left. */
        size_t attempts_left;

        /** Parameters read from the vehicle. Maps from parameter name to
         * the value.
         */
        std::unordered_map<std::string, vsm::mavlink::Pld_param_value> parameters;
    } read_parameters;

    /** Write parameters to the vehicle. */
    class Write_parameters: public Activity {
    public:

        using Activity::Activity;

        enum {
            ATTEMPTS = 3,
            RETRY_TIMEOUT = 1,
        };

        /** List of parameters. */
        typedef std::vector<vsm::mavlink::Pld_param_set> List;

        /** Start parameters writing. */
        void
        Enable(const List& parameters_to_write);

        /** Stop parameters writing. */
        virtual void
        On_disable() override;

        /** Parameter received for verification. */
        void
        On_param_value(vsm::mavlink::Message<vsm::mavlink::MESSAGE_ID::PARAM_VALUE>::Ptr);

        /** Try next parameters write attempt. */
        bool
        Try();

        /** Schedule retry timer. */
        void
        Schedule_timer();

        /** Retry timer. */
        vsm::Timer_processor::Timer::Ptr timer;

        /** Number of write attempts left. */
        size_t attempts_left;

        /** Parameters to write. */
        List parameters;
    } write_parameters;

    /** Data related to waypoints reading. */
    class Read_waypoints: public Activity {
    public:
        using Activity::Activity;

        /** Start waypoints reading. */
        void
        Enable();

        /** Stop waypoints reading. */
        virtual void
        On_disable() override;

        void
        On_count(vsm::mavlink::Message<vsm::mavlink::MESSAGE_ID::MISSION_COUNT>::Ptr);

        void
        On_item(vsm::mavlink::Message<vsm::mavlink::MESSAGE_ID::MISSION_ITEM>::Ptr);

        /** Read next waypoint. */
        void
        Read_next();

        vsm::Mavlink_demuxer::Key mission_count_handler;

        /** Number of waypoints remnainign to be read. */
        size_t waypoints_total;

        /** Current waypoint sequence number to read. */
        size_t waypoint_to_read;
    } read_waypoints;

    class Telemetry: public Activity {
    public:
        using Activity::Activity;

        /** Telemetry configuration parameters. */
        class Config {
        public:
            /** Telemetry rate in hertz. Multiple messages are sent from APM at
             * every telemetry period.
             */
            uint8_t TELEMETRY_RATE_HZ = 2;

            /** Telemetry is requested again if it was absent during this
             * amount of time. Should be considerably less frequent than
             * TELEMETRY_RATE_HZ.
             */
            std::chrono::seconds WATCHDOG_INTERVAL = std::chrono::seconds(5);

            /** Telemetry damping factor. Expected telemetry rate is multiplied
             * by this value to eliminate fluctuations around 100% quality.
             */
            double DAMPING_FACTOR = 0.85;
        } config;

        /** Defines how many times slower the link quality estimation should
         * be compared to Config::TELEMETRY_RATE_HZ.
         */
        static constexpr int ESTIMATION_RATE_MULTIPLIER = 4;

        /** Defines how many timer slower the heartbeat request should be
         * compared to Config::TELEMETRY_RATE_HZ.
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
        template<vsm::mavlink::MESSAGE_ID msg_id, typename... Args>
        void
        Register_telemetry_handler(Args&& ...args)
        {
            telemetry_message_types++;
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
        On_sys_status(vsm::mavlink::Message<vsm::mavlink::MESSAGE_ID::SYS_STATUS>::Ptr);

        void
        On_global_position_int(vsm::mavlink::Message<vsm::mavlink::MESSAGE_ID::GLOBAL_POSITION_INT>::Ptr);

        void
        On_attitude(vsm::mavlink::Message<vsm::mavlink::MESSAGE_ID::ATTITUDE>::Ptr);

        void
        On_vfr_hud(vsm::mavlink::Message<vsm::mavlink::MESSAGE_ID::VFR_HUD>::Ptr);

        void
        On_gps_raw(vsm::mavlink::Message<vsm::mavlink::MESSAGE_ID::GPS_RAW_INT>::Ptr);

        void
        On_raw_imu(vsm::mavlink::Message<vsm::mavlink::MESSAGE_ID::RAW_IMU>::Ptr);

        void
        On_scaled_pressure(vsm::mavlink::Message<vsm::mavlink::MESSAGE_ID::SCALED_PRESSURE>::Ptr);

        void
        On_radio(vsm::mavlink::Message<vsm::mavlink::apm::MESSAGE_ID::RADIO,
                                       vsm::mavlink::apm::Extension>::Ptr);

        /** Watchdog timer for telemetry receiving from vehicle. */
        vsm::Timer_processor::Timer::Ptr watchdog_timer;

        /** Heartbeat timer. */
        vsm::Timer_processor::Timer::Ptr heartbeat_timer;

        /** Link quality estimation timer. */
        vsm::Timer_processor::Timer::Ptr estimation_timer;

        /** Number of telemetry message types we are expecting to receive
         * with Config::TELEMETRY_RATE_HZ frequency. */
        size_t telemetry_message_types = 0;

        /** Indicate the presence of any telemetry message during last telemetry
         * check interval. If false, telemetry is requested again.
         */
        bool telemetry_alive = false;

        /** Number of telemetry messages received during last quality estimation
         * period. */
        size_t telemetry_messages_last = 0;

        /** Statistics from previous measurement. */
        vsm::Mavlink_decoder::Stats prev_stats;

        /** Accumulated rx errors which are not yet accounted by link quality
         * algorithm.
         */
        unsigned rx_errors_accum = 0;

        /** End-result of estimated link quality. */
        double link_quality = 0;

        /** Previous number of rx errors reported by 3DR RADIO message.
         * Used to control the difference and wrap-around. -1 means no previous
         * value is known.
         */
        int prev_rx_errors_3dr = -1;
    } telemetry;

    /** Data related to clear all missions processing. */
    class Clear_all_missions : public Activity {
    public:

        using Activity::Activity;

        /** Related constants. */
        enum {
            ATTEMPTS = 3,
            /** In seconds. */
            RETRY_TIMEOUT = 1,
        };

        /** Try to clear missions on a vehicle. */
        bool
        Try();

        /** Mission ack received for mission clear all. */
        void
        On_mission_ack(vsm::mavlink::Message<vsm::mavlink::MESSAGE_ID::MISSION_ACK>::Ptr);

        /** Enable class and start mission clearing with optional task request
         * continuation. */
        void
        Enable(const vsm::Clear_all_missions&);

        /** Disable this class and cancel any existing request. */
        virtual void
        On_disable() override;

        /** Schedule timer for retry operation. */
        void
        Schedule_timer();

        /** Information about mission clearing. */
        vsm::Clear_all_missions info;

        /** Remaining attempts towards vehicle. */
        size_t remaining_attempts = 0;

        /** Retry timer. */
        vsm::Timer_processor::Timer::Ptr timer;
    } clear_all_missions;

    /** Mavlink mission upload protocol. */
    class Mission_upload: public Activity {
    public:
        using Activity::Activity;

        enum {
            /** In seconds. */
            RETRY_TIMEOUT = 1,
            /** Number of attempts for single action. */
            ATTEMPTS_ACTION = 3,
            /** Number of attempts for the whole task. */
            ATTEMPTS_TOTAL = 30,
        };

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
        On_mission_ack(vsm::mavlink::Message<vsm::mavlink::MESSAGE_ID::MISSION_ACK>::Ptr);

        /** Mission item request. */
        void
        On_mission_request(vsm::mavlink::Message<vsm::mavlink::MESSAGE_ID::MISSION_REQUEST>::Ptr);

        /** Schedule timer for retry attempt. */
        void
        Schedule_timer();

        /** Try next attempt and decrement remaining attempts counters.
         * @return @a true if attempts left, otherwise @a false.
         */
        bool
        Next_attempt();

        /** Upload current action to vehicle. */
        void
        Upload_current_action();

        /** Mission items to be uploaded to the vehicle. */
        std::vector<vsm::mavlink::Payload_base::Ptr> mission_items;

        /** Current action being uploaded. */
        ssize_t current_action;

        /** Number of attempts left for current action. */
        size_t attempts_action_left;

        /** Number of attempts left for the whole task. */
        size_t attempts_total_left;

        /** Retry timer. */
        vsm::Timer_processor::Timer::Ptr timer;

    } mission_upload;

    /* Somewhat ugly friendship is needed to enable activities from derived
     * classes to access activities from base class.
     */
    friend class Ardupilot_vehicle;
    friend class Ardrone_vehicle;
};


#endif /* _MAVLINK_VEHICLE_H_ */
