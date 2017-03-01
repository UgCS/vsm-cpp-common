// Copyright (c) 2014, Smart Projects Holdings Ltd
// All rights reserved.
// See LICENSE file for license details.

/**
 * @file mavlink_vehicle_manager.h
 */
#ifndef _MAVLINK_VEHICLE_MANAGER_H_
#define _MAVLINK_VEHICLE_MANAGER_H_

#include <ugcs/vsm/vsm.h>
#include <mavlink_vehicle.h>

/** Abstract class implementing a Mavlink vehicle manager. The main detection
 * assumption is based on Mavlink heart-beat message presence from which
 * vehicle type, system and component id are taken to create a more specific
 * Mavlink vehicle instance by a subclass.
 */
class Mavlink_vehicle_manager: public ugcs::vsm::Request_processor {
    DEFINE_COMMON_CLASS(Mavlink_vehicle_manager, ugcs::vsm::Request_processor)
public:

    /** Constructor. */
    Mavlink_vehicle_manager(
            const std::string default_model_name,
            const std::string config_prefix,
            const ugcs::vsm::mavlink::Extension& extension);

protected:

    /** Method for Mavlink vehicle creation, to be overridden by subclass. */
    virtual Mavlink_vehicle::Ptr
    Create_mavlink_vehicle(
            ugcs::vsm::Mavlink_demuxer::System_id system_id,
            ugcs::vsm::Mavlink_demuxer::Component_id component_id,
            ugcs::vsm::mavlink::MAV_TYPE type,
            ugcs::vsm::Io_stream::Ref stream,
            ugcs::vsm::Socket_address::Ptr peer_addr,
            ugcs::vsm::Optional<std::string> mission_dump_path,
            std::string serial_number,
            std::string model_name,
            bool id_overridden = false) = 0;

    /** Subclass should override this method to register On_new_connection
     * methods to the transport detector.
     */
    virtual void
    Register_detectors() = 0;

    /** Add a pattern to trigger extended detection timeout. */
    void
    Add_timeout_extension_pattern(const regex::regex&);

    /** Handler for a new transport connection. */
    void
    Handle_new_connection(
    		std::string portname,
    		int baud,
    		ugcs::vsm::Socket_address::Ptr,
    		ugcs::vsm::Io_stream::Ref,
    		ugcs::vsm::mavlink::MAV_AUTOPILOT autopilot_type,
    		bool detect_frame = false,
    		ugcs::vsm::Optional<std::string> custom_model_name =
    				ugcs::vsm::Optional<std::string>(),
    		ugcs::vsm::Optional<std::string> custom_serial_number =
    				ugcs::vsm::Optional<std::string>());

    /** Default model name to use in UCS system id calculation. */
    std::string default_model_name;

    /** Configuration file prefix. */
    std::string config_prefix;

    /** Mavlink extension used for Mavlink stream creation. */
    const ugcs::vsm::mavlink::Extension& extension;

    /** Get vehicle manager worker. */
    ugcs::vsm::Request_worker::Ptr
    Get_worker();

    /** Called when manager is disabled. To be overridden by subclass, if
     * necessary. */
    virtual void
    On_manager_disable();

private:
    /** Maximum length of accumulated raw line. */
    static constexpr size_t MAX_RAW_LINE = 1024;

    bool
    On_timer();

    /** Read config and create vehicles. */
    void
    Load_vehicle_config();

    void
    Create_vehicle_wrapper(
            Mavlink_vehicle::Mavlink_stream::Ptr mav_stream,
            ugcs::vsm::mavlink::System_id_common system_id,
            uint8_t component_id
            );

    /** Prototcol detection polling interval. */
    const std::chrono::milliseconds TIMER_INTERVAL = std::chrono::milliseconds(100);

    /** Detector should receive at least this much bytes to fail detection. */
    const unsigned int MAX_UNDETECTED_BYTES = 300;

    /** Detector should wait this much milliseconds to fail detection.
     * The interval is so big because APM takes ~4 seconds to boot when connected directly to USB*/
    const std::chrono::milliseconds DETECTOR_TIMEOUT = std::chrono::milliseconds(6000);

    /** Timeout is extended to this value when raw data patterns are matched. */
    const std::chrono::milliseconds EXTENDED_TIMEOUT = std::chrono::milliseconds(5000);

    /** Handler for the event of protocol transition to OPERATIONAL state. */
    typedef ugcs::vsm::Callback_proxy<void> Ready_handler;

    /** Detector tries to detect frame 3 times.
     * If that fails it defaults to whatever is in the HEARTBEAT message.*/
    static const unsigned int FRAME_DETECTION_RETRIES = 3;

    /** Context of the managed vehicle. */
    struct Vehicle_ctx {
        Mavlink_vehicle::Ptr vehicle;
        ugcs::vsm::Io_stream::Ref stream;
    };

    /** Managed vehicles. */
    std::unordered_map<int, Vehicle_ctx> vehicles;

    /** Preconfigured serial numbers and model names:
     * system id maps to [model name, serial number]. */
    std::unordered_map<int, std::pair<std::string, std::string> > preconfigured;

    ugcs::vsm::Request_worker::Ptr worker;

    /** Trivial detector state. */
    class Detector_ctx {
    public:

        Detector_ctx(Detector_ctx&&) = default;

        Detector_ctx(
                int timeout,
                bool detect_frame,
                ugcs::vsm::Socket_address::Ptr peer_addr,
                ugcs::vsm::Optional<std::string> custom_model,
                ugcs::vsm::Optional<std::string> custom_serial,
                ugcs::vsm::mavlink::MAV_AUTOPILOT autopilot_type):
            timeout(timeout),
            frame_detection_retries(detect_frame?FRAME_DETECTION_RETRIES:0),
            custom_model(custom_model),
            custom_serial(custom_serial),
            peer_addr(peer_addr),
            autopilot_type(autopilot_type)
            {}

        ~Detector_ctx()
        {
            read_op.Abort();
        }

        /** Timeout counter. */
        int timeout;

        unsigned int frame_detection_retries;
        ugcs::vsm::mavlink::MAV_TYPE frame_type = ugcs::vsm::mavlink::MAV_TYPE_GENERIC;

        ugcs::vsm::Optional<std::string> custom_model;
        ugcs::vsm::Optional<std::string> custom_serial;
        ugcs::vsm::Socket_address::Ptr peer_addr;
        ugcs::vsm::mavlink::MAV_AUTOPILOT autopilot_type;

        /** Current stream read operation. */
        ugcs::vsm::Operation_waiter read_op;

        /** Current line of raw data. */
        std::string curr_line;
    };

    /** Map of Mavlink streams in detecting state (not bound to vehicles) and
     * detection timeout counter.
     * Stream lives in this state for 1 second until Mavlink protocol detected
     * or detection failed.
     */
    std::unordered_map<Mavlink_vehicle::Mavlink_stream::Ptr, Detector_ctx> detectors;

    /** Watchdog timer for detection. */
    ugcs::vsm::Timer_processor::Timer::Ptr watchdog_timer;

    /** Mission dump path. */
    ugcs::vsm::Optional<std::string> mission_dump_path;

    /** Patterns which extended detection timeout. */
    std::vector<regex::regex> extension_patterns;

    void
    Write_to_vehicle_timed_out(
            const ugcs::vsm::Operation_waiter::Ptr& waiter,
            Mavlink_vehicle::Mavlink_stream::Weak_ptr mav_stream);

    void
    On_param_value(
            ugcs::vsm::mavlink::Message<ugcs::vsm::mavlink::MESSAGE_ID::PARAM_VALUE>::Ptr message,
            Mavlink_vehicle::Mavlink_stream::Ptr mav_stream);

    /** Create new or update existing vehicles based on received system id
     * and type of the vehicle. */
    void
    On_heartbeat(
            ugcs::vsm::mavlink::Message<ugcs::vsm::mavlink::MESSAGE_ID::HEARTBEAT>::Ptr message,
            Mavlink_vehicle::Mavlink_stream::Ptr mav_stream);

    /** Raw data handler from Mavlink stream. */
    void
    On_raw_data(ugcs::vsm::Io_buffer::Ptr, Mavlink_vehicle::Mavlink_stream::Ptr);

    /** Handle full line from the raw data. */
    void
    Handle_raw_line(Detector_ctx&, const Mavlink_vehicle::Mavlink_stream::Ptr&);

    void
    Schedule_next_read(Mavlink_vehicle::Mavlink_stream::Ptr);

    /** Stream read completion handler. */
    void
    On_stream_read(ugcs::vsm::Io_buffer::Ptr, ugcs::vsm::Io_result, Mavlink_vehicle::Mavlink_stream::Ptr);

    /** Enable the manager. */
    virtual void
    On_enable() override;

    /** Disable the manager. */
    virtual void
    On_disable() override;

    /** Process disable event from the processor context. */
    void
    Process_on_disable(ugcs::vsm::Request::Ptr);
};

#endif /* _MAVLINK_VEHICLE_MANAGER_H_ */
