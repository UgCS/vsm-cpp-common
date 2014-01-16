// Copyright (c) 2014, Smart Projects Holdings Ltd
// All rights reserved.
// See LICENSE file for license details.

/**
 * @file mavlink_vehicle_manager.h
 */
#ifndef _MAVLINK_VEHICLE_MANAGER_H_
#define _MAVLINK_VEHICLE_MANAGER_H_

#include <vsm/vsm.h>
#include <mavlink_vehicle.h>

/** Abstract class implementing a Mavlink vehicle manager. The main detection
 * assumption is based on Mavlink heart-beat message presence from which
 * vehicle type, system and component id are taken to create a more specific
 * Mavlink vehicle instance by a subclass.
 */
class Mavlink_vehicle_manager: public vsm::Request_processor {
    DEFINE_COMMON_CLASS(Mavlink_vehicle_manager, vsm::Request_processor)
public:

    /** Constructor. */
    Mavlink_vehicle_manager(
            const std::string default_model_name,
            const std::string config_prefix,
            const vsm::mavlink::Extension& extension,
            size_t forced_max_read = 0);

protected:

    /** Method for Mavlink vehicle creation, to be overridden by subclass. */
    virtual Mavlink_vehicle::Ptr
    Create_mavlink_vehicle(
            vsm::Mavlink_demuxer::System_id system_id,
            vsm::Mavlink_demuxer::Component_id component_id,
            vsm::mavlink::MAV_TYPE type,
            vsm::Io_stream::Ref stream,
            std::string serial_number,
            std::string model_name) = 0;

    /** Subclass should override this method to register On_new_connection
     * methods to the transport detector.
     */
    virtual void
    Register_detectors() = 0;

    /** Handler for a new transport connection. */
    void
    On_new_connection(std::string portname, int baud, vsm::Io_stream::Ref);

    /** Default model name to use in UCS system id calculation. */
    std::string default_model_name;

    /** Configuration file prefix. */
    std::string config_prefix;

    /** Mavlink extension used for Mavlink stream creation. */
    const vsm::mavlink::Extension& extension;

    /** Maximum forced Mavlink stream read size. */
    size_t forced_max_read;

private:

    bool
    On_timer();

    /** Read config and create vehicles. */
    void
    Load_vehicle_config();

    /** Prototcol detection polling interval. */
    const std::chrono::milliseconds TIMER_INTERVAL = std::chrono::milliseconds(100);

    /** Detector should receive at least this much bytes to fail detection. */
    const unsigned int MAX_UNDETECTED_BYTES = 300;

    /** Detector should wait this much milliseconds to fail detection.
     * The interval is so big because APM takes ~4 seconds to boot when connected directly to USB*/
    const std::chrono::milliseconds DETECTOR_TIMEOUT = std::chrono::milliseconds(5000);

    /** Handler for the event of protocol transition to OPERATIONAL state. */
    typedef vsm::Callback_proxy<void> Ready_handler;

    /** Context of the managed vehicle. */
    struct Vehicle_ctx {
        Mavlink_vehicle::Ptr vehicle;
        vsm::Io_stream::Ref stream;
    };

    /** Managed vehicles. */
    std::unordered_map<int, Vehicle_ctx> vehicles;

    /** Preconfigured serial numbers and model names:
     * system id maps to [model name, serial number]. */
    std::unordered_map<int, std::pair<std::string, std::string> > preconfigured;

    vsm::Request_worker::Ptr worker;

    /** Trivial detector state. */
    class Detector_ctx {
    public:

        Detector_ctx(Detector_ctx&&) = default;

        Detector_ctx(int timeout) :
            timeout(timeout) {}

        ~Detector_ctx()
        {
            read_op.Abort();
        }

        /** Timeout counter. */
        int timeout;

        /** Current stream read operation. */
        vsm::Operation_waiter read_op;
    };

    /** Map of Mavlink streams in detecting state (not bound to vehicles) and
     * detection timeout counter.
     * Stream lives in this state for 1 second until Mavlink protocol detected
     * or detection failed.
     */
    std::unordered_map<vsm::Mavlink_stream::Ptr, Detector_ctx> detectors;

    /** Watchdog timer for detection. */
    vsm::Timer_processor::Timer::Ptr watchdog_timer;

    /** Create new or update existing vehicles based on received system id
     * and type of the vehicle. */
    void
    On_heartbeat(
            vsm::mavlink::Message<vsm::mavlink::MESSAGE_ID::HEARTBEAT>::Ptr message,
            vsm::Mavlink_stream::Ptr mav_stream);

    void
    Schedule_next_read(vsm::Mavlink_stream::Ptr);

    /** Stream read completion handler. */
    void
    On_stream_read(vsm::Io_buffer::Ptr, vsm::Io_result, vsm::Mavlink_stream::Ptr);

    /** Enable the manager. */
    virtual void
    On_enable() override;

    /** Disable the manager. */
    virtual void
    On_disable() override;

    /** Process disable event from the processor context. */
    void
    Process_on_disable(vsm::Request::Ptr);
};

#endif /* _MAVLINK_VEHICLE_MANAGER_H_ */
