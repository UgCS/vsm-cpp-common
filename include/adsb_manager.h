// Copyright (c) 2014, Smart Projects Holdings Ltd
// All rights reserved.
// See LICENSE file for license details.

/**
 * @file adsb_manager.h
 */
#ifndef _ADSB_MANAGER_H_
#define _ADSB_MANAGER_H_

#include <ugcs/vsm/request_worker.h>
#include <ugcs/vsm/io_stream.h>
#include <micro_adsb_device.h>
#include <unordered_set>
#include <adsb_processor.h>

/** Manager for ADS-B devices. Supposed to know about all available ADS-B
 * device drivers. Currently:
 *   - MicroADS-B
 */
class Adsb_manager: public ugcs::vsm::Request_processor {
    DEFINE_COMMON_CLASS(Adsb_manager, ugcs::vsm::Request_processor)

public:

    /** Construct manager with custom prefix. */
    Adsb_manager(const std::string&);

private:

    /** Enable manager. */
    virtual void
    On_enable() override;

    /** Enable the manager in the processor context. */
    void
    Process_on_enable(ugcs::vsm::Request::Ptr);

    /** Disable manager. */
    virtual void
    On_disable() override;

    /** Disable the manager in the processor context. */
    void
    Process_on_disable(ugcs::vsm::Request::Ptr);

    /** New connection available for detection. */
    void
    On_new_connection(
    		std::string,
    		int,
    		ugcs::vsm::Socket_address::Ptr,
    		ugcs::vsm::Io_stream::Ref);

    /** Frames receiving initialized on Micro ADS-B device. */
    void
    On_micro_adsb_init_frames_receiving(ugcs::vsm::Io_result,
            Micro_adsb_device::Ptr, ugcs::vsm::Io_stream::Ref);

    /** ADS-B frame received. */
    void
    On_frame_received(ugcs::vsm::Io_buffer::Ptr, ugcs::vsm::Io_result, Adsb_device::Ptr, ugcs::vsm::Io_stream::Ref);

    /** ADS-B report received. */
    void
    On_adsb_report(ugcs::vsm::Adsb_report);

    /** Schedule next reading of ADS-B report. */
    void
    Read_adsb_report();

    /** Configuration prefix. */
    const std::string config_prefix;

    /** Trivial detector context. */
    class Detector_ctx {
    public:

        Detector_ctx(Adsb_device::Ptr device) :
            device(device) {}

        Detector_ctx(Detector_ctx&&) = default;

        ~Detector_ctx()
        {
            read_op.Abort();
            init_op.Abort();
            /* Don't disable the device, because it could be given to
             * ADS-B processor on successful detection.
             */
        }

        /** Device. */
        Adsb_device::Ptr device;

        /** Current read frame operation. */
        ugcs::vsm::Operation_waiter read_op;

        /** Current init frames receiving operation. */
        ugcs::vsm::Operation_waiter init_op;

    };

    /** Streams being detected. */
    std::unordered_map<ugcs::vsm::Io_stream::Ref, Detector_ctx, ugcs::vsm::Io_stream::Ref::Hasher>
        streams_under_detection;

    /** The stream of ADS-B reports read by this manager. */
    Adsb_processor::Reports_stream::Ptr reports_stream;

    /** Current ADS-B report reading operation. */
    ugcs::vsm::Operation_waiter report_read_op;

    /** Dedicated worker. */
    ugcs::vsm::Request_worker::Ptr worker;

};

#endif /* _ADSB_MANAGER_H_ */
