// Copyright (c) 2014, Smart Projects Holdings Ltd
// All rights reserved.
// See LICENSE file for license details.

/**
 * @file adsb_manager.h
 */
#ifndef _ADSB_MANAGER_H_
#define _ADSB_MANAGER_H_

#include <vsm/request_worker.h>
#include <vsm/io_stream.h>
#include <micro_adsb_device.h>
#include <unordered_set>
#include <adsb_processor.h>

/** Manager for ADS-B devices. Supposed to know about all available ADS-B
 * device drivers. Currently:
 *   - MicroADS-B
 */
class Adsb_manager: public vsm::Request_processor {
    DEFINE_COMMON_CLASS(Adsb_manager, vsm::Request_processor)

public:

    /** Construct manager with custom prefix. */
    Adsb_manager(const std::string&);

private:

    /** Enable manager. */
    virtual void
    On_enable() override;

    /** Enable the manager in the processor context. */
    void
    Process_on_enable(vsm::Request::Ptr);

    /** Disable manager. */
    virtual void
    On_disable() override;

    /** Disable the manager in the processor context. */
    void
    Process_on_disable(vsm::Request::Ptr);

    /** New connection available for detection. */
    void
    On_new_connection(std::string, int, vsm::Io_stream::Ref);

    /** Frames receiving initialized on Micro ADS-B device. */
    void
    On_micro_adsb_init_frames_receiving(vsm::Io_result,
            Micro_adsb_device::Ptr, vsm::Io_stream::Ref);

    /** ADS-B frame received. */
    void
    On_frame_received(vsm::Io_buffer::Ptr, vsm::Io_result, Adsb_device::Ptr, vsm::Io_stream::Ref);

    /** ADS-B report received. */
    void
    On_adsb_report(vsm::Adsb_report);

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
        vsm::Operation_waiter read_op;

        /** Current init frames receiving operation. */
        vsm::Operation_waiter init_op;

    };

    /** Streams being detected. */
    std::unordered_map<vsm::Io_stream::Ref, Detector_ctx, vsm::Io_stream::Ref::Hasher>
        streams_under_detection;

    /** The stream of ADS-B reports read by this manager. */
    Adsb_processor::Reports_stream::Ptr reports_stream;

    /** Current ADS-B report reading operation. */
    vsm::Operation_waiter report_read_op;

    /** Dedicated worker. */
    vsm::Request_worker::Ptr worker;

};

#endif /* _ADSB_MANAGER_H_ */
