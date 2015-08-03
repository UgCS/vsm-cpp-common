// Copyright (c) 2014, Smart Projects Holdings Ltd
// All rights reserved.
// See LICENSE file for license details.

/**
 * @file adsb_device.h
 */
#ifndef _ADSB_DEVICE_H_
#define _ADSB_DEVICE_H_

#include <ugcs/vsm/request_context.h>
#include <ugcs/vsm/io_stream.h>
#include <queue>
#include <ugcs/vsm/io_request.h>
#include <ugcs/vsm/peripheral_device.h>
#include <adsb_device_processor.h>
#include <ugcs/vsm/reference_guard.h>



/** Base interface for a device capable of receiving raw ADS-B radio frames.
 * Management and control of a specific ADS-B device is to be done by a
 * sub-class, while base class is used to receive raw ADSB-B frames independent
 * from the actual device.
 */
class Adsb_device: public ugcs::vsm::Request_processor, public ugcs::vsm::Peripheral_device {
    DEFINE_COMMON_CLASS(Adsb_device, ugcs::vsm::Request_processor);
/*std::enable_shared_from_this<Adsb_device> {

    DEFINE_COMMON_CLASS(Adsb_device, Adsb_device);*/
public:
    /** Default prototype for read frame operation completion handler.
     * ugcs::vsm::Io_buffer is valid only if ugcs::vsm::Io_result is OK. */
    typedef ugcs::vsm::Callback_proxy<void, ugcs::vsm::Io_buffer::Ptr, ugcs::vsm::Io_result> Read_frame_handler;


    /** Default prototype for init frames receiving completion handler. */
    typedef ugcs::vsm::Callback_proxy<void, ugcs::vsm::Io_result> Init_frames_receiving_handler;

    /** Maximum number of ADS-B frames which could be queued in the device. This
     * queue is overflowed, if the user does not read the frames quickly
     * enough. */
    constexpr static size_t MAX_PENDING = 32;

    /** Construct a device with given name. */
    Adsb_device(const std::string& name);

    /** Enalbe the device. */
    void
    Enable();

    /** Disable the device, do cleanups. */
    void
    Disable();

    /** Release the stream, keep device running */
    void
    Release_stream();

    /**
     * Read raw ADS-B frame, thats is including parity/checksum fields.
     * @param handler Handler of the read operation completion.
     */
    ugcs::vsm::Operation_waiter
    Read_frame(Read_frame_handler handler, ugcs::vsm::Request_completion_context::Ptr);

    /**
     * Initialization function. Should be implemented in children of Ads-B device.
     * @param handler Handler of the read operation completion.
     */
    virtual ugcs::vsm::Operation_waiter
    Init_frames_receiving(Init_frames_receiving_handler, ugcs::vsm::Request_completion_context::Ptr) =0;

    /** Get the name of the device which helps human to identify the physically
     * connected instance, for example "Micro ADS-B on [COM3]".
     */
    const std::string&
    Get_name() const;

    /** Builder for initialize frames receiving handler. */
    DEFINE_CALLBACK_BUILDER(Make_init_frames_receiving_handler,
            (ugcs::vsm::Io_result), (ugcs::vsm::Io_result::OTHER_FAILURE));

    /** Builder for read frame handler. */
    DEFINE_CALLBACK_BUILDER(Make_read_frame_handler,
            (ugcs::vsm::Io_buffer::Ptr, ugcs::vsm::Io_result),
            (nullptr, ugcs::vsm::Io_result::OTHER_FAILURE))

    /** Return the number of the frames, which are lost due to the device queue
     * overflow. Method invocation clears the counter.
     */
    size_t
    Get_lost_frames();

    /** Return the current status of device heartbeat.
     * True means heartbeat is ok, false means it has timed out.
     * Timings are device-specific.
     */
    virtual bool
    Get_is_heartbeat_ok() =0;

    /** Return true if heartbeat timeout is past shutdown threshold
     */
    virtual bool
    Get_is_heartbeat_shutdown() =0;


protected:

    /** Enable event handler is sub-class. Called from the ADS-B device
     * processor context. */
    virtual void
    On_enable() =0;

    /** Disable event handler is sub-class. Called from the ADS-B device
     * processor context. */
    virtual void
    On_disable() =0;

    /** Push new ADS-B frame to the device. Should be called from the
     * ADS-B device processor context only. */
    void
    Push_frame(ugcs::vsm::Io_buffer::Ptr);

    /** Close the device. All pending and new requests will be completed with
     * appropriate result code.
     */
    void
    Close();

    /** Get common completion context handled by ADS-B device processor. */
    ugcs::vsm::Request_completion_context::Ptr
    Get_completion_context();

    /** Get associated ADS-B device processor.
     * @throw Invalid_op_exception if processor is already destroyed.
     */
    Adsb_device_processor::Ptr
    Get_processor();

    /** Individual device worker */
    ugcs::vsm::Request_worker::Ptr adsb_worker;

    /** Stores last heartbeat time */
    std::chrono::system_clock::time_point last_heartbeat_received;

    /** Updates heartbeat info */
    void
    Update_heartbeat();

    /** True when device is shutting down. */
    bool shutting_down = false;


private:

    /** Try to push read queue. It is pushed only when at least one
     * read frame request and one frame are available. */
    void
    Push_read_queue();

    /** Process read request in a processor context. */
    void
    On_read_frame(ugcs::vsm::Read_request::Ptr);

    /** Read request completion handler. */
    void
    On_read_completed(Read_frame_handler);

    /** Process enable event in processor context. */
    void
    Process_on_enable(ugcs::vsm::Request::Ptr);

    /** Process disable event in processor context. */
    void
    Process_on_disable(ugcs::vsm::Request::Ptr);

    /** Name of the device. */
    const std::string name;

    /** Buffered frames, ready to be read. */
    std::queue<ugcs::vsm::Io_buffer::Ptr> frames;

    /** Pending read requests. */
    std::queue<ugcs::vsm::Read_request::Ptr> read_requests;

    /** Associated processor. */
    Adsb_device_processor::Weak_ptr processor;

    /** True when device is closed and can not be anymore read from. */
    bool closed = false;

    /** Current number of lost frames due to overflow. */
    std::atomic_size_t lost_frames = { 0 };

    /** Mutex for ADS-B device. */
    std::mutex adsb_device_arbiter;
};

#endif /* _ADSB_DEVICE_H_ */
