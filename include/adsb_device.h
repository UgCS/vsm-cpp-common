// Copyright (c) 2014, Smart Projects Holdings Ltd
// All rights reserved.
// See LICENSE file for license details.

/**
 * @file adsb_device.h
 */
#ifndef _ADSB_DEVICE_H_
#define _ADSB_DEVICE_H_

#include <vsm/request_context.h>
#include <vsm/io_stream.h>
#include <queue>
#include <vsm/io_request.h>
#include <adsb_device_processor.h>

/** Base interface for a device capable of receiving raw ADS-B radio frames.
 * Management and control of a specific ADS-B device is to be done by a
 * sub-class, while base class is used to receive raw ADSB-B frames independent
 * from the actual device.
 */
class Adsb_device: public std::enable_shared_from_this<Adsb_device> {
    DEFINE_COMMON_CLASS(Adsb_device, Adsb_device);
public:
    /** Default prototype for read frame operation completion handler.
     * vsm::Io_buffer is valid only if vsm::Io_result is OK. */
    typedef vsm::Callback_proxy<void, vsm::Io_buffer::Ptr, vsm::Io_result> Read_frame_handler;

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

    /**
     * Read raw ADS-B frame, thats is including parity/checksum fields.
     * @param handler Handler of the read operation completion.
     */
    vsm::Operation_waiter
    Read_frame(Read_frame_handler handler, vsm::Request_completion_context::Ptr);

    /** Get the name of the device which helps human to identify the physically
     * connected instance, for example "Micro ADS-B on [COM3]".
     */
    const std::string&
    Get_name() const;

    /** Builder for read frame handler. */
    DEFINE_CALLBACK_BUILDER(Make_read_frame_handler,
            (vsm::Io_buffer::Ptr, vsm::Io_result),
            (nullptr, vsm::Io_result::OTHER_FAILURE))

    /** Return the number of the frames, which are lost due to the device queue
     * overflow. Method invocation clears the counter.
     */
    size_t
    Get_lost_frames();

protected:

    /** Enable event handler is sub-class. Called from the ADS-B device
     * processor context. */
    virtual void
    On_enable() {};

    /** Disable event handler is sub-class. Called from the ADS-B device
     * processor context. */
    virtual void
    On_disable() {};

    /** Push new ADS-B frame to the device. Should be called from the
     * ADS-B device processor context only. */
    void
    Push_frame(vsm::Io_buffer::Ptr);

    /** Close the device. All pending and new requests will be completed with
     * appropriate result code.
     */
    void
    Close();

    /** Get common completion context handled by ADS-B device processor. */
    vsm::Request_completion_context::Ptr
    Get_completion_context();

    /** Get associated ADS-B device processor.
     * @throw Invalid_op_exception if processor is already destroyed.
     */
    Adsb_device_processor::Ptr
    Get_processor();

private:

    /** Try to push read queue. It is pushed only when at least one
     * read frame request and one frame are available. */
    void
    Push_read_queue();

    /** Process read request in a processor context. */
    void
    On_read_frame(vsm::Read_request::Ptr);

    /** Read request completion handler. */
    void
    On_read_completed(Read_frame_handler);

    /** Process enable event in processor context. */
    void
    Process_on_enable(vsm::Request::Ptr);

    /** Process disable event in processor context. */
    void
    Process_on_disable(vsm::Request::Ptr);

    /** Name of the device. */
    const std::string name;

    /** Buffered frames, ready to be read. */
    std::queue<vsm::Io_buffer::Ptr> frames;

    /** Pending read requests. */
    std::queue<vsm::Read_request::Ptr> read_requests;

    /** Associated processor. */
    Adsb_device_processor::Weak_ptr processor;

    /** True when device is closed and can not be anymore read from. */
    bool closed = false;

    /** Current number of lost frames due to overflow. */
    std::atomic_size_t lost_frames;
};

#endif /* _ADSB_DEVICE_H_ */
