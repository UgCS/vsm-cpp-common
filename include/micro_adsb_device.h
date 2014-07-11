// Copyright (c) 2014, Smart Projects Holdings Ltd
// All rights reserved.
// See LICENSE file for license details.

/**
 * @file micro_adsb_device.h
 */
#ifndef _MICRO_ADSB_DEVICE_H_
#define _MICRO_ADSB_DEVICE_H_

#include <adsb_device.h>
#include <ugcs/vsm/io_stream.h>
#include <ugcs/vsm/text_stream_filter.h>

/** Micro-ADSB device implementation. */
class Micro_adsb_device: public Adsb_device {
    DEFINE_COMMON_CLASS(Micro_adsb_device, Adsb_device)
public:
    /** Construct new class to handle a Micro ADS-B device connected though
     * a stream.
     * @param stream Stream to read from.
     * @param close_stream Should the stream be closed when device is disabled.
     */
    Micro_adsb_device(
            ugcs::vsm::Io_stream::Ref stream, bool close_stream = true);

    /** Default prototype for read version operation completion handler. */
    typedef ugcs::vsm::Callback_proxy<void, uint8_t, ugcs::vsm::Io_result> Read_version_handler;

    /** Default prototype for init frames receiving completion handler. */
    typedef ugcs::vsm::Callback_proxy<void, ugcs::vsm::Io_result> Init_frames_receiving_handler;

    /** Builder for read version handler. */
    DEFINE_CALLBACK_BUILDER(Make_read_version_handler, (uint8_t, ugcs::vsm::Io_result),
            (0, ugcs::vsm::Io_result::OTHER_FAILURE));

    /** Builder for initialize frames receiving handler. */
    DEFINE_CALLBACK_BUILDER(Make_init_frames_receiving_handler,
            (ugcs::vsm::Io_result), (ugcs::vsm::Io_result::OTHER_FAILURE));

    /** Read firmware version of the device. */
    ugcs::vsm::Operation_waiter
    Read_version(Read_version_handler, ugcs::vsm::Request_completion_context::Ptr);

    /** Initialize frames receiving. */
    ugcs::vsm::Operation_waiter
    Init_frames_receiving(Init_frames_receiving_handler, ugcs::vsm::Request_completion_context::Ptr);

private:

    /** Useful constants. */
    enum {
        /** Number of command execution attempts before bailing out.
         * Micro ADS-B device sometimes ignores commands when frame
         * receiving is enabled, so several attempts should be made.
         * This is hidden from the user.
         */
        CMD_ATTEMPTS = 10,
    };

    /** Timeout for command response from the device. */
    constexpr static std::chrono::milliseconds CMD_RESPONSE_TIMEOUT =
            std::chrono::milliseconds(500);

    /** Connection with the Micro ADS-B USB dongle. Supposed to be serial stream
     * but actually it does not matter here.
     */
    ugcs::vsm::Io_stream::Ref stream;

    /** Should the stream be closed on disable. */
    bool close_stream;

    /** Text stream filter for data parsing. */
    ugcs::vsm::Text_stream_filter::Ptr filter;

    /** Current read version handler, if any. Only one at a time. */
    Read_version_handler read_version_handler;

    /** Current read version request, if any. */
    ugcs::vsm::Request::Ptr read_version_request;

    /** Number of processed read version attempts. */
    size_t read_version_attempts;

    /** Current init frames receiving user handler, if any. Only one at a time. */
    Init_frames_receiving_handler init_frames_receiving_handler;

    /** Current init frames receving request. */
    ugcs::vsm::Request::Ptr init_frames_receiving_request;

    /** Number of processed init frames receiving attempts. */
    size_t init_frames_receiving_attempts;

    /** Scheduled write operations. */
    std::queue<ugcs::vsm::Operation_waiter> write_ops;

    /** Enable the device. */
    virtual void
    On_enable() override;

    /** Disable the device. */
    virtual void
    On_disable() override;

    /** Write a command to the device. CR symbol is added automatically. */
    void
    Write_cmd(const std::string& cmd);

    /** Write completion handler. */
    void
    On_write_completed(ugcs::vsm::Io_result);

    /** Correctly formatted version response handler. */
    bool
    Read_version_handler_cb(regex::smatch*, ugcs::vsm::Text_stream_filter::Lines_list*, ugcs::vsm::Io_result);

    /** Read version handler in the processor context. */
    void
    On_read_version();

    /** Try to read the version. */
    void
    Read_version_try();

    /** Complete read version handler request. */
    void
    Complete_read_version_request(uint8_t, ugcs::vsm::Io_result);

    /** Correctly formatted init frames receiving response handler. */
    bool
    Init_frames_receiving_handler_cb(regex::smatch*,
            ugcs::vsm::Text_stream_filter::Lines_list*, ugcs::vsm::Io_result);

    /** Init frames receiving handler in the processor context. */
    void
    On_init_frames_receiving();

    /** Complete init frames receiving request. */
    void
    Complete_init_frames_receiving_request(ugcs::vsm::Io_result);

    /** Try to init frames receiving. */
    void
    Init_frames_receiving_try();

    /** Correctly formatted frame handler. */
    bool
    Read_frame_handler_cb(regex::smatch*, ugcs::vsm::Text_stream_filter::Lines_list*, ugcs::vsm::Io_result);
};

#endif /* _MICRO_ADSB_DEVICE_H_ */
