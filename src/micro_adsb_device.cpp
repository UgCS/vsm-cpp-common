// Copyright (c) 2014, Smart Projects Holdings Ltd
// All rights reserved.
// See LICENSE file for license details.

#include <micro_adsb_device.h>

constexpr std::chrono::milliseconds Micro_adsb_device::CMD_RESPONSE_TIMEOUT;

bool
line_handler(const std::string* str)
{
    for (auto c: *str) {
        if (c == '#') {
            LOG_INFO("CAPTURED: %s", str->c_str());
        }
    }
    return false;
}

Micro_adsb_device::Micro_adsb_device(
        ugcs::vsm::Io_stream::Ref stream, bool close_stream) :
        Adsb_device("Micro ADS-B on [" + stream->Get_name() + "]"),
        stream(stream), close_stream(close_stream),
        filter(ugcs::vsm::Text_stream_filter::Create(stream, Get_completion_context()))
{
}

ugcs::vsm::Operation_waiter
Micro_adsb_device::Read_version(
        Read_version_handler handler,
        ugcs::vsm::Request_completion_context::Ptr ctx)
{
    if (read_version_handler) {
        VSM_EXCEPTION(ugcs::vsm::Invalid_op_exception, "Read version already in progress");
    }
    read_version_handler = handler;
    read_version_attempts = 0;
    ASSERT(!read_version_request);

    read_version_request = ugcs::vsm::Request::Create();
    read_version_request->Set_processing_handler(
            ugcs::vsm::Make_callback(
                    &Micro_adsb_device::On_read_version,
                    Shared_from_this()));

    read_version_request->Set_completion_handler(ctx, handler);
    Get_processor()->Submit_request(read_version_request);

    return read_version_request;
}

ugcs::vsm::Operation_waiter
Micro_adsb_device::Init_frames_receiving(
        Init_frames_receiving_handler handler,
        ugcs::vsm::Request_completion_context::Ptr ctx)
{
    if (init_frames_receiving_handler) {
        VSM_EXCEPTION(ugcs::vsm::Invalid_op_exception, "Init frames receiving already in progress");
    }
    init_frames_receiving_handler = handler;
    init_frames_receiving_attempts = 0;
    ASSERT(!init_frames_receiving_request);

    init_frames_receiving_request = ugcs::vsm::Request::Create();
    init_frames_receiving_request->Set_processing_handler(
            ugcs::vsm::Make_callback(
                    &Micro_adsb_device::On_init_frames_receiving,
                    Shared_from_this()));

    init_frames_receiving_request->Set_completion_handler(ctx, handler);
    Get_processor()->Submit_request(init_frames_receiving_request);

    return init_frames_receiving_request;
}

void
Micro_adsb_device::On_enable()
{
    //filter->Set_line_handler(ugcs::vsm::Text_stream_filter::Make_line_handler(&line_handler));
    /* Example output: @00011AD59C088D3C675558BF01CB5A44478EC142;#0000001A;
     *                 @000123993F78200017B070DE22;#000000D8;
     *
     * Frames are always being read and pushed to the device.
     */
    filter->Add_entry(
            regex::regex("@(([:xdigit:]){12})(([:xdigit:][:xdigit:])+);#(([:xdigit:]){8,});"),
            ugcs::vsm::Text_stream_filter::Make_match_handler(
                    &Micro_adsb_device::Read_frame_handler_cb,
                    Shared_from_this()));

    filter->Enable();

}

void
Micro_adsb_device::On_disable()
{
    while (!write_ops.empty()) {
        write_ops.front().Abort();
        write_ops.pop();
    }
    if (read_version_request) {
        read_version_request->Abort();
        read_version_request = nullptr;
    }
    read_version_handler = Read_version_handler();

    if (init_frames_receiving_request) {
        init_frames_receiving_request->Abort();
        init_frames_receiving_request = nullptr;
    }
    init_frames_receiving_handler = Init_frames_receiving_handler();

    filter->Disable(close_stream);
    filter = nullptr;
    stream = nullptr;
}

void
Micro_adsb_device::Write_cmd(const std::string& cmd)
{
    write_ops.emplace(stream->Write(
            ugcs::vsm::Io_buffer::Create(cmd + "\r"),
            Make_write_callback(
                    &Micro_adsb_device::On_write_completed,
                    Shared_from_this()),
            Get_completion_context()));
}

void
Micro_adsb_device::On_write_completed(ugcs::vsm::Io_result)
{
    ASSERT(!write_ops.empty());
    /* Not done, because it is expected to be currently processed. */
    ASSERT(!write_ops.front().Is_done());
    write_ops.front().Abort();
    write_ops.pop();
}

bool
Micro_adsb_device::Read_version_handler_cb(
        regex::smatch * match,
        ugcs::vsm::Text_stream_filter::Lines_list*,
        ugcs::vsm::Io_result result)
{
    if (result == ugcs::vsm::Io_result::OK) {
        const char* match_str = (*match)[1].str().c_str();
        uint8_t version = static_cast<uint8_t>(std::strtoul(match_str, nullptr, 16));
        Complete_read_version_request(version, ugcs::vsm::Io_result::OK);
    } else if (result == ugcs::vsm::Io_result::TIMED_OUT) {
        if (read_version_attempts >= CMD_ATTEMPTS) {
            Complete_read_version_request(0, ugcs::vsm::Io_result::TIMED_OUT);
        } else {
            Read_version_try();
        }
    } else {
        Complete_read_version_request(0, result);
    }
    return false;
}

void
Micro_adsb_device::On_read_version()
{
    Read_version_try();
}

void
Micro_adsb_device::Read_version_try()
{
    Write_cmd("#00");
    /* Example output: #00-00-0E-04-00-00-00-00-00-00-00-00-00-00-00-00 */
    filter->Add_entry(
            regex::regex("#00-[:xdigit:][:xdigit:]-([:xdigit:][:xdigit:])-04([:xdigit:]|-)*"),
            ugcs::vsm::Text_stream_filter::Make_match_handler(
                    &Micro_adsb_device::Read_version_handler_cb,
                    Shared_from_this()),
                    CMD_RESPONSE_TIMEOUT);
    read_version_attempts++;
}

void
Micro_adsb_device::Complete_read_version_request(
        uint8_t version, ugcs::vsm::Io_result result)
{
    ASSERT(read_version_handler);
    ASSERT(read_version_request);
    read_version_handler.Set_args(version, result);
    read_version_handler = Read_version_handler();
    auto tmp = std::move(read_version_request);
    tmp->Complete();
}

bool
Micro_adsb_device::Init_frames_receiving_handler_cb(
        regex::smatch*,
        ugcs::vsm::Text_stream_filter::Lines_list*,
        ugcs::vsm::Io_result result)
{
    if (result == ugcs::vsm::Io_result::TIMED_OUT) {
        if (init_frames_receiving_attempts < CMD_ATTEMPTS) {
            Init_frames_receiving_try();
            return false;
        }
    }
    Complete_init_frames_receiving_request(result);
    return false;
}

void
Micro_adsb_device::On_init_frames_receiving()
{
    Init_frames_receiving_try();
}

void
Micro_adsb_device::Complete_init_frames_receiving_request(ugcs::vsm::Io_result result)
{
    ASSERT(init_frames_receiving_handler);
    ASSERT(init_frames_receiving_request);

    init_frames_receiving_handler.Set_args(result);
    init_frames_receiving_handler = Init_frames_receiving_handler();
    auto tmp = std::move(init_frames_receiving_request);
    tmp->Complete();
}

void
Micro_adsb_device::Init_frames_receiving_try()
{
    /* We want time stamp and frame counter. */
    Write_cmd("#43-33");
    //Write_cmd("#43-34");
    /* Example output: #43-EF-00-00-00-00-00-00-00-00-00-00-00-00-00-00- */
    filter->Add_entry(
            regex::regex("#43-[:xdigit:][:xdigit:]-00-([:xdigit:]|-)*"),
            ugcs::vsm::Text_stream_filter::Make_match_handler(
                    &Micro_adsb_device::Init_frames_receiving_handler_cb,
                    Shared_from_this()),
                    CMD_RESPONSE_TIMEOUT);
    init_frames_receiving_attempts++;
}

bool
Micro_adsb_device::Read_frame_handler_cb(
        regex::smatch* match,
        ugcs::vsm::Text_stream_filter::Lines_list*,
        ugcs::vsm::Io_result result)
{
    if (result != ugcs::vsm::Io_result::OK) {
        Close();
        return false;
    } else {
        std::string frame_str = (*match)[3].str();
        /* Reserve frame buffer, each byte takes 2 hex characters.
         * Regex captures only by pairs, so we are sure about even
         * number of input characters.
         */
        std::vector<uint8_t> frame(frame_str.size() >> 1);
        for (unsigned i = 0; i < frame_str.size(); i += 2) {
            uint8_t c = static_cast<uint8_t>(
                    std::strtoul(frame_str.substr(i, 2).c_str(), nullptr, 16));
            frame[i >> 1] = c;
        }
        Push_frame(ugcs::vsm::Io_buffer::Create(std::move(frame)));
        return true;
    }
}
