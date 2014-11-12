// Copyright (c) 2014, Smart Projects Holdings Ltd
// All rights reserved.
// See LICENSE file for license details.

#include <adsb_device.h>

using namespace ugcs::vsm;

Adsb_device::Adsb_device(const std::string& name) :
        name(name)
{
    processor = Adsb_device_processor::Get_instance();
}

void
Adsb_device::Enable()
{
    auto req = ugcs::vsm::Request::Create();
    req->Set_processing_handler(
            Make_callback(
                    &Adsb_device::Process_on_enable,
                    Shared_from_this(),
                    req));

    Get_processor()->Submit_request(req);
    ugcs::vsm::Operation_waiter waiter(req);
    waiter.Wait(false);
}

void
Adsb_device::Disable()
{
    auto req = ugcs::vsm::Request::Create();
    req->Set_processing_handler(
            Make_callback(
                    &Adsb_device::Process_on_disable,
                    Shared_from_this(),
                    req));

    Get_processor()->Submit_request(req);
    req->Wait_done(false);
}

ugcs::vsm::Operation_waiter
Adsb_device::Read_frame(
        Read_frame_handler handler,
        ugcs::vsm::Request_completion_context::Ptr ctx)
{
    if (!handler) {
        VSM_EXCEPTION(Invalid_param_exception, "ADS-B frame handler is empty.");
    }

    auto proc = Get_processor();

    auto req = ugcs::vsm::Read_request::Create(
            handler.template Get_arg<0>(), 0, 0, nullptr, 0,
            handler.template Get_arg<1>());

    req->Set_processing_handler(
            Make_callback(
                    &Adsb_device::On_read_frame,
                    Shared_from_this(),
                    req));

    req->Set_completion_handler(
            ctx,
            Make_callback(
                    &Adsb_device::On_read_completed,
                    Shared_from_this(),
                    handler));

    proc->Submit_request(req);
    return req;
}

const std::string&
Adsb_device::Get_name() const
{
    return name;
}

size_t
Adsb_device::Get_lost_frames()
{
    return lost_frames.exchange(0);
}

void
Adsb_device::Push_frame(ugcs::vsm::Io_buffer::Ptr frame)
{
    if (closed) {
        Close();
    } else if (frames.size() < MAX_PENDING) {
        frames.push(frame);
        Push_read_queue();
    } else {
        lost_frames.fetch_add(1);
        LOG_WARN("ADS-B frame is lost, current loss is %zu frames.",
                lost_frames.load());
    }
}

void
Adsb_device::Push_read_queue()
{
    while(!read_requests.empty() && !frames.empty()) {
        auto req = read_requests.front();
        read_requests.pop();

        auto lock = req->Lock();
        if (!req->Is_done()) {
            req->Set_buffer_arg(frames.front(), lock);
            req->Set_result_arg(ugcs::vsm::Io_result::OK, lock);
            frames.pop();
        }
        lock.unlock();

        /* Complete in done state doesn't harm. */
        req->Complete();
    }
}

void
Adsb_device::Close()
{
    closed = true;
    /* Complete any pending requests. */
    while (!read_requests.empty() && !frames.empty()) {
        Push_read_queue();
    }
    /* Discard remaining requests. */
    while (!read_requests.empty()) {
        auto req = read_requests.front();
        read_requests.pop();
        req->Set_result_arg(ugcs::vsm::Io_result::CLOSED);
        req->Complete();
    }

    frames = std::move(decltype(frames)());
}

ugcs::vsm::Request_completion_context::Ptr
Adsb_device::Get_completion_context()
{
    return Get_processor()->Get_completion_context();
}

Adsb_device_processor::Ptr
Adsb_device::Get_processor()
{
    auto proc = processor.lock();
    if (!proc) {
        VSM_EXCEPTION(Invalid_op_exception, "ADS-B device processor already destroyed.");
    }
    return proc;
}

void
Adsb_device::On_read_frame(ugcs::vsm::Read_request::Ptr request)
{
    if (closed) {
        Close();
    } else {
        read_requests.push(request);
        Push_read_queue();
    }
}

void
Adsb_device::On_read_completed(Read_frame_handler handler)
{
    handler();
}

void
Adsb_device::Process_on_enable(ugcs::vsm::Request::Ptr request)
{
    On_enable();
    request->Complete();
}

void
Adsb_device::Process_on_disable(ugcs::vsm::Request::Ptr request)
{
    while(!read_requests.empty()) {
        read_requests.front()->Abort();
        read_requests.pop();
    }
    On_disable();
    request->Complete();
}
