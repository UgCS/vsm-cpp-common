// Copyright (c) 2014, Smart Projects Holdings Ltd
// All rights reserved.
// See LICENSE file for license details.

#include <adsb_manager.h>
#include <vsm/transport_detector.h>
#include <adsb_processor.h>
#include <vsm/cucs_processor.h>

Adsb_manager::Adsb_manager(const std::string& prefix) :
        vsm::Request_processor("Adsb manager processor"),
        config_prefix(prefix)
{
}

void
Adsb_manager::On_enable()
{
    worker = vsm::Request_worker::Create(
            "Adsb manager worker",
            std::initializer_list<vsm::Request_container::Ptr>{Shared_from_this()});
    worker->Enable();

    auto req = vsm::Request::Create();
    req->Set_processing_handler(
            Make_callback(
                    &Adsb_manager::Process_on_enable,
                    Shared_from_this(),
                    req));
    Submit_request(req);
    req->Wait_done(false);
}

void
Adsb_manager::Process_on_enable(vsm::Request::Ptr request)
{
    reports_stream = Adsb_processor::Get_instance()->Open_stream();
    Read_adsb_report();

    vsm::Transport_detector::Get_instance()->Add_detector(
            config_prefix,
            vsm::Transport_detector::Make_connect_handler(
                    &Adsb_manager::On_new_connection, Shared_from_this()),
                    Shared_from_this());

    request->Complete();
}

void
Adsb_manager::On_disable()
{
    auto req = vsm::Request::Create();
    req->Set_processing_handler(
            Make_callback(
                    &Adsb_manager::Process_on_disable,
                    Shared_from_this(),
                    req));
    Submit_request(req);
    req->Wait_done(false);
    Set_disabled();
    worker->Disable();
    worker = nullptr;
}

void
Adsb_manager::Process_on_disable(vsm::Request::Ptr request)
{
    for(auto& iter: streams_under_detection) {
        /* Disable device. */
        iter.second.device->Disable();
    }
    streams_under_detection.clear();
    reports_stream->Close();
    report_read_op.Abort();
    request->Complete();
}

void
Adsb_manager::On_new_connection(std::string, int, vsm::Io_stream::Ref stream)
{
    Micro_adsb_device::Ptr device = Micro_adsb_device::Create(stream, false);
    device->Enable();

    auto iter = streams_under_detection.emplace(stream, Adsb_device::Ptr(device));

    Detector_ctx& ctx = iter.first->second;

    ctx.read_op = device->Read_frame(
            Adsb_device::Make_read_frame_handler(
                    &Adsb_manager::On_frame_received,
                    Shared_from_this(),
                    device, stream), worker);

    ctx.init_op = device->Init_frames_receiving(
            Micro_adsb_device::Make_init_frames_receiving_handler(
                    &Adsb_manager::On_micro_adsb_init_frames_receiving,
                    Shared_from_this(),
                    device, stream), worker);
}

void
Adsb_manager::On_micro_adsb_init_frames_receiving(
        vsm::Io_result result,
        Micro_adsb_device::Ptr device,
        vsm::Io_stream::Ref stream)
{
    auto found = streams_under_detection.find(stream);
    ASSERT(found != streams_under_detection.end());

    streams_under_detection.erase(found);

    if (result != vsm::Io_result::OK) {
        LOG_DEBUG("MicroADS-B device not detected on [%s].",
                stream->Get_name().c_str());
        device->Disable();
        vsm::Transport_detector::Get_instance()->Protocol_not_detected(stream);
    } else {
        LOG_INFO("[%s] detected (frames initialization succeeded).",
                device->Get_name().c_str());
        Adsb_processor::Get_instance()->Add_device(device);
    }
}

void
Adsb_manager::On_frame_received(vsm::Io_buffer::Ptr buffer,
        vsm::Io_result result, Adsb_device::Ptr device, vsm::Io_stream::Ref stream)
{
    auto found = streams_under_detection.find(stream);
    ASSERT(found != streams_under_detection.end());

    if (result != vsm::Io_result::OK) {
        LOG_DEBUG("MicroADS-B device not detected on [%s].",
                stream->Get_name().c_str());
        streams_under_detection.erase(found);
        device->Disable();
        vsm::Transport_detector::Get_instance()->Protocol_not_detected(stream);
        return;
    } else if (buffer->Get_length() == vsm::Adsb_frame::SIZE) {
        vsm::Adsb_frame::Ptr frame = vsm::Adsb_frame::Create(buffer);
        if (frame->Verify_checksum()) {
            LOG_INFO("[%s] detected (valid frame received).",
                    device->Get_name().c_str());
            streams_under_detection.erase(found);
            Adsb_processor::Get_instance()->Add_device(device);
            return;
        }
    }

    /* Read more. */
    Detector_ctx& ctx = found->second;
    ctx.read_op.Abort();
    ctx.read_op = device->Read_frame(
            Adsb_device::Make_read_frame_handler(
                    &Adsb_manager::On_frame_received,
                    Shared_from_this(),
                    device, stream), worker);
}

void
Adsb_manager::On_adsb_report(vsm::Adsb_report report)
{
    vsm::Cucs_processor::Get_instance()->Send_adsb_report(report);
    Read_adsb_report();
}

void
Adsb_manager::Read_adsb_report()
{
    report_read_op.Abort();
    report_read_op = reports_stream->Read(
            Adsb_processor::Reports_stream::Make_report_callback(
                    &Adsb_manager::On_adsb_report,
                    Shared_from_this()), worker);
}
