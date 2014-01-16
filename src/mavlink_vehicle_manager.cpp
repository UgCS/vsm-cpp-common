// Copyright (c) 2014, Smart Projects Holdings Ltd
// All rights reserved.
// See LICENSE file for license details.

#include <mavlink_vehicle_manager.h>

using namespace vsm;

Mavlink_vehicle_manager::Mavlink_vehicle_manager(
        const std::string default_model_name,
        const std::string config_prefix,
        const mavlink::Extension& extension,
        size_t forced_max_read) :
    Request_processor("Mavlink vehicle manager processor"),
    default_model_name(default_model_name),
    config_prefix(config_prefix),
    extension(extension),
    forced_max_read(forced_max_read)
{
}

void
Mavlink_vehicle_manager::On_enable()
{
    Request_processor::On_enable();

    worker = vsm::Request_worker::Create(
        "Mavlink vehicle manager worker",
        std::initializer_list<vsm::Request_container::Ptr>{Shared_from_this()});

    worker->Enable();

    watchdog_timer = Timer_processor::Get_instance()->Create_timer(
            TIMER_INTERVAL,
            Make_callback(&Mavlink_vehicle_manager::On_timer, this),
            worker);

    Load_vehicle_config();
}

void
Mavlink_vehicle_manager::On_disable()
{
    auto req = Request::Create();
    req->Set_processing_handler(
            Make_callback(
                    &Mavlink_vehicle_manager::Process_on_disable,
                    Shared_from_this(),
                    req));
    worker->Submit_request(req);
    req->Wait_done(false);
    Set_disabled();
    worker->Disable();
    worker = nullptr;
}

void
Mavlink_vehicle_manager::Process_on_disable(Request::Ptr request)
{
    for (auto& v : vehicles) {
        v.second.vehicle->Disable();
    }
    for (auto& s : detectors) {
        s.first->Disable();
    }
    detectors.clear();
    vehicles.clear();

    watchdog_timer->Cancel();
    watchdog_timer = nullptr;

    request->Complete();
}

void
Mavlink_vehicle_manager::Load_vehicle_config()
{
    auto props = vsm::Properties::Get_instance().get();

    /* Load vehicle data */
    for (auto it = props->begin(config_prefix); it != props->end(); it++) {
        auto token_index = 2;
        try {
            if (it[token_index] == "serial_port") {
                continue;   /* this is handled by detector->Add_detector() call */
            }

            if (it[token_index] == "custom") {
                /* Custom vehicle defined, lets look for its system id,
                 * model name and serial number, too. */
                auto vpref = config_prefix + ".custom." + it[token_index + 1];
                auto system_id = props->Get_int(vpref + ".system_id");
                auto model_name = props->Get(vpref + ".model_name");
                auto serial_number = props->Get(vpref + ".serial_number");
                preconfigured[system_id] = std::make_pair(model_name, serial_number);
            }
        }
        catch (vsm::Exception& ex) {
            LOG_INFO("Error while reading custom vehicle: %s", ex.what());
        }
    }

    LOG_INFO("%lu vehicle(-s) configured.", vehicles.size());

    Register_detectors();
}

void
Mavlink_vehicle_manager::On_heartbeat(
        mavlink::Message<mavlink::MESSAGE_ID::HEARTBEAT>::Ptr message,
        Mavlink_stream::Ptr mav_stream)
{
    std::string serial_number;
    std::string model_name;
    auto system_id = message->Get_sender_system_id();
    auto component_id = message->Get_sender_component_id();

    auto it = vehicles.find(system_id);
    if (it == vehicles.end()) {
    	auto preconf = preconfigured.find(system_id);
    	if (preconf != preconfigured.end()) {
    	    model_name = preconf->second.first;
    	    serial_number = preconf->second.second;
    	} else {
    	    serial_number = std::to_string(system_id);
    	    model_name = default_model_name;
    	}
        it = vehicles.emplace(system_id, Vehicle_ctx()).first;
    }

    auto &ctx = it->second;
    /* Note the vehicle reference! */
    auto& vehicle = ctx.vehicle;
    if (vehicle) {
        model_name = vehicle->Get_model_name();
        serial_number = vehicle->Get_serial_number();
        if (!ctx.stream->Is_closed()) {
            LOG_WARNING("Vehicle [%s:%s] Mavlink id %d is reachable via different link.",
                    model_name.c_str(),
                    serial_number.c_str(),
                    system_id);
            ctx.stream->Close();
        }
        vehicle->Disable();
    }

    LOG_INFO("Creating vehicle: [%s:%s] Mavlink id %d",
            model_name.c_str(),
            serial_number.c_str(),
            system_id);

    vehicle = Create_mavlink_vehicle(
            system_id,
            component_id,
            static_cast<mavlink::MAV_TYPE>(message->payload->type.Get()),
            mav_stream->Get_stream(),
            serial_number,
            model_name
            );

    vehicle->Enable();
    /* Keep the stream to see when it will be closed to delete the vehicle. */
    ctx.stream = mav_stream->Get_stream();

    /* Disable Mavlink stream used by manager, because vehicle now has full
     * control over it. */
    mav_stream->Disable();

    /* Mavlink detected, so remote this stream from detectors. */
    detectors.erase(mav_stream);
}

void
Mavlink_vehicle_manager::Schedule_next_read(Mavlink_stream::Ptr mav_stream)
{
    auto stream = mav_stream->Get_stream();
    if (stream) {
        /* Mavlink stream still belongs to manager, so continue reading. */
        size_t to_read = mav_stream->Get_decoder().Get_next_read_size();
        size_t max_read;
        if (!forced_max_read) {
            max_read = to_read;
        } else {
            max_read = forced_max_read;
        }
        auto iter = detectors.find(mav_stream);
        ASSERT(iter != detectors.end());
        Detector_ctx& ctx = iter->second;
        ctx.read_op.Abort();
        ctx.read_op = stream->Read(
                max_read,
                to_read,
                Make_read_callback(
                        &Mavlink_vehicle_manager::On_stream_read,
                        Shared_from_this(),
                        mav_stream),
                        worker);
    }
}

void
Mavlink_vehicle_manager::On_stream_read(
        Io_buffer::Ptr buffer,
        Io_result result,
        Mavlink_stream::Ptr mav_stream)
{
    /* Make sure, Mavlink stream is still in use by manager. */
    if (mav_stream->Get_stream()) {
        if (result == Io_result::OK) {
            mav_stream->Get_decoder().Decode(buffer);
            Schedule_next_read(mav_stream);
        } else {
            /* Stream error during detection. */
            mav_stream->Get_stream()->Close();
            detectors.erase(mav_stream);
            mav_stream->Disable();
        }
    }
}

bool
Mavlink_vehicle_manager::On_timer()
{
    for (auto iter = detectors.begin(); iter != detectors.end(); ) {
        auto mav_stream = iter->first;
        auto &ctx = iter->second;
        auto &stats = mav_stream->Get_decoder().Get_stats();

        /* Decrement timer to catch timeout */
        ctx.timeout--;
        /* Received enough bytes, but still have not matched any valid Mavlink message...
         * OR...
         * Timeout occurred
         */
        if (	(	stats.bytes_received > MAX_UNDETECTED_BYTES
        		&&	stats.bad_checksum == 0
        		&& 	stats.handled == 0
        		&& 	stats.no_handler == 0)
            ||	ctx.timeout == 0) {
            auto stream = mav_stream->Get_stream();
            /* Signal transport_detector that this is not our protocol. */
            LOG_INFO("Mavlink not detected on stream [%s].", stream->Get_name().c_str());
            mav_stream->Disable();
            iter = detectors.erase(iter);
            Transport_detector::Get_instance()->Protocol_not_detected(stream);
            continue;
        }
        iter++;
    }

    for (auto iter = vehicles.begin(); iter != vehicles.end(); ) {
        auto stream = iter->second.stream;
        if (!stream->Is_closed()) {
            /* Vehicle still alive. */
            iter++;
            continue;
        }
        /* Cleanup dead vehicle. */
        iter->second.vehicle->Disable();
        iter = vehicles.erase(iter);
    }
    return true;
}

void
Mavlink_vehicle_manager::On_new_connection(
        std::string name,
        int baud,
        vsm::Io_stream::Ref stream)
{
    auto mav_stream = Mavlink_stream::Create(stream, extension);
    mav_stream->Bind_decoder_demuxer();

    LOG_INFO("New connection: %s:%d", name.c_str(), baud);
    mav_stream->Get_demuxer().
            Register_handler<mavlink::MESSAGE_ID::HEARTBEAT, mavlink::Extension>(
            Make_mavlink_demuxer_handler<mavlink::MESSAGE_ID::HEARTBEAT, mavlink::Extension>(
                    &Mavlink_vehicle_manager::On_heartbeat,
                    Shared_from_this(),
                    mav_stream));

    detectors.emplace(
            mav_stream,
            DETECTOR_TIMEOUT / TIMER_INTERVAL);

    Schedule_next_read(mav_stream);
}
