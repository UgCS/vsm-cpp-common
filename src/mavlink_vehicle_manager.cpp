// Copyright (c) 2014, Smart Projects Holdings Ltd
// All rights reserved.
// See LICENSE file for license details.

#include <mavlink_vehicle_manager.h>

using namespace ugcs::vsm;

Mavlink_vehicle_manager::Mavlink_vehicle_manager(
        const std::string default_model_name,
        const std::string config_prefix,
        const mavlink::Extension& extension) :
    Request_processor("Mavlink vehicle manager processor"),
    default_model_name(default_model_name),
    config_prefix(config_prefix),
    extension(extension)
{
}

void
Mavlink_vehicle_manager::On_enable()
{
    Request_processor::On_enable();

    worker = ugcs::vsm::Request_worker::Create(
        "Mavlink vehicle manager worker",
        std::initializer_list<ugcs::vsm::Request_container::Ptr>{Shared_from_this()});

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
	On_manager_disable();

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
    auto props = ugcs::vsm::Properties::Get_instance().get();

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
        catch (ugcs::vsm::Exception& ex) {
            LOG_INFO("Error while reading custom vehicle: %s", ex.what());
        }
    }

    auto dump_var = config_prefix + ".mission_dump_path";

    if (props->Exists(dump_var)) {
        std::string filename = props->Get(dump_var);
        mission_dump_path = filename;
    }

    if (!preconfigured.empty()) {
    	LOG_INFO("%zu custom vehicle(-s) configured.", preconfigured.size());
    }

    Register_detectors();
}

void
Mavlink_vehicle_manager::Add_timeout_extension_pattern(const regex::regex& re)
{
    extension_patterns.push_back(re);
}

void
Mavlink_vehicle_manager::Write_to_vehicle_timed_out(
        const Operation_waiter::Ptr& waiter,
        Mavlink_vehicle::Mavlink_stream::Weak_ptr mav_stream)
{
    auto locked = mav_stream.lock();
    Io_stream::Ref stream = locked ? locked->Get_stream() : nullptr;
    std::string server_info =
            stream ? stream->Get_name() : "already disconnected";
    LOG_DBG("Write timeout on [%s] detected.", server_info.c_str());
    waiter->Abort();
}

void
Mavlink_vehicle_manager::On_param_value(
        mavlink::Message<mavlink::MESSAGE_ID::PARAM_VALUE>::Ptr message,
        Mavlink_vehicle::Mavlink_stream::Ptr mav_stream)
{
    if (message->payload->param_id.Get_string() == "FRAME") {
        auto det_iter = detectors.find(mav_stream);
        if (det_iter != detectors.end() && message->payload->param_value == 2.0) {
            det_iter->second.frame_type =
                    static_cast<mavlink::MAV_TYPE>(mavlink::ugcs::MAV_TYPE::MAV_TYPE_IRIS);
        }
        auto system_id = message->Get_sender_system_id();
        auto component_id = message->Get_sender_component_id();
        Create_vehicle_wrapper(
                mav_stream,
                system_id,
                component_id
                );
    }
}

void
Mavlink_vehicle_manager::Create_vehicle_wrapper(
        Mavlink_vehicle::Mavlink_stream::Ptr mav_stream,
        mavlink::System_id_common system_id,
        uint8_t component_id
        )
{
    std::string serial_number;
    std::string model_name;
    auto frame_type = mavlink::MAV_TYPE::MAV_TYPE_QUADROTOR;
    bool is_preconfigured = false;
    auto it = vehicles.find(system_id);
    ugcs::vsm::Socket_address::Ptr peer_addr = nullptr;

    if (it == vehicles.end()) {
        auto preconf = preconfigured.find(system_id);
        if (preconf != preconfigured.end()) {
            model_name = preconf->second.first;
            serial_number = preconf->second.second;
            is_preconfigured = true;
        } else {
            serial_number = std::to_string(system_id);
            model_name = default_model_name;
        }
        it = vehicles.emplace(system_id, Vehicle_ctx()).first;
    }

    auto det_iter = detectors.find(mav_stream);
    ugcs::vsm::Optional<std::string> custom_model_name;
    ugcs::vsm::Optional<std::string> custom_serial_number;

    if (det_iter != detectors.end()) {
        custom_model_name = det_iter->second.custom_model;
        custom_serial_number = det_iter->second.custom_serial;
        frame_type = det_iter->second.frame_type;
        peer_addr = det_iter->second.peer_addr;
    }

    auto &ctx = it->second;
    if (!is_preconfigured) {
        model_name = custom_model_name ? *custom_model_name : model_name;
        serial_number = custom_serial_number ? *custom_serial_number : serial_number;
    }
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
            frame_type,
            mav_stream->Get_stream(),
            peer_addr,
            mission_dump_path,
            serial_number,
            model_name,
            is_preconfigured
            );

    vehicle->Enable();
    /* Keep the stream to see when it will be closed to delete the vehicle. */
    ctx.stream = mav_stream->Get_stream();

    /* Disable Mavlink stream used by manager, because vehicle now has full
     * control over it. */
    mav_stream->Disable();

    /* Mavlink detected, so remove this stream from detectors. */
    detectors.erase(mav_stream);
}

void
Mavlink_vehicle_manager::On_heartbeat(
        mavlink::Message<mavlink::MESSAGE_ID::HEARTBEAT>::Ptr message,
        Mavlink_vehicle::Mavlink_stream::Ptr mav_stream)
{
    std::string serial_number;
    std::string model_name;
    auto system_id = message->Get_sender_system_id();
    auto component_id = message->Get_sender_component_id();

    auto det_iter = detectors.find(mav_stream);
    if (det_iter != detectors.end()) {
        det_iter->second.frame_type = static_cast<mavlink::MAV_TYPE>(message->payload->type.Get());
        if (det_iter->second.frame_detection_retries) {
            // Try frame detection only once.
            det_iter->second.frame_detection_retries--;

            mavlink::Pld_param_request_read payload;
            payload->target_component = component_id;
            payload->target_system = system_id;
            payload->param_id = "FRAME";
            payload->param_index = -1;

            mav_stream->Send_message(
                    payload,
                    Mavlink_vehicle::VSM_SYSTEM_ID,
                    Mavlink_vehicle::VSM_COMPONENT_ID,
                    Mavlink_vehicle::WRITE_TIMEOUT,
                    Make_timeout_callback(
                            &Mavlink_vehicle_manager::Write_to_vehicle_timed_out, this, mav_stream),
                            Get_worker());
        } else {
            Create_vehicle_wrapper(
                    mav_stream,
                    system_id,
                    component_id
                    );
        }
    }
}

void
Mavlink_vehicle_manager::On_raw_data(
        ugcs::vsm::Io_buffer::Ptr buffer,
        Mavlink_vehicle::Mavlink_stream::Ptr mav_stream)
{
    auto iter = detectors.find(mav_stream);
    auto& ctx = iter->second;
    auto str = buffer->Get_string();
    for(auto c: str) {
        bool handle = false;
        if (!c || c == '\r' || c == '\n') {
            handle = !ctx.curr_line.empty();
        } else {
            ctx.curr_line += c;
            handle = ctx.curr_line.length() >= MAX_RAW_LINE;
        }
        if (handle) {
            Handle_raw_line(ctx, mav_stream);
            ctx.curr_line.clear();
        }
    }
}

void
Mavlink_vehicle_manager::Handle_raw_line(
        Detector_ctx& ctx,
        const Mavlink_vehicle::Mavlink_stream::Ptr& mav_stream)
{
    for(auto& re: extension_patterns) {
        regex::smatch smatch;
        if (regex::regex_search(ctx.curr_line, smatch, re)) {
            LOG_DEBUG("Detection timeout extended due to pattern match: %s",
                    ctx.curr_line.c_str());
            ctx.timeout += EXTENDED_TIMEOUT / TIMER_INTERVAL;
            mav_stream->Get_decoder().Register_raw_data_handler(
                    Mavlink_vehicle::Mavlink_stream::Decoder::Raw_data_handler());
            break;
        }
    }
}

void
Mavlink_vehicle_manager::Schedule_next_read(Mavlink_vehicle::Mavlink_stream::Ptr mav_stream)
{
    auto stream = mav_stream->Get_stream();
    if (stream) {
        /* Mavlink stream still belongs to manager, so continue reading. */
        size_t to_read = mav_stream->Get_decoder().Get_next_read_size();
        size_t max_read;
        if (stream->Get_type() == Io_stream::Type::UDP) {
            max_read = ugcs::vsm::mavlink::MAX_MAVLINK_PACKET_SIZE;
        } else {
            max_read = to_read;
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
        Mavlink_vehicle::Mavlink_stream::Ptr mav_stream)
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

Request_worker::Ptr
Mavlink_vehicle_manager::Get_worker()
{
	return worker;
}

void
Mavlink_vehicle_manager::On_manager_disable()
{

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
        		&&  stats.bad_length == 0
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
Mavlink_vehicle_manager::Handle_new_connection(
        std::string name,
        int baud,
        ugcs::vsm::Socket_address::Ptr peer_addr,
        ugcs::vsm::Io_stream::Ref stream,
        bool detect_frame,
        ugcs::vsm::Optional<std::string> custom_model_name,
        ugcs::vsm::Optional<std::string> custom_serial_number)
{
    auto mav_stream = Mavlink_vehicle::Mavlink_stream::Create(stream, extension);
    mav_stream->Bind_decoder_demuxer();

    LOG_INFO("New connection [%s:%d].", name.c_str(), baud);
    mav_stream->Get_demuxer().
            Register_handler<mavlink::MESSAGE_ID::HEARTBEAT, mavlink::Extension>(
            Mavlink_demuxer::Make_handler<mavlink::MESSAGE_ID::HEARTBEAT, mavlink::Extension>(
                    &Mavlink_vehicle_manager::On_heartbeat,
                    Shared_from_this(),
                    mav_stream));

    mav_stream->Get_demuxer().
            Register_handler<mavlink::MESSAGE_ID::PARAM_VALUE, mavlink::Extension>(
            Mavlink_demuxer::Make_handler<mavlink::MESSAGE_ID::PARAM_VALUE, mavlink::Extension>(
                    &Mavlink_vehicle_manager::On_param_value,
                    Shared_from_this(),
                    mav_stream));

    detectors.emplace(
            std::piecewise_construct,
            std::forward_as_tuple(mav_stream),
            std::forward_as_tuple(
                    DETECTOR_TIMEOUT / TIMER_INTERVAL,
                    detect_frame,
                    peer_addr,
                    custom_model_name,
                    custom_serial_number
                    ));

    mav_stream->Get_decoder().Register_raw_data_handler(
            Mavlink_vehicle::Mavlink_stream::Decoder::Make_raw_data_handler(
                    &Mavlink_vehicle_manager::On_raw_data,
                    Shared_from_this(),
                    mav_stream));

    Schedule_next_read(mav_stream);
}
