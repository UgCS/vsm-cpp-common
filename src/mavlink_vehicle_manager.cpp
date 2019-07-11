// Copyright (c) 2018, Smart Projects Holdings Ltd
// All rights reserved.
// See LICENSE file for license details.

#include <mavlink_vehicle_manager.h>

using namespace ugcs::vsm;

Mavlink_vehicle_manager::Mavlink_vehicle_manager(
        const std::string default_model_name,
        const std::string config_prefix) :
    Request_processor("Mavlink vehicle manager processor"),
    default_model_name(default_model_name),
    config_prefix(config_prefix)
{
}

void
Mavlink_vehicle_manager::On_enable()
{
    Request_processor::On_enable();

    manager_worker = ugcs::vsm::Request_worker::Create(
        "Mavlink vehicle manager worker",
        std::initializer_list<ugcs::vsm::Request_container::Ptr>{Shared_from_this()});

    vehicle_processor = Request_processor::Create("Mavlink vehicle processor");

    vehicle_worker = ugcs::vsm::Request_worker::Create(
        "Mavlink vehicle worker",
        std::initializer_list<ugcs::vsm::Request_container::Ptr>{vehicle_processor});

    manager_worker->Enable();
    vehicle_processor->Enable();
    vehicle_worker->Enable();

    watchdog_timer = Timer_processor::Get_instance()->Create_timer(
            TIMER_INTERVAL,
            Make_callback(&Mavlink_vehicle_manager::On_timer, this),
            manager_worker);

    Transport_detector::Get_instance()->Add_detector(
        Transport_detector::Make_connect_handler(
            &Mavlink_vehicle_manager::Handle_new_injector,
            Shared_from_this()),
        Shared_from_this(),
        "mavlink.injection");

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
    manager_worker->Submit_request(req);
    req->Wait_done(false);

    vehicle_processor->Disable();
    vehicle_worker->Disable();

    Set_disabled();
    manager_worker->Disable();
    manager_worker = nullptr;
}

void
Mavlink_vehicle_manager::Process_on_disable(Request::Ptr request)
{
    On_manager_disable();

    for (auto& i : injection_readers) {
        i.second.Abort();
        if (i.first->Get_stream()) {
            i.first->Get_stream()->Close();
        }
        i.first->Disable();
    }
    injection_readers.clear();

    for (auto& v : vehicles) {
        v.second.vehicle->Disable();
    }
    {
        std::lock_guard<std::mutex> lock(detector_mutex);
        for (auto& s : detectors) {
            s.first->Disable();
        }
        detectors.clear();
    }
    vehicles.clear();

    watchdog_timer->Cancel();
    watchdog_timer = nullptr;

    request->Complete();
}

void
Mavlink_vehicle_manager::Handle_new_injector(
    std::string, int, Socket_address::Ptr, Io_stream::Ref stream)
{
    auto mav_stream = Mavlink_stream::Create(stream);
    mav_stream->Bind_decoder_demuxer();
    mav_stream->Get_demuxer().Register_default_handler(
        Mavlink_demuxer::Make_default_handler(
                    &Mavlink_vehicle_manager::Default_mavlink_handler,
                    Shared_from_this()));

    LOG_INFO("MAVlink injection enabled on %s", stream->Get_name().c_str());
    Schedule_injection_read(mav_stream);
}

bool
Mavlink_vehicle_manager::Default_mavlink_handler(
    Io_buffer::Ptr buf,
    mavlink::MESSAGE_ID_TYPE message_id,
    uint8_t sys,
    uint8_t cmp,
    uint8_t)
{
    int target;
    switch (message_id) {
    case mavlink::MESSAGE_ID::COMMAND_LONG:
        target = mavlink::Message<mavlink::MESSAGE_ID::COMMAND_LONG>::Create(0, 0, 0, buf)->payload->target_system;
        break;
    case mavlink::MESSAGE_ID::COMMAND_INT:
        target = mavlink::Message<mavlink::MESSAGE_ID::COMMAND_INT>::Create(0, 0, 0, buf)->payload->target_system;
        break;
    case mavlink::MESSAGE_ID::GPS_INJECT_DATA:
        target = mavlink::Message<mavlink::MESSAGE_ID::GPS_INJECT_DATA>::Create(0, 0, 0, buf)->payload->target_system;
        break;
    case mavlink::MESSAGE_ID::GPS_RTCM_DATA:
        // Messages which do not have target are broadcasted to all vehicles.
        target = mavlink::SYSTEM_ID_NONE;
        break;
    default:
        return false;
    }

    if (target == mavlink::SYSTEM_ID_NONE) {
        // Messages which do not have target or target is ALL (0) are broadcasted to all vehicles.
        for (auto it : vehicles) {
            it.second.vehicle->Inject_message(message_id, sys, cmp, buf);
        }
    } else {
        // If message has target then send to that specific vehicle.
        auto it = vehicles.find(target);
        if (it != vehicles.end()) {
            it->second.vehicle->Inject_message(message_id, sys, cmp, buf);
        }
    }
    return false;
}

void
Mavlink_vehicle_manager::Schedule_injection_read(Mavlink_stream::Ptr mav_stream)
{
    auto stream = mav_stream->Get_stream();
    if (stream) {
        if (manager_worker->Is_enabled()) {
            size_t to_read = mav_stream->Get_decoder().Get_next_read_size();
            injection_readers[mav_stream].Abort();
            injection_readers.emplace(
                mav_stream,
                stream->Read(
                    0,
                    to_read,
                    Make_read_callback(
                        [this](Io_buffer::Ptr buffer, Io_result result, Mavlink_stream::Ptr mav_stream)
                        {
                            if (result == Io_result::OK) {
                                mav_stream->Get_decoder().Decode(buffer);
                                Schedule_injection_read(mav_stream);
                            } else {
                                if (mav_stream->Get_stream()) {
                                    mav_stream->Get_stream()->Close();
                                }
                                mav_stream->Disable();
                                injection_readers.erase(mav_stream);
                            }
                        },
                        mav_stream),
                    manager_worker));
        } else {
            mav_stream->Disable();
            stream->Close();
        }
    }
}

void
Mavlink_vehicle_manager::Load_vehicle_config()
{
    auto props = ugcs::vsm::Properties::Get_instance().get();

    /* Load vehicle data */
    for (auto it = props->begin(config_prefix); it != props->end(); it++) {
        auto token_index = 2;
        try {
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

    if (props->Exists("vehicle.detection_timeout")) {
        int t = props->Get_int("vehicle.detection_timeout");
        if (t > 0 && t <= 100) {
            detection_timeout = std::chrono::seconds(t);
            LOG_INFO("Vehicle detection timeout set to %d seconds.", t);
        }
    }

    if (props->Exists("mavlink.vsm_system_id")) {
        vsm_system_id = props->Get_int("mavlink.vsm_system_id");
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
    ugcs::vsm::Mavlink_stream::Weak_ptr mav_stream)
{
    auto locked = mav_stream.lock();
    Io_stream::Ref stream = locked ? locked->Get_stream() : nullptr;
    std::string server_info =
            stream ? stream->Get_name() : "already disconnected";
    LOG_DBG("Write timeout on [%s] detected.", server_info.c_str());
    waiter->Abort();
}

void
Mavlink_vehicle_manager::Create_vehicle_wrapper(
    ugcs::vsm::Mavlink_stream::Ptr mav_stream,
    uint8_t system_id,
    uint8_t component_id)
{
    std::string serial_number;
    std::string model_name;
    auto frame_type = mavlink::MAV_TYPE::MAV_TYPE_QUADROTOR;
    bool is_preconfigured = false;
    ugcs::vsm::Socket_address::Ptr peer_addr = nullptr;

    auto preconf = preconfigured.find(system_id);
    if (preconf != preconfigured.end()) {
        model_name = preconf->second.first;
        serial_number = preconf->second.second;
        is_preconfigured = true;
    } else {
        serial_number = std::to_string(system_id);
        model_name = default_model_name;
    }
    auto it = vehicles.emplace(system_id, Vehicle_ctx()).first;

    ugcs::vsm::Optional<std::string> custom_model_name;
    ugcs::vsm::Optional<std::string> custom_serial_number;

    {
        std::lock_guard<std::mutex> lock(detector_mutex);
        auto det_iter = detectors.find(mav_stream);
        if (det_iter != detectors.end()) {
            custom_model_name = det_iter->second.custom_model;
            custom_serial_number = det_iter->second.custom_serial;
            frame_type = det_iter->second.frame_type;
            peer_addr = det_iter->second.peer_addr;
        }
    }

    auto &ctx = it->second;
    if (!is_preconfigured) {
        model_name = custom_model_name ? *custom_model_name : model_name;
        serial_number = custom_serial_number ? *custom_serial_number : serial_number;
    }
    /* Note the vehicle reference! */
    auto& vehicle = ctx.vehicle;

    LOG_INFO("Creating vehicle: [%s:%s] Mavlink ID: %d, Type: %d on %s",
            model_name.c_str(),
            serial_number.c_str(),
            system_id,
            frame_type,
            mav_stream->Get_stream()->Get_name().c_str());

    vehicle = Create_mavlink_vehicle(
            system_id,
            component_id,
            frame_type,
            mav_stream,
            peer_addr,
            mission_dump_path,
            serial_number,
            model_name,
            vehicle_processor,
            vehicle_worker);

    vehicle->Enable();

    // Registration must be done explicitly by each vehicle.

    /* Keep the stream to see when it will be closed to delete the vehicle. */
    ctx.stream = mav_stream->Get_stream();
}

void
Mavlink_vehicle_manager::On_heartbeat(
    mavlink::Message<mavlink::MESSAGE_ID::HEARTBEAT>::Ptr message,
    ugcs::vsm::Mavlink_stream::Ptr mav_stream)
{
    if (!Mavlink_vehicle::Is_vehicle_heartbeat_valid(message)) {
        return;
    }

    // Ignore uninitialized vehicle.
    if (    message->payload->system_status == mavlink::MAV_STATE_UNINIT
        ||  message->payload->system_status == mavlink::MAV_STATE_BOOT
        ||  message->payload->system_status == mavlink::MAV_STATE_POWEROFF) {
        return;
    }

    auto system_id = message->Get_sender_system_id();
    auto component_id = message->Get_sender_component_id();

    std::unique_lock<std::mutex> lock(detector_mutex);
    auto det_iter = detectors.find(mav_stream);
    if (det_iter != detectors.end()) {
        auto stream = mav_stream->Get_stream();
        if (message->payload->autopilot != det_iter->second.autopilot_type) {
            LOG("Failed detection expect %d, received %d.",
                static_cast<int>(det_iter->second.autopilot_type),
                static_cast<int>(message->payload->autopilot));
            // This will break setups when ardupilot and px4 are on the same link.
            mav_stream->Disable();
            detectors.erase(det_iter);
            /* Signal transport_detector that this is not our protocol. */
            Transport_detector::Get_instance()->Protocol_not_detected(stream);
        } else {
            auto it = vehicles.find(system_id);
            if (it == vehicles.end()) {
                det_iter->second.frame_type = static_cast<mavlink::MAV_TYPE>(message->payload->type.Get());
                lock.unlock();
                Create_vehicle_wrapper(
                        mav_stream,
                        system_id,
                        component_id);
            } else {
                if (it->second.stream != stream) {
                    LOG_WARNING("Existing vehicle with mavlink id %d is reachable via different link: %s.",
                        system_id,
                        stream->Get_name().c_str());
                }
                // Restart detection if vehicle is already connected.
                // This allows automatic reconnect on this link when current vehicle times out.
                det_iter->second.timeout = detection_timeout / TIMER_INTERVAL;
            }
        }
    }
}

void
Mavlink_vehicle_manager::On_raw_data(
    ugcs::vsm::Io_buffer::Ptr buffer,
    ugcs::vsm::Mavlink_stream::Ptr mav_stream)
{
    std::lock_guard<std::mutex> lock(detector_mutex);
    auto iter = detectors.find(mav_stream);
    if (iter == detectors.end()) {
        return;
    }
    auto& ctx = iter->second;
    auto str = buffer->Get_string();
    for (auto c : str) {
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
    const ugcs::vsm::Mavlink_stream::Ptr& mav_stream)
{
    if (mav_stream->Get_decoder().Get_common_stats().handled) {
        // We do not need this any more if at least one mavlink message is received.
        mav_stream->Get_decoder().Register_raw_data_handler(
            ugcs::vsm::Mavlink_stream::Decoder::Raw_data_handler());
    }
    for (auto& re : extension_patterns) {
        regex::smatch smatch;
        if (regex::regex_search(ctx.curr_line, smatch, re)) {
            LOG_DEBUG("Detection timeout extended due to pattern match: %s",
                    ctx.curr_line.c_str());
            ctx.timeout += EXTENDED_TIMEOUT / TIMER_INTERVAL;
            mav_stream->Get_decoder().Register_raw_data_handler(
                ugcs::vsm::Mavlink_stream::Decoder::Raw_data_handler());
            break;
        }
    }
}

void
Mavlink_vehicle_manager::Schedule_next_read(ugcs::vsm::Mavlink_stream::Ptr mav_stream)
{
    auto stream = mav_stream->Get_stream();
    if (stream) {
        /* Mavlink stream still belongs to manager, so continue reading. */
        size_t to_read = mav_stream->Get_decoder().Get_next_read_size();
        std::lock_guard<std::mutex> lock(detector_mutex);
        auto iter = detectors.find(mav_stream);
        if (iter == detectors.end()) {
            return;
        }
        Detector_ctx& ctx = iter->second;
        ctx.read_op.Abort();
        ctx.read_op = stream->Read(
                0,
                to_read,
                Make_read_callback(
                        &Mavlink_vehicle_manager::On_stream_read,
                        Shared_from_this(),
                        mav_stream),
                        vehicle_worker);
    }
}

void
Mavlink_vehicle_manager::On_stream_read(
    Io_buffer::Ptr buffer,
    Io_result result,
    ugcs::vsm::Mavlink_stream::Ptr mav_stream)
{
    /* Make sure, Mavlink stream is still in use by manager. */
    if (mav_stream->Get_stream()) {
        if (result == Io_result::OK) {
            mav_stream->Get_decoder().Decode(buffer);
            Schedule_next_read(mav_stream);
        } else {
            /* Stream error during detection. */
            mav_stream->Get_stream()->Close();
        }
    }
}

void
Mavlink_vehicle_manager::On_manager_disable()
{
}

bool
Mavlink_vehicle_manager::On_timer()
{
    for (auto iter = vehicles.begin(); iter != vehicles.end(); ) {
        auto v = iter->second.vehicle;
        if (!v->is_active || iter->second.stream->Is_closed()) {
            v->Disable();
            iter = vehicles.erase(iter);
        } else {
            iter++;
        }
    }

    std::lock_guard<std::mutex> lock(detector_mutex);
    for (auto iter = detectors.begin(); iter != detectors.end(); ) {
        auto mav_stream = iter->first;
        auto &ctx = iter->second;
        auto &stats = mav_stream->Get_decoder().Get_common_stats();

        /* Decrement timer to catch timeout */
        ctx.timeout--;
        /* Received enough bytes, but still have not matched any valid Mavlink message...
         * OR...
         * Timeout occurred
         */
        if (    (   stats.bytes_received > MAX_UNDETECTED_BYTES
                &&  stats.handled == 0
                &&  stats.no_handler == 0)
            ||  ctx.timeout == 0) {
            auto stream = mav_stream->Get_stream();
            if (stream && !stream->Is_closed()) {
                LOG_INFO("Mavlink not detected on stream [%s].", stream->Get_name().c_str());
                stream->Close();
            }
            iter = detectors.erase(iter);
        } else {
            iter++;
        }
    }
    return true;
}

void
Mavlink_vehicle_manager::Handle_new_connection(
        std::string name,
        int baud,
        ugcs::vsm::Socket_address::Ptr peer_addr,
        ugcs::vsm::Io_stream::Ref stream,
        ugcs::vsm::mavlink::MAV_AUTOPILOT autopilot_type,
        ugcs::vsm::Optional<std::string> custom_model_name,
        ugcs::vsm::Optional<std::string> custom_serial_number)
{
    auto mav_stream = ugcs::vsm::Mavlink_stream::Create(stream);
    mav_stream->Bind_decoder_demuxer();

    LOG_INFO("New connection [%s:%d].", name.c_str(), baud);

    // Register HB handler which is going to get processed in manager_thread.
    // This means that each HB is processed twice: in vehicle and in here.
    // But that is not a big overhead. To fix this we need to "unregister"
    // this generic handler for created vehicles. Current demuxer implementation
    // does not support this.
    mav_stream->Get_demuxer().
        Register_handler<mavlink::MESSAGE_ID::HEARTBEAT, mavlink::Extension>(
        Mavlink_demuxer::Make_handler<mavlink::MESSAGE_ID::HEARTBEAT, mavlink::Extension>(
            &Mavlink_vehicle_manager::On_heartbeat,
            Shared_from_this(),
            mav_stream),
            Mavlink_demuxer::SYSTEM_ID_ANY,
            Mavlink_demuxer::COMPONENT_ID_ANY,
            Shared_from_this());

    {
        std::lock_guard<std::mutex> lock(detector_mutex);
        detectors.emplace(
            std::piecewise_construct,
            std::forward_as_tuple(mav_stream),
            std::forward_as_tuple(
                detection_timeout / TIMER_INTERVAL,
                peer_addr,
                custom_model_name,
                custom_serial_number,
                autopilot_type));
    }

    mav_stream->Get_decoder().Register_raw_data_handler(
        ugcs::vsm::Mavlink_stream::Decoder::Make_raw_data_handler(
            &Mavlink_vehicle_manager::On_raw_data,
            Shared_from_this(),
            mav_stream));

    // Workaround for case when vehicle is connected via UDP via esp-link device.
    // It sends telemetry to broadcast address by default and starts unicasting only
    // if there is traffic from GS. Let's send valid HB message to trigger the
    // unicasting and then proceed as usual. (Vehicle is sending HB to vehicle after detection already).
    // TODO: Remove this once I learn how to receive broadcasts on connected UDP socket.
    if (stream->Get_type() == Io_stream::Type::UDP) {
        mavlink::Pld_heartbeat hb;
        hb->custom_mode = 0;
        hb->type = mavlink::MAV_TYPE::MAV_TYPE_GCS;
        hb->autopilot = mavlink::MAV_AUTOPILOT::MAV_AUTOPILOT_INVALID;
        hb->base_mode = mavlink::MAV_MODE::MAV_MODE_PREFLIGHT;
        hb->system_status = mavlink::MAV_STATE::MAV_STATE_UNINIT;
        mav_stream->Send_message(
            hb,
            vsm_system_id,
            Mavlink_vehicle::VSM_COMPONENT_ID,
            Mavlink_vehicle::WRITE_TIMEOUT,
            Make_timeout_callback(
                &Mavlink_vehicle_manager::Write_to_vehicle_timed_out, this, mav_stream),
                manager_worker);
    }

    Schedule_next_read(mav_stream);
}
