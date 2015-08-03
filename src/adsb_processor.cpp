// Copyright (c) 2014, Smart Projects Holdings Ltd
// All rights reserved.
// See LICENSE file for license details.

#include <adsb_processor.h>
#include <cpr_decoder.h>
//#include <iostream>

ugcs::vsm::Singleton<Adsb_processor> Adsb_processor::singleton;

const size_t Adsb_processor::MAX_AIRCRAFTS = 10000; /* 10K */

Airborne_cpr_decoder Adsb_processor::Aircraft::airborne_decoder;

Surface_cpr_decoder Adsb_processor::Aircraft::surface_decoder;

Adsb_processor::Adsb_processor() :
        ugcs::vsm::Request_processor("Adsb processor")
{

}

void
Adsb_processor::Add_device(Adsb_device::Ptr device)
{
    std::unique_lock<std::mutex> lock(mutex);
    if (aircrafts.find(device) != aircrafts.end()) {
        VSM_EXCEPTION(ugcs::vsm::Invalid_op_exception, "ADS-B device already added");
    }
    Adsb_device_ctx& ctx = aircrafts.emplace(device, Adsb_device_ctx()).first->second;
    if (_initiate_reading) {
        Schedule_frame_read(device, ctx);
    }
}

void
Adsb_processor::Remove_device(Adsb_device::Ptr device)
{
    std::unique_lock<std::mutex> lock(mutex);
    /* Make a copy, to protect from aircrafts map modification during iteration. */
    std::list<Adsb_device::Ptr> devices_copy;
    auto iter = aircrafts.find(device);
    if(iter != aircrafts.end()) {
   		Unschedule_frame_read(iter->second);
        for(auto& craft: iter->second.aircrafts) {
            craft.second->Disable();
        }
        device->Disable();
        aircrafts.erase(device);
    		LOG_INFO("Device [%s] has been removed.",
        		iter->first->Get_name().c_str());
    }
    else {
    LOG_ERROR("Attempting to remove non-existent device!");
    ASSERT(false);
    }
}

Adsb_processor::Reports_stream::Reports_stream(Adsb_processor::Ptr processor) :
        processor(processor)
{
}

ugcs::vsm::Operation_waiter
Adsb_processor::Reports_stream::Read(
        Report_handler handler,
        ugcs::vsm::Request_completion_context::Ptr ctx)
{
    auto proc = processor.lock();
    if (!proc) {
        VSM_EXCEPTION(ugcs::vsm::Invalid_param_exception, "Processor already destroyed.");
    }
    auto req = Read_request::Create();
    req->Set_processing_handler(Make_callback(
            &Adsb_processor::Reports_stream::On_read,
            Shared_from_this(),
            req));

    req->Set_completion_handler(ctx,
            Make_callback(
                    &Adsb_processor::Reports_stream::Read_request::On_completed,
                    req,
                    handler));

    proc->Submit_request(req);

    return req;
}

void
Adsb_processor::Reports_stream::Close()
{
    auto proc = processor.lock();
    if (!proc) {
        VSM_EXCEPTION(ugcs::vsm::Invalid_param_exception, "Processor already destroyed.");
    }
    auto req = Request::Create();
    req->Set_processing_handler(
            Make_callback(
                    &Adsb_processor::On_stream_close,
                    proc,
                    Shared_from_this(),
                    req));
    proc->Submit_request(req);
}

size_t
Adsb_processor::Reports_stream::Get_lost_reports()
{
    return lost_reports.exchange(0);
}

void
Adsb_processor::Reports_stream::Disable()
{
    while (!requests.empty()) {
        requests.front()->Abort();
        requests.pop();
    }
}

void
Adsb_processor::Reports_stream::Read_request::On_completed(Report_handler handler)
{
    handler(report);
}

void
Adsb_processor::Reports_stream::On_read(Read_request::Ptr request)
{
    requests.push(request);
    Push_read_queue();
}

void
Adsb_processor::Reports_stream::Push_report(const ugcs::vsm::Adsb_report& report)
{
    if (reports.size() < MAX_PENDING) {
        reports.push(report);
        Push_read_queue();
    } else {
        lost_reports.fetch_add(1);
        LOG_WARN("ADS-B report is lost, current loss is %zu reports.", lost_reports.load());
    }
}

void
Adsb_processor::Reports_stream::Push_read_queue()
{
    while(!reports.empty() && !requests.empty()) {
        auto req = requests.front();
        requests.pop();

        auto lock = req->Lock();
        if (!req->Is_done()) {
            /* Fill the report data for the user. */
            req->report = reports.front();
            reports.pop();
        }

        /* Complete in done state doesn't harm. */
        req->Complete(Request::Status::OK, std::move(lock));
    }
}

Adsb_processor::Reports_stream::Ptr
Adsb_processor::Open_stream()
{
    auto stream = Reports_stream::Create(Shared_from_this());
    auto req = Request::Create();
    req->Set_processing_handler(
            Make_callback(
                    &Adsb_processor::On_stream_open,
                    Shared_from_this(),
                    stream,
                    req));
    Submit_request(req);
    req->Wait_done(false);
    return stream;
}

std::string
Adsb_processor::Aircraft::State_to_str(State state)
{
    switch (state) {
    case State::INITIALIZATION: return "INITIALIZATION";
    case State::ACQUISITION: return "ACQUISITION";
    case State::TRACKING: return "TRACKING";
    default: VSM_EXCEPTION(ugcs::vsm::Internal_error_exception, "Unknown aircraft state");
    }
}

Adsb_processor::Aircraft::Aircraft(
        const ugcs::vsm::Adsb_frame::ICAO_address& address,
        Destroy_handler destory_handler,
        Report_handler report_handler) :
    address(address),
    destroy_handler(destory_handler),
    report_handler(report_handler)
{

}

void
Adsb_processor::Aircraft::Disable()
{
    if (timer) {
        timer->Cancel();
        timer = nullptr;
    }
}

void
Adsb_processor::Aircraft::Process(
        const ugcs::vsm::Adsb_frame::Airborne_position_message& msg,
        ugcs::vsm::Request_completion_context::Ptr& completion_ctx)
{
    if (airborne && !*airborne) {
        /* Switch from on ground to airborne. Reinit. */
        LOG_INFO("Aircraft %s takeoff detected.", To_string().c_str());
        Set_state(State::INITIALIZATION);
    }
    airborne = true;
    if (msg.Is_altitude_available()) {
        altitude = msg.Get_altitude();
    }
    if (Process_position(msg, airborne_decoder)) {
        Reschedule_timer(completion_ctx);
    }
}

void
Adsb_processor::Aircraft::Process(
        const ugcs::vsm::Adsb_frame::Surface_position_message& msg,
        ugcs::vsm::Request_completion_context::Ptr& completion_ctx)
{
    if (airborne && *airborne) {
        /* Switch from from airborne to surface. Reinit. */
        LOG_INFO("Aircraft %s landing detected.", To_string().c_str());
        Set_state(State::INITIALIZATION);
    }
    airborne = false;
    if (msg.Is_heading_available()) {
        heading = msg.Get_heading();
    }
    if (msg.Is_speed_available()) {
        horizontal_speed = msg.Get_speed();
    }
    if (Process_position(msg, surface_decoder)) {
        Reschedule_timer(completion_ctx);
    }
}

void
Adsb_processor::Aircraft::Process(
        const ugcs::vsm::Adsb_frame::Aircraft_id_and_cat_message& msg,
        ugcs::vsm::Request_completion_context::Ptr& completion_ctx)
{
    bool present = ident ? true : false;

    if ((ident && *ident != msg.Get_id()) ||
        (category && *category != msg.Get_emitter_category())) {
        LOG_INFO("ADS-B aircraft %s identification changed on the fly.",
                To_string().c_str());
        Set_state(State::INITIALIZATION);
    }
    ident = msg.Get_id();
    category = msg.Get_emitter_category();

    if (!present) {
        LOG_INFO("Aircraft %s has identified itself.",
                To_string().c_str());
    }

    if (state == State::INITIALIZATION) {
        /* Only init state is prolonged by this message. */
        Reschedule_timer(completion_ctx);
    }
}

void
Adsb_processor::Aircraft::Process(
        const ugcs::vsm::Adsb_frame::Airborne_velocity_message& message,
        ugcs::vsm::Request_completion_context::Ptr& completion_context)
{
    if (message.Is_horizontal_speed_available()) {
        horizontal_speed = message.Get_horizontal_speed();
    }

    if (message.Is_vertical_speed_available()) {
        vertical_speed = message.Get_vertical_speed();
    }

    if (message.Is_heading_available()) {
        heading = message.Get_heading();
    }

    if (state == State::ACQUISITION) {
        if (second_global_decode_done) {
            Set_state(State::TRACKING);
        }
    }
    if (state == State::TRACKING) {
        /* Do not generate a report immediately, because it produces a report
         * with the same coordinates as in previous report which is confusing.
         * Solution is to have a separate message for the velocity, for example. */
        //Generate_report();
    }
    if (state == State::INITIALIZATION) {
        Reschedule_timer(completion_context);
    }
}

std::string
Adsb_processor::Aircraft::To_string()
{
    std::string val = "[ICAO:" + address.To_hex_string() + "]";
    if (ident) {
        val += " [" + *ident + "]";
    } else {
        val += " [-no-ident-]";
    }
    return val;
}

void
Adsb_processor::Aircraft::Set_state(Adsb_processor::Aircraft::State new_state)
{
    LOG_DEBUG("ADS-B aircraft %s state %s %s -> %s",
            To_string().c_str(),
            new_state == state ? "reset" : "change",
            State_to_str(state).c_str(), State_to_str(new_state).c_str());

    state = new_state;

    even = ugcs::vsm::Adsb_frame::Airborne_position_message();
    odd = ugcs::vsm::Adsb_frame::Airborne_position_message();

    switch (new_state) {
    case Aircraft::State::INITIALIZATION:
        recent_position.Disengage();
        airborne.Disengage();
        heading.Disengage();
        horizontal_speed.Disengage();
        vertical_speed.Disengage();
        altitude.Disengage();
        break;
    case Aircraft::State::ACQUISITION:
        second_global_decode_done = false;
        break;
    case Aircraft::State::TRACKING:
        break;
    }
}

bool
Adsb_processor::Aircraft::Airborne_velocity_received()
{
    return horizontal_speed || vertical_speed || heading;
}

void
Adsb_processor::Aircraft::Reschedule_timer(
        ugcs::vsm::Request_completion_context::Ptr& completion_ctx)
{
    if (timer) {
        timer->Cancel();
    }
    std::chrono::seconds interval;
    switch (state) {
    case State::INITIALIZATION:
        interval = INITIALIZATION_TIMEOUT;
        break;
    case State::ACQUISITION:
        interval = ACQUISITION_TIMEOUT;
        break;
    case State::TRACKING:
        ASSERT(airborne);
        if (*airborne) {
            interval = TRACKING_TIMEOUT_AIRBORNE;
        } else {
            interval = TRACKING_TIMEOUT_SURFACE;
        }
        break;
    }
    timer = ugcs::vsm::Timer_processor::Get_instance()->
            Create_timer(interval,
                    Make_callback(
                            &Aircraft::Timer_handler,
                            Shared_from_this(),
                            completion_ctx),
                    completion_ctx);
}

bool
Adsb_processor::Aircraft::Timer_handler(
        ugcs::vsm::Request_completion_context::Ptr completion_ctx)
{
    LOG_DEBUG("Aircraft %s state [%s] timed out.",
            To_string().c_str(), State_to_str(state).c_str());

    switch (state) {
    case State::INITIALIZATION:
        Destroy();
        return false;
    case State::ACQUISITION:
        Set_state(State::INITIALIZATION);
        break;
    case State::TRACKING:
        Set_state(State::INITIALIZATION);
        break;
    }
    Reschedule_timer(completion_ctx);
    return false;
}

bool
Adsb_processor::Aircraft::Process_position(
        const ugcs::vsm::Adsb_frame::Position_message& pos,
        const Cpr_decoder& decoder)
{
    ugcs::vsm::Optional<double> distance_to_receiver;

    if (pos.Get_CPR_format()) {
        odd = pos;
    } else {
        even = pos;
    }

    switch (state) {
    case State::INITIALIZATION:
        if (!even.Is_empty() && !odd.Is_empty()) {
            recent_position = decoder.Global_decode(even, odd);
            if (!recent_position ||
                !decoder.Check_receiver_distance(*recent_position, distance_to_receiver)) {
                break;
            }
            /* First global decode OK. */
            if (*airborne) {
                /* Only airborne requires additional ambiguity check. */
                Set_state(State::ACQUISITION);
            } else {
                Set_state(State::TRACKING);
            }
        }
        break;
    case State::ACQUISITION:
        recent_position = decoder.Local_decode(*recent_position, pos);
        if (!second_global_decode_done && !even.Is_empty() && !odd.Is_empty()) {
            auto second_global = decoder.Global_decode(even, odd);
            if (second_global) {
                /* Second global decode is OK, compare with recent local to agree. */
                double distance =
                        ugcs::vsm::Wgs84_position(*recent_position).Distance(*second_global);
                if (decoder.Check_global_local_distance(distance)) {
                    second_global_decode_done = true;
                    if (Airborne_velocity_received()) {
                        Set_state(State::TRACKING);
                    }
                    /* Otherwise wait for velocity to come. */
                } else {
                    LOG_INFO("Global/local ambiguity check for aircraft %s "
                            "failed with distance %f between (%f,%f) and (%f,%f).",
                            To_string().c_str(),
                            distance,
                            (*recent_position).latitude, (*recent_position).longitude,
                            (*second_global).latitude, (*second_global).longitude);
                    Set_state(State::INITIALIZATION);
                }
            }
        }
        break;
    case State::TRACKING:
        auto new_local_pos = decoder.Local_decode(*recent_position, pos);
        if (!decoder.Check_receiver_distance(new_local_pos, distance_to_receiver)) {
            LOG_INFO("Aircraft %s new position is too far from the receiver, "
                    "%f meters.", To_string().c_str(), *distance_to_receiver);
            return false;

        }
        if (!decoder.Check_local_distance(new_local_pos, *recent_position)) {
            /* Ignore this message. This could be an message from different
             * aircraft with the same ICAO address. For now, duplicate ICAO
             * addresses are not supported, because it is considered to be
             * a very rare situation. */
            double distance = ugcs::vsm::Wgs84_position(new_local_pos).Distance(*recent_position);
            LOG_INFO("Aircraft %s position ignored, distance delta %f meters.",
                    To_string().c_str(), distance);
            return false;
        }
        recent_position = new_local_pos;
        Generate_report();
        break;
    }

    return true;
}

void
Adsb_processor::Aircraft::Generate_report()
{
    ASSERT(recent_position);
    report_handler(std::move(ugcs::vsm::Adsb_report(address, ident, *recent_position, altitude, heading,
            horizontal_speed, vertical_speed)));
}

void
Adsb_processor::Aircraft::Destroy()
{
    destroy_handler();
}

void
Adsb_processor::On_enable()
{
    completion_ctx = ugcs::vsm::Request_completion_context::Create("Adsb processor completion");
    worker = ugcs::vsm::Request_worker::Create(
            "Adsb processor worker",
            std::initializer_list<ugcs::vsm::Request_container::Ptr>{
                completion_ctx, Shared_from_this()});
    completion_ctx->Enable();
    worker->Enable();
}

void
Adsb_processor::On_disable()
{
    auto req = Request::Create();
    req->Set_processing_handler(
            Make_callback(
                    &Adsb_processor::Process_on_disable,
                    Shared_from_this(),
                    req));
    Submit_request(req);
    req->Wait_done(false);
    Set_disabled();
    worker->Disable();
}

void
Adsb_processor::Process_on_disable(Request::Ptr request)
{
    /* Make a copy, to protect from aircrafts map modification during iteration. */
    std::list<Adsb_device::Ptr> devices_copy;
    for (auto &iter: aircrafts) {
        devices_copy.push_back(iter.first);
    }

    for (auto &iter: devices_copy) {
        /* Disable the device. */
//    	if (iter->Is_enabled())
        iter->Disable();
    }
    for (auto& ctx: aircrafts) {
        for(auto& craft: ctx.second.aircrafts) {
            /* Disable aircraft. */
            craft.second->Disable();
        }
    }

    aircrafts.clear();
    if (streams.size()) {
        LOG_ERR("Adsb processor still has %zu report streams opened while disabling.",
                streams.size());
        ASSERT(false);
    }
    for (auto& stream: streams) {
        stream->Disable();
    }
    completion_ctx->Disable();
    completion_ctx = nullptr;
    request->Complete();
}

void
Adsb_processor::Schedule_frame_read(
        Adsb_device::Ptr device,
        Adsb_device_ctx& ctx)
{
    ctx.read_frame_op.Abort();
    ctx.read_frame_op = device->Read_frame(
            Adsb_device::Make_read_frame_handler(
                    &Adsb_processor::Adsb_frame_received,
                    Shared_from_this(),
                    device),
                    completion_ctx);
}

void
Adsb_processor::Unschedule_frame_read(
        Adsb_device_ctx& ctx)
{
    ctx.read_frame_op.Abort();
}
//XXX
void
Adsb_processor::Adsb_frame_received(
        ugcs::vsm::Io_buffer::Ptr buffer,
        ugcs::vsm::Io_result result,
        Adsb_device::Ptr device)
{
    auto ctx_iter = aircrafts.find(device);
    ASSERT(ctx_iter != aircrafts.end());

//    std::cout << "Frame received: " << buffer->Get_hex() << ". ICAO address: [" << buffer->Get_hex().substr(2, 6);

    if (result == ugcs::vsm::Io_result::OK) {
        Schedule_frame_read(device, ctx_iter->second);
        if (buffer->Get_length() == ugcs::vsm::Adsb_frame::SIZE) {
            ugcs::vsm::Adsb_frame::Ptr frame = ugcs::vsm::Adsb_frame::Create(buffer);
            if (!frame->Verify_checksum()) {
//            	std::cout << "] Checksum error!\n";
                LOG_DEBUG("ADS-B frame dropped, bad checksum.");
                return;
            }
//        	std::cout << "]\n";
            switch (frame->Get_DF()) {
            case ugcs::vsm::Adsb_frame::Downlink_format::DF_17:
                Process_DF_17(frame, device);
                break;
            case ugcs::vsm::Adsb_frame::Downlink_format::DF_18:
                Process_DF_18(frame, device);
                break;
            case ugcs::vsm::Adsb_frame::Downlink_format::DF_19:
                Process_DF_19(frame, device);
                break;
            default:
//            	std::cout << "] Wrong format!\n";
                LOG_DEBUG("Unsupported ADS-B downlink format (%d), dropped.", frame->Get_DF());
                break;
            }
        } else {
//        	std::cout << "] Wrong size!\n";
            LOG_DEBUG("ADS-B frame incorrect size (%zu)", buffer->Get_length());
        }
    } else {
        /* Disable and erase all aircrafts tracked by this device. */
        for (auto& craft: ctx_iter->second.aircrafts) {
            craft.second->Disable();
        }
        /* Disable the device. */
        ctx_iter->first->Disable();
        aircrafts.erase(ctx_iter);
        LOG_INFO("ADS-B frame read error (%d), removing device [%s].",
                result, device->Get_name().c_str());
    }
}

Adsb_processor::Aircraft::Ptr
Adsb_processor::Lookup_aircarft(
        const Adsb_device::Ptr& device,
        const ugcs::vsm::Adsb_frame::ICAO_address& address,
        bool check_limit)
{
    Aircraft::Ptr acraft;

    auto device_iter = aircrafts.find(device);
    ASSERT(device_iter != aircrafts.end());
    auto& map = device_iter->second.aircrafts;
    auto iter = map.find(address);
    if (iter == map.end()) {
        if (check_limit && map.size() >= MAX_AIRCRAFTS) {
            LOG_WARNING("Too many ADS-B aircraft, new aircraft [%s] ignored. "
                        "Do you really have around %zu visible aircrafts? "
                        "If not, please submit a bug report.",
                        address.To_hex_string().c_str(),
                        MAX_AIRCRAFTS);
            return nullptr;
        }

        acraft = Aircraft::Create(
                address,
                Make_callback(
                        &Adsb_processor::Aircraft_destroyed,
                        Shared_from_this(),
                        device,
                        address),
                Make_callback(
                        &Adsb_processor::Aircraft_report,
                        Shared_from_this(),
                        ugcs::vsm::Adsb_report()));


        map[address] = acraft;
        LOG_INFO("New ADS-B aircraft %s created by device [%s].",
                acraft->To_string().c_str(),
                device->Get_name().c_str());
    } else {
        acraft = iter->second;
    }
    return acraft;
}

void
Adsb_processor::Aircraft_destroyed(
        Adsb_device::Ptr device,
        ugcs::vsm::Adsb_frame::ICAO_address address)
{
    auto ctx = aircrafts.find(device);
    ASSERT(ctx != aircrafts.end());
    auto& map = ctx->second.aircrafts;
    auto iter = map.find(address);
    ASSERT(iter != map.end());
    LOG_INFO("Aircraft %s removed, device [%s] still has %zu aircraft.",
            iter->second->To_string().c_str(),
            device->Get_name().c_str(), map.size() - 1);
    iter->second->Disable();
    map.erase(iter);
}

void
Adsb_processor::Aircraft_report(ugcs::vsm::Adsb_report report)
{
    for(auto& stream: streams) {
        stream->Push_report(report);
    }
}

void
Adsb_processor::Process_DF_17(ugcs::vsm::Adsb_frame::Ptr& frame, Adsb_device::Ptr& device)
{
    /* All DF_17 frames has ME field. */
    Process_ME_field(frame, device);
}

void
Adsb_processor::Process_DF_18(ugcs::vsm::Adsb_frame::Ptr& frame, Adsb_device::Ptr& device)
{
    /* Filter frames based on CF which have ME field. */
    switch (frame->Get_CF()) {
    case ugcs::vsm::Adsb_frame::CF_values::CF_0:
    case ugcs::vsm::Adsb_frame::CF_values::CF_1:
    case ugcs::vsm::Adsb_frame::CF_values::CF_6:
        Process_ME_field(frame, device);
        break;
    default:
        LOG_DEBUG("ADS-B frame CF value %d not supported, ignored.", frame->Get_CF());
    }
}

void
Adsb_processor::Process_DF_19(ugcs::vsm::Adsb_frame::Ptr& frame, Adsb_device::Ptr& device)
{
    /* Filter frames based on AF which have ME field. */
    switch (frame->Get_AF()) {
    case ugcs::vsm::Adsb_frame::AF_values::AF_0:
        Process_ME_field(frame, device);
        break;
    default:
        LOG_DEBUG("ADS-B frame AF value %d not supported, ignored.", frame->Get_AF());
    }
}

void
Adsb_processor::Process_ME_field(ugcs::vsm::Adsb_frame::Ptr& frame, Adsb_device::Ptr& device)
{
    /* Message type shortcut. */
    using Type = ugcs::vsm::Adsb_frame::ME_message::Type;

    switch (frame->Get_ME_type()) {
    case Type::AIRCRAFT_ID_AND_CAT_A_4:
    case Type::AIRCRAFT_ID_AND_CAT_B_3:
    case Type::AIRCRAFT_ID_AND_CAT_C_2:
    case Type::AIRCRAFT_ID_AND_CAT_D_1:
        Process_ME_message<ugcs::vsm::Adsb_frame::Aircraft_id_and_cat_message>(frame, device);
        break;
    case Type::SURFACE_POSITION_5:
    case Type::SURFACE_POSITION_6:
    case Type::SURFACE_POSITION_7:
    case Type::SURFACE_POSITION_8:
        Process_ME_message<ugcs::vsm::Adsb_frame::Surface_position_message>(frame, device);
        break;
    case Type::AIRBORNE_POSITION_9:
    case Type::AIRBORNE_POSITION_10:
    case Type::AIRBORNE_POSITION_11:
    case Type::AIRBORNE_POSITION_12:
    case Type::AIRBORNE_POSITION_13:
    case Type::AIRBORNE_POSITION_14:
    case Type::AIRBORNE_POSITION_15:
    case Type::AIRBORNE_POSITION_16:
    case Type::AIRBORNE_POSITION_17:
    case Type::AIRBORNE_POSITION_18:
    case Type::AIRBORNE_POSITION_20:
    case Type::AIRBORNE_POSITION_21:
    case Type::AIRBORNE_POSITION_22:
        Process_ME_message<ugcs::vsm::Adsb_frame::Airborne_position_message>(frame, device);
        break;
    case Type::AIRBORNE_VELOCITY_19:
        Process_airborne_velocity_message(frame, device);
        break;
    case Type::NO_POSITION_0:
    case Type::TEST_23:
    case Type::SURFACE_SYSTEM_STATUS_24:
    case Type::RESERVED_25:
    case Type::RESERVED_26:
    case Type::TRAJECTORY_CHANGE_RESERVED_27:
    case Type::STATUS_MESSAGE_28:
    case Type::TARGET_STATE_AND_STATUS_29:
    case Type::RESERVED_30:
    case Type::AIRCRAFT_OPERATIONAL_STATUS_31:
        /* Ignored. */
        break;
    default:
        /* All 5-bit values are handled in case, there should not be
         * anything else.
         */
        LOG_ERROR("Unexpected ADS-B ME frame type %d, ignored.",
                frame->Get_ME_type());
        ASSERT(false);
        break;
    }
}

void
Adsb_processor::Process_airborne_velocity_message(
        ugcs::vsm::Adsb_frame::Ptr& frame, Adsb_device::Ptr& device)
{
    /* Additional validation for sub-type. */
    auto sub_type = frame->Get_ME_subtype();
    if (sub_type == ugcs::vsm::Adsb_frame::ME_message::Sub_type::TYPE_1 ||
        sub_type == ugcs::vsm::Adsb_frame::ME_message::Sub_type::TYPE_2 ||
        sub_type == ugcs::vsm::Adsb_frame::ME_message::Sub_type::TYPE_3 ||
        sub_type == ugcs::vsm::Adsb_frame::ME_message::Sub_type::TYPE_4) {
        Process_ME_message<ugcs::vsm::Adsb_frame::Airborne_velocity_message>(frame, device);
    }
}

void
Adsb_processor::On_stream_open(Reports_stream::Ptr stream, Request::Ptr request)
{
    ASSERT(streams.find(stream) == streams.end());

    streams.insert(stream);

    request->Complete();
}

void
Adsb_processor::On_stream_close(Reports_stream::Ptr stream, Request::Ptr request)
{
    auto iter = streams.find(stream);
    /* Double close is OK. */
    if (iter != streams.end()) {
        streams.erase(iter);
        stream->Disable();
    }
    request->Complete();
}
