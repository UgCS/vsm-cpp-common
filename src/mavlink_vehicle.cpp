// Copyright (c) 2014, Smart Projects Holdings Ltd
// All rights reserved.
// See LICENSE file for license details.

#include <mavlink_vehicle.h>
#include <fstream>

constexpr std::chrono::seconds Mavlink_vehicle::WRITE_TIMEOUT;

using namespace vsm;

void
Mavlink_vehicle::On_enable()
{
    auto request = Request::Create();
    request->Set_processing_handler(
            Make_callback(&Mavlink_vehicle::On_enable_vehicle, this, request));
    Get_processing_ctx()->Submit_request(request);
}

void
Mavlink_vehicle::On_disable()
{
    auto request = Request::Create();
    request->Set_processing_handler(
            Make_callback(&Mavlink_vehicle::On_disable_vehicle, this, request));
    Get_processing_ctx()->Submit_request(request);
    Operation_waiter waiter(request);
    waiter.Wait(false);
}

void
Mavlink_vehicle::On_enable_vehicle(Request::Ptr request)
{
    mav_stream->Bind_decoder_demuxer();
    Schedule_next_read();
    /* Consider this is uptime start. */
    last_connect = std::chrono::steady_clock::now();
    Wait_for_vehicle();

#if 0
    mav_stream->Get_demuxer().Register_default_handler(
            Make_mavlink_demuxer_default_handler(
                    &Mavlink_vehicle::Default_mavlink_handler,
                    this));
#endif

    request->Complete();
}

void
Mavlink_vehicle::On_disable_vehicle(Request::Ptr request)
{
    Disable_activities();

    read_op.Abort();

    mav_stream->Disable();
    mav_stream = nullptr;

    request->Complete();
}

void
Mavlink_vehicle::Wait_for_vehicle()
{
    Disable_activities();
    heartbeat.Enable();
    statistics.Enable();
}

bool
Mavlink_vehicle::Default_mavlink_handler(mavlink::MESSAGE_ID_TYPE message_id,
        mavlink::System_id system_id, uint8_t component_id)
{
    LOG_WARN("Message %u unsupported from [%u:%u].", message_id,
            system_id, component_id);
    return false;
}

void
Mavlink_vehicle::On_read_handler(Io_buffer::Ptr buffer, Io_result result)
{
    /* Check for spurious read completion after vehicle is disabled. */
    if (Is_enabled()) {
        if (result == Io_result::OK) {
            mav_stream->Get_decoder().Decode(buffer);
            Schedule_next_read();
        } else {
            mav_stream->Get_stream()->Close();
            Disable_activities();
        }
    }
}

void
Mavlink_vehicle::Schedule_next_read()
{
    read_op.Abort();
    size_t to_read = mav_stream->Get_decoder().Get_next_read_size();
    size_t max_read = forced_max_read ? forced_max_read : to_read;
    read_op = mav_stream->Get_stream()->Read(max_read, to_read,
            Make_read_callback(&Mavlink_vehicle::On_read_handler,
                               Shared_from_this()),
            Get_completion_ctx());
}

void
Mavlink_vehicle::Disable_activities()
{
    for (auto act : activities) {
        act->Disable();
    }
}

void
Mavlink_vehicle::Write_to_vehicle_timed_out(
        const Operation_waiter::Ptr& waiter,
        Mavlink_stream::Weak_ptr mav_stream)
{
    auto locked = mav_stream.lock();
    Io_stream::Ref stream = locked ? locked->Get_stream() : nullptr;
    std::string server_info =
            stream ? stream->Get_name() : "already disconnected";
    LOG_DEBUG("Write timeout towards Vehicle [%s:%s] at [%s] detected.",
            Get_model_name().c_str(), Get_serial_number().c_str(),
            server_info.c_str());
    waiter->Abort();
}

void
Mavlink_vehicle::Activity::Disable()
{
    On_disable();
    next_action = Next_action();
}

void
Mavlink_vehicle::Activity::Set_next_action(Next_action next_action)
{
    this->next_action = next_action;
}

void
Mavlink_vehicle::Activity::Call_next_action(bool success)
{
    auto action = std::move(next_action);
    Disable();
    if (action) {
        action(success);
    }
}

void
Mavlink_vehicle::Heartbeat::Enable()
{
    first_ok_received = false;
    received_count = 0;
    Register_mavlink_handler<mavlink::MESSAGE_ID::HEARTBEAT>(
            &Heartbeat::On_heartbeat,
            this,
            Mavlink_demuxer::COMPONENT_ID_ANY);

    timer = Timer_processor::Get_instance()->
            Create_timer(std::chrono::seconds(MAX_INTERVAL),
                    Make_callback(&Mavlink_vehicle::Heartbeat::On_timer, this),
                    vehicle.Get_completion_ctx());
}

void
Mavlink_vehicle::Heartbeat::On_disable()
{
    Unregister_mavlink_handlers();
    if (timer) {
        timer->Cancel();
        timer = nullptr;
    }
}

void
Mavlink_vehicle::Heartbeat::On_heartbeat(
        mavlink::Message<mavlink::MESSAGE_ID::HEARTBEAT>::Ptr message)
{
    received_count++;
    mavlink::MAV_STATE system_status =
            static_cast<mavlink::MAV_STATE>(message->payload->system_status.Get());
    auto uptime = std::chrono::duration_cast<std::chrono::seconds>(
            std::chrono::steady_clock::now() - vehicle.last_connect);

    vehicle.Set_system_status(
            static_cast<mavlink::MAV_MODE_FLAG>(message->payload->base_mode.Get()),
            system_status,
            Vehicle::Custom_mode(true, true),
            uptime);
    if (!Is_system_status_ok(system_status)) {
        LOG_INFO("Heartbeat system status not OK (%d)", system_status);
    } else if (!first_ok_received) {
        first_ok_received = true;
        LOG_INFO("First heartbeat received");
        vehicle.telemetry.Enable();
        /* Don't read waypoints. It is for debug only. */
        //vehicle.read_waypoints.Enable();
    }
}

bool
Mavlink_vehicle::Heartbeat::On_timer()
{
    if (!received_count) {
        if (first_ok_received) {
            LOG_INFO("Heartbeat lost. Still waiting...");
            vehicle.Wait_for_vehicle();
        } else {
            LOG_INFO("Heartbeat lost. Vehicle disconnected.");
            /* Will trigger disconnect handler. */
            vehicle.mav_stream->Get_stream()->Close();
        }
    }
    received_count = 0;
    return true;
}

bool
Mavlink_vehicle::Heartbeat::Is_system_status_ok(mavlink::MAV_STATE system_status)
{
    /* Skip obviously bad statuses which doesn't assume operable vehicle. */
    return system_status == mavlink::MAV_STATE::MAV_STATE_STANDBY ||
            system_status == mavlink::MAV_STATE::MAV_STATE_ACTIVE ||
            system_status == mavlink::MAV_STATE::MAV_STATE_CRITICAL ||
            system_status == mavlink::MAV_STATE::MAV_STATE_EMERGENCY;
}

void
Mavlink_vehicle::Statistics::Enable()
{
    auto timer_proc = Timer_processor::Get_instance();
    timer = timer_proc->Create_timer(std::chrono::seconds(COLLECTION_INTERVAL),
            Make_callback(&Mavlink_vehicle::Statistics::On_timer, this),
            vehicle.Get_completion_ctx());

    Register_mavlink_handler<mavlink::MESSAGE_ID::STATUSTEXT>(&Statistics::On_status_text, this,
            Mavlink_demuxer::COMPONENT_ID_ANY);
}

void
Mavlink_vehicle::Statistics::On_disable()
{
    if (timer) {
        timer->Cancel();
        timer = nullptr;
    }
    num_of_processed_messages = 0;
    Unregister_mavlink_handlers();
}

bool
Mavlink_vehicle::Statistics::On_timer()
{
    auto& stats = vehicle.mav_stream->Get_decoder().Get_stats();
    LOG_INFO("%lld Mavlink messages processed, link quality %.1f%%",
            static_cast<long long>(stats.handled - num_of_processed_messages),
            vehicle.telemetry.link_quality * 100);
    num_of_processed_messages = stats.handled;
    return true;
}

void
Mavlink_vehicle::Statistics::On_status_text(
        mavlink::Message<mavlink::MESSAGE_ID::STATUSTEXT>::Ptr message)
{
    LOG_INFO("STATUS_TEXT: %s", message->payload->text.Get_string().c_str());
}

void
Mavlink_vehicle::Read_parameters::Enable()
{
    attempts_left = ATTEMPTS;
    parameters.clear();

    Register_mavlink_handler<mavlink::MESSAGE_ID::PARAM_VALUE>(
            &Read_parameters::On_param_value, this, Mavlink_demuxer::COMPONENT_ID_ANY);
    Try();
}

void
Mavlink_vehicle::Read_parameters::On_disable()
{
    if (timer) {
        timer->Cancel();
        timer = nullptr;
    }

    Unregister_mavlink_handlers();
}

bool
Mavlink_vehicle::Read_parameters::Try()
{
    if (!attempts_left--) {
        Disable();
        return false;
    }
    mavlink::Pld_param_request_list msg;
    Fill_target_ids(msg);
    Send_message(msg);
    Schedule_timer();
    return false;
}

void
Mavlink_vehicle::Read_parameters::Schedule_timer()
{
    if (timer) {
        timer->Cancel();
    }
    timer = Timer_processor::Get_instance()->Create_timer(
            std::chrono::seconds(RETRY_TIMEOUT),
            Make_callback(&Read_parameters::Try, this),
            vehicle.Get_completion_ctx());
}

void
Mavlink_vehicle::Read_parameters::On_param_value(
        mavlink::Message<mavlink::MESSAGE_ID::PARAM_VALUE>::Ptr message)
{
    switch (message->payload->param_type) {
    case mavlink::MAV_PARAM_TYPE::MAV_PARAM_TYPE_UINT8:
    case mavlink::MAV_PARAM_TYPE::MAV_PARAM_TYPE_UINT16:
    case mavlink::MAV_PARAM_TYPE::MAV_PARAM_TYPE_UINT32:
        Print_param(message, static_cast<uint32_t>(message->payload->param_value));
        break;
    case mavlink::MAV_PARAM_TYPE::MAV_PARAM_TYPE_INT8:
    case mavlink::MAV_PARAM_TYPE::MAV_PARAM_TYPE_INT16:
    case mavlink::MAV_PARAM_TYPE::MAV_PARAM_TYPE_INT32:
        Print_param(message, static_cast<int32_t>(message->payload->param_value));
        break;
    case mavlink::MAV_PARAM_TYPE::MAV_PARAM_TYPE_REAL32:
        Print_param(message, static_cast<float>(message->payload->param_value));
        break;
    default:
        LOG_WARNING("PARAM [%s:%d] = Unsupported format!",
                            message->payload->param_id.Get_string().c_str(),
                            message->payload->param_index.Get());
    }
    parameters[message->payload->param_id.Get_string()] = message->payload;
    if (message->payload->param_count - 1 ==
        message->payload->param_index) {
        Disable();
    } else {
        Schedule_timer();
    }
}

void
Mavlink_vehicle::Write_parameters::Enable(const List& parameters_to_write)
{
    Register_mavlink_handler<mavlink::MESSAGE_ID::PARAM_VALUE>(
            &Write_parameters::On_param_value, this, Mavlink_demuxer::COMPONENT_ID_ANY);

    parameters = parameters_to_write;
    attempts_left = ATTEMPTS;
    Try();
}

void
Mavlink_vehicle::Write_parameters::On_disable()
{
    if (timer) {
        timer->Cancel();
        timer = nullptr;
    }
    Unregister_mavlink_handlers();
    parameters.clear();
}

void
Mavlink_vehicle::Write_parameters::On_param_value(
        mavlink::Message<mavlink::MESSAGE_ID::PARAM_VALUE>::Ptr message)
{
    if (message->payload->param_id.Get_string() !=
        parameters.back()->param_id.Get_string()) {
        LOG_WARNING("Parameter writing validation failed. Name [%s] expected, "
                "but [%s] received.",
                parameters.back()->param_id.Get_string().c_str(),
                message->payload->param_id.Get_string().c_str());
        /* Don't reschedule immediately. Maybe it was some out of order
         * message? So just wait for retry timeout.
         */
        return;
    }

    if (message->payload->param_value.Get() !=
        parameters.back()->param_value.Get()) {
        LOG_WARNING("Parameter writing validation failed for [%s]. Value [%f] "
                "expected, but [%f] received.",
                parameters.back()->param_id.Get_string().c_str(),
                parameters.back()->param_value.Get(),
                message->payload->param_value.Get());
        Try();
        return;
    }
    LOG_DEBUG("Parameter [%s:%f] write verified successfully.",
            message->payload->param_id.Get_string().c_str(),
            message->payload->param_value.Get());
    /* Written OK. */
    parameters.pop_back();
    attempts_left = ATTEMPTS;
    Try();
}

bool
Mavlink_vehicle::Write_parameters::Try()
{
    if (parameters.empty()) {
        Call_next_action(true);
        return false;
    }

    if (!attempts_left--) {
        LOG_INFO("All parameters write attempts failed.");
        Call_next_action(false);
        return false;
    }
    LOG_INFO("Writing parameter [%s:%f].",
            parameters.back()->param_id.Get_string().c_str(),
            parameters.back()->param_value.Get());
    Send_message(parameters.back());
    /* Expecting also new parameter value to be send back for verification. */
    Schedule_timer();
    return false;
}

void
Mavlink_vehicle::Write_parameters::Schedule_timer()
{
    if (timer) {
        timer->Cancel();
    }
    timer = Timer_processor::Get_instance()->Create_timer(
            std::chrono::seconds(RETRY_TIMEOUT),
            Make_callback(&Write_parameters::Try, this),
            vehicle.Get_completion_ctx());
}

void
Mavlink_vehicle::Read_waypoints::Enable()
{
    mission_count_handler =
            Register_mavlink_handler<mavlink::MESSAGE_ID::MISSION_COUNT>(
                    &Read_waypoints::On_count,
                    this,
                    Mavlink_demuxer::COMPONENT_ID_ANY);

    mavlink::Pld_mission_request_list req;
    Fill_target_ids(req);
    Send_message(req);
}

void
Mavlink_vehicle::Read_waypoints::On_disable()
{
    Unregister_mavlink_handlers();
    mission_count_handler.Reset();
}

void
Mavlink_vehicle::Read_waypoints::On_count(
        mavlink::Message<mavlink::MESSAGE_ID::MISSION_COUNT>::Ptr message)
{
    if (message->payload->target_component != VSM_COMPONENT_ID ||
        message->payload->target_system != VSM_SYSTEM_ID) {
        /* Not to us, ignore. */
        return;
    }
    waypoints_total = message->payload->count;
    LOG_DEBUG("Reading of %lu waypoints...", waypoints_total);
    waypoint_to_read = 0;
    Unregister_mavlink_handler(mission_count_handler);
    Register_mavlink_handler<mavlink::MESSAGE_ID::MISSION_ITEM>(
            &Read_waypoints::On_item,
            this,
            Mavlink_demuxer::COMPONENT_ID_ANY);
    Read_next();
}

void
Mavlink_vehicle::Read_waypoints::On_item(
        mavlink::Message<mavlink::MESSAGE_ID::MISSION_ITEM>::Ptr message)
{
    if (message->payload->target_component != VSM_COMPONENT_ID ||
        message->payload->target_system != VSM_SYSTEM_ID) {
        /* Not to us, ignore. */
        return;
    }
    if (message->payload->seq == waypoint_to_read) {
        waypoint_to_read++;
        LOG_DEBUG("Waypoint %lu received:", waypoint_to_read);
        LOG_DEBUG("%s", message->payload.Dump().c_str());
    }
    Read_next();
}

void
Mavlink_vehicle::Read_waypoints::Read_next()
{
    if (waypoints_total == waypoint_to_read) {
        mavlink::Pld_mission_ack ack;
        Fill_target_ids(ack);
        ack->type = mavlink::MAV_MISSION_RESULT::MAV_MISSION_ACCEPTED;
        Send_message(ack);
        LOG_DEBUG("%lu waypoints received.", waypoints_total);
        Disable();
    } else {
        mavlink::Pld_mission_request req;
        Fill_target_ids(req);
        req->seq = waypoint_to_read;
        Send_message(req);
    }
}

void
Mavlink_vehicle::Telemetry::Enable()
{
    telemetry_messages_last = 0;
    telemetry_alive = false;
    telemetry_message_types = 0;
    prev_stats = vehicle.mav_stream->Get_decoder().Get_stats();
    prev_rx_errors_3dr = -1;
    rx_errors_accum = 0;
    link_quality = 0;

    Register_telemetry_handler<mavlink::MESSAGE_ID::SYS_STATUS>(
            &Telemetry::On_sys_status, this, Mavlink_demuxer::COMPONENT_ID_ANY);

    Register_telemetry_handler<mavlink::MESSAGE_ID::GLOBAL_POSITION_INT>(
            &Telemetry::On_global_position_int, this, Mavlink_demuxer::COMPONENT_ID_ANY);

    Register_telemetry_handler<mavlink::MESSAGE_ID::ATTITUDE>(
            &Telemetry::On_attitude, this, Mavlink_demuxer::COMPONENT_ID_ANY);

    Register_telemetry_handler<mavlink::MESSAGE_ID::VFR_HUD>(
            &Telemetry::On_vfr_hud, this, Mavlink_demuxer::COMPONENT_ID_ANY);

    Register_telemetry_handler<mavlink::MESSAGE_ID::GPS_RAW_INT>(
            &Telemetry::On_gps_raw, this, Mavlink_demuxer::COMPONENT_ID_ANY);

    Register_telemetry_handler<mavlink::MESSAGE_ID::RAW_IMU>(
            &Telemetry::On_raw_imu, this, Mavlink_demuxer::COMPONENT_ID_ANY);

    Register_telemetry_handler<mavlink::MESSAGE_ID::SCALED_PRESSURE>(
            &Telemetry::On_scaled_pressure, this, Mavlink_demuxer::COMPONENT_ID_ANY);

    Register_mavlink_handler<mavlink::apm::MESSAGE_ID::RADIO, mavlink::apm::Extension>(
            &Telemetry::On_radio, this, Mavlink_demuxer::COMPONENT_ID_ANY,
            Optional<Mavlink_demuxer::System_id>(Mavlink_demuxer::SYSTEM_ID_ANY));

    watchdog_timer = Timer_processor::Get_instance()->Create_timer(
            config.WATCHDOG_INTERVAL,
            Make_callback(&Mavlink_vehicle::Telemetry::On_telemetry_check, this),
            vehicle.Get_completion_ctx());

    heartbeat_timer = Timer_processor::Get_instance()->Create_timer(
        std::chrono::milliseconds(
            (std::chrono::milliseconds::period::den / config.TELEMETRY_RATE_HZ) *
                HEARTBEAT_RATE_MULTIPLIER),
        Make_callback(&Mavlink_vehicle::Telemetry::On_heartbeat, this),
        vehicle.Get_completion_ctx());

    estimation_timer = Timer_processor::Get_instance()->Create_timer(
        std::chrono::milliseconds(
            (std::chrono::milliseconds::period::den / config.TELEMETRY_RATE_HZ) *
                ESTIMATION_RATE_MULTIPLIER),
        Make_callback(&Mavlink_vehicle::Telemetry::On_estimate_link_quality, this),
        vehicle.Get_completion_ctx());

    /* Causes telemetry to be requested immediately. */
    On_telemetry_check();
}

void
Mavlink_vehicle::Telemetry::On_disable()
{
    if (watchdog_timer) {
        watchdog_timer->Cancel();
        watchdog_timer = nullptr;
    }

    if (heartbeat_timer) {
        heartbeat_timer->Cancel();
        heartbeat_timer = nullptr;
    }

    if (estimation_timer) {
        estimation_timer->Cancel();
        estimation_timer = nullptr;
    }

    Unregister_mavlink_handlers();
}

bool
Mavlink_vehicle::Telemetry::On_telemetry_check()
{
    if (!telemetry_alive) {
        LOG_INFO("Requesting telemetry...");
        for (mavlink::MAV_DATA_STREAM id : {
            mavlink::MAV_DATA_STREAM::MAV_DATA_STREAM_ALL}) {
            mavlink::Pld_request_data_stream msg;
            Fill_target_ids(msg);
            msg->req_stream_id = id;
            msg->req_message_rate = config.TELEMETRY_RATE_HZ;
            msg->start_stop = 1; /* start */
            Send_message(msg);
        }
    }
    telemetry_alive = false;
    return true;
}

bool
Mavlink_vehicle::Telemetry::On_heartbeat()
{
    mavlink::Pld_heartbeat hb;
    hb->custom_mode = 0;
    hb->type = mavlink::MAV_TYPE::MAV_TYPE_GCS;
    hb->autopilot = mavlink::MAV_AUTOPILOT::MAV_AUTOPILOT_INVALID;
    hb->base_mode = mavlink::MAV_MODE::MAV_MODE_PREFLIGHT;
    hb->system_status = mavlink::MAV_STATE::MAV_STATE_UNINIT;

    Send_message(hb);

    return true;
}

bool
Mavlink_vehicle::Telemetry::On_estimate_link_quality()
{
    /* Calculate link quality based on expected rate. */
    double expected_telemetry_rate =
            config.DAMPING_FACTOR * telemetry_message_types * ESTIMATION_RATE_MULTIPLIER;

    double rate_quality = telemetry_messages_last / expected_telemetry_rate;
    if (rate_quality > 1) {
        /* Compensate damping factor. */
        rate_quality = 1;
    }
    telemetry_messages_last = 0;

    double error_quality = 0;

    /* Calculate link quality based on error rate. */
    Mavlink_decoder::Stats new_stats = vehicle.mav_stream->Get_decoder().Get_stats();
    double ok_diff = new_stats.handled - prev_stats.handled;
    double err_diff = (new_stats.bad_checksum + new_stats.unknown_id) -
                      (prev_stats.bad_checksum + prev_stats.unknown_id);
    err_diff += rx_errors_accum;
    rx_errors_accum = 0;
    prev_stats = new_stats;

    if (ok_diff > 0 && err_diff >= 0) {
        error_quality = ok_diff / (ok_diff + err_diff);
    }

    /* Calculate the average between rate and error quality. Rate quality affects
     * the average in inverse ratio of the rate quality itself. I.e. the better
     * rate quality is, the less it affects the average. In turn, error quality
     * affects the average proportionally to rate quality. So when rate is good,
     * then quality depends more on error quality, when rate is bad, the quality
     * is bad too.
     */
    double quality = rate_quality * (1 - rate_quality) + rate_quality * error_quality;

    /* Calculate rolling average. */
    link_quality = link_quality * (1 - QUALITY_RA_QUOT) + (QUALITY_RA_QUOT) * quality;

    return true;
}

void
Mavlink_vehicle::Telemetry::On_sys_status(
        mavlink::Message<mavlink::MESSAGE_ID::SYS_STATUS>::Ptr message)
{
    telemetry_messages_last++;
    telemetry_alive = true;

    auto report = vehicle.Open_telemetry_report();
    report->Set<tm::Battery_voltage>(
        static_cast<double>(message->payload->voltage_battery) / 1000.0);
    if (message->payload->current_battery != -1) {
        report->Set<tm::Battery_current>(
            static_cast<double>(message->payload->current_battery) / 100.0);
    }
    report->Set<tm::Link_quality>(link_quality);
}

void
Mavlink_vehicle::Telemetry::On_global_position_int(
        mavlink::Message<mavlink::MESSAGE_ID::GLOBAL_POSITION_INT>::Ptr message)
{
    telemetry_messages_last++;
    telemetry_alive = true;

    auto report = vehicle.Open_telemetry_report();
    Geodetic_tuple pos(
        static_cast<double>(message->payload->lat) / 1e7 / 180.0 * M_PI,
        static_cast<double>(message->payload->lon) / 1e7 / 180.0 * M_PI,
#if 0
        /* XXX Use relative altitude until UCS is able to ignore bad absolute
         * altitude.
         */
        static_cast<double>(message->payload->relative_alt) / 1000.0);
#endif
        static_cast<double>(message->payload->alt) / 1000.0);
    report->Set<tm::Position>(pos);

    report->Set<tm::Rel_altitude>(
            static_cast<double>(message->payload->relative_alt) / 1000.0);

    if (message->payload->hdg != 0xffff) {
        report->Set<tm::Heading>(
            static_cast<double>(message->payload->hdg) / 100.0);
    }

    double vx = static_cast<double>(message->payload->vx) / 100.0;
    double vy = static_cast<double>(message->payload->vy) / 100.0;
    report->Set<tm::Ground_speed>(sqrt(vx * vx + vy * vy));

    double vz = static_cast<double>(message->payload->vz) / 100.0;
    report->Set<tm::Climb_rate>(vz);
}

void
Mavlink_vehicle::Telemetry::On_attitude(
        mavlink::Message<mavlink::MESSAGE_ID::ATTITUDE>::Ptr message)
{
    telemetry_messages_last++;
    telemetry_alive = true;

    auto report = vehicle.Open_telemetry_report();
    report->Set<tm::Pitch>(message->payload->pitch.Get());
    report->Set<tm::Roll>(message->payload->roll.Get());
    report->Set<tm::Yaw>(message->payload->yaw.Get());
}

void
Mavlink_vehicle::Telemetry::On_vfr_hud(
        mavlink::Message<mavlink::MESSAGE_ID::VFR_HUD>::Ptr)
{
    /* Duplicated in other messages. */
    telemetry_messages_last++;
    telemetry_alive = true;
}

void
Mavlink_vehicle::Telemetry::On_gps_raw(
        mavlink::Message<mavlink::MESSAGE_ID::GPS_RAW_INT>::Ptr message)
{
    telemetry_messages_last++;
    telemetry_alive = true;

    auto report = vehicle.Open_telemetry_report();
    if (message->payload->satellites_visible != 255) {
        report->Set<tm::Gps_satellites_count>(message->payload->satellites_visible.Get());
    }
    switch (message->payload->fix_type) {
    case 0:
    case 1:
        report->Set<tm::Gps_fix_type>(tm::Gps_fix_type_e::NONE);
        break;
    case 2:
        report->Set<tm::Gps_fix_type>(tm::Gps_fix_type_e::_2D);
        break;
    case 3:
        report->Set<tm::Gps_fix_type>(tm::Gps_fix_type_e::_3D);
        break;
    }
}

void
Mavlink_vehicle::Telemetry::On_raw_imu(
    mavlink::Message<mavlink::MESSAGE_ID::RAW_IMU>::Ptr)
{
    /* Unused. */
    telemetry_messages_last++;
    telemetry_alive = true;
}

void
Mavlink_vehicle::Telemetry::On_scaled_pressure(
    mavlink::Message<mavlink::MESSAGE_ID::SCALED_PRESSURE>::Ptr)
{
    /* Unused. */
    telemetry_messages_last++;
    telemetry_alive = true;
}

void
Mavlink_vehicle::Telemetry::On_radio(
        mavlink::Message<mavlink::apm::MESSAGE_ID::RADIO,
                         mavlink::apm::Extension>::Ptr msg)
{
    if (prev_rx_errors_3dr < 0) {
        /* Initial value. */
        prev_rx_errors_3dr = msg->payload->rxerrors;
        return;
    }

    if (prev_rx_errors_3dr > msg->payload->rxerrors) {
        /* Mavlink counter wrapped. */
        LOG_DEBUG("3DR RADIO rxerrors counter wrapped (%u->%u)",
                prev_rx_errors_3dr, msg->payload->rxerrors.Get());
        rx_errors_accum += msg->payload->rxerrors;
    } else {
        rx_errors_accum += msg->payload->rxerrors - prev_rx_errors_3dr;
    }
    prev_rx_errors_3dr = msg->payload->rxerrors;
}

bool
Mavlink_vehicle::Clear_all_missions::Try()
{
    if (!remaining_attempts--) {
        LOG_WARN("Clear_all_missions all attempts failed.");
        Call_next_action(false);
        return false;
    }

    mavlink::Pld_mission_clear_all mca;
    Fill_target_ids(mca);
    Send_message(mca);
    Schedule_timer();
    return false;
}

void
Mavlink_vehicle::Clear_all_missions::On_mission_ack(
        mavlink::Message<mavlink::MESSAGE_ID::MISSION_ACK>::Ptr message)
{
    if (message->payload->target_component != VSM_COMPONENT_ID ||
        message->payload->target_system != VSM_SYSTEM_ID) {
        /* Not to us, ignore. */
        return;
    }

    if (message->payload->type != mavlink::MAV_MISSION_RESULT::MAV_MISSION_ACCEPTED) {
        /* Wrong code, retry? */
        LOG_WARN("Mission clear all ACK bad result %d",
                message->payload->type.Get());
        return;
    }
    LOG_INFO("All missions are cleared successfully.");
    Call_next_action(true);
}

void
Mavlink_vehicle::Clear_all_missions::Enable(
        const vsm::Clear_all_missions& info)
{
    this->info = info;
    remaining_attempts = ATTEMPTS;

    Register_mavlink_handler<mavlink::MESSAGE_ID::MISSION_ACK>(
            &Clear_all_missions::On_mission_ack,
            this,
            Mavlink_demuxer::COMPONENT_ID_ANY);

    Try();
}

void
Mavlink_vehicle::Clear_all_missions::On_disable()
{
    Unregister_mavlink_handlers();

    if (timer) {
        timer->Cancel();
        timer = nullptr;
    }
}

void
Mavlink_vehicle::Clear_all_missions::Schedule_timer()
{
    if (timer) {
        timer->Cancel();
    }
    timer = Timer_processor::Get_instance()->Create_timer(
                std::chrono::seconds(RETRY_TIMEOUT),
                Make_callback(&Clear_all_missions::Try, this),
                vehicle.Get_completion_ctx());
}

void
Mavlink_vehicle::Mission_upload::Enable()
{
    attempts_action_left = ATTEMPTS_ACTION;
    attempts_total_left = ATTEMPTS_TOTAL;
    current_action = -1; /* Nothing requested yet. */

    Register_mavlink_handler<mavlink::MESSAGE_ID::MISSION_REQUEST>(
            &Mission_upload::On_mission_request,
            this,
            Mavlink_demuxer::COMPONENT_ID_ANY);

    Try();
}

void
Mavlink_vehicle::Mission_upload::On_disable()
{
    Unregister_mavlink_handlers();

    if (timer) {
        timer->Cancel();
        timer = nullptr;
    }
}

bool
Mavlink_vehicle::Mission_upload::Try()
{
    if (!Next_attempt()) {
        LOG_WARN("All task upload attempts failed.");
        Call_next_action(false);
        return false;
    }

    if (current_action == -1) {
        /* Initiate upload procedure. */
        mavlink::Pld_mission_count mc;
        mc->count = mission_items.size();
        Fill_target_ids(mc);
        Send_message(mc);
    } else {
        Upload_current_action();
    }
    Schedule_timer();
    return false;
}

void
Mavlink_vehicle::Mission_upload::On_mission_ack(
        mavlink::Message<mavlink::MESSAGE_ID::MISSION_ACK>::Ptr message)
{
    LOG_INFO("MISSION ACK: %s", message->payload.Dump().c_str());
    if (message->payload->target_component != VSM_COMPONENT_ID ||
        message->payload->target_system != VSM_SYSTEM_ID) {
        /* Not for us, ignore. */
        return;
    }
    ASSERT(current_action + 1 ==
           static_cast<ssize_t>(mission_items.size()));
    if (message->payload->type ==
        mavlink::MAV_MISSION_RESULT::MAV_MISSION_ACCEPTED) {
        Call_next_action(true);
    }
}

void
Mavlink_vehicle::Mission_upload::On_mission_request(
        mavlink::Message<mavlink::MESSAGE_ID::MISSION_REQUEST>::Ptr message)
{
    /* There is a bug in APM which always sends target system and component
     * as 0 in mission request. So disable the check.
     */
#ifdef _apm_bug_mission_request_fixed_
    if (message->payload->target_component != DEFAULT_MAVLINK_COMPONENT_ID ||
        message->payload->target_system != DEFAULT_VSM_SYSTEM_ID) {
        /* Not for us, ignore. */
        return;
    }
#endif

    if (current_action == -1 && message->payload->seq) {
        /* Wrong sequence. First is expected always. */
        return;
    }

    if (message->payload->seq >= mission_items.size()) {
        /* Wrong sequence, out of bounds. */
        return;
    } else if (current_action == message->payload->seq) {
        /* Request for the same item. */
        if (!Next_attempt()) {
            LOG_INFO("No more attempts left for the same mission item.");
            Call_next_action(false);
            return;
        }
    } else if (current_action + 1 != message->payload->seq) {
        /* Wrong sequence. */
        return;
    } else {
        attempts_action_left = ATTEMPTS_ACTION;
        current_action++;
    }

    if (current_action + 1 ==
        static_cast<ssize_t>(mission_items.size())) {
        /* Last action requested, ack is awaited to finish the sequence. */
        Register_mavlink_handler<mavlink::MESSAGE_ID::MISSION_ACK>(
                    &Mission_upload::On_mission_ack,
                    this,
                    Mavlink_demuxer::COMPONENT_ID_ANY);
    }

    Upload_current_action();
    Schedule_timer();
}

void
Mavlink_vehicle::Mission_upload::Schedule_timer()
{
    if (timer) {
        timer->Cancel();
    }
    timer = Timer_processor::Get_instance()->Create_timer(
            std::chrono::seconds(RETRY_TIMEOUT),
            Make_callback(&Mission_upload::Try, this),
            vehicle.Get_completion_ctx());
}

bool
Mavlink_vehicle::Mission_upload::Next_attempt()
{
    return attempts_action_left-- && attempts_total_left--;
}

void
Mavlink_vehicle::Mission_upload::Upload_current_action()
{
    ASSERT(current_action < static_cast<ssize_t>(mission_items.size()));
    mavlink::Payload_base& msg = *mission_items[current_action];
    LOG_INFO("Uploading [%s][seq %ld] to [%s:%s]",
            msg.Get_name(),
            current_action,
            vehicle.Get_model_name().c_str(),
            vehicle.Get_serial_number().c_str());
    Send_message(msg);
}
