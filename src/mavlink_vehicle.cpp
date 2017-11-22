// Copyright (c) 2017, Smart Projects Holdings Ltd
// All rights reserved.
// See LICENSE file for license details.

#include <mavlink_vehicle.h>
#include <fstream>

constexpr std::chrono::seconds Mavlink_vehicle::WRITE_TIMEOUT;
constexpr int Mavlink_vehicle::Telemetry::ESTIMATION_RATE_MULTIPLIER;

namespace {
static const std::vector<std::string> SENSOR_NAMES = {
    "GYRO",
    "ACCEL",
    "MAG",
    "ABS_PRESSURE",
    "DIFF_PRESSURE",
    "GPS",
    "OPT_FLOW",
    "VISION_POS",
    "LASER_POS",
    "EXT_GND_TRUTH",
    "ANG_RATE_CTRL",
    "ATTI_STAB",
    "YAW_POS",
    "Z_ALT_CTRL",
    "XY_POS_CTRL",
    "MOTOR_OUTPUT",
    "RC_RECEIVER",
    "GYRO2",
    "ACCEL2",
    "MAG2",
    "GEOFENCE",
    "AHRS",
    "TERRAIN"
};
}

using namespace ugcs::vsm;

void
Mavlink_vehicle::On_enable()
{
    mav_stream->Bind_decoder_demuxer();
    Schedule_next_read();
    Wait_for_vehicle();
}

void
Mavlink_vehicle::On_disable()
{
    Disable_activities();

    read_op.Abort();

    mav_stream->Disable();
    mav_stream = nullptr;
}

ugcs::vsm::mavlink::MAV_TYPE
Mavlink_vehicle::Get_mav_type() const
{
    return type;
}

void
Mavlink_vehicle::Wait_for_vehicle()
{
    heartbeat.Enable();
    statistics.Enable();
}

std::string
Mavlink_vehicle::Get_failed_sensor_report()
{
    std::string ret;
    for (size_t i = 0; i < SENSOR_NAMES.size(); i++) {
        size_t bit = (1 << i);
        if (current_sensors_present & bit) {
            if ((current_sensor_health & bit)  == 0) {
                if (ret.empty()) {
                    ret = "Sensor FAILED: ";
                } else {
                    ret += ", ";
                }
                ret += SENSOR_NAMES[i];
            }
        }
    }
    return ret;
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
    size_t max_read;
    if (mav_stream->Get_stream()->Get_type() == Io_stream::Type::UDP) {
        max_read = ugcs::vsm::MIN_UDP_PAYLOAD_SIZE_TO_READ;
    } else {
        max_read = to_read;
    }
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
Mavlink_vehicle::Process_heartbeat(
        ugcs::vsm::mavlink::Message<ugcs::vsm::mavlink::MESSAGE_ID::HEARTBEAT>::Ptr)
{
    t_is_armed->Set_value_na();
    t_control_mode->Set_value_na();
    t_uplink_present->Set_value(true);
    t_downlink_present->Set_value(true);
    Commit_to_ucs();
}

void
Mavlink_vehicle::Send_message(mavlink::Payload_base& payload)
{
    mav_stream->Send_message(
            payload,
            VSM_SYSTEM_ID,
            VSM_COMPONENT_ID,
            Mavlink_vehicle::WRITE_TIMEOUT,
            Make_timeout_callback(
                    &Mavlink_vehicle::Write_to_vehicle_timed_out,
                    this,
                    mav_stream),
            Get_completion_ctx());
}

void
Mavlink_vehicle::Set_message_interval(
        mavlink::MESSAGE_ID_TYPE id,
        int interval)
{
    mavlink::Pld_command_long msg;
    msg->target_system = real_system_id;
    msg->target_component = real_component_id;
    msg->command = mavlink::MAV_CMD_SET_MESSAGE_INTERVAL;
    msg->param1 = id;
    msg->param2 = interval;
    Send_message(msg);
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
    VEHICLE_LOG_DBG((*this), "Write timeout on [%s] detected.", server_info.c_str());
    waiter->Abort();
}

bool
Mavlink_vehicle::Is_mission_upload_active()
{
    return (mission_upload.Is_active());
}

void
Mavlink_vehicle::Update_boot_time(std::chrono::milliseconds boot_duration)
{
    // Only if there is nonzero boot time because
    // some vehicles (e.g. vehicle-emulator) do not send boot time.
    if (boot_duration > boot_duration.zero()) {
        auto current_vehicle_boot_time = std::chrono::steady_clock::now() - boot_duration;

        if (last_known_vehicle_boot_time_known) {
            if (report_relative_altitude) {
                if (current_vehicle_boot_time - last_known_vehicle_boot_time > ALTITUDE_ORIGIN_RESET_TRESHOLD) {
                    VEHICLE_LOG_INF((*this), "Vehicle rebooted %" PRId64 " s ago, resetting altitude origin...",
                            std::chrono::duration_cast<std::chrono::seconds>(boot_duration).count());
                    Reset_altitude_origin();
                }
            }
        } else {
            // First time this vsm sees the vehicle.
            VEHICLE_LOG_INF((*this), "Vehicle booted %" PRId64 " s ago.",
                    std::chrono::duration_cast<std::chrono::seconds>(boot_duration).count());
            if (report_relative_altitude) {
                if (boot_duration > ALTITUDE_ORIGIN_RESET_TRESHOLD) {
                    // Do not reset altitude origin if vehicle has been working
                    // already for more than 15 seconds.
                    VEHICLE_LOG_INF((*this), "Not resetting altitude origin on connect");
                } else {
                    VEHICLE_LOG_INF((*this), "Resetting altitude origin on connect");
                    Reset_altitude_origin();
                }
            }
            last_known_vehicle_boot_time_known = true;
        }

        // save the boot time on each packet to handle vehicle clock drift against system clock.
        last_known_vehicle_boot_time = current_vehicle_boot_time;
    }
}

const std::string
Mavlink_vehicle::Mav_result_to_string(int r)
{
    switch (static_cast<mavlink::MAV_RESULT>(r)) {
    case mavlink::MAV_RESULT_ACCEPTED:
        return "Success";
    case mavlink::MAV_RESULT_DENIED:
        return "Command denied";
    case mavlink::MAV_RESULT_FAILED:
        return "Command failed";
    case mavlink::MAV_RESULT_UNSUPPORTED:
        return "Command unsupported";
    case mavlink::MAV_RESULT_TEMPORARILY_REJECTED:
        return "Command temporarily rejected";
    }
    return "UNDEFINED";
}

const std::string
Mavlink_vehicle::Mav_mission_result_to_string(int r)
{
    switch (static_cast<mavlink::MAV_MISSION_RESULT>(r)) {
    case mavlink::MAV_MISSION_ACCEPTED:
        return "Success";
    case mavlink::MAV_MISSION_DENIED:
        return "Route denied";
    case mavlink::MAV_MISSION_ERROR:
        return "Route error";
    case mavlink::MAV_MISSION_INVALID:
        return "Route invalid";
    case mavlink::MAV_MISSION_INVALID_PARAM1:
        return "Invalid param1";
    case mavlink::MAV_MISSION_INVALID_PARAM2:
        return "Invalid param2";
    case mavlink::MAV_MISSION_INVALID_PARAM3:
        return "Invalid param3";
    case mavlink::MAV_MISSION_INVALID_PARAM4:
        return "Invalid param4";
    case mavlink::MAV_MISSION_INVALID_PARAM5_X:
        return "Invalid param5";
    case mavlink::MAV_MISSION_INVALID_PARAM6_Y:
        return "Invalid param6";
    case mavlink::MAV_MISSION_INVALID_PARAM7:
        return "Invalid param7";
    case mavlink::MAV_MISSION_INVALID_SEQUENCE:
        return "Invalid sequence";
    case mavlink::MAV_MISSION_NO_SPACE:
        return "Route too big";
    case mavlink::MAV_MISSION_UNSUPPORTED:
        return "Route unsupported";
    case mavlink::MAV_MISSION_UNSUPPORTED_FRAME:
        return "Invalid frame";
    }
    return "UNDEFINED";
}

bool
Mavlink_vehicle::Verify_parameter(const std::string&, float, ugcs::vsm::mavlink::MAV_PARAM_TYPE&)
{
    return false;
}

void
Mavlink_vehicle::Set_parameters_from_properties(const std::string& prefix)
{
    auto pr = ugcs::vsm::Properties::Get_instance();
    Write_parameters::List param_list;
    for (auto prop = pr->begin(prefix); prop != pr->end(); prop++) {
        try {
            if (prop.Get_count() == 4) {
                std::string name = prop[3];
                float value = pr->Get_float((*prop));
                mavlink::MAV_PARAM_TYPE type;
                if (Verify_parameter(name, value, type)) {
                    mavlink::Pld_param_set param;
                    param->target_system = real_system_id;
                    param->target_component = real_component_id;
                    param->param_id = name;
                    param->param_type = type;
                    param->param_value = value;
                    param_list.emplace_back(param);
                } else {
                    VEHICLE_LOG_ERR((*this), "Invalid parameter/value specified (%s %f)", name.c_str(), value);
                }
            }
        } catch (...) {
            continue;
        }
    }

    if (param_list.size()) {
        write_parameters.Enable(param_list);
    }
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
Mavlink_vehicle::Activity::Call_next_action(bool success, const std::string& msg)
{
    auto action = std::move(next_action);
    Disable();
    if (action) {
        action(success, msg);
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

    // Ignore heartbeats of type GCS. Sent by Ardupilot 3.3.1 SITL.
    if (message->payload->type == mavlink::MAV_TYPE_GCS) {
        return;
    }

    vehicle.base_mode = message->payload->base_mode.Get();

    int system_status = message->payload->system_status.Get();
    switch (system_status) {
        case mavlink::MAV_STATE_STANDBY:
            vehicle.t_autopilot_status->Set_value(proto::AUTOPILOT_STATUS_STANDBY);
            break;
        case mavlink::MAV_STATE_ACTIVE:
            vehicle.t_autopilot_status->Set_value(proto::AUTOPILOT_STATUS_ACTIVE);
            break;
        case mavlink::MAV_STATE_CRITICAL:
            vehicle.t_autopilot_status->Set_value(proto::AUTOPILOT_STATUS_CRITICAL);
            if (vehicle.current_autopilot_status != system_status) {
                vehicle.Add_status_message("Attention! Autopilot is in CRITICAL state!");
            }
            break;
        case mavlink::MAV_STATE_EMERGENCY:
            vehicle.t_autopilot_status->Set_value(proto::AUTOPILOT_STATUS_EMERGENCY);
            if (vehicle.current_autopilot_status != system_status) {
                vehicle.Add_status_message("Attention! Autopilot is in EMERGENCY state!");
            }
            break;
        default:
            vehicle.t_autopilot_status->Set_value_na();
            break;
    }

    vehicle.current_autopilot_status = system_status;

    vehicle.Process_heartbeat(message);

    if (!first_ok_received) {
        first_ok_received = true;
        VEHICLE_LOG_INF(vehicle, "First heartbeat received.");
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
            VEHICLE_LOG_INF(vehicle, "Heartbeat lost. Still waiting...");
            vehicle.Disable_activities();
            vehicle.Wait_for_vehicle();
        } else {
            VEHICLE_LOG_INF(vehicle, "Heartbeat lost. Vehicle disconnected.");
            /* Will trigger disconnect handler. */
            vehicle.mav_stream->Get_stream()->Close();
        }
    }
    received_count = 0;
    return true;
}

void
Mavlink_vehicle::Statistics::Enable()
{
    auto timer_proc = Timer_processor::Get_instance();
    timer = timer_proc->Create_timer(std::chrono::seconds(COLLECTION_INTERVAL),
            Make_callback(&Mavlink_vehicle::Statistics::On_timer, this),
            vehicle.Get_completion_ctx());

    Register_mavlink_handler<mavlink::MESSAGE_ID::STATUSTEXT>(
            &Statistics::On_status_text,
            this,
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
    VEHICLE_LOG_DBG(vehicle, "%lld Mavlink messages processed, link quality %.1f%%",
        static_cast<long long>(stats.handled - num_of_processed_messages),
        vehicle.telemetry.link_quality * 100);
    if (stats.bad_checksum != num_of_csum_errors) {
        VEHICLE_LOG_DBG(vehicle, "%lld Mavlink csum errors",
            static_cast<long long>(stats.bad_checksum - num_of_csum_errors));
        num_of_csum_errors = stats.bad_checksum;
    }
    num_of_processed_messages = stats.handled;
    return true;
}

void
Mavlink_vehicle::Statistics::On_status_text(
        mavlink::Message<mavlink::MESSAGE_ID::STATUSTEXT>::Ptr message)
{
    VEHICLE_LOG_INF(vehicle, "STATUS_TEXT: %s", message->payload->text.Get_string().c_str());
    vehicle.Add_status_message(message->payload->text.Get_string());
    vehicle.Commit_to_ucs();
    if(statustext_handler) {
        statustext_handler(message);
    }
}

void
Mavlink_vehicle::Read_parameters::Enable(std::unordered_set<std::string> names)
{
    attempts_left = try_count;
    param_names = names;
    Register_mavlink_handler<mavlink::MESSAGE_ID::PARAM_VALUE>(
        &Read_parameters::On_param_value, this, Mavlink_demuxer::COMPONENT_ID_ANY);
    Try();
}

bool
Mavlink_vehicle::Read_parameters::Is_enabled()
{
    return timer != nullptr;
}

void
Mavlink_vehicle::Read_parameters::On_disable()
{
    if (timer) {
        timer->Cancel();
        timer = nullptr;
    }
    param_names.clear();
    Unregister_mavlink_handlers();
}

bool
Mavlink_vehicle::Read_parameters::Try()
{
    if (!attempts_left--) {
        Disable();
        return false;
    }
    if (param_names.empty()) {
        mavlink::Pld_param_request_list msg;
        Fill_target_ids(msg);
        Send_message(msg);
    } else {
        // Read single parameter
        mavlink::Pld_param_request_read msg;
        Fill_target_ids(msg);
        msg->param_index = -1;
        for (auto& n : param_names) {
            msg->param_id = n;
            Send_message(msg);
        }
    }
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
        retry_timeout,
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
        VEHICLE_LOG_WRN(vehicle, "PARAM [%s:%d] = Unsupported format!",
                            message->payload->param_id.Get_string().c_str(),
                            message->payload->param_index.Get());
    }

    if (parameter_handler) {
        parameter_handler(message->payload);
    }

    if (param_names.empty()) {
        // all parameters.
        if (message->payload->param_count - 1 == message->payload->param_index) {
            Disable();
        } else {
            Schedule_timer();
        }
    } else {
        // single parameters
        param_names.erase(message->payload->param_id.Get_string());
        if (param_names.empty()) {
            // no more parameters;
            Call_next_action(true);
            Disable();
        } else {
            // wait for next one.
            Schedule_timer();
        }
    }
}

void
Mavlink_vehicle::Read_string_parameters::Enable(std::unordered_set<std::string> names)
{
    attempts_left = try_count;
    param_names = std::move(names);

    Register_mavlink_handler<mavlink::sph::MESSAGE_ID::PARAM_STR_VALUE,mavlink::sph::Extension>(
            &Read_string_parameters::On_param_value, this, Mavlink_demuxer::COMPONENT_ID_ANY);
    Try();
}

void
Mavlink_vehicle::Read_string_parameters::On_disable()
{
    if (timer) {
        timer->Cancel();
        timer = nullptr;
    }
    param_names.clear();
    Unregister_mavlink_handlers();
}

bool
Mavlink_vehicle::Read_string_parameters::Try()
{
    if (attempts_left--) {
        mavlink::sph::Pld_param_str_request_read msg;
        Fill_target_ids(msg);
        msg->param_index = -1;
        for (auto& n : param_names) {
            msg->param_id = n;
            Send_message(msg);
        }
        Schedule_timer();
    } else {
        Disable();
    }
    return false;
}

void
Mavlink_vehicle::Read_string_parameters::Schedule_timer()
{
    if (timer) {
        timer->Cancel();
    }
    timer = Timer_processor::Get_instance()->Create_timer(
        retry_timeout,
        Make_callback(&Read_string_parameters::Try, this),
        vehicle.Get_completion_ctx());
}

void
Mavlink_vehicle::Read_string_parameters::On_param_value(
        mavlink::Message<mavlink::sph::MESSAGE_ID::PARAM_STR_VALUE,
        ugcs::vsm::mavlink::sph::Extension>::Ptr message)
{
    LOG_INFO("PARAM [%s (%d)] = %s",
            message->payload->param_id.Get_string().c_str(),
            message->payload->param_index.Get(),
            message->payload->param_value.Get_string().c_str());

    if (parameter_handler) {
        parameter_handler(message->payload);
    }

    if (param_names.empty()) {
        Disable();
    } else {
        // single parameters
        param_names.erase(message->payload->param_id.Get_string());
        if (param_names.empty()) {
            // no more parameters;
            Disable();
        } else {
            // wait for next one.
            Schedule_timer();
        }
    }
}

void
Mavlink_vehicle::Read_version::Enable()
{
    attempts_left = ATTEMPTS;
    Register_mavlink_handler<mavlink::MESSAGE_ID::AUTOPILOT_VERSION>(
            &Read_version::On_version, this, Mavlink_demuxer::COMPONENT_ID_ANY);
    Try();
}

void
Mavlink_vehicle::Read_version::On_disable()
{
    if (timer) {
        timer->Cancel();
        timer = nullptr;
    }
    Unregister_mavlink_handlers();
}

bool
Mavlink_vehicle::Read_version::Try()
{
    if (!attempts_left--) {
        Disable();
        return false;
    }
    mavlink::apm::Pld_autopilot_version_request msg;
    Fill_target_ids(msg);
    Send_message(msg);
    Schedule_timer();
    return false;
}

void
Mavlink_vehicle::Read_version::Schedule_timer()
{
    if (timer) {
        timer->Cancel();
    }
    timer = Timer_processor::Get_instance()->Create_timer(
            std::chrono::seconds(RETRY_TIMEOUT),
            Make_callback(&Read_version::Try, this),
            vehicle.Get_completion_ctx());
}

void
Mavlink_vehicle::Read_version::On_version(
        mavlink::Message<mavlink::MESSAGE_ID::AUTOPILOT_VERSION>::Ptr message)
{
    if (version_handler) {
        version_handler(message->payload);
    }
    Disable();
}

void
Mavlink_vehicle::Write_parameters::Enable(const List& parameters_to_write)
{
    Register_mavlink_handler<mavlink::MESSAGE_ID::PARAM_VALUE>(
        &Write_parameters::On_param_value, this, Mavlink_demuxer::COMPONENT_ID_ANY);
    parameters = parameters_to_write;
    attempts_left = try_count;
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
        /* Not the parameter we requested.
         * Don't reschedule immediately, wait for retry timeout.
         */
        return;
    }

    if (message->payload->param_value.Get() !=
        parameters.back()->param_value.Get()) {
        VEHICLE_LOG_WRN(vehicle, "Parameter writing validation failed for [%s]. Value [%f] "
                "expected, but [%f] received.",
                parameters.back()->param_id.Get_string().c_str(),
                parameters.back()->param_value.Get(),
                message->payload->param_value.Get());
        Try();
        return;
    }
    VEHICLE_LOG_DBG(vehicle, "Parameter [%s:%f] write verified successfully.",
            message->payload->param_id.Get_string().c_str(),
            message->payload->param_value.Get());
    /* Written OK. */
    parameters.pop_back();
    attempts_left = try_count;
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
        Call_next_action(false, "All parameter write attempts failed.");
        return false;
    }
    VEHICLE_LOG_INF(vehicle, "Writing parameter [%s:%f].",
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
        retry_timeout,
        Make_callback(&Write_parameters::Try, this),
        vehicle.Get_completion_ctx());
}

void
Mavlink_vehicle::Do_command_long::Enable(const List& commands_list)
{
    Register_mavlink_handler<mavlink::MESSAGE_ID::COMMAND_ACK>(
        &Do_command_long::On_command_ack, this, Mavlink_demuxer::COMPONENT_ID_ANY);
    commands = commands_list;
    attempts_left = try_count;
    Try();
}

void
Mavlink_vehicle::Do_command_long::On_disable()
{
    if (timer) {
        timer->Cancel();
        timer = nullptr;
    }
    Unregister_mavlink_handlers();
    commands.clear();
}

void
Mavlink_vehicle::Do_command_long::On_command_ack(
        mavlink::Message<mavlink::MESSAGE_ID::COMMAND_ACK>::Ptr message)
{
    VEHICLE_LOG_DBG(vehicle, "COMMAND_ACK for command %d, res=%d",
            message->payload->command.Get(), message->payload->result.Get());

    auto cmd = commands.front();
    if (message->payload->command.Get() == cmd->command.Get()) {
        // This is a response to our command.
        commands.pop_back();
        if (message->payload->result == mavlink::MAV_RESULT::MAV_RESULT_ACCEPTED) {
            attempts_left = try_count;
            Try();
        } else {
            auto p = message->payload->result.Get();
            Call_next_action(false, "Result: " + std::to_string(p) + " (" + Mav_result_to_string(p).c_str() + ")");
            // command failed. return failure to ucs.
            commands.clear();
            Disable();
        }
    }
}

bool
Mavlink_vehicle::Do_command_long::Try()
{
    if (commands.empty()) {
        Call_next_action(true);
        return false;
    }

    if (!attempts_left--) {
        Call_next_action(false, "All command attempts failed.");
        return false;
    }
    VEHICLE_LOG_INF(vehicle, "Sending command %s",
        commands.back().Get_name());
    Send_message(commands.back());
    Schedule_timer();
    return false;
}

void
Mavlink_vehicle::Do_command_long::Schedule_timer()
{
    if (timer) {
        timer->Cancel();
    }
    timer = Timer_processor::Get_instance()->Create_timer(
        retry_timeout,
        Make_callback(&Do_command_long::Try, this),
        vehicle.Get_completion_ctx());
}

void
Mavlink_vehicle::Read_waypoints::Enable()
{
    if (timer) {
        return; // in progress.
    }
    Register_mavlink_handler<mavlink::MESSAGE_ID::MISSION_COUNT>(
            &Read_waypoints::On_count,
            this,
            Mavlink_demuxer::COMPONENT_ID_ANY);
    retries = 3;
    Get_count();
}

void
Mavlink_vehicle::Read_waypoints::Get_home_location()
{
    if (timer) {
        return; // in progress.
    }
    items_total = 1;
    item_to_read = 0;
    Register_mavlink_handler<mavlink::MESSAGE_ID::MISSION_ITEM>(
        &Read_waypoints::On_item,
        this,
        Mavlink_demuxer::COMPONENT_ID_ANY);
    retries = 1;
    Get_next_item();
}

void
Mavlink_vehicle::Read_waypoints::On_disable()
{
    Cancel_timer();
    Unregister_mavlink_handlers();
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

    items_total = message->payload->count;
    item_to_read = 0;

    VEHICLE_LOG_DBG(vehicle, "Reading of %zu waypoints...", items_total);

    Register_mavlink_handler<mavlink::MESSAGE_ID::MISSION_ITEM>(
        &Read_waypoints::On_item,
        this,
        Mavlink_demuxer::COMPONENT_ID_ANY);

    Register_mavlink_handler<mavlink::MESSAGE_ID::MISSION_ACK>(
        &Read_waypoints::On_mission_ack,
        this,
        Mavlink_demuxer::COMPONENT_ID_ANY);

    retries = 3;
    Get_next_item();
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
    if (message->payload->seq == item_to_read) {
        item_to_read++;
        if (item_handler) {
            item_handler(message->payload);
        }
    }
    retries = 3;
    Get_next_item();
}

void
Mavlink_vehicle::Read_waypoints::On_mission_ack(
        mavlink::Message<mavlink::MESSAGE_ID::MISSION_ACK>::Ptr message)
{
    if (message->payload->target_component != VSM_COMPONENT_ID ||
        message->payload->target_system != VSM_SYSTEM_ID) {
        /* Not to us, ignore. */
        return;
    }
    Cancel_timer();
    Call_next_action(false,
        std::string("MISSION_ACK: ")
        + std::to_string(message->payload->type)
        + " ("
        + Mav_mission_result_to_string(message->payload->type)
        + ")");
}

bool
Mavlink_vehicle::Read_waypoints::Get_count()
{
    Cancel_timer();
    if (retries) {
        retries--;
        mavlink::Pld_mission_request_list req;
        Fill_target_ids(req);
        Send_message(req);
        timer = Timer_processor::Get_instance()->Create_timer(
            std::chrono::seconds(1),
            Make_callback(&Read_waypoints::Get_count, this),
            vehicle.Get_completion_ctx());
    } else {
        Call_next_action(false, "Timeout while getting item count");
    }
    return false;
}

bool
Mavlink_vehicle::Read_waypoints::Get_next_item()
{
    Cancel_timer();
    if (items_total == item_to_read) {
        mavlink::Pld_mission_ack ack;
        Fill_target_ids(ack);
        ack->type = mavlink::MAV_MISSION_RESULT::MAV_MISSION_ACCEPTED;
        Send_message(ack);
        Call_next_action(true);
    } else {
        if (retries) {
            retries--;
            mavlink::Pld_mission_request req;
            Fill_target_ids(req);
            req->seq = item_to_read;
            Send_message(req);
            timer = Timer_processor::Get_instance()->Create_timer(
                        std::chrono::seconds(1),
                        Make_callback(&Read_waypoints::Get_next_item, this),
                        vehicle.Get_completion_ctx());
        } else {
            Call_next_action(false, "Timeout while getting mission item");
        }
    }
    return false;
}

bool
Mavlink_vehicle::Read_waypoints::In_progress()
{
    return timer != nullptr;
}

void
Mavlink_vehicle::Read_waypoints::Cancel_timer()
{
    if (timer) {
        timer->Cancel();
        timer = nullptr;
    }
}

void
Mavlink_vehicle::Telemetry::Enable()
{
    telemetry_messages_last = 0;
    telemetry_alive = false;
    prev_stats = vehicle.mav_stream->Get_decoder().Get_stats();
    prev_rx_errors_3dr = -1;
    rx_errors_accum = 0;
    link_quality = 0.5;

    Register_telemetry_handler<mavlink::MESSAGE_ID::SYS_STATUS>(
            &Telemetry::On_sys_status, this, Mavlink_demuxer::COMPONENT_ID_ANY);

    Register_telemetry_handler<mavlink::MESSAGE_ID::GLOBAL_POSITION_INT>(
            &Telemetry::On_global_position_int, this, Mavlink_demuxer::COMPONENT_ID_ANY);

    Register_telemetry_handler<mavlink::MESSAGE_ID::ATTITUDE>(
            &Telemetry::On_attitude, this, Mavlink_demuxer::COMPONENT_ID_ANY);

    Register_telemetry_handler<mavlink::MESSAGE_ID::MISSION_CURRENT>(
            &Telemetry::On_mission_current, this, Mavlink_demuxer::COMPONENT_ID_ANY);

    Register_telemetry_handler<mavlink::MESSAGE_ID::VFR_HUD>(
            &Telemetry::On_vfr_hud, this, Mavlink_demuxer::COMPONENT_ID_ANY);

    Register_telemetry_handler<mavlink::MESSAGE_ID::GPS_RAW_INT>(
            &Telemetry::On_gps_raw, this, Mavlink_demuxer::COMPONENT_ID_ANY);

    Register_mavlink_handler<mavlink::apm::MESSAGE_ID::RADIO, mavlink::apm::Extension>(
            &Telemetry::On_radio, this, Mavlink_demuxer::COMPONENT_ID_ANY,
            Optional<Mavlink_demuxer::System_id>(Mavlink_demuxer::SYSTEM_ID_ANY));

    Register_telemetry_handler<mavlink::MESSAGE_ID::ALTITUDE>(
            &Telemetry::On_altitude, this, Mavlink_demuxer::COMPONENT_ID_ANY);

    Register_telemetry_handler<mavlink::MESSAGE_ID::SERVO_OUTPUT_RAW>(
            &Telemetry::On_servo_output_raw, this, Mavlink_demuxer::COMPONENT_ID_ANY);

    Register_telemetry_handler<mavlink::MESSAGE_ID::POSITION_TARGET_GLOBAL_INT>(
            &Telemetry::On_target_position, this, Mavlink_demuxer::COMPONENT_ID_ANY);

    watchdog_timer = Timer_processor::Get_instance()->Create_timer(
            config.WATCHDOG_INTERVAL,
            Make_callback(&Mavlink_vehicle::Telemetry::On_telemetry_check, this),
            vehicle.Get_completion_ctx());

    heartbeat_timer = Timer_processor::Get_instance()->Create_timer(
        std::chrono::milliseconds(
            (std::chrono::milliseconds::period::den / vehicle.telemetry_rate_hz) *
                HEARTBEAT_RATE_MULTIPLIER),
        Make_callback(&Mavlink_vehicle::Telemetry::On_heartbeat, this),
        vehicle.Get_completion_ctx());

    estimation_timer = Timer_processor::Get_instance()->Create_timer(
        std::chrono::seconds(ESTIMATION_RATE_MULTIPLIER),
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

void
Mavlink_vehicle::Initialize_telemetry()
{
    for (mavlink::MAV_DATA_STREAM id : {
        mavlink::MAV_DATA_STREAM::MAV_DATA_STREAM_EXTRA1,   // attitude
        mavlink::MAV_DATA_STREAM::MAV_DATA_STREAM_EXTRA2,   // vfr_hud
        mavlink::MAV_DATA_STREAM::MAV_DATA_STREAM_POSITION, // gps
        mavlink::MAV_DATA_STREAM::MAV_DATA_STREAM_EXTENDED_STATUS, // gps fix & sysstatus
        mavlink::MAV_DATA_STREAM::MAV_DATA_STREAM_RC_CHANNELS  // output pwm
        }) {
        mavlink::Pld_request_data_stream msg;
        msg->target_system = real_system_id;
        msg->target_component = real_component_id;
        msg->req_stream_id = id;
        msg->req_message_rate = telemetry_rate_hz;
        msg->start_stop = 1; /* start */
        Send_message(msg);
        Send_message(msg);
    }
    // We are counting 6 messages as telemetry:
    // SYS_STATUS, GLOBAL_POSITION_INT, ATTITUDE, VFR_HUD, GPS_RAW_INT, ALTITUDE
    expected_telemetry_rate = telemetry_rate_hz * 6;
}

bool
Mavlink_vehicle::Telemetry::On_telemetry_check()
{
    if (telemetry_alive) {
        telemetry_alive = false;
    } else {
        // No telemetry data received lately.
        // Request telemetry only if there is no active mission upload.
        if (!vehicle.Is_mission_upload_active()) {
            VEHICLE_LOG_INF(vehicle, "Requesting telemetry...");
            vehicle.Initialize_telemetry();
        }
    }
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
    double telemetry_rate =
        config.DAMPING_FACTOR * vehicle.expected_telemetry_rate * ESTIMATION_RATE_MULTIPLIER;

    double rate_quality = telemetry_messages_last / telemetry_rate;
    if (rate_quality > 1) {
        /* Compensate damping factor. */
        rate_quality = 1;
    }
    double error_quality = 0;

    /* Calculate link quality based on error rate. */
    auto new_stats = vehicle.mav_stream->Get_decoder().Get_stats();
    double ok_diff = new_stats.handled - prev_stats.handled;
    double err_diff = (new_stats.bad_checksum + new_stats.unknown_id) -
                      (prev_stats.bad_checksum + prev_stats.unknown_id);
    err_diff += rx_errors_accum;
    rx_errors_accum = 0;
    prev_stats = new_stats;

    // Do not update link quality if mission upload is in progress.
    // We could calculate quality based on upload messages during upload.
    // But that requires taking into account the baud_rate and I do not want to
    // implement that right now as it there no real value.
    if (!vehicle.Is_mission_upload_active()) {
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
    }

    // round to the 2nd decimal to avoid sending on each update.
    vehicle.t_gcs_link_quality->Set_value(std::roundf(link_quality * 100.0) / 100.0);
    vehicle.Commit_to_ucs();

    telemetry_messages_last = 0;

    return true;
}

void
Mavlink_vehicle::Telemetry::On_sys_status(
        mavlink::Message<mavlink::MESSAGE_ID::SYS_STATUS>::Ptr message)
{
    telemetry_messages_last++;
    telemetry_alive = true;

    vehicle.t_main_voltage->Set_value(
        static_cast<double>(message->payload->voltage_battery) / 1000.0);
    if (message->payload->current_battery != -1) {
        vehicle.t_main_current->Set_value(
            static_cast<double>(message->payload->current_battery) / 100.0);
    }

    // ArDrone and ArduPlane always report 0 in onboard_control_sensors_health field,
    // So, set this only for arducopter.
    if (vehicle.Get_autopilot_type() == "ardupilot") {  //TODO: move to built in enum
        switch (vehicle.Get_mav_type()) {
        case mavlink::MAV_TYPE::MAV_TYPE_QUADROTOR:
        case mavlink::MAV_TYPE::MAV_TYPE_HEXAROTOR:
        case mavlink::MAV_TYPE::MAV_TYPE_OCTOROTOR:
        case mavlink::MAV_TYPE::MAV_TYPE_TRICOPTER:
        case mavlink::MAV_TYPE::MAV_TYPE_HELICOPTER:
        case mavlink::MAV_TYPE::MAV_TYPE_COAXIAL:
            if (message->payload->onboard_control_sensors_health & mavlink::MAV_SYS_STATUS_SENSOR_RC_RECEIVER) {
                vehicle.t_rc_link_quality->Set_value(1);
            } else {
                vehicle.t_rc_link_quality->Set_value(0);
            }
            break;
        default:
            break;
        }
    }

    int new_sensor_health = message->payload->onboard_control_sensors_health;
    int new_sensor_enabled = message->payload->onboard_control_sensors_enabled;
    // Ignore sensor removal
    vehicle.current_sensors_present |= message->payload->onboard_control_sensors_present;

    std::string enabled;
    std::string disabled;
    std::string failed;
    std::string healthy;
    for (size_t i = 0; i < SENSOR_NAMES.size(); i++) {
        size_t bit = (1 << i);
        if (vehicle.current_sensors_present & bit) {
            if ((vehicle.current_sensor_enabled & bit) != (new_sensor_enabled & bit)) {
                // sensor health changed.
                if ((new_sensor_enabled & bit)) {
                    if (enabled.empty()) {
                        enabled = "Enabled sensor: ";
                    } else {
                        enabled += ", ";
                    }
                    enabled += SENSOR_NAMES[i];
                } else {
                    if (disabled.empty()) {
                        disabled = "Disabled sensor: ";
                    } else {
                        disabled += ", ";
                    }
                    disabled += SENSOR_NAMES[i];
                }
            }
            if ((vehicle.current_sensor_health & bit) != (new_sensor_health & bit)) {
                // sensor health changed.
                if ((new_sensor_health & bit)) {
                    if (healthy.empty()) {
                        healthy = "Sensor Healthy: ";
                    } else {
                        healthy += ", ";
                    }
                    healthy += SENSOR_NAMES[i];
                } else {
                    if (failed.empty()) {
                        failed = "Sensor FAILED: ";
                    } else {
                        failed += ", ";
                    }
                    failed += SENSOR_NAMES[i];
                }
            }
        }
    }
    if (!enabled.empty()) {
        vehicle.Add_status_message(enabled);
    }
    if (!disabled.empty()) {
        vehicle.Add_status_message(disabled);
    }
    if (!healthy.empty()) {
        vehicle.Add_status_message(healthy);
    }
    if (!failed.empty()) {
        vehicle.Add_status_message(failed);
    }
    vehicle.current_sensor_health = new_sensor_health;
    vehicle.current_sensor_enabled = new_sensor_enabled;

    vehicle.Commit_to_ucs();
}

void
Mavlink_vehicle::Telemetry::On_global_position_int(
        mavlink::Message<mavlink::MESSAGE_ID::GLOBAL_POSITION_INT>::Ptr message)
{
    telemetry_messages_last++;
    telemetry_alive = true;

    vehicle.t_latitude->Set_value(static_cast<double>(message->payload->lat) / 1e7 / 180.0 * M_PI);
    vehicle.t_longitude->Set_value(static_cast<double>(message->payload->lon) / 1e7 / 180.0 * M_PI);
    vehicle.t_altitude_amsl->Set_value(static_cast<double>(message->payload->alt) / 1000.0);

    auto relative_altitude = static_cast<double>(message->payload->relative_alt) / 1000.0;

    if (vehicle.report_relative_altitude) {
        vehicle.t_altitude_raw->Set_value(relative_altitude);
    }

    // Vertical speed calculations.
    // Ardupilot reports crazy values in vz speed, ArDrone reports 0.
    // So, calculate vspeed from altitude changes which look much better.

    auto current_time_since_boot = static_cast<double>(message->payload->time_boot_ms) / 1000.0;

    if (static_cast<mavlink::ugcs::MAV_AUTOPILOT>(vehicle.Get_autopilot()) == mavlink::ugcs::MAV_AUTOPILOT_AR_DRONE) {
        // ArDrone reports time in microseconds, not milliseconds.
        current_time_since_boot /= 1000.0;
        // ArDrone reports horizontal speed always as 0.
        // Therefore we do not set it and let it stay N/A.
        // Same with course as it is derived from vx,vy.
    } else {
        double vx = static_cast<double>(message->payload->vx) / 100.0;
        double vy = static_cast<double>(message->payload->vy) / 100.0;

        float course = std::atan2(vy, vx);
        if (std::isnormal(course)) {
            vehicle.t_course->Set_value(course);
        } else {
            vehicle.t_course->Set_value_na();
        }
        vehicle.t_ground_speed->Set_value(hypot(vx,vy));
    }

    if (current_time_since_boot > prev_time_since_boot) {
        vehicle.t_vertical_speed->Set_value((relative_altitude - prev_altitude) / (current_time_since_boot - prev_time_since_boot));
    }
    prev_time_since_boot = current_time_since_boot;
    prev_altitude = relative_altitude;
    vehicle.Commit_to_ucs();
}

void
Mavlink_vehicle::Telemetry::On_attitude(
        mavlink::Message<mavlink::MESSAGE_ID::ATTITUDE>::Ptr message)
{
    telemetry_messages_last++;
    telemetry_alive = true;

    vehicle.t_pitch->Set_value(message->payload->pitch.Get());
    vehicle.t_roll->Set_value(message->payload->roll.Get());
    vehicle.t_heading->Set_value(message->payload->yaw.Get());
    vehicle.Commit_to_ucs();
}

void
Mavlink_vehicle::Telemetry::On_mission_current(
        mavlink::Message<mavlink::MESSAGE_ID::MISSION_CURRENT>::Ptr message)
{
    int cmd = message->payload->seq.Get();
    if (vehicle.current_mission_item_index != cmd) {
        VEHICLE_LOG_INF(vehicle, "Current mission item: %d", cmd);
    }
    vehicle.current_mission_item_index = cmd;

    if (    vehicle.Is_flight_mode(proto::FLIGHT_MODE_WAYPOINTS)
        ||  vehicle.Is_flight_mode(proto::FLIGHT_MODE_HOLD))
    {
        vehicle.t_current_command->Set_value(cmd);
    } else {
        vehicle.t_current_command->Set_value_na();
    }
    vehicle.Commit_to_ucs();
}

void
Mavlink_vehicle::Telemetry::On_vfr_hud(
        mavlink::Message<mavlink::MESSAGE_ID::VFR_HUD>::Ptr message)
{
    /* Duplicated in other messages. */
    telemetry_messages_last++;
    telemetry_alive = true;

    // Report airspeed only if value is non-zero and not equal to groundspeed in the same message.
    // air==ground assumes that airspeed sensor is not present.
    // At least it is what ardupilot sends.
    if (    message->payload->airspeed.Get() != message->payload->groundspeed.Get()
        &&  message->payload->airspeed.Get() != 0) {
        vehicle.t_air_speed->Set_value(message->payload->airspeed.Get());
        vehicle.Commit_to_ucs();
    }
}

constexpr std::chrono::milliseconds
    Mavlink_vehicle::ALTITUDE_ORIGIN_RESET_TRESHOLD;
void
Mavlink_vehicle::Telemetry::On_gps_raw(
        mavlink::Message<mavlink::MESSAGE_ID::GPS_RAW_INT>::Ptr message)
{
    telemetry_messages_last++;
    telemetry_alive = true;

    vehicle.Update_boot_time(std::chrono::milliseconds(message->payload->time_usec / 1000));

    if (message->payload->satellites_visible != 255) {
        vehicle.t_satellite_count->Set_value(message->payload->satellites_visible.Get());
    }
    switch (message->payload->fix_type) {
    case mavlink::GPS_FIX_TYPE_NO_GPS:
    case mavlink::GPS_FIX_TYPE_NO_FIX:
    case mavlink::GPS_FIX_TYPE_STATIC:
        vehicle.t_gps_fix->Set_value(ugcs::vsm::proto::GPS_FIX_TYPE_NONE);
        break;
    case mavlink::GPS_FIX_TYPE_2D_FIX:
        vehicle.t_gps_fix->Set_value(ugcs::vsm::proto::GPS_FIX_TYPE_2D);
        break;
    case mavlink::GPS_FIX_TYPE_3D_FIX:
        vehicle.t_gps_fix->Set_value(ugcs::vsm::proto::GPS_FIX_TYPE_3D);
        break;
    case mavlink::GPS_FIX_TYPE_DGPS:
        vehicle.t_gps_fix->Set_value(ugcs::vsm::proto::GPS_FIX_TYPE_DIFF);
        break;
    case mavlink::GPS_FIX_TYPE_RTK_FIXED:
        vehicle.t_gps_fix->Set_value(ugcs::vsm::proto::GPS_FIX_TYPE_RTK_FIXED);
        break;
    case mavlink::GPS_FIX_TYPE_RTK_FLOAT:
        vehicle.t_gps_fix->Set_value(ugcs::vsm::proto::GPS_FIX_TYPE_RTK_FLOAT);
        break;
    }
    vehicle.Commit_to_ucs();
}

void
Mavlink_vehicle::Telemetry::On_target_position(
    ugcs::vsm::mavlink::Message<ugcs::vsm::mavlink::MESSAGE_ID::POSITION_TARGET_GLOBAL_INT>::Ptr t)
{
//    LOG("Target = %04X %d %d %f ", t->payload->type_mask.Get(), t->payload->lat_int.Get(), t->payload->lon_int.Get(), t->payload->alt.Get());
    if ((t->payload->type_mask & 0x1) == 0) {
        double lat = t->payload->lat_int;
        vehicle.t_target_latitude->Set_value(lat / 10000000 * M_PI / 180);
    }

    if ((t->payload->type_mask & 0x2) == 0) {
        double lon = t->payload->lon_int;
        vehicle.t_target_longitude->Set_value(lon / 10000000 * M_PI / 180);
    }

    if ((t->payload->type_mask & 0x4) == 0) {
        switch (t->payload->coordinate_frame) {
        case mavlink::MAV_FRAME_GLOBAL:
        case mavlink::MAV_FRAME_GLOBAL_INT:
            vehicle.t_target_altitude_raw->Set_value_na();
            vehicle.t_target_altitude_amsl->Set_value(t->payload->alt);
            break;
        case mavlink::MAV_FRAME_GLOBAL_RELATIVE_ALT:
        case mavlink::MAV_FRAME_GLOBAL_RELATIVE_ALT_INT:
            vehicle.t_target_altitude_raw->Set_value(t->payload->alt);
            vehicle.t_target_altitude_amsl->Set_value_na();
            break;
        default:
            vehicle.t_target_altitude_raw->Set_value_na();
            vehicle.t_target_altitude_amsl->Set_value_na();
        }
    }

    if ((t->payload->type_mask & 0x7) != 0x7) {
        vehicle.Commit_to_ucs();
    }
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
        VEHICLE_LOG_DBG(vehicle, "3DR RADIO rxerrors counter wrapped (%u->%u)",
                prev_rx_errors_3dr, msg->payload->rxerrors.Get());
        rx_errors_accum += msg->payload->rxerrors;
    } else {
        rx_errors_accum += msg->payload->rxerrors - prev_rx_errors_3dr;
    }
    prev_rx_errors_3dr = msg->payload->rxerrors;
}

void
Mavlink_vehicle::Telemetry::On_altitude(
        mavlink::Message<mavlink::MESSAGE_ID::ALTITUDE>::Ptr message)
{
    telemetry_messages_last++;
    telemetry_alive = true;

    vehicle.t_altitude_amsl->Set_value(message->payload->altitude_amsl.Get());

    if (vehicle.report_relative_altitude) {
        vehicle.t_altitude_raw->Set_value(message->payload->altitude_relative.Get());
    }
    vehicle.Commit_to_ucs();
}

void
Mavlink_vehicle::Telemetry::On_servo_output_raw(
        mavlink::Message<mavlink::MESSAGE_ID::SERVO_OUTPUT_RAW>::Ptr message)
{
    telemetry_messages_last++;
    telemetry_alive = true;

    vehicle.t_servo_pwm_1->Set_value(message->payload->servo1_raw.Get());
    vehicle.t_servo_pwm_2->Set_value(message->payload->servo2_raw.Get());
    vehicle.t_servo_pwm_3->Set_value(message->payload->servo3_raw.Get());
    vehicle.t_servo_pwm_4->Set_value(message->payload->servo4_raw.Get());
    vehicle.t_servo_pwm_5->Set_value(message->payload->servo5_raw.Get());
    vehicle.t_servo_pwm_6->Set_value(message->payload->servo6_raw.Get());
    vehicle.t_servo_pwm_7->Set_value(message->payload->servo7_raw.Get());
    vehicle.t_servo_pwm_8->Set_value(message->payload->servo8_raw.Get());

    vehicle.Commit_to_ucs();
}

void
Mavlink_vehicle::Mission_upload::Enable()
{
    attempts_action_left = try_count;
    current_action = -1; /* Nothing requested yet. */

    Dump_mission();

    Register_mavlink_handler<mavlink::MESSAGE_ID::MISSION_REQUEST>(
        &Mission_upload::On_mission_request,
        this,
        Mavlink_demuxer::COMPONENT_ID_ANY);

    Register_mavlink_handler<mavlink::MESSAGE_ID::MISSION_ACK>(
        &Mission_upload::On_mission_ack,
        this,
        Mavlink_demuxer::COMPONENT_ID_ANY);
    Try();
}

bool
Mavlink_vehicle::Mission_upload::Is_active()
{
    return (timer != nullptr);
}

void
Mavlink_vehicle::Mission_upload::On_disable()
{
    Unregister_mavlink_handlers();
    final_ack_waiting = false;

    if (timer) {
        timer->Cancel();
        timer = nullptr;
    }
}

bool
Mavlink_vehicle::Mission_upload::Try()
{
    if (attempts_action_left--) {
        if (current_action == -1) {
            /* Initiate upload procedure. */
            mavlink::Pld_mission_count mc;
            mc->count = mission_items.size();
            Fill_target_ids(mc);
            Send_message(mc);
        }
        Schedule_timer(retry_timeout);
    } else {
        Call_next_action(false, "No request for mission item received");
    }
    return false;
}

void
Mavlink_vehicle::Mission_upload::On_mission_ack(
        mavlink::Message<mavlink::MESSAGE_ID::MISSION_ACK>::Ptr message)
{
    VEHICLE_LOG_INF(vehicle, "MISSION ACK: %s", message->payload.Dump().c_str());
    if (message->payload->target_component != VSM_COMPONENT_ID ||
        message->payload->target_system != VSM_SYSTEM_ID) {
        /* Not for us, ignore. */
        return;
    }
    if (message->payload->type == mavlink::MAV_MISSION_RESULT::MAV_MISSION_ACCEPTED) {
        Call_next_action(true);
    } else {
        auto p = message->payload->type.Get();
        Call_next_action(
                false,
                std::string("Result: ")
                + std::to_string(p)
                + " ("
                + Mav_mission_result_to_string(p)
                + ")");
        Disable();
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
        VEHICLE_LOG_INF(vehicle, "Requested same item: %ld", current_action);
    } else if (current_action + 1 != message->payload->seq) {
        /* Wrong sequence. */
        return;
    } else {
        attempts_action_left = try_count;
        current_action++;
    }

    if (current_action + 1 == static_cast<ssize_t>(mission_items.size()) && !final_ack_waiting) {
        final_ack_waiting = true;
    }

    Upload_current_action();
}

void
Mavlink_vehicle::Mission_upload::Schedule_timer(std::chrono::milliseconds timeout)
{
    if (timer) {
        timer->Cancel();
    }
    timer = Timer_processor::Get_instance()->Create_timer(
        timeout,
        Make_callback(&Mission_upload::Try, this),
        vehicle.Get_completion_ctx());
}

void
Mavlink_vehicle::Mission_upload::Upload_current_action()
{
    ASSERT(current_action < static_cast<ssize_t>(mission_items.size()));
    mavlink::Payload_base& msg = *mission_items[current_action];
    VEHICLE_LOG_INF(vehicle, "Uploading [%s][seq %zu].",
            msg.Get_name(),
            current_action);
    Send_message(msg);
    Schedule_timer(retry_timeout);
}

void
Mavlink_vehicle::Mission_upload::Dump_mission()
{
    if (!vehicle.mission_dump_path) {
        return;
    }
    std::string filename = *vehicle.mission_dump_path;

    auto now = std::chrono::system_clock::now();
    std::time_t t = std::chrono::system_clock::to_time_t(now);
    char ts_buf[128];
    std::strftime(ts_buf, sizeof(ts_buf), "_%Y%m%d-%H%M%S", std::localtime(&t));
    filename += ts_buf;
    std::unique_ptr<std::fstream> f = std::unique_ptr<std::fstream>
        (new std::fstream(filename, std::ios_base::out|std::ios_base::app));
    if (!f->is_open()) {
        VEHICLE_LOG_WRN(vehicle, "Failed to open mission dumping file: %s", filename.c_str());
    } else {
        (*f) << std::ctime(&t)
        << "Dumping mission for [" << vehicle.Get_model_name() << ":" << vehicle.Get_serial_number() << "]:"
        << std::endl << std::endl;
        for (auto& iter: mission_items) {
            (*f) << iter->Dump() << std::endl;
        }
        (*f) << std::endl << "END OF THE MISSION DUMP" << std::endl << std::endl;
        VEHICLE_LOG_INF(vehicle, "Mission dump file created: %s", filename.c_str());
    }
}

uint32_t
Mavlink_vehicle::Get_mission_item_hash(mavlink::Pld_mission_item& msg)
{
    Crc32 h;
    // Hash command ID.
    h.Add_short(msg->command);
    // Do not hash command parameters because downloaded arguments
    // tend to differ from uploaded arguments.
    // hash coordinates only for commands which have coords.
    if (    msg->command == mavlink::MAV_CMD_NAV_WAYPOINT
        ||  msg->command == mavlink::MAV_CMD_NAV_TAKEOFF
        ||  msg->command == mavlink::MAV_CMD_NAV_LOITER_TIME
        ||  msg->command == mavlink::MAV_CMD_NAV_LOITER_TURNS
        ||  msg->command == mavlink::MAV_CMD_NAV_LOITER_UNLIM
        ||  msg->command == mavlink::MAV_CMD_NAV_LAND
        ||  msg->command == mavlink::MAV_CMD_NAV_SPLINE_WAYPOINT
        ||  msg->command == mavlink::MAV_CMD_NAV_TAKEOFF_LOCAL
        ||  msg->command == mavlink::MAV_CMD_NAV_LAND_LOCAL
        ||  msg->command == mavlink::MAV_CMD_NAV_VTOL_LAND
        ||  msg->command == mavlink::MAV_CMD_NAV_VTOL_TAKEOFF
        ||  msg->command == mavlink::MAV_CMD_NAV_ROI)
    {
        h.Add_int(static_cast<int>(msg->x.Get() * 100000));  // +/- 10m
        h.Add_int(static_cast<int>(msg->y.Get() * 100000));  // +/- 10m
        h.Add_int(static_cast<int>(msg->y.Get() * 100));     // +/- 10cm
    }
    return h.Get();
}

bool
Mavlink_vehicle::Is_current_command(int cmd)
{
    if (!t_current_command->Is_value_na()) {
        if (auto mi = current_route.Get_item_ref(current_mission_item_index)) {
            if (mi && (*mi)->command == cmd) {
                // there is a saved mission which has this idx.
                return true;
            }
        }
    }
    return false;
}

bool
Mavlink_vehicle::Is_current_command_last()
{
    return ((current_mission_item_index > 0) && current_route.Get_item_count() == current_mission_item_index + 1);
}

void
Mavlink_vehicle::Mavlink_route::Reset()
{
    items.clear();
}

void
Mavlink_vehicle::Mavlink_route::Add_item(const mavlink::Pld_mission_item& mi)
{
    items[mi->seq] = mi;
}

int
Mavlink_vehicle::Mavlink_route::Get_item_count()
{
    return items.size();
}

const mavlink::Pld_mission_item*
Mavlink_vehicle::Mavlink_route::Get_item_ref(int idx)
{
    auto i = items.find(idx);
    if (i == items.end()) {
        return nullptr;
    } else {
        return &i->second;
    }
}
