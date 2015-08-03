// Copyright (c) 2014, Smart Projects Holdings Ltd
// All rights reserved.
// See LICENSE file for license details.

#include <adsb_manager.h>
#include <ugcs/vsm/transport_detector.h>
#include <adsb_processor.h>
#include <ugcs/vsm/cucs_processor.h>
#include <ugcs/vsm/peripheral_message.h>
#include <adsb_device.h>


/* Macro for ADS-B detectors */
#define ADD_ADSB_DETECTOR(SPECIFIC_DEVICE) \
    ugcs::vsm::Transport_detector::Get_instance()->Add_detector( \
            config_prefix + SPECIFIC_DEVICE::Get_config_prefix(), \
            ugcs::vsm::Transport_detector::Make_connect_handler( \
                    &SPECIFIC_DEVICE::On_new_connection), \
                    Shared_from_this())

ugcs::vsm::Singleton<Adsb_manager> Adsb_manager::singleton;

Adsb_manager::Adsb_manager(const std::string& prefix) :
        ugcs::vsm::Request_processor("Adsb manager processor"),
        config_prefix(prefix)
{
}

void
Adsb_manager::On_enable()
{
    worker = ugcs::vsm::Request_worker::Create(
            "Adsb manager worker",
            std::initializer_list<ugcs::vsm::Request_container::Ptr>{Shared_from_this()});
    worker->Enable();

    auto req = ugcs::vsm::Request::Create();
    req->Set_processing_handler(
            Make_callback(
                    &Adsb_manager::Process_on_enable,
                    Shared_from_this(),
                    req));
    Submit_request(req);


    ugcs::vsm::Cucs_processor::Get_instance()->Register_on_new_ucs_connection(
    		ugcs::vsm::Callback_proxy<void>(
    				Make_callback(
    						&Adsb_manager::On_new_cucs_connection,
    						Shared_from_this()
    					)));

	heartbeat_service_timer = ugcs::vsm::Timer_processor::Get_instance()->Create_timer(
		std::chrono::milliseconds(1000),
		ugcs::vsm::Make_callback(
				&Adsb_manager::On_heartbeat_service_tick,
					Shared_from_this()),
			worker);

    req->Wait_done(false);
}

void
Adsb_manager::Process_on_enable(ugcs::vsm::Request::Ptr request)
{
    reports_stream = Adsb_processor::Get_instance()->Open_stream();
    Read_adsb_report();

    /* microADSB */
    ADD_ADSB_DETECTOR(Micro_adsb_device);

    /* TestDevice */

    /* Finished setting up device detectors */
    request->Complete();
}

void
Adsb_manager::On_disable()
{
	shutting_down = true;
  	ugcs::vsm::Timer_processor::Get_instance()->Cancel_timer(heartbeat_service_timer);
  	heartbeat_service_timer = nullptr;

	auto req = ugcs::vsm::Request::Create();
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
Adsb_manager::Process_on_disable(ugcs::vsm::Request::Ptr request)
{
    for(auto& iter: streams_under_detection) {
        /* Disable device. */
        iter.second.device->Disable();
    }
    streams_under_detection.clear();

    adsb_devices_arbiter.lock();
	for(auto& iter: devices) {
		ugcs::vsm::Peripheral_message::Peripheral_update::Ptr upd;
/*		upd = ugcs::vsm::Peripheral_message::Peripheral_update::Create(iter.first->Get_id(),
			ugcs::vsm::Peripheral_message::PERIPHERAL_STATE::PERIPHERAL_STATE_HEARTBEAT_NOK);
		ugcs::vsm::Cucs_processor::Get_instance()->Send_peripheral_update(*upd);*/

		upd = ugcs::vsm::Peripheral_message::Peripheral_update::Create(iter.first->Get_id(),
			ugcs::vsm::Peripheral_message::PERIPHERAL_STATE::PERIPHERAL_STATE_DISCONNECTED);
		ugcs::vsm::Cucs_processor::Get_instance()->Send_peripheral_update(*upd);
	}
	adsb_devices_arbiter.unlock();


    reports_stream->Close();
    report_read_op.Abort();

    request->Complete();
}

void
Adsb_manager::On_adsb_report(ugcs::vsm::Adsb_report report)
{
    ugcs::vsm::Cucs_processor::Get_instance()->Send_adsb_report(report);
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

//XXX
void
Adsb_manager::On_new_connection(
		Adsb_device::Ptr device,
		std::string,
		int,
		ugcs::vsm::Socket_address::Ptr,
		ugcs::vsm::Io_stream::Ref stream)
{

/*	if(Is_shutting_down()) {
		LOG_DEBUG("New device detected during shutdown.");
	return;
	}*/
	LOG_DEBUG("Device [%s] found on port [%s]", device->Get_friendly_name().c_str(), device->Get_port_name().c_str());
    device->Enable();

    auto iter = streams_under_detection.emplace(stream, device);

    Detector_ctx& ctx = iter.first->second;

    ctx.read_op = device->Read_frame(
            Adsb_device::Make_read_frame_handler(
                    &Adsb_manager::On_frame_received,
                    Shared_from_this(),
                    device, stream), worker);

    ctx.init_op = device->Init_frames_receiving(
            Adsb_device::Make_init_frames_receiving_handler(
                    &Adsb_manager::On_init_frames_receiving,
                    Shared_from_this(),
                    device, stream), worker);
}

void
Adsb_manager::On_init_frames_receiving(
        ugcs::vsm::Io_result result,
        Adsb_device::Ptr device,
        ugcs::vsm::Io_stream::Ref stream)
{
    auto found = streams_under_detection.find(stream);
    ASSERT(found != streams_under_detection.end());

    streams_under_detection.erase(found);

    if (result != ugcs::vsm::Io_result::OK) {
        LOG_DEBUG("Device not detected on [%s].",
                stream->Get_name().c_str());
        device->Disable();
        ugcs::vsm::Transport_detector::Get_instance()->Protocol_not_detected(stream);
    } else {
        LOG_INFO("[%s] detected (frames initialization succeeded).",
                device->Get_name().c_str());
    	Add_device(device);

        Adsb_processor::Get_instance()->Add_device(device);
    }
}

void
Adsb_manager::On_frame_received(ugcs::vsm::Io_buffer::Ptr buffer,
        ugcs::vsm::Io_result result, Adsb_device::Ptr device, ugcs::vsm::Io_stream::Ref stream)
{
    auto found = streams_under_detection.find(stream);
    ASSERT(found != streams_under_detection.end());

    if (result != ugcs::vsm::Io_result::OK) {
        LOG_DEBUG("Device not detected on [%s].",
        		stream->Get_name().c_str());
        streams_under_detection.erase(found);
        device->Disable();
        ugcs::vsm::Transport_detector::Get_instance()->Protocol_not_detected(stream);
        return;
    } else if (buffer->Get_length() == ugcs::vsm::Adsb_frame::SIZE) {
        ugcs::vsm::Adsb_frame::Ptr frame = ugcs::vsm::Adsb_frame::Create(buffer);
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

ugcs::vsm::Request_completion_context::Ptr
Adsb_manager::Get_worker() {
	return worker;
}


/** Heartbeat-related section, works with device collection
 * and provides tick-based function that regularly updates server
 * on current device heartbeat status.
 */
bool
Adsb_manager::Add_device(Adsb_device::Ptr dev) {
	ugcs::vsm::Peripheral_message::Peripheral_register::Ptr rep;
    adsb_devices_arbiter.lock();
	if (devices.emplace(dev, false).second) {
	// Notify device addition here

		rep = ugcs::vsm::Peripheral_message::Peripheral_register::Create(dev->Get_id(),
				dev->Get_dev_type(), dev->Get_friendly_name(), dev->Get_port_name());
		ugcs::vsm::Cucs_processor::Get_instance()->Send_peripheral_register(*rep);
	    adsb_devices_arbiter.unlock();
	    return true;

	}
	adsb_devices_arbiter.unlock();
	return false;
}

bool
Adsb_manager::Remove_device(Adsb_device::Ptr dev) {
	ugcs::vsm::Peripheral_message::Peripheral_update::Ptr upd;
	adsb_devices_arbiter.lock();
	if(devices.erase(dev)) {
		//notify device removal here

		upd = ugcs::vsm::Peripheral_message::Peripheral_update::Create(dev->Get_id(),
				ugcs::vsm::Peripheral_message::PERIPHERAL_STATE::PERIPHERAL_STATE_DISCONNECTED);
		ugcs::vsm::Cucs_processor::Get_instance()->Send_peripheral_update(*upd);

        Adsb_processor::Get_instance()->Remove_device(dev);
	    adsb_devices_arbiter.unlock();
		return true;
	}
    adsb_devices_arbiter.unlock();
	return false;
}

bool
Adsb_manager::On_heartbeat_service_tick() {
	ugcs::vsm::Peripheral_message::Peripheral_update::Ptr upd;
    adsb_devices_arbiter.lock();
    std::list<Adsb_device::Ptr> devices_copy;

	for(auto& iter: devices) {
		if (iter.first->Get_is_heartbeat_ok()) {

			// Send heartbeat ok signal
			upd = ugcs::vsm::Peripheral_message::Peripheral_update::Create(iter.first->Get_id(),
					ugcs::vsm::Peripheral_message::PERIPHERAL_STATE::PERIPHERAL_STATE_HEARTBEAT_OK);
			ugcs::vsm::Cucs_processor::Get_instance()->Send_peripheral_update(*upd);

			// If heartbeat received for the first time after a period of inactivity
			if(iter.second == false) {
				LOG_INFO("Device [%s] reports heartbeat acquired.", iter.first->Get_name().c_str());
			}

			iter.second = true;
		}
		else {

			// Send heartbeat not ok signal
			upd = ugcs::vsm::Peripheral_message::Peripheral_update::Create(iter.first->Get_id(),
								ugcs::vsm::Peripheral_message::PERIPHERAL_STATE::PERIPHERAL_STATE_HEARTBEAT_NOK);
						ugcs::vsm::Cucs_processor::Get_instance()->Send_peripheral_update(*upd);

			// If heartbeat not received for the first time after a period of activity
			if(iter.second == true) {
				LOG_INFO("Device [%s] reports heartbeat timeout.", iter.first->Get_name().c_str());
			}

			iter.second = false;

			// If device reports timeout threshold to shutdown
			if(iter.first->Get_is_heartbeat_shutdown()) {
				LOG_INFO("Device [%s] timed out.", iter.first->Get_name().c_str());
				devices_copy.push_back(iter.first);
			}
		}
	}
    adsb_devices_arbiter.unlock();

    for (auto &iter: devices_copy) {
        Remove_device(iter);
    }
    devices_copy.clear();
	return true;
}

void
Adsb_manager::On_new_cucs_connection() {
    LOG_INFO("Broadcasting device list due to new UCS connection...");
    auto req = ugcs::vsm::Request::Create();
    req->Set_processing_handler(
            Make_callback(
                    &Adsb_manager::On_new_cucs_connection_cb,
                    Shared_from_this(),
                    req));
    Submit_request(req);

}

void
Adsb_manager::On_new_cucs_connection_cb(ugcs::vsm::Request::Ptr request) {
	ugcs::vsm::Peripheral_message::Peripheral_register::Ptr rep;
	ugcs::vsm::Peripheral_message::Peripheral_update::Ptr upd;
    adsb_devices_arbiter.lock();
	for(auto& iter: devices) {
		rep = ugcs::vsm::Peripheral_message::Peripheral_register::Create(iter.first->Get_id(),
				iter.first->Get_dev_type(), iter.first->Get_friendly_name(), iter.first->Get_port_name());
		ugcs::vsm::Cucs_processor::Get_instance()->Send_peripheral_register(*rep);

/*		if (iter.first->Get_is_heartbeat_ok())
			upd = ugcs::vsm::Peripheral_message::Peripheral_update::Create(iter.first->Get_id(),
					ugcs::vsm::Peripheral_message::PERIPHERAL_STATE::PERIPHERAL_STATE_HEARTBEAT_OK);
		else
			upd = ugcs::vsm::Peripheral_message::Peripheral_update::Create(iter.first->Get_id(),
					ugcs::vsm::Peripheral_message::PERIPHERAL_STATE::PERIPHERAL_STATE_HEARTBEAT_NOK);
		ugcs::vsm::Cucs_processor::Get_instance()->Send_peripheral_update(*upd);
		*/
	}
	adsb_devices_arbiter.unlock();
    request->Complete();
}
