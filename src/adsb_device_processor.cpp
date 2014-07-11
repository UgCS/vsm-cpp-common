// Copyright (c) 2014, Smart Projects Holdings Ltd
// All rights reserved.
// See LICENSE file for license details.

#include <adsb_device_processor.h>

ugcs::vsm::Singleton<Adsb_device_processor> Adsb_device_processor::singleton;

Adsb_device_processor::Adsb_device_processor() :
        ugcs::vsm::Request_processor("Adsb device processor")
{

}

ugcs::vsm::Request_completion_context::Ptr
Adsb_device_processor::Get_completion_context()
{
    return completion_ctx;
}

void
Adsb_device_processor::On_enable()
{
    completion_ctx = ugcs::vsm::Request_completion_context::Create("Adsb device processor completion");
    worker = ugcs::vsm::Request_worker::Create(
            "Adsb device processor worker",
            std::initializer_list<Request_container::Ptr>
            {
                completion_ctx, Shared_from_this()
            });
    completion_ctx->Enable();
    worker->Enable();
}

void
Adsb_device_processor::On_disable()
{
    Set_disabled();
    worker->Disable();
    completion_ctx->Disable();
    completion_ctx = nullptr;
    worker = nullptr;
}
