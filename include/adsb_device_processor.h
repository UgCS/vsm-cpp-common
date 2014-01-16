// Copyright (c) 2014, Smart Projects Holdings Ltd
// All rights reserved.
// See LICENSE file for license details.

/**
 * @file adsb_device_processor.h
 */
#ifndef _ADSB_DEVICE_PROCESSOR_H_
#define _ADSB_DEVICE_PROCESSOR_H_

#include <vsm/request_worker.h>
#include <vsm/singleton.h>

/** Helper processor for managing ADS-B devices, i.e. low level operations long
 * before ADS-B frames decoding.
 */
class Adsb_device_processor:public vsm::Request_processor {
    DEFINE_COMMON_CLASS(Adsb_device_processor, vsm::Request_processor)

public:

    /** Constructor. */
    Adsb_device_processor();

    /** Get global or create new processor instance. */
    template <typename... Args>
    static Ptr
    Get_instance(Args &&... args)
    {
        return singleton.Get_instance(std::forward<Args>(args)...);
    }

    /** Get common completion context. */
    vsm::Request_completion_context::Ptr
    Get_completion_context();

private:

    /** Enable the processor. */
    virtual void
    On_enable() override;

    /** Disable the processor. */
    virtual void
    On_disable() override;

    /** Dedicated worker. */
    vsm::Request_worker::Ptr worker;

    /** Common completion context for all ADS-B devices. */
    vsm::Request_completion_context::Ptr completion_ctx;

    /** ADS-B device processor singleton instance. */
    static vsm::Singleton<Adsb_device_processor> singleton;

};

#endif /* _ADSB_DEVICE_PROCESSOR_H_ */
