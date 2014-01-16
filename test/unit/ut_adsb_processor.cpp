// Copyright (c) 2014, Smart Projects Holdings Ltd
// All rights reserved.
// See LICENSE file for license details.

#include <UnitTest++.h>
#include <adsb_processor.h>
#include <micro_adsb_device.h>
#include <vsm/file_processor.h>

using namespace vsm;

class Adsb_processor_tester: public Adsb_processor
{
    DEFINE_COMMON_CLASS(Adsb_processor_tester, Adsb_processor)
public:
    void
    Aircraft_ICAO_address_unique()
    {
        _initiate_reading = false;
        Init();
        Add_device(device);

        /* Aircrafts with the same ICAO address are always the same. */
        Adsb_frame::ICAO_address addr(Adsb_frame::ICAO_address::Type::REAL, 1,2,3);
        Adsb_frame::ICAO_address addr_copy = addr;
        Adsb_frame::ICAO_address addr_new(Adsb_frame::ICAO_address::Type::REAL, 1,2,4);
        /* Same as addr, but different type. */
        Adsb_frame::ICAO_address addr_new_anon(Adsb_frame::ICAO_address::Type::ANONYMOUS, 1,2,3);

        Adsb_processor::Aircraft::Ptr lookup1 = Lookup_aircarft(device, addr);
        Adsb_processor::Aircraft::Ptr lookup2 = Lookup_aircarft(device, addr);

        CHECK_EQUAL(lookup1, lookup2);

        lookup2 = Lookup_aircarft(device, addr_copy);

        CHECK_EQUAL(lookup1, lookup2);

        lookup2 = Lookup_aircarft(device, addr_new);

        CHECK(lookup1 != lookup2);

        Adsb_processor::Aircraft::Ptr lookup3 = Lookup_aircarft(device, addr_new_anon);

        CHECK(lookup3 != lookup1);
        CHECK(lookup3 != lookup2);

        CHECK_EQUAL(3, aircrafts[device].aircrafts.size());
    }

    void
    Aircrafts_size_limit()
    {
        _initiate_reading = false;
        Init();
        Add_device(device);

        size_t i;
        for (i = 0; i < MAX_AIRCRAFTS; i++) {
            Adsb_frame::ICAO_address addr(Adsb_frame::ICAO_address::Type::REAL, i, i >> 8, i >> 16);
            Adsb_processor::Aircraft::Ptr acraft = Lookup_aircarft(device, addr, true);
            CHECK(acraft != nullptr);
        }
        Adsb_frame::ICAO_address addr(Adsb_frame::ICAO_address::Type::REAL, i, i >> 8, i >> 16);
        /* Limit reached. */
        Adsb_processor::Aircraft::Ptr acraft = Lookup_aircarft(device, addr, true);
        CHECK(acraft == nullptr);
        /* Limit reached, but no check. */
        acraft = Lookup_aircarft(device, addr);
        CHECK(acraft != nullptr);
    }

    void
    Init()
    {
        auto proc = File_processor::Get_instance();
        auto stream = proc->Open("dummy_test_file", "w");
        device = Micro_adsb_device::Create(stream);
    }

    Micro_adsb_device::Ptr device;
};

TEST(aircraft_icao_address_unique)
{
    auto adsb_device_proc = Adsb_device_processor::Get_instance();
    adsb_device_proc->Enable();
    auto proc = File_processor::Get_instance();
    proc->Enable();
    auto tester = Adsb_processor_tester::Create();
    tester->Enable();
    tester->Aircraft_ICAO_address_unique();
    /* Workaround for UGCS-406 */
    std::this_thread::sleep_for(std::chrono::seconds(1));
    tester->Disable();
    proc->Disable();
    adsb_device_proc->Disable();
}

TEST(aircrafts_size_limit)
{
    auto adsb_device_proc = Adsb_device_processor::Get_instance();
    adsb_device_proc->Enable();
    auto proc = File_processor::Get_instance();
    proc->Enable();
    auto tester = Adsb_processor_tester::Create();
    tester->Enable();
    tester->Aircrafts_size_limit();
    /* Workaround for UGCS-406 */
    std::this_thread::sleep_for(std::chrono::seconds(1));
    tester->Disable();
    proc->Disable();
    adsb_device_proc->Disable();
}
