// Copyright (c) 2014, Smart Projects Holdings Ltd
// All rights reserved.
// See LICENSE file for license details.

#include <cpr_decoder.h>
#include <vsm/vsm.h>
#include <fstream>

#include <UnitTest++.h>

using namespace vsm;

constexpr double tolerance_meters = 5.1; /* Official CPR tolerance. */
constexpr double earth_equator_to_pole_meters = 10000000; /* 10.000 Km */
constexpr double tolerance = 90 * (tolerance_meters / earth_equator_to_pole_meters);

/*
 * Longitude tolerance at latitudes 'colder' that 80 degrees yields to
 * some 120 meters. This shouldn't be a problem in practice, however.
 * Official documents say, that tolerance is reduced from 5.1 to some 10 meters,
 * but current implementation (despite being exactly as described in the docs)
 * gives only 120 meters tolerance. I don't know why.
 */
constexpr double lon_tolerane_after80 = 0.0011;

/* Longitude tolerance is generally worse than latitude. */
constexpr double lon_tolerane = tolerance * 3;

#define FILL_LAT_LON(_frame, _lat, _lon, _format) \
    _frame.ME[2] |= (_format << 2) & 0x4; \
    _frame.ME[2] |= (_lat >> 15) & 3; \
    _frame.ME[3] = (_lat >> 7) & 0xff; \
    _frame.ME[4] |= (_lat & 0x7f) << 1; \
    _frame.ME[4] |= (_lon >> 16) & 1; \
    _frame.ME[5] |= (_lon >> 8) & 0xff; \
    _frame.ME[6] |= _lon & 0xff;

Adsb_frame::Airborne_position_message
Create_airborne_position(int lat, int lon, bool format)
{
    Adsb_frame::Frame frame;
    memset(&frame, 0, sizeof(frame));
    FILL_LAT_LON(frame, lat, lon, format);
    return Adsb_frame::Create(Io_buffer::Create(&frame, sizeof(frame)));
}

TEST(notify_about_tolerance)
{
    LOG_INFO("CPR latitude tolerance = %f degrees (%.3f meters)", tolerance, tolerance_meters);
    LOG_INFO("CPR longitude tolerance = %f degrees (%f degrees close to the poles).",
            lon_tolerane, lon_tolerane_after80);
}

/* Check for several local decodings after global decode. */
TEST_FIXTURE(Airborne_cpr_decoder, cpr_airborne_local_decoding)
{
    auto frame1 = Create_airborne_position(65500, 71361, false);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    auto frame2 = Create_airborne_position(51301, 98304, true);
    auto res = Global_decode(frame1, frame2);
    CHECK(res);
    CHECK_CLOSE(38.998346, (*res).latitude * 180 / M_PI, tolerance);
    CHECK_CLOSE(-74, (*res).longitude * 180 / M_PI, lon_tolerane);

    auto frame3 = Create_airborne_position(65536, 71361, false);
    auto local1 = Local_decode(*res, frame3);
    CHECK_CLOSE(39, local1.latitude * 180 / M_PI, tolerance);
    CHECK_CLOSE(-74.000025, local1.longitude * 180 / M_PI, lon_tolerane);

    /* Make sure that new position with distance slightly more that 6 NM is discarded. */
    auto frame3_far1 = Create_airborne_position(67718, 71386, false);
    auto local1_far1 = Local_decode(local1, frame3_far1);
    CHECK_CLOSE(39.099884, local1_far1.latitude * 180 / M_PI, tolerance);
    CHECK_CLOSE(-73.998533, local1_far1.longitude * 180 / M_PI, tolerance);
    double distance = Wgs84_position(local1).Distance(local1_far1);
    CHECK(distance / 1.852 > 6000);


    /* Both positions below are slightly less that 6 NM from previous one. */
    auto frame4 = Create_airborne_position(67716, 71398, false);
    auto local2 = Local_decode(local1, frame4);
    CHECK_CLOSE(39.099792, local2.latitude * 180 / M_PI, tolerance);
    CHECK_CLOSE(-73.997816, local2.longitude * 180 / M_PI, lon_tolerane);
    distance = Wgs84_position(local1).Distance(local2);
    CHECK(distance / 1.852 < 6000);

    auto frame5 = Create_airborne_position(53480, 98340, true);
    auto local3 = Local_decode(local2, frame5);
    CHECK_CLOSE(39.099788, local3.latitude * 180 / M_PI, tolerance);
    CHECK_CLOSE(-73.997803, local3.longitude * 180 / M_PI, lon_tolerane);
    distance = Wgs84_position(local1).Distance(local3);
    CHECK(distance / 1.852 < 6000);
}

/* Check for several local decodings after global decode. */
TEST_FIXTURE(Surface_cpr_decoder, cpr_surface_local_decoding)
{
    Geodetic_tuple receiver_pos(39 * M_PI / 180.0, -74 * M_PI / 180.0, 0);
    Set_receiver_position(receiver_pos);

    auto frame1 = Create_airborne_position(130929, 23302, false);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    auto frame2 = Create_airborne_position(74133, 0, true);
    auto res = Global_decode(frame1, frame2);
    CHECK(res);
    CHECK_CLOSE(38.998357, (*res).latitude * 180 / M_PI, tolerance);
    CHECK_CLOSE(-73.999995, (*res).longitude * 180 / M_PI, lon_tolerane);

    auto frame3 = Create_airborne_position(0, 23302, false);
    auto local1 = Local_decode(*res, frame3);
    CHECK_CLOSE(39, local1.latitude * 180 / M_PI, tolerance);
    CHECK_CLOSE(-73.999995, local1.longitude * 180 / M_PI, lon_tolerane);

    /* Make sure that new positions with distances more that 0.75 NM are discarded. */
    auto frame3_far1 = Create_airborne_position(5373, 23424, false);
    auto local1_far1 = Local_decode(local1, frame3_far1);
    CHECK_CLOSE(39.061489, local1_far1.latitude * 180 / M_PI, tolerance);
    CHECK_CLOSE(-73.998174, local1_far1.longitude * 180 / M_PI, tolerance);
    double distance = Wgs84_position(local1).Distance(local1_far1);
    CHECK(distance / 1.852 > 3690);

    frame3_far1 = Create_airborne_position(79557, 120, true);
    local1_far1 = Local_decode(local1, frame3_far1);
    CHECK_CLOSE(39.061482, local1_far1.latitude * 180 / M_PI, tolerance);
    CHECK_CLOSE(-73.998169, local1_far1.longitude * 180 / M_PI, tolerance);
    distance = Wgs84_position(local1).Distance(local1_far1);
    CHECK(distance / 1.852 > 3690);

    /* Both positions below are less that 0.75 NM from previous one. */
    auto frame4 = Create_airborne_position(898, 23424, false);
    auto local2 = Local_decode(local1, frame4);
    CHECK_CLOSE(39.010277, local2.latitude * 180 / M_PI, tolerance);
    CHECK_CLOSE(-73.998174, local2.longitude * 180 / M_PI, lon_tolerane);
    distance = Wgs84_position(local1).Distance(local2);
    CHECK(distance / 1.852 < 624);

    auto frame5 = Create_airborne_position(75157, 120, true);
    auto local3 = Local_decode(local2, frame5);
    CHECK_CLOSE(39.010275, local3.latitude * 180 / M_PI, tolerance);
    CHECK_CLOSE(-73.998169, local3.longitude * 180 / M_PI, lon_tolerane);
    distance = Wgs84_position(local1).Distance(local3);
    CHECK(distance / 1.852 < 624);
}

/* CPR decoding test class for tabular data read from the file.
 * The file content is from the official docs.
 */
template<class Decoder, const char* file_name>
class Cpr_decoding: public Decoder
{
public:
    void
    Run()
    {
        std::fstream file(file_name, std::ios_base::in);
        if (!file.is_open()) {
            CHECK(false);
            throw "Can not open CPR decoding test data file!";
        }
        int count = 0;
        while (!file.eof()) {
            double lat_deg, lon_deg;
            uint32_t unused, lat_even, lon_even, lat_odd, lon_odd;
            if (!(file >> lat_deg)) {
                CHECK(false);
                throw "Can not read the file!";
            }
            file >> std::hex >> unused;
            file >> lon_deg >> std::hex >> unused >>
            lat_even >> lon_even >> lat_odd >> lon_odd;

            Geodetic_tuple receiver_position(lat_deg * M_PI / 180, lon_deg * M_PI / 180, 0);
            Decoder::Set_receiver_position(receiver_position);

            auto even = Create_airborne_position(lat_even, lon_even, false);
            CHECK_EQUAL(lat_even, even.Get_CPR_latitude());
            CHECK_EQUAL(lon_even, even.Get_CPR_longitude());

            /* Odd frame should have more recent received time. */
            std::this_thread::sleep_for(std::chrono::milliseconds(1));

            auto odd = Create_airborne_position(lat_odd, lon_odd, true);
            CHECK_EQUAL(lat_odd, odd.Get_CPR_latitude());
            CHECK_EQUAL(lon_odd, odd.Get_CPR_longitude());

            CHECK(even.Get_received_time() != odd.Get_received_time());
            auto res = Decoder::Global_decode(even, odd);
            CHECK(res);
            CHECK_CLOSE(lat_deg, (*res).latitude * 180 / M_PI, tolerance);
            /* Check also local decoding, it should match with global,
             * because it is calculate based on the same data, but using
             * different method.
             */
            auto local = Decoder::Local_decode(*res, odd);
            CHECK_CLOSE(lat_deg, local.latitude * 180 / M_PI, tolerance);
            /* Tolerance is worse closer to the poles, but who cares? */
            if (std::abs(lat_deg) >= 80) {
                CHECK_CLOSE(lon_deg, (*res).longitude * 180 / M_PI, lon_tolerane_after80);
                CHECK_CLOSE(lon_deg, local.longitude * 180 / M_PI, lon_tolerane_after80);
            } else {
                CHECK_CLOSE(lon_deg, (*res).longitude * 180 / M_PI, lon_tolerane);
                CHECK_CLOSE(lon_deg, local.longitude * 180 / M_PI, lon_tolerane);
            }
            count++;
        }
        CHECK_EQUAL(142, count);


    }
};

constexpr char airborne_file_name[] = "resources/cpr_airborne_decode_test.txt";

typedef Cpr_decoding<Airborne_cpr_decoder, airborne_file_name> Cpr_airborne_decoding;

TEST_FIXTURE(Cpr_airborne_decoding, cpr_airborne_decoding)
{
    Run();
}

constexpr char surface_file_name[] = "resources/cpr_surface_decode_test.txt";

typedef Cpr_decoding<Surface_cpr_decoder, surface_file_name> Cpr_surface_decoding;

TEST_FIXTURE(Cpr_surface_decoding, cpr_surface_decoding)
{
    Run();
}
