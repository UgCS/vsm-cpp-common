// Copyright (c) 2014, Smart Projects Holdings Ltd
// All rights reserved.
// See LICENSE file for license details.

/** Tests for ADS-B frame encoded values. */
#include <vsm/adsb_frame.h>
#include <fstream>
#include <UnitTest++.h>

using namespace vsm;

#define FILL_GILLHAM_CODE(                                                       \
    A1_, A2_, A4_, B1_, B2_, B4_, C1_, C2_, C4_, D1_, D2_, D4_, frame_)          \
    /* Set most valuable bits and also Q bit to zero to indicate Gillham
     * code format. */                                                           \
    frame_.ME[1] =                                                               \
    C1_ << 7 | A1_ << 6 | C2_ << 5 | A2_ << 4 | C4_ << 3 | A4_ << 2 | B1_ << 1;  \
    /* Clear 4 least significant bits of Gillham code. They are following
     * previous byte, so bits are located in _most_ significant part of the
     * native byte value.
     */                                                                          \
    frame_.ME[2] &= 0x0f;                                                        \
    frame_.ME[2] |= B2_ << 7 | D2_ << 6 | B4_ << 5 | D4_ << 4;

int N_to_altitude(int N)
{
    return N * 25 - 1000;
}


TEST(altitude_gillham_encoding)
{
    Adsb_frame::Frame frame;
    memset(&frame, 0, sizeof(frame));

    /* File taken from http://www.airsport-corp.com/modecascii.txt */
    std::fstream file("resources/gillham_altitude_codes.txt", std::ios_base::in);
    if (!file.is_open()) {
        throw "Can not open Gillham codes file!";
    }

    int count = 0;
    while (!file.eof()) {
        int altitude, A1, A2, A4, B1, B2, B4, C1, C2, C4, D1, D2, D4, squawk;
        file >> altitude >>
        A1 >> A2 >> A4 >> B1 >> B2 >> B4 >> C1 >> C2 >> C4 >> D1 >> D2 >> D4 >>
        squawk;
        count++;
        FILL_GILLHAM_CODE(A1, A2, A4, B1, B2, B4, C1, C2, C4, D1, D2, D4, frame);
        Adsb_frame::Ptr adsb_frame =
                Adsb_frame::Create(Io_buffer::Create(&frame, sizeof(frame)));
        Adsb_frame::Airborne_position_message aiborne_pos(adsb_frame);

        CHECK_EQUAL(altitude, aiborne_pos.Get_altitude(true));
    }
    CHECK_EQUAL(1280, count);
}

TEST(altitude_binary_encoding)
{
    Adsb_frame::Frame frame;
    memset(&frame, 0, sizeof(frame));
    /* 4 lowest bits. */
    for (int low4 = 0; low4 <= 0xf; low4++) {
        /* 7 highest bits. */
        for (int high7 = 0; high7 <= 0x7f; high7++) {
            frame.ME[1] = 0x01; /* Q bit set means binary encoding. */
            frame.ME[1] |= high7 << 1;
            frame.ME[2] = low4 << 4; /* Following previous byte. */
            Adsb_frame::Ptr adsb_frame =
                    Adsb_frame::Create(Io_buffer::Create(&frame, sizeof(frame)));
            Adsb_frame::Airborne_position_message aiborne_pos(adsb_frame);
            CHECK_EQUAL(N_to_altitude(low4 + high7 * 16), aiborne_pos.Get_altitude(true));
        }
    }
}
