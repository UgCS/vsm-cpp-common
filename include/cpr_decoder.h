// Copyright (c) 2014, Smart Projects Holdings Ltd
// All rights reserved.
// See LICENSE file for license details.

/**
 * @file cpr_decoder.h
 */
#ifndef _CPR_DECODER_H_
#define _CPR_DECODER_H_

#include <ugcs/vsm/adsb_frame.h>
#include <ugcs/vsm/optional.h>
#include <ugcs/vsm/coordinates.h>

namespace internal {

/** Parameters for airborne messages CPR decoding for surface participant. */
struct Airborne_params {

    /** Maximum interval between even and odd position messages allowed to
     * perform the global decode.
     */
    static constexpr std::chrono::seconds GLOBAL_DECODE_INTERVAL =
            std::chrono::seconds(10);

    /** Maximum distance in meters (6 NM per official docs) between two
     * locally decoded airborne positions.
     */
    static constexpr double MAX_LOCAL_DISTANCE = 6000 * 1.852;

    /** Globally and locally decoded position should be equal within this
     * range, in meters. */
    static constexpr double MAX_GLOBAL_LOCAL_DIFFERENCE = 5;

    /** Denominator value used in calculations. */
    static constexpr double DENOM = 360;

    /** Indicator, whether these parameters are surface or airborne. */
    static constexpr bool IS_SURFACE = false;

    /** The approximate position of the receiver (antenna). Should be
     * accurate within some 100Km precision. Used only for surface position
     * decoding. If absent, surface decoding will always fail.
     */
    ugcs::vsm::Optional<ugcs::vsm::Geodetic_tuple> receiver_position;
};

// @{
/** Parameters for surface messages CPR decoding for surface participant.
 * See @ref internal::Airborne_params for comments. */
struct Surface_params {
    static constexpr std::chrono::seconds GLOBAL_DECODE_INTERVAL =
                std::chrono::seconds(25);

    /** Maximum distance in meters (2.5 NM per official docs) between two
     * locally decoded surface positions.
     */
    static constexpr double MAX_LOCAL_DISTANCE = 2500 * 1.852;

    static constexpr double MAX_GLOBAL_LOCAL_DIFFERENCE = 1.25;

    static constexpr double DENOM = 90;

    static constexpr bool IS_SURFACE = true;

    ugcs::vsm::Optional<ugcs::vsm::Geodetic_tuple> receiver_position;
};
// @}

} /* namespace internal */

/** Compact Position Reporting format decoder. */
class Cpr_decoder {
public:

    /** Globally unambiguous CPR decoding based on even and odd frames.
     * @param frame1 Even/odd frame.
     * @param frame2 Odd/even frame.
     * @return Non empty position with zero altitude on successful decoding,
     * otherwise empty optional instance.
     */
    virtual
    ugcs::vsm::Optional<ugcs::vsm::Geodetic_tuple>
    Global_decode(
            const ugcs::vsm::Adsb_frame::Position_message& frame1,
            const ugcs::vsm::Adsb_frame::Position_message& frame2) const = 0;

    /**
     * Locally unambiguous CPR decoding.
     * @param ref Reference position.
     * @param frame Recently received frame with new position.
     * @return Decoded position.
     */
    virtual
    ugcs::vsm::Geodetic_tuple
    Local_decode(
            const ugcs::vsm::Geodetic_tuple& ref,
            const ugcs::vsm::Adsb_frame::Position_message& frame) const = 0;

    /** Set the position of the receiver to be able to decode surface positions. */
    virtual void
    Set_receiver_position(const ugcs::vsm::Geodetic_tuple& pos) = 0;

    /** Check the distance between given positions to be less that maximum
     * distance after local decoding. Reasonableness test.
     * @return true if distance is less than maximum, otherwise false.
     */
    virtual bool
    Check_local_distance(const ugcs::vsm::Geodetic_tuple& p1, const ugcs::vsm::Geodetic_tuple& p2) const = 0;

    /** Check the distance between given position and the location of the
     * receiver, if known.
     * @param p The position to check.
     * @param distance Distance is return, if known.
     * @return true if distance between given position and receiver is reasonable
     * or receiver position is not known, otherwise false.
     */
    virtual bool
    Check_receiver_distance(
            const ugcs::vsm::Geodetic_tuple& p,
            ugcs::vsm::Optional<double>& distance) const = 0;

    /** Checks whether given distance between globally and locally decoded
     * positions is within reasonable range.
     */
    virtual bool
    Check_global_local_distance(double distance) const = 0;

    /** Virtual destructor. */
    virtual
    ~Cpr_decoder() {};

protected:

    /** Number of zones on the globe. */
    static constexpr double NZ = 15;

    /** Maximum reasonable receiving difference via latitude or longitude axis,
     * in degrees. Yields to some 2.000 Km. Needed only for surface decoding
     * ambiguity resolution, because there are multiple solutions spread by
     * multiples of 90 degrees of latitude or longitude. Solution closer
     * than this distance to the receiver position is chosen. */
    static constexpr double MAX_RECEIVING_DISTANCE = 90.0 / 5;

    /** Maximum receiving distance of the antenna (1000Km) in meters. Should be
     * enough even for a very good receiver. Typical range is around 200Km.
     */
    static constexpr double MAX_ANTENNA_RANGE = 1000 * 1000;


    /** 17-th power of 2 constant. Related to the 17-bit size of the lat/lon
     * fields of the ADS-B position messages. */
    static constexpr double N2x17 = 131072;

    /** Number of longitude zones of the latitude angle. */
    int
    NL(double latitude) const
    {

        /* Copy-paste logic from the official doc. */
        if (latitude == 0) {
            return 59;
        } else if (std::abs(latitude) == 87) {
            return 2;
        } else if (std::abs(latitude) > 87) {
            return 1;
        } else {
            double numerator = 1 - std::cos(M_PI / (2 * NZ));
            double denominator = std::cos((M_PI / 180) * std::abs(latitude));
            denominator = denominator * denominator;
            double fraction = 1 - numerator / denominator;
            return std::floor(2 * M_PI * (1 / std::acos(fraction)));
        }
    }

    /** Non-negative modulus. */
    double
    Mod(double x, double y) const
    {
        return x - y * std::floor(x / y);
    }
	
	/** Normalize latitude value of global decode.
	 * @return false if normalization is not possible,
	 * true otherwise.
	 */
	bool
	Normalize_global_decode_lat(double& lat) const
	{
	    double lat_abs = std::abs(lat);
		/*
         * Southern hemisphere values of latitude will fall in the range from 270° to
         * 360°. Subtract 360 from such values, thereby restoring latitude to the range
         * from -90 to +90.
		 * Allow 1 degree calculation error close to the poles. This is due to the fact,
		 * that calculations could give a value which is just slightly more than 90 and
		 * should be trimmed.
         */
		if (lat_abs > 91) {
		    lat -= 360;
			lat_abs = std::abs(lat);
		}
		if (lat_abs > 91) {
		    /* Still unreasonable. */
		    return false;
		}
		if (lat_abs > 90) {
		    lat = std::copysign(90, lat);
		}
		return true;
	}

};

/** Specialization for airborne and surface message types. */
template<class Params>
class Cpr_decoder_spec:public Cpr_decoder {
public:

    /** Globally unambiguous CPR decoding based on even and odd frames.
     * @param frame1 Even/odd frame.
     * @param frame2 Odd/even frame.
     * @return Non empty position with zero altitude on successful decoding,
     * otherwise empty optional instance.
     */
    virtual
    ugcs::vsm::Optional<ugcs::vsm::Geodetic_tuple>
    Global_decode(
            const ugcs::vsm::Adsb_frame::Position_message& frame1,
            const ugcs::vsm::Adsb_frame::Position_message& frame2) const override
    {
        constexpr double Dlat0 = Params::DENOM / (4 * NZ - 0);
        constexpr double Dlat1 = Params::DENOM / (4 * NZ - 1);
        bool recent_parity;
        const ugcs::vsm::Adsb_frame::Position_message* even;
        const ugcs::vsm::Adsb_frame::Position_message* odd;

        ASSERT(frame1.Get_CPR_format() != frame2.Get_CPR_format());

        std::chrono::seconds diff;

        if (frame1.Get_received_time() > frame2.Get_received_time()) {
            recent_parity = frame1.Get_CPR_format();
            diff = std::chrono::duration_cast<std::chrono::seconds>(
                    frame1.Get_received_time() - frame2.Get_received_time());
        } else {
            recent_parity = frame2.Get_CPR_format();
            diff = std::chrono::duration_cast<std::chrono::seconds>(
                    frame2.Get_received_time() - frame1.Get_received_time());
        }

        if (diff > Params::GLOBAL_DECODE_INTERVAL) {
            return ugcs::vsm::Optional<ugcs::vsm::Geodetic_tuple>();
        }

        if (frame1.Get_CPR_format()) {
            odd = &frame1;
            even = &frame2;
        } else {
            odd = &frame2;
            even = &frame1;
        }

        /* Latitude index. */
        int j = std::floor((59.0 * even->Get_CPR_latitude() -
                60.0 * odd->Get_CPR_latitude()) / N2x17 + 0.5);
        double Rlat0 = Dlat0 * (Mod(j, 60 - 0) + even->Get_CPR_latitude() / N2x17);
        double Rlat1 = Dlat1 * (Mod(j, 60 - 1) + odd->Get_CPR_latitude() / N2x17);

        if (params.IS_SURFACE) {
            /* Surface position decode needs receiver position to be known. */
            if (!params.receiver_position) {
                return ugcs::vsm::Optional<ugcs::vsm::Geodetic_tuple>();
            } else {
                double receiver_lat =
                        ((*params.receiver_position).latitude * 180.0) / M_PI;
                bool found = false;
                for (auto d: {0, 90, -90}) {
                    if (std::abs(Rlat0 + d - receiver_lat) < MAX_RECEIVING_DISTANCE) {
                        Rlat0 += d;
                        Rlat1 += d;
                        found = true;
                        break;
                    }
                }
                if (!found) {
                    /* Still bad? */
                    return ugcs::vsm::Optional<ugcs::vsm::Geodetic_tuple>();
                }
            }
        } else {
			if (!Normalize_global_decode_lat(Rlat0) ||
			    !Normalize_global_decode_lat(Rlat1)) {
			    return ugcs::vsm::Optional<ugcs::vsm::Geodetic_tuple>();
            }
        }
        double NLRlat0 = NL(Rlat0);
        double NLRlat1 = NL(Rlat1);

        if (NLRlat0 != NLRlat1) {
            /* Aircraft crossing zone boundary. Unable to do global decode, wait for
             * next positions.
             */
            return ugcs::vsm::Optional<ugcs::vsm::Geodetic_tuple>();
        }

        double n = recent_parity ? NLRlat1 - 1 : NLRlat0;
        n = n > 1 ? n : 1;
        double Dlon = Params::DENOM / n;

        double NL_ = recent_parity ? NLRlat1 : NLRlat0;

        /* Longitude index. */
        double m = std::floor(
                (even->Get_CPR_longitude() * (NL_ - 1) - odd->Get_CPR_longitude() * NL_) /
                N2x17 + 0.5);

        double Rlon = Dlon * (Mod(m, n) +
                (recent_parity ? odd->Get_CPR_longitude() : even->Get_CPR_longitude()) / N2x17);
        if (params.IS_SURFACE) {
            double receiver_lon =
                    ((*params.receiver_position).longitude * 180.0) / M_PI;
            bool found = false;
            for (auto d: {0, 90, -90, 180, -180, 270, -270}) {
                if (std::abs(Rlon + d - receiver_lon) < MAX_RECEIVING_DISTANCE) {
                    Rlon += d;
                    found = true;
                    break;
                }
            }
            if (!found) {
                /* Still bad? */
                return ugcs::vsm::Optional<ugcs::vsm::Geodetic_tuple>();
            }
        } else {
            if (Rlon >= 180) {
                Rlon -= 360;
            }
        }
        return ugcs::vsm::Optional<ugcs::vsm::Geodetic_tuple>(ugcs::vsm::Geodetic_tuple(
                (recent_parity ? Rlat1 : Rlat0) * M_PI / 180.0,
                Rlon * M_PI / 180.0, 0));
    }

    /**
     * Locally unambiguous CPR decoding.
     * @param ref Reference position.
     * @param frame Recently received frame with new position.
     * @return Decoded position.
     */
    virtual
    ugcs::vsm::Geodetic_tuple
    Local_decode(
            const ugcs::vsm::Geodetic_tuple& ref,
            const ugcs::vsm::Adsb_frame::Position_message& frame) const override
    {
        double ref_latitude = ref.latitude * 180 / M_PI;
        double ref_longitude = ref.longitude * 180 / M_PI;
        int i = frame.Get_CPR_format() ? 1 : 0;
        double Dlat = Params::DENOM / (4 * NZ - i);

        /* Latitude zone index. */
        int j = std::floor(ref_latitude / Dlat) +
                std::floor(0.5 + (Mod(ref_latitude, Dlat)/Dlat) -
                        frame.Get_CPR_latitude() / N2x17);
        double Rlat = Dlat * (j + frame.Get_CPR_latitude() / N2x17);

        double Dlon;
        double NLRlat = NL(Rlat);
        if (NLRlat - i > 0) {
            Dlon = Params::DENOM / (NLRlat - i);
        } else {
            Dlon = Params::DENOM;
        }

        /* Longitude zone. */
        int m = std::floor(ref_longitude / Dlon) +
                std::floor(0.5 + Mod(ref_longitude, Dlon) / Dlon -
                        frame.Get_CPR_longitude() / N2x17);

        double Rlon = Dlon * (m + frame.Get_CPR_longitude() / N2x17);

        return ugcs::vsm::Geodetic_tuple(Rlat * M_PI / 180.0, Rlon * M_PI / 180.0, 0);
    }

    virtual void
    Set_receiver_position(const ugcs::vsm::Geodetic_tuple& pos) override
    {
        params.receiver_position = pos;
    }

    virtual bool
    Check_local_distance(
            const ugcs::vsm::Geodetic_tuple& p1,
            const ugcs::vsm::Geodetic_tuple& p2) const override
    {
        auto distance = ugcs::vsm::Wgs84_position(p1).Distance(p2);
        return distance < Params::MAX_LOCAL_DISTANCE;
    }

    virtual bool
    Check_receiver_distance(
            const ugcs::vsm::Geodetic_tuple& p,
            ugcs::vsm::Optional<double>& distance) const override
    {
        distance.Disengage();
        if (!params.receiver_position) {
            return true;
        }
        distance = ugcs::vsm::Wgs84_position(p).Distance(*params.receiver_position);
        return *distance < MAX_ANTENNA_RANGE;
    }

    virtual bool
    Check_global_local_distance(double distance) const override
    {
        return distance < Params::MAX_GLOBAL_LOCAL_DIFFERENCE;
    }

protected:

    /** Decoder parameters. */
    Params params;
};

/** CPR airborne decoder. */
typedef Cpr_decoder_spec<internal::Airborne_params> Airborne_cpr_decoder;

/** CPR surface decoder. */
typedef Cpr_decoder_spec<internal::Surface_params> Surface_cpr_decoder;

#endif /* _CPR_DECODER_H_ */
