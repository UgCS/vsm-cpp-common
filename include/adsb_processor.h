// Copyright (c) 2014, Smart Projects Holdings Ltd
// All rights reserved.
// See LICENSE file for license details.

/**
 * @file adsb_processor.h
 */
#ifndef _ADSB_PROCESSOR_H_
#define _ADSB_PROCESSOR_H_

#include <adsb_device.h>
#include <unordered_map>
#include <unordered_set>
#include <ugcs/vsm/request_worker.h>
#include <cpr_decoder.h>
#include <ugcs/vsm/adsb_report.h>
#include <queue>

/** ADS-B processor for reading of ADS-B data and forming easy-to use
 * position reports. Currently, only airborne aircrafts are tracked. Ground
 * surface information is ignored.
 */
class Adsb_processor: public ugcs::vsm::Request_processor {
    DEFINE_COMMON_CLASS(Adsb_processor, ugcs::vsm::Request_container)
public:

    /** Constructor. */
    Adsb_processor();

    /** Add ADS-B device to be listened by this processor. Processor sets its
     * own completion context for device. */
    void
    Add_device(Adsb_device::Ptr);

    /** Get global or create new processor instance. */
    template <typename... Args>
    static Ptr
    Get_instance(Args &&... args)
    {
        return singleton.Get_instance(std::forward<Args>(args)...);
    }

    /** Stream for receiving ADS-B reports. It is never closed by itself. */
    class Reports_stream: public std::enable_shared_from_this<Reports_stream> {
        DEFINE_COMMON_CLASS(Reports_stream, Reports_stream)

    public:

        /** Default maximum number of reports which can be pending in the
         * stream. Reports queue overflow can occur when the reading happens
         * slower than reports are generated, for example, when system is
         * overloaded.
         */
        constexpr static size_t MAX_PENDING = 32;

        /** Constructor to be used by processor only. */
        Reports_stream(Adsb_processor::Ptr);

        /** ADS-B report handler. */
        typedef ugcs::vsm::Callback_proxy<void, ugcs::vsm::Adsb_report> Report_handler;

        /** Builder for report callback. */
        DEFINE_CALLBACK_BUILDER(Make_report_callback, (ugcs::vsm::Adsb_report), (ugcs::vsm::Adsb_report()))

        /** Read next report. */
        ugcs::vsm::Operation_waiter
        Read(Report_handler, ugcs::vsm::Request_completion_context::Ptr);

        /** Close the stream. Should be called when stream is not read anymore. */
        void
        Close();

        /** Get the number of lost reports occurred due to reports queue
         * overflow. Invocation of this methods clears the counter. */
        size_t
        Get_lost_reports();

    protected:

        /** Disable the stream. */
        void
        Disable();

        /** Report read request. */
        class Read_request:public Request {
            DEFINE_COMMON_CLASS(Read_request, Request)

        public:

            /** Completion handler. */
            void
            On_completed(Report_handler);

            /** ADS-B report to be delivered to the user. */
            ugcs::vsm::Adsb_report report;
        };

        /** Process read request. */
        void
        On_read(Read_request::Ptr);

        /** Push new report to the stream. */
        void
        Push_report(const ugcs::vsm::Adsb_report&);

        /** Try to push the read queue. It is pushed only if at least one
         * read request and report is present. */
        void
        Push_read_queue();

        /** Not yet read reports. Awaiting read requests. */
        std::queue<ugcs::vsm::Adsb_report> reports;

        /** Pending read request. Awaiting reports. */
        std::queue<Read_request::Ptr> requests;

        /** Number of lost reports since last Get_lost_reports call. */
        std::atomic_size_t lost_reports = { 0 };

        /** Associated processor. */
        Adsb_processor::Weak_ptr processor;

        friend class Adsb_processor;

    };

    /** Open reports stream. */
    Reports_stream::Ptr
    Open_stream();

protected:

    /** Information about an aircraft which is, was or will be tracked. The
     * instance of this class is created as soon as first piece of information
     * about the aircraft is available and is destroyed when there no information
     * updates about this aircraft during some long enough time. It usually
     * happens when aircraft flies far away or just landed, for example. */
    class Aircraft: public std::enable_shared_from_this<Aircraft> {
        DEFINE_COMMON_CLASS(Aircraft, Aircraft)

    public:

        /** Handler for vehicle instance destruction. */
        typedef ugcs::vsm::Callback_proxy<void> Destroy_handler;

        /** Handler for ADS-B reports. */
        typedef ugcs::vsm::Callback_proxy<void, ugcs::vsm::Adsb_report> Report_handler;

        /** State of the aircraft in respect to ADS-B tracking information
         * availability in a given moment of time.
         */
        enum State {
            /** Initial state, first message received, waiting for global
             * position decode.
             */
            INITIALIZATION,
            /**
             * Global position decoding is complete, waiting for velocity.
             */
            ACQUISITION,
            /** Aircraft is being tracked in real time. */
            TRACKING,
        };

        /** Get string representation of the State. */
        static std::string
        State_to_str(State);

        /** Construct the aircraft. ICAO address is minimum required info.
         * It is present in all ADS-B messages.
         */
        Aircraft(const ugcs::vsm::Adsb_frame::ICAO_address&, Destroy_handler, Report_handler);

        /** Disable the instance. */
        void
        Disable();

        /** Process airborne position message. */
        void
        Process(const ugcs::vsm::Adsb_frame::Airborne_position_message& message,
                ugcs::vsm::Request_completion_context::Ptr& completion_ctx);

        /** Process surface position message. */
        void
        Process(const ugcs::vsm::Adsb_frame::Surface_position_message& message,
                ugcs::vsm::Request_completion_context::Ptr& completion_ctx);

        /** Process aircraft identification and category message. */
        void
        Process(const ugcs::vsm::Adsb_frame::Aircraft_id_and_cat_message& message,
                ugcs::vsm::Request_completion_context::Ptr& completion_ctx);

        /** Process airborne velocity message. */
        void
        Process(const ugcs::vsm::Adsb_frame::Airborne_velocity_message& message,
                ugcs::vsm::Request_completion_context::Ptr& completion_ctx);

        /** Get current string representation of an aircraft. */
        std::string
        To_string();

    private:

        /** ICAO address of this aircraft. */
        const ugcs::vsm::Adsb_frame::ICAO_address address;

        /** Destroy handler. */
        Destroy_handler destroy_handler;

        /** Report handler. */
        Report_handler report_handler;

        /** Even position message for global decode. */
        ugcs::vsm::Adsb_frame::Position_message even;

        /** Odd position message for global decode. */
        ugcs::vsm::Adsb_frame::Position_message odd;

        /** The most recently known position of the aircraft. */
        ugcs::vsm::Optional<ugcs::vsm::Geodetic_tuple> recent_position;

        /** Ambiguity check in acquisition state. */
        bool second_global_decode_done = false;

        /** Airborne/surface status of the aircraft. */
        ugcs::vsm::Optional<bool> airborne;

        /** Airborne CPR decoder. */
        static Airborne_cpr_decoder airborne_decoder;

        /** Surface CPR decoder. */
        static Surface_cpr_decoder surface_decoder;

        /** State::INITIALIZATION timeout. */
        static constexpr std::chrono::seconds INITIALIZATION_TIMEOUT =
                std::chrono::seconds(200);

        /** State::ACQUISITION timeout. */
        static constexpr std::chrono::seconds ACQUISITION_TIMEOUT =
                std::chrono::seconds(120);

        /** State::TRACKING timeout for airborne. */
        static constexpr std::chrono::seconds TRACKING_TIMEOUT_AIRBORNE =
                std::chrono::seconds(25);

        /** State::TRACKING timeout for surface. */
        static constexpr std::chrono::seconds TRACKING_TIMEOUT_SURFACE =
                std::chrono::seconds(200);

        /** Aircraft category. */
        ugcs::vsm::Optional<ugcs::vsm::Adsb_frame::Aircraft_id_and_cat_message::Emitter_category>
        category;

        /** Aircraft identification. */
        ugcs::vsm::Optional<std::string> ident;

        /** Heading, in radians clockwise from North. */
        ugcs::vsm::Optional<double> heading;

        /** Horizontal speed, either ground or air, in m/s. */
        ugcs::vsm::Optional<double> horizontal_speed;

        /** Vertical speed, in m/s. */
        ugcs::vsm::Optional<double> vertical_speed;

        /** Altitude, in meters. */
        ugcs::vsm::Optional<double> altitude;

        /** Current state of the aircraft. */
        State state = State::INITIALIZATION;

        /** Message receiving timer, depends on state. */
        ugcs::vsm::Timer_processor::Timer::Ptr timer;

        /** Change current state to a new state. */
        void
        Set_state(State new_state);

        /** Return true if at least one airborne velocity parameters is valid. */
        bool
        Airborne_velocity_received();

        /** Cancel existing (if any), and schedule new timer. */
        void
        Reschedule_timer(ugcs::vsm::Request_completion_context::Ptr& completion_ctx);

        /** Timer handler. Behavior depends on current state. */
        bool
        Timer_handler(ugcs::vsm::Request_completion_context::Ptr completion_ctx);

        /** Process position using appropriate decoder.
         * @return true if message was successfully processed by this aircraft,
         * otherwise false. */
        bool
        Process_position(
                const ugcs::vsm::Adsb_frame::Position_message& pos,
                const Cpr_decoder& decoder);

        /** Generate new report based on current data. */
        void
        Generate_report();

        /** Destroy the aircraft instance. */
        void
        Destroy();
    };

    /** Maximum number of simultaneously managed aircrafts. The value is
     * pretty big to suit almost every need, but still provides some
     * protection for memory overflow. It this limit is hit, it most
     * probably mean some software bug related to cleanup.
     */
    static const size_t MAX_AIRCRAFTS;

    /** Lock for structures accessed from user threads. */
    std::mutex mutex;

    /** Separate worker for this processor. */
    ugcs::vsm::Request_worker::Ptr worker;

    /** Completion context of this processor. */
    ugcs::vsm::Request_completion_context::Ptr completion_ctx;

    /** The map of aircrafts indexed by ICAO address. */
    typedef std::unordered_map<ugcs::vsm::Adsb_frame::ICAO_address,
            Aircraft::Ptr,
            ugcs::vsm::Adsb_frame::ICAO_address::Hasher> Aircrafts_map;

    /** Trivial context of an ADS-B device registered in processor. */
    class Adsb_device_ctx {
    public:

        Adsb_device_ctx() = default;

        Adsb_device_ctx(Adsb_device_ctx&&) = default;

        ~Adsb_device_ctx()
        {
            read_frame_op.Abort();
        }

        /** Current read frame operation of this device. */
        ugcs::vsm::Operation_waiter read_frame_op;

        /** Aircrafts created by this device. */
        Aircrafts_map aircrafts;
    };

    /** Context of each managed ADS-B device. The same physical
     * aircraft can be simultaneously tracked by several devices. The tracking
     * from processor point of view is independent. */
    std::unordered_map<Adsb_device::Ptr, Adsb_device_ctx> aircrafts;

    /** For testing only. Controls the reading from the device. */
    bool _initiate_reading = true;

    /** Enable the processor. */
    virtual void
    On_enable() override;

    /** Disable the processor. */
    virtual void
    On_disable() override;

    /** Process disable in the processor context. */
    void
    Process_on_disable(Request::Ptr request);

    /** Schedule next ADS-B frame reading for a given device. */
    void
    Schedule_frame_read(Adsb_device::Ptr, Adsb_device_ctx&);

    /** Raw ADS-B frames handler. */
    void
    Adsb_frame_received(ugcs::vsm::Io_buffer::Ptr, ugcs::vsm::Io_result, Adsb_device::Ptr);

    /** Lookup aircraft based on ICAO address.
     * @param device Device which received the frame.
     * @param address ICAO address to lookup.
     * @param check_limit When true, new aircraft will not be created if
     * MAX_AIRCRAFTS limit is reached.
     * @return Existing or newly created aircraft, null if check_limit is
     * true and MAX_AIRCRAFTS limit is reached.
     */
    Aircraft::Ptr
    Lookup_aircarft(
            const Adsb_device::Ptr& device,
            const ugcs::vsm::Adsb_frame::ICAO_address& address,
            bool check_limit = false);

    /** Aircraft destroy handler. */
    void
    Aircraft_destroyed(Adsb_device::Ptr, ugcs::vsm::Adsb_frame::ICAO_address);

    /** Aircraft report handler. */
    void
    Aircraft_report(ugcs::vsm::Adsb_report);

    /** Process downlink format 17 message. */
    void
    Process_DF_17(ugcs::vsm::Adsb_frame::Ptr&, Adsb_device::Ptr&);

    /** Process downlink format 18 message. */
    void
    Process_DF_18(ugcs::vsm::Adsb_frame::Ptr&, Adsb_device::Ptr&);

    /** Process downlink format 19 message. */
    void
    Process_DF_19(ugcs::vsm::Adsb_frame::Ptr&, Adsb_device::Ptr&);

    /** Process ME field for a frame which is known to contain one. */
    void
    Process_ME_field(ugcs::vsm::Adsb_frame::Ptr&, Adsb_device::Ptr&);

    template<class ME_message_type>
    void
    Process_ME_message(ugcs::vsm::Adsb_frame::Ptr& frame, Adsb_device::Ptr& device)
    {
        ME_message_type msg(frame);
        auto aircraft = Lookup_aircarft(device, msg.Get_ICAO_address(), true);
        if (!aircraft) {
            return;
        }
        aircraft->Process(msg, completion_ctx);
    }

    /** Process airborne velocity message. */
    void
    Process_airborne_velocity_message(ugcs::vsm::Adsb_frame::Ptr&, Adsb_device::Ptr&);

    /** New stream opened in processor context. */
    void
    On_stream_open(Reports_stream::Ptr, Request::Ptr);

    /** Close the stream. */
    void
    On_stream_close(Reports_stream::Ptr, Request::Ptr);

    /** Currently opened report streams. */
    std::unordered_set<Reports_stream::Ptr> streams;

    /** ADS-B processor singleton instance. */
    static ugcs::vsm::Singleton<Adsb_processor> singleton;
};

#endif /* ADSB_PROCESSOR_H_ */
