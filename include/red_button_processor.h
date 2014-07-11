// Copyright (c) 2014, Smart Projects Holdings Ltd
// All rights reserved.
// See LICENSE file for license details.

/*
 * Processor for Dream Cheeky Big Red Button hardware.
 */

#ifndef RED_BUTTON_PROCESSOR_H_
#define RED_BUTTON_PROCESSOR_H_

#include <ugcs/vsm/vsm.h>

class Red_button_processor: public ugcs::vsm::Request_processor {
    DEFINE_COMMON_CLASS(Red_button_processor, Request_container)
public:
    /** Button event handler.
     * @param pressed Indicates whether the button was pressed or released.
     * @param result Is either Io_result::OK or Io_result::CLOSED if the button
     *      was disconnected.
     * @return true to continue events reception, false to remove registered
     *      handler from the processor.
     */
    typedef ugcs::vsm::Callback_proxy<bool, bool, ugcs::vsm::Io_result> Event_handler;

    /** Builder for event handler. */
    DEFINE_CALLBACK_BUILDER(
            Make_event_handler,
            (bool, ugcs::vsm::Io_result),
            (false, ugcs::vsm::Io_result::OK))

    Red_button_processor();

    /** Listen for button state change event. The event is sent when button is
     * pressed, released or disconnected.
     *
     * @param handler Event handler.
     * @param comp_ctx Context for handler invocation. nullptr indicates that
     *      the processor completion context will be used.
     */
    ugcs::vsm::Operation_waiter
    Listen(Event_handler handler,
           ugcs::vsm::Request_completion_context::Ptr ctx = nullptr);

    bool
    Is_connected() const
    {
        return static_cast<bool>(rb_stream);
    }

    bool
    Is_pressed() const
    {
        return cur_state == State_code::PRESSED;
    }

private:
    /** Event ID for the queued events. */
    typedef size_t Event_id;

    static constexpr uint32_t VENDOR_ID = 0x1d34;
    static constexpr uint32_t PRODUCT_ID = 0x000d;
    static constexpr size_t REPORT_SIZE = 9;
    /** Size of event queue, should be power of two. */
    static constexpr size_t EVENT_QUEUE_SIZE = 32;
    /** Special value for indication of no reference to an event. */
    static constexpr Event_id EVENT_ID_NONE = SIZE_MAX;
    static constexpr std::chrono::milliseconds POLL_INTERVAL =
        std::chrono::milliseconds(100);
    static constexpr std::chrono::seconds CONNECT_DELAY =
        std::chrono::seconds(2);

    /** State codes reported by the device. */
    enum class State_code {
        INITIAL = 0x15,
        PRESSED = 0x16,
        RELEASED = 0x17
    };

    ugcs::vsm::Request_completion_context::Ptr comp_ctx;
    ugcs::vsm::Request_worker::Ptr worker;

    /** Button device stream. */
    ugcs::vsm::Hid_processor::Stream::Ref rb_stream;
    ugcs::vsm::Timer_processor::Timer::Ptr poll_timer, connect_timer;
    /** Current button state. */
    volatile State_code cur_state = State_code::INITIAL;

    /** Represents registered listener. */
    struct Listener {
        ugcs::vsm::Request::Ptr req;
        Event_handler handler;

        Listener(ugcs::vsm::Request::Ptr req, Event_handler handler):
            req(req), handler(handler)
        {}
    };

    struct Event {
        /** Button state. INITIAL indicates device disconnection. */
        State_code state;
    };

    Event event_queue[EVENT_QUEUE_SIZE];
    /** Index of the first and one-past-last event in the queue. They are mapped
     * as modulo of queue size to the physical slots.
     */
    Event_id event_first = 0, event_last = 0;

    std::list<Listener> listeners;

    /** Fixed reset and poll device requests. */
    ugcs::vsm::Io_buffer::Ptr reset_request, poll_request;

    /** Handle processor enabling. */
    virtual void
    On_enable() override;

    /** Handle disabling request. */
    virtual void
    On_disable() override;

    /** Create request for the events starting from the specified next
     * event (which can be EVENT_ID_NONE to process only new events).
     */
    ugcs::vsm::Request::Ptr
    Create_listen_request(Event_handler handler,
                          ugcs::vsm::Request_completion_context::Ptr ctx,
                          Event_id next_event = EVENT_ID_NONE);

    /** @param next_event Points to next required event ID. May be EVENT_ID_NONE
     * to indicate interest in the new events. Should be updated with new value
     * before the request is completed.
     */
    void
    Handle_listen_request(ugcs::vsm::Request::Ptr req, Event_handler handler,
                          Event_id *next_event);

    void
    Handle_event(Event_handler handler,
                 ugcs::vsm::Request_completion_context::Ptr ctx,
                 Event_id next_event);

    void
    Read_handler(ugcs::vsm::Io_buffer::Ptr data, ugcs::vsm::Io_result result);

    bool
    Poll_timer();

    /** Try to create button connection. rb_stream is opened if succeeded. */
    void
    Connect();

    /** Called by timer. Should return true to reschedule connection attempt. */
    bool
    Delayed_connect();

    /** Handle device disconnection. */
    void
    Disconnect();

    /** Schedule device read request. */
    void
    Read();

    /** Send reset request to the device. */
    void
    Reset();

    /** Send poll request to the device. */
    void
    Poll();

    /** Schedule connection attempt. */
    void
    Schedule_connect();

    /** Enable device polling. */
    void
    Start_polling();

    void
    Fire_event_handler(Event_id event_id, ugcs::vsm::Request::Ptr req,
                       Event_handler handler);

    /** Fire all queued handlers for the last event. */
    void
    Fire_handlers();

    void
    Add_event(State_code code);
};

#endif /* RED_BUTTON_PROCESSOR_H_ */
