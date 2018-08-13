// Copyright (c) 2018, Smart Projects Holdings Ltd
// All rights reserved.
// See LICENSE file for license details.

#include <red_button_processor.h>

constexpr std::chrono::milliseconds Red_button_processor::POLL_INTERVAL;
constexpr std::chrono::seconds Red_button_processor::CONNECT_DELAY;

Red_button_processor::Red_button_processor() :
        ugcs::vsm::Request_processor("Red button processor")
{
    comp_ctx = ugcs::vsm::Request_completion_context::Create(
            "Red button completion",
            Get_waiter());

    /* Create fixed hardware requests. */
    uint8_t reset_buf[8] = {3, 2, 0, 0, 0, 0, 0, 0};
    uint8_t poll_buf[8] = {0, 0, 0, 0, 0, 0, 0, 2};
    reset_request = ugcs::vsm::Io_buffer::Create(reset_buf, sizeof(reset_buf));
    poll_request = ugcs::vsm::Io_buffer::Create(poll_buf, sizeof(poll_buf));
}

void
Red_button_processor::On_enable()
{
    ugcs::vsm::Request_processor::On_enable();
    comp_ctx->Enable();
    worker = ugcs::vsm::Request_worker::Create(
        "Red button worker",
        std::initializer_list<Request_container::Ptr>{Shared_from_this(), comp_ctx});
    worker->Enable();

    if (Delayed_connect()) {
        Schedule_connect();
    }
}

void
Red_button_processor::On_disable()
{
    worker->Disable();
    worker = nullptr;
    comp_ctx->Disable();

    for (Listener &listener: listeners) {
        listener.req->Abort();
    }
    listeners.clear();

    ugcs::vsm::Request_processor::On_disable();
}

ugcs::vsm::Operation_waiter
Red_button_processor::Listen(Event_handler handler,
                                   ugcs::vsm::Request_completion_context::Ptr ctx)
{
    ugcs::vsm::Request::Ptr req = Create_listen_request(handler, ctx);
    Submit_request(req);
    return req;
}

ugcs::vsm::Request::Ptr
Red_button_processor::Create_listen_request(Event_handler handler,
                                            ugcs::vsm::Request_completion_context::Ptr ctx,
                                            Event_id next_event)
{
    if (!ctx) {
        ctx = comp_ctx;
    }
    ugcs::vsm::Request::Ptr req = ugcs::vsm::Request::Create();
    auto comp_handler =
        ugcs::vsm::Make_callback(&Red_button_processor::Handle_event, this,
                           handler, ctx, next_event);
    req->Set_completion_handler(ctx, comp_handler);
    req->Set_processing_handler(
        ugcs::vsm::Make_callback(&Red_button_processor::Handle_listen_request, this,
                           req, handler, &comp_handler->template Get_arg<2>()));
    return req;
}

void
Red_button_processor::Read()
{
    rb_stream->Read(REPORT_SIZE, REPORT_SIZE,
                    ugcs::vsm::Make_read_callback(&Red_button_processor::Read_handler, this),
                    comp_ctx);
}

void
Red_button_processor::Schedule_connect()
{
    if (connect_timer) {
        return;
    }
    connect_timer = ugcs::vsm::Timer_processor::Get_instance()->Create_timer
        (CONNECT_DELAY, Make_callback(&Red_button_processor::Delayed_connect, this),
         comp_ctx);
}

void
Red_button_processor::Start_polling()
{
    if (poll_timer) {
        return;
    }
    poll_timer = ugcs::vsm::Timer_processor::Get_instance()->Create_timer
        (POLL_INTERVAL, Make_callback(&Red_button_processor::Poll_timer, this),
         comp_ctx);
}

void
Red_button_processor::Connect()
{
    try {
        rb_stream = ugcs::vsm::Hid_processor::Get_instance()->Open(VENDOR_ID, PRODUCT_ID);
        LOG("Red button connected");
        Reset();
        Read();
        Start_polling();
    } catch (ugcs::vsm::Hid_processor::Exception &) {
        rb_stream = nullptr;
    }
}

bool
Red_button_processor::Delayed_connect()
{
    Connect();
    if (rb_stream) {
        connect_timer = nullptr;
        return false;
    }
    return true;
}

void
Red_button_processor::Disconnect()
{
    if (!rb_stream) {
        return;
    }
    rb_stream->Close();
    rb_stream = nullptr;
    cur_state = State_code::INITIAL;
    LOG("Red button disconnected");
}

void
Red_button_processor::Reset()
{
    try {
        rb_stream->Set_output_report(reset_request);
    } catch (ugcs::vsm::Hid_processor::Invalid_state_exception &) {
        Disconnect();
    }
}

void
Red_button_processor::Poll()
{
    try {
        rb_stream->Set_output_report(poll_request);
    } catch (ugcs::vsm::Hid_processor::Invalid_state_exception &) {
        Disconnect();
    }
}

void
Red_button_processor::Fire_event_handler(Event_id event_id, ugcs::vsm::Request::Ptr req,
                                         Event_handler handler)
{
    Event &event = event_queue[event_id & (EVENT_QUEUE_SIZE - 1)];
    ugcs::vsm::Io_result result =
        event.state == State_code::INITIAL ? ugcs::vsm::Io_result::CLOSED : ugcs::vsm::Io_result::OK;
    bool pressed = event.state == State_code::PRESSED;

    auto lock = req->Lock();
    if (req->Is_processing()) {
        handler.Set_args(pressed, result);
        req->Complete(ugcs::vsm::Request::Status::OK, std::move(lock));
    }
}

void
Red_button_processor::Handle_listen_request(ugcs::vsm::Request::Ptr req,
                                            Event_handler handler,
                                            Event_id *next_event)
{
    if (*next_event < event_first) {
        /* Some events will be discarded. */
        *next_event = event_first;
    }

    if (*next_event == EVENT_ID_NONE ||
        *next_event >= event_last) {

        /* Wait for future events requested. Place in the list. */
        *next_event = event_last + 1;
        listeners.emplace_back(req, handler);
        return;
    }

    (*next_event)++;
    Fire_event_handler((*next_event) - 1, req, handler);
}

void
Red_button_processor::Handle_event(Event_handler handler,
                                   ugcs::vsm::Request_completion_context::Ptr ctx,
                                   Event_id last_event)
{
    if (handler.Invoke()) {
        ugcs::vsm::Request::Ptr req = Create_listen_request(handler, ctx, last_event);
        Submit_request(req);
    }
}

void
Red_button_processor::Fire_handlers()
{
    for (Listener &l: listeners) {
        Fire_event_handler(event_last - 1, l.req, l.handler);
    }
    listeners.clear();
}

void
Red_button_processor::Add_event(State_code code)
{
    Event &event = event_queue[event_last & (EVENT_QUEUE_SIZE - 1)];
    event.state = code;
    event_last++;
    if (event_first < event_last - EVENT_QUEUE_SIZE) {
        event_first = event_last - EVENT_QUEUE_SIZE;
    }
}

void
Red_button_processor::Read_handler(ugcs::vsm::Io_buffer::Ptr data, ugcs::vsm::Io_result result)
{
    Event_id cur_event_id = event_last;

    if (result == ugcs::vsm::Io_result::OK) {
        State_code state =
            static_cast<State_code>(reinterpret_cast<const uint8_t *>(data->Get_data())[1]);
        if (state != cur_state) {
            cur_state = state;

            Add_event(cur_state);
        }
        Read();
    } else {
        Disconnect();
        Add_event(State_code::INITIAL);
    }

    if (event_last != cur_event_id) {
        Fire_handlers();
    }
}

bool
Red_button_processor::Poll_timer()
{
    if (!rb_stream) {
        Schedule_connect();
        poll_timer = nullptr;
        return false;
    }
    Poll();
    if (!rb_stream) {
        Schedule_connect();
        poll_timer = nullptr;
        return false;
    }
    return true;
}
