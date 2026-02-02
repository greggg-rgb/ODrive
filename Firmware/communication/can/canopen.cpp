#include "canopen.hpp"

#include <lely/co/rpdo.h>
#include <lely/co/sdev.h>
#include <lely/co/time.h>
#include <lely/co/tpdo.h>
#include <odrive_main.h>
#include <string.h>

extern "C" {
extern const struct co_sdev odrive_co;
}

static void on_time(co_time_t* time, const struct timespec* tp, void* data) {
    (void)time;
    (void)data;

    // Update the wall clock, _not_ the monotonic clock used by the CAN
    // network.
    clock_settime(CLOCK_REALTIME, tp);
}

// CAN send callback
int CANopen::can_send_func(const struct can_msg* msg, void* data) {
    CANopen* self = get_self(data);

    // Convert Lely CAN message to ODrive format
    can_Message_t odrive_msg;
    odrive_msg.id = msg->id;
    odrive_msg.isExt = (msg->flags & CAN_FLAG_IDE) != 0;
    odrive_msg.rtr = (msg->flags & CAN_FLAG_RTR) != 0;
    odrive_msg.len = msg->len;
    memcpy(odrive_msg.buf, msg->data, msg->len);

    return self->canbus_->send_message(odrive_msg) ? 0 : -1;
}

// Process incoming CAN message
void CANopen::on_can_message(const can_Message_t& msg) {
    if (!initialized_) return;

    // Convert to Lely format
    struct can_msg lely_msg;
    lely_msg.id = msg.id;
    lely_msg.flags = 0;
    if (msg.isExt) lely_msg.flags |= CAN_FLAG_IDE;
    if (msg.rtr) lely_msg.flags |= CAN_FLAG_RTR;
    lely_msg.len = msg.len;
    memcpy(lely_msg.data, msg.buf, msg.len);

    // Pass to CAN network
    can_net_recv(net_, &lely_msg);
}

void CANopen::shutdown() {
    if (!initialized_) return;

    if (nmt_) {
        co_nmt_destroy(nmt_);
        nmt_ = nullptr;
    }

    if (dev_) {
        co_dev_destroy(dev_);
        dev_ = nullptr;
    }

    if (net_) {
        can_net_destroy(net_);
        net_ = nullptr;
    }

    initialized_ = false;
}

// Initialize CANopen stack
bool CANopen::init() {
    if (initialized_) {
        return false;
    }
    Axis& axis = axes[0];
    node_id_ = axis.config_.can.node_id;

    // Initialize CAN network interface
    net_ = can_net_create();
    if (!net_) {
        return false;
    }
    can_net_set_send_func(net_, &can_send_func, this);

    // Initialize clock
    clock_gettime(CLOCK_MONOTONIC, &last_time_);
    can_net_set_time(net_, &last_time_);

    // Create device from static object dictionary
    dev_ = co_dev_create_from_sdev(&odrive_co);
    if (!dev_) {
        can_net_destroy(net_);
        return false;
    }

    // Set node ID
    co_dev_set_id(dev_, node_id_);

    // Create NMT service
    nmt_ = co_nmt_create(net_, dev_);
    if (!nmt_) {
        co_dev_destroy(dev_);
        can_net_destroy(net_);
        return false;
    }
    // Start NMT service (reset node)
    co_nmt_cs_ind(nmt_, CO_NMT_CS_RESET_NODE);

    // Set NMT callback
    co_nmt_set_cs_ind(nmt_, &nmt_cs_ind, this);

    // Setup SDO download + RPDO callbacks (writes from master)
    co_sub_t* sub;

    // 0x6040: Controlword
    sub = co_dev_find_sub(dev_, 0x6040, 0x00);
    if (sub) co_sub_set_dn_ind(sub, &sdo_dn_6040, this);

    // 0x6060: Modes of operation
    sub = co_dev_find_sub(dev_, 0x6060, 0x00);
    if (sub) co_sub_set_dn_ind(sub, &sdo_dn_6060, this);

    // 0x607A: Target position
    sub = co_dev_find_sub(dev_, 0x607A, 0x00);
    if (sub) co_sub_set_dn_ind(sub, &sdo_dn_607A, this);

    // 0x60FF: Target velocity
    sub = co_dev_find_sub(dev_, 0x60FF, 0x00);
    if (sub) co_sub_set_dn_ind(sub, &sdo_dn_60FF, this);

    // 0x6071: Target torque
    sub = co_dev_find_sub(dev_, 0x6071, 0x00);
    if (sub) co_sub_set_dn_ind(sub, &sdo_dn_6071, this);

    // 0x6072: Max torque
    sub = co_dev_find_sub(dev_, 0x6072, 0x00);
    if (sub) co_sub_set_dn_ind(sub, &sdo_dn_6072, this);

    // 0x6081: Profile velocity
    sub = co_dev_find_sub(dev_, 0x6081, 0x00);
    if (sub) co_sub_set_dn_ind(sub, &sdo_dn_6081, this);

    // 0x6083: Profile accel
    sub = co_dev_find_sub(dev_, 0x6083, 0x00);
    if (sub) co_sub_set_dn_ind(sub, &sdo_dn_6083, this);

    // 0x6083: Profile decel
    sub = co_dev_find_sub(dev_, 0x6084, 0x00);
    if (sub) co_sub_set_dn_ind(sub, &sdo_dn_6084, this);

    // Setup SDO upload + TPDO callbacks (reads from master)
    // 0x603F: Error code
    sub = co_dev_find_sub(dev_, 0x603F, 0x00);
    if (sub) co_sub_set_up_ind(sub, &sdo_up_603F, this);

    // 0x6041: Statusword
    sub = co_dev_find_sub(dev_, 0x6041, 0x00);
    if (sub) co_sub_set_up_ind(sub, &sdo_up_6041, this);

    // 0x6061: Mode display
    sub = co_dev_find_sub(dev_, 0x6061, 0x00);
    if (sub) co_sub_set_up_ind(sub, &sdo_up_6061, this);

    // 0x6064: Position actual value
    sub = co_dev_find_sub(dev_, 0x6064, 0x00);
    if (sub) co_sub_set_up_ind(sub, &sdo_up_6064, this);

    // 0x606C: Velocity actual value
    sub = co_dev_find_sub(dev_, 0x606C, 0x00);
    if (sub) co_sub_set_up_ind(sub, &sdo_up_606C, this);

    // 0x6077: Torque actual value
    sub = co_dev_find_sub(dev_, 0x6077, 0x00);
    if (sub) co_sub_set_up_ind(sub, &sdo_up_6077, this);

    // 0x6079: DC Link circuit voltage
    sub = co_dev_find_sub(dev_, 0x6079, 0x00);
    if (sub) co_sub_set_up_ind(sub, &sdo_up_6079, this);

    // 0x2101: DC Bus current
    sub = co_dev_find_sub(dev_, 0x2101, 0x00);
    if (sub) co_sub_set_up_ind(sub, &sdo_up_2101, this);

    // // Setup RPDO indication callbacks
    // for (int i = 1; i <= 4; i++) {
    //     co_rpdo_t* rpdo = co_nmt_get_rpdo(nmt_, i);
    //     if (rpdo) {
    //         co_rpdo_set_ind(rpdo, &rpdo_ind, this);
    //     }
    // }

    // // Setup TPDO indication callbacks
    // for (int i = 1; i <= 4; i++) {
    //     co_tpdo_t* tpdo = co_nmt_get_tpdo(nmt_, i);
    //     if (tpdo) {
    //         co_tpdo_set_ind(tpdo, &tpdo_ind, this);
    //     }
    // }

    // Subscribe to all CAN messages
    MsgIdFilterSpecs filter;
    filter.id = (uint16_t)0;
    filter.mask = 0;  // Accept all

    auto callback = [](void* ctx, const can_Message_t& msg) {
        static_cast<CANopen*>(ctx)->on_can_message(msg);
    };

    if (!canbus_->subscribe(filter, callback, this, &subscription_handle_)) {
        co_nmt_destroy(nmt_);
        co_dev_destroy(dev_);
        can_net_destroy(net_);
        return false;
    }

    initialized_ = true;
    return true;
}

bool CANopen::renew_subscription() {
    // Re-initialize everything
    if (subscription_handle_) {
        canbus_->unsubscribe(subscription_handle_);
        subscription_handle_ = nullptr;
    }

    shutdown();
    return init();
}

// Service the CANopen stack
uint32_t CANopen::service_stack() {
    if (!initialized_) return 1000;

    bool node_id_changed = (axes[0].config_.can.node_id != node_id_);
    if (node_id_changed) {
        renew_subscription();
    }

    // Update clock
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);

    // Trigger any callback if it is the time
    can_net_set_time(net_, &now);

    last_time_ = now;

    return 50;  // Service every 50ms aka 20hz
}

// NMT command indication
void CANopen::nmt_cs_ind(co_nmt_t* nmt, co_unsigned8_t cs, void* data) {
    switch (cs) {
        case CO_NMT_CS_START:
            // Entered operational state
            // Reset the TIME indication function, since the service may
            // have been restarted.
            co_time_set_ind(co_nmt_get_time(nmt), on_time, NULL);

            break;
        case CO_NMT_CS_STOP:
            // Entered stopped state
            break;
        case CO_NMT_CS_ENTER_PREOP:
            // Entered pre-operational state
            co_time_set_ind(co_nmt_get_time(nmt), &on_time, NULL);

            break;
        case CO_NMT_CS_RESET_NODE:
            // Reset node
            NVIC_SystemReset();

            break;
        case CO_NMT_CS_RESET_COMM:
            // Do nothing
            break;
    }
}

// SDO download callback: 0x6040 Controlword
co_unsigned32_t CANopen::sdo_dn_6040(co_sub_t* sub, struct co_sdo_req* req, void* data) {
    CANopen* self = get_self(data);
    co_unsigned32_t ac = 0;

    co_unsigned16_t type = co_sub_get_type(sub);
    union co_val val;

    // Wait for complete value
    if (co_sdo_req_dn_val(req, type, &val, &ac) == -1)
        return ac;

    // Process controlword
    self->process_controlword(val.u16);

    // Write to OD
    co_sub_dn(sub, &val);
    co_val_fini(type, &val);
    return ac;
}

// SDO download callback: 0x6060 Modes of operation
co_unsigned32_t CANopen::sdo_dn_6060(co_sub_t* sub, struct co_sdo_req* req, void* data) {
    CANopen* self = get_self(data);
    co_unsigned32_t ac = 0;

    co_unsigned16_t type = co_sub_get_type(sub);
    union co_val val;

    if (co_sdo_req_dn_val(req, type, &val, &ac) == -1)
        return ac;

    // Handle mode change
    self->handle_mode_change(val.i8);

    co_sub_dn(sub, &val);
    co_val_fini(type, &val);
    return ac;
}

// SDO download callback: 0x607A Target position
co_unsigned32_t CANopen::sdo_dn_607A(co_sub_t* sub, struct co_sdo_req* req, void* data) {
    co_unsigned32_t ac = 0;

    co_unsigned16_t type = co_sub_get_type(sub);
    union co_val val;

    if (co_sdo_req_dn_val(req, type, &val, &ac) == -1)
        return ac;

    // Set target position on axis
    Axis& axis = axes[0];  // TODO: multi-axis support
    axis.controller_.set_input_pos(static_cast<float>(val.i32) / 16383.0f);

    co_sub_dn(sub, &val);
    co_val_fini(type, &val);
    return ac;
}

// SDO download callback: 0x60FF Target velocity
co_unsigned32_t CANopen::sdo_dn_60FF(co_sub_t* sub, struct co_sdo_req* req, void* data) {
    co_unsigned32_t ac = 0;

    co_unsigned16_t type = co_sub_get_type(sub);
    union co_val val;

    if (co_sdo_req_dn_val(req, type, &val, &ac) == -1)
        return ac;

    // Set target velocity
    Axis& axis = axes[0];
    axis.controller_.input_vel_ = static_cast<float>(val.i32) / 16383.0f;  // Convert ticks/s to turns/s

    co_sub_dn(sub, &val);
    co_val_fini(type, &val);
    return ac;
}

// SDO download callback: 0x6071 Target torque
co_unsigned32_t CANopen::sdo_dn_6071(co_sub_t* sub, struct co_sdo_req* req, void* data) {
    co_unsigned32_t ac = 0;

    co_unsigned16_t type = co_sub_get_type(sub);
    union co_val val;

    if (co_sdo_req_dn_val(req, type, &val, &ac) == -1)
        return ac;

    // Set target torque
    Axis& axis = axes[0];

    float max_torque = axis.motor_.effective_current_lim() * axis.motor_.config_.torque_constant;
    axis.controller_.input_torque_ = max_torque * (static_cast<float>(val.i16) / 1000.0f);

    co_sub_dn(sub, &val);
    co_val_fini(type, &val);
    return ac;
}

// SDO download callback: 0x6072 Max torque
co_unsigned32_t CANopen::sdo_dn_6072(co_sub_t* sub, struct co_sdo_req* req, void* data) {
    co_unsigned32_t ac = 0;

    co_unsigned16_t type = co_sub_get_type(sub);
    union co_val val;

    if (co_sdo_req_dn_val(req, type, &val, &ac) == -1)
        return ac;

    // Set torque limit. The input is 0.1‰ of rated torque
    Axis& axis = axes[0];

    float max_torque = axis.motor_.effective_current_lim() * axis.motor_.config_.torque_constant;
    axis.motor_.config_.torque_lim = (static_cast<float>(val.i16) / 1000.0f) * max_torque;

    co_sub_dn(sub, &val);
    co_val_fini(type, &val);
    return ac;
}

// SDO download callback: 0x6081 Profile velocity
co_unsigned32_t CANopen::sdo_dn_6081(co_sub_t* sub, struct co_sdo_req* req, void* data) {
    co_unsigned32_t ac = 0;

    co_unsigned16_t type = co_sub_get_type(sub);
    union co_val val;

    if (co_sdo_req_dn_val(req, type, &val, &ac) == -1)
        return ac;

    // Set torque limit. The input is 0.1‰ of rated torque
    Axis& axis = axes[0];

    axis.trap_traj_.config_.vel_limit = static_cast<float>(val.u32) / 16383.0f;  // Convert ticks/s (14 bit) to turns/s

    co_sub_dn(sub, &val);
    co_val_fini(type, &val);
    return ac;
}

// SDO download callback: 0x6083 Profile accel
co_unsigned32_t CANopen::sdo_dn_6083(co_sub_t* sub, struct co_sdo_req* req, void* data) {
    co_unsigned32_t ac = 0;

    co_unsigned16_t type = co_sub_get_type(sub);
    union co_val val;

    if (co_sdo_req_dn_val(req, type, &val, &ac) == -1)
        return ac;

    // Set torque limit. The input is 0.1‰ of rated torque
    Axis& axis = axes[0];

    axis.trap_traj_.config_.accel_limit = static_cast<float>(val.u32) / 16383.0f;  // Convert ticks/s (14 bit) to turns/s

    co_sub_dn(sub, &val);
    co_val_fini(type, &val);
    return ac;
}

// SDO download callback: 0x6084 Profile decel
co_unsigned32_t CANopen::sdo_dn_6084(co_sub_t* sub, struct co_sdo_req* req, void* data) {
    co_unsigned32_t ac = 0;

    co_unsigned16_t type = co_sub_get_type(sub);
    union co_val val;

    if (co_sdo_req_dn_val(req, type, &val, &ac) == -1)
        return ac;

    // Set torque limit. The input is 0.1‰ of rated torque
    Axis& axis = axes[0];

    axis.trap_traj_.config_.decel_limit = static_cast<float>(val.u32) / 16383.0f;  // Convert ticks/s (14 bit) to turns/s

    co_sub_dn(sub, &val);
    co_val_fini(type, &val);
    return ac;
}

// SDO upload callback: 0x603F Error code
co_unsigned32_t CANopen::sdo_up_603F(const co_sub_t* sub, struct co_sdo_req* req, void* data) {
    co_unsigned16_t type = co_sub_get_type(sub);

    co_unsigned16_t err_code = 0;

    // Only report general error
    // TODO: Report detailed error
    Axis& axis = axes[0];
    if (axis.motor_.error_) {
        // Set motor error
        err_code |= (1 << 0);
    }

    if (axis.controller_.error_) {
        // Set controller error
        err_code |= (1 << 1);
    }

    if (axis.encoder_.error_) {
        // Set encoder error
        err_code |= (1 << 2);
    }

    if (axis.error_) {
        // Set axis error
        err_code |= (1 << 3);
    }

    co_unsigned32_t ac = 0;
    co_sdo_req_up_val(req, type, &err_code, &ac);
    return ac;
}

// SDO upload callback: 0x6041 Statusword
co_unsigned32_t CANopen::sdo_up_6041(const co_sub_t* sub, struct co_sdo_req* req, void* data) {
    using namespace CIA402;

    co_unsigned16_t type = co_sub_get_type(sub);

    // Get current axis state
    Axis& axis = axes[0];
    co_unsigned16_t statusword = 0;

    switch (axis.current_state_) {
        case Axis::AXIS_STATE_IDLE:
            statusword = make_statusword(STATE_READY_TO_SWITCH_ON);  // Ready to switch on
            break;
        case Axis::AXIS_STATE_CLOSED_LOOP_CONTROL:
            statusword = make_statusword(STATE_OPERATION_ENABLED,
                                         true,                                // voltage enabled
                                         false,                               // no warning
                                         axis.controller_.trajectory_done_);  // Operation enabled
            break;
        case Axis::AXIS_STATE_FULL_CALIBRATION_SEQUENCE:
        case Axis::AXIS_STATE_MOTOR_CALIBRATION:
        case Axis::AXIS_STATE_ENCODER_INDEX_SEARCH:
        case Axis::AXIS_STATE_ENCODER_OFFSET_CALIBRATION:
            statusword = make_statusword(STATE_SWITCH_ON_DISABLED);  // Switch on disabled (calibrating)
            break;
        default:
            if (axis.error_ != Axis::ERROR_NONE) {
                statusword = make_statusword(STATE_FAULT, false);  // Fault
            } else {
                statusword = make_statusword(STATE_SWITCH_ON_DISABLED);  // Switch on disabled
            }
            break;
    }

    // Add warning bit if there are non-critical errors
    if (axis.motor_.error_ != 0 || axis.encoder_.error_ != 0 || axis.controller_.error_ != 0) {
        statusword |= SW_WARNING;
    }

    co_unsigned32_t ac = 0;
    co_sdo_req_up_val(req, type, &statusword, &ac);
    return ac;
}

// SDO upload callback: 0x6064 Position actual value
co_unsigned32_t CANopen::sdo_up_6064(const co_sub_t* sub, struct co_sdo_req* req, void* data) {
    co_unsigned16_t type = co_sub_get_type(sub);

    Axis& axis = axes[0];
    float pos = axis.controller_.pos_estimate_linear_src_.any().value_or(0.0f);
    co_integer32_t pos_counts = static_cast<co_integer32_t>(pos * 16383.0f);  // Convert turns to tick, emulate a 14 bit absolute encoder

    co_unsigned32_t ac = 0;
    co_sdo_req_up_val(req, type, &pos_counts, &ac);
    return ac;
}

// SDO upload callback: 0x606C Velocity actual value
co_unsigned32_t CANopen::sdo_up_606C(const co_sub_t* sub, struct co_sdo_req* req, void* data) {
    co_unsigned16_t type = co_sub_get_type(sub);

    Axis& axis = axes[0];
    float vel = axis.controller_.vel_estimate_src_.any().value_or(0.0f);
    co_integer32_t vel_counts = static_cast<co_integer32_t>(vel * 16383.0f);  // Convert turns/s to tick/s, emulate a 14 bit absolute encoder

    co_unsigned32_t ac = 0;
    co_sdo_req_up_val(req, type, &vel_counts, &ac);
    return ac;
}

// SDO upload callback: 0x6077 Torque actual value
co_unsigned32_t CANopen::sdo_up_6077(const co_sub_t* sub, struct co_sdo_req* req, void* data) {
    co_unsigned16_t type = co_sub_get_type(sub);

    Axis& axis = axes[0];
    float max_torque = axis.motor_.effective_current_lim() * axis.motor_.config_.torque_constant;
    float iq_measured = axis.motor_.current_control_.Iq_measured_;
    float torque = iq_measured * axis.motor_.config_.torque_constant;

    co_integer16_t torque_per_rated_torque = static_cast<co_integer16_t>(
        (torque / max_torque) * 1000.0f);  // in 0.1‰ of rated torque

    co_unsigned32_t ac = 0;
    co_sdo_req_up_val(req, type, &torque_per_rated_torque, &ac);
    return ac;
}

// SDO upload callback: 0x6061 Mode display
co_unsigned32_t CANopen::sdo_up_6061(const co_sub_t* sub, struct co_sdo_req* req, void* data) {
    co_unsigned16_t type = co_sub_get_type(sub);

    Axis& axis = axes[0];

    // Update modes of operation display
    co_integer8_t mode_display = 0;
    switch (axis.controller_.config_.control_mode) {
        case Controller::CONTROL_MODE_POSITION_CONTROL:
            if (axis.controller_.config_.input_mode == Controller::INPUT_MODE_TRAP_TRAJ) {
                // PP mode
                mode_display = 1;
            } else {
                // CSP mode
                mode_display = 8;
            }
            break;
        case Controller::CONTROL_MODE_VELOCITY_CONTROL:
            if (axis.controller_.config_.input_mode == Controller::INPUT_MODE_VEL_RAMP) {
                // VP mode
                mode_display = 3;
            } else {
                // CSP mode
                mode_display = 9;
            }
            break;
        case Controller::CONTROL_MODE_TORQUE_CONTROL:
            mode_display = 4;
            break;
        case Controller::CONTROL_MODE_VOLTAGE_CONTROL:
            // Shouldn't use this mode, internal use
            mode_display = 125;
            break;
        default:
            // Unknown mode
            mode_display = 126;
    }

    co_unsigned32_t ac = 0;
    co_sdo_req_up_val(req, type, &mode_display, &ac);
    return ac;
}

// SDO upload callback: 0x6079 DC link circuit voltage
co_unsigned32_t CANopen::sdo_up_6079(const co_sub_t* sub, struct co_sdo_req* req, void* data) {
    co_unsigned16_t type = co_sub_get_type(sub);

    co_unsigned32_t voltage_fixed = static_cast<co_unsigned32_t>(vbus_voltage * 1000.0f);
    co_unsigned32_t ac = 0;
    co_sdo_req_up_val(req, type, &voltage_fixed, &ac);
    return ac;
}

// SDO upload callback: 0x2101 Bus current
co_unsigned32_t CANopen::sdo_up_2101(const co_sub_t* sub, struct co_sdo_req* req, void* data) {
    co_unsigned16_t type = co_sub_get_type(sub);

    co_real32_t ibus = static_cast<co_real32_t>(ibus_);
    co_unsigned32_t ac = 0;
    co_sdo_req_up_val(req, type, &ibus, &ac);
    return ac;
}

// Process CIA 402 controlword
void CANopen::process_controlword(co_unsigned16_t controlword) {
    using namespace CIA402;
    Axis& axis = axes[0];  // TODO: multi-axis

    // Check for fault reset (0->1 transition on bit 7)
    if (is_fault_reset_requested(controlword, last_controlword_)) {
        odrv.clear_errors();
    }

    uint16_t interest_mask = 0x008F;

    if ((controlword & interest_mask) == (last_controlword_ & interest_mask)) {
        last_controlword_ = controlword;  // Keep history updated
        return;
    }

    // Store for next edge detection
    last_controlword_ = controlword;

    // State machine (bits 0-3)
    uint8_t cmd = controlword & 0x000F;

    switch (cmd) {
        case 0x06:  // CMD_SHUTDOWN or similar patterns
        case 0x00:  // CMD_DISABLE_VOLTAGE
            axis.requested_state_ = Axis::AXIS_STATE_IDLE;
            break;

        case 0x07:  // CMD_SWITCH_ON
            // Switch on but not operation enabled
            axis.requested_state_ = Axis::AXIS_STATE_IDLE;
            break;

        case 0x0F:  // CMD_ENABLE_OPERATION
            axis.requested_state_ = Axis::AXIS_STATE_CLOSED_LOOP_CONTROL;
            break;
    }

    if ((cmd & CW_QUICK_STOP) == 0 && (cmd & CW_ENABLE_VOLTAGE)) {
        axis.requested_state_ = Axis::AXIS_STATE_IDLE;
    }

    // Feed the watchdog when we received a control word
    axis.watchdog_feed();
}

// Handle mode of operation change
void CANopen::handle_mode_change(co_integer8_t mode) {
    Axis& axis = axes[0];

    auto des_mode = Controller::CONTROL_MODE_POSITION_CONTROL;
    auto des_input = Controller::INPUT_MODE_PASSTHROUGH;

    switch (mode) {
        case 1:  // Profile position mode
            des_mode = Controller::CONTROL_MODE_POSITION_CONTROL;
            des_input = Controller::INPUT_MODE_TRAP_TRAJ;
            break;
        case 3:  // Profile velocity mode
            des_mode = Controller::CONTROL_MODE_VELOCITY_CONTROL;
            des_input = Controller::INPUT_MODE_VEL_RAMP;
            break;
        case 4:  // Torque profile mode
            des_mode = Controller::CONTROL_MODE_TORQUE_CONTROL;
            des_input = Controller::INPUT_MODE_PASSTHROUGH;
            break;
        case 8:  // CSP - Cyclic synchronous position
            des_mode = Controller::CONTROL_MODE_POSITION_CONTROL;
            des_input = Controller::INPUT_MODE_PASSTHROUGH;
            break;
        case 9:  // CSV - Cyclic synchronous velocity
            des_mode = Controller::CONTROL_MODE_VELOCITY_CONTROL;
            des_input = Controller::INPUT_MODE_PASSTHROUGH;
            break;
        case 10:  // CST - Cyclic synchronous torque
            des_mode = Controller::CONTROL_MODE_TORQUE_CONTROL;
            des_input = Controller::INPUT_MODE_PASSTHROUGH;
            break;
    }

    if (des_mode != axis.controller_.config_.control_mode || des_input != axis.controller_.config_.input_mode) {
        // Only change mode when requested
        axis.controller_.config_.control_mode = des_mode;
        axis.controller_.config_.input_mode = des_input;
        axis.controller_.control_mode_updated();
    }
}