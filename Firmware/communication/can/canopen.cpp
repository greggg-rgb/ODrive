#include "canopen.hpp"

#include <lely/co/sdev.h>
#include <lely/co/time.h>
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
    uint8_t node_id_ = axis.config_.can.node_id;

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

    // Setup SDO download callbacks (writes from master)
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

    // Setup SDO upload callbacks (reads from master)
    // 0x6041: Statusword
    sub = co_dev_find_sub(dev_, 0x6041, 0x00);
    if (sub) co_sub_set_up_ind(sub, &sdo_up_6041, this);

    // 0x6064: Position actual value
    sub = co_dev_find_sub(dev_, 0x6064, 0x00);
    if (sub) co_sub_set_up_ind(sub, &sdo_up_6064, this);

    // 0x606C: Velocity actual value
    sub = co_dev_find_sub(dev_, 0x606C, 0x00);
    if (sub) co_sub_set_up_ind(sub, &sdo_up_606C, this);

    // 0x6077: Torque actual value
    sub = co_dev_find_sub(dev_, 0x6077, 0x00);
    if (sub) co_sub_set_up_ind(sub, &sdo_up_6077, this);

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
    can_net_set_time(net_, &now);

    last_time_ = now;

    // Update OD values that are read frequently
    update_od_from_axis();

    return 1;  // Service every 1ms
}

// Update OD from axis state (for reads)
void CANopen::update_od_from_axis() {
    if (!initialized_ || !dev_) return;

    // These values are updated in the OD so SDO reads and TPDO transmissions
    // get fresh data. We don't use co_dev_set_val_* here because those
    // functions are meant for configuration. Instead, we update the
    // sub-object values directly when they're read via SDO callbacks.
}

// NMT command indication
void CANopen::nmt_cs_ind(co_nmt_t* nmt, co_unsigned8_t cs, void* data) {
    CANopen* self = get_self(data);

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
            exit(0);

            break;
        case CO_NMT_CS_RESET_COMM:
            // Reset communication
            self->renew_subscription();
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
    axis.controller_.set_input_pos_and_steps(static_cast<float>(val.i32));

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
    axis.controller_.input_vel_ = static_cast<float>(val.i32) / 1000.0f;

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
    axis.controller_.input_torque_ = static_cast<float>(val.i16) / 1000.0f;

    co_sub_dn(sub, &val);
    co_val_fini(type, &val);
    return ac;
}

// SDO upload callback: 0x6041 Statusword
co_unsigned32_t CANopen::sdo_up_6041(const co_sub_t* sub, struct co_sdo_req* req, void* data) {
    co_unsigned16_t type = co_sub_get_type(sub);

    // Get current axis state
    Axis& axis = axes[0];
    co_unsigned16_t statusword = 0;

    switch (axis.current_state_) {
        case Axis::AXIS_STATE_IDLE:
            statusword = 0x0231;  // Ready to switch on
            break;
        case Axis::AXIS_STATE_CLOSED_LOOP_CONTROL:
            statusword = 0x0237;  // Operation enabled
            break;
        case Axis::AXIS_STATE_FULL_CALIBRATION_SEQUENCE:
        case Axis::AXIS_STATE_MOTOR_CALIBRATION:
        case Axis::AXIS_STATE_ENCODER_INDEX_SEARCH:
        case Axis::AXIS_STATE_ENCODER_OFFSET_CALIBRATION:
            statusword = 0x0217;  // Switch on disabled (calibrating)
            break;
        default:
            if (axis.error_ != Axis::ERROR_NONE) {
                statusword = 0x0208;  // Fault
            } else {
                statusword = 0x0210;  // Switch on disabled
            }
            break;
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
    co_integer32_t pos_counts = static_cast<co_integer32_t>(pos * static_cast<float>(0x3FFF));  // Convert turns to tick, emulate a 14 bit absolute encoder

    co_unsigned32_t ac = 0;
    co_sdo_req_up_val(req, type, &pos_counts, &ac);
    return ac;
}

// SDO upload callback: 0x606C Velocity actual value
co_unsigned32_t CANopen::sdo_up_606C(const co_sub_t* sub, struct co_sdo_req* req, void* data) {
    co_unsigned16_t type = co_sub_get_type(sub);

    Axis& axis = axes[0];
    float vel = axis.controller_.vel_estimate_src_.any().value_or(0.0f);
    co_integer32_t vel_counts = static_cast<co_integer32_t>(vel * static_cast<float>(0x3FFF));  // Convert turns/s to tick/s, emulate a 14 bit absolute encoder

    co_unsigned32_t ac = 0;
    co_sdo_req_up_val(req, type, &vel_counts, &ac);
    return ac;
}

// SDO upload callback: 0x6077 Torque actual value
co_unsigned32_t CANopen::sdo_up_6077(const co_sub_t* sub, struct co_sdo_req* req, void* data) {
    co_unsigned16_t type = co_sub_get_type(sub);

    Axis& axis = axes[0];
    co_integer16_t torque = static_cast<co_integer16_t>(
        axis.motor_.current_control_.Iq_measured_ * 1000.0f);  // Read Iq current in milliamps instead

    co_unsigned32_t ac = 0;
    co_sdo_req_up_val(req, type, &torque, &ac);
    return ac;
}

// Process CIA 402 controlword
void CANopen::process_controlword(co_unsigned16_t controlword) {
    Axis& axis = axes[0];  // TODO: multi-axis

    // Bit 7: Fault reset
    if (controlword & 0x0080) {
        odrv.clear_errors();
    }

    // State machine (bits 0-3)
    uint8_t cmd = controlword & 0x000F;

    switch (cmd) {
        case 0x06:  // Shutdown
        case 0x00:  // Disable voltage
            axis.requested_state_ = Axis::AXIS_STATE_IDLE;
            break;
        case 0x0F:  // Enable operation
            axis.requested_state_ = Axis::AXIS_STATE_CLOSED_LOOP_CONTROL;
            break;
    }
}

// Handle mode of operation change
void CANopen::handle_mode_change(co_integer8_t mode) {
    Axis& axis = axes[0];

    switch (mode) {
        case 1:  // Profile position mode
            axis.controller_.config_.control_mode = Controller::CONTROL_MODE_POSITION_CONTROL;
            axis.controller_.config_.input_mode = Controller::INPUT_MODE_TRAP_TRAJ;
            break;
        case 3:  // Profile velocity mode
            axis.controller_.config_.control_mode = Controller::CONTROL_MODE_VELOCITY_CONTROL;
            axis.controller_.config_.input_mode = Controller::INPUT_MODE_VEL_RAMP;
            break;
        case 4:  // Torque profile mode
            axis.controller_.config_.control_mode = Controller::CONTROL_MODE_TORQUE_CONTROL;
            axis.controller_.config_.input_mode = Controller::INPUT_MODE_PASSTHROUGH;
            break;
        case 8:  // CSP - Cyclic synchronous position
            axis.controller_.config_.control_mode = Controller::CONTROL_MODE_POSITION_CONTROL;
            axis.controller_.config_.input_mode = Controller::INPUT_MODE_PASSTHROUGH;
            break;
        case 9:  // CSV - Cyclic synchronous velocity
            axis.controller_.config_.control_mode = Controller::CONTROL_MODE_VELOCITY_CONTROL;
            axis.controller_.config_.input_mode = Controller::INPUT_MODE_PASSTHROUGH;
            break;
        case 10:  // CST - Cyclic synchronous torque
            axis.controller_.config_.control_mode = Controller::CONTROL_MODE_TORQUE_CONTROL;
            axis.controller_.config_.input_mode = Controller::INPUT_MODE_PASSTHROUGH;
            break;
    }

    axis.controller_.control_mode_updated();
}