#ifndef __CANOPEN_HPP
#define __CANOPEN_HPP

#ifndef LELY_NO_CXX
#define LELY_NO_CXX 1
#endif

#include <lely/can/net.h>
#include <lely/co/dev.h>
#include <lely/co/nmt.h>
#include <lely/co/obj.h>
#include <lely/co/pdo.h>
#include <lely/co/sdo.h>
#include <lely/co/sync.h>
#include <lely/co/val.h>

#include "canbus.hpp"

// Forward declarations
class Axis;

// CiA 402 Statusword bits (0x6041)
namespace CIA402 {

// Statusword bit definitions
enum StatuswordBits : uint16_t {
    SW_READY_TO_SWITCH_ON = (1 << 0),      // Bit 0
    SW_SWITCHED_ON = (1 << 1),             // Bit 1
    SW_OPERATION_ENABLED = (1 << 2),       // Bit 2
    SW_FAULT = (1 << 3),                   // Bit 3
    SW_VOLTAGE_ENABLED = (1 << 4),         // Bit 4
    SW_QUICK_STOP = (1 << 5),              // Bit 5
    SW_SWITCH_ON_DISABLED = (1 << 6),      // Bit 6
    SW_WARNING = (1 << 7),                 // Bit 7
    SW_MANUFACTURER_SPECIFIC = (1 << 8),   // Bit 8
    SW_REMOTE = (1 << 9),                  // Bit 9
    SW_TARGET_REACHED = (1 << 10),         // Bit 10
    SW_INTERNAL_LIMIT_ACTIVE = (1 << 11),  // Bit 11
    // Bits 12-15: Operation mode specific and manufacturer specific
};

// Standard statusword states (combination of bits)
enum StatuswordState : uint16_t {
    // State machine states (bits 0-6 define the state)
    // Mask: x0xx xxxx b
    STATE_NOT_READY_TO_SWITCH_ON = 0x0000,  // 0b0x00 0000
    STATE_SWITCH_ON_DISABLED = 0x0040,      // 0b0x40 0000 (bit 6 set)
    STATE_READY_TO_SWITCH_ON = 0x0021,      // 0b0x21 (bits 0, 5 set)
    STATE_SWITCHED_ON = 0x0023,             // 0b0x23 (bits 0, 1, 5 set)
    STATE_OPERATION_ENABLED = 0x0027,       // 0b0x27 (bits 0, 1, 2, 5 set)
    STATE_QUICK_STOP_ACTIVE = 0x0007,       // 0b0x07 (bits 0, 1, 2 set)
    STATE_FAULT_REACTION_ACTIVE = 0x000F,   // 0b0x0F (bits 0, 1, 2, 3 set)
    STATE_FAULT = 0x0008,                   // 0b0x08 (bit 3 set only)
};

// Mask for state machine bits (bits 0-6)
constexpr uint16_t STATUSWORD_STATE_MASK = 0x006F;

// Controlword bit definitions (0x6040)
enum ControlwordBits : uint16_t {
    CW_SWITCH_ON = (1 << 0),         // Bit 0
    CW_ENABLE_VOLTAGE = (1 << 1),    // Bit 1
    CW_QUICK_STOP = (1 << 2),        // Bit 2
    CW_ENABLE_OPERATION = (1 << 3),  // Bit 3
    // Bits 4-6: Operation mode specific
    CW_FAULT_RESET = (1 << 7),  // Bit 7
    CW_HALT = (1 << 8),         // Bit 8
    // Bits 9-15: Manufacturer specific or operation mode specific
};

// Standard controlword commands (bits 0-3)
enum ControlwordCommand : uint16_t {
    CMD_SHUTDOWN = 0x0006,             // 0b0110 - Transition to Ready to Switch On
    CMD_SWITCH_ON = 0x0007,            // 0b0111 - Transition to Switched On
    CMD_SWITCH_ON_ENABLE_OP = 0x000F,  // 0b1111 - Transition to Operation Enabled
    CMD_DISABLE_VOLTAGE = 0x0000,      // 0b0x00 - Transition to Switch On Disabled
    CMD_QUICK_STOP = 0x0002,           // 0b0x10 - Transition to Quick Stop
    CMD_DISABLE_OPERATION = 0x0007,    // 0b0111 - Transition to Switched On (from Op Enabled)
    CMD_ENABLE_OPERATION = 0x000F,     // 0b1111 - Transition to Operation Enabled
    CMD_FAULT_RESET = 0x0080,          // Bit 7: 0->1 transition resets fault
};

// Mask for command bits (bits 0-3, bit 7)
constexpr uint16_t CONTROLWORD_COMMAND_MASK = 0x008F;

// Helper functions
inline uint16_t make_statusword(StatuswordState base_state, bool voltage_enabled = true,
                                bool warning = false, bool target_reached = false) {
    uint16_t sw = base_state;
    if (voltage_enabled) sw |= SW_VOLTAGE_ENABLED;
    if (warning) sw |= SW_WARNING;
    if (target_reached) sw |= SW_TARGET_REACHED;
    return sw;
}

inline bool is_fault_reset_requested(uint16_t controlword, uint16_t previous_controlword) {
    // Fault reset requires 0->1 transition on bit 7
    return ((controlword & CW_FAULT_RESET) && !(previous_controlword & CW_FAULT_RESET));
}

inline uint16_t get_controlword_command(uint16_t controlword) {
    return controlword & CONTROLWORD_COMMAND_MASK;
}

}  // namespace CIA402

class CANopen {
   public:
    CANopen(CanBusBase* canbus) : canbus_(canbus) {}

    bool init();
    uint32_t service_stack();
    void shutdown();

    // CAN message receive callback
    void on_can_message(const can_Message_t& msg);

   private:
    bool renew_subscription();

    // Static callbacks for Lely CO stack
    static int can_send_func(const struct can_msg* msg, void* data);
    static void nmt_cs_ind(co_nmt_t* nmt, co_unsigned8_t cs, void* data);

    // PDO callbacks
    static void rpdo_ind(co_rpdo_t* pdo, co_unsigned32_t ac, const void* ptr, size_t n, void* data);
    static void tpdo_ind(co_tpdo_t* pdo, co_unsigned32_t ac, const void* ptr, size_t n, void* data);

    // SDO callbacks for specific objects
    static co_unsigned32_t sdo_dn_6040(co_sub_t* sub, struct co_sdo_req* req, void* data);
    static co_unsigned32_t sdo_dn_607A(co_sub_t* sub, struct co_sdo_req* req, void* data);
    static co_unsigned32_t sdo_dn_60FF(co_sub_t* sub, struct co_sdo_req* req, void* data);
    static co_unsigned32_t sdo_dn_6071(co_sub_t* sub, struct co_sdo_req* req, void* data);
    static co_unsigned32_t sdo_dn_6060(co_sub_t* sub, struct co_sdo_req* req, void* data);

    static co_unsigned32_t sdo_up_603F(const co_sub_t* sub, struct co_sdo_req* req, void* data);
    static co_unsigned32_t sdo_up_6041(const co_sub_t* sub, struct co_sdo_req* req, void* data);
    static co_unsigned32_t sdo_up_6064(const co_sub_t* sub, struct co_sdo_req* req, void* data);
    static co_unsigned32_t sdo_up_606C(const co_sub_t* sub, struct co_sdo_req* req, void* data);
    static co_unsigned32_t sdo_up_6077(const co_sub_t* sub, struct co_sdo_req* req, void* data);

    // PDO handling
    void handle_rpdo1(const void* data, size_t n);
    void handle_rpdo2(const void* data, size_t n);
    void handle_rpdo3(const void* data, size_t n);
    void handle_rpdo4(const void* data, size_t n);

    void prepare_tpdo1();
    void prepare_tpdo2();
    void prepare_tpdo3();
    void prepare_tpdo4();

    // Update functions
    void update_od_from_axis();
    void process_controlword(co_unsigned16_t controlword);
    void handle_mode_change(co_integer8_t mode);

    CanBusBase* canbus_;
    CanBusBase::CanSubscription* subscription_handle_;
    uint8_t node_id_ = 1;

    // Lely CO objects
    can_net_t* net_ = nullptr;
    co_dev_t* dev_ = nullptr;
    co_nmt_t* nmt_ = nullptr;

    // Track previous controlword for fault reset edge detection
    co_unsigned16_t last_controlword_ = 0;

    // Timing
    struct timespec last_time_;

    // State
    bool initialized_ = false;

    // Helper to get self pointer from NMT
    static CANopen* get_self(void* data) {
        return static_cast<CANopen*>(data);
    }
};

#endif  // __CANOPEN_HPP