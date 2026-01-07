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
#include <lely/co/val.h>

#include "canbus.hpp"

// Forward declarations
class Axis;

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

    // SDO callbacks for specific objects
    static co_unsigned32_t sdo_dn_6040(co_sub_t* sub, struct co_sdo_req* req, void* data);
    static co_unsigned32_t sdo_dn_607A(co_sub_t* sub, struct co_sdo_req* req, void* data);
    static co_unsigned32_t sdo_dn_60FF(co_sub_t* sub, struct co_sdo_req* req, void* data);
    static co_unsigned32_t sdo_dn_6071(co_sub_t* sub, struct co_sdo_req* req, void* data);
    static co_unsigned32_t sdo_dn_6060(co_sub_t* sub, struct co_sdo_req* req, void* data);

    static co_unsigned32_t sdo_up_6041(const co_sub_t* sub, struct co_sdo_req* req, void* data);
    static co_unsigned32_t sdo_up_6064(const co_sub_t* sub, struct co_sdo_req* req, void* data);
    static co_unsigned32_t sdo_up_606C(const co_sub_t* sub, struct co_sdo_req* req, void* data);
    static co_unsigned32_t sdo_up_6077(const co_sub_t* sub, struct co_sdo_req* req, void* data);

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
    co_sync_t* sync_ = nullptr;
    co_rpdo_t* rpdo_ = nullptr;
    co_tpdo_t* tpdo_ = nullptr;

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