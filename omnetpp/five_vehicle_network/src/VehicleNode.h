//
// Vehicle Node Header
// Phase 2 - Five Vehicle Communication Network
//

#ifndef __FIVE_VEHICLE_NETWORK_VEHICLENODE_H_
#define __FIVE_VEHICLE_NETWORK_VEHICLENODE_H_

#include <omnetpp.h>
#include "CollisionWarningMsg_m.h"

using namespace omnetpp;

namespace five_vehicle_network {

/**
 * VehicleNode represents a vehicular communication terminal (OBU - On-Board Unit).
 * It demonstrates:
 * - Local event scheduling (self-message)
 * - Emergency message initiation upon detecting hazard (e.g. sudden braking at t=15s)
 * - Multi-hop forwarding along the linear highway topology
 * - End-to-end latency measurement and hop-count verification
 */
class VehicleNode : public cSimpleModule
{
  protected:
    int nodeId;
    std::string nodeName;
    bool isInitiator;
    int targetNodeId;
    simtime_t sendTime;

    cMessage *selfEventMsg;

    // Simulation statistics
    simsignal_t e2eDelaySignal;
    simsignal_t hopCountSignal;

  protected:
    virtual void initialize() override;
    virtual void handleMessage(cMessage *msg) override;
    virtual void forwardMessage(CollisionWarningMsg *warningMsg);
    virtual void finish() override;

  public:
    VehicleNode();
    virtual ~VehicleNode();
};

} // namespace five_vehicle_network

#endif
