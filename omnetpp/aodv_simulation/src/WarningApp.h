//
// Collision Warning Application Header
// Phase 3 - AODV VANET Simulation
//

#ifndef __AODV_SIMULATION_WARNINGAPP_H_
#define __AODV_SIMULATION_WARNINGAPP_H_

#include <omnetpp.h>
#include "inet/common/INETDefs.h"
#include "inet/transportlayer/contract/udp/UdpSocket.h"
#include "inet/networklayer/common/L3AddressResolver.h"

using namespace omnetpp;
using namespace inet;

namespace aodv_simulation {

/**
 * WarningApp generates and receives emergency vehicular warning messages
 * across the multi-hop AODV wireless network.
 */
class WarningApp : public cSimpleModule, public UdpSocket::ICallback
{
  protected:
    UdpSocket socket;
    std::string nodeName;
    int nodeId;
    bool isSender;
    L3Address destAddress;
    int destPort;
    int localPort;
    simtime_t startTime;
    simtime_t sendInterval;
    int packetCount;
    int packetsSent;
    std::string warningType;
    std::string messageId;
    int packetSize;

    cMessage *timerMsg;

    // Signals for metric recording
    simsignal_t e2eDelaySignal;
    simsignal_t packetReceivedSignal;
    simsignal_t packetSentSignal;

  protected:
    virtual void initialize(int stage) override;
    virtual int numInitStages() const override { return NUM_INIT_STAGES; }
    virtual void handleMessage(cMessage *msg) override;
    virtual void finish() override;

    // UdpSocket::ICallback methods
    virtual void socketDataArrived(UdpSocket *socket, Packet *packet) override;
    virtual void socketErrorArrived(UdpSocket *socket, Indication *indication) override;
    virtual void socketClosed(UdpSocket *socket) override;

    virtual void sendWarningPacket();

  public:
    WarningApp();
    virtual ~WarningApp();
};

} // namespace aodv_simulation

#endif
