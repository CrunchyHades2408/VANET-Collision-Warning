//
// Veins Warning Application Header
// Phase 4 - SUMO-OMNeT++ Veins Integration
//

#ifndef __VEINS_INTEGRATION_VEINSWARNINGAPP_H_
#define __VEINS_INTEGRATION_VEINSWARNINGAPP_H_

#include <omnetpp.h>
#include "inet/common/INETDefs.h"
#include "inet/transportlayer/contract/udp/UdpSocket.h"
#include "inet/networklayer/common/L3AddressResolver.h"

using namespace omnetpp;
using namespace inet;

namespace veins_integration {

/**
 * VeinsWarningApp implements application-layer collision warning dissemination
 * for mobile vehicle nodes driven by SUMO via Veins / TraCI.
 */
class VeinsWarningApp : public cSimpleModule, public UdpSocket::ICallback
{
  protected:
    UdpSocket socket;
    std::string nodeName;
    std::string sumoVehicleId;
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

    // Measurement signals
    simsignal_t e2eDelaySignal;
    simsignal_t packetReceivedSignal;
    simsignal_t packetSentSignal;

  protected:
    virtual void initialize(int stage) override;
    virtual int numInitStages() const override { return NUM_INIT_STAGES; }
    virtual void handleMessage(cMessage *msg) override;
    virtual void finish() override;

    // UdpSocket callbacks
    virtual void socketDataArrived(UdpSocket *socket, Packet *packet) override;
    virtual void socketErrorArrived(UdpSocket *socket, Indication *indication) override;
    virtual void socketClosed(UdpSocket *socket) override;

    virtual void sendWarningPacket();

  public:
    VeinsWarningApp();
    virtual ~VeinsWarningApp();
};

} // namespace veins_integration

#endif
