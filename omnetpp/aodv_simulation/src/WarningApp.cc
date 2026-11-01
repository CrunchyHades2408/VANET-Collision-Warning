//
// Collision Warning Application Implementation
// Phase 3 - AODV VANET Simulation
//

#include "WarningApp.h"
#include "inet/common/packet/Packet.h"
#include "inet/common/packet/chunk/ByteCountChunk.h"

namespace aodv_simulation {

Define_Module(WarningApp);

WarningApp::WarningApp()
{
    timerMsg = nullptr;
    packetsSent = 0;
}

WarningApp::~WarningApp()
{
    cancelAndDelete(timerMsg);
}

void WarningApp::initialize(int stage)
{
    cSimpleModule::initialize(stage);

    if (stage == INITSTAGE_LOCAL) {
        nodeName = par("nodeName").stringValue();
        nodeId = par("nodeId");
        isSender = par("isSender").boolValue();
        destPort = par("destPort");
        localPort = par("localPort");
        startTime = par("startTime").doubleValue();
        sendInterval = par("sendInterval").doubleValue();
        packetCount = par("packetCount");
        warningType = par("warningType").stringValue();
        messageId = par("messageId").stringValue();
        packetSize = par("packetSize");

        e2eDelaySignal = registerSignal("e2eDelay");
        packetReceivedSignal = registerSignal("packetReceived");
        packetSentSignal = registerSignal("packetSent");

        socket.setOutputGate(gate("socketOut"));
        socket.setCallback(this);
    }
    else if (stage == INITSTAGE_APPLICATION_LAYER) {
        socket.bind(localPort);

        if (isSender) {
            const char *destStr = par("destAddress").stringValue();
            destAddress = L3AddressResolver().resolve(destStr);

            timerMsg = new cMessage("SendWarningTimer");
            scheduleAt(startTime, timerMsg);
            EV_INFO << "[" << nodeName << "] Scheduled warning transmission to "
                    << destAddress << ":" << destPort << " at t = " << startTime << " s\n";
        }
    }
}

void WarningApp::handleMessage(cMessage *msg)
{
    if (msg == timerMsg) {
        sendWarningPacket();
        packetsSent++;
        if (packetsSent < packetCount) {
            scheduleAt(simTime() + sendInterval, timerMsg);
        }
    }
    else {
        socket.processMessage(msg);
    }
}

void WarningApp::sendWarningPacket()
{
    std::string pktName = messageId + "_" + std::to_string(packetsSent + 1);
    Packet *packet = new Packet(pktName.c_str());

    // Create 256-byte payload representing vehicle telemetry & hazard status
    const auto& payload = makeShared<ByteCountChunk>(B(packetSize));
    packet->insertAtBack(payload);

    EV_WARN << "\n=======================================================\n"
            << " [" << nodeName << "] TRANSMITTING WARNING PACKET '" << pktName << "'\n"
            << " Destination IP   : " << destAddress << ":" << destPort << "\n"
            << " Hazard Type      : " << warningType << "\n"
            << " Transmission Time: " << simTime() << " s\n"
            << " Routing Protocol : AODV Route Discovery Initiated if No Route Exists\n"
            << "=======================================================\n";

    emit(packetSentSignal, 1);
    socket.sendTo(packet, destAddress, destPort);
}

void WarningApp::socketDataArrived(UdpSocket *socket, Packet *packet)
{
    simtime_t e2eDelay = simTime() - packet->getCreationTime();

    EV_WARN << "\n*******************************************************\n"
            << " [" << nodeName << "] SUCCESS: WARNING PACKET DELIVERED!\n"
            << " Packet Name    : " << packet->getName() << "\n"
            << " Byte Length    : " << packet->getByteLength() << " bytes\n"
            << " End-to-End Latency: " << (e2eDelay * 1000.0) << " ms\n"
            << " Reception Time : " << simTime() << " s\n"
            << " Driver Display : FORWARD COLLISION HAZARD ALERT ACTIVATED!\n"
            << "*******************************************************\n\n";

    emit(e2eDelaySignal, e2eDelay.dbl());
    emit(packetReceivedSignal, 1);
    delete packet;
}

void WarningApp::socketErrorArrived(UdpSocket *socket, Indication *indication)
{
    EV_ERROR << "[" << nodeName << "] Socket error occurred: " << indication->getName() << endl;
    delete indication;
}

void WarningApp::socketClosed(UdpSocket *socket)
{
    EV_INFO << "[" << nodeName << "] Socket closed.\n";
}

void WarningApp::finish()
{
    EV_INFO << "[" << nodeName << "] Simulation finished. Application terminated cleanly.\n";
}

} // namespace aodv_simulation
