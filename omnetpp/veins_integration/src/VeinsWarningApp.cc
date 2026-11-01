//
// Veins Warning Application Implementation
// Phase 4 - SUMO-OMNeT++ Veins Integration
//

#include "VeinsWarningApp.h"
#include "inet/common/packet/Packet.h"
#include "inet/common/packet/chunk/ByteCountChunk.h"

namespace veins_integration {

Define_Module(VeinsWarningApp);

VeinsWarningApp::VeinsWarningApp()
{
    timerMsg = nullptr;
    packetsSent = 0;
}

VeinsWarningApp::~VeinsWarningApp()
{
    cancelAndDelete(timerMsg);
}

void VeinsWarningApp::initialize(int stage)
{
    cSimpleModule::initialize(stage);

    if (stage == INITSTAGE_LOCAL) {
        nodeName = par("nodeName").stringValue();
        sumoVehicleId = par("sumoVehicleId").stringValue();
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

            timerMsg = new cMessage("SendVeinsWarningTimer");
            scheduleAt(startTime, timerMsg);
            EV_INFO << "[" << nodeName << " (" << sumoVehicleId << ")] Scheduled emergency alert transmission to "
                    << destAddress << ":" << destPort << " at t = " << startTime << " s\n";
        }
    }
}

void VeinsWarningApp::handleMessage(cMessage *msg)
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

void VeinsWarningApp::sendWarningPacket()
{
    std::string pktName = messageId + "_" + std::to_string(packetsSent + 1);
    Packet *packet = new Packet(pktName.c_str());

    const auto& payload = makeShared<ByteCountChunk>(B(packetSize));
    packet->insertAtBack(payload);

    EV_WARN << "\n=======================================================\n"
            << " [" << nodeName << " (" << sumoVehicleId << ")] TRANSMITTING VEHICULAR WARNING PACKET\n"
            << " Packet Name    : " << pktName << "\n"
            << " Destination IP : " << destAddress << ":" << destPort << "\n"
            << " Hazard Type    : " << warningType << "\n"
            << " Timestamp      : " << simTime() << " s\n"
            << " Routing Engine : AODV Over Dynamic Vehicular Topology\n"
            << "=======================================================\n";

    emit(packetSentSignal, 1);
    socket.sendTo(packet, destAddress, destPort);
}

void VeinsWarningApp::socketDataArrived(UdpSocket *socket, Packet *packet)
{
    simtime_t e2eDelay = simTime() - packet->getCreationTime();

    EV_WARN << "\n*******************************************************\n"
            << " [" << nodeName << " (" << sumoVehicleId << ")] SUCCESS: WARNING PACKET DELIVERED VIA AODV!\n"
            << " Packet Name      : " << packet->getName() << "\n"
            << " Length           : " << packet->getByteLength() << " bytes\n"
            << " End-to-End Delay : " << (e2eDelay * 1000.0) << " ms\n"
            << " Delivery Time    : " << simTime() << " s\n"
            << " On-Board Alert   : FORWARD COLLISION HAZARD DETECTED IN SUMO TRAFFIC!\n"
            << "*******************************************************\n\n";

    emit(e2eDelaySignal, e2eDelay.dbl());
    emit(packetReceivedSignal, 1);
    delete packet;
}

void VeinsWarningApp::socketErrorArrived(UdpSocket *socket, Indication *indication)
{
    EV_ERROR << "[" << nodeName << " (" << sumoVehicleId << ")] Socket error: " << indication->getName() << endl;
    delete indication;
}

void VeinsWarningApp::socketClosed(UdpSocket *socket)
{
    EV_INFO << "[" << nodeName << " (" << sumoVehicleId << ")] Socket closed.\n";
}

void VeinsWarningApp::finish()
{
    EV_INFO << "[" << nodeName << " (" << sumoVehicleId << ")] Application finished.\n";
}

} // namespace veins_integration
