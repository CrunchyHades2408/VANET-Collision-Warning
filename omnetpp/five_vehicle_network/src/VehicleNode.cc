//
// Vehicle Node Implementation
// Phase 2 - Five Vehicle Communication Network
//

#include "VehicleNode.h"

namespace five_vehicle_network {

Define_Module(VehicleNode);

VehicleNode::VehicleNode()
{
    selfEventMsg = nullptr;
}

VehicleNode::~VehicleNode()
{
    cancelAndDelete(selfEventMsg);
}

void VehicleNode::initialize()
{
    nodeId = par("nodeId");
    nodeName = par("nodeName").stringValue();
    isInitiator = par("isInitiator").boolValue();
    targetNodeId = par("targetNodeId");
    sendTime = par("sendTime").doubleValue();

    e2eDelaySignal = registerSignal("e2eDelay");
    hopCountSignal = registerSignal("hopCount");

    EV_INFO << "[" << nodeName << "] Initialized. Node ID: " << nodeId
            << ", Initiator: " << (isInitiator ? "YES" : "NO") << endl;

    // If this node is configured as the initiator (V1), schedule self-event
    // to simulate collision detection and trigger emergency broadcast
    if (isInitiator) {
        selfEventMsg = new cMessage("TriggerCollisionWarning");
        scheduleAt(sendTime, selfEventMsg);
        EV_INFO << "[" << nodeName << "] Scheduled collision warning broadcast at t = "
                << sendTime << " s." << endl;
    }
}

void VehicleNode::handleMessage(cMessage *msg)
{
    // Case 1: Self-scheduled event triggered (Sudden braking event detected)
    if (msg == selfEventMsg) {
        EV_WARN << "\n=======================================================\n"
                << " [" << nodeName << "] HAZARD DETECTED: SUDDEN BRAKING AT t = "
                << simTime() << " s!\n"
                << " Initiating Multi-hop Collision Warning Message to V" << targetNodeId << "\n"
                << "=======================================================\n";

        CollisionWarningMsg *warningMsg = new CollisionWarningMsg("EMERGENCY_BRAKE_ALERT");
        warningMsg->setSenderId(nodeId);
        warningMsg->setTargetId(targetNodeId);
        warningMsg->setHopCount(0);
        warningMsg->setWarningType("EMERGENCY_BRAKE");
        warningMsg->setEventTime(simTime().dbl());
        warningMsg->setPosX(350.0);   // Matches SUMO position at t=15s
        warningMsg->setPosY(-4.8);    // Lane 0
        warningMsg->setSpeed(13.3 / 3.6); // Decelerating speed

        // Transmit to the immediate downstream vehicle via gRight
        EV_INFO << "[" << nodeName << "] Transmitting alert packet out gate 'gRight$o' -> Hop 1\n";
        send(warningMsg, "gRight$o");
        return;
    }

    // Case 2: Incoming network message received on a communication gate
    CollisionWarningMsg *warningMsg = dynamic_cast<CollisionWarningMsg *>(msg);
    if (!warningMsg) {
        EV_ERROR << "[" << nodeName << "] Received unknown message type. Discarding." << endl;
        delete msg;
        return;
    }

    int senderId = warningMsg->getSenderId();
    int targetId = warningMsg->getTargetId();
    int currentHops = warningMsg->getHopCount() + 1;
    warningMsg->setHopCount(currentHops);

    simtime_t e2eDelay = simTime() - warningMsg->getEventTime();

    EV_INFO << "[" << nodeName << "] RECEIVED Packet '" << warningMsg->getName() << "'\n"
            << "   Originator : Vehicle V" << senderId << "\n"
            << "   Destination: Vehicle V" << targetId << "\n"
            << "   Current Hop: " << currentHops << "\n"
            << "   Elapsed E2E Delay: " << (e2eDelay * 1000.0) << " ms\n";

    // Check if this node is the intended recipient
    if (nodeId == targetId) {
        EV_WARN << "\n*******************************************************\n"
                << " [" << nodeName << "] SUCCESS: ALERT DELIVERED TO TARGET!\n"
                << " Incident Origin : Vehicle V" << senderId << "\n"
                << " Total Hops      : " << currentHops << "\n"
                << " Total Latency   : " << (e2eDelay * 1000.0) << " ms\n"
                << " Action Taken    : Forward Collision Warning (FCW) Displayed to Driver\n"
                << "*******************************************************\n\n";

        emit(e2eDelaySignal, e2eDelay.dbl());
        emit(hopCountSignal, currentHops);

        delete warningMsg;
    }
    else {
        // Forward along the multi-hop path towards destination
        forwardMessage(warningMsg);
    }
}

void VehicleNode::forwardMessage(CollisionWarningMsg *warningMsg)
{
    EV_INFO << "[" << nodeName << "] FORWARDING message to next hop towards V"
            << warningMsg->getTargetId() << " (Hop " << warningMsg->getHopCount() << ")\n";

    // In linear highway topology:
    // If target is to the right (targetId > nodeId), forward via gRight
    // If target is to the left (targetId < nodeId), forward via gLeft
    if (warningMsg->getTargetId() > nodeId) {
        if (gate("gRight$o")->isConnected()) {
            send(warningMsg, "gRight$o");
        } else {
            EV_ERROR << "[" << nodeName << "] Cannot forward: gRight is not connected!" << endl;
            delete warningMsg;
        }
    } else {
        if (gate("gLeft$o")->isConnected()) {
            send(warningMsg, "gLeft$o");
        } else {
            EV_ERROR << "[" << nodeName << "] Cannot forward: gLeft is not connected!" << endl;
            delete warningMsg;
        }
    }
}

void VehicleNode::finish()
{
    EV_INFO << "[" << nodeName << "] Simulation finished. Module terminated cleanly.\n";
}

} // namespace five_vehicle_network
