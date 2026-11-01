//
// Veins Collision Warning Application Module
// Phase 4 - SUMO-OMNeT++ Veins Integration
//

package veins_integration.ned;

import inet.applications.contract.IApp;

simple VeinsWarningApp like IApp
{
    parameters:
        string nodeName = default("vehicle_");
        string sumoVehicleId = default("V1");
        bool isSender = default(false);
        string destAddress = default("10.0.0.5");
        int destPort = default(5000);
        int localPort = default(5000);
        double startTime @unit(s) = default(15.0s);
        double sendInterval @unit(s) = default(1.0s);
        int packetCount = default(5);
        string warningType = default("SUDDEN_BRAKING_WARNING");
        string messageId = default("VEINS_WARN_001");
        int packetSize @unit(B) = default(256B);
        @display("i=block/app");

    gates:
        input socketIn @labels(UdpControlInfo/up);
        output socketOut @labels(UdpControlInfo/down);
}
