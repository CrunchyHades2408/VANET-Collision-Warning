//
// Collision Warning Application Module
// Phase 3 - AODV VANET Simulation
//

package aodv_simulation.ned;

import inet.applications.contract.IApp;

simple WarningApp like IApp
{
    parameters:
        string nodeName = default("V");
        int nodeId = default(1);
        bool isSender = default(false);
        string destAddress = default("10.0.0.5");
        int destPort = default(5000);
        int localPort = default(5000);
        double startTime @unit(s) = default(15.0s);
        double sendInterval @unit(s) = default(1.0s);
        int packetCount = default(1);
        string warningType = default("SUDDEN_BRAKING_WARNING");
        string messageId = default("WARN_001");
        int packetSize @unit(B) = default(256B);
        @display("i=block/app");

    gates:
        input socketIn @labels(UdpControlInfo/up);
        output socketOut @labels(UdpControlInfo/down);
}
