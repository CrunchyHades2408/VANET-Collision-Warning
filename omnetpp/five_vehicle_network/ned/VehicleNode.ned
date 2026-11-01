//
// Vehicle Node Simple Module Definition
// Phase 2 - Five Vehicle Communication Network
//

package five_vehicle_network.ned;

simple VehicleNode
{
    parameters:
        int nodeId;                        // Numeric ID of this vehicle (1 to 5)
        string nodeName = default("V");    // Display name prefix
        bool isInitiator = default(false); // True if this vehicle triggers warning
        int targetNodeId = default(5);     // Target destination vehicle ID
        double sendTime @unit(s) = default(15.0s); // When to transmit warning
        @display("i=device/car");

    gates:
        inout gLeft;   // Connection to preceding vehicle (towards V1)
        inout gRight;  // Connection to following vehicle (towards V5)
}
