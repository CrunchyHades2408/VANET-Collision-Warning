//
// Dynamic Veins Vehicle Module with AODV Routing
// Phase 4 - SUMO-OMNeT++ Veins Integration
//

package veins_integration.ned;

import inet.node.aodv.AodvRouter;

module VeinsAodvVehicle extends AodvRouter
{
    parameters:
        @display("i=device/car");
        mobility.typename = default("TraCIMobility");
        numWlanInterfaces = default(1);
        wlan[0].typename = default("AckingWirelessInterface");
        wlan[0].radio.typename = default("UnitDiskRadio");
        wlan[0].mac.typename = default("AckingMac");
}
