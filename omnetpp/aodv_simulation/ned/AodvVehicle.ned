//
// AODV Vehicle Node Module
// Phase 3 - AODV VANET Simulation
//

package aodv_simulation.ned;

import inet.node.aodv.AodvRouter;

module AodvVehicle extends AodvRouter
{
    parameters:
        @display("i=device/car");
        numWlanInterfaces = default(1);
        wlan[0].typename = default("AckingWirelessInterface");
        wlan[0].radio.typename = default("UnitDiskRadio");
        wlan[0].mac.typename = default("AckingMac");
}
