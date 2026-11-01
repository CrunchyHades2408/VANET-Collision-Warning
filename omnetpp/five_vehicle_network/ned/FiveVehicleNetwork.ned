//
// Five Vehicle Highway Network Topology
// Phase 2 - Multi-hop Communication Topology
//

package five_vehicle_network.ned;

network FiveVehicleNetwork
{
    parameters:
        @display("bgb=700,200;bgu=m");

    types:
        channel HighwayDSRCChannel extends ned.DatarateChannel
        {
            datarate = 6Mbps;      // Dedicated Short-Range Communications (DSRC / 802.11p)
            delay = 0.5ms;         // Radio propagation and PHY/MAC transmission delay
            ber = 0.0;             // Bit error rate for baseline test
        }

    submodules:
        v1: VehicleNode {
            parameters:
                nodeId = 1;
                nodeName = "V1";
                isInitiator = true;
                targetNodeId = 5;
                sendTime = 15.0s;
                @display("p=100,100;i=device/car,red");
        }
        v2: VehicleNode {
            parameters:
                nodeId = 2;
                nodeName = "V2";
                isInitiator = false;
                @display("p=220,100;i=device/car,gold");
        }
        v3: VehicleNode {
            parameters:
                nodeId = 3;
                nodeName = "V3";
                isInitiator = false;
                @display("p=340,100;i=device/car,blue");
        }
        v4: VehicleNode {
            parameters:
                nodeId = 4;
                nodeName = "V4";
                isInitiator = false;
                @display("p=460,100;i=device/car,cyan");
        }
        v5: VehicleNode {
            parameters:
                nodeId = 5;
                nodeName = "V5";
                isInitiator = false;
                @display("p=580,100;i=device/car,green");
        }

    connections:
        v1.gRight <--> HighwayDSRCChannel <--> v2.gLeft;
        v2.gRight <--> HighwayDSRCChannel <--> v3.gLeft;
        v3.gRight <--> HighwayDSRCChannel <--> v4.gLeft;
        v4.gRight <--> HighwayDSRCChannel <--> v5.gLeft;
}
