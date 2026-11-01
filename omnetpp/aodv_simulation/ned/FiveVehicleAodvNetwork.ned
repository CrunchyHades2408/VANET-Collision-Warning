//
// Five Vehicle Wireless Highway Network Topology with AODV Routing
// Phase 3 - AODV Multi-Hop VANET Simulation
//

package aodv_simulation.ned;

import inet.networklayer.configurator.ipv4.Ipv4NetworkConfigurator;
import inet.physicallayer.wireless.unitdisk.UnitDiskRadioMedium;
import inet.visualizer.canvas.integrated.IntegratedCanvasVisualizer;

network FiveVehicleAodvNetwork
{
    parameters:
        @display("bgb=1300,300;bgu=m");

    submodules:
        radioMedium: UnitDiskRadioMedium {
            parameters:
                @display("p=80,50");
        }

        configurator: Ipv4NetworkConfigurator {
            parameters:
                config = xml("<config><interface hosts='v*' address='10.0.0.x' netmask='255.255.255.0'/></config>");
                @display("p=220,50");
        }

        visualizer: IntegratedCanvasVisualizer {
            parameters:
                @display("p=360,50");
        }

        v1: AodvVehicle {
            parameters:
                @display("p=100,180;i=device/car,red");
        }

        v2: AodvVehicle {
            parameters:
                @display("p=350,180;i=device/car,gold");
        }

        v3: AodvVehicle {
            parameters:
                @display("p=600,180;i=device/car,blue");
        }

        v4: AodvVehicle {
            parameters:
                @display("p=850,180;i=device/car,cyan");
        }

        v5: AodvVehicle {
            parameters:
                @display("p=1100,180;i=device/car,green");
        }
}
