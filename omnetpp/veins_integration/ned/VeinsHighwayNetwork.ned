//
// Veins Highway Co-Simulation Network with Dynamic SUMO Traffic & AODV Routing
// Phase 4 - SUMO-OMNeT++ Veins Integration
//

package veins_integration.ned;

import inet.networklayer.configurator.ipv4.Ipv4NetworkConfigurator;
import inet.physicallayer.wireless.unitdisk.UnitDiskRadioMedium;
import inet.visualizer.canvas.integrated.IntegratedCanvasVisualizer;
import org.car2x.veins.subprojects.veins_inet.TraCIScenarioManagerFork;

network VeinsHighwayNetwork
{
    parameters:
        @display("bgb=3500,500;bgu=m");

    submodules:
        radioMedium: UnitDiskRadioMedium {
            parameters:
                @display("p=100,60");
        }

        configurator: Ipv4NetworkConfigurator {
            parameters:
                config = xml("<config><interface hosts='vehicle_*' address='10.0.0.x' netmask='255.255.255.0'/></config>");
                @display("p=260,60");
        }

        manager: TraCIScenarioManagerFork {
            parameters:
                connectAt = 0s;
                updateInterval = 0.1s;
                moduleType = "veins_integration.ned.VeinsAodvVehicle";
                moduleName = "vehicle_";
                host = "localhost";
                port = 9999;
                configFile = "highway.sumocfg";
                autoShutdown = true;
                @display("p=420,60");
        }

        visualizer: IntegratedCanvasVisualizer {
            parameters:
                @display("p=580,60");
        }
}
