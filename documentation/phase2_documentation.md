# Phase 2 Documentation: OMNeT++ Fundamentals and Five-Node Network

## 1. Introduction to OMNeT++ Discrete-Event Simulation
OMNeT++ (Objective Modular Network Tested in C++) is an extensible, component-based C++ simulation framework primarily used for building network simulators.

### 1.1 Core File Types in OMNeT++
1. **Network Description Files (`.ned`)**:
   - Written in the NED language.
   - Declaratively define network structure, hierarchy, modules, channels, parameters, and gate interconnections.
   - Separate topology configuration from C++ operational code.
2. **Simulation Configuration Files (`.ini`)**:
   - Supply runtime parameters to modules.
   - Select which network definition to instantiate.
   - Set simulation execution limits (`sim-time-limit`, `cpu-time-limit`), seed values, and output collection modes (`.vec`, `.sca`).
3. **Message Definition Files (`.msg`)**:
   - High-level descriptions of packet structures, fields, and datatypes.
   - Compiled by OMNeT++'s `opp_msgc` tool into standard C++ header (`_m.h`) and implementation (`_m.cc`) classes with automatic memory management, getters, and setters.
4. **C++ Implementation Files (`.h` / `.cc`)**:
   - Provide the imperative behavioral logic for simple modules.
   - Subclass `cSimpleModule` and implement core lifecycle methods: `initialize()`, `handleMessage(cMessage *msg)`, and `finish()`.

---

## 2. Five-Vehicle Multi-Hop Network Architecture

### 2.1 Network Topology (`FiveVehicleNetwork.ned`)
The simulation represents five sequential vehicles traveling along the simulated highway:
$$\text{V1} \longleftrightarrow \text{V2} \longleftrightarrow \text{V3} \longleftrightarrow \text{V4} \longleftrightarrow \text{V5}$$
- **Channel**: `HighwayDSRCChannel`
  - **Standard**: IEEE 802.11p / WAVE Dedicated Short-Range Communications (DSRC)
  - **Data Rate**: $6.0\text{ Mbps}$
  - **Propagation Delay**: $0.5\text{ ms}$
  - **Bit Error Rate**: $0.0$ (baseline ideal channel)

### 2.2 Packet Specification (`CollisionWarningMsg.msg`)
The message payload mirrors real-world European ETSI Cooperative Awareness Messages (CAM) and Decentralized Environmental Notification Messages (DENM):
```protobuf
message CollisionWarningMsg {
    int senderId;           // Originating vehicle (V1)
    int targetId;           // Intended destination (V5)
    int hopCount = 0;       // Incremented by each forwarder
    string warningType;     // "EMERGENCY_BRAKE"
    double eventTime;       // Origin timestamp (15.0 s)
    double posX;            // Highway coordinate X (348.5 m)
    double posY;            // Highway coordinate Y (-4.8 m)
    double speed;           // Origin vehicle speed (3.68 m/s)
}
```

### 2.3 Modular Behavioral Logic (`VehicleNode.cc`)
1. **Self-Message Scheduling**:
   - Vehicle `V1` is designated as the initiator (`isInitiator = true`).
   - At initialization, `V1` schedules an internal self-event `cMessage("TriggerCollisionWarning")` at simulation time $t = 15.0\text{ s}$, directly synchronizing with the sudden-braking event from Phase 1.
2. **Hazard Generation ($t = 15.0\text{ s}$)**:
   - Upon timer expiry, `V1` creates `CollisionWarningMsg`, populates its telemetry fields, and transmits it via `send(warningMsg, "gRight$o")` to `V2`.
3. **Multi-Hop Forwarding ($V2 \to V3 \to V4$)**:
   - Each intermediate vehicle receives the message, inspects `targetId`, increments `hopCount`, records elapsed latency, and forwards the packet downstream.
4. **Target Reception ($V5$)**:
   - Vehicle `V5` detects `nodeId == targetId`, terminates forwarding, logs end-to-end delivery metrics, and triggers the Forward Collision Warning alert.

---

## 3. Multi-Hop Transmission Trace and Performance Results

### 3.1 Step-by-Step Delivery Log
| Step | Simulation Time (s) | Node | Action | Hop Count | Hop Latency (ms) | Cumulative Latency (ms) |
|---|---|---|---|---|---|---|
| **0** | **15.00000** | **V1** | Hazard Detected & Alert Dispatched | 0 | 0.000 | 0.0000 |
| **1** | **15.00084** | **V2** | Packet Received & Forwarded | 1 | 0.841 | 0.8413 |
| **2** | **15.00168** | **V3** | Packet Received & Forwarded | 2 | 0.841 | 1.6827 |
| **3** | **15.00252** | **V4** | Packet Received & Forwarded | 3 | 0.841 | 2.5240 |
| **4** | **15.00337** | **V5** | **Alert Delivered to Destination** | **4** | 0.841 | **3.3653** |

### 3.2 Latency Breakdown
- **Packet Size**: $256\text{ bytes} = 2048\text{ bits}$.
- **Transmission (Serialization) Delay**:
  $$t_{\text{tx}} = \frac{2048\text{ bits}}{6 \times 10^6\text{ bps}} \approx 0.3413\text{ ms}$$
- **Propagation Delay**: $t_{\text{prop}} = 0.5000\text{ ms}$.
- **Single-Hop Delay**: $t_{\text{hop}} = t_{\text{tx}} + t_{\text{prop}} = 0.8413\text{ ms}$.
- **Total End-to-End Delay (4 Hops)**:
  $$T_{\text{total}} = 4 \times 0.8413\text{ ms} = 3.3653\text{ ms} \approx 0.00337\text{ s}$$

> **Key Finding**: In vehicular ad-hoc networks, wireless multi-hop packet delivery takes approximately **3.4 milliseconds** over 4 hops. Compared to human reaction time ($\approx 1.0 - 1.5\text{ seconds}$), multi-hop wireless warnings reach downstream vehicles virtually instantaneously, providing drivers and automated braking systems with vital seconds of advanced warning.

---

## 4. Fundamental Networking & Routing Concepts for Phase 3

### 4.1 Unicast vs. Broadcast
- **Broadcast**: Message is transmitted to all neighboring nodes within radio transmission range without addressing a specific recipient. Essential for safety beacons (CAMs) and route discovery queries (RREQs).
- **Unicast**: Message is directed to a specific target network address. Used for payload delivery (DENMs) and route replies (RREPs).

### 4.2 Proactive vs. Reactive Routing
- **Proactive (Table-Driven, e.g. OLSR, DSDV)**: Nodes continuously maintain background routing tables for all possible destinations. High routing overhead in dynamic VANETs due to rapid topological changes.
- **Reactive (On-Demand, e.g. AODV, DSR)**: Routes are established strictly when a source needs to communicate with a destination. Highly bandwidth-efficient for mobile ad-hoc environments.

### 4.3 AODV (Ad hoc On-Demand Distance Vector) Routing Architecture
AODV establishes dynamic, loop-free multi-hop wireless routes through four key phases:

```
[Source Node]                                         [Destination Node]
      |                                                        |
      |--------- Broadcast RREQ (Hop 1) -------> [Intermediate] |
      |                                                |       |
      |                                                |--- RREQ (Hop 2) ---> [Destination]
      |                                                |       |
      |                                                |<-- Unicast RREP ---- [Destination]
      |<-------- Unicast RREP (Hop 1) -------- [Intermediate] |
      |                                                        |
   === Route Established: Forward Data Packets along Path ===
      |=======================================================>|
```

1. **Route Request (RREQ)**:
   - When a source node requires a route to an unknown destination, it broadcasts an `RREQ` packet containing `<Source IP, Source Sequence Number, Broadcast ID, Destination IP, Destination Sequence Number, Hop Count>`.
2. **Reverse Path Setup**:
   - Each intermediate node that receives the `RREQ` records a reverse route entry pointing back toward the source node.
3. **Route Reply (RREP)**:
   - When the destination (or an intermediate node with an active, fresh route) receives the `RREQ`, it unicasts an `RREP` packet back along the established reverse path to the source.
4. **Forward Path Setup**:
   - As the `RREP` travels back to the source, intermediate nodes create forward route entries pointing toward the destination.
5. **Route Maintenance & Route Error (RERR)**:
   - In dynamic VANETs, if a vehicle moves out of range, the link breaks. The detecting node invalidates the route and broadcasts an `RERR` packet to upstream nodes, prompting a fresh route discovery.
