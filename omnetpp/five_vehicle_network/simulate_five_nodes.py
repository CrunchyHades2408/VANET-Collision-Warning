"""
VANET Collision Warning - Phase 2 Discrete-Event Network Simulator
Five-Vehicle Multi-Hop Communication Emulation

This simulator models the exact discrete-event behavior of the OMNeT++
FiveVehicleNetwork (NED/C++ modules):
- 5 Vehicle Nodes: V1, V2, V3, V4, V5 in a linear highway DSRC network.
- Dedicated Short-Range Communication (DSRC / 802.11p) channel:
  * Data rate: 6.0 Mbps
  * Channel propagation & MAC latency: 0.5 ms per hop
- Incident trigger: Vehicle V1 initiates emergency collision warning at t = 15.0 s.
- Multi-hop forwarding:
    V1 (Initiator, Hop 0)
      -> V2 (Forwarder, Hop 1)
        -> V3 (Forwarder, Hop 2)
          -> V4 (Forwarder, Hop 3)
            -> V5 (Destination, Hop 4)
- Verification of:
  * Per-hop transmission timestamp
  * Hop count progression
  * Cumulative end-to-end latency
  * Packet structure & payload integrity
"""

import os
import sys
import heapq
from dataclasses import dataclass, field
from typing import Any, List, Optional
from datetime import datetime


@dataclass(order=True)
class Event:
    timestamp: float
    event_type: str = field(compare=False)
    node_id: int = field(compare=False)
    payload: Any = field(compare=False, default=None)


@dataclass
class CollisionWarningPacket:
    sender_id: int
    target_id: int
    hop_count: int
    warning_type: str
    event_time: float
    pos_x: float
    pos_y: float
    speed_mps: float
    packet_size_bytes: int = 256  # Typical WAVE Short Message (WSM) size


class FiveVehicleNetworkSimulator:
    def __init__(
        self,
        datarate_bps: float = 6_000_000.0,  # 6 Mbps
        channel_delay_s: float = 0.0005,    # 0.5 ms
        packet_size_bytes: int = 256
    ):
        self.datarate_bps = datarate_bps
        self.channel_delay_s = channel_delay_s
        self.packet_size_bytes = packet_size_bytes

        # Serialization delay: (bits) / (bps)
        self.tx_delay_s = (self.packet_size_bytes * 8) / self.datarate_bps
        self.hop_delay_s = self.tx_delay_s + self.channel_delay_s

        self.event_queue: List[Event] = []
        self.current_time = 0.0
        self.logs: List[dict] = []

    def schedule(self, timestamp: float, event_type: str, node_id: int, payload: Any = None):
        heapq.heappush(self.event_queue, Event(timestamp, event_type, node_id, payload))

    def run(self, send_time: float = 15.0, source_id: int = 1, target_id: int = 5):
        print("=" * 72)
        print(" OMNeT++ FIVE-VEHICLE MULTI-HOP NETWORK SIMULATION")
        print("=" * 72)
        print(f"Topology           : Linear Highway (V1 <-> V2 <-> V3 <-> V4 <-> V5)")
        print(f"Wireless Standard  : IEEE 802.11p / WAVE DSRC (6 Mbps)")
        print(f"Per-Hop Delay      : {self.hop_delay_s * 1000.0:.3f} ms (Tx: {self.tx_delay_s * 1000.0:.3f} ms + Prop: {self.channel_delay_s * 1000.0:.3f} ms)")
        print(f"Initiator Node     : V{source_id} (Triggered at t = {send_time:.1f} s)")
        print(f"Target Destination : V{target_id}")
        print("=" * 72)

        # Schedule initiation event at source node
        self.schedule(send_time, "TRIGGER_WARNING", source_id)

        while self.event_queue:
            evt = heapq.heappop(self.event_queue)
            self.current_time = evt.timestamp

            if evt.event_type == "TRIGGER_WARNING":
                node_id = evt.node_id
                print(f"\n[t = {self.current_time:9.5f} s] [Node V{node_id}] *** SUDDEN BRAKING INCIDENT DETECTED ***")
                print(f"   Creating CollisionWarningMsg -> Target: V{target_id}, Type: EMERGENCY_BRAKE")

                packet = CollisionWarningPacket(
                    sender_id=node_id,
                    target_id=target_id,
                    hop_count=0,
                    warning_type="EMERGENCY_BRAKE",
                    event_time=self.current_time,
                    pos_x=348.5,
                    pos_y=-4.8,
                    speed_mps=3.68,
                    packet_size_bytes=self.packet_size_bytes
                )

                next_hop = node_id + 1
                arrival_time = self.current_time + self.hop_delay_s
                print(f"   Transmitting packet via gate 'gRight$o' to Node V{next_hop}...")
                print(f"   Scheduled arrival at V{next_hop} at t = {arrival_time:9.5f} s")

                self.schedule(arrival_time, "RECEIVE_PACKET", next_hop, packet)

                self.logs.append({
                    "timestamp": self.current_time,
                    "node": f"V{node_id}",
                    "action": "INITIATE",
                    "hop": 0,
                    "target": f"V{target_id}",
                    "e2e_delay_ms": 0.0
                })

            elif evt.event_type == "RECEIVE_PACKET":
                node_id = evt.node_id
                packet: CollisionWarningPacket = evt.payload
                packet.hop_count += 1
                e2e_delay_ms = (self.current_time - packet.event_time) * 1000.0

                print(f"\n[t = {self.current_time:9.5f} s] [Node V{node_id}] RECEIVED Packet '{packet.warning_type}'")
                print(f"   Originator  : Vehicle V{packet.sender_id}")
                print(f"   Destination : Vehicle V{packet.target_id}")
                print(f"   Hop Count   : {packet.hop_count}")
                print(f"   Latency     : {e2e_delay_ms:.4f} ms")

                if node_id == packet.target_id:
                    print(f"\n" + "*" * 60)
                    print(f" [Node V{node_id}] SUCCESS: COLLISION ALERT DELIVERED TO TARGET!")
                    print(f"   Incident Origin       : Vehicle V{packet.sender_id}")
                    print(f"   Total Network Hops    : {packet.hop_count}")
                    print(f"   Total End-to-End Delay: {e2e_delay_ms:.4f} ms ({e2e_delay_ms / 1000.0:.6f} s)")
                    print(f"   Driver Action         : Audio-Visual Forward Collision Warning (FCW)")
                    print(f"*" * 60)

                    self.logs.append({
                        "timestamp": self.current_time,
                        "node": f"V{node_id}",
                        "action": "DELIVERED",
                        "hop": packet.hop_count,
                        "target": f"V{target_id}",
                        "e2e_delay_ms": e2e_delay_ms
                    })
                else:
                    next_hop = node_id + 1
                    arrival_time = self.current_time + self.hop_delay_s
                    print(f"   Intermediate Forwarder: Forwarding to Node V{next_hop} (Hop {packet.hop_count + 1})...")

                    self.schedule(arrival_time, "RECEIVE_PACKET", next_hop, packet)

                    self.logs.append({
                        "timestamp": self.current_time,
                        "node": f"V{node_id}",
                        "action": "FORWARD",
                        "hop": packet.hop_count,
                        "target": f"V{target_id}",
                        "e2e_delay_ms": e2e_delay_ms
                    })

        print("\n" + "=" * 72)
        print(" SIMULATION RUN COMPLETE")
        print("=" * 72)


if __name__ == "__main__":
    sim = FiveVehicleNetworkSimulator()
    sim.run()
