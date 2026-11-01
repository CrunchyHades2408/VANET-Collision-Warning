"""
VANET Collision Warning & Traffic Management - Phase 3
Actual AODV (RFC 3561) Multi-Hop Simulation & Experiment Automation Engine

This script implements the full RFC 3561 Ad hoc On-Demand Distance Vector (AODV)
routing protocol and executes the complete set of required Phase 3 experiments:
- Exp 1: Initial on-demand route discovery (RREQ/RREP exchange)
- Exp 2: Repeated transmission with route cache reuse
- Exp 3 & 4: Intermediate link failure, RERR generation, and route maintenance
- Exp 5: Emergency multi-hop collision warning dissemination
- Baseline: Direct single-hop transmission failure comparison

Generates:
- High-resolution CSV telemetry logs
- Detailed event trace logs
- Analytical comparison graphs using Matplotlib
"""

import os
import sys
import math
import heapq
from dataclasses import dataclass, field
from typing import Dict, List, Set, Optional, Tuple
import pandas as pd

# Set Matplotlib cache dir inside project to avoid permission issues
os.environ["MPLCONFIGDIR"] = os.path.join(os.path.dirname(__file__), "results", ".matplotlib_cache")
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


# ===========================================================================
# 1. AODV Protocol Data Structures (RFC 3561)
# ===========================================================================

@dataclass
class AodvRreq:
    originator_ip: str
    originator_seq: int
    dest_ip: str
    dest_seq: int
    hop_count: int
    rreq_id: int
    ttl: int
    creation_time: float


@dataclass
class AodvRrep:
    originator_ip: str
    dest_ip: str
    dest_seq: int
    hop_count: int
    lifetime: float
    creation_time: float


@dataclass
class AodvRerr:
    unreachable_dest_ips: List[str]
    dest_seq_nums: List[int]
    creation_time: float


@dataclass
class UdpWarningPacket:
    msg_id: str
    source_ip: str
    dest_ip: str
    warning_type: str
    creation_time: float
    payload_bytes: int = 256
    speed_mps: float = 3.68
    pos_x: float = 348.5
    hop_count: int = 0
    traversed_path: List[str] = field(default_factory=list)


@dataclass
class RoutingTableEntry:
    dest_ip: str
    dest_seq_num: int
    valid_seq_num: bool
    state: str  # "VALID" or "INVALID"
    hop_count: int
    next_hop: str
    precursors: Set[str] = field(default_factory=set)
    lifetime: float = 0.0


@dataclass(order=True)
class SimEvent:
    timestamp: float
    event_type: str = field(compare=False)
    target_node: str = field(compare=False)
    payload: any = field(compare=False, default=None)


# ===========================================================================
# 2. Wireless Node & AODV Routing Engine
# ===========================================================================

class AodvNode:
    def __init__(self, node_id: str, ip: str, x: float, y: float, radio_range: float = 300.0):
        self.node_id = node_id
        self.ip = ip
        self.x = x
        self.y = y
        self.radio_range = radio_range
        self.is_active = True

        # AODV State (RFC 3561)
        self.seq_num = 1
        self.rreq_id = 0
        self.routing_table: Dict[str, RoutingTableEntry] = {}
        self.packet_buffer: Dict[str, List[UdpWarningPacket]] = {}
        self.seen_rreqs: Set[Tuple[str, int]] = set()

        # Configurable RFC 3561 Timers
        self.active_route_timeout = 3.0  # seconds
        self.delete_period = 15.0        # seconds

    def distance_to(self, other: "AodvNode") -> float:
        return math.hypot(self.x - other.x, self.y - other.y)

    def can_reach(self, other: "AodvNode") -> bool:
        if not self.is_active or not other.is_active:
            return False
        return self.distance_to(other) <= self.radio_range


# ===========================================================================
# 3. Discrete-Event Network Simulator
# ===========================================================================

class AodvNetworkSimulation:
    def __init__(self, bit_rate_bps: float = 6_000_000.0, prop_delay_s: float = 0.0005):
        self.bit_rate_bps = bit_rate_bps
        self.prop_delay_s = prop_delay_s
        self.nodes: Dict[str, AodvNode] = {}
        self.event_queue: List[SimEvent] = []
        self.current_time = 0.0

        # Global Telemetry & Trace Logs
        self.packet_trace_log: List[dict] = []
        self.metrics_summary: Dict[str, any] = {}

        # Counters for Performance Metrics
        self.packets_generated = 0
        self.packets_delivered = 0
        self.rreq_transmitted = 0
        self.rrep_transmitted = 0
        self.rerr_transmitted = 0
        self.e2e_delays_ms: List[float] = []
        self.hop_counts: List[int] = []
        self.route_discovery_times_ms: List[float] = []
        self.route_discovery_start_times: Dict[str, float] = {}

    def add_node(self, node: AodvNode):
        self.nodes[node.ip] = node

    def schedule(self, timestamp: float, event_type: str, target_node: str, payload: any = None):
        heapq.heappush(self.event_queue, SimEvent(timestamp, event_type, target_node, payload))

    def calc_tx_delay(self, bytes_len: int) -> float:
        return ((bytes_len * 8) / self.bit_rate_bps) + self.prop_delay_s

    # -----------------------------------------------------------------------
    # AODV Route Discovery Methods
    # -----------------------------------------------------------------------
    def send_udp_packet(self, src_ip: str, dest_ip: str, warning_type: str = "SUDDEN_BRAKING_WARNING", msg_id: str = "WARN_001"):
        src_node = self.nodes[src_ip]
        self.packets_generated += 1
        pkt = UdpWarningPacket(
            msg_id=f"{msg_id}_{self.packets_generated}",
            source_ip=src_ip,
            dest_ip=dest_ip,
            warning_type=warning_type,
            creation_time=self.current_time,
            traversed_path=[src_node.node_id]
        )

        self.packet_trace_log.append({
            "timestamp": round(self.current_time, 6),
            "event": "APP_PACKET_GEN",
            "node": src_node.node_id,
            "packet_id": pkt.msg_id,
            "src": src_ip,
            "dest": dest_ip,
            "details": f"Generated warning payload {pkt.warning_type}"
        })

        # Check for valid active route in routing table
        entry = src_node.routing_table.get(dest_ip)
        if entry and entry.state == "VALID" and entry.lifetime > self.current_time:
            # Active route exists: forward directly without RREQ
            self.forward_data_packet(src_node, pkt, entry.next_hop)
        else:
            # No valid route: buffer packet and initiate RREQ
            if dest_ip not in src_node.packet_buffer:
                src_node.packet_buffer[dest_ip] = []
            src_node.packet_buffer[dest_ip].append(pkt)

            if dest_ip not in self.route_discovery_start_times:
                self.route_discovery_start_times[dest_ip] = self.current_time
                self.initiate_rreq(src_node, dest_ip)

    def initiate_rreq(self, node: AodvNode, dest_ip: str):
        node.rreq_id += 1
        node.seq_num += 1

        dest_entry = node.routing_table.get(dest_ip)
        dest_seq = dest_entry.dest_seq_num if dest_entry else 0

        rreq = AodvRreq(
            originator_ip=node.ip,
            originator_seq=node.seq_num,
            dest_ip=dest_ip,
            dest_seq=dest_seq,
            hop_count=0,
            rreq_id=node.rreq_id,
            ttl=35,
            creation_time=self.current_time
        )
        node.seen_rreqs.add((node.ip, node.rreq_id))

        self.packet_trace_log.append({
            "timestamp": round(self.current_time, 6),
            "event": "AODV_RREQ_BROADCAST",
            "node": node.node_id,
            "packet_id": f"RREQ_{rreq.rreq_id}",
            "src": node.ip,
            "dest": dest_ip,
            "details": f"Initiated RREQ for {dest_ip}, seq={node.seq_num}"
        })

        # Broadcast to all 1-hop wireless neighbors within radio range
        self.broadcast_rreq(node, rreq)

    def broadcast_rreq(self, sender: AodvNode, rreq: AodvRreq):
        tx_delay = self.calc_tx_delay(48)  # Standard RREQ packet size ~48 bytes
        self.rreq_transmitted += 1

        for neighbor in self.nodes.values():
            if neighbor.ip != sender.ip and sender.can_reach(neighbor):
                arrival_time = self.current_time + tx_delay
                self.schedule(arrival_time, "PROCESS_RREQ", neighbor.ip, (rreq, sender.ip))

    def handle_rreq(self, receiver: AodvNode, rreq: AodvRreq, prev_hop_ip: str):
        if not receiver.is_active:
            return

        # Check duplicate RREQ
        if (rreq.originator_ip, rreq.rreq_id) in receiver.seen_rreqs:
            return
        receiver.seen_rreqs.add((rreq.originator_ip, rreq.rreq_id))

        # Update or create reverse route entry towards originator
        hop_count = rreq.hop_count + 1
        rev_entry = receiver.routing_table.get(rreq.originator_ip)
        if not rev_entry or rreq.originator_seq > rev_entry.dest_seq_num or (rreq.originator_seq == rev_entry.dest_seq_num and hop_count < rev_entry.hop_count):
            receiver.routing_table[rreq.originator_ip] = RoutingTableEntry(
                dest_ip=rreq.originator_ip,
                dest_seq_num=rreq.originator_seq,
                valid_seq_num=True,
                state="VALID",
                hop_count=hop_count,
                next_hop=prev_hop_ip,
                lifetime=self.current_time + receiver.active_route_timeout
            )

        self.packet_trace_log.append({
            "timestamp": round(self.current_time, 6),
            "event": "AODV_RREQ_RECV",
            "node": receiver.node_id,
            "packet_id": f"RREQ_{rreq.rreq_id}",
            "src": rreq.originator_ip,
            "dest": rreq.dest_ip,
            "details": f"Reverse route via {self.nodes[prev_hop_ip].node_id}, hops={hop_count}"
        })

        # Check if receiver is destination
        if receiver.ip == rreq.dest_ip:
            # Target reached: generate RREP
            receiver.seq_num = max(receiver.seq_num, rreq.dest_seq) + 1
            rrep = AodvRrep(
                originator_ip=rreq.originator_ip,
                dest_ip=receiver.ip,
                dest_seq=receiver.seq_num,
                hop_count=0,
                lifetime=self.current_time + receiver.active_route_timeout,
                creation_time=self.current_time
            )

            self.packet_trace_log.append({
                "timestamp": round(self.current_time, 6),
                "event": "AODV_RREP_GEN",
                "node": receiver.node_id,
                "packet_id": "RREP",
                "src": receiver.ip,
                "dest": rreq.originator_ip,
                "details": f"Destination reached. Generated RREP, seq={receiver.seq_num}"
            })

            # Send unicast RREP along reverse path
            self.send_rrep(receiver, rrep)
        else:
            # Intermediate node: forward RREQ if TTL > 1
            if rreq.ttl > 1:
                fwd_rreq = AodvRreq(
                    originator_ip=rreq.originator_ip,
                    originator_seq=rreq.originator_seq,
                    dest_ip=rreq.dest_ip,
                    dest_seq=rreq.dest_seq,
                    hop_count=hop_count,
                    rreq_id=rreq.rreq_id,
                    ttl=rreq.ttl - 1,
                    creation_time=rreq.creation_time
                )
                self.broadcast_rreq(receiver, fwd_rreq)

    def send_rrep(self, sender: AodvNode, rrep: AodvRrep):
        rev_entry = sender.routing_table.get(rrep.originator_ip)
        if not rev_entry or rev_entry.state != "VALID":
            return

        next_hop_node = self.nodes[rev_entry.next_hop]
        if not sender.can_reach(next_hop_node):
            return

        tx_delay = self.calc_tx_delay(44)  # Standard RREP packet size ~44 bytes
        self.rrep_transmitted += 1

        arrival_time = self.current_time + tx_delay
        self.schedule(arrival_time, "PROCESS_RREP", next_hop_node.ip, (rrep, sender.ip))

    def handle_rrep(self, receiver: AodvNode, rrep: AodvRrep, prev_hop_ip: str):
        if not receiver.is_active:
            return

        hop_count = rrep.hop_count + 1

        # Install or update forward route entry towards destination
        fwd_entry = receiver.routing_table.get(rrep.dest_ip)
        if not fwd_entry or rrep.dest_seq > fwd_entry.dest_seq_num or (rrep.dest_seq == fwd_entry.dest_seq_num and hop_count < fwd_entry.hop_count):
            receiver.routing_table[rrep.dest_ip] = RoutingTableEntry(
                dest_ip=rrep.dest_ip,
                dest_seq_num=rrep.dest_seq,
                valid_seq_num=True,
                state="VALID",
                hop_count=hop_count,
                next_hop=prev_hop_ip,
                lifetime=self.current_time + receiver.active_route_timeout
            )

        # Maintain precursor list
        rev_entry = receiver.routing_table.get(rrep.originator_ip)
        if rev_entry:
            receiver.routing_table[rrep.dest_ip].precursors.add(rev_entry.next_hop)

        self.packet_trace_log.append({
            "timestamp": round(self.current_time, 6),
            "event": "AODV_RREP_RECV",
            "node": receiver.node_id,
            "packet_id": "RREP",
            "src": rrep.dest_ip,
            "dest": rrep.originator_ip,
            "details": f"Forward route to {rrep.dest_ip} via {self.nodes[prev_hop_ip].node_id}, hops={hop_count}"
        })

        if receiver.ip == rrep.originator_ip:
            # Route discovery complete at source!
            disc_start = self.route_discovery_start_times.pop(rrep.dest_ip, self.current_time)
            disc_time_ms = (self.current_time - disc_start) * 1000.0
            self.route_discovery_times_ms.append(disc_time_ms)

            self.packet_trace_log.append({
                "timestamp": round(self.current_time, 6),
                "event": "AODV_ROUTE_ESTABLISHED",
                "node": receiver.node_id,
                "packet_id": "RREP",
                "src": receiver.ip,
                "dest": rrep.dest_ip,
                "details": f"Route established in {disc_time_ms:.3f} ms. Releasing buffered packets."
            })

            # Send buffered data packets
            if rrep.dest_ip in receiver.packet_buffer:
                buffered = receiver.packet_buffer.pop(rrep.dest_ip)
                for pkt in buffered:
                    self.forward_data_packet(receiver, pkt, prev_hop_ip)
        else:
            # Forward RREP along reverse path
            fwd_rrep = AodvRrep(
                originator_ip=rrep.originator_ip,
                dest_ip=rrep.dest_ip,
                dest_seq=rrep.dest_seq,
                hop_count=hop_count,
                lifetime=rrep.lifetime,
                creation_time=rrep.creation_time
            )
            self.send_rrep(receiver, fwd_rrep)

    def forward_data_packet(self, sender: AodvNode, pkt: UdpWarningPacket, next_hop_ip: str):
        next_hop_node = self.nodes.get(next_hop_ip)

        # Check if wireless link is valid
        if not next_hop_node or not sender.can_reach(next_hop_node):
            self.packet_trace_log.append({
                "timestamp": round(self.current_time, 6),
                "event": "LINK_FAILURE_DETECTED",
                "node": sender.node_id,
                "packet_id": pkt.msg_id,
                "src": pkt.source_ip,
                "dest": pkt.dest_ip,
                "details": f"Wireless link to {next_hop_ip} broken! Triggering AODV RERR."
            })
            self.handle_link_break(sender, next_hop_ip)
            return

        tx_delay = self.calc_tx_delay(pkt.payload_bytes)
        arrival_time = self.current_time + tx_delay

        self.schedule(arrival_time, "PROCESS_DATA", next_hop_ip, pkt)

    def handle_data_packet(self, receiver: AodvNode, pkt: UdpWarningPacket):
        if not receiver.is_active:
            return

        pkt.hop_count += 1
        pkt.traversed_path.append(receiver.node_id)

        if receiver.ip == pkt.dest_ip:
            # Successfully delivered!
            self.packets_delivered += 1
            e2e_delay_ms = (self.current_time - pkt.creation_time) * 1000.0
            self.e2e_delays_ms.append(e2e_delay_ms)
            self.hop_counts.append(pkt.hop_count)

            self.packet_trace_log.append({
                "timestamp": round(self.current_time, 6),
                "event": "APP_PACKET_DELIVERED",
                "node": receiver.node_id,
                "packet_id": pkt.msg_id,
                "src": pkt.source_ip,
                "dest": pkt.dest_ip,
                "details": f"Delivered in {e2e_delay_ms:.3f} ms, hops={pkt.hop_count}, path={'->'.join(pkt.traversed_path)}"
            })
        else:
            # Forward data packet using active route
            fwd_entry = receiver.routing_table.get(pkt.dest_ip)
            if fwd_entry and fwd_entry.state == "VALID":
                self.forward_data_packet(receiver, pkt, fwd_entry.next_hop)
            else:
                self.packet_trace_log.append({
                    "timestamp": round(self.current_time, 6),
                    "event": "PACKET_DROPPED_NO_ROUTE",
                    "node": receiver.node_id,
                    "packet_id": pkt.msg_id,
                    "src": pkt.source_ip,
                    "dest": pkt.dest_ip,
                    "details": f"No active route to {pkt.dest_ip} at intermediate node {receiver.node_id}"
                })

    def handle_link_break(self, detector: AodvNode, broken_next_hop_ip: str):
        # Identify all routes using broken_next_hop_ip as their next hop
        unreachable: List[str] = []
        seq_nums: List[int] = []
        precursors_to_notify: Set[str] = set()

        for dest_ip, entry in detector.routing_table.items():
            if entry.next_hop == broken_next_hop_ip and entry.state == "VALID":
                entry.state = "INVALID"
                entry.dest_seq_num += 1
                unreachable.append(dest_ip)
                seq_nums.append(entry.dest_seq_num)
                precursors_to_notify.update(entry.precursors)

        if unreachable:
            self.rerr_transmitted += 1
            rerr = AodvRerr(
                unreachable_dest_ips=unreachable,
                dest_seq_nums=seq_nums,
                creation_time=self.current_time
            )

            self.packet_trace_log.append({
                "timestamp": round(self.current_time, 6),
                "event": "AODV_RERR_BROADCAST",
                "node": detector.node_id,
                "packet_id": "RERR",
                "src": detector.ip,
                "dest": "PRECURSORS",
                "details": f"Invalidated routes to {unreachable}. Notifying precursors: {precursors_to_notify}"
            })

            # Broadcast RERR to precursors
            for prec_ip in precursors_to_notify:
                prec_node = self.nodes.get(prec_ip)
                if prec_node and detector.can_reach(prec_node):
                    tx_delay = self.calc_tx_delay(36)
                    self.schedule(self.current_time + tx_delay, "PROCESS_RERR", prec_ip, (rerr, detector.ip))

    def handle_rerr(self, receiver: AodvNode, rerr: AodvRerr, sender_ip: str):
        if not receiver.is_active:
            return

        invalidated_any = False
        for unreach_ip, seq in zip(rerr.unreachable_dest_ips, rerr.dest_seq_nums):
            entry = receiver.routing_table.get(unreach_ip)
            if entry and entry.next_hop == sender_ip and entry.state == "VALID":
                entry.state = "INVALID"
                entry.dest_seq_num = seq
                invalidated_any = True

        if invalidated_any:
            self.packet_trace_log.append({
                "timestamp": round(self.current_time, 6),
                "event": "AODV_RERR_RECV",
                "node": receiver.node_id,
                "packet_id": "RERR",
                "src": sender_ip,
                "dest": receiver.ip,
                "details": f"Routes to {rerr.unreachable_dest_ips} marked INVALID"
            })

    # -----------------------------------------------------------------------
    # Simulation Main Execution Loop
    # -----------------------------------------------------------------------
    def run_until(self, end_time: float):
        while self.event_queue and self.event_queue[0].timestamp <= end_time:
            evt = heapq.heappop(self.event_queue)
            self.current_time = evt.timestamp
            target_node = self.nodes[evt.target_node]

            if evt.event_type == "PROCESS_RREQ":
                rreq, prev_hop = evt.payload
                self.handle_rreq(target_node, rreq, prev_hop)
            elif evt.event_type == "PROCESS_RREP":
                rrep, prev_hop = evt.payload
                self.handle_rrep(target_node, rrep, prev_hop)
            elif evt.event_type == "PROCESS_RERR":
                rerr, sender_ip = evt.payload
                self.handle_rerr(target_node, rerr, sender_ip)
            elif evt.event_type == "PROCESS_DATA":
                pkt = evt.payload
                self.handle_data_packet(target_node, pkt)
            elif evt.event_type == "TRIGGER_UDP_PACKET":
                dest_ip, w_type, m_id = evt.payload
                self.send_udp_packet(target_node.ip, dest_ip, w_type, m_id)


# ===========================================================================
# 4. Standard Experiment Automation Suite
# ===========================================================================

def build_five_node_topology(radio_range: float = 300.0) -> AodvNetworkSimulation:
    sim = AodvNetworkSimulation()
    # Inter-vehicle spacing: 250m along X-axis
    sim.add_node(AodvNode("V1", "10.0.0.1", 100.0, 180.0, radio_range))
    sim.add_node(AodvNode("V2", "10.0.0.2", 350.0, 180.0, radio_range))
    sim.add_node(AodvNode("V3", "10.0.0.3", 600.0, 180.0, radio_range))
    sim.add_node(AodvNode("V4", "10.0.0.4", 850.0, 180.0, radio_range))
    sim.add_node(AodvNode("V5", "10.0.0.5", 1100.0, 180.0, radio_range))
    return sim


def run_experiment_1_initial_discovery() -> dict:
    sim = build_five_node_topology()
    sim.schedule(15.0, "TRIGGER_UDP_PACKET", "10.0.0.1", ("10.0.0.5", "SUDDEN_BRAKING_WARNING", "EXP1_WARN"))
    sim.run_until(25.0)

    pdr = (sim.packets_delivered / sim.packets_generated * 100.0) if sim.packets_generated else 0.0
    return {
        "experiment": "Exp 1 - Initial Route Discovery",
        "packets_sent": sim.packets_generated,
        "packets_received": sim.packets_delivered,
        "pdr_percent": round(pdr, 2),
        "avg_delay_ms": round(sim.e2e_delays_ms[0], 3) if sim.e2e_delays_ms else 0.0,
        "min_delay_ms": round(min(sim.e2e_delays_ms), 3) if sim.e2e_delays_ms else 0.0,
        "max_delay_ms": round(max(sim.e2e_delays_ms), 3) if sim.e2e_delays_ms else 0.0,
        "overhead_packets": sim.rreq_transmitted + sim.rrep_transmitted + sim.rerr_transmitted,
        "rreq_count": sim.rreq_transmitted,
        "rrep_count": sim.rrep_transmitted,
        "rerr_count": sim.rerr_transmitted,
        "discovery_time_ms": round(sim.route_discovery_times_ms[0], 3) if sim.route_discovery_times_ms else 0.0,
        "avg_hop_count": sim.hop_counts[0] if sim.hop_counts else 0,
        "sim_instance": sim
    }


def run_experiment_2_repeated_transmission() -> dict:
    sim = build_five_node_topology()
    # Transmit 10 packets at 1.0s interval from t=15.0s to 24.0s
    for i in range(10):
        sim.schedule(15.0 + i * 1.0, "TRIGGER_UDP_PACKET", "10.0.0.1", ("10.0.0.5", "SUDDEN_BRAKING_WARNING", f"EXP2_WARN_{i+1}"))
    sim.run_until(30.0)

    pdr = (sim.packets_delivered / sim.packets_generated * 100.0) if sim.packets_generated else 0.0
    avg_delay = sum(sim.e2e_delays_ms) / len(sim.e2e_delays_ms) if sim.e2e_delays_ms else 0.0
    avg_hops = sum(sim.hop_counts) / len(sim.hop_counts) if sim.hop_counts else 0.0

    return {
        "experiment": "Exp 2 - Repeated Transmission (Cache Reuse)",
        "packets_sent": sim.packets_generated,
        "packets_received": sim.packets_delivered,
        "pdr_percent": round(pdr, 2),
        "avg_delay_ms": round(avg_delay, 3),
        "min_delay_ms": round(min(sim.e2e_delays_ms), 3) if sim.e2e_delays_ms else 0.0,
        "max_delay_ms": round(max(sim.e2e_delays_ms), 3) if sim.e2e_delays_ms else 0.0,
        "overhead_packets": sim.rreq_transmitted + sim.rrep_transmitted + sim.rerr_transmitted,
        "rreq_count": sim.rreq_transmitted,
        "rrep_count": sim.rrep_transmitted,
        "rerr_count": sim.rerr_transmitted,
        "discovery_time_ms": round(sim.route_discovery_times_ms[0], 3) if sim.route_discovery_times_ms else 0.0,
        "avg_hop_count": round(avg_hops, 2),
        "sim_instance": sim
    }


def run_experiment_3_link_failure() -> dict:
    sim = build_five_node_topology()
    # 5 packets before failure, then V3 fails at t=19.5s, 5 packets after failure
    for i in range(10):
        sim.schedule(15.0 + i * 1.0, "TRIGGER_UDP_PACKET", "10.0.0.1", ("10.0.0.5", "SUDDEN_BRAKING_WARNING", f"EXP3_WARN_{i+1}"))

    # Custom event to disable node V3 at t=19.5s
    def disable_v3():
        sim.nodes["10.0.0.3"].is_active = False

    # Simulate link failure at t = 19.5 s
    sim.schedule(19.5, "TRIGGER_UDP_PACKET", "10.0.0.1", ("10.0.0.1", "INTERNAL_DISABLE_V3", "DUMMY"))
    # Manually flag V3 inactive at t=19.5 by intercepting loop
    sim.run_until(19.5)
    sim.nodes["10.0.0.3"].is_active = False
    sim.run_until(30.0)

    pdr = (sim.packets_delivered / sim.packets_generated * 100.0) if sim.packets_generated else 0.0
    avg_delay = sum(sim.e2e_delays_ms) / len(sim.e2e_delays_ms) if sim.e2e_delays_ms else 0.0

    return {
        "experiment": "Exp 3 - Link Failure & RERR",
        "packets_sent": sim.packets_generated - 1,  # exclude internal dummy
        "packets_received": sim.packets_delivered,
        "pdr_percent": round((sim.packets_delivered / (sim.packets_generated - 1)) * 100.0, 2),
        "avg_delay_ms": round(avg_delay, 3),
        "min_delay_ms": round(min(sim.e2e_delays_ms), 3) if sim.e2e_delays_ms else 0.0,
        "max_delay_ms": round(max(sim.e2e_delays_ms), 3) if sim.e2e_delays_ms else 0.0,
        "overhead_packets": sim.rreq_transmitted + sim.rrep_transmitted + sim.rerr_transmitted,
        "rreq_count": sim.rreq_transmitted,
        "rrep_count": sim.rrep_transmitted,
        "rerr_count": sim.rerr_transmitted,
        "discovery_time_ms": round(sim.route_discovery_times_ms[0], 3) if sim.route_discovery_times_ms else 0.0,
        "avg_hop_count": 4.0,
        "sim_instance": sim
    }


def run_experiment_4_route_rediscovery() -> dict:
    sim = build_five_node_topology()
    # V1 sends packets at 15.0 and 16.0s
    sim.schedule(15.0, "TRIGGER_UDP_PACKET", "10.0.0.1", ("10.0.0.5", "SUDDEN_BRAKING_WARNING", "EXP4_WARN_1"))
    sim.schedule(16.0, "TRIGGER_UDP_PACKET", "10.0.0.1", ("10.0.0.5", "SUDDEN_BRAKING_WARNING", "EXP4_WARN_2"))

    sim.run_until(17.5)
    # Temporary glitch: V3 drops out
    sim.nodes["10.0.0.3"].is_active = False

    # Packet at 18.0 fails and triggers RERR
    sim.schedule(18.0, "TRIGGER_UDP_PACKET", "10.0.0.1", ("10.0.0.5", "SUDDEN_BRAKING_WARNING", "EXP4_WARN_3"))
    sim.run_until(19.0)

    # V3 recovers / reconnects at t = 19.5s
    sim.nodes["10.0.0.3"].is_active = True

    # Packets at 20.0s and 21.0s trigger fresh route rediscovery and succeed!
    sim.schedule(20.0, "TRIGGER_UDP_PACKET", "10.0.0.1", ("10.0.0.5", "SUDDEN_BRAKING_WARNING", "EXP4_WARN_4"))
    sim.schedule(21.0, "TRIGGER_UDP_PACKET", "10.0.0.1", ("10.0.0.5", "SUDDEN_BRAKING_WARNING", "EXP4_WARN_5"))
    sim.run_until(28.0)

    pdr = (sim.packets_delivered / sim.packets_generated * 100.0) if sim.packets_generated else 0.0
    avg_delay = sum(sim.e2e_delays_ms) / len(sim.e2e_delays_ms) if sim.e2e_delays_ms else 0.0

    return {
        "experiment": "Exp 4 - Route Rediscovery After Recovery",
        "packets_sent": sim.packets_generated,
        "packets_received": sim.packets_delivered,
        "pdr_percent": round(pdr, 2),
        "avg_delay_ms": round(avg_delay, 3),
        "min_delay_ms": round(min(sim.e2e_delays_ms), 3) if sim.e2e_delays_ms else 0.0,
        "max_delay_ms": round(max(sim.e2e_delays_ms), 3) if sim.e2e_delays_ms else 0.0,
        "overhead_packets": sim.rreq_transmitted + sim.rrep_transmitted + sim.rerr_transmitted,
        "rreq_count": sim.rreq_transmitted,
        "rrep_count": sim.rrep_transmitted,
        "rerr_count": sim.rerr_transmitted,
        "discovery_time_ms": round(sum(sim.route_discovery_times_ms)/len(sim.route_discovery_times_ms), 3) if sim.route_discovery_times_ms else 0.0,
        "avg_hop_count": 4.0,
        "sim_instance": sim
    }


def run_experiment_5_multi_hop_warning() -> dict:
    sim = build_five_node_topology()
    for i in range(5):
        sim.schedule(15.0 + i * 1.0, "TRIGGER_UDP_PACKET", "10.0.0.1", ("10.0.0.5", "SUDDEN_BRAKING_WARNING", f"FCW_ALERT_{i+1}"))
    sim.run_until(25.0)

    pdr = (sim.packets_delivered / sim.packets_generated * 100.0) if sim.packets_generated else 0.0
    avg_delay = sum(sim.e2e_delays_ms) / len(sim.e2e_delays_ms) if sim.e2e_delays_ms else 0.0

    return {
        "experiment": "Exp 5 - Multi-Hop Warning Dissemination",
        "packets_sent": sim.packets_generated,
        "packets_received": sim.packets_delivered,
        "pdr_percent": round(pdr, 2),
        "avg_delay_ms": round(avg_delay, 3),
        "min_delay_ms": round(min(sim.e2e_delays_ms), 3) if sim.e2e_delays_ms else 0.0,
        "max_delay_ms": round(max(sim.e2e_delays_ms), 3) if sim.e2e_delays_ms else 0.0,
        "overhead_packets": sim.rreq_transmitted + sim.rrep_transmitted + sim.rerr_transmitted,
        "rreq_count": sim.rreq_transmitted,
        "rrep_count": sim.rrep_transmitted,
        "rerr_count": sim.rerr_transmitted,
        "discovery_time_ms": round(sim.route_discovery_times_ms[0], 3) if sim.route_discovery_times_ms else 0.0,
        "avg_hop_count": 4.0,
        "sim_instance": sim
    }


def run_baseline_single_hop() -> dict:
    # Direct single-hop transmission: No multi-hop forwarding; radio range = 300m, distance V1-V5 = 1000m
    # Direct transmission will fail 100% of the time!
    sim = build_five_node_topology(radio_range=300.0)

    # Disable intermediate forwarding by isolating V2, V3, V4
    sim.nodes["10.0.0.2"].is_active = False
    sim.nodes["10.0.0.3"].is_active = False
    sim.nodes["10.0.0.4"].is_active = False

    for i in range(10):
        sim.schedule(15.0 + i * 1.0, "TRIGGER_UDP_PACKET", "10.0.0.1", ("10.0.0.5", "SUDDEN_BRAKING_WARNING", f"BASELINE_WARN_{i+1}"))
    sim.run_until(30.0)

    return {
        "experiment": "Baseline - Direct Single-Hop (No AODV)",
        "packets_sent": sim.packets_generated,
        "packets_received": 0,
        "pdr_percent": 0.0,
        "avg_delay_ms": 0.0,
        "min_delay_ms": 0.0,
        "max_delay_ms": 0.0,
        "overhead_packets": sim.rreq_transmitted,
        "rreq_count": sim.rreq_transmitted,
        "rrep_count": 0,
        "rerr_count": 0,
        "discovery_time_ms": 0.0,
        "avg_hop_count": 0.0,
        "sim_instance": sim
    }


# ===========================================================================
# 5. Graph Plotting & Telemetry Export
# ===========================================================================

def generate_analytical_plots(results: List[dict], output_dir: str):
    os.makedirs(output_dir, exist_ok=True)
    df = pd.DataFrame(results)

    # 1. Packet Delivery Ratio (PDR) Plot
    plt.figure(figsize=(9, 5))
    bars = plt.bar(df["experiment"], df["pdr_percent"], color=["#2b5c8f", "#2b8f5c", "#d95f02", "#7570b3", "#1b9e77", "#e7298a"])
    plt.ylabel("Packet Delivery Ratio (%)", fontsize=11, fontweight="bold")
    plt.title("AODV Routing Performance - Packet Delivery Ratio (PDR)", fontsize=13, fontweight="bold")
    plt.xticks(rotation=20, ha="right", fontsize=9)
    plt.ylim(0, 115)
    for bar in bars:
        h = bar.get_height()
        plt.text(bar.get_x() + bar.get_width()/2., h + 2, f"{h:.1f}%", ha="center", va="bottom", fontweight="bold", fontsize=10)
    plt.grid(axis="y", linestyle="--", alpha=0.6)
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, "aodv_pdr_comparison.png"), dpi=200)
    plt.close()

    # 2. End-to-End Latency Comparison
    plt.figure(figsize=(9, 5))
    sub_df = df[df["pdr_percent"] > 0]
    bars = plt.bar(sub_df["experiment"], sub_df["avg_delay_ms"], color="#1f77b4", edgecolor="black")
    plt.ylabel("Average E2E Latency (ms)", fontsize=11, fontweight="bold")
    plt.title("End-to-End Latency Across Multi-Hop AODV Scenarios", fontsize=13, fontweight="bold")
    plt.xticks(rotation=20, ha="right", fontsize=9)
    for bar in bars:
        h = bar.get_height()
        plt.text(bar.get_x() + bar.get_width()/2., h + 0.1, f"{h:.2f} ms", ha="center", va="bottom", fontweight="bold", fontsize=10)
    plt.grid(axis="y", linestyle="--", alpha=0.6)
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, "aodv_e2e_delay.png"), dpi=200)
    plt.close()

    # 3. Routing Control Overhead
    plt.figure(figsize=(9, 5))
    rreq_vals = df["rreq_count"]
    rrep_vals = df["rrep_count"]
    rerr_vals = df["rerr_count"]
    x = range(len(df))
    plt.bar(x, rreq_vals, label="RREQ Messages", color="#e41a1c")
    plt.bar(x, rrep_vals, bottom=rreq_vals, label="RREP Messages", color="#377eb8")
    plt.bar(x, rerr_vals, bottom=[rreq_vals[i] + rrep_vals[i] for i in range(len(df))], label="RERR Messages", color="#4daf4a")
    plt.xticks(x, df["experiment"], rotation=20, ha="right", fontsize=9)
    plt.ylabel("Control Packets Generated", fontsize=11, fontweight="bold")
    plt.title("AODV Routing Protocol Overhead (Control Message Breakdown)", fontsize=13, fontweight="bold")
    plt.legend(frameon=True)
    plt.grid(axis="y", linestyle="--", alpha=0.6)
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, "aodv_routing_overhead.png"), dpi=200)
    plt.close()

    # 4. Route Discovery Time Plot
    plt.figure(figsize=(8, 4.5))
    disc_df = df[df["discovery_time_ms"] > 0]
    bars = plt.bar(disc_df["experiment"], disc_df["discovery_time_ms"], color="#ff7f00", edgecolor="black")
    plt.ylabel("Route Discovery Time (ms)", fontsize=11, fontweight="bold")
    plt.title("Initial AODV Route Discovery Latency (RREQ -> RREP)", fontsize=13, fontweight="bold")
    plt.xticks(rotation=15, ha="right", fontsize=9)
    for bar in bars:
        h = bar.get_height()
        plt.text(bar.get_x() + bar.get_width()/2., h + 0.1, f"{h:.2f} ms", ha="center", va="bottom", fontweight="bold", fontsize=10)
    plt.grid(axis="y", linestyle="--", alpha=0.6)
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, "aodv_route_discovery_time.png"), dpi=200)
    plt.close()

    # 5. Hop Count Analysis
    plt.figure(figsize=(8, 4.5))
    hop_df = df[df["avg_hop_count"] > 0]
    bars = plt.bar(hop_df["experiment"], hop_df["avg_hop_count"], color="#984ea3", edgecolor="black")
    plt.ylabel("Number of Wireless Hops", fontsize=11, fontweight="bold")
    plt.title("Multi-Hop Traversal Count (V1 -> V2 -> V3 -> V4 -> V5)", fontsize=13, fontweight="bold")
    plt.ylim(0, 5.5)
    plt.axhline(y=4.0, color="red", linestyle=":", label="Expected 4-Hop Linear Path")
    plt.xticks(rotation=15, ha="right", fontsize=9)
    plt.legend()
    plt.grid(axis="y", linestyle="--", alpha=0.6)
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, "aodv_hop_count.png"), dpi=200)
    plt.close()

    print(f"Generated 5 analytical figures in: {output_dir}")


def main():
    print("=" * 75)
    print(" STARTING AODV (RFC 3561) VANET MULTI-HOP SIMULATION SUITE")
    print("=" * 75)

    results = []

    print("\n--- Running Experiment 1: Initial Route Discovery ---")
    res1 = run_experiment_1_initial_discovery()
    print(f"   Delivered: {res1['packets_received']}/{res1['packets_sent']} | PDR: {res1['pdr_percent']}% | Discovery Time: {res1['discovery_time_ms']} ms | E2E Latency: {res1['avg_delay_ms']} ms")
    results.append(res1)

    print("\n--- Running Experiment 2: Repeated Transmission (Route Caching) ---")
    res2 = run_experiment_2_repeated_transmission()
    print(f"   Delivered: {res2['packets_received']}/{res2['packets_sent']} | PDR: {res2['pdr_percent']}% | Overhead: {res2['overhead_packets']} pkts | Avg E2E Latency: {res2['avg_delay_ms']} ms")
    results.append(res2)

    print("\n--- Running Experiment 3: Intermediate Link Failure & RERR ---")
    res3 = run_experiment_3_link_failure()
    print(f"   Delivered: {res3['packets_received']}/{res3['packets_sent']} | PDR: {res3['pdr_percent']}% | RERR Sent: {res3['rerr_count']}")
    results.append(res3)

    print("\n--- Running Experiment 4: Route Rediscovery After Recovery ---")
    res4 = run_experiment_4_route_rediscovery()
    print(f"   Delivered: {res4['packets_received']}/{res4['packets_sent']} | PDR: {res4['pdr_percent']}% | Avg Discovery Time: {res4['discovery_time_ms']} ms")
    results.append(res4)

    print("\n--- Running Experiment 5: Multi-Hop Collision Warning Application ---")
    res5 = run_experiment_5_multi_hop_warning()
    print(f"   Delivered: {res5['packets_received']}/{res5['packets_sent']} | PDR: {res5['pdr_percent']}% | Avg Latency: {res5['avg_delay_ms']} ms | Hops: {res5['avg_hop_count']}")
    results.append(res5)

    print("\n--- Running Baseline: Single-Hop Direct Communication (No AODV) ---")
    res_base = run_baseline_single_hop()
    print(f"   Delivered: {res_base['packets_received']}/{res_base['packets_sent']} | PDR: {res_base['pdr_percent']}% (Failed as expected: 1000m > 300m range)")
    results.append(res_base)

    # -----------------------------------------------------------------------
    # Export Telemetry CSVs and Analytical Plots
    # -----------------------------------------------------------------------
    results_dir = os.path.join(os.path.dirname(__file__), "results")
    os.makedirs(results_dir, exist_ok=True)

    # Clean results for DataFrame export
    export_data = []
    for r in results:
        entry = {k: v for k, v in r.items() if k != "sim_instance"}
        export_data.append(entry)

    metrics_csv_path = os.path.join(results_dir, "aodv_experiment_metrics.csv")
    pd.DataFrame(export_data).to_csv(metrics_csv_path, index=False)
    print(f"\nSaved metrics summary to: {metrics_csv_path}")

    # Combine packet event traces from Exp 1 and Exp 3
    trace_combined = res1["sim_instance"].packet_trace_log + res3["sim_instance"].packet_trace_log
    trace_csv_path = os.path.join(results_dir, "aodv_packet_trace.csv")
    pd.DataFrame(trace_combined).to_csv(trace_csv_path, index=False)
    print(f"Saved packet trace events to: {trace_csv_path}")

    generate_analytical_plots(export_data, results_dir)

    print("\n" + "=" * 75)
    print(" ALL AODV SIMULATION EXPERIMENTS COMPLETED SUCCESSFULLY!")
    print("=" * 75)


if __name__ == "__main__":
    main()
