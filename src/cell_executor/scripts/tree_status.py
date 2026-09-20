#!/usr/bin/env python3
"""This prints the executor's live behavior-tree status from its Groot2 port, without Groot2.
It is for when Groot2 itself is not available, as on the ARM64 VM. It speaks the same ZeroMQ request and reply
protocol as Groot2 (BT::Groot2Publisher, protocol 2): it asks for the full tree once, then polls
the node statuses and prints the running branch, with any failed nodes, each time it changes
(--all prints every node). Run it with ros2 run cell_executor tree_status.py to follow a task
until it ends, or with --once for one snapshot.
"""
import argparse
import random
import struct
import sys
import time
import xml.etree.ElementTree as ET

import zmq

PROTOCOL = 2
STATUS = {0: "IDLE", 1: "RUNNING", 2: "SUCCESS", 3: "FAILURE", 4: "SKIPPED"}


class Groot2Client:
    def __init__(self, host, port, timeout_ms):
        self.socket = zmq.Context.instance().socket(zmq.REQ)
        self.socket.setsockopt(zmq.RCVTIMEO, timeout_ms)
        self.socket.setsockopt(zmq.LINGER, 0)
        self.socket.connect(f"tcp://{host}:{port}")

    def request(self, kind):
        uid = random.getrandbits(32)
        self.socket.send(struct.pack("<BBI", PROTOCOL, ord(kind), uid))
        parts = self.socket.recv_multipart()
        protocol, reply_kind, reply_uid = struct.unpack("<BBI", parts[0][:6])
        if (protocol, chr(reply_kind), reply_uid) != (PROTOCOL, kind, uid):
            raise RuntimeError(f"unexpected reply header {parts[0][:6]!r}")
        return parts[0][6:22], parts[1:]  # tree id, payload

    def tree(self):
        tree_id, payload = self.request("T")
        return tree_id, payload[0].decode()

    def statuses(self):
        tree_id, payload = self.request("S")
        data = payload[0]
        return tree_id, {struct.unpack_from("<H", data, i)[0]: data[i + 2]
                         for i in range(0, len(data) - 2, 3)}


def node_table(xml_text):
    """uid -> (depth, label) in document order, from the tree XML, whose nodes carry _uid."""
    nodes = {}
    root = ET.fromstring(xml_text)

    def walk(element, depth):
        uid = element.get("_uid")
        if uid is not None:
            label = element.tag
            name = element.get("name") or element.get("title")
            if name and name != element.tag:
                label += f" '{name}'"
            for key in ("object", "target", "pose", "filter", "index"):
                if element.get(key):
                    label += f" {key}={element.get(key)}"
            nodes[int(uid)] = (depth, label)
            depth += 1
        for child in element:
            if child.tag != "TreeNodesModel":
                walk(child, depth)

    walk(root, 0)
    return nodes


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=1667)
    ap.add_argument("--once", action="store_true", help="print one snapshot and exit")
    ap.add_argument("--period", type=float, default=1.0, help="s between snapshots")
    ap.add_argument("--all", action="store_true", help="print every node, not just active ones")
    args = ap.parse_args()

    client = Groot2Client(args.host, args.port, timeout_ms=2000)
    try:
        tree_id, xml_text = client.tree()
    except zmq.Again:
        sys.exit(f"no behavior tree on {args.host}:{args.port} (is a task running?)")
    nodes = node_table(xml_text)
    print(f"tree {tree_id.hex()[:8]}: {len(nodes)} nodes")
    last = None
    while True:
        try:
            current_id, status = client.statuses()
        except zmq.Again:
            print("executor went away (task finished)")
            return
        if current_id != tree_id:
            print("a new task started")
            tree_id, xml_text = client.tree()
            nodes = node_table(xml_text)
            continue
        lines = []
        for uid, (depth, label) in nodes.items():
            s = STATUS.get(status.get(uid, 0), "?")
            if args.all or s in ("RUNNING", "FAILURE"):
                lines.append(f"{'  ' * depth}{label}  [{s}]")
        snapshot = "\n".join(lines)
        if snapshot != last:
            print(time.strftime("%H:%M:%S"), "-" * 40)
            print(snapshot or "(nothing running)")
            last = snapshot
        if args.once:
            return
        time.sleep(args.period)


if __name__ == "__main__":
    main()
