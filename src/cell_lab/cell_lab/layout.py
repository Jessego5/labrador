"""
This is the little piece of workspace geometry the lab node needs, where a rack slot's pocket
floor is, read from the same workspace yaml cell_workspace loads. It follows
cell_workspace::Workspace::placePose for trays (the slot's centre at the pocket floor, rows
along the tray's x and columns along its y, rotated by the tray's yaw), so a sample the lab
sets out stands exactly where the skills would have placed it. Only trays are handled, because
samples only ever start in a rack.
"""
import math


def slot_floor(workspace, target):
    """(x, y, z) of a tray slot's pocket floor for "rack_in/A1", or for a one-slot tray's bare
    name. Raises KeyError for anything that is not a tray slot."""
    tray_name, _, slot = target.partition("/")
    trays = workspace.get("trays") or {}
    if tray_name not in trays:
        raise KeyError(f"'{target}' is not a tray slot in the workspace")
    t = trays[tray_name]
    if not slot:
        if t["rows"] * t["cols"] != 1:
            raise KeyError(f"'{target}' names a tray with several slots, name one")
        slot = "A1"
    row, col = ord(slot[0]) - ord("A"), int(slot[1:]) - 1
    if not (0 <= row < t["rows"] and 0 <= col < t["cols"]):
        raise KeyError(f"'{target}' is outside the tray's grid")
    lx, ly = row * t["pitch"][0], col * t["pitch"][1]
    yaw = t.get("yaw", 0.0)
    ox, oy, oz = t["origin"]
    return (ox + math.cos(yaw) * lx - math.sin(yaw) * ly,
            oy + math.sin(yaw) * lx + math.cos(yaw) * ly,
            oz - t["slot_depth"])
