# This is one object found by perception, as the detector in cell_perception reports it and the
# DetectObjects skill returns it. The pose is the centre of the bounding box of
# the object type the measurement matched, so a pick can be planned from it directly.
string id                          # unique within one detection, e.g. "blue_cylinder_2"
string type                        # object type from cell_description/config/objects.yaml
string object_class                # cube, cylinder, peg
string color
geometry_msgs/PoseStamped pose     # centre of the bounding box (yaw from the top face for boxes)
geometry_msgs/Vector3 size         # bounding box of the matched type
float32 confidence                 # 0..1, how well the measurement matches the type
