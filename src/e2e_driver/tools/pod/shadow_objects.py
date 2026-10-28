#!/usr/bin/env python3
"""shadow_objects.py SCORER_DIR (shadow master, shadow.sh): the GT object lists the shadow collection driver's nodes read,
with Object Info's velocity turned into map axes (it comes in the object's own frame: 10/28 shadow run, 0.4 m/s off as
body frame vs 20 m/s as map frame; the collection nodes read it as map axes). From /collect_only/objects_all and
gt-objects-filter's standing objects (remapped to /shadow/objects_static):
  /objects                 the planners: the standing objects, plus on the highway (route s 1118.7-1741.7, sections
                           11-13: frenet_path's, path_planner selection.yaml) the moving vehicles, so frenet_path can
                           check the target lane's gap before a lane change (shadow_patches/frenet_gap.patch;
                           gt-objects-filter drops every moving object)
  /shadow/objects_map      the GT AEB node: every standing object and every visible moving one
  /shadow/yield_objects    the GT yield node: the same, but no vehicles in the roundabout section (9): there
                           roundabout_merge decides the entry from the ring traffic (user 10/28: the yield node held the
                           car at the entry for every ring NPC; R1 had no yield node)
Visible: the yield and AEB nodes get a moving object (>= MOVING, the checker's static_mps) only while the model could
see it (user 10/28: labels braked for pedestrians before they were in view; "only on the dynamic ones": standing
objects and the planners' objects stay unfiltered, the highway's rear gap is frenet_gap.patch's): the label checker's
visible() (SCORER_DIR/label_check.py, its hidden_at_t0 criterion and scope) from the GT ego pose
(/collect_only/ego_status) over the whole Object Info frame, so any object can hide another. No pose yet: standing only."""
import ast
import copy
import math
import sys
from pathlib import Path

ROUNDABOUT, HIGHWAY_SECTIONS = (9,), (11, 12, 13)
MOVING = 0.3


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def load_visible(scorer):
    """label_check.visible (and _segment_boxes) from their source with label_check.yaml's visibility block, without
    label_check's imports (pandas, the scorer's map: not in the ROS python) -> visible(x, y, yaw, objects) -> bools."""
    import numpy as np
    import yaml
    src = (Path(scorer) / "label_check.py").read_text()
    ns = dict(np=np, math=math)
    for node in ast.parse(src).body:
        if isinstance(node, ast.FunctionDef) and node.name in ("_segment_boxes", "visible"):
            exec(ast.get_source_segment(src, node), ns)
    cfg = yaml.safe_load((Path(scorer) / "label_check.yaml").read_text())["visibility"]
    return lambda x, y, yaw, objs: ns["visible"](x, y, yaw, objs, cfg)


def as_tuple(o):
    """an interfaces/Object as visible() takes it: (id, label, x, y, yaw, vx, vy, size x, size y)."""
    p = o.bbox_pose.position
    return (o.id, o.label, p.x, p.y, yaw_of(o.bbox_pose.orientation), o.velocity.x, o.velocity.y, o.bbox_size.x, o.bbox_size.y)


def main():
    import rospy
    from interfaces.msg import Objects, SectionAnnotation
    from nav_msgs.msg import Odometry

    visible = load_visible(sys.argv[1])
    rospy.init_node("shadow_objects")
    st = dict(section=0, static=None, ego=None)
    pub_plan = rospy.Publisher("/objects", Objects, queue_size=1)
    pub_map = rospy.Publisher("/shadow/objects_map", Objects, queue_size=1)
    pub_yield = rospy.Publisher("/shadow/yield_objects", Objects, queue_size=1)

    def on_ego(m):
        st["ego"] = (m.pose.pose.position.x, m.pose.pose.position.y, yaw_of(m.pose.pose.orientation))

    def on_all(m):
        out = copy.deepcopy(m)
        for o in out.objects:                      # object frame -> map axes
            if o.velocity_available:
                h = yaw_of(o.bbox_pose.orientation)
                vx, vy = o.velocity.x, o.velocity.y
                o.velocity.x, o.velocity.y = vx * math.cos(h) - vy * math.sin(h), vx * math.sin(h) + vy * math.cos(h)
        seen = visible(*st["ego"], [as_tuple(o) for o in out.objects]) if st["ego"] else [False] * len(out.objects)
        vis = copy.copy(out)
        vis.objects = [o for o, s in zip(out.objects, seen) if s or math.hypot(o.velocity.x, o.velocity.y) < MOVING]
        pub_map.publish(vis)
        y = copy.deepcopy(vis)
        if st["section"] in ROUNDABOUT:
            y.objects = [o for o in y.objects if o.label != "vehicle"]
        pub_yield.publish(y)
        if st["static"] is not None:
            plan = copy.deepcopy(st["static"])
            plan.header = out.header
            if st["section"] in HIGHWAY_SECTIONS:
                have = {o.id for o in plan.objects}
                plan.objects += [o for o in out.objects if o.label == "vehicle" and o.id not in have
                                 and math.hypot(o.velocity.x, o.velocity.y) >= MOVING]
            pub_plan.publish(plan)

    rospy.Subscriber("/collect_only/ego_status", Odometry, on_ego, queue_size=1)
    rospy.Subscriber("/section_annotation", SectionAnnotation, lambda m: st.__setitem__("section", m.state), queue_size=1)
    rospy.Subscriber("/shadow/objects_static", Objects, lambda m: st.__setitem__("static", m), queue_size=1)
    rospy.Subscriber("/collect_only/objects_all", Objects, on_all, queue_size=1)
    rospy.spin()


if __name__ == "__main__":
    main()
