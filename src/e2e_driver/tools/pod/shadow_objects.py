#!/usr/bin/env python3
"""shadow_objects.py (shadow master, shadow.sh): the GT object lists the shadow collection driver's nodes read, with
Object Info's velocity turned into map axes (it comes in the object's own frame: 10/28 shadow run, 0.4 m/s off as
body frame vs 20 m/s as map frame; the collection nodes read it as map axes). From /collect_only/objects_all and
gt-objects-filter's standing objects (remapped to /shadow/objects_static):
  /objects                 the planners: the standing objects, plus on the highway (route s 1118.7-1741.7, sections
                           11-12) the moving vehicles, so frenet_path can check the target lane's gap before a lane
                           change (shadow_patches/frenet_gap.patch; gt-objects-filter drops every moving object)
  /shadow/objects_map      the GT AEB node: every object
  /shadow/yield_objects    the GT yield node: every object, but no vehicles in the roundabout section (9): there
                           roundabout_merge decides the entry from the ring traffic (user 10/28: the yield node held the
                           car at the entry for every ring NPC; R1 had no yield node)"""
import copy
import math

ROUNDABOUT, HIGHWAY_SECTIONS = (9,), (11, 12)
MOVING = 0.3


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def main():
    import rospy
    from interfaces.msg import Objects, SectionAnnotation

    rospy.init_node("shadow_objects")
    st = dict(section=0, static=None)
    pub_plan = rospy.Publisher("/objects", Objects, queue_size=1)
    pub_map = rospy.Publisher("/shadow/objects_map", Objects, queue_size=1)
    pub_yield = rospy.Publisher("/shadow/yield_objects", Objects, queue_size=1)

    def on_all(m):
        out = copy.deepcopy(m)
        for o in out.objects:                      # object frame -> map axes
            if o.velocity_available:
                h = yaw_of(o.bbox_pose.orientation)
                vx, vy = o.velocity.x, o.velocity.y
                o.velocity.x, o.velocity.y = vx * math.cos(h) - vy * math.sin(h), vx * math.sin(h) + vy * math.cos(h)
        pub_map.publish(out)
        y = copy.deepcopy(out)
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

    rospy.Subscriber("/section_annotation", SectionAnnotation, lambda m: st.__setitem__("section", m.state), queue_size=1)
    rospy.Subscriber("/shadow/objects_static", Objects, lambda m: st.__setitem__("static", m), queue_size=1)
    rospy.Subscriber("/collect_only/objects_all", Objects, on_all, queue_size=1)
    rospy.spin()


if __name__ == "__main__":
    main()
