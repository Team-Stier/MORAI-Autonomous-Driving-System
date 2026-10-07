"""Build numeric ROS payload inspectors from the installed genmsg definitions."""
import os
from pathlib import Path
import xml.etree.ElementTree as ET


def emit_numeric_traits(manifest):
    """Return includes and complete node_pipeline::detail specializations."""
    message_types = set()
    for port in list(manifest.get("inputs", {}).values()) + [manifest.get("output", {})] + [
            port for core in manifest.get("cores", []) for port in core.get("inputs", []) + core.get("outputs", [])]:
        name = port.get("type", "")
        if "/" in name or (name.count("::") == 1 and "<" not in name):
            message_types.add(name.replace("::", "/"))
    # These concrete runtime/semantic inspectors are defined in message_traits.hpp.
    runtime_types = {"interfaces/CompetitionVehicleStatus", "interfaces/Objects", "interfaces/Path",
        "interfaces/SectionAnnotation", "interfaces/TrafficLight", "morai_msgs/CtrlCmd", "nav_msgs/Odometry",
        "sensor_msgs/CompressedImage", "sensor_msgs/Imu", "sensor_msgs/NavSatFix", "sensor_msgs/PointCloud2",
        "tf2_msgs/TFMessage", "std_msgs/Float64"}
    from genmsg.msgs import BUILTIN_TYPES
    from genmsg.msg_loader import MsgContext, load_msg_by_type
    search = {}
    package = Path(manifest["package_dir"]).resolve()
    roots = {Path("/opt/ros/noetic/share")}
    roots.update(parent / "src" for parent in package.parents if (parent / "src").is_dir())
    roots.update(parent / "src" for parent in Path(__file__).resolve().parents if (parent / "src").is_dir())
    roots.update(Path(path) for path in os.environ.get("ROS_PACKAGE_PATH", "").split(":") if path)
    roots.update(Path(path) / "share" for path in os.environ.get("CMAKE_PREFIX_PATH", "").split(":") if path)
    for root in sorted(roots, key=lambda path: (path.name != "src", str(path))):
        for directory in sorted(set(root.glob("*/msg")) | set(root.glob("*/*/msg")) | ({root / "msg"} if (root / "msg").is_dir() else set())):
            package_xml = directory.parent / "package.xml"
            name = ET.parse(package_xml).getroot().findtext("name") if package_xml.is_file() else directory.parent.name
            search.setdefault(name, []).append(str(directory))
    message_types = {name for name in message_types if any(
        (Path(directory) / (name.split("/")[1] + ".msg")).is_file() for directory in search.get(name.split("/")[0], []))}
    message_types.update(runtime_types)
    context = MsgContext.create_default()
    specs = {}

    def load(name):
        if name in specs:
            return
        spec = load_msg_by_type(context, name, search)
        specs[name] = spec
        for field in spec.parsed_fields():
            if field.base_type not in BUILTIN_TYPES:
                load(field.base_type if "/" in field.base_type else spec.package + "/" + field.base_type)

    for name in sorted(message_types):
        load(name)
    manifest["message_definitions"] = sorted({str((Path(directory) / (name.split("/")[1] + ".msg")).resolve())
        for name in specs for directory in [next(directory for directory in search[name.split("/")[0]]
            if (Path(directory) / (name.split("/")[1] + ".msg")).is_file())]})
    lines = ["#include <node_pipeline/detail/native_traits.hpp>"] + ['#include <' + name + '.h>' for name in sorted(specs)]
    lines += ["namespace node_pipeline { namespace detail {"]
    # Forward declarations let nested message/container inspectors resolve before their definitions.
    for name in sorted(specs):
        native = name.replace("/", "::")
        lines += ["template<> struct NumericInspector<" + native + "> {",
                  "  static void inspect(const " + native + "&, const HealthLimits&, std::vector<Kind>&);", "};"]
    for name, spec in sorted(specs.items()):
        native = name.replace("/", "::")
        lines += ["inline void NumericInspector<" + native + ">::inspect(const " + native + "& m, const HealthLimits& limits, std::vector<Kind>& issues) {"]
        for field in spec.parsed_fields():
            base = field.base_type
            # ROS timestamps remain metadata, not configured payload number ranges.
            if base in {"time", "duration"} or name == "std_msgs/Header":
                continue
            if base == "std_msgs/Header":
                continue
            guard = ""
            if name == "interfaces/Object" and field.name in {"velocity", "acceleration"}:
                guard = "if (m." + field.name + "_available) "
            lines += ["  " + guard + "NumericInspector<typename std::decay<decltype(m." + field.name + ")>::type>::inspect(m." + field.name + ", limits, issues);"]
        lines += ["  (void)m; (void)limits; (void)issues;", "}"]
    for name, spec in sorted(specs.items()):
        if name in runtime_types:
            continue
        native = name.replace("/", "::")
        lines += ["template<> struct MessageTraits<" + native + "> {",
                  "  static void inspect(const " + native + "& m, const HealthLimits& limits, std::vector<Kind>& issues) {"]
        if any(field.name == "header" and field.base_type == "std_msgs/Header" for field in spec.parsed_fields()):
            lines += ["    inspectFrame(m.header.frame_id, limits, issues);"]
        lines += ["    NumericInspector<" + native + ">::inspect(m, limits, issues);"]
        lines += ["  }", "};"]
    return "\n".join(lines + ["}}", ""])
