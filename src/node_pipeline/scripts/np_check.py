#!/usr/bin/env python3
"""실행 패키지의 경로·YAML·include·AST 계약을 검사한다. 위반이 있으면 빌드를 중단한다."""
import argparse
import ctypes
import glob
import os
import re
import sys
import xml.etree.ElementTree as ElementTree

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import np_generate as generator  # noqa: E402

SOURCE_SUFFIXES = (".h", ".hh", ".hpp", ".hxx", ".c", ".cc", ".cpp", ".cxx")
EXTENSION_KINDS = ("cores", "validators", "policies", "parameters", "banks", "libraries")
STD_HEADERS = {
    "algorithm", "any", "array", "atomic", "bitset", "cassert", "cctype", "cerrno", "cfenv", "cfloat", "chrono",
    "cinttypes", "climits", "clocale", "cmath", "complex", "condition_variable", "csetjmp", "csignal", "cstdarg",
    "cstddef", "cstdint", "cstdio", "cstdlib", "cstring", "ctime", "cwchar", "cwctype", "deque", "exception",
    "forward_list", "fstream", "functional", "future", "initializer_list", "iomanip", "ios", "iosfwd", "iostream",
    "istream", "iterator", "limits", "list", "locale", "map", "memory", "mutex", "new", "numeric", "optional",
    "ostream", "queue", "random", "ratio", "regex", "scoped_allocator", "set", "shared_mutex", "sstream", "stack",
    "stdexcept", "streambuf", "string", "system_error", "thread", "tuple", "type_traits", "typeindex", "typeinfo",
    "unordered_map", "unordered_set", "utility", "valarray", "vector",
}
FORBIDDEN_STD = {"fstream", "thread", "future", "mutex", "shared_mutex", "condition_variable"}
FORBIDDEN_INCLUDE_ROOTS = {"ros", "roscpp", "rosbag", "tf2_ros", "actionlib", "message_filters", "image_transport",
                           "nodelet", "pluginlib", "ros.h", "XmlRpc.h", "XmlRpcValue.h"}
SYSTEM_DEPENDS = {"eigen": ("Eigen", "unsupported"), "pugixml-dev": ("pugixml.hpp",), "boost": ("boost",),
                  "yaml-cpp": ("yaml-cpp",), "libopencv-dev": ("opencv2",), "opencv": ("opencv2",)}
FORBIDDEN_SYMBOLS = (
    "ros::NodeHandle", "ros::Publisher", "ros::Subscriber", "ros::ServiceClient", "ros::ServiceServer", "ros::Timer",
    "ros::SteadyTimer", "ros::WallTimer", "ros::init", "ros::spin", "ros::spinOnce", "ros::AsyncSpinner",
    "ros::MultiThreadedSpinner", "ros::CallbackQueue", "ros::param", "ros::master", "ros::service", "ros::topic",
    "ros::console", "ros::shutdown", "ros::package", "ros::this_node", "ros::names", "ros::Time::now",
    "ros::WallTime::now", "ros::SteadyTime::now", "std::thread", "std::async", "std::basic_fstream",
    "std::basic_ifstream", "std::basic_ofstream", "std::basic_filebuf", "std::fstream", "std::ifstream",
    "std::ofstream", "std::filebuf", "std::mutex", "std::system", "fopen", "freopen", "popen", "system", "fork",
    "execv", "execvp", "execl", "socket", "connect", "sendto", "recvfrom", "node_pipeline::detail",
)

# libclang cursor·type 상수다.
TRANSLATION_UNIT, NAMESPACE, LINKAGE_SPEC = 300, 22, 23
STRUCT, UNION, CLASS, ENUM, CLASS_TEMPLATE, PARTIAL_SPECIALIZATION = 2, 3, 4, 5, 31, 32
FUNCTION, METHOD, CONSTRUCTOR, DESTRUCTOR, CONVERSION, FUNCTION_TEMPLATE = 8, 21, 24, 25, 26, 30
VAR = 9
REFERENCE_KINDS = {43, 45, 46, 47, 49, 50, 101, 102, 103}
STORAGE_STATIC = 3
ARRAY_TYPES = {112, 114, 115, 116}
CHILD_CONTINUE, CHILD_RECURSE = 1, 2


class CXString(ctypes.Structure):
    _fields_ = [("data", ctypes.c_void_p), ("flags", ctypes.c_uint)]


class CXCursor(ctypes.Structure):
    _fields_ = [("kind", ctypes.c_int), ("xdata", ctypes.c_int), ("data", ctypes.c_void_p * 3)]


class CXType(ctypes.Structure):
    _fields_ = [("kind", ctypes.c_int), ("data", ctypes.c_void_p * 2)]


class CXSourceLocation(ctypes.Structure):
    _fields_ = [("ptr_data", ctypes.c_void_p * 2), ("int_data", ctypes.c_uint)]


VISITOR = ctypes.CFUNCTYPE(ctypes.c_int, CXCursor, CXCursor, ctypes.c_void_p)


class Clang:
    def __init__(self):
        candidates = sorted(glob.glob("/usr/lib/llvm-*/lib/libclang.so*"), reverse=True) + \
            sorted(glob.glob("/usr/lib/*/libclang-[0-9]*.so*"), reverse=True) + \
            sorted(glob.glob("/usr/lib/libclang*.so*"), reverse=True)
        lib = None
        for candidate in candidates:
            try:
                loaded = ctypes.CDLL(candidate)
            except OSError:
                continue
            if hasattr(loaded, "clang_createIndex"):
                lib = loaded
                break
        if lib is None:
            raise RuntimeError("libclang not found; install rosdep key libclang-dev")
        signatures = {
            "clang_createIndex": ([ctypes.c_int, ctypes.c_int], ctypes.c_void_p),
            "clang_parseTranslationUnit": ([ctypes.c_void_p, ctypes.c_char_p, ctypes.POINTER(ctypes.c_char_p),
                                            ctypes.c_int, ctypes.c_void_p, ctypes.c_uint, ctypes.c_uint], ctypes.c_void_p),
            "clang_getTranslationUnitCursor": ([ctypes.c_void_p], CXCursor),
            "clang_visitChildren": ([CXCursor, VISITOR, ctypes.c_void_p], ctypes.c_uint),
            "clang_getCursorSpelling": ([CXCursor], CXString),
            "clang_getCString": ([CXString], ctypes.c_char_p),
            "clang_disposeString": ([CXString], None),
            "clang_getCursorLocation": ([CXCursor], CXSourceLocation),
            "clang_getExpansionLocation": ([CXSourceLocation, ctypes.POINTER(ctypes.c_void_p), ctypes.POINTER(ctypes.c_uint),
                                            ctypes.POINTER(ctypes.c_uint), ctypes.POINTER(ctypes.c_uint)], None),
            "clang_getFileName": ([ctypes.c_void_p], CXString),
            "clang_getCursorSemanticParent": ([CXCursor], CXCursor),
            "clang_getCursorReferenced": ([CXCursor], CXCursor),
            "clang_Cursor_isNull": ([CXCursor], ctypes.c_int),
            "clang_getCursorType": ([CXCursor], CXType),
            "clang_getCanonicalType": ([CXType], CXType),
            "clang_getArrayElementType": ([CXType], CXType),
            "clang_isConstQualifiedType": ([CXType], ctypes.c_uint),
            "clang_Cursor_getStorageClass": ([CXCursor], ctypes.c_int),
            "clang_getNumDiagnostics": ([ctypes.c_void_p], ctypes.c_uint),
            "clang_getDiagnostic": ([ctypes.c_void_p, ctypes.c_uint], ctypes.c_void_p),
            "clang_getDiagnosticSeverity": ([ctypes.c_void_p], ctypes.c_int),
            "clang_formatDiagnostic": ([ctypes.c_void_p, ctypes.c_uint], CXString),
            "clang_defaultDiagnosticDisplayOptions": ([], ctypes.c_uint),
            "clang_disposeDiagnostic": ([ctypes.c_void_p], None),
            "clang_disposeTranslationUnit": ([ctypes.c_void_p], None),
            "clang_disposeIndex": ([ctypes.c_void_p], None),
        }
        for name, (arguments, result) in signatures.items():
            function = getattr(lib, name)
            function.argtypes = arguments
            function.restype = result
        self.lib = lib

    def text(self, value):
        raw = self.lib.clang_getCString(value)
        result = raw.decode("utf-8", "replace") if raw else ""
        self.lib.clang_disposeString(value)
        return result

    def spelling(self, cursor):
        return self.text(self.lib.clang_getCursorSpelling(cursor))

    def file_of(self, cursor):
        location = self.lib.clang_getCursorLocation(cursor)
        file_handle = ctypes.c_void_p()
        self.lib.clang_getExpansionLocation(location, ctypes.byref(file_handle), None, None, None)
        return os.path.realpath(self.text(self.lib.clang_getFileName(file_handle))) if file_handle.value else ""

    def line_of(self, cursor):
        location = self.lib.clang_getCursorLocation(cursor)
        line = ctypes.c_uint()
        self.lib.clang_getExpansionLocation(location, None, ctypes.byref(line), None, None)
        return line.value

    def is_null(self, cursor):
        return bool(self.lib.clang_Cursor_isNull(cursor))

    def parent(self, cursor):
        return self.lib.clang_getCursorSemanticParent(cursor)

    def qualified_name(self, cursor):
        parts = []
        current = cursor
        while not self.is_null(current) and current.kind != TRANSLATION_UNIT:
            name = self.spelling(current)
            if name and not name.startswith("__"):
                parts.append(name)
            current = self.parent(current)
        return "::".join(reversed(parts))

    def is_const(self, cursor):
        type_ = self.lib.clang_getCanonicalType(self.lib.clang_getCursorType(cursor))
        while type_.kind in ARRAY_TYPES:
            type_ = self.lib.clang_getCanonicalType(self.lib.clang_getArrayElementType(type_))
        return bool(self.lib.clang_isConstQualifiedType(type_))


def forbidden_symbol(name):
    return any(name == symbol or name.startswith(symbol + "::") for symbol in FORBIDDEN_SYMBOLS)


def ast_errors(clang, path, role, own_core, cores_dir, arguments):
    errors = []
    encoded = [argument.encode() for argument in arguments]
    array = (ctypes.c_char_p * len(encoded))(*encoded)
    index = clang.lib.clang_createIndex(0, 0)
    unit = clang.lib.clang_parseTranslationUnit(index, path.encode(), array, len(encoded), None, 0, 0)
    if not unit:
        clang.lib.clang_disposeIndex(index)
        return [f"{path}: libclang could not parse the file"]
    for i in range(clang.lib.clang_getNumDiagnostics(unit)):
        diagnostic = clang.lib.clang_getDiagnostic(unit, i)
        if clang.lib.clang_getDiagnosticSeverity(diagnostic) >= 3:
            errors.append(clang.text(clang.lib.clang_formatDiagnostic(
                diagnostic, clang.lib.clang_defaultDiagnosticDisplayOptions())))
        clang.lib.clang_disposeDiagnostic(diagnostic)
    target = os.path.realpath(path)
    own_core = os.path.realpath(own_core) if own_core else ""

    def report(cursor, message):
        errors.append(f"{path}:{clang.line_of(cursor)}: {message}")

    def visit(cursor, _parent, _data):
        if clang.file_of(cursor) != target:
            return CHILD_CONTINUE
        kind = cursor.kind
        if kind == VAR:
            parent = clang.parent(cursor)
            constant = clang.is_const(cursor)
            if parent.kind in (TRANSLATION_UNIT, NAMESPACE, LINKAGE_SPEC) and not constant:
                report(cursor, f"global mutable state '{clang.spelling(cursor)}' is not allowed")
            elif parent.kind in (STRUCT, CLASS, UNION, CLASS_TEMPLATE, PARTIAL_SPECIALIZATION) and not constant:
                report(cursor, f"static mutable member '{clang.spelling(cursor)}' is not allowed")
            elif clang.lib.clang_Cursor_getStorageClass(cursor) == STORAGE_STATIC and not constant:
                report(cursor, f"static mutable variable '{clang.spelling(cursor)}' is not allowed")
        elif kind == FUNCTION and clang.spelling(cursor) == "main":
            report(cursor, "an extension file must not define main()")
        if kind in REFERENCE_KINDS:
            referenced = clang.lib.clang_getCursorReferenced(cursor)
            if not clang.is_null(referenced):
                name = clang.qualified_name(referenced)
                if forbidden_symbol(name):
                    report(cursor, f"'{name}' is not allowed in extension files")
                declared_in = clang.file_of(referenced)
                if declared_in.startswith(cores_dir) and declared_in not in (target, own_core):
                    report(cursor, f"'{name}' belongs to another Core")
        return CHILD_RECURSE

    callback = VISITOR(visit)
    clang.lib.clang_visitChildren(clang.lib.clang_getTranslationUnitCursor(unit), callback, None)
    clang.lib.clang_disposeTranslationUnit(unit)
    clang.lib.clang_disposeIndex(index)
    return errors


def package_dependencies(package_dir):
    root = ElementTree.parse(os.path.join(package_dir, "package.xml")).getroot()
    names = set()
    for tag in ("depend", "build_depend", "build_export_depend"):
        names.update(element.text.strip() for element in root.findall(tag) if element.text)
    return names


def execution_packages(src_dir, own):
    packages = set()
    for config in glob.glob(os.path.join(src_dir, "*", "config", "*", "node.yaml")):
        name = os.path.basename(os.path.dirname(os.path.dirname(os.path.dirname(config))))
        if name != own:
            packages.add(name)
    return packages


def include_errors(path, role, stem, bank, package, dependencies, others):
    errors = []
    with open(path, encoding="utf-8") as stream:
        text = stream.read()
    allowed_roots = set(dependencies)
    for dependency in dependencies:
        allowed_roots.update(SYSTEM_DEPENDS.get(dependency, ()))
    for match in re.finditer(r'^\s*#\s*include\s*[<"]([^>"]+)[>"]', text, re.M):
        include = match.group(1)
        line = text.count("\n", 0, match.start()) + 1
        root = include.split("/")[0]
        where = f"{path}:{line}: #include <{include}>"
        if include in STD_HEADERS:
            if include in FORBIDDEN_STD:
                errors.append(f"{where} is not allowed in extension files")
            continue
        if root in FORBIDDEN_INCLUDE_ROOTS or include.startswith("XmlRpc"):
            errors.append(f"{where} is ROS communication and not allowed")
        elif root == "node_pipeline":
            if include != "node_pipeline/extension.hpp":
                errors.append(f"{where}: only node_pipeline/extension.hpp is public")
        elif root == package:
            parts = include.split("/")
            kind = parts[1] if len(parts) > 2 else ""
            permitted = kind == "libraries" or \
                (role == "core" and bank and include == f"{package}/banks/{bank}.hpp") or \
                (role == "parameters" and include in (f"{package}/cores/{stem}.hpp", f"{package}/banks/{stem}.hpp"))
            if not permitted:
                errors.append(f"{where} is not allowed for a {role} file")
        elif root in others:
            errors.append(f"{where} couples execution packages")
        elif root not in allowed_roots:
            errors.append(f"{where} is not a dependency declared in package.xml")
    return errors


def source_files(directory):
    for root, directories, files in os.walk(directory):
        directories[:] = [d for d in directories if not d.startswith(".")]
        for name in files:
            if name.endswith(SOURCE_SUFFIXES):
                yield os.path.join(root, name)


def check(package_dir, package, src_dir, include_dirs, use_ast=True):
    errors = []
    package_dir = os.path.realpath(package_dir)
    allowed_prefixes = [os.path.join("include", package, kind) + os.sep for kind in EXTENSION_KINDS]
    allowed_prefixes += [os.path.join("src", "libraries") + os.sep, "test" + os.sep]
    for path in source_files(package_dir):
        relative = os.path.relpath(path, package_dir)
        if not any(relative.startswith(prefix) for prefix in allowed_prefixes):
            errors.append(f"{relative}: C++ source outside the official extension paths")

    config_dir = os.path.join(package_dir, "config")
    nodes = sorted(d for d in os.listdir(config_dir) if os.path.isfile(os.path.join(config_dir, d, "node.yaml"))) \
        if os.path.isdir(config_dir) else []
    if not nodes:
        errors.append("config/<node>/node.yaml is required")
    for root, directories, files in os.walk(config_dir):
        directories[:] = [d for d in directories if not d.startswith(".")]
        for name in files:
            relative = os.path.relpath(os.path.join(root, name), config_dir).split(os.sep)
            valid = len(relative) == 2 and relative[0] in nodes and relative[1] in ("node.yaml", "selection.yaml")
            valid = valid or (len(relative) == 3 and relative[0] in nodes and
                              relative[1] in ("registrations", "parameters") and relative[2].endswith(".yaml"))
            if not valid:
                errors.append(f"config/{'/'.join(relative)}: unexpected configuration file")

    checked = {}
    for path in glob.glob(os.path.join(package_dir, "include", package, "libraries", "**", "*"), recursive=True) + \
            glob.glob(os.path.join(package_dir, "src", "libraries", "**", "*"), recursive=True):
        if path.endswith(SOURCE_SUFFIXES):
            checked[(path, "library")] = ("", "")
    for node in nodes:
        if not os.path.isfile(os.path.join(config_dir, node, "selection.yaml")):
            errors.append(f"config/{node}/selection.yaml is required")
        try:
            model = generator.load_node(package_dir, package, node)
        except generator.ConfigError as error:
            errors.append(str(error))
            continue
        for role, stem, core in model.extension_files():
            kind = {"core": "cores", "validator": "validators", "policy": "policies", "bank": "banks",
                    "parameters": "parameters"}[role]
            path = generator.extension_path(package_dir, package, kind, stem)
            checked[(path, role)] = (core.stem, core.bank if role == "core" else "")

    dependencies = package_dependencies(package_dir)
    others = execution_packages(src_dir, package)
    for (path, role), (stem, bank) in sorted(checked.items()):
        errors += include_errors(path, role, stem, bank, package, dependencies, others)
    if use_ast and checked:
        clang = Clang()
        arguments = ["-x", "c++", "-std=c++14", "-Wno-pragma-once-outside-header"] + \
            [f"-I{directory}" for directory in include_dirs]
        cores_dir = os.path.join(package_dir, "include", package, "cores") + os.sep
        for (path, role), (stem, _bank) in sorted(checked.items()):
            own_core = generator.extension_path(package_dir, package, "cores", stem) if role == "parameters" else ""
            errors += ast_errors(clang, path, role, own_core, cores_dir, arguments)
    return errors


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--package-dir", required=True)
    parser.add_argument("--package", required=True)
    parser.add_argument("--src-dir", required=True)
    parser.add_argument("--include-dirs-file", required=True)
    parser.add_argument("--stamp", required=True)
    args = parser.parse_args()
    with open(args.include_dirs_file, encoding="utf-8") as stream:
        include_dirs = [line.strip() for line in stream if line.strip()]
    errors = check(args.package_dir, args.package, args.src_dir, include_dirs)
    if errors:
        print(f"node_pipeline contract check failed for {args.package}:", file=sys.stderr)
        for error in errors:
            print(f"  {error}", file=sys.stderr)
        return 1
    with open(args.stamp, "w", encoding="utf-8") as stream:
        stream.write("ok\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())
