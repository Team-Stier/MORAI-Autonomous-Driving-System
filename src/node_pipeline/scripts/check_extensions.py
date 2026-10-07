#!/usr/bin/env python3
"""Mandatory, location-scoped Clang AST checks of package-owned extensions."""
import argparse
import ctypes as C
import ctypes.util
import json
import mmap
from pathlib import Path
import re
import shlex
import shutil
import sys


class ContractError(ValueError):
    pass


class Cursor(C.Structure):
    _fields_ = [("kind", C.c_uint), ("xdata", C.c_int), ("data", C.c_void_p * 3)]


class Type(C.Structure):
    _fields_ = [("kind", C.c_int), ("data", C.c_void_p * 2)]


class String(C.Structure):
    _fields_ = [("data", C.c_void_p), ("flags", C.c_uint)]


class Location(C.Structure):
    _fields_ = [("data", C.c_void_p * 2), ("int_data", C.c_uint)]


class Range(C.Structure):
    _fields_ = [("data", C.c_void_p * 2), ("begin", C.c_uint), ("end", C.c_uint)]


Visitor = C.CFUNCTYPE(C.c_uint, Cursor, Cursor, C.c_void_p)


class Clang:
    def __init__(self, executable):
        executable = shutil.which(executable)
        if not executable:
            raise ContractError("mandatory clang++ executable is unavailable")
        prefix = Path(executable).resolve().parent.parent
        candidates = [prefix / "lib" / "libclang.so.1", prefix / "lib" / "libclang.so",
                      prefix / "lib" / "libclang.dylib", prefix / "lib" / "libclang.dll"]
        library = next((str(path) for path in candidates if path.is_file()), None)
        library = library or ctypes.util.find_library("clang")
        if not library:
            raise ContractError("mandatory libclang AST library is unavailable for " + executable)
        self.lib = C.CDLL(library)
        functions = {
            "createIndex": (C.c_void_p, [C.c_int, C.c_int]),
            "disposeIndex": (None, [C.c_void_p]),
            "parseTranslationUnit2": (C.c_int, [C.c_void_p, C.c_char_p, C.POINTER(C.c_char_p), C.c_int, C.c_void_p, C.c_uint, C.c_uint, C.POINTER(C.c_void_p)]),
            "disposeTranslationUnit": (None, [C.c_void_p]),
            "getTranslationUnitCursor": (Cursor, [C.c_void_p]),
            "visitChildren": (C.c_uint, [Cursor, Visitor, C.c_void_p]),
            "hashCursor": (C.c_uint, [Cursor]),
            "getCursorKindSpelling": (String, [C.c_uint]),
            "getCursorSpelling": (String, [Cursor]),
            "getCursorUSR": (String, [Cursor]),
            "getCursorSemanticParent": (Cursor, [Cursor]),
            "getCursorReferenced": (Cursor, [Cursor]),
            "getCanonicalCursor": (Cursor, [Cursor]),
            "getCursorType": (Type, [Cursor]),
            "getCanonicalType": (Type, [Type]),
            "isConstQualifiedType": (C.c_uint, [Type]),
            "CXXField_isMutable": (C.c_uint, [Cursor]),
            "Cursor_getStorageClass": (C.c_uint, [Cursor]),
            "getTypeSpelling": (String, [Type]),
            "getTypeDeclaration": (Cursor, [Type]),
            "getPointeeType": (Type, [Type]),
            "getArrayElementType": (Type, [Type]),
            "Type_getSizeOf": (C.c_longlong, [Type]),
            "getResultType": (Type, [Type]),
            "getNumArgTypes": (C.c_int, [Type]),
            "getArgType": (Type, [Type, C.c_uint]),
            "Type_getNumTemplateArguments": (C.c_int, [Type]),
            "Type_getTemplateArgumentAsType": (Type, [Type, C.c_uint]),
            "getCursorLocation": (Location, [Cursor]),
            "Location_isInSystemHeader": (C.c_int, [Location]),
            "getCursorExtent": (Range, [Cursor]),
            "getRangeStart": (Location, [Range]),
            "getRangeEnd": (Location, [Range]),
            "getTypeKindSpelling": (String, [C.c_int]),
            "getExpansionLocation": (None, [Location, C.POINTER(C.c_void_p), C.POINTER(C.c_uint), C.POINTER(C.c_uint), C.POINTER(C.c_uint)]),
            "getFileName": (String, [C.c_void_p]),
            "getIncludedFile": (C.c_void_p, [Cursor]),
            "getCString": (C.c_char_p, [String]),
            "disposeString": (None, [String]),
            "getNumDiagnostics": (C.c_uint, [C.c_void_p]),
            "getDiagnostic": (C.c_void_p, [C.c_void_p, C.c_uint]),
            "getDiagnosticSeverity": (C.c_uint, [C.c_void_p]),
            "formatDiagnostic": (String, [C.c_void_p, C.c_uint]),
            "disposeDiagnostic": (None, [C.c_void_p]),
            "Cursor_getNumArguments": (C.c_int, [Cursor]),
            "Cursor_getArgument": (Cursor, [Cursor, C.c_uint]),
        }
        for name, (result, arguments) in functions.items():
            function = getattr(self.lib, "clang_" + name)
            function.restype, function.argtypes = result, arguments
            setattr(self, name, function)
        self.kind_names = {}
        self.names = {}
        self.files = {}
        self.numeric = re.compile(rb"(?<=[,{])\s*[-+]?(?:[0-9]+(?:\.[0-9]*)?|\.[0-9]+)(?:[eE][-+]?[0-9]+)?[fFlL]?(?=[\s,}])")

    @staticmethod
    def key(cursor):
        # clang_hashCursor is a 32-bit hash, not a unique AST identity.
        return cursor.kind, cursor.xdata, tuple(cursor.data)

    def text(self, value):
        raw = self.getCString(value)
        result = raw.decode("utf-8", errors="replace") if raw else ""
        self.disposeString(value)
        return result

    def kind(self, cursor):
        if cursor.kind not in self.kind_names:
            self.kind_names[cursor.kind] = self.text(self.getCursorKindSpelling(cursor.kind))
        return self.kind_names[cursor.kind]

    def qualified(self, cursor):
        cursor = self.getCanonicalCursor(cursor)
        key = self.key(cursor)
        if key in self.names:
            return self.names[key]
        if self.kind(cursor) in {"InvalidFile", "NoDeclFound", "NotImplemented", "InvalidCode", "TranslationUnit"}:
            return ""
        spelling = self.text(self.getCursorSpelling(cursor))
        parent = self.qualified(self.getCursorSemanticParent(cursor))
        result = parent + "::" + spelling if parent and spelling else parent or spelling
        self.names[key] = result
        return result

    def location(self, cursor):
        file, line, column, offset = C.c_void_p(), C.c_uint(), C.c_uint(), C.c_uint()
        self.getExpansionLocation(self.getCursorLocation(cursor), C.byref(file), C.byref(line), C.byref(column), C.byref(offset))
        if file.value not in self.files:
            path = self.text(self.getFileName(file)) if file.value else ""
            self.files[file.value] = str(Path(path).resolve()) if path else ""
        return self.files[file.value], line.value

    def children(self, cursor):
        result = []

        @Visitor
        def visit(child, parent, data):
            result.append(child)
            return 1

        self.visitChildren(cursor, visit, None)
        return result

    def numeric_initializer(self, cursor):
        """Skip only large literal arrays of builtin arithmetic values."""
        value_type = self.getCanonicalType(self.getCursorType(cursor))
        array_size = self.Type_getSizeOf(value_type)
        element = self.getArrayElementType(value_type)
        if not element.kind:
            return False
        while element.kind:
            value_type, element = element, self.getArrayElementType(element)
        if self.text(self.getTypeKindSpelling(value_type.kind)) not in {"Bool", "Char_U", "UChar", "Char16", "Char32", "UShort", "UInt", "ULong", "ULongLong", "UInt128", "Char_S", "SChar", "WChar", "Short", "Int", "Long", "LongLong", "Int128", "Float", "Double", "LongDouble", "Half", "Float16"}:
            return False
        extent = self.getCursorExtent(cursor)
        ranges = []
        for location in (self.getRangeStart(extent), self.getRangeEnd(extent)):
            file, line, column, offset = C.c_void_p(), C.c_uint(), C.c_uint(), C.c_uint()
            self.getExpansionLocation(location, C.byref(file), C.byref(line), C.byref(column), C.byref(offset))
            ranges.append((file.value, offset.value))
        same_file = ranges[0][0] and ranges[0][0] == ranges[1][0]
        source_size = ranges[1][1] - ranges[0][1] if same_file else 0
        if source_size < 65536 and array_size < 65536:
            return False
        if same_file:
            path = self.text(self.getFileName(C.c_void_p(ranges[0][0])))
            with open(path, "rb") as source, mmap.mmap(source.fileno(), 0, access=mmap.ACCESS_READ) as data:
                text = data[ranges[0][1]:ranges[1][1]]
                if not self.numeric.sub(b"", text).translate(None, b" \t\r\n,{}"):
                    return True
        # Included literals can have a short source range but a large array
        # type. Inspect their AST without retaining millions of child cursors.
        literal_kinds = {"InitListExpr", "IntegerLiteral", "FloatingLiteral", "CXXBoolLiteralExpr", "UnexposedExpr"}
        sources = {}

        @Visitor
        def visit(child, parent, data):
            kind = self.kind(child)
            if kind == "UnaryOperator":
                file, line, column, offset = C.c_void_p(), C.c_uint(), C.c_uint(), C.c_uint()
                self.getExpansionLocation(self.getRangeStart(self.getCursorExtent(child)), C.byref(file), C.byref(line), C.byref(column), C.byref(offset))
                if not file.value:
                    return 0
                try:
                    if file.value not in sources:
                        path = self.text(self.getFileName(file))
                        source = open(path, "rb")
                        sources[file.value] = (source, mmap.mmap(source.fileno(), 0, access=mmap.ACCESS_READ))
                    if sources[file.value][1][offset.value:offset.value + 1] not in {b"+", b"-"}:
                        return 0
                except (OSError, ValueError):
                    return 0
                return 2
            return 2 if kind in literal_kinds else 0

        try:
            return not self.visitChildren(cursor, visit, None)
        finally:
            for source, data in sources.values():
                data.close()
                source.close()


def compile_flags(database, unit):
    unit = Path(unit).resolve()
    entries = json.loads(Path(database).read_text())
    entry = next((item for item in entries if (Path(item["directory"]) / item["file"]).resolve() == unit), None)
    if not entry:
        raise ContractError("no real compile command for generated translation unit: " + str(unit))
    arguments = entry.get("arguments") or shlex.split(entry["command"])
    result, index = [], 1
    while index < len(arguments):
        argument = arguments[index]
        if argument in {"-o", "-MF", "-MT", "-MQ"}:
            index += 2
            continue
        if argument in {"-c", "-MD", "-MMD", "-MP"} or (not argument.startswith("-") and (Path(entry["directory"]) / argument).resolve() == unit):
            index += 1
            continue
        result.append(argument)
        index += 1
    # libclang does not run a driver; explicitly preserve the command's working directory.
    result += ["-working-directory=" + entry["directory"]]
    return result


# A public type does not authorize access to its implementation members.
SDK_MEMBERS = {
    "Parameters": {"contains", "require", "get", "child", "openAsset", "Parameters", "~Parameters"},
    "InputView": {"present", "get", "modify", "remove", "~InputView"},
    "ProfileHealth": {"inputs", "issues", "ProfileHealth", "~ProfileHealth"},
    "InputHealth": {"input", "issues", "green", "InputHealth", "~InputHealth"},
    "Issue": {"input", "kind", "Issue", "~Issue"},
    "Kind": {"Missing", "InvalidStamp", "FutureStamp", "FrameMismatch", "NaN", "Inf", "OutRangePositive",
             "OutRangeNegative", "Stale", "ReceiveTimeout", "Skew", "Count"},
    "Visualizer": {"advertise", "Visualizer", "~Visualizer"},
    "VisualizerPublisher": {"publish", "VisualizerPublisher", "~VisualizerPublisher"},
    "RemoveInvalid": {"apply", "RemoveInvalid", "~RemoveInvalid"},
    "NoParameters": {"operator()", "NoParameters", "~NoParameters"},
    "NoValidator": {"operator()", "NoValidator", "~NoValidator"},
}
ROLE_TYPES = {
    "core": {"Visualizer", "VisualizerPublisher"}, "bank": set(),
    "parameter_loader": {"Parameters", "NoParameters"},
    "validator": {"InputView", "ProfileHealth", "InputHealth", "Kind", "NoValidator"},
    "policy": {"InputView", "ProfileHealth", "InputHealth", "Kind", "RemoveInvalid", "Issue", "highestPriorityIssue"},
    "helper": {"Parameters", "InputView", "ProfileHealth", "InputHealth", "Kind"},
}
FILE_TYPES = {"basic_ifstream", "basic_ofstream", "basic_fstream", "basic_filebuf", "ifstream", "ofstream", "fstream", "filebuf"}
EXTERNAL_APIS = {
    "fopen", "freopen", "fclose", "open", "openat", "creat", "read", "write", "pread", "pwrite", "remove", "rename", "unlink", "mmap",
    "tmpfile", "tmpnam", "tempnam", "fdopen", "fflush", "setbuf", "setvbuf", "setbuffer", "setlinebuf",
    "fread", "fwrite", "fgetc", "fgets", "fputc", "fputs", "getc", "getchar", "putc", "putchar", "puts", "ungetc",
    "fprintf", "fscanf", "printf", "scanf", "vfprintf", "vfscanf", "vprintf", "vscanf", "perror",
    "fgetpos", "fsetpos", "fseek", "ftell", "rewind", "clearerr", "feof", "ferror", "fileno", "fseeko", "ftello",
    "getline", "getdelim", "flockfile", "ftrylockfile", "funlockfile", "sync_with_stdio",
    "fopen64", "freopen64", "tmpfile64", "fgetpos64", "fsetpos64", "fseeko64", "ftello64",
    "fread_unlocked", "fwrite_unlocked", "fflush_unlocked", "fgetc_unlocked", "fgets_unlocked", "fputc_unlocked", "fputs_unlocked",
    "getc_unlocked", "getchar_unlocked", "putc_unlocked", "putchar_unlocked", "clearerr_unlocked", "feof_unlocked", "ferror_unlocked", "fileno_unlocked",
    "fwide", "fgetwc", "getwc", "getwchar", "fputwc", "putwc", "putwchar", "fgetws", "fputws", "ungetwc",
    "fwprintf", "fwscanf", "wprintf", "wscanf", "vfwprintf", "vfwscanf", "vwprintf", "vwscanf",
    "system", "popen", "pclose", "fork", "vfork", "execve", "execvp", "execl", "execlp", "posix_spawn", "exit", "_Exit", "quick_exit",
    "socket", "connect", "bind", "listen", "accept", "send", "sendto", "recv", "recvfrom", "getenv", "setenv", "putenv",
    "clock", "time", "gettimeofday", "clock_gettime", "sleep", "usleep", "nanosleep",
}
SYSTEM_IO_HEADERS = re.compile(
    r"/(?:stdio(?:_ext)?|unistd|fcntl|dirent|poll|spawn|dlfcn|mqueue|semaphore|netdb|signal|"
    r"sys/(?:socket|uio|wait|mman|select|ipc|msg|sem|shm|stat|statvfs|file|ioctl|poll|epoll|eventfd|signalfd|timerfd|inotify|sendfile|resource|prctl)|"
    r"bits/(?:stdio2?|unistd(?:_ext)?|fcntl2?|socket2?|poll2?|mqueue2?|dlfcn))\.h$")
STDIO_MEMORY_APIS = {"sprintf", "snprintf", "vsprintf", "vsnprintf", "sscanf", "vsscanf", "asprintf", "vasprintf",
                    "__sprintf_chk", "__snprintf_chk", "__vsprintf_chk", "__vsnprintf_chk", "__isoc99_sscanf", "__isoc99_vsscanf"}
STDLIB_VALUE_APIS = {
    "malloc", "calloc", "realloc", "reallocarray", "free", "aligned_alloc", "posix_memalign", "valloc", "memalign",
    "abs", "labs", "llabs", "div", "ldiv", "lldiv", "atoi", "atol", "atoll", "atof", "qsort", "qsort_r", "bsearch",
    "mblen", "mbtowc", "wctomb", "mbstowcs", "wcstombs", "a64l", "rand_r", "random_r", "srandom_r", "initstate_r", "setstate_r",
    "drand48_r", "erand48_r", "lrand48_r", "nrand48_r", "mrand48_r", "jrand48_r", "srand48_r", "seed48_r", "lcong48_r",
}
DECLARATIONS = {"StructDecl", "ClassDecl", "ClassTemplate", "ClassTemplatePartialSpecialization"}
REFERENCE_KINDS = {"TypeRef", "TemplateRef", "DeclRefExpr", "MemberRefExpr", "CallExpr", "CXXBaseSpecifier"}
TYPE_KINDS = {"VarDecl", "FieldDecl", "ParmDecl", "FunctionDecl", "CXXMethod", "TypedefDecl", "TypeAliasDecl"} | REFERENCE_KINDS


def check(manifest, database, unit, executable):
    clang = Clang(executable)
    index = clang.createIndex(0, 0)
    translation_unit = C.c_void_p()
    arguments = compile_flags(database, unit)
    array = (C.c_char_p * len(arguments))(*(arg.encode() for arg in arguments))
    # Detailed preprocessing records expose actual included files even for macro includes.
    parsed = clang.parseTranslationUnit2(index, str(Path(unit).resolve()).encode(), array, len(arguments), None, 0, 1, C.byref(translation_unit))
    if parsed or not translation_unit.value:
        clang.disposeIndex(index)
        raise ContractError("Clang could not parse the generated translation unit (error " + str(parsed) + ")")
    try:
        diagnostics = []
        for number in range(clang.getNumDiagnostics(translation_unit)):
            diagnostic = clang.getDiagnostic(translation_unit, number)
            if clang.getDiagnosticSeverity(diagnostic) >= 3:
                diagnostics.append(clang.text(clang.formatDiagnostic(diagnostic, 3)))
            clang.disposeDiagnostic(diagnostic)
        if diagnostics:
            raise ContractError("Clang compile errors:\n" + "\n".join(diagnostics))
        package = Path(manifest["package_dir"]).resolve()
        extensions = {}
        for item in manifest["extensions"]:
            extensions.setdefault(str(package / "include" / item["header"]), []).append(item)
        owned, parents, core_declarations = [], {}, []
        errors = set()

        @Visitor
        def collect(cursor, parent, data):
            path, line = clang.location(cursor)
            if "/include/" in path and "/cores/" in path and clang.kind(cursor) in DECLARATIONS:
                core_declarations.append(cursor)
            if path.startswith(str(package) + "/"):
                if clang.kind(cursor) == "InitListExpr" and clang.numeric_initializer(cursor):
                    return 1
                if path != str(Path(unit).resolve()):
                    owned.append(cursor)
                    parents[clang.key(cursor)] = parent
                return 2
            # Keep every namespace open: owned global definitions and explicit
            # specializations can be outside the package namespace. Library
            # class/function bodies have no owned extension source to inspect.
            return 2 if clang.kind(cursor) in {"Namespace", "LinkageSpec", "TranslationUnit"} else 1

        clang.visitChildren(clang.getTranslationUnitCursor(translation_unit), collect, None)
        known_cores = {core["type"] for core in manifest["cores"]}
        banks = {bank["type"] for bank in manifest["banks"].values()}
        # An unregistered process class is still a computation Core, not a helper escape.
        for cursor in core_declarations + owned:
            if clang.kind(cursor) in DECLARATIONS:
                if any(clang.kind(child) in {"CXXMethod", "FunctionTemplate"} and clang.text(clang.getCursorSpelling(child)) == "process" for child in clang.children(cursor)):
                    class_name = clang.qualified(cursor)
                    if class_name not in banks:
                        known_cores.add(class_name)

        def report(cursor, message):
            path, line = clang.location(cursor)
            errors.add(path + ":" + str(line) + ": " + message)

        includes = {}
        for cursor in owned:
            if clang.kind(cursor) == "inclusion directive":
                included = clang.getIncludedFile(cursor)
                if included:
                    path, _ = clang.location(cursor)
                    includes.setdefault(path, set()).add(str(Path(clang.text(clang.getFileName(included))).resolve()))
        include_owners = {}
        for header in extensions:
            pending, seen = [header], set()
            while pending:
                path = pending.pop()
                if path not in seen:
                    seen.add(path)
                    include_owners.setdefault(path, set()).add(header)
                    pending.extend(includes.get(path, ()))
        strategy_roles = {}
        for path, owners in include_owners.items():
            roles = [role for owner in owners for role in extensions[owner]]
            if roles and all(role["role"] == "policy" for role in roles) and any(
                    Path(owner).parent.name == "policies" and path.startswith(str(Path(owner).parent) + "/") for owner in owners):
                strategy_roles[path] = roles

        def role_for(cursor):
            path, _ = clang.location(cursor)
            roles = extensions.get(path, []) or strategy_roles.get(path, [])
            return roles[0] if roles else {"role": "helper", "core": ""}

        def check_name(cursor, target, constructing=False):
            if not target:
                return
            role = role_for(cursor)
            for core in known_cores:
                if target == core or target.startswith(core + "::"):
                    allowed = role["core"] == core and (role["role"] == "core" or (role["role"] == "parameter_loader" and not constructing))
                    if not allowed:
                        report(cursor, "other Core ownership/reference/call is forbidden: " + core)
            if target.startswith("node_pipeline::"):
                parts = target[len("node_pipeline::"):].split("::")
                head = parts[0]
                allowed = head in ROLE_TYPES.get(role["role"], set())
                if len(parts) > 1:
                    allowed = allowed and len(parts) == 2 and parts[1] in SDK_MEMBERS.get(head, set())
                if head == "InputView" and len(parts) > 1 and parts[1] in {"modify", "remove"}:
                    allowed = allowed and role["role"] == "policy"
                if not allowed:
                    report(cursor, "Pipeline internals or SDK member outside extension role are forbidden: " + target)
            if target.startswith("ros::"):
                head = target[len("ros::"):].split("::")[0]
                if head not in {"Time", "TimeBase", "Duration", "DurationBase", "WallDuration"}:
                    report(cursor, "raw ROS API is forbidden: " + target)
            parts = target.split("::")
            if any(part in FILE_TYPES for part in parts) or target.startswith(("std::filesystem::", "std::experimental::filesystem::")):
                report(cursor, "direct file access is forbidden; use Parameters.openAsset: " + target)
            if target.startswith(("boost::filesystem::", "boost::asio::", "zmq::", "mqtt::", "httplib::", "curl_")):
                report(cursor, "direct file/communication API is forbidden: " + target)
            referenced = clang.getCanonicalCursor(clang.getCursorReferenced(cursor))
            declaration_path, _ = clang.location(referenced)
            name, header = parts[-1], Path(declaration_path).name
            stdlib_value = name in STDLIB_VALUE_APIS or name.startswith(("strto", "strfrom"))
            wide_value = name.startswith(("wcs", "wmem", "mb")) or name in {"btowc", "wctob", "swprintf", "vswprintf", "swscanf", "vswscanf"}
            system_io = clang.kind(referenced) in {"FunctionDecl", "FunctionTemplate"} and clang.Location_isInSystemHeader(clang.getCursorLocation(referenced)) and (
                bool(SYSTEM_IO_HEADERS.search(declaration_path)) or
                (header in {"stdlib.h", "stdlib2.h", "stdlib-float.h"} and not stdlib_value) or
                (header in {"wchar.h", "wchar2.h"} and not wide_value))
            system_io = system_io and name not in STDIO_MEMORY_APIS and name != "swab"
            if ((len(parts) == 1 or parts[0] == "std") and name in EXTERNAL_APIS) or system_io:
                arity = clang.Cursor_getNumArguments(referenced)
                if arity < 0:
                    arity = sum(clang.kind(child) == "ParmDecl" for child in clang.children(referenced))
                remove_algorithm = parts[0] == "std" and parts[-1] == "remove" and arity == 3
                stream_getline = parts[0] == "std" and parts[-1] == "getline"
                if not (remove_algorithm or stream_getline):
                    report(cursor, "external file/process/communication/clock API is forbidden: " + target)
            if (target.startswith("std::chrono::") and target.endswith("::now")) or any(
                    part in {"jthread", "async", "const_pointer_cast", "reinterpret_pointer_cast"} for part in parts) or target == "std::thread::detach":
                report(cursor, "external execution/clock or constness bypass is forbidden: " + target)

        def check_type(cursor, type_value, seen=None):
            seen = seen or set()
            if not type_value.kind:
                return
            type_value = clang.getCanonicalType(type_value)
            spelling = clang.text(clang.getTypeSpelling(type_value))
            if spelling in seen:
                return
            seen.add(spelling)
            declaration = clang.getTypeDeclaration(type_value)
            check_name(cursor, clang.qualified(declaration))
            for child_type in (clang.getPointeeType(type_value), clang.getArrayElementType(type_value), clang.getResultType(type_value)):
                if child_type.kind:
                    check_type(cursor, child_type, seen)
            for number in range(max(0, clang.getNumArgTypes(type_value))):
                check_type(cursor, clang.getArgType(type_value, number), seen)
            for number in range(max(0, clang.Type_getNumTemplateArguments(type_value))):
                check_type(cursor, clang.Type_getTemplateArgumentAsType(type_value, number), seen)

        def mutable_indirection(type_value, seen=None):
            seen = seen or set()
            type_value = clang.getCanonicalType(type_value)
            spelling = clang.text(clang.getTypeSpelling(type_value))
            if spelling in seen:
                return False
            seen.add(spelling)
            pointee = clang.getPointeeType(type_value)
            if pointee.kind and clang.text(clang.getTypeKindSpelling(pointee.kind)) not in {"FunctionProto", "FunctionNoProto"} and (
                    not clang.isConstQualifiedType(pointee) or mutable_indirection(pointee, seen)):
                return True
            element = clang.getArrayElementType(type_value)
            if element.kind and mutable_indirection(element, seen):
                return True
            declaration = clang.getTypeDeclaration(type_value)
            if clang.qualified(declaration) == "std::function":
                return True
            if clang.qualified(declaration) in {"std::shared_ptr", "std::unique_ptr", "std::weak_ptr", "std::reference_wrapper", "boost::shared_ptr", "boost::weak_ptr"}:
                pointee = clang.Type_getTemplateArgumentAsType(type_value, 0)
                if pointee.kind and (not clang.isConstQualifiedType(pointee) or mutable_indirection(pointee, seen)):
                    return True
            path, _ = clang.location(declaration)
            if path.startswith(str(package) + "/"):
                for field in clang.children(declaration):
                    if clang.kind(field) == "FieldDecl" and (clang.CXXField_isMutable(field) or mutable_indirection(clang.getCursorType(field), seen)):
                        return True
            for number in range(max(0, clang.Type_getNumTemplateArguments(type_value))):
                argument = clang.Type_getTemplateArgumentAsType(type_value, number)
                if argument.kind and mutable_indirection(argument, seen):
                    return True
            return False

        def mutable_shared(cursor):
            if clang.kind(cursor) != "VarDecl":
                return False
            parent = clang.getCursorSemanticParent(cursor)
            shared = clang.kind(parent) in DECLARATIONS | {"Namespace", "TranslationUnit", "LinkageSpec"} or clang.Cursor_getStorageClass(cursor) in {2, 3, 4}
            value_type = clang.getCanonicalType(clang.getCursorType(cursor))
            return shared and (not clang.isConstQualifiedType(value_type) or mutable_indirection(value_type))

        def native_ros_value_method(target):
            prefixes = ("Time::", "TimeBase::", "Duration::", "DurationBase::", "WallDuration::")
            operations = {"Time", "TimeBase", "Duration", "DurationBase", "WallDuration", "toSec", "toNSec",
                          "fromSec", "fromNSec", "isZero", "operator+", "operator-", "operator+=", "operator-=",
                          "operator=", "operator==", "operator!=", "operator<", "operator<=", "operator>", "operator>=",
                          "operator*", "operator/", "operator*=", "operator/=", "~Time", "~Duration", "~WallDuration"}
            return any(target.startswith("ros::" + prefix) for prefix in prefixes) and target.split("::")[-1] in operations

        for cursor in owned:
            kind = clang.kind(cursor)
            if kind == "inclusion directive":
                included = clang.getIncludedFile(cursor)
                included_path = clang.text(clang.getFileName(included)) if included else ""
                if "/node_pipeline/" in included_path and not included_path.endswith("/node_pipeline/sdk.hpp"):
                    report(cursor, "only node_pipeline/sdk.hpp may be included: " + included_path)
            if kind == "FunctionDecl" and clang.text(clang.getCursorSpelling(cursor)) == "main":
                report(cursor, "extension main entry point is forbidden")
            if kind == "UnionDecl" and any(clang.getPointeeType(clang.getCursorType(field)).kind for field in clang.children(cursor) if clang.kind(field) == "FieldDecl"):
                report(cursor, "raw pointer/reference aliasing union is forbidden")
            if kind in DECLARATIONS | {"Namespace", "FunctionDecl", "CXXMethod"}:
                own_name = clang.qualified(cursor)
                if own_name.startswith("node_pipeline::"):
                    report(cursor, "extension cannot define or specialize Pipeline internals: " + own_name)
            if kind in TYPE_KINDS:
                check_type(cursor, clang.getCursorType(cursor))
            if kind in REFERENCE_KINDS:
                referenced = clang.getCursorReferenced(cursor)
                target = clang.qualified(referenced)
                check_name(cursor, target, kind == "CallExpr")
                if mutable_shared(referenced):
                    report(cursor, "extern/global mutable shared state is forbidden: " + target)
                if target.startswith("ros::") and clang.kind(referenced) in {"FunctionDecl", "CXXMethod", "FunctionTemplate"} and not native_ros_value_method(target):
                    report(cursor, "unauthorized ROS call or API reference: " + target)
                if kind == "CallExpr" and target.split("::")[-1] in {"memcpy", "memmove"} and clang.Cursor_getNumArguments(cursor):
                    argument = clang.Cursor_getArgument(cursor, 0)
                    for expression in clang.children(argument):
                        destination = clang.getPointeeType(clang.getCanonicalType(clang.getCursorType(expression)))
                        if destination.kind and clang.getPointeeType(destination).kind:
                            report(cursor, "raw pointer/reference byte-copy alias bypass is forbidden")
            if mutable_shared(cursor):
                report(cursor, "global/static mutable shared state is forbidden")
            if kind in {"CXXConstCastExpr", "CXXReinterpretCastExpr"}:
                report(cursor, "constness/raw address cast bypass is forbidden")
            if kind == "CStyleCastExpr":
                result_type = clang.getCanonicalType(clang.getCursorType(cursor))
                type_kind = clang.text(clang.getTypeKindSpelling(result_type.kind))
                if type_kind in {"Pointer", "LValueReference", "RValueReference", "MemberPointer"}:
                    report(cursor, "C-style raw pointer/reference cast is forbidden")
        if errors:
            raise ContractError("\n".join(sorted(errors)))
        return len(owned)
    finally:
        clang.disposeTranslationUnit(translation_unit)
        clang.disposeIndex(index)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", required=True)
    parser.add_argument("--compile-commands", required=True)
    parser.add_argument("--translation-unit", required=True)
    parser.add_argument("--clang", default="clang++")
    args = parser.parse_args()
    try:
        count = check(json.loads(Path(args.manifest).read_text()), args.compile_commands, args.translation_unit, args.clang)
        print("Clang extension contract passed (" + str(count) + " owned AST nodes)")
    except (ContractError, OSError, ValueError) as error:
        print("extension contract error: " + str(error), file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
