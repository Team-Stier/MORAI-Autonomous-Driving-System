#pragma once
#include <object_detector/cores/utonia_bbox.hpp>
#include <node_pipeline/extension.hpp>
#include <stdexcept>
#include <string>

namespace object_detector {
inline std::runtime_error utoniaPythonError() {
  PyObject *kind=nullptr,*value=nullptr,*trace=nullptr;
  PyErr_Fetch(&kind,&value,&trace);
  auto text=UtoniaBbox::ownPython(value ? PyObject_Str(value) : nullptr);
  const std::string message=text ? PyUnicode_AsUTF8(text.get()) : "Python asset initialization failed";
  Py_XDECREF(kind);Py_XDECREF(value);Py_XDECREF(trace);
  return std::runtime_error(message);
}

inline UtoniaBboxParameters loadUtoniaBboxParameters(const node_pipeline::ParamReader& params) {
  const auto home=params.text("python_home");
  const auto site=params.text("python_site_packages");
  const auto device=params.text("device");
  const auto expected=params.text("checkpoint_sha256");
  if (home.empty() || site.empty() || device!="cuda" ||
      expected!="680fbd523a70a11a3d0e65f3da5a52725cd065d35ab77fa597dbc8c88c2e05bc")
    throw std::invalid_argument("utonia_bbox requires the selected checkpoint and CUDA Python environment");
  if (!Py_IsInitialized()) {
    PyConfig config;PyConfig_InitPythonConfig(&config);
    config.write_bytecode=0;config.install_signal_handlers=0;config.parse_argv=0;config.site_import=0;
    const std::wstring wide_home(home.begin(),home.end());
    const std::wstring executable=wide_home+L"/bin/python3.12";
    auto status=PyConfig_SetString(&config,&config.home,wide_home.c_str());
    if (!PyStatus_Exception(status))
      status=PyConfig_SetString(&config,&config.executable,executable.c_str());
    if (!PyStatus_Exception(status)) status=Py_InitializeFromConfig(&config);
    const std::string error=PyStatus_Exception(status) ?
        (status.err_msg ? status.err_msg : "Python initialization failed") : "";
    PyConfig_Clear(&config);
    if (!error.empty()) throw std::runtime_error(error);
    PyEval_SaveThread();
  }
  const auto gil=PyGILState_Ensure();
  try {
    auto scope=UtoniaBbox::ownPython(PyDict_New());
    if (!scope || PyDict_SetItemString(scope.get(),"__builtins__",PyEval_GetBuiltins())!=0)
      throw utoniaPythonError();
    const auto put_text=[&](const char* name,const std::string& value) {
      auto text=UtoniaBbox::ownPython(PyUnicode_FromStringAndSize(value.data(),value.size()));
      if (!text || PyDict_SetItemString(scope.get(),name,text.get())!=0) throw utoniaPythonError();
    };
    put_text("python_site",site);put_text("device",device);put_text("expected_sha",expected);
    auto sources=UtoniaBbox::ownPython(PyDict_New());
    if (!sources) throw utoniaPythonError();
    for (const auto& module : params.keys("encoder_sources")) {
      const auto source=params.asset("encoder_sources/"+module);
      auto text=UtoniaBbox::ownPython(PyUnicode_FromStringAndSize(source.data(),source.size()));
      if (!text || PyDict_SetItemString(sources.get(),module.c_str(),text.get())!=0)
        throw utoniaPythonError();
    }
    if (PyDict_SetItemString(scope.get(),"sources",sources.get())!=0) throw utoniaPythonError();
    auto result=UtoniaBbox::ownPython(PyRun_String(R"PY(
import sys, types, importlib.machinery, hashlib, io
sys.path.insert(0, python_site)
sys.dont_write_bytecode=True
import torch
torch.set_num_threads(2)
if not torch.cuda.is_available():
    raise RuntimeError('utonia_bbox requires CUDA')
# Load the copied upstream modules from ParamReader assets without a runtime source-path dependency.
for package in ['utonia', 'utonia.serialization']:
    module=types.ModuleType(package)
    module.__path__=[]
    module.__package__=package
    module.__spec__=importlib.machinery.ModuleSpec(package,loader=None,is_package=True)
    sys.modules[package]=module
for name in ['utonia.serialization.hilbert','utonia.serialization.z_order',
             'utonia.serialization.default','utonia.serialization','utonia.utils',
             'utonia.structure','utonia.module','utonia.model']:
    module=sys.modules.get(name) if name=='utonia.serialization' else types.ModuleType(name)
    module.__file__='<'+name+'>'
    module.__package__=name if name=='utonia.serialization' else name.rpartition('.')[0]
    sys.modules[name]=module
    exec(compile(sources[name],module.__file__,'exec'),module.__dict__)
    parent,_,child=name.rpartition('.')
    setattr(sys.modules[parent],child,module)
)PY",Py_file_input,scope.get(),scope.get()));
    if (!result) throw utoniaPythonError();
    result=UtoniaBbox::ownPython(PyRun_String(UtoniaBbox::modelSource(),Py_file_input,scope.get(),scope.get()));
    if (!result) throw utoniaPythonError();
    const auto ptx=params.asset("hilbert_ptx_file");
    auto ptx_bytes=UtoniaBbox::ownPython(PyBytes_FromStringAndSize(ptx.data(),ptx.size()));
    if (!ptx_bytes || PyDict_SetItemString(scope.get(),"hilbert_ptx",ptx_bytes.get())!=0)
      throw utoniaPythonError();
    put_text("expected_hilbert_sha",params.text("hilbert_ptx_sha256"));
    const auto checkpoint=params.asset("checkpoint_file");
    auto bytes=UtoniaBbox::ownPython(PyBytes_FromStringAndSize(checkpoint.data(),checkpoint.size()));
    if (!bytes || PyDict_SetItemString(scope.get(),"checkpoint_bytes",bytes.get())!=0)
      throw utoniaPythonError();
    result=UtoniaBbox::ownPython(PyRun_String(R"PY(
if hashlib.sha256(checkpoint_bytes).hexdigest()!=expected_sha:
    raise ValueError('utonia_bbox checkpoint SHA-256 mismatch')
checkpoint=torch.load(io.BytesIO(checkpoint_bytes),map_location='cpu',weights_only=True)
if checkpoint['model_type']!='utonia' or checkpoint['parameter_count']!=140633847:
    raise ValueError('utonia_bbox checkpoint architecture mismatch')
if checkpoint['history_offsets_sec']!=[-.8,-.6,-.4,-.2,0.] or checkpoint['target_offsets_sec']!=[i*.5 for i in range(11)]:
    raise ValueError('utonia_bbox checkpoint time-axis mismatch')
if hashlib.sha256(hilbert_ptx).hexdigest()!=expected_hilbert_sha:
    raise ValueError("utonia_bbox Hilbert PTX SHA-256 mismatch")
predictor=_Inference(checkpoint,device,hilbert_ptx)
del checkpoint, checkpoint_bytes, sources, hilbert_ptx
)PY",Py_file_input,scope.get(),scope.get()));
    if (!result) throw utoniaPythonError();
    PyObject* callable=PyDict_GetItemString(scope.get(),"predictor");
    if (!callable || !PyCallable_Check(callable)) throw std::runtime_error("utonia_bbox predictor is not callable");
    Py_INCREF(callable);
    UtoniaBboxParameters output;output.predictor=UtoniaBbox::ownPython(callable);
    result.reset();bytes.reset();sources.reset();scope.reset();
    PyGILState_Release(gil);
    return output;
  } catch (...) {
    PyGILState_Release(gil);
    throw;
  }
}
}  // namespace object_detector
