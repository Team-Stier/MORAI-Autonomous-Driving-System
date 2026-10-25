#define PY_SSIZE_T_CLEAN
#include <object_detector/cores/utonia_bbox.hpp>
#include <object_detector/cores/tracking.hpp>
#include <object_detector/parameters/utonia_bbox.hpp>
#include <object_detector/validators/utonia_clusters.hpp>
#include <gtest/gtest.h>
#include <fstream>
#include <sstream>
#include <cstdlib>

namespace {
using object_detector::UtoniaBbox;
using object_detector::UtoniaBboxParameters;

std::string packageDir() {
  const auto* path=std::getenv("UTONIA_TEST_PACKAGE_DIR");
  return path ? path : NP_PACKAGE_DIR;
}

std::shared_ptr<PyObject> predictor(const std::string& body) {
  if (!Py_IsInitialized()) {
    const auto params=node_pipeline::ParamReader::fromFile(
        packageDir()+"/config/object_detector/parameters/utonia_bbox.yaml",packageDir());
    const auto* override_home=std::getenv("UTONIA_TEST_PYTHON_HOME");
    const std::string home=override_home ? override_home : params.text("python_home");
    const std::wstring wide(home.begin(),home.end());
    PyConfig config;PyConfig_InitPythonConfig(&config);
    config.install_signal_handlers=0;
    auto status=PyConfig_SetString(&config,&config.home,wide.c_str());
    if (!PyStatus_Exception(status)) status=Py_InitializeFromConfig(&config);
    const std::string error=PyStatus_Exception(status) ? status.err_msg : "";
    PyConfig_Clear(&config);
    if (!error.empty()) throw std::runtime_error(error);
    PyEval_SaveThread();
  }
  const auto gil = PyGILState_Ensure();
  auto scope = UtoniaBbox::ownPython(PyDict_New());
  PyDict_SetItemString(scope.get(), "__builtins__", PyEval_GetBuiltins());
  auto result = UtoniaBbox::ownPython(PyRun_String(body.c_str(), Py_file_input, scope.get(), scope.get()));
  EXPECT_NE(nullptr, result.get());
  PyObject* callback = PyDict_GetItemString(scope.get(), "predict");
  Py_XINCREF(callback);
  auto output = UtoniaBbox::ownPython(callback);
  scope.reset();
  result.reset();
  PyGILState_Release(gil);
  return output;
}

object_detector::Cloud box(double center_x) {
  object_detector::Cloud cloud;
  for (double x : {-2., 2.}) for (double y : {-1., 1.}) for (double z : {-0.5, 0.5})
    cloud.push_back(pcl::PointXYZ(center_x+x, y, z));
  return cloud;
}

const char* kStraight = R"PY(
import struct
def predict(points, mask, count):
    assert count == 1 and len(points) == 5*128*3*4 and len(mask) == 5*128
    assert all(sum(mask[h*128:(h+1)*128]) == 8 for h in range(5))
    return struct.pack('<77f', *[v for i in range(11) for v in [i*.5, 0., 0., 0., 0., 0., 1.]])
)PY";
}

TEST(UtoniaBbox, RealHistoryThenRelativeOriginMotionKeepsCurrentBboxAndTracking) {
  UtoniaBboxParameters params;
  params.predictor = predictor(kStraight);
  UtoniaBbox core(params);
  for (int i = 0; i < 8; ++i) {
    const auto objects = std::get<0>(core.process({box(10.)}, {}, ros::Time(100, i*100000000)));
    ASSERT_EQ(1u, objects.objects.size());
    EXPECT_FALSE(objects.objects[0].prediction_available);
  }
  auto output = std::get<0>(core.process({box(10.)}, {}, ros::Time(100, 800000000)));
  ASSERT_EQ(1u, output.objects.size());
  const auto& object = output.objects[0];
  ASSERT_TRUE(object.prediction_available);
  EXPECT_DOUBLE_EQ(10., object.position.x);
  EXPECT_EQ(object.position, object.bbox_pose.position);
  EXPECT_DOUBLE_EQ(4., object.bbox_size.x);
  EXPECT_DOUBLE_EQ(2., object.bbox_size.y);
  EXPECT_DOUBLE_EQ(1., object.bbox_size.z);
  for (std::size_t i = 0; i < 10; ++i)
    EXPECT_NEAR(10.+.5*(i+1), object.predicted_bbox_poses[i].position.x, 1e-6);
  object_detector::Tracking tracking;
  auto tracked = tracking.process(output, ros::Time(100, 800000000));
  EXPECT_TRUE(tracked.objects[0].prediction_available);
  EXPECT_EQ(object.predicted_bbox_poses, tracked.objects[0].predicted_bbox_poses);
  EXPECT_TRUE(output.header.stamp.isZero());
  EXPECT_TRUE(output.header.frame_id.empty());
}

TEST(UtoniaBbox, GapReentryAndBackwardTimeDoNotReuseStaleHistory) {
  UtoniaBboxParameters params;
  params.predictor = predictor(kStraight);
  UtoniaBbox core(params);
  for (int i = 0; i < 9; ++i)
    core.process({box(10.)}, {}, ros::Time(100, i*100000000));
  auto after_gap = std::get<0>(core.process({box(10.)}, {}, ros::Time(105)));
  EXPECT_FALSE(after_gap.objects[0].prediction_available);
  auto after_reset = std::get<0>(core.process({box(10.)}, {}, ros::Time(90)));
  EXPECT_FALSE(after_reset.objects[0].prediction_available);
  EXPECT_TRUE(std::get<0>(core.process({}, {}, ros::Time(90,100000000))).objects.empty());
}

TEST(UtoniaBbox, BatchesReadyObjectsAndAssociatesHistoryAfterOrderChanges) {
  UtoniaBboxParameters params;
  params.predictor=predictor(R"PY(
import struct
def predict(points, mask, count):
    assert count==2 and len(points)==2*5*128*3*4 and len(mask)==2*5*128
    xyz=struct.unpack('<'+str(len(points)//4)+'f',points)
    # Current output order is x=30 then x=10; all histories must follow each object.
    assert xyz[0]>20. and xyz[5*128*3]<20.
    poses=[v for i in range(11) for v in [i*.5,0.,0.,0.,0.,0.,1.]]
    return struct.pack('<154f',*(poses+poses))
)PY");
  UtoniaBbox core(params);
  for (int i=0;i<8;++i)
    core.process({box(10.),box(30.)},{},ros::Time(100,i*100000000));
  const auto output=std::get<0>(core.process({box(30.),box(10.)},{},ros::Time(100,800000000)));
  ASSERT_EQ(2u,output.objects.size());
  ASSERT_TRUE(output.objects[0].prediction_available);
  ASSERT_TRUE(output.objects[1].prediction_available);
  EXPECT_NEAR(35.,output.objects[0].predicted_bbox_poses[9].position.x,1e-6);
  EXPECT_NEAR(15.,output.objects[1].predicted_bbox_poses[9].position.x,1e-6);
}

TEST(UtoniaBbox, FiveObjectSceneUsesOneInferenceCall) {
  UtoniaBboxParameters params;
  params.predictor=predictor(R"PY(
import struct
def predict(points, mask, count):
    assert count==5
    poses=[v for i in range(11) for v in [i*.5,0.,0.,0.,0.,0.,1.]]
    return struct.pack('<385f',*(poses*count))
)PY");
  UtoniaBbox core(params);
  object_detector::Clusters clusters;
  for (int i=0;i<5;++i) clusters.push_back(box(10.+20.*i));
  interfaces::Objects output;
  for (int i=0;i<9;++i) output=std::get<0>(core.process(clusters,{},ros::Time(100,i*100000000)));
  ASSERT_EQ(5u,output.objects.size());
  for (std::size_t i=0;i<5;++i) {
    ASSERT_TRUE(output.objects[i].prediction_available);
    EXPECT_NEAR(15.+20.*i,output.objects[i].predicted_bbox_poses[9].position.x,1e-6);
  }
}

TEST(UtoniaBbox, FullRigidOffsetRotatesWithOriginInsteadOfAddingMapOffset) {
  UtoniaBboxParameters params;
  params.predictor = predictor(R"PY(
import struct, math
def predict(points, mask, count):
    poses = [0., 0., 0., 0., 0., 0., 1.]
    poses += [v for i in range(10) for v in [5., 7., 2., 0., 0., math.sqrt(.5), math.sqrt(.5)]]
    return struct.pack('<77f', *poses)
)PY");
  UtoniaBbox core(params);
  interfaces::Objects output;
  for (int i=0; i<9; ++i)
    output = std::get<0>(core.process({box(10.)}, {}, ros::Time(100,i*100000000)));
  ASSERT_TRUE(output.objects[0].prediction_available);
  const auto& future = output.objects[0].predicted_bbox_poses[0];
  EXPECT_NEAR(5., future.position.x, 2e-6);
  EXPECT_NEAR(17., future.position.y, 2e-6);
  EXPECT_NEAR(2., future.position.z, 2e-6);
  EXPECT_NEAR(std::sqrt(.5), future.orientation.z, 1e-6);
  EXPECT_NEAR(std::sqrt(.5), future.orientation.w, 1e-6);
}

TEST(UtoniaBbox, FullRigidTransformMatchesOriginalGtForFourVehicleModels) {
  std::ifstream stream(packageDir()+"/test/data/utonia_gt_bbox.csv");
  ASSERT_TRUE(stream.good());
  std::string line;
  std::getline(stream,line);
  std::size_t count=0;
  while (std::getline(stream,line)) {
    std::replace(line.begin(),line.end(),',',' ');
    std::istringstream row(line);
    std::array<geometry_msgs::Pose,4> poses;
    for (auto& pose : poses) {
      auto& p=pose.position; auto& q=pose.orientation;
      ASSERT_TRUE(bool(row>>p.x>>p.y>>p.z>>q.x>>q.y>>q.z>>q.w));
    }
    std::array<geometry_msgs::Pose,11> origins;
    origins.fill(poses[2]);origins[0]=poses[0];
    const auto actual=UtoniaBbox::bboxPoses(origins,poses[1])[0];
    const auto& expected=poses[3];
    const auto& a=actual.position;const auto& b=expected.position;
    EXPECT_LE(std::sqrt((a.x-b.x)*(a.x-b.x)+(a.y-b.y)*(a.y-b.y)+(a.z-b.z)*(a.z-b.z)),.003);
    const auto& q=actual.orientation;const auto& r=expected.orientation;
    EXPECT_NEAR(1.,std::abs(q.x*r.x+q.y*r.y+q.z*r.z+q.w*r.w),1e-12);
    ++count;
  }
  EXPECT_EQ(100u,count);
}

TEST(UtoniaBbox, RejectsUnsupportedClustersInValidatorAndKeepsEmptySceneValid) {
  object_detector::UtoniaClusters validator;
  std::vector<node_pipeline::Kind> issues;
  validator({}, {},issues);
  EXPECT_TRUE(issues.empty());
  object_detector::Cloud small;small.resize(5);
  validator({small}, {},issues);
  EXPECT_NE(issues.end(),std::find(issues.begin(),issues.end(),node_pipeline::Kind::OutRangeNegative));
  issues.clear();
  auto cloud=box(10.);cloud[0].x=std::numeric_limits<float>::quiet_NaN();
  validator({cloud}, {},issues);
  EXPECT_NE(issues.end(),std::find(issues.begin(),issues.end(),node_pipeline::Kind::NaN));
}

TEST(UtoniaBbox, InferenceExceptionLeavesObservationAndNextCallRecovers) {
  UtoniaBboxParameters params;
  params.predictor=predictor(R"PY(
import struct
calls=0
def predict(points, mask, count):
    global calls
    calls+=1
    if calls==1: raise RuntimeError('controlled regression failure')
    return struct.pack('<77f', *[v for i in range(11) for v in [i*.5,0.,0.,0.,0.,0.,1.]])
)PY");
  UtoniaBbox core(params);
  interfaces::Objects output;
  for (int i=0;i<9;++i) output=std::get<0>(core.process({box(10.)},{},ros::Time(100,i*100000000)));
  ASSERT_EQ(1u,output.objects.size());
  EXPECT_FALSE(output.objects[0].prediction_available);
  EXPECT_DOUBLE_EQ(10.,output.objects[0].position.x);
  output=std::get<0>(core.process({box(10.)},{},ros::Time(100,900000000)));
  EXPECT_TRUE(output.objects[0].prediction_available);
}

TEST(UtoniaBbox, DISABLED_SelectedCheckpointRunsThroughNativeCore) {
  const auto params=node_pipeline::ParamReader::fromFile(
      packageDir()+"/config/object_detector/parameters/utonia_bbox.yaml",packageDir());
  auto parameters=object_detector::loadUtoniaBboxParameters(params);
  std::ifstream source(packageDir()+"/test/data/utonia_input.bin",std::ios::binary);
  ASSERT_TRUE(source.good());
  std::array<float,5*128*3> points{};
  std::array<unsigned char,5*128> mask{};
  source.read(reinterpret_cast<char*>(points.data()),points.size()*sizeof(float));
  source.read(reinterpret_cast<char*>(mask.data()),mask.size());
  ASSERT_TRUE(source.good());
  UtoniaBbox core(parameters);
  interfaces::Objects output;
  for (std::size_t h=0;h<5;++h) {
    object_detector::Cloud cloud;
    for (std::size_t p=0;p<128;++p) if (mask[h*128+p]) {
      const auto* v=points.data()+(h*128+p)*3;
      cloud.push_back(pcl::PointXYZ(v[0],v[1],v[2]));
    }
    output=std::get<0>(core.process({cloud},{},ros::Time(100,h*200000000)));
  }
  ASSERT_EQ(1u,output.objects.size());
  ASSERT_TRUE(output.objects[0].prediction_available);
  std::ifstream expected(packageDir()+"/test/data/utonia_expected_origin.bin",std::ios::binary);
  ASSERT_TRUE(expected.good());
  std::array<float,77> raw{};
  expected.read(reinterpret_cast<char*>(raw.data()),sizeof(raw));
  ASSERT_TRUE(expected.good());
  std::array<geometry_msgs::Pose,11> origins;
  for (std::size_t i=0;i<11;++i) {
    const auto* v=raw.data()+7*i;auto& p=origins[i].position;auto& q=origins[i].orientation;
    p.x=v[0];p.y=v[1];p.z=v[2];q.x=v[3];q.y=v[4];q.z=v[5];q.w=v[6];
    const auto norm=std::sqrt(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w);
    q.x/=norm;q.y/=norm;q.z/=norm;q.w/=norm;
  }
  const auto wanted=UtoniaBbox::bboxPoses(origins,output.objects[0].bbox_pose);
  for (std::size_t i=0;i<10;++i) {
    const auto& a=output.objects[0].predicted_bbox_poses[i];const auto& b=wanted[i];
    EXPECT_NEAR(a.position.x,b.position.x,1e-4);
    EXPECT_NEAR(a.position.y,b.position.y,1e-4);
    EXPECT_NEAR(a.position.z,b.position.z,1e-4);
    EXPECT_NEAR(a.orientation.x,b.orientation.x,1e-5);
    EXPECT_NEAR(a.orientation.y,b.orientation.y,1e-5);
    EXPECT_NEAR(a.orientation.z,b.orientation.z,1e-5);
    EXPECT_NEAR(a.orientation.w,b.orientation.w,1e-5);
  }
}

TEST(UtoniaBbox, DISABLED_FusedSpatialCodesMatchOfficialIntegerCodes) {
  const auto params=node_pipeline::ParamReader::fromFile(
      packageDir()+"/config/object_detector/parameters/utonia_bbox.yaml",packageDir());
  auto parameters=object_detector::loadUtoniaBboxParameters(params);
  const auto gil=PyGILState_Ensure();
  auto scope=UtoniaBbox::ownPython(PyDict_New());
  PyDict_SetItemString(scope.get(),"__builtins__",PyEval_GetBuiltins());
  PyDict_SetItemString(scope.get(),"predictor",parameters.predictor.get());
  auto result=UtoniaBbox::ownPython(PyRun_String(R"PY(
import torch
from utonia.serialization.hilbert import encode
from utonia.serialization.z_order import xyz2key
generator=torch.Generator().manual_seed(20261025)
fused=predictor.hilbert
for depth in range(1,17):
    for dtype in (torch.int32,torch.int64):
        points=torch.randint(0,1<<depth,(257,3),dtype=dtype,generator=generator)
        for coordinates in (points,points[:,[1,0,2]]):
            expected=encode(coordinates,3,depth)
            actual=fused(coordinates.to('cuda'),3,depth).cpu()
            assert torch.equal(expected,actual), (depth,dtype)
            expected_morton=xyz2key(*coordinates.unbind(1),depth=depth)
            actual_morton=fused.morton(coordinates.to('cuda'),depth).cpu()
            assert torch.equal(expected_morton,actual_morton), (depth,dtype,'morton')
empty=fused(torch.empty((0,3),dtype=torch.int32,device='cuda'),3,16)
assert empty.dtype==torch.int64 and empty.shape==(0,)
empty_morton=fused.morton(torch.empty((0,3),dtype=torch.int64,device='cuda'),16)
assert empty_morton.dtype==torch.int64 and empty_morton.shape==(0,)
)PY",Py_file_input,scope.get(),scope.get()));
  const bool passed=bool(result);
  if (!passed) PyErr_Print();
  result.reset();scope.reset();
  PyGILState_Release(gil);
  ASSERT_TRUE(passed);
}
