#pragma once
#ifndef PY_SSIZE_T_CLEAN
#define PY_SSIZE_T_CLEAN
#endif
#include <Python.h>
#include <interfaces/Objects.h>
#include <interfaces/CompetitionVehicleStatus.h>
#include <object_detector/libraries/types.hpp>
#include <ros/console.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <deque>
#include <limits>
#include <memory>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

namespace object_detector {
struct UtoniaBboxParameters {
  std::shared_ptr<PyObject> predictor;
};

class UtoniaBbox {
 public:
  explicit UtoniaBbox(UtoniaBboxParameters parameters) : parameters_(std::move(parameters)) {}

  static std::shared_ptr<PyObject> ownPython(PyObject* object) {
    return std::shared_ptr<PyObject>(object, [](PyObject* value) {
      if (value && Py_IsInitialized()) {
        const auto gil = PyGILState_Ensure();
        Py_DECREF(value);
        PyGILState_Release(gil);
      }
    });
  }

  std::tuple<interfaces::Objects, interfaces::CompetitionVehicleStatus> process(
      const Clusters& clusters, const interfaces::CompetitionVehicleStatus& competition,
      const ros::Time& stamp) {
    const auto now = stamp.toNSec();
    if (last_stamp_ != 0 && (now <= last_stamp_ || now-last_stamp_ > 250000000)) tracks_.clear();
    last_stamp_ = now;
    interfaces::Objects output;
    std::vector<std::array<double,3>> centers;
    for (const auto& cloud : clusters) {
      output.objects.push_back(makeObb(cloud));
      std::array<double,3> center{};
      for (const auto& point : cloud) {center[0]+=point.x; center[1]+=point.y; center[2]+=point.z;}
      for (auto& value : center) value /= cloud.size();
      centers.push_back(center);
    }
    const auto matches = associate(centers);
    std::vector<Track> next;
    std::vector<float> points;
    std::vector<unsigned char> masks;
    std::vector<std::size_t> ready;
    for (std::size_t i=0; i<clusters.size(); ++i) {
      Track track;
      if (matches[i] >= 0) track = std::move(tracks_[matches[i]]);
      track.center = centers[i];
      track.frames.push_back({now, clusters[i]});
      while (track.frames.size()>12 || now-track.frames.front().stamp>1000000000) track.frames.pop_front();
      std::array<std::size_t,5> indices{};
      bool complete = now>=800000000;
      for (std::size_t h=0; h<5 && complete; ++h) {
        const auto wanted = now-(4-h)*200000000;
        std::uint64_t best = std::numeric_limits<std::uint64_t>::max();
        for (std::size_t j=0; j<track.frames.size(); ++j) {
          const auto t = track.frames[j].stamp;
          const auto skew = t>wanted ? t-wanted : wanted-t;
          if (skew<=best) {best=skew; indices[h]=j;}
        }
        complete = best<=80000000 && (h==0 || indices[h]>indices[h-1]);
      }
      if (complete) {
        ready.push_back(i);
        for (auto index : indices) {
          const auto& cloud = track.frames[index].cloud;
          const std::size_t count=std::min<std::size_t>(128,cloud.size());
          for (std::size_t p=0; p<128; ++p) {
            masks.push_back(p<count);
            if (p<count) {
              const auto& point=cloud[p*(cloud.size()-1)/(count-1)];
              points.insert(points.end(), {point.x,point.y,point.z});
            } else points.insert(points.end(), {0.f,0.f,0.f});
          }
        }
      }
      next.push_back(std::move(track));
    }
    tracks_ = std::move(next);
    // One scene batch avoids repeated host dispatch delaying the next observation.
    if (!ready.empty()) {
      const auto count=ready.size();
      std::vector<float> poses(count*77);
      if (infer(points.data(),masks.data(),count,poses)) {
        for (std::size_t j=0;j<count;++j) {
          auto& object = output.objects[ready[j]];
          std::array<geometry_msgs::Pose,11> origins;
          for (std::size_t h=0; h<11; ++h) origins[h]=poseFrom(poses.data()+77*j+7*h);
          object.predicted_bbox_poses = bboxPoses(origins,object.bbox_pose);
          object.prediction_available = true;
        }
      }
    }
    return std::make_tuple(std::move(output),competition);
  }

  static const char* modelSource();
  static const char* hilbertSource();

  static boost::array<geometry_msgs::Pose,10> bboxPoses(
      const std::array<geometry_msgs::Pose,11>& origins,const geometry_msgs::Pose& observed_bbox) {
    const auto offset=compose(inverse(origins[0]),observed_bbox);
    boost::array<geometry_msgs::Pose,10> result;
    for (std::size_t i=0; i<10; ++i) result[i]=compose(origins[i+1],offset);
    return result;
  }

 private:
  struct Frame {std::uint64_t stamp; Cloud cloud;};
  struct Track {std::array<double,3> center{}; std::deque<Frame> frames;};
  UtoniaBboxParameters parameters_;
  std::vector<Track> tracks_;
  std::uint64_t last_stamp_=0;

  std::vector<int> associate(const std::vector<std::array<double,3>>& centers) const {
    const std::size_t rows=centers.size(), cols=tracks_.size(), n=std::max(rows,cols);
    std::vector<std::vector<double>> cost(n,std::vector<double>(n,1000000.));
    for (std::size_t i=0; i<rows; ++i) for (std::size_t j=0; j<cols; ++j) {
      double d2=0.;
      for (std::size_t k=0; k<3; ++k) {const double d=centers[i][k]-tracks_[j].center[k]; d2+=d*d;}
      if (d2<=25.) cost[i][j]=std::sqrt(d2);
    }
    std::vector<double> u(n+1),v(n+1);
    std::vector<std::size_t> p(n+1),way(n+1);
    for (std::size_t i=1; i<=n; ++i) {
      p[0]=i; std::size_t j0=0;
      std::vector<double> minimum(n+1,std::numeric_limits<double>::infinity());
      std::vector<bool> used(n+1,false);
      do {
        used[j0]=true; const auto i0=p[j0];
        double delta=std::numeric_limits<double>::infinity(); std::size_t j1=0;
        for (std::size_t j=1; j<=n; ++j) if (!used[j]) {
          const double cur=cost[i0-1][j-1]-u[i0]-v[j];
          if (cur<minimum[j]) {minimum[j]=cur; way[j]=j0;}
          if (minimum[j]<delta) {delta=minimum[j]; j1=j;}
        }
        for (std::size_t j=0; j<=n; ++j) {
          if (used[j]) {u[p[j]]+=delta; v[j]-=delta;} else minimum[j]-=delta;
        }
        j0=j1;
      } while (p[j0]!=0);
      do {const auto j1=way[j0]; p[j0]=p[j1]; j0=j1;} while (j0!=0);
    }
    std::vector<int> result(rows,-1);
    for (std::size_t j=1; j<=cols; ++j)
      if (p[j]>0 && p[j]<=rows && cost[p[j]-1][j-1]<=5.) result[p[j]-1]=static_cast<int>(j-1);
    return result;
  }

  bool infer(const float* points, const unsigned char* masks,std::size_t batch,std::vector<float>& output) const {
    const auto gil=PyGILState_Ensure();
    auto xyz=ownPython(PyBytes_FromStringAndSize(reinterpret_cast<const char*>(points),batch*5*128*3*sizeof(float)));
    auto mask=ownPython(PyBytes_FromStringAndSize(reinterpret_cast<const char*>(masks),batch*5*128));
    auto count=ownPython(PyLong_FromSize_t(batch));
    auto value=ownPython(xyz && mask && count ? PyObject_CallFunctionObjArgs(
        parameters_.predictor.get(),xyz.get(),mask.get(),count.get(),nullptr) : nullptr);
    const bool success=value && PyBytes_Check(value.get()) && PyBytes_Size(value.get())==Py_ssize_t(batch*77*sizeof(float));
    if (success) std::memcpy(output.data(),PyBytes_AsString(value.get()),batch*77*sizeof(float));
    else {
      PyObject *kind=nullptr,*error=nullptr,*trace=nullptr;
      PyErr_Fetch(&kind,&error,&trace);
      auto text=ownPython(error ? PyObject_Str(error) : nullptr);
      ROS_ERROR("Utonia inference produced no trajectory: %s",text ? PyUnicode_AsUTF8(text.get()) : "invalid output buffer");
      Py_XDECREF(kind); Py_XDECREF(error); Py_XDECREF(trace);
    }
    value.reset(); count.reset(); mask.reset(); xyz.reset();
    PyGILState_Release(gil);
    return success;
  }

  static geometry_msgs::Pose poseFrom(const float* v) {
    geometry_msgs::Pose pose;
    pose.position.x=v[0]; pose.position.y=v[1]; pose.position.z=v[2];
    const double norm=std::sqrt(double(v[3])*v[3]+double(v[4])*v[4]+double(v[5])*v[5]+double(v[6])*v[6]);
    pose.orientation.x=v[3]/norm; pose.orientation.y=v[4]/norm;
    pose.orientation.z=v[5]/norm; pose.orientation.w=v[6]/norm;
    return pose;
  }
  static geometry_msgs::Point rotate(const geometry_msgs::Quaternion& q,const geometry_msgs::Point& p) {
    const double tx=2.*(q.y*p.z-q.z*p.y),ty=2.*(q.z*p.x-q.x*p.z),tz=2.*(q.x*p.y-q.y*p.x);
    geometry_msgs::Point r;
    r.x=p.x+q.w*tx+q.y*tz-q.z*ty; r.y=p.y+q.w*ty+q.z*tx-q.x*tz; r.z=p.z+q.w*tz+q.x*ty-q.y*tx;
    return r;
  }
  static geometry_msgs::Pose compose(const geometry_msgs::Pose& a,const geometry_msgs::Pose& b) {
    geometry_msgs::Pose c; c.position=rotate(a.orientation,b.position);
    c.position.x+=a.position.x; c.position.y+=a.position.y; c.position.z+=a.position.z;
    const auto& q=a.orientation; const auto& r=b.orientation; auto& s=c.orientation;
    s.x=q.w*r.x+q.x*r.w+q.y*r.z-q.z*r.y;
    s.y=q.w*r.y-q.x*r.z+q.y*r.w+q.z*r.x;
    s.z=q.w*r.z+q.x*r.y-q.y*r.x+q.z*r.w;
    s.w=q.w*r.w-q.x*r.x-q.y*r.y-q.z*r.z;
    return c;
  }
  static geometry_msgs::Pose inverse(const geometry_msgs::Pose& a) {
    geometry_msgs::Pose result; result.orientation=a.orientation;
    result.orientation.x*=-1.;result.orientation.y*=-1.;result.orientation.z*=-1.;
    geometry_msgs::Point negative;negative.x=-a.position.x;negative.y=-a.position.y;negative.z=-a.position.z;
    result.position=rotate(result.orientation,negative);return result;
  }
  using XY = std::pair<double, double>;
  static constexpr double kPi = 3.14159265358979323846;

  static interfaces::Object makeAabb(const Cloud& cloud) {
    auto minimum = cloud.front();
    auto maximum = minimum;
    for (const auto& point : cloud) {
      minimum.x = std::min(minimum.x, point.x);
      minimum.y = std::min(minimum.y, point.y);
      minimum.z = std::min(minimum.z, point.z);
      maximum.x = std::max(maximum.x, point.x);
      maximum.y = std::max(maximum.y, point.y);
      maximum.z = std::max(maximum.z, point.z);
    }
    interfaces::Object object;
    // AABB uses the extrema midpoint, not the point centroid; sizes are full lengths.
    object.position.x = (static_cast<double>(minimum.x) + maximum.x) / 2.0;
    object.position.y = (static_cast<double>(minimum.y) + maximum.y) / 2.0;
    object.position.z = (static_cast<double>(minimum.z) + maximum.z) / 2.0;
    object.bbox_pose.position = object.position;
    object.bbox_pose.orientation.x = 0.0;
    object.bbox_pose.orientation.y = 0.0;
    object.bbox_pose.orientation.z = 0.0;
    object.bbox_pose.orientation.w = 1.0;
    object.bbox_size.x = static_cast<double>(maximum.x) - minimum.x;
    object.bbox_size.y = static_cast<double>(maximum.y) - minimum.y;
    object.bbox_size.z = static_cast<double>(maximum.z) - minimum.z;
    object.label = "unknown";
    object.velocity_available = false;
    object.acceleration_available = false;
    return object;
  }

  static double canonicalYaw(double yaw_rad) {
    yaw_rad = std::fmod(yaw_rad + kPi / 2.0, kPi);
    if (yaw_rad < 0.0) yaw_rad += kPi;
    return yaw_rad - kPi / 2.0;
  }

  static double cross(const XY& a, const XY& b, const XY& c) {
    return (b.first-a.first)*(c.second-a.second) -
           (b.second-a.second)*(c.first-a.first);
  }

  static std::vector<XY> convexHull(std::vector<XY> points) {
    std::sort(points.begin(), points.end());
    points.erase(std::unique(points.begin(), points.end()), points.end());
    if (points.size() <= 2) return points;
    std::vector<XY> hull;
    hull.reserve(2 * points.size());
    for (const auto& point : points) {
      while (hull.size() >= 2 && cross(hull[hull.size()-2], hull.back(), point) <= 0.0)
        hull.pop_back();
      hull.push_back(point);
    }
    const auto lower_size = hull.size();
    for (auto it = points.rbegin()+1; it != points.rend(); ++it) {
      while (hull.size() > lower_size && cross(hull[hull.size()-2], hull.back(), *it) <= 0.0)
        hull.pop_back();
      hull.push_back(*it);
    }
    hull.pop_back();  // The first point occurs at both ends.
    return hull;
  }

  struct Rectangle {
    double center_x, center_y, length, width, yaw_rad;
  };

  static Rectangle rectangleAt(const std::vector<XY>& hull, const XY& origin, double yaw_rad) {
    const double c = std::cos(yaw_rad), s = std::sin(yaw_rad);
    double min_u = std::numeric_limits<double>::infinity(), min_v = min_u;
    double max_u = -min_u, max_v = -min_u;
    for (const auto& point : hull) {
      // Translate before projecting to reduce cancellation under translation.
      const double x = point.first-origin.first, y = point.second-origin.second;
      const double u = c*x + s*y, v = -s*x + c*y;
      min_u = std::min(min_u, u); max_u = std::max(max_u, u);
      min_v = std::min(min_v, v); max_v = std::max(max_v, v);
    }
    const double u = (min_u+max_u)/2.0, v = (min_v+max_v)/2.0;
    Rectangle rectangle{origin.first+c*u-s*v, origin.second+s*u+c*v,
                        max_u-min_u, max_v-min_v, yaw_rad};
    if (rectangle.length < rectangle.width) {
      std::swap(rectangle.length, rectangle.width);
      rectangle.yaw_rad += kPi/2.0;
    }
    rectangle.yaw_rad = canonicalYaw(rectangle.yaw_rad);
    return rectangle;
  }

  static int compareScalar(double a, double b) {
    // Roundoff ties only: no geometric zero-area threshold or float-size padding.
    const double tolerance = 32.0 * std::numeric_limits<double>::epsilon() *
                             std::max(std::abs(a), std::abs(b));
    if (a < b-tolerance) return -1;
    if (a > b+tolerance) return 1;
    return 0;
  }

  static bool precedes(const Rectangle& a, const Rectangle& b) {
    // Minimum area, then shorter long/short sides, then |canonical yaw|, then yaw.
    for (const auto& values : {XY{a.length*a.width, b.length*b.width},
                              XY{a.length, b.length}, XY{a.width, b.width},
                              XY{std::abs(a.yaw_rad), std::abs(b.yaw_rad)}}) {
      const int order = compareScalar(values.first, values.second);
      if (order != 0) return order < 0;
    }
    return a.yaw_rad < b.yaw_rad;
  }

  static interfaces::Object makeObb(const Cloud& cloud) {
    auto object = makeAabb(cloud);  // Retains Z extrema and Object semantics.
    std::vector<XY> points;
    points.reserve(cloud.size());
    for (const auto& point : cloud) points.emplace_back(point.x, point.y);
    const auto hull = convexHull(std::move(points));
    // One XY point: zero XY extent and identity yaw, possibly nonzero Z extent.
    if (hull.size() == 1) return object;

    Rectangle best{};
    if (hull.size() == 2) {
      // Two points or exact collinearity: endpoints define a zero-width line box.
      const auto& a = hull.front();
      const auto& b = hull.back();
      best = {(a.first+b.first)/2.0, (a.second+b.second)/2.0,
              std::hypot(b.first-a.first, b.second-a.second), 0.0,
              canonicalYaw(std::atan2(b.second-a.second, b.first-a.first))};
    } else {
      // Every minimum-area rectangle has a side parallel to a convex-hull edge.
      // O(n log n + h^2); h is the number of unique convex-hull vertices.
      // Near-zero area hulls use this same calculation without division by area.
      const auto& origin = hull.front();
      best = rectangleAt(hull, origin, std::atan2(hull[1].second-origin.second,
                                                 hull[1].first-origin.first));
      for (std::size_t i = 1; i < hull.size(); ++i) {
        const auto& a = hull[i];
        const auto& b = hull[(i+1) % hull.size()];
        const auto candidate = rectangleAt(hull, origin, std::atan2(b.second-a.second,
                                                                   b.first-a.first));
        if (precedes(candidate, best)) best = candidate;
      }
    }
    object.position.x = best.center_x;
    object.position.y = best.center_y;
    object.bbox_pose.position = object.position;
    object.bbox_pose.orientation.z = std::sin(best.yaw_rad/2.0);
    object.bbox_pose.orientation.w = std::cos(best.yaw_rad/2.0);
    object.bbox_size.x = best.length;
    object.bbox_size.y = best.width;
    return object;
  }
};
}  // namespace object_detector

namespace object_detector {
inline const char* UtoniaBbox::modelSource() { return R"UTONIA(
import torch
from torch import nn
from torch.nn import functional as F
from torch.nn.attention import SDPBackend, sdpa_kernel
from types import MethodType
from utonia.model import PointTransformerV3, SerializedAttention

def normalize_quaternion(q):
    norm=q.norm(dim=-1,keepdim=True)
    identity=torch.zeros_like(q); identity[...,3]=1.
    return torch.where(norm>1e-8,q/norm.clamp_min(1e-8),identity)

OFFICIAL_CONFIG = dict(
    in_channels=9, order=('z', 'z-trans', 'hilbert', 'hilbert-trans'),
    stride=(2, 2, 2, 2), enc_depths=(3, 3, 3, 12, 3),
    enc_channels=(54, 108, 216, 432, 576), enc_num_head=(3, 6, 12, 24, 32),
    enc_patch_size=(1024, 1024, 1024, 1024, 1024), mlp_ratio=4,
    qkv_bias=True, qk_scale=None, attn_drop=0., proj_drop=0., drop_path=.3,
    shuffle_orders=True, pre_norm=True, enable_rpe=False, enable_flash=True,
    upcast_attention=False, upcast_softmax=False, traceable=True,
    enc_mode=True, mask_token=True, rope_base=10)


def _tensor_attention(qkv_flat, coord, slots, valid, order, inverse, inv_freq, num_heads, scale):
    channels = qkv_flat.shape[-1] // 3
    qkv = qkv_flat[order].reshape(-1, 3, num_heads, channels // num_heads)
    q, k, v = qkv.unbind(dim=1)
    angles = coord[order].unsqueeze(-1) * inv_freq
    angles = torch.cat((angles, angles), dim=-1).flatten(1).unsqueeze(1)
    cosine, sine = (angles.cos(), angles.sin())
    chunk = q.shape[-1] // 3
    q_axes = q.reshape(q.shape[0], q.shape[1], 3, chunk)
    k_axes = k.reshape(k.shape[0], k.shape[1], 3, chunk)
    q_rotate = torch.cat((-q_axes[..., chunk // 2:], q_axes[..., :chunk // 2]), dim=-1).flatten(-2)
    k_rotate = torch.cat((-k_axes[..., chunk // 2:], k_axes[..., :chunk // 2]), dim=-1).flatten(-2)
    q = q * cosine + q_rotate * sine
    k = k * cosine + k_rotate * sine
    packed = qkv.new_zeros((valid.numel(), 3, num_heads, channels // num_heads))
    packed = packed.index_copy(0, slots, torch.stack((q, k, v), dim=1))
    q, k, v = [value.transpose(1, 2).float() for value in packed.reshape(valid.shape[0], valid.shape[1], 3, num_heads, -1).unbind(dim=2)]
    features = F.scaled_dot_product_attention(q, k, v, attn_mask=valid[:, None, None, :], dropout_p=0.0, scale=scale)
    return features.transpose(1, 2).reshape(-1, channels)[slots][inverse]


def _ragged_attention(self, point):
    if '_trajectory_attention_layout' not in point:
        starts=F.pad(point.offset,(1,0))[:-1]
        lengths=point.offset-starts
        maximum=int(lengths.max().item())
        cloud=point.batch[point.serialized_order[0]]
        slots=cloud*maximum+torch.arange(len(cloud),device=cloud.device)-starts[cloud]
        valid=torch.arange(maximum,device=cloud.device)[None,:]<lengths[:,None]
        point['_trajectory_attention_layout']=slots,valid
    slots,valid=point['_trajectory_attention_layout']
    qkv=self.qkv(point.feat)
    with sdpa_kernel(SDPBackend.MATH):
        feature=self.tensor_attention(qkv,point.coord,slots,valid,
            point.serialized_order[self.order_index],point.serialized_inverse[self.order_index],
            self.rope.inv_freq,self.num_heads,self.scale)
    point.feat=self.proj_drop(self.proj(feature))
    return point


class UtoniaTrajectoryModel(nn.Module):
    def __init__(self, pretrained_path=None, width=256, heads=8, temporal_layers=4,
                 dropout=.1, min_points=6, grid_size=.05):
        super().__init__()
        if (width < 1 or heads < 1 or width % heads or temporal_layers < 1
                or not 6 <= min_points <= 128 or not 0 <= dropout < 1
                or not 0 < grid_size < float('inf')):
            raise ValueError('Valid attention sizes, minimum 6 points, dropout and grid size are required')
        self.config = dict(width=width, heads=heads, temporal_layers=temporal_layers,
                           dropout=dropout, min_points=min_points, grid_size=grid_size)
        self.min_points = min_points
        self.grid_size = grid_size
        self.register_buffer('xyz_mean', torch.zeros(3))
        self.register_buffer('xyz_scale', torch.ones(3))
        self.encoder = PointTransformerV3(**dict(OFFICIAL_CONFIG, enable_flash=False,
            upcast_attention=True, upcast_softmax=True, shuffle_orders=False))
        tensor_attention=torch.compile(_tensor_attention,dynamic=True,fullgraph=True,
            options={"triton.cudagraphs":False,"max_autotune":False})
        for module in self.encoder.modules():
            if isinstance(module, SerializedAttention):
                module.tensor_attention=tensor_attention
                module.forward = MethodType(_ragged_attention, module)
        self.frame_projection = nn.Sequential(nn.LayerNorm(576), nn.Linear(576, width))
        self.history_embedding = nn.Parameter(torch.randn(1, 5, width) * .02)
        self.future_queries = nn.Parameter(torch.randn(1, 11, width) * .02)
        self.temporal = nn.ModuleList([
            nn.TransformerEncoderLayer(width, heads, 4 * width, dropout,
                                       activation='gelu', batch_first=True, norm_first=True)
            for _ in range(temporal_layers)])
        self.temporal_norm = nn.LayerNorm(width)
        self.pose_head = nn.Sequential(nn.Linear(width, width), nn.GELU(), nn.Linear(width, 7))
        nn.init.normal_(self.pose_head[-1].weight, std=.001)
        nn.init.zeros_(self.pose_head[-1].bias)
        with torch.no_grad():
            self.pose_head[-1].bias[6] = 1.

    def forward(self, points, point_mask):
        batch_size = points.shape[0]
        cloud_count = batch_size * 5
        coord = points[point_mask] - self.xyz_mean
        # Fixed train-global map reference, meter scale and grid origin across every frame.
        grid = torch.floor(coord / self.grid_size).long() + 32768
        cloud = torch.arange(cloud_count, device=points.device).reshape(batch_size, 5, 1)
        cloud = cloud.expand(-1, -1, 128)[point_mask]
        keys, inverse, counts = torch.unique(torch.cat((cloud[:, None], grid), dim=1),
                                             dim=0, return_inverse=True, return_counts=True)
        voxel_coord = coord.new_zeros((len(keys), 3)).index_add(0, inverse, coord)
        voxel_coord = voxel_coord / counts[:, None]
        encoded = self.encoder(dict(coord=voxel_coord, grid_coord=keys[:, 1:].int(),
                                    batch=keys[:, 0], feat=torch.cat((voxel_coord,
                                    voxel_coord.new_zeros((len(keys), 6))), dim=-1)))
        pooled = encoded.feat.new_zeros((cloud_count, 576)).index_add(0, encoded.batch, encoded.feat)
        pooled = pooled / torch.bincount(encoded.batch, minlength=cloud_count)[:, None]
        frames = self.frame_projection(pooled).reshape(batch_size, 5, -1) + self.history_embedding
        sequence = torch.cat((frames, self.future_queries.expand(batch_size, -1, -1)), dim=1)
        for layer in self.temporal:
            sequence = layer(sequence)
        raw_pose = self.pose_head(self.temporal_norm(sequence[:, 5:]))
        normalized = torch.where(point_mask[..., None], (points - self.xyz_mean) / self.xyz_scale, 0.)
        anchor = normalized[:, -1].sum(dim=1) / point_mask[:, -1].sum(dim=1, keepdim=True)
        position = (raw_pose[..., :3] + anchor[:, None]) * self.xyz_scale + self.xyz_mean
        return torch.cat((position, normalize_quaternion(raw_pose[..., 3:])), dim=-1)


class _FusedHilbert:
    def __init__(self, ptx_bytes):
        import ctypes as C
        self.C=C
        self.driver=C.CDLL('libcuda.so.1')
        self.driver.cuModuleLoadData.argtypes=[C.POINTER(C.c_void_p),C.c_void_p]
        self.driver.cuModuleGetFunction.argtypes=[C.POINTER(C.c_void_p),C.c_void_p,C.c_char_p]
        self.driver.cuModuleUnload.argtypes=[C.c_void_p]
        self.driver.cuLaunchKernel.argtypes=[C.c_void_p,C.c_uint,C.c_uint,C.c_uint,
            C.c_uint,C.c_uint,C.c_uint,C.c_uint,C.c_void_p,C.POINTER(C.c_void_p),C.c_void_p]
        for name in ('cuModuleLoadData','cuModuleGetFunction','cuModuleUnload','cuLaunchKernel'):
            getattr(self.driver,name).restype=C.c_int
        self.module=C.c_void_p()
        self.function=C.c_void_p()
        self.morton_function=C.c_void_p()
        self.ptx=C.create_string_buffer(ptx_bytes)
        torch.cuda.check_error(self.driver.cuModuleLoadData(C.byref(self.module),self.ptx))
        torch.cuda.check_error(self.driver.cuModuleGetFunction(C.byref(self.function),self.module,b'fused_hilbert'))
        torch.cuda.check_error(self.driver.cuModuleGetFunction(C.byref(self.morton_function),self.module,b'fused_morton'))
    def __del__(self):
        module=getattr(self,'module',None)
        if module and module.value:
            self.driver.cuModuleUnload(module)
    def __call__(self, coords, num_dims, num_bits):
        return self._encode(self.function,coords,num_bits)
    def morton(self, coords, depth=16):
        return self._encode(self.morton_function,coords,depth)
    def _encode(self, function, coords, depth):
        C=self.C
        coords=coords.contiguous()
        output=torch.empty(len(coords),dtype=torch.int64,device=coords.device)
        if not len(coords):
            return output
        values=[C.c_void_p(coords.data_ptr()),C.c_void_p(output.data_ptr()),
                C.c_int(len(coords)),C.c_int(depth),C.c_int(coords.element_size())]
        arguments=(C.c_void_p*len(values))(*[C.addressof(value) for value in values])
        stream=torch.cuda.current_stream(coords.device)
        torch.cuda.check_error(self.driver.cuLaunchKernel(function,(len(coords)+127)//128,1,1,
            128,1,1,0,stream.cuda_stream,arguments,None))
        coords.record_stream(stream)
        return output

class _Inference:
    def __init__(self, checkpoint, device, hilbert_ptx):
        import torch._inductor.config
        import torch._dynamo.config
        torch.set_num_threads(2)
        torch._inductor.config.compile_threads=1
        torch._dynamo.config.cache_size_limit=64
        self.model=UtoniaTrajectoryModel(**checkpoint['model_config'])
        self.model.load_state_dict(checkpoint['model_state'],strict=True)
        self.model.eval().to(device)
        self.device=device
        self.hilbert=_FusedHilbert(hilbert_ptx)
        import utonia.serialization.default as serialization
        serialization.hilbert_encode_=self.hilbert
        serialization.z_order_encode=self.hilbert.morton
        torch.set_float32_matmul_precision('highest')
        torch.backends.cuda.matmul.allow_tf32=False
        torch.backends.cudnn.allow_tf32=False
        # Compile dense and single-voxel attention before live callbacks.
        with torch.inference_mode():
            warm=self.model.xyz_mean.reshape(1,1,1,3).expand(4,5,128,3).clone()
            warm_mask=torch.ones((4,5,128),dtype=torch.bool,device=device)
            warm[...,0]+=torch.arange(128,device=device)*.06
            self.model(warm,warm_mask)
            warm.copy_(self.model.xyz_mean)
            self.model(warm,warm_mask)
            torch.cuda.synchronize()
    def __call__(self, xyz_bytes, mask_bytes, count):
        points=torch.frombuffer(bytearray(xyz_bytes),dtype=torch.float32).reshape(count,5,128,3).to(self.device)
        mask=torch.frombuffer(bytearray(mask_bytes),dtype=torch.bool).reshape(count,5,128).to(self.device)
        with torch.inference_mode():
            prediction=self.model(points,mask)
        return prediction.cpu().contiguous().numpy().tobytes()
)UTONIA"; }
}  // namespace object_detector

namespace object_detector {
inline const char* UtoniaBbox::hilbertSource() { return R"HILBERT(
#if defined(__CUDACC__)
#define UTONIA_DEVICE __device__
#else
#define UTONIA_DEVICE
#endif
UTONIA_DEVICE inline unsigned long long utonia_hilbert_index(
    unsigned long long first, unsigned long long second,
    unsigned long long third, int depth) {
  unsigned long long x[3] = {first, second, third};
  for (int bit = depth - 1; bit >= 0; --bit) {
    const unsigned long long q = 1ULL << bit;
    const unsigned long long lower = q - 1;
    for (int dim = 0; dim < 3; ++dim) {
      if (x[dim] & q) x[0] ^= lower;
      else {
        const unsigned long long change = (x[0] ^ x[dim]) & lower;
        x[0] ^= change;
        x[dim] ^= change;
      }
    }
  }
  unsigned long long code = 0;
  for (int bit = depth - 1; bit >= 0; --bit)
    for (int dim = 0; dim < 3; ++dim)
      code = (code << 1) | ((x[dim] >> bit) & 1);
  for (int shift = 1; shift < 64; shift *= 2) code ^= code >> shift;
  return code;
}
UTONIA_DEVICE inline unsigned long long utonia_morton_index(
    unsigned long long first, unsigned long long second,
    unsigned long long third, int depth) {
  unsigned long long code = 0;
  for (int bit = 0; bit < depth; ++bit) {
    code |= ((first >> bit) & 1ULL) << (3 * bit + 2);
    code |= ((second >> bit) & 1ULL) << (3 * bit + 1);
    code |= ((third >> bit) & 1ULL) << (3 * bit);
  }
  return code;
}
#if defined(__CUDACC__)
extern "C" __global__ void fused_hilbert(const void* coords, long long* output,
    int count, int depth, int item_bytes) {
  const int i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i >= count) return;
  unsigned long long x[3];
  for (int dim = 0; dim < 3; ++dim)
    x[dim] = item_bytes == 8 ? ((const long long*)coords)[i * 3 + dim]
                            : ((const int*)coords)[i * 3 + dim];
  output[i] = utonia_hilbert_index(x[0], x[1], x[2], depth);
}
extern "C" __global__ void fused_morton(const void* coords, long long* output,
    int count, int depth, int item_bytes) {
  const int i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i >= count) return;
  unsigned long long x[3];
  for (int dim = 0; dim < 3; ++dim)
    x[dim] = item_bytes == 8 ? ((const long long*)coords)[i * 3 + dim]
                            : ((const int*)coords)[i * 3 + dim];
  output[i] = utonia_morton_index(x[0], x[1], x[2], depth);
}
#endif
)HILBERT"; }
}
