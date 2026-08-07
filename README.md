# fcl_distance

**FCL 0.7.0 中 mesh × mesh 距离计算的 header-only 独立提取（no-obbrss-refit 分支）。**

> 本分支在 `main` 基础上进一步移除了 `OBBRSS` 包围体与 refit/update 机制，
> 只保留 `AABB / RSS / kIOS` 三种包围体和一次性建树。

从 [FCL (Flexible Collision Library)](https://github.com/flexible-collision-library/fcl)
0.7.0（BSD-3-Clause，master@e5efcc4）中抽出 `fcl::distance` 在**网格对网格**这一
场景下的完整引用闭包，做成纯头文件：源码拷到别处，`#include` 即可用，
**不需要编译 FCL、不需要 CMake 配置步骤**。

`fcl::distance` 的调用接口与上游完全一致。

## 依赖

**只有 Eigen 3，且只需要头文件路径。**

上游 FCL 还依赖 libccd，那是 `GST_LIBCCD` 求解器的底座。mesh × mesh 距离
根本不走求解器——`BVHDistance()` 拿到 solver 指针后第一行就是 `FCL_UNUSED(nsolver)`，
叶子测试直接调用解析的 `TriangleDistance<S>::triDistance`。所以 GJK / EPA / libccd
整块都不在本提取里，**不需要 `-lccd`**。

## 使用

```cpp
#include "fcl/narrowphase/distance.h"
#include "fcl/geometry/bvh/BVH_model.h"

// 建网格（RSS 是本分支上 mesh 距离的推荐包围体）
fcl::BVHModel<fcl::RSS<double>> m1, m2;
m1.beginModel();
m1.addSubModel(vertices1, triangles1);   // std::vector<fcl::Vector3d>, std::vector<fcl::Triangle>
m1.endModel();
m1.computeLocalAABB();
// m2 同理

fcl::Transform3d tf1 = fcl::Transform3d::Identity();
fcl::Transform3d tf2 = fcl::Transform3d::Identity();
tf2.translation() = fcl::Vector3d(3, 0, 0);

fcl::DistanceRequest<double> request;
request.enable_nearest_points = true;
fcl::DistanceResult<double> result;

double d = fcl::distance(&m1, tf1, &m2, tf2, request, result);
// d = 两网格间最小距离；result.nearest_points[0/1] 为各自表面上的最近点
// result.b1 / result.b2 为对应的三角形索引
```

编译：

```bash
g++ -std=c++11 -O2 -Ifcl_distance/include -I/usr/include/eigen3 your.cpp
```

## 覆盖范围

- `fcl::distance` 的两组入口：`CollisionGeometry*` 与 `CollisionObject*`
- 网格 `BVHModel<BV>`，包围体支持 `AABB / RSS / kIOS`
  （上游为 mesh × mesh 注册 4 种，本分支移除了 `OBBRSS`；`KDOP` 上游未注册）
- 最近点输出（`enable_nearest_points`）与最近三角形索引（`result.b1/b2`）
- BVH 的构建、拟合（`BVFitter`）、划分（`BVSplitter`）
- 顶点替换（`beginReplaceModel`/`replaceSubModel`/`endReplaceModel`）——
  距离路径内部要靠它把位姿烘焙进顶点

**不含**：形状（Box/Sphere/…）、mesh × 形状、GJK/EPA 求解器、碰撞检测
（`fcl::collide`）、broadphase、连续碰撞、octree、`OBBRSS`、refit/update。

两点行为差异，都是 mesh × mesh 场景下的必然结果：

- `DistanceRequest::gjk_solver_type` 无效——mesh × mesh 本来就不用求解器
- `DistanceRequest::enable_signed_distance` 无效——上游用 `collide()` 回退来算
  负距离，但 `triDistance` 永不返回负值（重叠时精确返回 0），该回退在 mesh × mesh
  下是死代码。重叠网格返回 0，与上游默认行为一致。

## 验证

在 Ubuntu 24.04 / gcc 13.3 / Eigen 3.4.0 上实测：

| 检查 | 结果 |
|---|---|
| 编译 + 链接（无 `-lccd`） | **通过**，5 秒 |
| `tests/test_mesh_distance.cpp` 黄金值 | **46 项检查，0 失败** |
| 三种包围体结果互相一致 | **通过**（AABB/RSS/kIOS 同值） |
| 与上游 libfcl 0.7.0 数值对拍 | **440 组查询逐字节完全一致** |

对拍方式：`tools/crosscheck_dump.cpp` 同一份源码分别链接系统 libfcl 0.7.0 和本
提取，跑 200 组随机位姿（立方体网格 12 面 + 环形网格 128 面，分离与穿透各半），
输出距离和最近点，`cmp` 逐字节比较。

```bash
g++ -std=c++14 -O2 -DCROSSCHECK_UPSTREAM tools/crosscheck_dump.cpp -o up -lfcl -lccd && ./up > up.txt
g++ -std=c++14 -O2 -Iinclude tools/crosscheck_dump.cpp -o pt && ./pt > pt.txt
cmp up.txt pt.txt && echo IDENTICAL
```

## 与上游代码的关系

56 个头文件中，53 个来自上游 FCL，逐字未改（除每个文件开头一行来源注释、
删除 `extern template` 声明、以及 6 处从 `src/*.cpp` 内联进头文件的非模板定义）。

有 4 个文件为服务本场景做了**删减**，每个文件顶部都写明了删了什么、为什么：

| 文件 | 改动 |
|---|---|
| `narrowphase/detail/distance_func_matrix-inl.h` | 上游注册 192 个分发项；本分支只保留 mesh × mesh 的 3 项（`BV_AABB/BV_RSS/BV_kIOS` 对角线），其余整函数删除 |
| `narrowphase/distance-inl.h` | 删掉 `collide()` 符号距离回退（mesh × mesh 下是死代码）；双求解器分支合并为 `detail::MeshDistanceSolver` 占位类型 |
| `narrowphase/detail/traversal/collision_node.h/-inl.h` | 只保留 `distance(node)` 驱动，删掉 `collide` / `selfCollide` / `collide2` |
| `narrowphase/detail/traversal/traversal_recurse.h/-inl.h` | 只保留 `distanceRecurse` / `distanceQueueRecurse` 及其 `BVT/BVTQ` 辅助结构 |

另删除 `math/bv/kDOP.h/-inl.h`（563 行）及 `BVH_model-inl.h` 中 3 个
`GetNodeTypeImpl<KDOP<S,N>>` 特化：`KDOP` 从未被上游注册为 mesh × mesh 的包围体，
`BVHModel<KDOP<...>>` 在本提取中无法用于距离计算。

本分支相对 `main` 的额外删减：

| 内容 | 说明 |
|---|---|
| `math/bv/OBBRSS.h/-inl.h` 及全部 `OBBRSS` 特化 | 分发矩阵注册项、`BVHDistanceImpl`、`MeshDistanceTraversalNodeOBBRSS`、`BVFitter`/`BVSplitter`/`BVNode`/`BVHModel` 中的特化、`OBBRSS_fit_functions` |
| `BVHModel` 的 refit 族 | `refitTree` / `refitTree_topdown` / `refitTree_bottomup` / `recursiveRefitTree_bottomup` |
| `BVHModel` 的 update 族 | `beginUpdateModel` / `updateVertex` / `updateTriangle` / `updateSubModel` / `endUpdateModel`、`prev_vertices` 成员 |
| `math/bv/utility.h/-inl.h` | 只服务 refit 的 `fit<BV>()` 与全部 `*_fit_functions` |
| `math/geometry` 的 7 个自由函数 | `generateCoordinateSystem`、`circumCircleComputation`、`relativeTransform`、`triple`、`hat`、`normalize`、`combine` |
| `math/bv/OBB` 的 4 个自由函数 | `obbDisjoint`、`computeVertices`、`merge_largedist`、`merge_smalldist` |
| `BVHModel::makeParentRelative` 及其 `MakeParentRelativeRecurseImpl` 特化 | 把包围体转成相对父节点的坐标，只服务有向 BV 的碰撞遍历 |
| 包围体的碰撞查询面 | `AABB/RSS/kIOS/OBB` 的 `overlap`（成员与自由函数）、`contain(BV)`、`AABB::axisOverlap`、`AABB::expand`，以及 `BVHModel::memUsage` |

其中 `math/geometry` 与 `math/bv/OBB` 两项在 `main` 上是**活代码**——它们只在 refit
路径上被调用。refit 移除后才成为死代码。包围体的碰撞查询面则是随更早的碰撞链路
移除而失去调用方的。所有删除都逐项做了「删掉重编 + 输出逐字节比对」验证。

保留的公开 API：`addVertex` / `addTriangle` / `addSubModel(点云)` /
`replaceVertex` / `replaceTriangle` 在本测试里未被调用，探测显示"可删"，但它们是
用户建网格的正常入口，故保留。`contain(const Vector3&)`（点包含）被 `BVSplitter`
使用，也保留。

`endReplaceModel()` 失去了 `refit` / `bottomup` 两个参数，恒定重建树。这不是可选
简化：通用包围体（AABB）的距离路径在 `initialize()` 里就是用
`beginReplaceModel`/`replaceSubModel`/`endReplaceModel(use_refit=false)` 把位姿
烘焙进顶点的，本来走的就是重建分支。

保留的代码里，**上游已知的 bug 与未初始化行为一律未改**（如 `RSS::operator+` 中
`bv.axis.col(2)` 取自 `this->axis`、`kIOS::encloseSphere` 硬编码 `float`、
`MeshDistanceTraversalNode` 从默认构造的 request 读 `rel_err/abs_err`（恒 0）、
`RSS()/kIOS/Triangle()` 等未初始化成员），因此数值结果与上游一致——
440 组对拍逐字节相同即是证明。

`fcl/config.h` 与 `fcl/export.h` 是 CMake 生成物，本仓库用静态版本替代。

## 目录

```
include/fcl/**            头文件树，56 个（含 -inl.h）
docs/REFERENCE_CHAIN.md   fcl::distance 引用链路分析
tests/test_mesh_distance.cpp     黄金值测试
tools/port_from_upstream.py      从上游重新生成本树
tools/verify_against_upstream.py 逐文件 diff 分类校验
tools/crosscheck_dump.cpp        与原版 FCL 数值对拍采样器
tools/crosscheck_compare.py      对拍结果比对
```

## 许可

BSD-3-Clause，与上游 FCL 相同；每个文件保留原始版权声明，根目录 `LICENSE`
为上游许可全文。本仓库是独立提取，非 FCL 官方项目。
