# `fcl::distance` 完整引用链路（FCL 0.7.0, master@e5efcc4）

本文是对 `D:\dev\fcl`（BSD-3-Clause）中 `fcl::distance` 函数的完整调用链与文件级
引用闭包的分析结果，也是 fcl_distance 独立复刻库的移植依据。

---

## 1. 公开入口

`include/fcl/narrowphase/distance.h`：

```cpp
template <typename S>
S distance(const CollisionObject<S>* o1, const CollisionObject<S>* o2,
           const DistanceRequest<S>& request, DistanceResult<S>& result);

template <typename S>
S distance(const CollisionGeometry<S>* o1, const Transform3<S>& tf1,
           const CollisionGeometry<S>* o2, const Transform3<S>& tf2,
           const DistanceRequest<S>& request, DistanceResult<S>& result);
```

另有两个带求解器参数的内部重载（`distance-inl.h`，供上面两个入口与 broadphase 使用）：

```cpp
template <typename NarrowPhaseSolver>
S distance(const CollisionObject<S>*, const CollisionObject<S>*,
           const NarrowPhaseSolver* nsolver, const DistanceRequest<S>&, DistanceResult<S>&);
template <typename NarrowPhaseSolver>
S distance(const CollisionGeometry<S>*, const Transform3<S>&,
           const CollisionGeometry<S>*, const Transform3<S>&,
           const NarrowPhaseSolver* nsolver, const DistanceRequest<S>&, DistanceResult<S>&);
```

关键输入输出结构（`narrowphase/distance_request.h` / `distance_result.h`）：

| `DistanceRequest<S>` 成员 | 默认值 |
|---|---|
| `enable_nearest_points` | `false` |
| `enable_signed_distance` | `false` |
| `rel_err` / `abs_err` | `0.0` / `0.0` |
| `distance_tolerance` | `1e-6` |
| `gjk_solver_type` | **`GST_LIBCCD`** |

`DistanceResult<S>`：`min_distance`（初值 `std::numeric_limits<S>::max()`）、
`nearest_points[2]`（默认**未初始化**）、`o1/o2`、`b1/b2`（`intptr_t`，图元 id 或
`NONE=-1`）、`update(...)` 只在严格更小时更新。
`DistanceRequest::isSatisfied(result)` = `result.min_distance <= 0`，每个分发目标
函数入口都先查它（结果对象跨多次调用复用时的短路）。

## 2. 顶层分发流程（distance-inl.h）

```
distance(o1, o2, request, result)
 ├─ switch(request.gjk_solver_type)
 │    ├─ GST_LIBCCD → 构造 detail::GJKSolver_libccd<S>，solver.distance_tolerance = request.distance_tolerance
 │    └─ GST_INDEP  → 构造 detail::GJKSolver_indep<S>， solver.gjk_tolerance      = request.distance_tolerance
 └─ distance(o1, tf1, o2, tf2, &solver, request, result)
      ├─ getDistanceFunctionLookTable<NarrowPhaseSolver>()   // 函数局部 static 分发表
      │    └─ detail::DistanceFunctionMatrix<NarrowPhaseSolver>  // 构造时填 NODE_COUNT×NODE_COUNT 函数指针
      ├─ 若 (object_type1==OT_GEOM && object_type2==OT_BVH)：查表[node2][node1] 并交换实参
      │  否则：查表[node1][node2]；表项为空 → std::cerr 警告，res = max()
      ├─ res = 表项(o1, tf1, o2, tf2, nsolver, request, result)
      └─ 符号距离回退（见 §6）：
         if (res && result.min_distance < 0 && request.enable_signed_distance) {
             if (solver 是 GJKSolver_libccd 且两者都是 OT_GEOM) return res;   // libccd 已给出真实负距离
             collide(o1, tf1, o2, tf2, nsolver, {enable_contact=true}, cres); // 进入碰撞链路!
             取穿透最深的 contact： result.min_distance = -max_pen_depth;
             enable_nearest_points 时 nearest_points[0]=[1]=contact.pos;
         }
```

`NODE_TYPE` 枚举顺序（分发表下标，`geometry/collision_geometry.h`）：
`BV_UNKNOWN, BV_AABB, BV_OBB, BV_RSS, BV_kIOS, BV_OBBRSS, BV_KDOP16, BV_KDOP18,
BV_KDOP24, GEOM_BOX, GEOM_SPHERE, GEOM_ELLIPSOID, GEOM_CAPSULE, GEOM_CONE,
GEOM_CYLINDER, GEOM_CONVEX, GEOM_PLANE, GEOM_HALFSPACE, GEOM_TRIANGLE,
GEOM_OCTREE, NODE_COUNT`。

## 3. 分发矩阵注册全表（distance_func_matrix-inl.h，192 项）

- **GEOM×GEOM**：9 种基本形状（Box, Sphere, Ellipsoid, Capsule, Cone, Cylinder,
  Convex, Plane, Halfspace）两两注册 `ShapeShapeDistance<S1,S2,Solver>`。
  例外：`[GEOM_HALFSPACE][GEOM_ELLIPSOID]` **没有注册**（上游如此）。
- **BVH×GEOM**：`BV ∈ {AABB, OBB, RSS, KDOP16, KDOP18, KDOP24, kIOS, OBBRSS}` ×
  9 形状 → `BVHShapeDistancer<BV,Shape,Solver>::distance`。
  其中 RSS/kIOS/OBBRSS 有**有向特化**（不拷贝模型），其余 BV 走通用版
  （深拷贝 BVHModel、把顶点变换进世界系再重建树）。
- **GEOM×BVH**：不注册；顶层用 swap 规则转到 BVH×GEOM。
- **BVH×BVH**：只有同型 4 项：`[BV_AABB][BV_AABB]`、`[BV_RSS][BV_RSS]`、
  `[BV_kIOS][BV_kIOS]`、`[BV_OBBRSS][BV_OBBRSS]` → `BVHDistance<BV>`。
- **OcTree 相关**（`#if FCL_HAVE_OCTOMAP`）：GEOM_OCTREE×9 形状（双向）、
  OCTREE×OCTREE、OCTREE×BVH（双向）→ OcTreeSolver 系列（本复刻按无 octomap
  构建裁剪，行为与上游 `FCL_HAVE_OCTOMAP=0` 一致）。

## 4. 各分支调用链

### 4.1 GEOM×GEOM

```
ShapeShapeDistance<Shape1,Shape2,Solver>
 └─ detail::ShapeDistanceTraversalNode + initialize(...)
    └─ detail::distance(node)                        // collision_node.h 驱动
       └─ distanceRecurse(node,0,0)                  // 双叶 → 直接 leafTesting
          └─ request.enable_signed_distance
             ? nsolver->shapeSignedDistance(s1,tf1,s2,tf2,&d,&p1,&p2)
             : nsolver->shapeDistance(...)
```

**GJKSolver_libccd 路径**（`gjk_solver_libccd-inl.h` + `convexity_based_algorithm/gjk_libccd-inl.h`）：

- `shapeDistance` 解析特化对（8 个）：Sphere-Box/Capsule/Cylinder/Sphere、
  Capsule-Capsule（`primitive_shape_algorithm/` 中的解析函数）；
  其余走 `GJKDistance` → `GJKDistanceImpl(..., ccdGJKDist2)`：
  - `ccdGJKDist2` = FCL 自研 `__ccdGJK`（标准 GJK，doSimplex2/3/4）判交；
    相交 → 返回 -1；分离 → `_ccdDist`（支撑映射迭代收缩单纯形，
    `ccdVec3PointSegmentDist2` / `ccdVec3PointTriDist2WithWitness` 求最近点，
    `extractClosestPoints` 提取双物体见证点）。
  - 支撑函数经 `GJKInitializer<S,Shape>` 特征类：`createGJKObject`（构造
    `ccd_box_t` 等含位姿的对象）+ `supportBox/Cap/Cyl/Cone/Sphere/Ellipsoid/Convex`。
- `shapeSignedDistance` **没有任何特化对**，全部走 `GJKSignedDistance` →
  `ccdGJKSignedDist`：`__ccdGJK` 相交时 → `ccdPtInit` 建多面体 →
  `__ccdEPA`（FCL 自研 EPA：simplexToPolytope2/4、convert2SimplexToTetrahedron、
  expandPolytope + ComputeVisiblePatch、`validateNearestFeatureOfPolytopeBeingEdge`
  修正）→ `depth = -sqrt(nearest->dist)`，`penEPAPosClosest` 提取见证点。
- 外部 libccd 提供的部分（复刻为 ccd_lite）：`ccd_vec3_t/ccd_quat_t/ccd_t` 类型、
  vec3/quat 内联运算、`ccd_vec3_origin`、`ccd_points_on_sphere`、
  `__ccdSupport`、`ccdPtInit/Destroy/AddVertex/AddEdge/AddFace/Nearest`、
  `ccdVec3PointTriDist2(WithWitness)`、`ccdVec3PointSegmentDist2`、
  `ccdMPRIntersect/ccdMPRPenetration`（后两个只被碰撞链用）。

**GJKSolver_indep 路径**（`gjk_solver_indep-inl.h` + `gjk.h/epa.h/minkowski_diff.h`）：

- `shapeDistance` 特化对与 libccd 完全相同（同 8 个解析函数）；其余走
  `detail::GJK<S>`：`MinkowskiDiff<S>` 支撑（`toshape1`/`toshape0` 双坐标系），
  `Project<S>::projectLine/Triangle/TetrahedraOrigin`（`math/detail/project.h`）
  投影收缩；`Valid` → `distance = ray.norm()` 及见证点；`Inside/Failed` →
  `*distance = -1, return false`。
- `shapeSignedDistance` 是 TODO 桩：直接转发 `shapeDistance`（穿透时只得 -1），
  于是顶层 `min_distance<0` 触发 §6 的 collide() 回退。
- `shapeIntersect` 需要穿透深度时用 `EPA<S>`（`epa-inl.h`，注意 depth 取负号）。

### 4.2 BVH×GEOM（网格 × 基本形状）

```
BVHShapeDistancer<RSS|kIOS|OBBRSS, Shape, Solver>::distance   // 有向特化
 └─ MeshShapeDistanceTraversalNode{RSS,kIOS,OBBRSS} + setupMeshShapeDistanceOrientedNode
    ├─ computeBV(shape, tf2, node.model2_bv)                  // shape/utility.h 特化表
    ├─ preprocess: distancePreprocessOrientedNode → 0 号三角形 vs shape 播种 min_distance
    ├─ BVTesting: fcl::distance(tf1.linear(), tf1.translation(), model2_bv, mesh_bv)
    │             // math/bv/RSS-inl.h rectDistance（~700行）| kIOS | OBBRSS→RSS
    ├─ leafTesting: meshShapeDistanceOrientedNodeLeafTesting
    │   └─ nsolver->shapeTriangleDistance(shape, tf2, p1,p2,p3, tf1, &d, &cp2, &cp1)
    │        // Sphere 有解析特化 sphereTriangleDistance；其余走各自 GJK
    └─ postprocess: 空（求解器直接给世界系点）
```

通用 BV（AABB/OBB/KDOP）版本 `BVHShapeDistancer` 主模板：深拷贝 BVHModel →
`initialize` 把 tf1 烘焙进顶点、`endReplaceModel(false,false)` 全量重建树 →
`BVTesting` 用 `bv.distance(model2_bv)`（OBB/KDOP 的 `distance()` 是
"not implemented" 桩，打印 cerr 并返回 0 → 无剪枝但结果仍正确）。

遍历骨架：`traversal_recurse-inl.h::distanceRecurse`（先序 DFS，先算两个孩子的
BVTesting，先访问更近者，第二个孩子在第一个递归完成后重新 canStop 判断；
`canStop(c)` = `(c >= min_distance - abs_err) && (c*(1+rel_err) >= min_distance)`，
而 rel_err/abs_err 实际恒 0）。`qsize>2` 时有 BFS 优先队列版
`distanceQueueRecurse`（distance() 默认 qsize=2，走 DFS）。

### 4.3 BVH×BVH（网格 × 网格）

```
BVHDistance<RSS|kIOS|OBBRSS>                                   // 有向特化
 └─ MeshDistanceTraversalNode{RSS,kIOS,OBBRSS} + setupMeshDistanceOrientedNode
    ├─ node.tf = tf1.inverse(Isometry) * tf2                   // 相对位姿，不拷模型
    ├─ preprocess: 三角形0 vs 三角形0 播种
    ├─ BVTesting: fcl::distance(tf.linear(), tf.translation(), bv1, bv2)
    └─ leafTesting: TriangleDistance<S>::triDistance(t11..t13, t21..t23, tf, P1, P2)
       // primitive_shape_algorithm/triangle_distance-inl.h（PQP 风格 segPoints）
       // 返回值 >= 0；两三角形相交时精确返回 0（不会是负）
    └─ postprocess: enable_nearest_points 时把局部系 P/Q 用 tf1 变回世界系
AABB 版（BVHDistanceImpl 主模板）：深拷贝两模型 + 世界系重建 + 无向遍历
```

BVHModel 构建链（用户建网格时）：`beginModel/addSubModel/endModel` →
`buildTree` → `BVFitter<BV>::fit`（`getCovariance` → `eigen_old`（Jacobi）→
`axisFromEigen` → `getExtentAndCenter` / `getRadiusAndOriginAndRectangleSize`）
+ `BVSplitter<BV>`（默认 SPLIT_METHOD_MEAN）→ `recursiveBuildTree`。

### 4.4 符号距离回退 → 碰撞链（可达性结论）

| 组合 | 回退可达？ | 原因 |
|---|---|---|
| GEOM×GEOM + libccd | 否 | EPA 已给真实负距离，顶层显式 early-return |
| GEOM×GEOM + indep | **是** | shapeSignedDistance 桩返回 -1 |
| BVH×GEOM（双向）| **是**（两种 solver）| shapeTriangleDistance 穿透返回 -1 |
| BVH×BVH | 否 | triDistance ≥ 0，min_distance 不会 < 0 |

但注意：`collide()` 一旦被调用，`CollisionFunctionMatrix` 构造函数会对**全部**
注册项取地址（odr-use），从而实例化整个碰撞链（含网格-网格碰撞
`MeshCollisionTraversalNode` + `Intersect<S>` 三角形相交内核 + `boxBox2` 等）——
所以复刻必须移植完整碰撞闭包，缺一个文件都无法编译。

碰撞链结构（`collision-inl.h` + `collision_func_matrix-inl.h`）与距离链同构：
GEOM×GEOM 81 项 `ShapeShapeCollide`（叶子 `nsolver->shapeIntersect`，特化解析对
+ 通用 GJK+EPA / MPR）、BVH×GEOM `BVHShapeCollider`（OBB/RSS/kIOS/OBBRSS 有向
特化，叶子 `shapeTriangleIntersect`）、BVH×BVH 8 项（叶子
`Intersect<S>::intersect_Triangle`）。

## 5. 文件级引用闭包（复刻清单）

按目录（`.h` 均含配套 `-inl.h`；★ = 上游死代码但按规约保留）：

```
fcl/config.h, fcl/export.h                       [静态化生成物]
fcl/common/    types.h(接 eigen_lite), unused.h, warning.h
fcl/math/      constants.h, triangle.h, geometry.h
fcl/math/detail/   project.h, polysolver.h
fcl/math/bv/   AABB, OBB, RSS, OBBRSS, kIOS, kDOP, utility
fcl/math/      variance3        [仅被 BVH_utility 引用]
fcl/geometry/  collision_geometry
fcl/geometry/shape/  shape_base, box, sphere, ellipsoid, capsule, cone,
                     cylinder, convex, halfspace, plane, triangle_p,
                     representation, utility(computeBV/constructBox)
fcl/geometry/bvh/    BVH_internal, BVH_model, BVH_utility, BV_node, BV_node_base
fcl/geometry/bvh/detail/  BV_fitter(+base), BV_splitter(+base), BVH_front
fcl/narrowphase/     gjk_solver_type, collision_object,
                     distance_request/result, collision_request/result,
                     contact, contact_point, cost_source,
                     distance(+inl), collision(+inl)
fcl/narrowphase/detail/  distance_func_matrix, collision_func_matrix,
                     gjk_solver_indep, gjk_solver_libccd,
                     failed_at_this_configuration
fcl/narrowphase/detail/convexity_based_algorithm/
                     gjk, epa, minkowski_diff, gjk_libccd,
                     alloc, list, simplex, support, polytope   [libccd 私有头的上游内置副本]
fcl/narrowphase/detail/primitive_shape_algorithm/
                     box_box, capsule_capsule, sphere_box, sphere_capsule,
                     sphere_cylinder, sphere_sphere, sphere_triangle,
                     halfspace, plane, triangle_distance, intersect
fcl/narrowphase/detail/traversal/
                     traversal_node_base, traversal_recurse, collision_node
fcl/narrowphase/detail/traversal/distance/
                     distance_traversal_node_base, shape_distance_traversal_node,
                     bvh_distance_traversal_node, mesh_distance_traversal_node,
                     bvh_shape_distance_traversal_node, mesh_shape_distance_traversal_node,
                     shape_bvh_distance_traversal_node★, shape_mesh_distance_traversal_node★
fcl/narrowphase/detail/traversal/collision/
                     collision_traversal_node_base, shape_collision_traversal_node,
                     bvh_collision_traversal_node, mesh_collision_traversal_node,
                     bvh_shape_collision_traversal_node, mesh_shape_collision_traversal_node,
                     shape_bvh_collision_traversal_node★, shape_mesh_collision_traversal_node★,
                     mesh_continuous_collision_traversal_node,  [被 collision_func_matrix 无条件 include]
                     intersect                                   [三角形相交内核]
```

共 177 个头文件（含 `-inl.h`），其中 174 个来自上游，3 个为本提取新增
（`config.h`、`export.h` 静态替代 CMake 生成物；`fcl.h` 伞头）。

**裁剪**（不在 distance 运行时闭包内）：broadphase 全部、continuous collision /
conservative advancement 全部、`math/motion`、`math/sampler`、`math/rng`、octree
（octomap）、`common/{profiler,time,exception}`。
`distance_func_matrix-inl.h` 对 4 个 conservative advancement 头的 include 为
无用引用，移植中删除。

## 6. 外部依赖边界

本提取**保留上游 FCL 的全部外部依赖**，不做任何替换：

| 依赖 | 用途 | 宿主工程需提供 |
|---|---|---|
| **Eigen 3** | 唯一的数学层：`common/types.h` 中 `Vector3/Matrix3/Transform3/Quaternion/AngleAxis/Translation3/VectorN` 全部是 Eigen 别名；代码大量使用 `linear()/translation()/col()/noalias()/array()/asDiagonal()/逗号初始化/SelfAdjointEigenSolver/unitOrthogonal` | 头文件路径 |
| **libccd** | `GST_LIBCCD`（FCL 默认求解器）的底座：`ccd_vec3_t/ccd_quat_t/ccd_t` 类型与内联运算，以及 `polytope.c / support.c / vec3.c / mpr.c` 中的 `__ccdSupport`、`ccdPt{Init,Destroy,AddVertex,AddEdge,AddFace,Nearest}`、`ccdVec3PointTriDist2`、`ccdVec3PointSegmentDist2`、`ccdMPR{Intersect,Penetration}` | `-lccd` 链接 |
| **octomap** | OcTree 几何（可选） | 不需要——按 `FCL_HAVE_OCTOMAP=0` 裁剪，与上游 `-DFCL_WITH_OCTOMAP=OFF` 构建行为一致 |

注：`convexity_based_algorithm/{alloc,list,simplex,support,polytope}.h` 是**上游 FCL
自带的 libccd 私有头副本**（不是本提取新增），原样保留，仍 `#include <ccd/*.h>`。

## 7. 保真声明

- 除下述四类改动外，与上游**逐字节相同**（由 `tools/verify_against_upstream.py`
  逐文件 diff 分类验证，任何未分类差异都会报错）：
  1. 每个文件开头一行来源注释；
  2. 删除 `extern template` 声明——header-only 构建下没有 libfcl 提供显式实例化，
     保留它们会导致链接错误；
  3. 6 处从 `src/*.cpp` 内联进头文件的非模板定义（`Triangle`、`BVNodeBase`、
     `BVHFrontNode`/`updateFrontList`、`ThrowFailedAtThisConfiguration`、
     `halfspaceIntersectTolerance`、`planeIntersectTolerance`）；
  4. 删除 5 处未被使用的 include（`collision_geometry.h` 里的 `motion_base.h`、
     `distance_func_matrix-inl.h` 里的 4 个 conservative advancement 头）。
- 因此**上游的已知 bug 与未初始化行为一并保留**，例如：
  `distance-inl.h` 回退分支误用 `std::numeric_limits<S>::min()`（应为 `lowest()`）、
  `RSS::operator+` 中 `bv.axis.col(2)` 取自 `this->axis`、
  `kIOS` 自由函数 `translate` 丢弃 obb 平移、
  `MeshDistanceTraversalNode` 从默认构造的 request 读 `rel_err/abs_err`（恒 0）、
  `boxBoxIntersect` 对 `*contacts_` 是赋值而非追加、
  `sphereTriangleDistance(dist,p1,p2)` 穿透时不写 `*dist`，
  以及 `OBB()/RSS()/kIOS/Triangle()/Contact()/DistanceResult::nearest_points` 等
  未初始化成员。数值结果因此与上游 FCL 0.7.0 一致。
- `fcl/config.h`、`fcl/export.h` 为 CMake 生成物，用静态版本替代
  （`FCL_HAVE_OCTOMAP=0`、`FCL_HAVE_SSE=0`、`FCL_ENABLE_PROFILING=0`，
  均可由外部 `-D` 覆盖）。

