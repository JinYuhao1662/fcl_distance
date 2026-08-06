# fcl_distance

**FCL 0.7.0 `fcl::distance` 的 header-only 独立提取。**

从 [FCL (Flexible Collision Library)](https://github.com/flexible-collision-library/fcl)
0.7.0（BSD-3-Clause，master@e5efcc4）中抽出 `fcl::distance` 的完整引用闭包
（含它内部需要的 `fcl::collide` 回退链路），做成**纯头文件**：源码拷到别处，
`#include` 即可用，**不需要编译 FCL、不需要 CMake 配置步骤**。

接口与上游 FCL 完全一致（`fcl::` 命名空间下的类型、函数签名、默认参数、枚举值
逐字保留）。

## 依赖

与上游 FCL 相同，只有两个，需由宿主工程提供：

| 依赖 | 用途 | 编译/链接 |
|---|---|---|
| **Eigen 3** | 数学类型（`fcl::Vector3<S>` 等均是 Eigen 别名） | 仅需头文件路径 |
| **libccd** | `GST_LIBCCD` 求解器（FCL 默认）的底层几何例程 | 需要 `-lccd` 链接 |

不需要 octomap（本提取按 `FCL_HAVE_OCTOMAP=0` 构建，等价于上游
`-DFCL_WITH_OCTOMAP=OFF`）。

## 使用

```cpp
#include "fcl/narrowphase/distance.h"   // 或 #include "fcl/fcl.h"

fcl::Sphere<double> s1(1.0), s2(1.0);
fcl::Transform3d tf1 = fcl::Transform3d::Identity();
fcl::Transform3d tf2 = fcl::Transform3d::Identity();
tf2.translation() = fcl::Vector3d(3, 0, 0);

fcl::DistanceRequest<double> request;
request.enable_nearest_points = true;
fcl::DistanceResult<double> result;

double d = fcl::distance(&s1, tf1, &s2, tf2, request, result);
// d == 1.0；result.nearest_points[0/1] 为两物体上的最近点
```

编译：

```bash
g++ -std=c++11 -O2 -Ifcl_distance/include -I/usr/include/eigen3 your.cpp -lccd
```

单头版本（`tools/amalgamate.py` 生成，把整棵头文件树摊平成一个文件，Eigen 与
libccd 仍作为外部 `#include`）：

```cpp
#include "fcl_distance.hpp"
```

## 覆盖范围

- `fcl::distance`：`CollisionObject` 与 `CollisionGeometry` 两组入口
- 9 种基本形状 `Box / Sphere / Ellipsoid / Capsule / Cone / Cylinder /
  Convex / Plane / Halfspace`（+ `TriangleP`），全部两两组合
- 网格 `BVHModel<BV>`：`AABB / OBB / RSS / OBBRSS / kIOS / KDOP<16|18|24>`；
  网格×形状、网格×网格距离
- 两种求解器：`GST_LIBCCD`（默认）与 `GST_INDEP`
- 符号距离（`enable_signed_distance`）及其内部的 `fcl::collide` 回退链路
  ——因此 `fcl::collide` 也完整可用
- 最近点输出（`enable_nearest_points`）

**不含**（不属于 `fcl::distance` 闭包）：broadphase、continuous collision /
conservative advancement、octomap/OcTree、`fcl::common::{Profiler,Time}`。

## 与上游的差异

`tools/verify_against_upstream.py` 会逐文件 diff 并给每个差异分类，运行结果：

```
shipped headers        : 177
change categories:
  provenance marker                   174 hunks   每个文件开头一行来源注释
  extern template removed             245 hunks   header-only 必需（原本由 libfcl 提供实例化）
  definition inlined from .cpp          6 hunks   见下
  pruned-subsystem include removed      5 hunks   motion_base.h ×1、conservative advancement ×4（均为未使用的 include）
OK: every difference from upstream falls into an expected category.
```

即：**除上述四类之外，与上游逐字节相同**——包括上游已知的 bug 与未初始化行为，
一律保留，以保证数值结果一致。

从 `src/*.cpp` 内联进头文件的 6 处非模板定义（原本编进 libfcl）：

| 头文件 | 来自 |
|---|---|
| `math/triangle.h` | `src/math/triangle.cpp`（`Triangle` 的 5 个方法）|
| `geometry/bvh/BV_node_base.h` | `src/geometry/bvh/BV_node_base.cpp`（4 个方法）|
| `geometry/bvh/detail/BVH_front.h` | `src/geometry/bvh/detail/BVH_front.cpp` |
| `narrowphase/detail/failed_at_this_configuration.h` | 同名 `.cpp`（`ThrowFailedAtThisConfiguration`）|
| `narrowphase/detail/primitive_shape_algorithm/halfspace.h` | 同名 `.cpp`（`halfspaceIntersectTolerance` 的两个特化）|
| `narrowphase/detail/primitive_shape_algorithm/plane.h` | 同名 `.cpp`（`planeIntersectTolerance` 的两个特化）|

`fcl/config.h` 与 `fcl/export.h` 是 CMake 生成物，本仓库用静态版本替代。

## 目录

```
include/fcl/**            与上游同路径的头文件树（177 个头，含 -inl.h）
single_include/           单头产物
docs/REFERENCE_CHAIN.md   fcl::distance 完整引用链路分析（调用图 + 文件清单）
tests/                    黄金值测试 + 全实例化编译测试
tools/port_from_upstream.py      从上游重新生成本树
tools/verify_against_upstream.py 逐文件 diff 分类校验
tools/amalgamate.py              生成单头
tools/crosscheck_dump.cpp        与原版 FCL 数值对拍采样器
tools/crosscheck_compare.py      对拍结果比对
```

## 测试

```bash
g++ -std=c++11 -O2 -Iinclude -I/usr/include/eigen3 tests/test_fcl_distance.cpp -lccd -o test_fcl_distance && ./test_fcl_distance
```

```bash
g++ -std=c++11 -fsyntax-only -Iinclude -I/usr/include/eigen3 tests/instantiate_all.cpp
```

与原版 FCL 数值对拍（需要能同时链接原版 libfcl 的环境）：

```bash
g++ -std=c++11 -O2 -DCROSSCHECK_UPSTREAM -I/usr/include/eigen3 tools/crosscheck_dump.cpp -lfcl -lccd -o dump_upstream && ./dump_upstream > upstream.txt
g++ -std=c++11 -O2 -Iinclude -I/usr/include/eigen3 tools/crosscheck_dump.cpp -lccd -o dump_port && ./dump_port > port.txt
python tools/crosscheck_compare.py upstream.txt port.txt
```

## 许可

BSD-3-Clause，与上游 FCL 相同；每个文件保留原始版权声明。
本仓库是独立提取，非 FCL 官方项目。
