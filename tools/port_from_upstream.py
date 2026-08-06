#!/usr/bin/env python3
"""Mechanically port FCL 0.7.0 headers into the fcl_distance tree.

fcl_distance is a header-only extraction of the fcl::distance reference
closure.  It keeps FCL's dependencies (Eigen, libccd) — the host project
already provides them — so the only transformations applied are:

  1. drop `extern template ...;` declarations (they promise definitions that
     used to live in libfcl.a; header-only builds must instantiate locally)
  2. drop #includes of subsystems outside the distance closure
  3. insert a one-line provenance marker before the include guard

Everything else is byte-identical to upstream.

    python tools/port_from_upstream.py --all
    python tools/port_from_upstream.py math/bv/OBB-inl.h [...]

Paths are relative to include/fcl/.
"""
import os
import re
import sys

UP = r"D:\dev\fcl\include\fcl"
PORT = r"D:\dev\fcl_distance\include\fcl"

# Includes of subsystems this extraction does not carry (see docs/REFERENCE_CHAIN.md).
DROPPED_INCLUDE_RE = re.compile(
    r'^\s*#\s*include\s+"fcl/('
    r'broadphase/|math/motion/|math/sampler/|math/rng|'
    r'common/profiler|common/time|'
    r'narrowphase/continuous_collision|'
    r'narrowphase/detail/conservative_advancement_func_matrix|'
    r'narrowphase/detail/traversal/distance/[a-z_]*conservative_advancement'
    r')')

EXTERN_TEMPLATE_START = re.compile(r'^\s*extern template\b')


def strip_statement(lines, i):
    """If lines[i] starts an `extern template` declaration, return the index
    just past its terminating ';' line; else None."""
    if not EXTERN_TEMPLATE_START.match(lines[i]):
        return None
    j = i
    while j < len(lines):
        if lines[j].rstrip().endswith(";"):
            return j + 1
        j += 1
    return j


def port_one(rel):
    src = os.path.join(UP, rel.replace("/", os.sep))
    dst = os.path.join(PORT, rel.replace("/", os.sep))
    if not os.path.isfile(src):
        raise SystemExit("no upstream file: " + src)
    with open(src, encoding="utf-8") as f:
        lines = f.read().splitlines()

    out = []
    stats = {"extern_template": 0, "dropped_include": 0}
    i = 0
    n = len(lines)
    marker_done = False
    while i < n:
        line = lines[i]

        j = strip_statement(lines, i)
        if j is not None:
            stats["extern_template"] += 1
            if j < n and lines[j].strip() == "":
                j += 1
            i = j
            continue

        if DROPPED_INCLUDE_RE.match(line):
            stats["dropped_include"] += 1
            i += 1
            continue

        if not marker_done and re.match(r'^\s*#\s*ifndef\s+\w+', line):
            out.append("// fcl_distance: header-only extraction of FCL 0.7.0 "
                       "include/fcl/%s" % rel)
            out.append("")
            marker_done = True

        out.append(line)
        i += 1

    os.makedirs(os.path.dirname(dst), exist_ok=True)
    with open(dst, "w", encoding="utf-8", newline="\n") as f:
        f.write("\n".join(out) + "\n")

    active = {k: v for k, v in stats.items() if v}
    print("  %-64s %5d lines  %s" % (rel, len(out), active if active else ""))
    return stats


ALL = """
common/types.h common/unused.h common/warning.h

math/constants.h math/triangle.h
math/geometry.h math/geometry-inl.h
math/variance3.h math/variance3-inl.h
math/detail/project.h math/detail/project-inl.h
math/detail/polysolver.h math/detail/polysolver-inl.h
math/bv/AABB.h math/bv/AABB-inl.h
math/bv/OBB.h math/bv/OBB-inl.h
math/bv/RSS.h math/bv/RSS-inl.h
math/bv/OBBRSS.h math/bv/OBBRSS-inl.h
math/bv/kIOS.h math/bv/kIOS-inl.h
math/bv/kDOP.h math/bv/kDOP-inl.h
math/bv/utility.h math/bv/utility-inl.h

geometry/collision_geometry.h geometry/collision_geometry-inl.h
geometry/shape/shape_base.h geometry/shape/shape_base-inl.h
geometry/shape/box.h geometry/shape/box-inl.h
geometry/shape/sphere.h geometry/shape/sphere-inl.h
geometry/shape/ellipsoid.h geometry/shape/ellipsoid-inl.h
geometry/shape/capsule.h geometry/shape/capsule-inl.h
geometry/shape/cone.h geometry/shape/cone-inl.h
geometry/shape/cylinder.h geometry/shape/cylinder-inl.h
geometry/shape/convex.h geometry/shape/convex-inl.h
geometry/shape/halfspace.h geometry/shape/halfspace-inl.h
geometry/shape/plane.h geometry/shape/plane-inl.h
geometry/shape/triangle_p.h geometry/shape/triangle_p-inl.h
geometry/shape/representation.h
geometry/shape/utility.h geometry/shape/utility-inl.h

geometry/bvh/BVH_internal.h
geometry/bvh/BVH_model.h geometry/bvh/BVH_model-inl.h
geometry/bvh/BVH_utility.h geometry/bvh/BVH_utility-inl.h
geometry/bvh/BV_node.h geometry/bvh/BV_node-inl.h
geometry/bvh/BV_node_base.h
geometry/bvh/detail/BVH_front.h
geometry/bvh/detail/BV_fitter.h geometry/bvh/detail/BV_fitter-inl.h
geometry/bvh/detail/BV_fitter_base.h
geometry/bvh/detail/BV_splitter.h geometry/bvh/detail/BV_splitter-inl.h
geometry/bvh/detail/BV_splitter_base.h

narrowphase/gjk_solver_type.h
narrowphase/collision_object.h narrowphase/collision_object-inl.h
narrowphase/contact.h narrowphase/contact-inl.h
narrowphase/contact_point.h narrowphase/contact_point-inl.h
narrowphase/cost_source.h narrowphase/cost_source-inl.h
narrowphase/collision_request.h narrowphase/collision_request-inl.h
narrowphase/collision_result.h narrowphase/collision_result-inl.h
narrowphase/distance_request.h narrowphase/distance_request-inl.h
narrowphase/distance_result.h narrowphase/distance_result-inl.h
narrowphase/collision.h narrowphase/collision-inl.h
narrowphase/distance.h narrowphase/distance-inl.h

narrowphase/detail/collision_func_matrix.h
narrowphase/detail/collision_func_matrix-inl.h
narrowphase/detail/distance_func_matrix.h
narrowphase/detail/distance_func_matrix-inl.h
narrowphase/detail/failed_at_this_configuration.h
narrowphase/detail/gjk_solver_indep.h narrowphase/detail/gjk_solver_indep-inl.h
narrowphase/detail/gjk_solver_libccd.h narrowphase/detail/gjk_solver_libccd-inl.h

narrowphase/detail/convexity_based_algorithm/alloc.h
narrowphase/detail/convexity_based_algorithm/list.h
narrowphase/detail/convexity_based_algorithm/simplex.h
narrowphase/detail/convexity_based_algorithm/support.h
narrowphase/detail/convexity_based_algorithm/polytope.h
narrowphase/detail/convexity_based_algorithm/gjk.h
narrowphase/detail/convexity_based_algorithm/gjk-inl.h
narrowphase/detail/convexity_based_algorithm/epa.h
narrowphase/detail/convexity_based_algorithm/epa-inl.h
narrowphase/detail/convexity_based_algorithm/minkowski_diff.h
narrowphase/detail/convexity_based_algorithm/minkowski_diff-inl.h
narrowphase/detail/convexity_based_algorithm/gjk_libccd.h
narrowphase/detail/convexity_based_algorithm/gjk_libccd-inl.h

narrowphase/detail/primitive_shape_algorithm/box_box.h
narrowphase/detail/primitive_shape_algorithm/box_box-inl.h
narrowphase/detail/primitive_shape_algorithm/capsule_capsule.h
narrowphase/detail/primitive_shape_algorithm/capsule_capsule-inl.h
narrowphase/detail/primitive_shape_algorithm/halfspace.h
narrowphase/detail/primitive_shape_algorithm/halfspace-inl.h
narrowphase/detail/primitive_shape_algorithm/plane.h
narrowphase/detail/primitive_shape_algorithm/plane-inl.h
narrowphase/detail/primitive_shape_algorithm/sphere_box.h
narrowphase/detail/primitive_shape_algorithm/sphere_box-inl.h
narrowphase/detail/primitive_shape_algorithm/sphere_capsule.h
narrowphase/detail/primitive_shape_algorithm/sphere_capsule-inl.h
narrowphase/detail/primitive_shape_algorithm/sphere_cylinder.h
narrowphase/detail/primitive_shape_algorithm/sphere_cylinder-inl.h
narrowphase/detail/primitive_shape_algorithm/sphere_sphere.h
narrowphase/detail/primitive_shape_algorithm/sphere_sphere-inl.h
narrowphase/detail/primitive_shape_algorithm/sphere_triangle.h
narrowphase/detail/primitive_shape_algorithm/sphere_triangle-inl.h
narrowphase/detail/primitive_shape_algorithm/triangle_distance.h
narrowphase/detail/primitive_shape_algorithm/triangle_distance-inl.h

narrowphase/detail/traversal/traversal_node_base.h
narrowphase/detail/traversal/traversal_node_base-inl.h
narrowphase/detail/traversal/traversal_recurse.h
narrowphase/detail/traversal/traversal_recurse-inl.h
narrowphase/detail/traversal/collision_node.h
narrowphase/detail/traversal/collision_node-inl.h

narrowphase/detail/traversal/distance/distance_traversal_node_base.h
narrowphase/detail/traversal/distance/distance_traversal_node_base-inl.h
narrowphase/detail/traversal/distance/shape_distance_traversal_node.h
narrowphase/detail/traversal/distance/shape_distance_traversal_node-inl.h
narrowphase/detail/traversal/distance/bvh_distance_traversal_node.h
narrowphase/detail/traversal/distance/bvh_distance_traversal_node-inl.h
narrowphase/detail/traversal/distance/mesh_distance_traversal_node.h
narrowphase/detail/traversal/distance/mesh_distance_traversal_node-inl.h
narrowphase/detail/traversal/distance/bvh_shape_distance_traversal_node.h
narrowphase/detail/traversal/distance/bvh_shape_distance_traversal_node-inl.h
narrowphase/detail/traversal/distance/mesh_shape_distance_traversal_node.h
narrowphase/detail/traversal/distance/mesh_shape_distance_traversal_node-inl.h
narrowphase/detail/traversal/distance/shape_bvh_distance_traversal_node.h
narrowphase/detail/traversal/distance/shape_bvh_distance_traversal_node-inl.h
narrowphase/detail/traversal/distance/shape_mesh_distance_traversal_node.h
narrowphase/detail/traversal/distance/shape_mesh_distance_traversal_node-inl.h

narrowphase/detail/traversal/collision/collision_traversal_node_base.h
narrowphase/detail/traversal/collision/collision_traversal_node_base-inl.h
narrowphase/detail/traversal/collision/shape_collision_traversal_node.h
narrowphase/detail/traversal/collision/shape_collision_traversal_node-inl.h
narrowphase/detail/traversal/collision/bvh_collision_traversal_node.h
narrowphase/detail/traversal/collision/bvh_collision_traversal_node-inl.h
narrowphase/detail/traversal/collision/mesh_collision_traversal_node.h
narrowphase/detail/traversal/collision/mesh_collision_traversal_node-inl.h
narrowphase/detail/traversal/collision/bvh_shape_collision_traversal_node.h
narrowphase/detail/traversal/collision/bvh_shape_collision_traversal_node-inl.h
narrowphase/detail/traversal/collision/mesh_shape_collision_traversal_node.h
narrowphase/detail/traversal/collision/mesh_shape_collision_traversal_node-inl.h
narrowphase/detail/traversal/collision/shape_bvh_collision_traversal_node.h
narrowphase/detail/traversal/collision/shape_bvh_collision_traversal_node-inl.h
narrowphase/detail/traversal/collision/shape_mesh_collision_traversal_node.h
narrowphase/detail/traversal/collision/shape_mesh_collision_traversal_node-inl.h
narrowphase/detail/traversal/collision/intersect.h
narrowphase/detail/traversal/collision/intersect-inl.h
narrowphase/detail/traversal/collision/mesh_continuous_collision_traversal_node.h
narrowphase/detail/traversal/collision/mesh_continuous_collision_traversal_node-inl.h
""".split()


def main():
    args = sys.argv[1:]
    targets = ALL if (not args or args[0] == "--all") else args
    total = {}
    for rel in targets:
        s = port_one(rel)
        for k, v in s.items():
            total[k] = total.get(k, 0) + v
    print("\ntotals:", {k: v for k, v in total.items() if v})
    print("ported %d files" % len(targets))


if __name__ == "__main__":
    main()
