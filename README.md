# voxgrid

Pure Dart hierarchical 3D spatial indexing library for game engines and physics broad-phases.

## Features

- **Dynamic Bounding Volume Hierarchy (`DynamicBvh<T>`)**: Surface-Area Heuristic (SAH) insertion, fattened proxy AABBs with velocity prediction, tree rotations, incremental updates, broad-phase overlap pair discovery, and branch-and-bound closest-hit ray traversal.
- **Loose Octree (`LooseOctree<T>`)**: Configurable looseness factor (default `2.0`), depth-bounded spatial partitioning where each object resides cleanly in a single node without cross-boundary duplication.
- **Zero-Dependency Geometry (`GridVec3`, `GridAabb`, `GridRay`, `GridFrustum`)**: Slab ray-AABB intersection, sphere containment/overlap, and 6-plane view frustum culling.
