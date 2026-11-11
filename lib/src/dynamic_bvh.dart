import 'dart:math' as math;

import 'geometry.dart';

const int _nullNode = -1;

final class _BvhNode<T> {
  GridAabb aabb = const GridAabb(GridVec3.zero, GridVec3.zero);
  GridAabb? tightBounds;
  T? item;
  int parentOrNext = _nullNode;
  int child1 = _nullNode;
  int child2 = _nullNode;
  int height = -1;

  bool get isLeaf => child1 == _nullNode;
}

/// Dynamic Surface-Area Heuristic (SAH) Bounding Volume Hierarchy with fattened
/// proxy bounds, velocity look-ahead expansion, incremental rotations, and
/// branch-and-bound closest-hit ray traversal.
final class DynamicBvh<T> {
  final double fatMargin;
  final double displacementMultiplier;

  final List<_BvhNode<T>> _nodes = [];
  final Map<T, int> _itemToProxy = {};

  int _root = _nullNode;
  int _freeList = _nullNode;
  int _count = 0;
  int _nodeCount = 0;
  int _reinsertionCount = 0;
  int _moveCallCount = 0;

  DynamicBvh({
    this.fatMargin = 0.15,
    this.displacementMultiplier = 2.0,
  });

  int get count => _count;
  int get nodeCount => _nodeCount;
  int get height => _root == _nullNode ? 0 : _nodes[_root].height;
  int get reinsertionCount => _reinsertionCount;
  int get moveCallCount => _moveCallCount;

  GridAabb? get rootBounds => _root == _nullNode ? null : _nodes[_root].aabb;

  int? proxyOf(T item) => _itemToProxy[item];
  T? itemAt(int proxyId) =>
      proxyId >= 0 && proxyId < _nodes.length ? _nodes[proxyId].item : null;

  GridAabb? boundsOf(int proxyId) =>
      proxyId >= 0 && proxyId < _nodes.length ? _nodes[proxyId].tightBounds : null;

  GridAabb? fatBoundsOf(int proxyId) =>
      proxyId >= 0 && proxyId < _nodes.length && _nodes[proxyId].height >= 0
          ? _nodes[proxyId].aabb
          : null;

  /// Inserts [item] with tight [bounds] and returns its integer proxy handle.
  int insert(T item, GridAabb bounds) {
    final proxyId = _allocateNode();
    final node = _nodes[proxyId];
    node.tightBounds = bounds;
    node.aabb = bounds.expand(fatMargin);
    node.item = item;
    node.height = 0;
    _insertLeaf(proxyId);
    _itemToProxy[item] = proxyId;
    _count++;
    return proxyId;
  }

  /// Moves existing [proxyId] to [newBounds], reinserting only when leaving its
  /// fattened volume (or shrinking excessively). Returns `true` if reinserted.
  bool moveProxy(
    int proxyId,
    GridAabb newBounds, [
    GridVec3 displacement = GridVec3.zero,
  ]) {
    if (proxyId < 0 || proxyId >= _nodes.length) return false;
    final node = _nodes[proxyId];
    if (node.height < 0 || !node.isLeaf) return false;

    _moveCallCount++;
    node.tightBounds = newBounds;

    // Expand fattened AABB in anticipation of velocity.
    var fat = newBounds.expand(fatMargin);
    final dx = displacement.x * displacementMultiplier;
    final dy = displacement.y * displacementMultiplier;
    final dz = displacement.z * displacementMultiplier;
    fat = GridAabb(
      GridVec3(
        dx < 0 ? fat.min.x + dx : fat.min.x,
        dy < 0 ? fat.min.y + dy : fat.min.y,
        dz < 0 ? fat.min.z + dz : fat.min.z,
      ),
      GridVec3(
        dx > 0 ? fat.max.x + dx : fat.max.x,
        dy > 0 ? fat.max.y + dy : fat.max.y,
        dz > 0 ? fat.max.z + dz : fat.max.z,
      ),
    );

    if (node.aabb.contains(newBounds)) {
      // Avoid redundant reinsertion if current fat box still tightly fits (within 4x).
      final huge = newBounds.expand(fatMargin * 4.0);
      if (huge.contains(node.aabb)) {
        return false;
      }
    }

    _removeLeaf(proxyId);
    node.aabb = fat;
    _insertLeaf(proxyId);
    _reinsertionCount++;
    return true;
  }

  /// Updates an item tracked via [insert], or inserts it if absent.
  int upsert(
    T item,
    GridAabb bounds, [
    GridVec3 displacement = GridVec3.zero,
  ]) {
    final existing = _itemToProxy[item];
    if (existing == null) {
      return insert(item, bounds);
    }
    moveProxy(existing, bounds, displacement);
    return existing;
  }

  bool removeItem(T item) {
    final proxyId = _itemToProxy[item];
    if (proxyId == null) return false;
    return removeProxy(proxyId);
  }

  bool removeProxy(int proxyId) {
    if (proxyId < 0 || proxyId >= _nodes.length) return false;
    final node = _nodes[proxyId];
    if (node.height < 0 || !node.isLeaf) return false;

    final item = node.item;
    if (item != null && _itemToProxy[item] == proxyId) {
      _itemToProxy.remove(item);
    }
    _removeLeaf(proxyId);
    _freeNode(proxyId);
    _count--;
    return true;
  }

  void clear() {
    _nodes.clear();
    _itemToProxy.clear();
    _root = _nullNode;
    _freeList = _nullNode;
    _count = 0;
    _nodeCount = 0;
    _reinsertionCount = 0;
    _moveCallCount = 0;
  }

  /// Queries all items whose tight bounds overlap [bounds].
  List<T> queryAabb(
    GridAabb bounds, {
    bool Function(T item)? filter,
    List<T>? out,
  }) {
    final results = out ?? <T>[];
    results.clear();
    if (_root == _nullNode) return results;

    final stack = <int>[_root];
    while (stack.isNotEmpty) {
      final nodeId = stack.removeLast();
      if (nodeId == _nullNode) continue;
      final node = _nodes[nodeId];
      if (!node.aabb.intersects(bounds)) continue;
      if (node.isLeaf) {
        final item = node.item;
        final tight = node.tightBounds;
        if (item != null &&
            tight != null &&
            tight.intersects(bounds) &&
            (filter == null || filter(item))) {
          results.add(item);
        }
      } else {
        stack.add(node.child1);
        stack.add(node.child2);
      }
    }
    return results;
  }

  /// Queries all items overlapping a sphere centered at [center] with [radius].
  List<T> querySphere(
    GridVec3 center,
    double radius, {
    bool Function(T item)? filter,
    List<T>? out,
  }) {
    final results = out ?? <T>[];
    results.clear();
    if (_root == _nullNode) return results;

    final sphereBox = GridAabb.fromCenterRadius(center, radius);
    final stack = <int>[_root];
    while (stack.isNotEmpty) {
      final nodeId = stack.removeLast();
      if (nodeId == _nullNode) continue;
      final node = _nodes[nodeId];
      if (!node.aabb.intersects(sphereBox)) continue;
      if (node.isLeaf) {
        final item = node.item;
        final tight = node.tightBounds;
        if (item != null &&
            tight != null &&
            tight.intersectsSphere(center, radius) &&
            (filter == null || filter(item))) {
          results.add(item);
        }
      } else {
        stack.add(node.child1);
        stack.add(node.child2);
      }
    }
    return results;
  }

  /// Queries all items intersecting [frustum].
  List<T> queryFrustum(
    GridFrustum frustum, {
    bool Function(T item)? filter,
    List<T>? out,
  }) {
    final results = out ?? <T>[];
    results.clear();
    if (_root == _nullNode) return results;

    final stack = <int>[_root];
    while (stack.isNotEmpty) {
      final nodeId = stack.removeLast();
      if (nodeId == _nullNode) continue;
      final node = _nodes[nodeId];
      if (!frustum.intersectsAabb(node.aabb)) continue;
      if (node.isLeaf) {
        final item = node.item;
        final tight = node.tightBounds;
        if (item != null &&
            tight != null &&
            frustum.intersectsAabb(tight) &&
            (filter == null || filter(item))) {
          results.add(item);
        }
      } else {
        stack.add(node.child1);
        stack.add(node.child2);
      }
    }
    return results;
  }

  /// Enumerates unique overlapping `(itemA, itemB)` pairs across all leaves using tight AABBs.
  List<(T, T)> queryOverlapPairs() {
    final pairs = <(T, T)>[];
    if (_root == _nullNode || _count < 2) return pairs;

    final stack = <int>[];
    for (final entry in _itemToProxy.entries) {
      final proxyA = entry.value;
      final nodeA = _nodes[proxyA];
      final tightA = nodeA.tightBounds;
      if (tightA == null) continue;

      stack.clear();
      stack.add(_root);
      while (stack.isNotEmpty) {
        final nodeId = stack.removeLast();
        if (nodeId == _nullNode) continue;
        final nodeB = _nodes[nodeId];
        if (!nodeB.aabb.intersects(tightA)) continue;
        if (nodeB.isLeaf) {
          // Enforce canonical ordering by proxy index so each pair appears once.
          if (nodeId > proxyA &&
              nodeB.tightBounds != null &&
              nodeB.tightBounds!.intersects(tightA)) {
            final itemB = nodeB.item;
            if (itemB != null) {
              pairs.add((entry.key, itemB));
            }
          }
        } else {
          stack.add(nodeB.child1);
          stack.add(nodeB.child2);
        }
      }
    }
    return pairs;
  }

  /// Finds the closest hit along [ray] within [maxDistance].
  ///
  /// Supply [narrowPhase] to refine exact collider geometry (returns hit distance or null).
  GridRayHit<T>? raycastClosest(
    GridRay ray, {
    double maxDistance = double.infinity,
    double? Function(T item, GridRay ray, double bestDist)? narrowPhase,
    bool Function(T item)? filter,
  }) {
    if (_root == _nullNode) return null;

    var bestT = maxDistance;
    int bestProxy = _nullNode;
    T? bestItem;

    final stack = <int>[_root];
    while (stack.isNotEmpty) {
      final nodeId = stack.removeLast();
      if (nodeId == _nullNode) continue;
      final node = _nodes[nodeId];
      final nodeT = ray.intersectAabb(node.aabb, bestT);
      if (nodeT == null) continue;

      if (node.isLeaf) {
        final item = node.item;
        final tight = node.tightBounds;
        if (item == null || tight == null) continue;
        if (filter != null && !filter(item)) continue;

        final aabbT = ray.intersectAabb(tight, bestT);
        if (aabbT == null) continue;

        final exactT = narrowPhase != null ? narrowPhase(item, ray, bestT) : aabbT;
        if (exactT != null && exactT <= bestT) {
          bestT = exactT;
          bestProxy = nodeId;
          bestItem = item;
        }
      } else {
        // Explore closer child first when ray hits both to maximize pruning.
        final c1 = node.child1;
        final c2 = node.child2;
        final t1 = ray.intersectAabb(_nodes[c1].aabb, bestT);
        final t2 = ray.intersectAabb(_nodes[c2].aabb, bestT);
        if (t1 != null && t2 != null) {
          if (t1 <= t2) {
            stack.add(c2);
            stack.add(c1);
          } else {
            stack.add(c1);
            stack.add(c2);
          }
        } else if (t1 != null) {
          stack.add(c1);
        } else if (t2 != null) {
          stack.add(c2);
        }
      }
    }

    if (bestProxy == _nullNode || bestItem == null) return null;
    return GridRayHit<T>(
      item: bestItem,
      proxyId: bestProxy,
      distance: bestT,
      point: ray.pointAt(bestT),
    );
  }

  /// Returns all items hit along [ray] within [maxDistance], sorted by ascending distance.
  List<GridRayHit<T>> raycastAll(
    GridRay ray, {
    double maxDistance = double.infinity,
    double? Function(T item, GridRay ray, double maxDist)? narrowPhase,
    bool Function(T item)? filter,
  }) {
    final hits = <GridRayHit<T>>[];
    if (_root == _nullNode) return hits;

    final stack = <int>[_root];
    while (stack.isNotEmpty) {
      final nodeId = stack.removeLast();
      if (nodeId == _nullNode) continue;
      final node = _nodes[nodeId];
      if (ray.intersectAabb(node.aabb, maxDistance) == null) continue;

      if (node.isLeaf) {
        final item = node.item;
        final tight = node.tightBounds;
        if (item == null || tight == null) continue;
        if (filter != null && !filter(item)) continue;

        final aabbT = ray.intersectAabb(tight, maxDistance);
        if (aabbT == null) continue;

        final exactT = narrowPhase != null ? narrowPhase(item, ray, maxDistance) : aabbT;
        if (exactT != null && exactT <= maxDistance) {
          hits.add(GridRayHit<T>(
            item: item,
            proxyId: nodeId,
            distance: exactT,
            point: ray.pointAt(exactT),
          ));
        }
      } else {
        stack.add(node.child1);
        stack.add(node.child2);
      }
    }

    hits.sort((a, b) => a.distance.compareTo(b.distance));
    return hits;
  }

  double computeSurfaceAreaSum() {
    var total = 0.0;
    for (final node in _nodes) {
      if (node.height >= 0) {
        total += node.aabb.surfaceArea;
      }
    }
    return total;
  }

  int _allocateNode() {
    if (_freeList != _nullNode) {
      final nodeId = _freeList;
      final node = _nodes[nodeId];
      _freeList = node.parentOrNext;
      node.parentOrNext = _nullNode;
      node.child1 = _nullNode;
      node.child2 = _nullNode;
      node.height = 0;
      node.item = null;
      node.tightBounds = null;
      _nodeCount++;
      return nodeId;
    }
    final node = _BvhNode<T>()..height = 0;
    _nodes.add(node);
    _nodeCount++;
    return _nodes.length - 1;
  }

  void _freeNode(int nodeId) {
    final node = _nodes[nodeId];
    node.item = null;
    node.tightBounds = null;
    node.height = -1;
    node.parentOrNext = _freeList;
    _freeList = nodeId;
    _nodeCount--;
  }

  void _insertLeaf(int leaf) {
    if (_root == _nullNode) {
      _root = leaf;
      _nodes[_root].parentOrNext = _nullNode;
      return;
    }

    final leafAabb = _nodes[leaf].aabb;
    var index = _root;
    while (!_nodes[index].isLeaf) {
      final node = _nodes[index];
      final child1 = node.child1;
      final child2 = node.child2;

      final area = node.aabb.surfaceArea;
      final combinedAabb = node.aabb.union(leafAabb);
      final combinedArea = combinedAabb.surfaceArea;

      // Cost of creating a new parent for this node and the new leaf.
      final cost = 2.0 * combinedArea;
      // Inheritance cost of pushing leaf down into this subtree.
      final inheritanceCost = 2.0 * (combinedArea - area);

      double descendCost(int childIdx) {
        final child = _nodes[childIdx];
        final merged = leafAabb.union(child.aabb);
        if (child.isLeaf) {
          return merged.surfaceArea + inheritanceCost;
        }
        return (merged.surfaceArea - child.aabb.surfaceArea) + inheritanceCost;
      }

      final cost1 = descendCost(child1);
      final cost2 = descendCost(child2);

      if (cost < cost1 && cost < cost2) {
        break;
      }
      index = cost1 < cost2 ? child1 : child2;
    }

    final sibling = index;
    final oldParent = _nodes[sibling].parentOrNext;
    final newParent = _allocateNode();
    _nodes[newParent].parentOrNext = oldParent;
    _nodes[newParent].aabb = leafAabb.union(_nodes[sibling].aabb);
    _nodes[newParent].height = _nodes[sibling].height + 1;

    if (oldParent != _nullNode) {
      if (_nodes[oldParent].child1 == sibling) {
        _nodes[oldParent].child1 = newParent;
      } else {
        _nodes[oldParent].child2 = newParent;
      }
    } else {
      _root = newParent;
    }

    _nodes[newParent].child1 = sibling;
    _nodes[newParent].child2 = leaf;
    _nodes[sibling].parentOrNext = newParent;
    _nodes[leaf].parentOrNext = newParent;

    _refitUpwards(newParent);
  }

  void _removeLeaf(int leaf) {
    if (leaf == _root) {
      _root = _nullNode;
      return;
    }

    final parent = _nodes[leaf].parentOrNext;
    final grandParent = _nodes[parent].parentOrNext;
    final sibling = _nodes[parent].child1 == leaf
        ? _nodes[parent].child2
        : _nodes[parent].child1;

    if (grandParent != _nullNode) {
      if (_nodes[grandParent].child1 == parent) {
        _nodes[grandParent].child1 = sibling;
      } else {
        _nodes[grandParent].child2 = sibling;
      }
      _nodes[sibling].parentOrNext = grandParent;
      _freeNode(parent);
      _refitUpwards(grandParent);
    } else {
      _root = sibling;
      _nodes[sibling].parentOrNext = _nullNode;
      _freeNode(parent);
    }
  }

  void _refitUpwards(int startIndex) {
    var index = startIndex;
    while (index != _nullNode) {
      index = _balance(index);
      final node = _nodes[index];
      final child1 = node.child1;
      final child2 = node.child2;
      node.height = 1 + math.max(_nodes[child1].height, _nodes[child2].height);
      node.aabb = _nodes[child1].aabb.union(_nodes[child2].aabb);
      index = node.parentOrNext;
    }
  }

  int _balance(int iA) {
    final a = _nodes[iA];
    if (a.isLeaf || a.height < 2) return iA;

    final iB = a.child1;
    final iC = a.child2;
    final b = _nodes[iB];
    final c = _nodes[iC];
    final balance = c.height - b.height;

    if (balance > 1) {
      final iF = c.child1;
      final iG = c.child2;
      final f = _nodes[iF];
      final g = _nodes[iG];

      c.child1 = iA;
      c.parentOrNext = a.parentOrNext;
      a.parentOrNext = iC;
      if (c.parentOrNext != _nullNode) {
        if (_nodes[c.parentOrNext].child1 == iA) {
          _nodes[c.parentOrNext].child1 = iC;
        } else {
          _nodes[c.parentOrNext].child2 = iC;
        }
      } else {
        _root = iC;
      }

      if (f.height > g.height) {
        c.child2 = iF;
        a.child2 = iG;
        g.parentOrNext = iA;
        a.aabb = b.aabb.union(g.aabb);
        c.aabb = a.aabb.union(f.aabb);
        a.height = 1 + math.max(b.height, g.height);
        c.height = 1 + math.max(a.height, f.height);
      } else {
        c.child2 = iG;
        a.child2 = iF;
        f.parentOrNext = iA;
        a.aabb = b.aabb.union(f.aabb);
        c.aabb = a.aabb.union(g.aabb);
        a.height = 1 + math.max(b.height, f.height);
        c.height = 1 + math.max(a.height, g.height);
      }
      return iC;
    }

    if (balance < -1) {
      final iD = b.child1;
      final iE = b.child2;
      final d = _nodes[iD];
      final e = _nodes[iE];

      b.child1 = iA;
      b.parentOrNext = a.parentOrNext;
      a.parentOrNext = iB;
      if (b.parentOrNext != _nullNode) {
        if (_nodes[b.parentOrNext].child1 == iA) {
          _nodes[b.parentOrNext].child1 = iB;
        } else {
          _nodes[b.parentOrNext].child2 = iB;
        }
      } else {
        _root = iB;
      }

      if (d.height > e.height) {
        b.child2 = iD;
        a.child1 = iE;
        e.parentOrNext = iA;
        a.aabb = c.aabb.union(e.aabb);
        b.aabb = a.aabb.union(d.aabb);
        a.height = 1 + math.max(c.height, e.height);
        b.height = 1 + math.max(a.height, d.height);
      } else {
        b.child2 = iE;
        a.child1 = iD;
        d.parentOrNext = iA;
        a.aabb = c.aabb.union(d.aabb);
        b.aabb = a.aabb.union(e.aabb);
        a.height = 1 + math.max(c.height, d.height);
        b.height = 1 + math.max(a.height, e.height);
      }
      return iB;
    }

    return iA;
  }
}
