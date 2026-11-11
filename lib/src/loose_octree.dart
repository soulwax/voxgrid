import 'dart:math' as math;

import 'geometry.dart';

final class _OctreeEntry<T> {
  final T item;
  final int id;
  GridAabb bounds;
  _OctreeNode<T> owner;

  _OctreeEntry({
    required this.item,
    required this.id,
    required this.bounds,
    required this.owner,
  });
}

final class _OctreeNode<T> {
  final GridVec3 center;
  final double tightHalfSize;
  final double looseHalfSize;
  final GridAabb looseBounds;
  final int depth;

  final List<_OctreeEntry<T>> entries = [];
  List<_OctreeNode<T>>? children;

  _OctreeNode({
    required this.center,
    required this.tightHalfSize,
    required double looseness,
    required this.depth,
  })  : looseHalfSize = tightHalfSize * looseness,
        looseBounds = GridAabb.fromCenterRadius(center, tightHalfSize * looseness);

  bool fitsTightlyInLoose(GridAabb itemBounds) => looseBounds.contains(itemBounds);
}

/// Loose Octree spatial index where every object lives in a single depth node
/// without cross-boundary duplication.
final class LooseOctree<T> {
  final GridAabb worldBounds;
  final double looseness;
  final int maxDepth;
  final int maxItemsPerNode;

  late _OctreeNode<T> _root;
  final Map<T, _OctreeEntry<T>> _entryMap = {};
  int _nextId = 1;

  LooseOctree({
    required this.worldBounds,
    this.looseness = 2.0,
    this.maxDepth = 6,
    this.maxItemsPerNode = 8,
  }) {
    if (looseness < 1.0) {
      throw ArgumentError.value(looseness, 'looseness', 'must be >= 1.0');
    }
    _resetRoot();
  }

  int get count => _entryMap.length;

  void _resetRoot() {
    final ext = worldBounds.halfExtents;
    final halfSize = math.max(1.0, math.max(ext.x, math.max(ext.y, ext.z)));
    _root = _OctreeNode<T>(
      center: worldBounds.center,
      tightHalfSize: halfSize,
      looseness: looseness,
      depth: 0,
    );
  }

  void clear() {
    _entryMap.clear();
    _resetRoot();
  }

  void upsert(T item, GridAabb bounds) {
    final existing = _entryMap[item];
    if (existing != null) {
      existing.bounds = bounds;
      final target = _selectNode(bounds);
      if (!identical(target, existing.owner)) {
        existing.owner.entries.remove(existing);
        target.entries.add(existing);
        existing.owner = target;
      }
      _maybeSubdivide(target);
      return;
    }

    final target = _selectNode(bounds);
    final entry = _OctreeEntry<T>(
      item: item,
      id: _nextId++,
      bounds: bounds,
      owner: target,
    );
    target.entries.add(entry);
    _entryMap[item] = entry;
    _maybeSubdivide(target);
  }

  bool remove(T item) {
    final entry = _entryMap.remove(item);
    if (entry == null) return false;
    entry.owner.entries.remove(entry);
    return true;
  }

  _OctreeNode<T> _selectNode(GridAabb bounds) {
    var node = _root;
    final center = bounds.center;
    final ext = bounds.halfExtents;
    final maxExtent = math.max(ext.x, math.max(ext.y, ext.z));

    while (node.depth < maxDepth) {
      final childLooseHalf = (node.tightHalfSize * 0.5) * looseness;
      if (maxExtent > childLooseHalf) break;

      if (node.children == null) break;
      final childIdx = _octantIndex(node.center, center);
      final child = node.children![childIdx];
      if (!child.fitsTightlyInLoose(bounds)) break;
      node = child;
    }
    return node;
  }

  void _maybeSubdivide(_OctreeNode<T> node) {
    if (node.children != null ||
        node.depth >= maxDepth ||
        node.entries.length <= maxItemsPerNode) {
      return;
    }

    final half = node.tightHalfSize * 0.5;
    final c = node.center;
    node.children = List<_OctreeNode<T>>.generate(8, (idx) {
      final ox = (idx & 1) != 0 ? half : -half;
      final oy = (idx & 2) != 0 ? half : -half;
      final oz = (idx & 4) != 0 ? half : -half;
      return _OctreeNode<T>(
        center: GridVec3(c.x + ox, c.y + oy, c.z + oz),
        tightHalfSize: half,
        looseness: looseness,
        depth: node.depth + 1,
      );
    });

    // Re-distribute existing entries that cleanly fit into child loose nodes.
    final kept = <_OctreeEntry<T>>[];
    for (final e in node.entries) {
      final childIdx = _octantIndex(node.center, e.bounds.center);
      final child = node.children![childIdx];
      if (child.fitsTightlyInLoose(e.bounds)) {
        child.entries.add(e);
        e.owner = child;
      } else {
        kept.add(e);
      }
    }
    node.entries
      ..clear()
      ..addAll(kept);
  }

  static int _octantIndex(GridVec3 pivot, GridVec3 point) {
    var idx = 0;
    if (point.x >= pivot.x) idx |= 1;
    if (point.y >= pivot.y) idx |= 2;
    if (point.z >= pivot.z) idx |= 4;
    return idx;
  }

  List<T> queryAabb(GridAabb area, {List<T>? out}) {
    final results = out ?? <T>[];
    results.clear();
    final stack = <_OctreeNode<T>>[_root];
    while (stack.isNotEmpty) {
      final node = stack.removeLast();
      if (!node.looseBounds.intersects(area)) continue;
      for (final e in node.entries) {
        if (e.bounds.intersects(area)) {
          results.add(e.item);
        }
      }
      final children = node.children;
      if (children != null) {
        for (final ch in children) {
          stack.add(ch);
        }
      }
    }
    return results;
  }

  List<T> querySphere(GridVec3 center, double radius, {List<T>? out}) {
    final results = out ?? <T>[];
    results.clear();
    final stack = <_OctreeNode<T>>[_root];
    while (stack.isNotEmpty) {
      final node = stack.removeLast();
      if (!node.looseBounds.intersectsSphere(center, radius)) continue;
      for (final e in node.entries) {
        if (e.bounds.intersectsSphere(center, radius)) {
          results.add(e.item);
        }
      }
      final children = node.children;
      if (children != null) {
        for (final ch in children) {
          stack.add(ch);
        }
      }
    }
    return results;
  }

  List<T> queryFrustum(GridFrustum frustum, {List<T>? out}) {
    final results = out ?? <T>[];
    results.clear();
    final stack = <_OctreeNode<T>>[_root];
    while (stack.isNotEmpty) {
      final node = stack.removeLast();
      if (!frustum.intersectsAabb(node.looseBounds)) continue;
      for (final e in node.entries) {
        if (frustum.intersectsAabb(e.bounds)) {
          results.add(e.item);
        }
      }
      final children = node.children;
      if (children != null) {
        for (final ch in children) {
          stack.add(ch);
        }
      }
    }
    return results;
  }

  GridRayHit<T>? raycastClosest(
    GridRay ray, {
    double maxDistance = double.infinity,
  }) {
    var bestT = maxDistance;
    _OctreeEntry<T>? bestEntry;

    final stack = <_OctreeNode<T>>[_root];
    while (stack.isNotEmpty) {
      final node = stack.removeLast();
      if (ray.intersectAabb(node.looseBounds, bestT) == null) continue;
      for (final e in node.entries) {
        final t = ray.intersectAabb(e.bounds, bestT);
        if (t != null && t <= bestT) {
          bestT = t;
          bestEntry = e;
        }
      }
      final children = node.children;
      if (children != null) {
        for (final ch in children) {
          stack.add(ch);
        }
      }
    }

    if (bestEntry == null) return null;
    return GridRayHit<T>(
      item: bestEntry.item,
      proxyId: bestEntry.id,
      distance: bestT,
      point: ray.pointAt(bestT),
    );
  }
}
