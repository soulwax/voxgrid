import 'dart:math' as math;

/// Lightweight immutable 3D vector used across `voxgrid`.
final class GridVec3 {
  final double x;
  final double y;
  final double z;

  const GridVec3(this.x, this.y, this.z);

  static const GridVec3 zero = GridVec3(0.0, 0.0, 0.0);

  double get lengthSquared => x * x + y * y + z * z;
  double get length => math.sqrt(lengthSquared);

  GridVec3 get normalized {
    final len = length;
    if (len < 1e-10) return zero;
    final inv = 1.0 / len;
    return GridVec3(x * inv, y * inv, z * inv);
  }

  GridVec3 operator +(GridVec3 o) => GridVec3(x + o.x, y + o.y, z + o.z);
  GridVec3 operator -(GridVec3 o) => GridVec3(x - o.x, y - o.y, z - o.z);
  GridVec3 operator -() => GridVec3(-x, -y, -z);
  GridVec3 operator *(double s) => GridVec3(x * s, y * s, z * s);

  double dot(GridVec3 o) => x * o.x + y * o.y + z * o.z;

  static GridVec3 min(GridVec3 a, GridVec3 b) => GridVec3(
        math.min(a.x, b.x),
        math.min(a.y, b.y),
        math.min(a.z, b.z),
      );

  static GridVec3 max(GridVec3 a, GridVec3 b) => GridVec3(
        math.max(a.x, b.x),
        math.max(a.y, b.y),
        math.max(a.z, b.z),
      );

  @override
  String toString() => 'GridVec3($x, $y, $z)';
}

/// 3D ray with precomputed reciprocal direction for fast slab intersection.
final class GridRay {
  final GridVec3 origin;
  final GridVec3 direction;
  final double invDirX;
  final double invDirY;
  final double invDirZ;

  GridRay({
    required this.origin,
    required GridVec3 direction,
  })  : direction = direction.normalized,
        invDirX = _safeInv(direction.normalized.x),
        invDirY = _safeInv(direction.normalized.y),
        invDirZ = _safeInv(direction.normalized.z);

  static double _safeInv(double v) => v.abs() < 1e-12 ? double.infinity : 1.0 / v;

  GridVec3 pointAt(double t) => origin + direction * t;

  /// Returns hit entry distance `>= 0` (0 when origin is inside [box]), or `null` if disjoint.
  double? intersectAabb(GridAabb box, [double maxDistance = double.infinity]) {
    var tMin = 0.0;
    var tMax = maxDistance;

    // X slab
    if (invDirX.isInfinite) {
      if (origin.x < box.min.x || origin.x > box.max.x) return null;
    } else {
      var t1 = (box.min.x - origin.x) * invDirX;
      var t2 = (box.max.x - origin.x) * invDirX;
      if (t1 > t2) {
        final tmp = t1;
        t1 = t2;
        t2 = tmp;
      }
      tMin = math.max(tMin, t1);
      tMax = math.min(tMax, t2);
      if (tMin > tMax) return null;
    }

    // Y slab
    if (invDirY.isInfinite) {
      if (origin.y < box.min.y || origin.y > box.max.y) return null;
    } else {
      var t1 = (box.min.y - origin.y) * invDirY;
      var t2 = (box.max.y - origin.y) * invDirY;
      if (t1 > t2) {
        final tmp = t1;
        t1 = t2;
        t2 = tmp;
      }
      tMin = math.max(tMin, t1);
      tMax = math.min(tMax, t2);
      if (tMin > tMax) return null;
    }

    // Z slab
    if (invDirZ.isInfinite) {
      if (origin.z < box.min.z || origin.z > box.max.z) return null;
    } else {
      var t1 = (box.min.z - origin.z) * invDirZ;
      var t2 = (box.max.z - origin.z) * invDirZ;
      if (t1 > t2) {
        final tmp = t1;
        t1 = t2;
        t2 = tmp;
      }
      tMin = math.max(tMin, t1);
      tMax = math.min(tMax, t2);
      if (tMin > tMax) return null;
    }

    return tMin;
  }
}

/// Axis-aligned bounding box with surface-area and containment helpers.
final class GridAabb {
  final GridVec3 min;
  final GridVec3 max;

  const GridAabb(this.min, this.max);

  factory GridAabb.fromCenterHalfExtents(GridVec3 center, GridVec3 halfExtents) =>
      GridAabb(center - halfExtents, center + halfExtents);

  factory GridAabb.fromCenterRadius(GridVec3 center, double radius) {
    final r = GridVec3(radius, radius, radius);
    return GridAabb(center - r, center + r);
  }

  GridVec3 get center => (min + max) * 0.5;
  GridVec3 get size => max - min;
  GridVec3 get halfExtents => (max - min) * 0.5;

  double get surfaceArea {
    final dx = math.max(0.0, max.x - min.x);
    final dy = math.max(0.0, max.y - min.y);
    final dz = math.max(0.0, max.z - min.z);
    return 2.0 * (dx * dy + dy * dz + dz * dx);
  }

  bool contains(GridAabb inner) =>
      min.x <= inner.min.x &&
      min.y <= inner.min.y &&
      min.z <= inner.min.z &&
      max.x >= inner.max.x &&
      max.y >= inner.max.y &&
      max.z >= inner.max.z;

  bool containsPoint(GridVec3 p) =>
      p.x >= min.x &&
      p.x <= max.x &&
      p.y >= min.y &&
      p.y <= max.y &&
      p.z >= min.z &&
      p.z <= max.z;

  bool intersects(GridAabb other) =>
      min.x <= other.max.x &&
      max.x >= other.min.x &&
      min.y <= other.max.y &&
      max.y >= other.min.y &&
      min.z <= other.max.z &&
      max.z >= other.min.z;

  bool intersectsSphere(GridVec3 center, double radius) {
    final cx = center.x.clamp(min.x, max.x);
    final cy = center.y.clamp(min.y, max.y);
    final cz = center.z.clamp(min.z, max.z);
    final dx = center.x - cx;
    final dy = center.y - cy;
    final dz = center.z - cz;
    return (dx * dx + dy * dy + dz * dz) <= radius * radius;
  }

  GridAabb expand(double margin) {
    final m = GridVec3(margin, margin, margin);
    return GridAabb(min - m, max + m);
  }

  GridAabb union(GridAabb other) => GridAabb(
        GridVec3.min(min, other.min),
        GridVec3.max(max, other.max),
      );
}

/// Half-space plane `(normal.dot(p) + distance >= 0)` is inside.
final class GridPlane {
  final GridVec3 normal;
  final double distance;

  const GridPlane(this.normal, this.distance);

  GridPlane get normalized {
    final len = normal.length;
    if (len < 1e-10) return const GridPlane(GridVec3(0, 1, 0), 0);
    final inv = 1.0 / len;
    return GridPlane(normal * inv, distance * inv);
  }

  double signedDistance(GridVec3 p) => normal.dot(p) + distance;
}

/// 6-plane viewing frustum for hierarchical broadphase culling.
final class GridFrustum {
  final List<GridPlane> planes;

  const GridFrustum(this.planes);

  /// Extracts normalized frustum planes from a column-major 4x4 view-projection matrix.
  factory GridFrustum.fromColumnMajorMatrix(List<double> m) {
    if (m.length < 16) {
      throw ArgumentError.value(m.length, 'm.length', 'expected 16 matrix floats');
    }
    GridPlane plane(double a, double b, double c, double d) =>
        GridPlane(GridVec3(a, b, c), d).normalized;

    // Gribb-Hartmann extraction for column-major matrix: row i = [m[i], m[4+i], m[8+i], m[12+i]]
    return GridFrustum([
      plane(m[3] + m[0], m[7] + m[4], m[11] + m[8], m[15] + m[12]), // left
      plane(m[3] - m[0], m[7] - m[4], m[11] - m[8], m[15] - m[12]), // right
      plane(m[3] + m[1], m[7] + m[5], m[11] + m[9], m[15] + m[13]), // bottom
      plane(m[3] - m[1], m[7] - m[5], m[11] - m[9], m[15] - m[13]), // top
      plane(m[3] + m[2], m[7] + m[6], m[11] + m[10], m[15] + m[14]), // near
      plane(m[3] - m[2], m[7] - m[6], m[11] - m[10], m[15] - m[14]), // far
    ]);
  }

  /// Tests conservative AABB intersection against all planes.
  bool intersectsAabb(GridAabb box) {
    for (final p in planes) {
      final n = p.normal;
      // Positive vertex (furthest along plane normal)
      final px = n.x >= 0.0 ? box.max.x : box.min.x;
      final py = n.y >= 0.0 ? box.max.y : box.min.y;
      final pz = n.z >= 0.0 ? box.max.z : box.min.z;
      if (n.x * px + n.y * py + n.z * pz + p.distance < 0.0) {
        return false;
      }
    }
    return true;
  }
}

/// Ray hit result returned by spatial broad-phase and narrow-phase queries.
final class GridRayHit<T> {
  final T item;
  final int proxyId;
  final double distance;
  final GridVec3 point;

  const GridRayHit({
    required this.item,
    required this.proxyId,
    required this.distance,
    required this.point,
  });
}
