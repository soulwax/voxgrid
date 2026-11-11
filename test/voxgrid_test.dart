import 'package:test/test.dart';
import 'package:voxgrid/voxgrid.dart';

void main() {
  group('voxgrid Spatial Partitioning', () {
    test('DynamicBvh supports SAH insertion, fattened proxy caching, overlap pairs & closest ray hit', () {
      final bvh = DynamicBvh<String>(fatMargin: 0.5);

      final pA = bvh.insert('alpha', GridAabb.fromCenterRadius(const GridVec3(0, 0, 0), 1.0));
      final pB = bvh.insert('beta', GridAabb.fromCenterRadius(const GridVec3(1.5, 0, 0), 1.0));
      bvh.insert('gamma', GridAabb.fromCenterRadius(const GridVec3(20, 0, 0), 1.0));

      expect(bvh.count, 3);

      // Overlap pairs strictly use tight bounds -> only alpha & beta overlap.
      final pairs = bvh.queryOverlapPairs();
      expect(pairs.length, 1);
      expect({pairs.single.$1, pairs.single.$2}, {'alpha', 'beta'});

      // Small motion inside fatMargin does NOT trigger tree reinsertion!
      final reinsertedSmall = bvh.moveProxy(
        pA,
        GridAabb.fromCenterRadius(const GridVec3(0.1, 0, 0), 1.0),
      );
      expect(reinsertedSmall, isFalse);
      expect(bvh.reinsertionCount, 0);

      // Large motion triggers reinsertion and shifts hit results accordingly.
      final reinsertedFar = bvh.moveProxy(
        pB,
        GridAabb.fromCenterRadius(const GridVec3(8.0, 0, 0), 1.0),
      );
      expect(reinsertedFar, isTrue);
      expect(bvh.reinsertionCount, 1);
      expect(bvh.queryOverlapPairs(), isEmpty);

      // Closest-hit ray along +X from (-5,0,0) hits 'alpha' first, then 'beta', then 'gamma'.
      final ray = GridRay(
        origin: const GridVec3(-5, 0, 0),
        direction: const GridVec3(1, 0, 0),
      );
      final closest = bvh.raycastClosest(ray);
      expect(closest, isNotNull);
      expect(closest!.item, 'alpha');

      final all = bvh.raycastAll(ray);
      expect(all.map((h) => h.item).toList(), ['alpha', 'beta', 'gamma']);

      // Removing a proxy cleanly shrinks count.
      expect(bvh.removeProxy(pA), isTrue);
      expect(bvh.raycastClosest(ray)?.item, 'beta');
    });

    test('DynamicBvh performs accurate sphere and frustum culling queries', () {
      final bvh = DynamicBvh<int>();
      for (int i = 0; i < 20; i++) {
        bvh.insert(i, GridAabb.fromCenterRadius(GridVec3(i * 4.0, 0.0, 0.0), 0.75));
      }

      final sphereHits = bvh.querySphere(const GridVec3(8.0, 0.0, 0.0), 4.5);
      expect(sphereHits.toSet(), {1, 2, 3});

      // Simple box frustum enclosing x in [-2, 10], y in [-5, 5], z in [-5, 5].
      final frustum = GridFrustum(const [
        GridPlane(GridVec3(1, 0, 0), 2.0),   // x >= -2
        GridPlane(GridVec3(-1, 0, 0), 10.0), // x <= 10
        GridPlane(GridVec3(0, 1, 0), 5.0),
        GridPlane(GridVec3(0, -1, 0), 5.0),
        GridPlane(GridVec3(0, 0, 1), 5.0),
        GridPlane(GridVec3(0, 0, -1), 5.0),
      ]);
      final visible = bvh.queryFrustum(frustum);
      expect(visible.toSet(), {0, 1, 2});
    });

    test('LooseOctree subdivides dense cells, updates relocated items & queries closest ray', () {
      final tree = LooseOctree<String>(
        worldBounds: GridAabb.fromCenterRadius(GridVec3.zero, 64.0),
        looseness: 2.0,
        maxDepth: 5,
        maxItemsPerNode: 2,
      );

      for (int i = 0; i < 12; i++) {
        tree.upsert(
          'obj_$i',
          GridAabb.fromCenterRadius(GridVec3(i * 2.0, 0.0, 0.0), 0.5),
        );
      }
      expect(tree.count, 12);

      // Update item across octree space
      tree.upsert('obj_0', GridAabb.fromCenterRadius(const GridVec3(-20, 10, 0), 0.5));

      final sphereMatches = tree.querySphere(const GridVec3(-20, 10, 0), 1.5);
      expect(sphereMatches, ['obj_0']);

      final rayHit = tree.raycastClosest(
        GridRay(
          origin: const GridVec3(-20, 10, 10),
          direction: const GridVec3(0, 0, -1),
        ),
      );
      expect(rayHit?.item, 'obj_0');
      expect(rayHit!.distance, closeTo(9.5, 1e-6));

      expect(tree.remove('obj_0'), isTrue);
      expect(tree.count, 11);
    });
  });
}
