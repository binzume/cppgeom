
#undef NDEBUG
#include <cassert>
#include <iostream>
#include <numbers>

#include "bsptree.h"

using namespace std;
using namespace geom;

void test_vector3() {
  Vector3 a(1, 2, 3);
  Vector3 b(4, 3, 0);
  assert(a == a);
  assert(b.length() == 5);
  assert((a - a).length() == 0);
}

void test_quaternion() {
  // 90degree, around y
  Quaternion q = Quaternion::fromAxisAngle(Vector3(0, 1, 0), numbers::pi / 2);
  cout << q.to_string() << endl;
  cout << q.applyTo(Vector3(1, 0, 0)) << endl;
  assert((q.applyTo(Vector3(1, 0, 0)) - Vector3(0, 0, -1)).length() < 1e-8);

  Quaternion q1(0, 0, 0, 1);
  assert(q1.length() == 1);
}

void test_plane() {
  // y=1 xz-plane
  auto plane =
      Plane::fromPoints(Vector3(0, 1, 0), Vector3(0, 1, 1), Vector3(1, 1, 0));
  assert(plane.isValid());

  assert(plane.w == 1);
  assert(plane.normal == Vector3(0, 1, 0));

  assert(plane.distanceTo(Vector3(123, 1, 456)) == 0);
  assert(plane.distanceTo(Vector3(123, 0, 456)) == 1);
  assert(plane.distanceTo(Vector3(123, 2, 456)) == 1);

  assert(plane.signedDistanceTo(Vector3(123, 0, 456)) == -1);

  PlaneT<FloatType> uninitialized;
  assert(!uninitialized.isValid());

  auto invalid =
      Plane::fromPoints(Vector3(0, 0, 0), Vector3(0, 0, 0), Vector3(0, 0, 0));
  assert(!invalid.isValid());
}

void test_ray() {
  // y=1 xz-plane
  auto plane =
      Plane::fromPoints(Vector3(0, 1, 0), Vector3(0, 1, 1), Vector3(1, 1, 0));

  Vector3 r;
  Ray ray0{{0, 0, 0}, {0, 1, 0}};
  assert(ray0.distanceTo(plane) == 1);
  assert(ray0.intersects(plane, r) == true);
  assert(r == Vector3(0, 1, 0));

  Ray ray1{{0, 0, 0}, {0, 0, 1}};
  assert(ray1.distanceTo(plane) < 0);
  assert(ray1.intersects(plane, r) == false);

  Ray ray2{{0, 2, 0}, {0, 1, 0}};
  assert(ray2.distanceTo(plane) < 0);
  assert(ray2.intersects(plane, r) == false);
}

void test_bsp() {
  // y=0 plane
  vector<Polygon<FloatType>> bsp_polygons{{{{0, 0, 0}, {0, 0, 1}, {1, 0, 0}}}};
  for (const auto &p : bsp_polygons) {
    cout << p << endl;
  }
  BSPNode bsp(bsp_polygons);

  assert(bsp_polygons[0].plane.distanceTo(Vector3(0, 0, 0)) == 0);
  assert(bsp_polygons[0].plane.distanceTo(Vector3(0, 42, 0)) == 42);
  assert(bsp.classifyPoint(Vector3(123, 0, 456)) == Plane::COPLANAR);
  assert(bsp.classifyPoint(Vector3(0, 1, 0)) == Plane::FRONT);  // outside
  assert(bsp.classifyPoint(Vector3(0, -1, 0)) == Plane::BACK);  // inside

  vector<Polygon<FloatType>> polygons{{{{0, 1, 0}, {1, -1, 0}, {-1, -1, 0}}}};

  vector<Polygon<FloatType>> in, out;
  bsp.splitPolygons(polygons, in, out);
  cout << "IN:" << in.size() << endl;
  for (const auto &p : in) {
    cout << p << endl;
  }
  cout << "OUT:" << out.size() << endl;
  for (const auto &p : out) {
    cout << p << endl;
  }

  Vector3 r;
  Ray ray(Vector3(0, 1, 0), Vector3(0, -1, 0).normalized());
  if (bsp.raycast(ray, r)) {
    assert(r == Vector3(0, 0, 0));
  } else {
    assert(false);
  }

  ray = Ray(Vector3(0, -1, 0), Vector3(0, -1, 0).normalized());
  if (bsp.raycast(ray, r)) {
    assert(r == Vector3(0, -1, 0));
  } else {
    assert(false);
  }
}

int main() {
  cout << "test_vector3" << endl;
  test_vector3();
  cout << "test_quaternion" << endl;
  test_quaternion();
  cout << "test_plane" << endl;
  test_plane();
  cout << "test_ray" << endl;
  test_ray();
  cout << "test_bsp" << endl;
  test_bsp();
  cout << "ok." << endl;
  return 0;
}
