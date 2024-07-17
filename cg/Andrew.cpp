#include <algorithm>
#include <cmath>
#include <iostream>
#include <vector>

const double EPS = 1e-10;

// 点
class Point {
public:
  double x, y;
  Point(double x = 0, double y = 0) : x(x), y(y) {}

  Point operator+(const Point& p) const { return Point(x + p.x, y + p.y); }
  Point operator-(const Point& p) const { return Point(x - p.x, y - p.y); }
  Point operator*(double a) const { return Point(x * a, y * a); }
  Point operator/(double a) const { return Point(x / a, y / a); }

  double lengthSquared() const { return x * x + y * y; }
  double normSquared() const { return x * x + y * y; }
  double length() const { return sqrt(lengthSquared()); }
  double norm() const { return sqrt(normSquared()); }

  bool operator<(const Point& p) const { return x != p.x ? x < p.x : y < p.y; }
  bool operator==(const Point& p) const { return fabs(x - p.x) < EPS && fabs(y - p.y) < EPS; }

  void print() const { std::cout << "x:" << x << " y:" << y << "\n"; }
};

typedef Point Vector;

bool cmp(const Point& a, const Point& b) { return a.x != b.x ? a.x < b.x : a.y < b.y; }

double cross(const Point& p, const Point& a, const Point& b) {
  Vector u = b - a;
  Vector v = p - a;
  return u.x * v.y - u.y * v.x;
}

std::vector<Point> andrew(std::vector<Point> points) {
  int k = 0, n = points.size();
  if (n < 4) return points;

  std::vector<Point> hull(2 * n);

  std::sort(points.begin(), points.end());  // 排序：按x从小到大，x相同时按y从小到大

  // 求下凸包
  for (int i = 0; i < n; i++) {
    while (k > 1 && cross(hull[k - 2], hull[k - 1], points[i]) <= 0) k--;
    hull[k++] = points[i];
  }

  // 求上凸包
  int t = k;
  for (int i = n - 2; i >= 0; i--) {
    while (k > t && cross(hull[k - 2], hull[k - 1], points[i]) <= 0) k--;
    hull[k++] = points[i];
  }

  hull.resize(k - 1);
  return hull;
}

int main() {
  // 测试点集
  std::vector<Point> points = {{1, 2}, {5, 8}, {-1, 3}, {10, 3}, {-1, -2}, {-2, -1}, {0, 0}, {5, 2}, {3, 4}, {3, 1}};

  std::vector<Point> hull = andrew(points);

  // 打印凸包中的点
  std::cout << "Convex Hull Points:\n";
  for (const Point& p : hull) {
    std::cout << "(" << p.x << ", " << p.y << ")\n";
  }

  return 0;
}