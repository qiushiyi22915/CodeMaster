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
typedef std::vector<Point> Polygon;

double dot(const Point& a, const Point& b) { return a.x * b.x + a.y * b.y; }
double dot(const Point& p, const Point& a, const Point& b) {
  Vector u = b - a;
  Vector v = p - a;
  return u.x * v.x + u.y * v.y;
}
double cross(const Vector& a, const Vector& b) { return a.x * b.y - b.x * a.y; }
double cross(const Point& p, const Point& a, const Point& b) {
  Vector u = b - a;
  Vector v = p - a;
  return u.x * v.y - u.y * v.x;
}

double len(const Point& a, const Point& b) { return (a - b).length(); }

Point rotate(const Point& p, double theta) {
  Point point;
  point.x = p.x * cos(theta) - p.y * sin(theta);
  point.y = p.x * sin(theta) + p.y * cos(theta);
  return point;
}

bool is_point_in_segment(const Point& p, const Point& a, const Point& b) {
  return (fabs(cross(p - a, b - a)) < EPS && dot(p - a, p - b) <= 0);  // 注意加上fabs，dot是<=0
}

double polygon_area(const std::vector<Point>& polygon) {
  double area = 0;
  int n = polygon.size();
  for (int i = 0; i < n; i++) {
    Point p1 = polygon[i];
    Point p2 = polygon[(i + 1) % n];  // 利用余数得到polygon[0]
    area += (p1.x * p2.y - p2.x * p1.y);
  }
  return fabs(area) / 2.0;
}

bool is_point_inside_polygon(const Point& p, const std::vector<Point>& polygon) {
  bool inside = false;
  int n = polygon.size();
  for (int i = 0; i < n; i++) {
    int j = (i + 1) % n;
    bool in_y = (polygon[i].y > p.y != polygon[j].y > p.y);
    if (in_y) {
      if (is_point_in_segment(p, polygon[i], polygon[j])) {
        std::cout << "in segement"
                  << "\n";
        return false;
      }
      // 计算交点坐标
      double intersection_x = (polygon[j].x - polygon[i].x) * (p.y - polygon[i].y) / (polygon[j].y - polygon[i].y) + polygon[i].x;
      // std::cout << "intersection_x: " << intersection_x << "\n";
      if (p.x < intersection_x) {
        inside = !inside;
      }
    }
  }
  return inside;
}

std::vector<Point> andrew(std::vector<Point> points) {
  int k = 0, n = points.size();
  if (n < 4) return points;

  std::vector<Point> hull(2 * n);

  std::sort(points.begin(), points.end());  // 排序：按x从小到大，x相同时按y从小到大

  // 求下凸包
  for (int i = 0; i < n; i++) {
    while (k > 1 && cross(points[i], hull[k - 2], hull[k - 1]) <= 0) k--;
    hull[k++] = points[i];
  }

  // 求上凸包
  int t = k;
  for (int i = n - 2; i >= 0; i--) {
    while (k > t && cross(points[i], hull[k - 2], hull[k - 1]) <= 0) k--;
    hull[k++] = points[i];
  }

  hull.resize(k - 1);

  return hull;
}

double find_polygon_max_distance(const std::vector<Point>& points) {
  if (points.size() < 2) return -1;
  if (points.size() == 2) return len(points[0], points[1]);

  double distance = 0;
  // 求解凸包
  std::vector<Point> polygon = andrew(points);
  // std::cout << polygon.capacity() << "\n";  // 它的容量等于之前在andrew中hull分配的容量, 所以push_back并不会引起扩容操作
  polygon.push_back(polygon.front());
  int n = polygon.size() - 1;
  for (int i = 0, j = i + 1; i < n; i++) {
    while (cross(polygon[j], polygon[i], polygon[i + 1]) < cross(polygon[j + 1], polygon[i], polygon[i + 1])) j = (j + 1) % n;
    distance = std::max(distance, std::max(len(polygon[j], polygon[i]), len(polygon[j], polygon[i + 1])));
  }
  return distance;
}

std::vector<Point> find_min_rectangle(const std::vector<Point>& points) {
  if (points.size() < 3) return points;

  int a = 2, b = 2, c;
  double area = 1e10;
  std::vector<Point> p(4);

  // 求解凸包
  std::vector<Point> polygon = andrew(points);
  polygon.push_back(polygon.front()); // 使得凸包闭环, 让while部分的判断书写更简便
 
  int n = polygon.size() - 1;  // 
  for (int i = 0; i < n; i++) {
    while (dot(polygon[a], polygon[i], polygon[i + 1]) < dot(polygon[a + 1], polygon[i], polygon[i + 1])) a = (a + 1) % n;
    while (cross(polygon[b], polygon[i], polygon[i + 1]) < cross(polygon[b + 1], polygon[i], polygon[i + 1])) b = (b + 1) % n;
    if (i == 0) c = b;
    while (dot(polygon[c], polygon[i + 1], polygon[i]) < dot(polygon[c + 1], polygon[i + 1], polygon[i])) c = (c + 1) % n;

    double d = len(polygon[i], polygon[i + 1]);
    double R = dot(polygon[a], polygon[i], polygon[i + 1]) / d;
    double L = dot(polygon[c], polygon[i + 1], polygon[i]) / d;
    double H = fabs(cross(polygon[b], polygon[i], polygon[i + 1])) / d;

    if ((L + R - d) * H < area) {
      area = (L + R - d) * H;
      p[0] = polygon[i + 1] + (polygon[i] - polygon[i + 1]) * L / d;
      p[1] = polygon[i] + (polygon[i + 1] - polygon[i]) * R / d;
      p[2] = p[1] + rotate((polygon[i + 1] - polygon[i]), M_PI_2) * H / d;
      p[3] = p[0] + rotate((polygon[i + 1] - polygon[i]), M_PI_2) * H / d;
    }
  }
  std::cout << "Points min rectangle area: " << area << "\n";
  return p;
}

int main() {
  Point A(0, 0);
  Point B(5, -2);
  Point C(3, 3);
  Point D(0, 5);
  Point E(1, 3);
  Point F(1, 1);

  Polygon polygon;
  polygon.push_back(A);
  polygon.push_back(B);
  polygon.push_back(C);
  polygon.push_back(D);

  std::vector<Point> points{{3, 3}, {3, 8}, {-2, 3}, {-6, 7}, {-8, 1}, {-3, -5}, {3, -2}, {7, -2}, {7, -5}};

  // std::cout << "Polygon area: " << polygon_area(polygon) << "\n";
  // std::cout << "Point E is inside polygon? " << std::boolalpha << is_point_inside_polygon(E, polygon) << "\n";
  // std::cout << "Point F is inside polygon? " << std::boolalpha << is_point_inside_polygon(F, polygon) << "\n";
  std::cout << "Points max distance: " << find_polygon_max_distance(points) << "\n";

  std::vector<Point> rectangle = find_min_rectangle(points);
  std::cout << "Rectangle points:" << "\n";
  for (auto p : rectangle)
    std::cout << " (" << p.x << ", " << p.y << ")"
              << "\n";

  return 0;
}
