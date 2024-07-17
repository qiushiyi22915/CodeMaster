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
  bool operator==(const Point& p) const {
    return fabs(x - p.x) < EPS && fabs(y - p.y) < EPS;
  }

  void print() const { std::cout << "(" << x << ", " << y << ") \n"; }
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

bool is_in_segment(const Point& p, const Point& a, const Point b) {
  return std::abs(cross(p, a, b)) < EPS && dot(p, a, b) >= 0;
}

class Box2d {
public:
  Point center;

  double width;   // 竖直边
  double length;  // 水平边
  double half_width;
  double half_length;

  double heading;
  double cos_heading;
  double sin_heading;

  double min_x;
  double min_y;
  double max_x;
  double max_y;
  std::vector<Point> corners;

  Box2d() = default;
  Box2d(const Point& center, double heading, double length, double width)
      : center(center),
        length(length),
        width(width),
        heading(heading),
        cos_heading(cos(heading)),
        sin_heading(sin(heading)),
        half_length(length / 2),
        half_width(width / 2) {
    const double dx1 = cos_heading * half_length;
    const double dy1 = sin_heading * half_length;
    const double dx2 = -sin_heading * half_width;
    const double dy2 = cos_heading * half_width;

    // 有点难记
    corners.emplace_back(center.x - dx1 - dx2, center.y - dy1 - dy2);  // 0
    corners.emplace_back(center.x + dx1 - dx2, center.y + dy1 - dy2);  // 1
    corners.emplace_back(center.x + dx1 + dx2, center.y + dy1 + dy2);  // 2
    corners.emplace_back(center.x - dx1 + dx2, center.y - dy1 + dy2);  // 3

    for (auto& corner : corners) {
      min_x = std::fmin(min_x, corner.x);
      max_x = std::fmax(max_x, corner.x);
      min_y = std::fmin(min_y, corner.y);
      max_y = std::fmax(max_y, corner.y);
    }
  }
};

bool sat(const Box2d& vel, const Box2d& obs) {
  // AABB粗检测
  if (vel.min_x > obs.max_x || vel.min_y > obs.max_y || obs.min_x > vel.max_x ||
      obs.min_y > vel.max_y) {
    return false;
  }

  // SAT精检测
  const double shift_x = obs.center.x - vel.center.x;
  const double shift_y = obs.center.y - vel.center.y;

  const double dx1 = vel.cos_heading * vel.half_length;
  const double dy1 = vel.sin_heading * vel.half_length;
  const double dx2 = -vel.sin_heading * vel.half_width;
  const double dy2 = vel.cos_heading * vel.half_width;
  const double dx3 = obs.cos_heading * obs.half_length;
  const double dy3 = obs.sin_heading * obs.half_length;
  const double dx4 = -obs.sin_heading * obs.half_width;
  const double dy4 = obs.cos_heading * obs.half_width;

  {  // 与apollo的代码实现做对比测试
     // const double dx1_ = vel.cos_heading * vel.half_length;
     // const double dy1_ = vel.sin_heading * vel.half_length;
     // const double dx2_ = vel.sin_heading * vel.half_width;
     // const double dy2_ = -vel.cos_heading * vel.half_width;
     // const double dx3_ = obs.cos_heading * obs.half_length;
     // const double dy3_ = obs.sin_heading * obs.half_length;
     // const double dx4_ = obs.sin_heading * obs.half_width;
     // const double dy4_ = -obs.cos_heading * obs.half_width;

    // // clang-format off
    // bool result1 = std::abs(shift_x * vel.cos_heading + shift_y * vel.sin_heading) <=
    //                 std::abs(dx3 * vel.cos_heading + dy3 * vel.sin_heading) +
    //                   std::abs(dx4 * vel.cos_heading + dy4 * vel.sin_heading) +
    //                     vel.half_length;
    // std::cout << std::abs(dx3 * vel.cos_heading + dy3 * vel.sin_heading) << "  "
    //           << std::abs(dx3_ * vel.cos_heading + dy3_ * vel.sin_heading) << "\n";
    // std::cout << std::abs(dx4 * vel.cos_heading + dy4 * vel.sin_heading) << "  "
    //           << std::abs(dx4_ * vel.cos_heading + dy4_ * vel.sin_heading) << "\n";

    // bool result2 = std::abs(shift_x * vel.sin_heading - shift_y * vel.cos_heading) <=
    //                 std::abs(dx3 * vel.sin_heading - dy3 * vel.cos_heading) +
    //                   std::abs(dx4 * vel.sin_heading - dy4 * vel.cos_heading) +
    //                     vel.half_width;
    // std::cout << std::abs(dx3 * vel.sin_heading - dy3 * vel.cos_heading) << "  "
    //           << std::abs(dx3_ * vel.sin_heading - dy3_ * vel.cos_heading) << "\n";
    // std::cout << std::abs(dx4 * vel.sin_heading - dy4 * vel.cos_heading) << "  "
    //           << std::abs(dx4_ * vel.sin_heading - dy4_ * vel.cos_heading) << "\n";

    // bool result3 = std::abs(shift_x * obs.cos_heading + shift_y * obs.sin_heading) <=
    //                 std::abs(dx1 * obs.cos_heading + dy1 * obs.sin_heading) +
    //                   std::abs(dx2 * obs.cos_heading + dy2 * obs.sin_heading) +
    //                     obs.half_length;
    // std::cout << std::abs(dx1 * obs.cos_heading + dy1 * obs.sin_heading) << "  "
    //           << std::abs(dx1_ * obs.cos_heading + dy1_ * obs.sin_heading) << "\n";
    // std::cout << std::abs(dx2 * obs.cos_heading + dy2 * obs.sin_heading) << "  "
    //           << std::abs(dx2_ * obs.cos_heading + dy2_ * obs.sin_heading) << "\n";

    // bool result4 = std::abs(shift_x * obs.sin_heading - shift_y * obs.cos_heading) <=
    //                 std::abs(dx1 * obs.sin_heading - dy1 * obs.cos_heading) +
    //                   std::abs(dx2 * obs.sin_heading - dy2 * obs.cos_heading) +
    //                     obs.half_width;
    // std::cout << std::abs(dx1 * obs.sin_heading - dy1 * obs.cos_heading) << "  "
    //           << std::abs(dx1_ * obs.sin_heading - dy1_ * obs.cos_heading) << "\n";
    // std::cout << std::abs(dx2 * obs.sin_heading - dy2 * obs.cos_heading) << "  "
    //           << std::abs(dx2_ * obs.sin_heading - dy2_ * obs.cos_heading) << "\n";

    // std::cout << result1 << "\n" << result2 << "\n" << result3 << "\n" << result4 <<
    // "\n"; return result1 && result2 && result3 && result4;
  }

  return std::abs(shift_x * vel.cos_heading + shift_y * vel.sin_heading) <=
             std::abs(dx3 * vel.cos_heading + dy3 * vel.sin_heading) +
                 std::abs(dx4 * vel.cos_heading + dy4 * vel.sin_heading) +
                 vel.half_length &&
         std::abs(shift_x * vel.sin_heading - shift_y * vel.cos_heading) <=
             std::abs(dx3 * vel.sin_heading - dy3 * vel.cos_heading) +
                 std::abs(dx4 * vel.sin_heading - dy4 * vel.cos_heading) +
                 vel.half_width &&
         std::abs(shift_x * obs.cos_heading + shift_y * obs.sin_heading) <=
             std::abs(dx1 * obs.cos_heading + dy1 * obs.sin_heading) +
                 std::abs(dx2 * obs.cos_heading + dy2 * obs.sin_heading) +
                 obs.half_length &&
         std::abs(shift_x * obs.sin_heading - shift_y * obs.cos_heading) <=
             std::abs(dx1 * obs.sin_heading - dy1 * obs.cos_heading) +
                 std::abs(dx2 * obs.sin_heading - dy2 * obs.cos_heading) + obs.half_width;
}

bool raycast(const Box2d& vel, const Box2d& obs) {
  int n = vel.corners.size();
  for (int i = 0; i < n; i++) {
    int m = obs.corners.size();
    int intersections = 0;
    for (int j = 0; j < m; j++) {
      Point p1 = obs.corners[j];
      Point p2 = obs.corners[(j + 1) % m];
      if ((vel.corners[i].y > p1.y != vel.corners[i].y > p2.y)) {
        if (is_in_segment(vel.corners[i], p1, p2)) return true;
        double intersection_x =
            (p2.x - p1.x) * (vel.corners[i].y - p1.y) / (p2.y - p1.y) +
            p1.x;  // p2.y - p1.y 不能为0
        if (intersection_x > vel.corners[i].x) {
          intersections++;
        }
      }
    }
    return intersections & 1;  // 快速判断是否为奇数
  }

  return false;
}

int main() {
  Box2d A({3, -3}, M_PI_2 / 3, 4, 2);
  Box2d B({4, 1}, M_PI_4, 2, 4);
  std::cout << "A is overlap B ? " << std::boolalpha << sat(A, B) << "\n";
  std::cout << "A is overlap B ? " << std::boolalpha << raycast(A, B) << "\n";
  return 0;
}