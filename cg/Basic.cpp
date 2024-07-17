#include <cmath>
#include <iostream>
#include <vector>

constexpr double EPS = 1e-6;

// 点
class Point {
public:
  double x, y;
  Point(double x = 0, double y = 0) : x(x), y(y) {}

  Point operator+(const Point& p) const { return Point{x + p.x, y + p.y}; }
  Point operator-(const Point& p) const { return Point{x - p.x, y - p.y}; }
  Point operator*(double a) const { return Point{x * a, y * a}; }
  Point operator/(double a) const { return Point{x / a, y / a}; }

  double lengthSquared() const { return x * x + y * y; }
  double length() const { return sqrt(lengthSquared()); }

  bool operator<(const Point& p) const { return x != p.x ? x < p.x : y < p.y; }
  bool operator==(const Point& p) const {
    return fabs(x - p.x) < EPS && fabs(y - p.y) < EPS;
  }

  void print() const { std::cout << "x:" << x << " y:" << y << "\n"; }
};
Point operator*(double k, const Point& p) { return Point{p.x * k, p.y * k}; }

bool cmp(const Point& a, const Point& b) { return a.x != b.x ? a.x < b.x : a.y < b.y; }

/****** 向量 ******/
typedef Point Vector;
// 外积、叉积：向量a和向量b所成的平行四边形面积
// 叉积有正负之分，即a×b=-b×a，角度θ是取前者逆时针转到后者的角度
double cross(const Vector& a, const Vector& b) { return a.x * b.y - b.x * a.y; }
// 内积/点积：向量a在向量b的投影乘以向量b的长度
double dot(const Vector& a, const Vector& b) { return a.x * b.x + a.y * b.y; }
// 向量夹角
double theta(const Vector& a, const Vector& b) {
  double length_ab = a.length() * b.length();
  return std::acos(dot(a, b) / length_ab);
}
// 求向量a逆时针旋转角度θ后的向量 (θ是弧度)  注：一般认为逆时针旋转方向为正方向
Vector rotated_vector(const Vector& a, double theta) {
  // 利用旋转矩阵
  /*
   *  | x'|    | cos(θ) -sin(θ) | | x |
   *  |   | =  |                | |   |
   *  | y'|    | sin(θ)  cos(θ) | | y |
   */
  return Vector(a.x * std::cos(theta) - a.y * std::sin(theta),
                a.x * std::sin(theta) + a.y * std::cos(theta));
}
// 点绕某点旋转（向量a的起始点不在原点时，逆时针旋转θ后的向量）
Vector rotated_vector_no_initial(const Point& a, const Point& b, double theta) {
  double x = b.x - a.x;
  double y = b.y - a.y;
  double rotated_x = x * std::cos(theta) - y * std::sin(theta);
  double rotated_y = y * std::cos(theta) + x * std::sin(theta);
  return Vector(rotated_x + a.x, rotated_y + a.y);
}

// 线段
struct Segment {
  Point p1, p2;
};

// 直线
typedef Segment Line;

/******点与直线******/
// 1. 直线的表示形式
/* a.一般式：ax+by+c=0，可以表示所有直线
 * b.斜截式: y=kx+b，无法处理竖直直线，需特殊判断
 * c.点斜式：y-y0 = k(x-x0)，同样无法处理竖直直线
 * d.向量式：r = r0 +
 * tv，r0是以原点为起点，直线上的一个固定点为终点的一个向量，v是直线上的方向向量（表示方向）
 */
// 2. 判断点是否在直线上（直线用两点表示）
bool point_on_line(const Point& a, const Point& b, const Point& p) {
  Vector v1 = p - a;
  Vector v2 = p - b;
  return fabs(cross(v1, v2)) < EPS;
}
// 3. 判断点是否在线段上
bool point_on_segment(const Point& a, const Point& b, const Point& p) {
  Vector v1 = p - a;
  Vector v2 = p - b;
  return fabs(cross(v1, v2)) < EPS &&
         dot(v1, v2) <=
             0;  // 这里注意等于零的情况，并且叉积的值等于零时，使用小于EPS要注意叉积取绝对值
}
// 4. 计算点到直线的距离
double distance_to_line(const Point& a, const Point& b, const Point& p) {
  if (a == b) {
    Vector v = p - a;
    return v.length();
  }
  Vector v = a - b;
  Vector v1 = a - p;
  Vector v2 = b - p;
  return fabs(cross(v1, v2)) / v.length();
}
// 5. 计算点到线段上的距离（需考虑点投影不在线段的情况）
double distance_point_to_segment(const Point& a, const Point& b, const Point& p) {
  Vector ab = b - a;
  Vector ap = p - a;
  Vector bp = p - b;

  // 检测P在AB的投影是否在线段上
  if (dot(ab, ap) < 0) return ap.length();  // 不在线段上，并且在靠近a的那一侧
  if (dot(ab, bp) > 0) return bp.length();  // 不在线段上，并且在靠近b的那一侧

  return fabs(cross(ap, ab)) / ab.length();  // 在线段上
}
// 6. 点到直线上的投影
Point project_point_on_line(const Point& a, const Point& b, const Point& p) {
  Vector ab = b - a;
  Vector ap = p - a;
  double t = dot(ab, ap) / ab.lengthSquared();  // aq与ab的比例

  return a + ab * t;
}
// 7. 点关于直线的映射（坐标）
Point reflect_point_across_line(const Point& a, const Point& b, const Point& p) {
  Vector ab = b - a;
  Vector ap = p - a;
  double t = dot(ab, ap) / ab.lengthSquared();

  Vector q = a + ab * t;  // 得到投影点

  return q * 2 - p;
}

// 8. 两直线的交点
Vector get_line_intersection(const Point& A, const Point& B, const Point& C,
                             const Point& D) {
  // version 1
  {
    // Vector AB = B - A;
    // Vector CD = D - C;
    // Vector CA = A - C;
    // Vector CB = B - C;

    // double sA = cross(CA, CD);  // 相当于一次函数的y
    // double sB = cross(CB, CD);

    // double k = (sB - sA) / AB.norm();  //
    // sB-sA相当于y的变化量，AB.norm()相当于x的变化量，得到一次函数斜率 
    // double t = -sA / k;                // 求y=0时所对应的t

    // return A + AB / AB.norm() * t;
  }

  Vector AC = C - A;
  Vector AB = B - A;
  Vector CD = D - C;

  if (fabs(cross(AB, CD)) < EPS)
    throw std::runtime_error("Lines are parallel or coincident");  // 排除平行或重叠情况

  double t = cross(AC, CD) / cross(AB, CD);
  return A + AB * t;
}

// 9. 判断线段与直线是否有交点
bool has_intersection_segment_line(const Point& A, const Point& B, const Point& C,
                                   const Point& D) {
  Vector AC = C - A;
  Vector AD = D - A;
  Vector BC = C - B;
  Vector BD = D - B;
  double cross_a_cd = cross(AC, AD);
  double cross_b_cd = cross(BC, BD);

  return cross_a_cd * cross_b_cd <=
         0;  // 等于0是线段端点与直线重合的情况(包括一个端点在直线上、整条线段与直线重合)
}

// 10. 判断线段与线段是否有交点
bool has_segments_intersection(const Point& A, const Point& B, const Point& C,
                               const Point& D) {
  Vector AC = C - A;
  Vector AD = D - A;
  Vector BC = C - B;
  Vector BD = D - B;

  double cross_a_cd = cross(AC, AD);
  double cross_b_cd = cross(BC, BD);
  double cross_c_ab = cross(AC, BC);
  double cross_d_ab = cross(AD, BD);

  return !(cross_a_cd * cross_b_cd > 0 || cross_c_ab * cross_d_ab > 0);
  // return cross_a_cd * cross_b_cd <= 0 && cross_c_ab * cross_d_ab <= 0;  //
  // 两种方式都可以
}

// 11. 计算两条线段的最短距离
double distance_segments(const Point& A, const Point& B, const Point& C, const Point& D) {
  if (has_segments_intersection(A, B, C, D)) return 0;
  return std::min(
      std::min(distance_point_to_segment(A, B, C), distance_point_to_segment(A, B, D)),
      std::min(distance_point_to_segment(C, D, A), distance_point_to_segment(C, D, B)));
}

// 圆
class Circle {
public:
  Point c;
  double r;
  Circle(Point c = Point(), double r = 0) : c(c), r(r) {}
};

// 多边形
typedef std::vector<Point> Polygon;

int main() {
  Point A(1, 2);
  Point B(2, 5);
  Point C(-1, 2);
  Point D(10, 3);
  Point E(1.5, 3.5);  // 点E是线段AB的中点
  Point F(-2, -1);    // AB与CF平行

  {  // 测试
     // Vector point = get_line_intersection(A, B, C, D);
     // std::cout << "line_intersection: (" << point.x << ", " << point.y << ")\n";

    // Point rotated_point = rotated_vector(A, M_PI_4);
    // std::cout << "rotated_point: (" << rotated_point.x << ", " << rotated_point.y <<
    // ")\n";

    // Point reflected_point = reflect_point_across_line(A, B, C);
    // std::cout << "reflected_point: (" << reflected_point.x << ", " << reflected_point.y
    // << ")\n";

    // Point projected_point = project_point_on_line(A, B, C);
    // std::cout << "projected_point: (" << projected_point.x << ", " << projected_point.y
    // << ")\n";

    // std::cout << "distance_point_to_segment: " << distance_point_to_segment(A, B, C) <<
    // "\n";

    // std::cout << "distance_to_line: " << distance_to_line(A, B, C) << "\n";

    // std::cout << "Point C is on segment? " << std::boolalpha << point_on_segment(A, B,
    // C) << "\n"; std::cout << "Point E is on segment? " << std::boolalpha <<
    // point_on_segment(A, B, E) << "\n";

    // std::cout << "Point C is on line? " << std::boolalpha << point_on_line(A, B, C) <<
    // "\n"; std::cout << "Point E is on line? " << std::boolalpha << point_on_line(A, B,
    // E) << "\n";

    // std::cout << "Has segments {AB} {CE} intersection? " << std::boolalpha <<
    // has_segments_intersection(A, B, C, E) << "\n"; std::cout << "Has segments {AB} {CF}
    // intersection? " << std::boolalpha << has_segments_intersection(A, B, C, F) << "\n";

    // std::cout << "Has segments {AB} to line {CE} intersection? " << std::boolalpha
    //           << has_intersection_segment_line(A, B, C, E) << "\n";
    // std::cout << "Has segments {AB} to line {CF} intersection? " << std::boolalpha
    //           << has_intersection_segment_line(A, B, C, F) << "\n";

    // std::cout << "distance_segments(AB - CD): " << distance_segments(A, B, C, D) <<
    // "\n"; std::cout << "distance_segments(AB - CF): " << distance_segments(A, B, C, F)
    // << "\n"; std::cout << "distance_segments(AB - DF): " << distance_segments(A, B, D,
    // F) << "\n";
  }

  return 0;
}