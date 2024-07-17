/*
 * @lc app=leetcode.cn id=69 lang=cpp
 *
 * [69] x 的平方根
 */

// @lc code=start
class Solution {
public:
  int mySqrt(int x) {
    int left = 1, right = x;
    int ans = 0;
    while (left <= right) {
      int middle = left + ((right - left) / 2);
      if (middle*middle <= x) {
        left = middle + 1;
        ans = middle;
      } else {
        right = middle - 1;
      }
    }
    return ans;
  }
};
// @lc code=end
