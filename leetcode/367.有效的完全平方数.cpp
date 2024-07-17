/*
 * @lc app=leetcode.cn id=367 lang=cpp
 *
 * [367] 有效的完全平方数
 */

// @lc code=start
class Solution {
public:
  bool isPerfectSquare(int num) {
    long long left = 1, right = num;
    while(left <= right) {
      long long middle = left + (right - left) / 2;
      long long sp = middle * middle;
      if (sp == num) {
        return true;
      } else if(sp < num) {
        left = middle + 1;
      } else {
        right = middle - 1;
      }
    }
    return false;
  }
};
// @lc code=end
