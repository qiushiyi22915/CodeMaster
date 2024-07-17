/*
 * @lc app=leetcode.cn id=7 lang=cpp
 *
 * [7] 整数反转
 */

// @lc code=start
class Solution {
public:
    int reverse(int x) {
        int x_reverse = 0;
        while (x) {
            if (x_reverse > INT_MAX / 10 || x_reverse < INT_MIN / 10) {
                return 0;
            }
            x_reverse = x_reverse * 10 + x % 10;
            x /= 10;
        }
        return x_reverse;
    }
};
// @lc code=end

