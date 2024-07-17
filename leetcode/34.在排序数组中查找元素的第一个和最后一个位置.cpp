/*
 * @lc app=leetcode.cn id=34 lang=cpp
 *
 * [34] 在排序数组中查找元素的第一个和最后一个位置
 */

// @lc code=start
#include <vector>
using namespace std;
class Solution {
public:
  vector<int> searchRange(vector<int>& nums, int target) {
    int left = 0, right = nums.size() - 1;
    int first = -1, last = -1;
    // 找第一个target
    while (left <= right) {
      int middle = left + ((right - left) / 2);
      if (nums[middle] < target) {
        left = middle + 1;
      } else if (nums[middle] > target) {
        right = middle - 1;
      } else {
        first = middle;
        right = middle - 1;
      }
    }

    left = 0;
    right = nums.size() - 1;
    while (left <= right) {
      int middle = left + ((right - left) / 2);
      if (nums[middle] < target) {
        left = middle + 1;
      } else if (nums[middle] > target) {
        right = middle - 1;
      } else {
        last = middle;
        left = middle + 1;
      }
    }

    return vector<int>{first, last};
  }
};
// @lc code=end
