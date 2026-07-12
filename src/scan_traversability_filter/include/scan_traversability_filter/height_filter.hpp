// Copyright 2026 neepu
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#ifndef SCAN_TRAVERSABILITY_FILTER__HEIGHT_FILTER_HPP_
#define SCAN_TRAVERSABILITY_FILTER__HEIGHT_FILTER_HPP_

#include <algorithm>
#include <cmath>

namespace scan_traversability_filter
{

struct HeightFilterParameters
{
  double max_height_range{0.3};
  double ramped_height_range_a{0.5};
  double ramped_height_range_b{0.8};
  double ramped_height_range_c{0.35};
};

inline bool isWithinHeightLimit(
  const double point_x, const double point_y, const double point_z,
  const double reference_x, const double reference_y, const double reference_z,
  const HeightFilterParameters & parameters)
{
  const double horizontal_distance = std::hypot(
    point_x - reference_x, point_y - reference_y);
  const double ramped_distance = std::max(
    horizontal_distance - parameters.ramped_height_range_b, 0.0);
  const double ramped_height_limit =
    ramped_distance * parameters.ramped_height_range_a +
    parameters.ramped_height_range_c;
  const double relative_height = point_z - reference_z;

  return relative_height <= parameters.max_height_range &&
         relative_height <= ramped_height_limit;
}

}  // namespace scan_traversability_filter

#endif  // SCAN_TRAVERSABILITY_FILTER__HEIGHT_FILTER_HPP_
