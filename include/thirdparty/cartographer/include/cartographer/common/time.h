/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef CARTOGRAPHER_COMMON_TIME_H_
#define CARTOGRAPHER_COMMON_TIME_H_

#include <chrono>
#include <ostream>
#include <ratio>
#include <string>

#include "cartographer/common/port.h"

namespace cartographer {
namespace common {

constexpr int64 kUtsEpochOffsetFromUnixEpochInSeconds =
    (719162ll * 24ll * 60ll * 60ll);

struct UniversalTimeScaleClock {
  using rep = int64;
  using period = std::ratio<1, 10000000>;
  using duration = std::chrono::duration<rep, period>;
  using time_point = std::chrono::time_point<UniversalTimeScaleClock>;
  static constexpr bool is_steady = true;

  static time_point now() noexcept
  {
        using namespace std::chrono;
        // std::chrono::time_point<std::chrono::system_clock> p1 = std::chrono::system_clock::now();
        // std::chrono::seconds sec(kUtsEpochOffsetFromUnixEpochInSeconds);
        // int64 microsecsinceepoch = std::chrono::duration_cast<duration>(p1.time_since_epoch()).count();
        // int64 microsec = std::chrono::microseconds(sec).count()*10;
        // std::cout << "microseconds since epoch: " << microsecsinceepoch << '\n';
        // std::cout << "microseconds since the Epoch which is January 1, 1 at the start of day in UTC: "
        //           << microsecsinceepoch - microsec << '\n';
        return time_point
        (
          duration_cast<duration>(system_clock::now().time_since_epoch())
        );
  }
};

// Represents Universal Time Scale durations and timestamps which are 64-bit
// integers representing the 100 nanosecond ticks since the Epoch which is
// January 1, 1 at the start of day in UTC.
using Duration = UniversalTimeScaleClock::duration;
using Time = UniversalTimeScaleClock::time_point;

// Convenience functions to create common::Durations.
Duration FromSeconds(const double seconds) {
  return std::chrono::duration_cast<Duration>(
      std::chrono::duration<double>(seconds));
}
Duration FromMilliseconds(int64 milliseconds);

// Returns the given duration in seconds.
double ToSeconds(const Duration duration) {
  return std::chrono::duration_cast<std::chrono::duration<double>>(duration)
      .count();
}

// Creates a time from a Universal Time Scale.
Time FromUniversal(const int64 ticks) { return Time(Duration(ticks)); }

// Outputs the Universal Time Scale timestamp for a given Time.
int64 ToUniversal(const Time time) { return time.time_since_epoch().count(); }

// For logging and unit tests, outputs the timestamp integer.
std::ostream& operator<<(std::ostream& os, const Time time) {
  os << std::to_string(ToUniversal(time));
  return os;
}

common::Duration FromMilliseconds(const int64 milliseconds) {
  return std::chrono::duration_cast<Duration>(
      std::chrono::milliseconds(milliseconds));
}
}  // namespace common
}  // namespace cartographer

#endif  // CARTOGRAPHER_COMMON_TIME_H_
