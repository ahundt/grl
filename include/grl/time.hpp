#ifndef GRL_TIME_HPP
#define GRL_TIME_HPP
#include <iostream> // std::cout
#include <chrono>
#include <ctime>   // localtime
#include <sstream> // stringstrea
#include <iomanip> // put_time
#include <string> // string

/// Convert the local date and time to a string formatted
/// as YEAR_MONTH_DAY_HOUR_MINUTE_SECOND, such as
/// 2017_10_31_18_21_56.
///
/// based on https://stackoverflow.com/a/17223443/99379
inline std::string current_date_and_time_string() {
	auto now = std::chrono::system_clock::now();
	auto in_time_t = std::chrono::system_clock::to_time_t(now);

	std::stringstream ss;
    constexpr auto format = "%Y_%m_%d_%H_%M_%S";
	ss << std::put_time(std::localtime(&in_time_t), format);
	return ss.str();
}

#endif // GRL_TIME_HPP
