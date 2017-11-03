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
inline std::string current_date_and_time_string()
{
    /// switch to commented version when all users are on GCC > 5.0
	/// std::get_time and std::put_time manipulators are not implemented on the GCC < 5.0
	/// https://stackoverflow.com/questions/14136833/stdput-time-implementation-status-in-gcc
	/*
	auto now = std::chrono::system_clock::now();
	auto in_time_t = std::chrono::system_clock::to_time_t(now);

	std::stringstream ss;
    constexpr auto format = "%Y_%m_%d_%H_%M_%S";
	ss << std::put_time(std::localtime(&in_time_t), format);
	return ss.str();
*/
///  Get a current timestamp in format: YYYY_MM_DD_HH_MM_SS
///  https://stackoverflow.com/questions/16357999/current-date-and-time-as-string

    time_t rawtime;
    struct tm * timeinfo;
    char buffer[80];
    time (&rawtime);
    timeinfo = localtime(&rawtime);
    strftime(buffer,sizeof(buffer),"%Y_%m_%d_%H_%M_%S",timeinfo);
    std::string str(buffer);

    return str;

}

#endif // GRL_TIME_HPP
