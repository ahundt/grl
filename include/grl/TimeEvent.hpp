#ifndef TIME_EVENT_HPP
#define TIME_EVENT_HPP

#include "cartographer/common/time.h"
#include <string>
#include <array>

namespace grl {

class TimeEvent
{
public:
        typedef std::array<uint8_t,512> UnsignedCharArray;

        /// Identifying string for this time stamped data topic
        /// something like "/opticaltracker/00000000/frame" where
        /// 00000000 is the serial number of the optical tracker.
        UnsignedCharArray             event_name;

        /// The time just before a data update request is made
        cartographer::common::Time    local_request_time;

        /// Identifying string for the clock used to drive the device
        /// something like "/opticaltracker/00000000/clock"
        /// if it is the clock internal to a sensor like an optical tracker
        UnsignedCharArray             device_clock_id;

        /// The time provided by the device specified by device_clock_id
        cartographer::common::Time    device_time;

        /// Identifying string for the clock used to drive the device
        /// or "/control_computer/clock/steady" if the device has no clock
        /// and the time is the desktop computer
        /// running the steady clock (vs clocks which might change time)
        UnsignedCharArray              local_clock_id;

        /// The time at which the data was received
        cartographer::common::Time     local_receive_time;

        /// The corrected local time which represents when the sensor
        /// data was actually captured.
        cartographer::common::Time     corrected_local_time;

        /// Estimated duration of the skew between the device clock
        /// and the local time clock
        cartographer::common::Duration clock_skew;

        /// The minimum expected delay in transporting the data request
        cartographer::common::Duration min_transport_delay;

};

}

#endif // TIME_EVENT_HPP
