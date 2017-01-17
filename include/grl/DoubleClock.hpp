#ifndef GRL_DOUBLE_CLOCK
#define GRL_DOUBLE_CLOCK

#include <boost/chrono/include.hpp>
#include <boost/chrono/duration.hpp>

namespace grl {

template <typename Clock>
struct DoubleClock {
    typedef double rep;
    typedef boost::ratio<1> period;
    typedef boost::chrono::duration<rep, period> duration;
    typedef boost::chrono::time_point<DoubleClock<Clock>> time_point;
    static const bool is_steady = Clock::is_steady;

    static time_point now() noexcept {
        return time_point(boost::chrono::duration_cast<duration>(
                   Clock::now().time_since_epoch()
               ));
    }

    static time_t to_time_t(const time_point& t) noexcept {
        return Clock::to_time_t(typename Clock::time_point(
                             boost::chrono::duration_cast<typename Clock::duration>(
                                 t.time_since_epoch()
                             )
                        ));
    }
    static time_point from_time_t(time_t t) noexcept {
        return time_point(boost::chrono::duration_cast<duration>(
                   Clock::from_time_t(t).time_since_epoch()
               ));
    }
};

} // namespace grl


#endif