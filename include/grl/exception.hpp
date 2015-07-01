#ifndef _GRL_EXCEPTION_HPP
#define _GRL_EXCEPTION_HPP

namespace grl {

    /// @see http://stackoverflow.com/questions/12498001/adding-several-boosterror-infos-of-the-same-type-to-a-boostexception
    typedef boost::error_info<struct tag_errmsg, std::string> errmsg_info;
    
    struct driver_initialization_error : public boost::exception, public std::exception
    {
      const char *what() const noexcept { return "driver initialization failed"; }
    };
}
#endif // _GRL_EXCEPTION_HPP