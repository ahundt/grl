/// @file KukaNanopb.hpp
///
/// Utility functions to enable boost::range to be used for nanopb repeated int and repeated double types
///
/// @see http://www.boost.org/doc/libs/1_57_0/libs/range/doc/html/range/reference/extending/method_2.html

#ifndef _KUKA_NANOPB_HPP_
#define _KUKA_NANOPB_HPP_

#include <iterator>         // for std::iterator_traits, std::distance()
#include <boost/range.hpp>
#include <boost/type_traits.hpp>
/// The pb_frimessages_callbacks.h file is defined in the kuka FRI library zip see GRL documentation for details.
#include "pb_frimessages_callbacks.h"

#ifdef BOOST_LIB_STD_CXX
/// @todo remove this when the libc++ bug linked below has been resolved
/// @see https://groups.google.com/forum/#!topic/boost-devel-archive/Pj0qsPYyFH0
/// @see https://svn.boost.org/trac/boost/ticket/9431
namespace std {
  template<> struct iterator_traits<tRepeatedDoubleArguments> {
    typedef std::ptrdiff_t difference_type;
	typedef double value_type;
	typedef double* pointer;
	typedef double& reference;
    typedef std::random_access_iterator_tag iterator_category;
  };
}

/// @todo remove this when the libc++ bug linked below has been resolved
/// @see https://groups.google.com/forum/#!topic/boost-devel-archive/Pj0qsPYyFH0
/// @see https://svn.boost.org/trac/boost/ticket/9431
namespace std {
  template<> struct iterator_traits<tRepeatedIntArguments> {
    typedef std::ptrdiff_t difference_type;
	typedef int64_t value_type;
	typedef int64_t* pointer;
	typedef int64_t& reference;
    typedef std::random_access_iterator_tag iterator_category;
  };
}
#endif

namespace boost
{
    //
    // Specialize metafunctions. We must include the range.hpp header.
    // We must open the 'boost' namespace.
    //
	template<>
	struct range_mutable_iterator< tRepeatedDoubleArguments >
	{
		typedef double* type;
	};

	template<>
	struct range_const_iterator< tRepeatedDoubleArguments >
	{
		//
		// Remark: this is defined similar to 'range_iterator'
		//         because the 'Pair' type does not distinguish
		//         between an iterator and a const_iterator.
		//
		typedef const double* type;
	};

    //
    // Specialize metafunctions. We must include the range.hpp header.
    // We must open the 'boost' namespace.
    //

	template<>
	struct range_mutable_iterator< tRepeatedIntArguments >
	{
		typedef int64_t* type;
	};

	template<>
	struct range_const_iterator< tRepeatedIntArguments >
	{
		//
		// Remark: this is defined similar to 'range_iterator'
		//         because the 'Pair' type does not distinguish
		//         between an iterator and a const_iterator.
		//
		typedef const int64_t* type;
	};

} // namespace 'boost'


    template<typename T> typename boost::range_mutable_iterator<T>::type range_begin(T&);
    template<typename T> typename boost::range_const_iterator<T>::type range_begin(const T&);
    template<typename T> typename boost::range_mutable_iterator<T>::type range_end(T&);
    template<typename T> typename boost::range_const_iterator<T>::type range_end(const T&);
	//
	// The required functions. These should be defined in
	// the same namespace as 'Pair', in this case
	// in namespace 'Foo'.
	//
	template<>
	inline double* range_begin( tRepeatedDoubleArguments& x )
	{
		return x.value;
	}

	template<>
	inline const double* range_begin( const tRepeatedDoubleArguments& x )
	{
		return x.value;
	}

	template<>
	inline double* range_end( tRepeatedDoubleArguments& x )
	{
		return x.value+x.size;
	}

	template<>
	inline const double* range_end( const tRepeatedDoubleArguments& x )
	{
		return x.value+x.size;
	}



	template<>
	inline int64_t* range_begin( repeatedIntArguments& x )
	{
		return x.value;
	}

	template<>
	inline const int64_t* range_begin( const tRepeatedIntArguments& x )
	{
		return x.value;
	}

	template<>
	inline int64_t* range_end( repeatedIntArguments& x )
	{
		return x.value+x.size;
	}

	template<>
	inline const int64_t* range_end( const repeatedIntArguments& x )
	{
		return x.value+x.size;
	}


#endif // _KUKA_NANOPB_HPP_