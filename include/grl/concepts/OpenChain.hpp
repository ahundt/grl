#ifndef _OPEN_CHAIN_HPP_
#define _OPEN_CHAIN_HPP_

// Copyright (c) 2015 Andrew Hundt, Baltimore, MD
// Copyright (c) 2008-2012 Bruno Lalande, Paris, France.
// Copyright (c) 2008-2012 Barend Gehrels, Amsterdam, the Netherlands.
// Copyright (c) 2009-2012 Mateusz Loskot, London, UK.


#include <boost/concept_check.hpp>
#include <boost/range/concepts.hpp>
#include <boost/type_traits/remove_const.hpp>

#include <boost/geometry/core/access.hpp>
#include <boost/geometry/core/mutable_range.hpp>
#include <boost/geometry/core/point_type.hpp>

#include <boost/geometry/geometries/concepts/point_concept.hpp>

namespace grl { namespace concept {
/*!
\brief OpenChain concept
\ingroup concepts
\par Formal definition:
The OpenChain concept is defined as following:
- there must be a specialization of traits::tag defining linestring_tag as type
- it must behave like a Boost.Range
- it must implement a std::back_insert_iterator
    - either by implementing push_back
    - or by specializing std::back_insert_iterator
\note to fulfill the concepts, no traits class has to be specialized to
define the joint type.
\par Example:
A custom linestring, defining the necessary specializations to fulfill to the concept.
Suppose that the following linestring is defined:
\dontinclude doxygen_5.cpp
\skip custom_linestring1
\until };
It can then be adapted to the concept as following:
\dontinclude doxygen_5.cpp
\skip adapt custom_linestring1
\until }}
\note
- Open chain refers to a single linear series of states.
The best example of this is an open kinematic chain,
such as a robot arm consisting of a series of revolute
and prismatic joints.
- There is also the registration macro BOOST_GEOMETRY_REGISTER_LINESTRING
- For registration of std::vector<P> (and deque, and list) it is enough to
include the header-file geometries/adapted/std_as_linestring.hpp. That registers
a vector as a linestring (so it cannot be registered as a linear ring then,
in the same source code).

\see Based on Boost.Geometry Linestring concept http://www.boost.org/doc/libs/release/libs/geometry/doc/html/geometry/reference/concepts/concept_linestring.html
    
*/
class OpenChain {

#ifndef DOXYGEN_NO_CONCEPT_MEMBERS
    typedef typename boost::geometry::concept::point_type<Geometry>::type point_type;

    BOOST_CONCEPT_ASSERT( (boost::geometry::concept::Point<point_type>) );
    BOOST_CONCEPT_ASSERT( (boost::RandomAccessRangeConcept<Geometry>) );

public :

    BOOST_CONCEPT_USAGE(Linestring)
    {
        Geometry* ls = 0;
        traits::clear<Geometry>::apply(*ls);
        traits::resize<Geometry>::apply(*ls, 0);
        point_type* point = 0;
        traits::push_back<Geometry>::apply(*ls, *point);
    }
#endif
};

class ConstOpenChain {
#ifndef DOXYGEN_NO_CONCEPT_MEMBERS
    typedef typename boost::geometry::concept::point_type<Geometry>::type point_type;

    BOOST_CONCEPT_ASSERT( (concept::ConstPoint<point_type>) );
    //BOOST_CONCEPT_ASSERT( (boost::RandomAccessRangeConcept<Geometry>) );
    // Relaxed the concept.
    BOOST_CONCEPT_ASSERT( (boost::ForwardRangeConcept<Geometry>) );


public :

    BOOST_CONCEPT_USAGE(ConstLinestring)
    {
    }
#endif
};

}
}}
#endif

