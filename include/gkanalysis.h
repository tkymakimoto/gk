/*
 * gkanalysis.h
 *
 *  Created on: 2017/09/15
 *      Author: tmakimoto
 */

#ifndef GKANALYSIS_H_
#define GKANALYSIS_H_

#include "gkgeometry.h"

namespace gk {

template<typename _Geometry1, typename _Geometry2>
struct distance_definition_type {
	typedef geometry_traits<_Geometry1> traits1;
	typedef geometry_traits<_Geometry2> traits2;

	check_same_dimension<traits1::Dimension, traits2::Dimension> ;

	typedef typename check_same_type<typename traits1::value_type,
			typename traits2::value_type>::value_type value_type;

};

#if __cplusplus >= 201103L

template<typename _Geometry1, typename _Geometry2>
using distance_type=distance_definition_type<_Geometry1,_Geometry2>::value_type;

#elif
#endif

namespace inner {
template<typename _Point1, typename _Point2>
distance_type<_Point1, _Point2> distance(const _Point1& a,
		const _Point2& b, point_tag, point_tag,
		dimension_tag<GK::GK_2D>) {

	return std::sqrt(a[GK::X] * b[GK::X] + a[GK::Y] * b[GK::Y]);
}

}  // namespace inner

template<typename _Geometry1, typename _Geometry2>
distance_type<_Geometry1, _Geometry2> distance(const _Geometry1& a,
		const _Geometry2& b) {

}

}  // namespace gk

#endif /* GKANALYSIS_H_ */
