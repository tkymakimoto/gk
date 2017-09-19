/*
 * gkfunctional.h
 *
 *  Created on: 2016/01/18
 *      Author: makitaku
 */

#ifndef GKFUNCTIONAL_H_
#define GKFUNCTIONAL_H_

namespace gk {

template<typename _T1, typename _T2>
struct product_type_definition {
	typedef void value_type;
};

#if __cplusplus >= 201103L

/**
 * @tparam __T1
 */
template<typename _T1, typename _T2>
using product=typename product_type_definition<_T1,_T2>::value_type;
#else
#endif

template<typename _T1, typename _T2>
struct quotient_type_definition {
	typedef void value_type;
};

#if __cplusplus >= 201103L
template<typename _T1, typename _T2>
using quotient = typename quotient_type_definition<_T1,_T2>::value_type;
#endif

#if __cplusplus >= 201103L
template<typename _T1, typename _T2>
product<_T1, _T2> multiply(const _T1& x, const _T2& y) {
	return x * y;
}

template<typename _T1, typename _T2>
product<_T1, _T2> operator*(const _T1& x, const _T2& y) {
	return x * y;
}

template<typename _T1, typename _T2, typename Result = product<_T1,
		_T2> >
struct multiplier {
	typedef _T1 first_argument_type;
	typedef _T2 second_argument_type;
	typedef Result result_type;

	Result operator()(const _T1& x, const _T2& y) const {
		return multiply(x, y);
	}
};

#else
/**
 * @brief A multiplication function object to multiply a @a _T1 type
 * by a @a _T2 type.
 *
 * @tparam _T1 First argument type.
 * @tparam _T2 Second argument type.
 * @tparam Result A result type to be computed.
 */
template<typename _T1, typename _T2, typename Result = typename product_type_definition<
_T1, _T2>::value_type> struct multiply {
	typedef _T1 first_argument_type;
	typedef _T2 second_argument_type;
	typedef Result result_type;

	/**
	 * @brief Multiplies @a x by @a y.
	 * @param x The value of @c _T1.
	 * @param y The value of @c _T2.
	 * @return
	 */
	Result operator()(const _T1& x, const _T2& y) const {
		return x * y;
	}
};
#endif

#if __cplusplus >= 201103L
template<typename _T1, typename _T2>
quotient<_T1, _T2> devide(const _T1& x, const _T2& y) {
	return x / y;
}

template<typename _T1, typename _T2>
quotient<_T1, _T2> operator/(const _T1& x, const _T2& y) {
	return x / y;
}

#else
/**
 * @brief A division function object to divide a @a _T1 type by a @a _T2 type.
 *
 * @tparam _T1 First argument type.
 * @tparam _T2 Second argument type.
 * @tparam Result A result type to be computed.
 */
template<typename _T1, typename _T2, typename Result = typename quotient_type_definition<_T1,
_T2>::value_type> struct divides {
	typedef _T1 first_argument_type;
	typedef _T2 second_argument_type;
	typedef Result result_type;

	Result operator()(const _T1& x, const _T2& y) const {
		return x / y;
	}
};
#endif

/*
 * Specialized structures.
 */

/**
 * @brief A specialized structure of a result that
 */
template<>
struct product_type_definition<float, float> {
	typedef float value_type;
};

template<>
struct product_type_definition<double, double> {
	typedef double value_type;
};

template<>
struct product_type_definition<long double, long double> {
	typedef long double value_type;
};

template<>
struct product_type_definition<float, double> {
	typedef float value_type;
};

template<>
struct product_type_definition<double, float> {
	typedef float value_type;
};

template<>
struct product_type_definition<double, long double> {
	typedef double value_type;
};

template<>
struct product_type_definition<long double, float> {
	typedef float value_typ;
};

template<>
struct product_type_definition<long double, double> {
	typedef double value_type;
};

template<>
struct quotient_type_definition<float, float> {
	typedef float value_type;
};

template<>
struct quotient_type_definition<double, double> {
	typedef double value_type;
};

template<>
struct quotient_type_definition<long double, long double> {
	typedef long double value_type;
};

template<>
struct quotient_type_definition<float, double> {
	typedef float value_type;
};

template<>
struct quotient_type_definition<double, float> {
	typedef float value_type;
};

template<>
struct quotient_type_definition<float, long double> {
	typedef float value_type;
};

template<>
struct quotient_type_definition<double, long double> {
	typedef double value_type;
};

template<>
struct quotient_type_definition<long double, float> {
	typedef float value_type;
};

template<>
struct quotient_type_definition<long double, double> {
	typedef double value_type;
};

}  // namespace gk

#endif /* GKFUNCTIONAL_H_ */
