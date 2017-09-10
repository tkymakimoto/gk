/*
 * gkfunctional.h
 *
 *  Created on: 2016/01/18
 *      Author: makitaku
 */

#ifndef GKFUNCTIONAL_H_
#define GKFUNCTIONAL_H_

namespace gk {

template<typename T1, typename T2>
struct product_type_definition {
	typedef void value_type;
};

#if __cplusplus >= 201103L

template<typename T1, typename T2>
using product=typename product_type_definition<T1,T2>::value_type;
#else
#endif

template<typename T1, typename T2>
struct quotient_type_definition {
	typedef void value_type;
};

#if __cplusplus >= 201103L
template<typename T1, typename T2>
using quotient = typename quotient_type_definition<T1,T2>::value_type;
#endif

#if __cplusplus >= 201103L
template<typename T1, typename T2>
product<T1, T2> multiply(const T1& x, const T2& y) {
	return x * y;
}

template<typename T1, typename T2>
product<T1, T2> operator*(const T1& x, const T2& y) {
	return x * y;
}

template<typename T1, typename T2, typename Result = product<T1, T2> >
struct multiplier {
	typedef T1 first_argument_type;
	typedef T2 second_argument_type;
	typedef Result result_type;

	Result operator()(const T1& x, const T2& y) const {
		return multiply(x, y);
	}
};

#else
/**
 * @brief A multiplication function object to multiply a @a T1 type
 * by a @a T2 type.
 *
 * @tparam T1 First argument type.
 * @tparam T2 Second argument type.
 * @tparam Result A result type to be computed.
 */
template<typename T1, typename T2, typename Result = typename product_type_definition<
T1, T2>::value_type> struct multiply {
	typedef T1 first_argument_type;
	typedef T2 second_argument_type;
	typedef Result result_type;

	/**
	 * @brief Multiplies @a x by @a y.
	 * @param x The value of @c T1.
	 * @param y The value of @c T2.
	 * @return
	 */
	Result operator()(const T1& x, const T2& y) const {
		return x * y;
	}
};
#endif

#if __cplusplus >= 201103L
template<typename T1, typename T2>
quotient<T1, T2> devide(const T1& x, const T2& y) {
	return x / y;
}

template<typename T1, typename T2>
quotient<T1, T2> operator/(const T1& x, const T2& y) {
	return x / y;
}

#else
/**
 * @brief A division function object to divide a @a T1 type by a @a T2 type.
 *
 * @tparam T1 First argument type.
 * @tparam T2 Second argument type.
 * @tparam Result A result type to be computed.
 */
template<typename T1, typename T2, typename Result = typename quotient_type_definition<T1,
T2>::value_type> struct divides {
	typedef T1 first_argument_type;
	typedef T2 second_argument_type;
	typedef Result result_type;

	Result operator()(const T1& x, const T2& y) const {
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
