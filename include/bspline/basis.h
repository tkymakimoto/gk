/*
 * basis.h
 *
 *  Created on: 2015/06/12
 *      Author: makitaku
 */

#ifndef BSPLINE_BASIS_H_
#define BSPLINE_BASIS_H_

#include <gkdef.h>
#include <vector>
#include <algorithm>

namespace gk {

/**
 * Namespace about B-spline.
 */
namespace bspl {

/**
 *
 * @param knot_vector_size
 * @param control_points_size
 * @return
 */
size_t degree(size_t knot_vector_size, size_t control_points_size) {
	return knot_vector_size - control_points_size - 1;
}

/**
 *
 * @param degree
 * @param knot_vector_size
 * @return
 */
size_t control_points_size(size_t degree, size_t knot_vector_size) {
	return knot_vector_size - degree - 1;
}

/**
 *
 * @param degree
 * @param control_points_size
 * @return
 */
size_t knot_vector_size(size_t degree, size_t control_points_size) {
	return control_points_size + degree + 1;
}

template<typename InputIterator>
std::pair<typename std::iterator_traits<InputIterator>::value_type,
		typename std::iterator_traits<InputIterator>::value_type> domain(
		std::size_t degree, InputIterator first, InputIterator last) {

	InputIterator s = first;
	std::advance(s, degree);

	InputIterator t = last;
	std::advance(t, -degree - 1);
	return std::make_pair(*s, *t);
}

/**
 * @brief Knot vector.
 *
 * @author Takuya Makimoto
 * @date 2016/01/07
 */
template<typename T>
class knotvector {
public:
	typedef T value_type;
	typedef std::vector<T> container_type;

	typedef typename container_type::const_reference const_reference;
	typedef typename container_type::const_iterator const_iterator;
	typedef typename container_type::const_reverse_iterator const_reverse_iterator;

public:
	knotvector() :
			X_() {
	}

	knotvector(const knotvector& other) :
			X_(other.X_) {
	}

	knotvector(std::size_t size) :
			X_(size) {
	}

	template<typename InputIterator>
	knotvector(InputIterator first, InputIterator last) :
			X_(first, last) {
		std::stable_sort(this->X_.begin(), this->X_.end());
	}

	~knotvector() {
	}

	bool empty() const {
		return this->X_.empty();
	}

	std::size_t size() const {
		return this->X_.size();
	}

	const_reference front() const {
		return this->X_.front();
	}

	const_reference back() const {
		return this->X_.back();
	}

	const_iterator begin() const {
		return this->X_.begin();
	}

	const_iterator end() const {
		return this->X_.end();
	}

	const_reverse_iterator rbegin() const {
		return this->X_.rbegin();
	}

	const_reverse_iterator rend() const {
		return this->X_.rend();
	}

	const_iterator insert(const value_type& t) {
		typename container_type::iterator p = std::upper_bound(this->X_.begin(),
				this->X_.end(), t);
		return this->X_.insert(p, t);
	}

	void insert(const T& t, std::size_t multiplicity) {
		typename container_type::iterator p = std::upper_bound(this->X_.begin(),
				this->X_.end(), t);
		this->X_.insert(p, multiplicity, t);
	}

	template<class InputIterator>
	void insert(InputIterator first, InputIterator last) {
		this->X_.insert(this->X_.end(), first, last);
		std::stable_sort(this->X_.begin(), this->X_.end());
	}

	/**
	 * @brief Erases the element at @a position.
	 * @param position
	 * @return
	 */
	const_iterator erase(const_iterator position) {
		return this->X_.erase(this->X_.begin() + (position - this->X_.begin()));
	}

	/**
	 * @brief Erases the elements between @a first and the previous position of
	 * @a last, [first, last).
	 * @param first
	 * @param last
	 * @return
	 */
	const_iterator erase(const_iterator first, const_iterator last) {
		return this->X_.erase(this->X_.begin() + (first - this->X_.begin()),
				this->X_.begin() + (last - this->X_.begin()));
	}

	value_type operator[](std::size_t n) const {
		return this->X_[n];
	}

	knotvector& operator=(const knotvector& rhs) {
		if (&rhs == this) {
			return *this;
		}

		this->X_ = rhs.X_;
		return *this;
	}

private:
	container_type X_;
};

/**
 *
 * @param degree
 * @param first
 * @param last
 * @param t
 * @return
 */
template<typename RandomAccessIterator, typename Parameter>
RandomAccessIterator segment_of(size_t degree, RandomAccessIterator first,
		RandomAccessIterator last, const Parameter& t) {
	const size_t order = degree + 1;
	const size_t n = std::distance(first, last) - order;
	RandomAccessIterator p = std::upper_bound(first + order, first + n, t);
	--p;
	return p;
}

/**
 * @brief Compute basis function values at a parameter @a t.
 * @param degree Degree of the basis function.
 * @param T Object of knot vector.
 * @param t Parameter.
 * @param N Beginning iterator of the basis function.
 * @param dorder Derivative order.
 *
 * @return The last iterator of the basis function.
 */
template<typename InputRandomAccessIterator, typename Parameter,
		typename OutputRandomAccessIterator>
OutputRandomAccessIterator basis_function(size_t degree,
		InputRandomAccessIterator first, InputRandomAccessIterator last,
		const Parameter& t, OutputRandomAccessIterator N, size_t dorder = 0) {

	const Parameter Zero = Parameter(GK_FLOAT_ZERO);
	const Parameter One = Parameter(GK_FLOAT_ONE);

	const size_t order = degree + 1;
	const size_t knot_size = std::distance(first, last);
	const size_t control_size = knot_size - order;

	const size_t segment = std::distance(first,
			std::upper_bound(first + order, first + control_size, t)) - 1;
	InputRandomAccessIterator T = first;

	/*
	 * Initialization.
	 */
	std::fill_n(N, control_size, Zero);

	N[segment] = One;

	if (degree == 0) {
		return ++N;
	}

	typedef std::vector<Parameter> coeff_t;
	for (size_t k = 2; k <= order - dorder; ++k) {

		coeff_t alpha(control_size);

		for (size_t j = segment - k + 2; j <= segment; j++) {
			const Parameter dt = T[j + k - 1] - T[j];
			alpha[j] = (dt == Zero) ? Zero : (t - T[j]) / dt;
		}

		N[segment - k + 1] = (One - alpha[segment - k + 2])
				* N[segment - k + 2];
		for (size_t j = segment - k + 2; j < segment; j++) {
			N[j] = alpha[j] * N[j] + (One - alpha[j + 1]) * N[j + 1];
		}
		N[segment] = alpha[segment] * N[segment];
	}

//	return N + control_size;

	if (dorder == 0) {
		return N + control_size;
	} else {

		for (size_t k = order - dorder + 1; k <= order; ++k) {
			const size_t p = k - 1; // degree

			coeff_t beta(control_size);

			for (size_t j = segment - k + 2; j <= segment; ++j) {
				const Parameter denominator = T[j + p] - T[j];
				beta[j] = (denominator == Zero) ? Zero : One / denominator;
			}

			N[segment - k + 1] = -beta[segment - k + 2] * N[segment - k + 2];
			N[segment - k + 1] *= p;
			for (std::size_t j = segment - k + 2; j < segment; ++j) {
				N[j] = p * (beta[j] * N[j] - beta[j + 1] * N[j + 1]);
			}
			N[segment] = p * beta[segment] * N[segment];
		}

		return N + control_size;
	}
}

}  // namespace bspl

} // namespace gk

#endif /* BSPLINE_BASIS_H_ */
