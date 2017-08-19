/*
 * bspline.h
 *
 *  Created on: 2014/07/16
 *      Author: makitaku
 */

#ifndef BSPLINE_BSPLINE_H_
#define BSPLINE_BSPLINE_H_

#include <vector>
#include <utility>
#include <algorithm>
#include <numeric>

#include "../gkvector.h"
#include "../gkaabb.h"
#include "basis.h"
#include "../algorithm/kernel.h"

namespace gk {

/**
 * @brief B-spline (Basis spline).
 *
 * @tparam Vector Type of a control point.
 * @tparam Parameter Type of a parameter.
 *
 * @author Takuya Makimoto
 * @date 2015
 */
template<typename Vector, typename Parameter>
class bspline: public geometry<free_curve_tag, Vector> {
public:
	typedef Vector vector_type;
	typedef bspl::knotvector<Parameter> knotvector_type;
	typedef std::vector<Vector> control_points;

public:

	/**
	 * @brief Default constructor.
	 */
	bspline() :
			T_(), Q_() {
	}

	/**
	 * @brief Copy constructor.
	 * @param other
	 */
	bspline(const bspline& other) :
			T_(other.T_), Q_(other.Q_) {

	}

	/**
	 *
	 * @param T A knot vector.
	 * @param Q An object of control points.
	 */
	bspline(const knotvector_type& T, const control_points& Q) :
			T_(T), Q_(Q) {
	}

	/**
	 * @brief
	 * @tparam KnotInputIterator
	 * @tparam VectorInputIterator
	 * @param T_first
	 * @param T_last
	 * @param Q_first
	 * @param Q_last
	 */
	template<typename KnotInputIterator, typename VectorInputIterator>
	bspline(KnotInputIterator T_first, KnotInputIterator T_last,
			VectorInputIterator Q_first, VectorInputIterator Q_last) :
			T_(T_first, T_last), Q_(Q_first, Q_last) {

	}

	/**
	 * @brief Destructor.
	 */
	~bspline() {
	}

	/**
	 * @brief Returns the degree of this B-spline.
	 * @return
	 */
	size_t degree() const {
		return this->degree_();
	}

	/**
	 * @brief Returns the knot vector.
	 * @return
	 */
	const knotvector_type& knot_vector() const {
		return this->T_;
	}

	/**
	 * @brief Returns the control points.
	 * @return
	 */
	const control_points& controls() const {
		return this->Q_;
	}

	/**
	 * @brief Returns the mutable control points.
	 * @return
	 */
	control_points& controls() {
		return this->Q_;
	}

	std::pair<Parameter, Parameter> domain() const {
		return bspl::domain(this->degree_(), this->T_.begin(), this->T_.end());
	}

	void insert(const Parameter& t) {
		this->insert_knot_(t);
	}

	/**
	 * @brief Subdivides this at a parameter @a t.
	 *
	 * @param t Parameter to subdivide.
	 * @param selection
	 *
	 * @return The other.
	 */
	bspline subdivide(const Parameter& t, gkselection selection = GK::Upper) {
		const std::size_t degree = this->degree_();
		const std::size_t order = degree + 1;

		if (t < this->T_[degree] || t > this->T_[this->Q_.size()]) {
			return bspline();
		}

		for (std::size_t i = 0; i < order; ++i) {
			this->insert_knot_(t);
		}

		typedef typename knotvector_type::const_iterator T_const_iterator;
		T_const_iterator first = this->T_.begin() + order;
		T_const_iterator end = this->T_.begin() + this->Q_.size() /*+ order*/;

		T_const_iterator position = std::upper_bound(first, end, t);

		const typename std::iterator_traits<T_const_iterator>::difference_type d =
				std::distance(this->T_.begin(), position);

		if (selection == GK::Upper) {
			const knotvector_type other_knotvector(position - order,
					this->T_.end());
			const control_points other_control_points(
					this->Q_.begin() + d - order, this->Q_.end());

			this->T_.erase(position, this->T_.end());
			this->Q_.erase(this->Q_.begin() + d - order, this->Q_.end());

			return bspline(other_knotvector, other_control_points);

		} else {
			const knotvector_type other_knotvector(this->T_.begin(), position);
			const control_points other_control_points(this->Q_.begin(),
					this->Q_.begin() + d - order);

			this->T_.erase(this->T_.begin(), position - order);
			this->Q_.erase(this->Q_.begin(), this->Q_.begin() + d - order);

			return bspline(other_knotvector, other_control_points);
		}
	}

//	/**
//	 * @brief
//	 * @param selection
//	 * @return
//	 */
//	bspline subdivide(gkselection selection = GK::Upper) {
//		const Parameter t_max = this->T_[this->Q_.size()];
//		const Parameter t_min = this->T_[this->compute_degree_(this->T_.size(),
//				this->Q_.size())];
//
//		return subdivide(0.5 * (t_max + t_min), selection);
//	}

//	void derivatise() {
//		const size_t degree = this->compute_degree_(this->T_.size(),
//				this->Q_.size());
//		const parameter Zero = parameter(GK_FLOAT_ZERO);
//
//		for (size_t i = 0; i < this->Q_.size() - 1; ++i) {
//			const parameter denominator = this->T_[i + degree + 1]
//					- this->T_[i + 1];
//			this->Q_[i] =
//					(denominator == Zero) ?
//							Zero :
//							degree * (this->Q_[i + 1] - this->Q_[i])
//									/ denominator;
//		}
//
//		this->Q_.pop_back();
//
//		this->T_.pop_back();
//		this->T_.erase(this->T_.begin());
//	}

	Vector operator()(const Parameter& t) const {
		std::vector < Parameter > N(this->Q_.size());
		bspl::basis_function(degree_(), this->T_.begin(), this->T_.end(), t,
				N.begin());

		return std::inner_product(this->Q_.begin(), this->Q_.end(), N.begin(),
				Vector());
	}

	template<typename InputIterator, typename OutputIterator>
	OutputIterator operator()(InputIterator first, InputIterator last,
			OutputIterator result) const {

		return result;
	}

	bspline& operator=(const bspline& rhs) {
		if (&rhs == this) {
			return *this;
		}

		this->T_ = rhs.T_;
		this->Q_ = rhs.Q_;

		return *this;
	}

private:
	knotvector_type T_;
	control_points Q_;

private:
	size_t degree_() const {
		return bspl::degree(this->T_.size(), this->Q_.size());
	}

	void insert_knot_(const Parameter& t) {
		const size_t degree = this->degree_();

		const std::pair<Parameter, Parameter> D = bspl::domain(degree,
				this->T_.begin(), this->T_.end());
		if (t < D.first || t > D.second) {
			return;
		}

		// section number
		const size_t k = knot_segment(this->T_, degree, t);

		this->Q_.insert(this->Q_.begin() + k, this->Q_[k]);

		for (size_t j = k; j > k - degree; --j) {
			const Parameter denominator = this->T_[j + degree] - this->T_[j];
			const Parameter alpha =
					(denominator == Parameter(GK_FLOAT_ZERO)) ?
							Parameter(GK_FLOAT_ZERO) :
							(t - this->T_[j]) / (denominator);

			this->Q_[j] = alpha * this->Q_[j]
					+ (Parameter(GK_FLOAT_ONE) - alpha) * this->Q_[j - 1];
		}

		this->T_.insert(t);
	}

};

template<typename Vector, typename Parameter>
std::pair<bspline<Vector, Parameter>, bspline<Vector, Parameter> > subdivide(
		const bspline<Vector, Parameter>& r, const Parameter& t) {
	bspline<Vector, Parameter> p = r;
	const bspline<Vector, Parameter> q = p.subdivide(t, GK::Upper);
	return std::make_pair(p, q);
}

template<typename Vector, typename Parameter>
bspline<Vector, Parameter> subdivide(const bspline<Vector, Parameter>& x,
		const Parameter& a, const Parameter& b) {
	bspline<Vector, Parameter> y = x;
	y.subdivide(a, GK::Lower);
	y.subdivide(b, GK::Upper);

	return y;
}

template<typename Vector, typename KnotVector, typename Parameter,
		typename OutputIterator>
OutputIterator subdivide(const bspline<Vector, KnotVector>& x,
		const Parameter& t, OutputIterator result) {
	bspline<Vector, Parameter> y = x;
	*result = y.subdivide(t, GK::Lower);
	++result;
	*result = y;
	return result;
}

template<typename Vector, typename Parameter>
std::pair<Vector, Vector> linearize(const bspline<Vector, Parameter>& x,
		typename vector_traits<Vector>::value_type& max_distance) {

	max_distance = typename vector_traits<Vector>::value_type(
	GK_FLOAT_ZERO);
	const std::pair<Parameter, Parameter> D = domain(x);
	bspline<Vector, Parameter> y = x;
	y.subdivide(D.first, GK::Lower);
	y.subdivide(D.second, GK::Upper);

	std::pair<Vector, Vector> result = std::make_pair(y.controls().front(),
			y.controls().back());
	typedef typename bspline<Vector, Parameter>::control_points controls;
	for (typename controls::iterator p = y.controls().begin();
			p != y.controls().end(); ++p) {
		const Vector v = alg::nearest_to_line(result.first,
				normalize(result.second - result.first), *p);

		const typename vector_traits<Vector>::value_type d = norm(v - *p);
		max_distance = std::max(max_distance, d);
	}

	return result;
}

template<typename Vector, typename Parameter>
aabb<Vector> boundary(const bspline<Vector, Parameter>& x) {
	return aabb<Vector>(x.controls().begin(), x.controls().end());
}

template<typename Vector, typename Parameter>
bspline<Vector, Parameter> derivatatise(const bspline<Vector, Parameter>& r) {

	typedef typename bspline<Vector, Parameter>::knotvector_type knotvector;
	const Parameter Zero(GK_FLOAT_ZERO);

	const size_t degree = r.degree();
	const knotvector T = r.knot_vector();
	const typename bspline<Vector, Parameter>::control_points Q = r.controls();

	typename bspline<Vector, Parameter>::control_points P(Q.size() - 1);
	for (size_t i = 0; i < P.size(); ++i) {
		const Parameter dt = T[i + degree + 1] - T[i + 1];
		P[i] = (dt == Zero) ? Zero : degree * (Q[i + 1] - Q[i]) / dt;
	}

	typedef typename knotvector::const_iterator T_const_iterator;
	T_const_iterator U_first = T.begin();
	std::advance(U_first, 1);

	T_const_iterator U_last = T.end();
	std::advance(U_last, -1);

	return bspline<Vector, Parameter>(U_first, U_last, P.begin(), P.end());
}

} // namespace gk

#endif /* BSPLINE_BSPLINE_H_ */
