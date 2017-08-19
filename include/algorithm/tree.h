/*
 * tree.h
 *
 *  Created on: 2015/10/14
 *      Author: makitaku
 */

#ifndef ALGORITHM_TREE_H_
#define ALGORITHM_TREE_H_

namespace gk {

template<typename T, std::size_t NumberOfChild = 2>
struct node {

	node* children[NumberOfChild];

	node operator[](size_t n) const {
	}
};

template<typename Node>
struct tree {
	Node root;
};

}  // namespace gk

#endif /* ALGORITHM_TREE_H_ */
