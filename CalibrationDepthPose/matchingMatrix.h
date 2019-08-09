#ifndef MATCHINGMATRIX_H
#define MATCHINGMATRIX_H

#include <Eigen/Dense>
#include <iostream>

namespace CalibrationDepthPose
{

using BooleanMatrix = Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic>;

class MatchingMatrix
{
public:
  MatchingMatrix(int n);

  void addMatch(int a, int b, bool reciprocal = false) {
    matrix(a, b) = true;
    matrix(b, a) |= reciprocal;
  }

  void erase() {
    matrix = BooleanMatrix::Zero(matrix.rows(), matrix.cols());
  }

  void setMatrix(BooleanMatrix const& mat) {
    if (mat.rows() == matrix.rows() && mat.cols() == matrix.cols()) {
      matrix = mat;
    } else {
      std::cerr << "setMatrix(mat): mat has not the correct dimension" << std::endl;
    }
  }

  bool operator()(int i, int j) const {
    return matrix(i, j);
  }

private:
  BooleanMatrix matrix;
};

}

#endif // MATCHINGMATRIX_H
