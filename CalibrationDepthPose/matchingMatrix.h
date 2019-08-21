#ifndef MATCHINGMATRIX_H
#define MATCHINGMATRIX_H

#include <Eigen/Dense>
#include <iostream>

namespace CalibrationDepthPose
{

struct PairsMatchingStrategy
{
  enum class Strategy
  {
    ALL,
    N_CONSECUTIVES_WITH_LOOP,
    N_CONSECUTIVES_NO_LOOP,
  } strategy = Strategy::N_CONSECUTIVES_WITH_LOOP;

  int nbNeighbours = 1;
};

std::ostream& operator <<(std::ostream& os, PairsMatchingStrategy const& strategy);

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

  unsigned int getSize() const {
    return matrix.cols();
  }

  void setFromPredefinedMatchingStrategy(PairsMatchingStrategy const& strategy);

  std::vector<std::pair<size_t, size_t> > getPairs() const;

  friend std::ostream& operator <<(std::ostream& os, MatchingMatrix const& matrix);


private:
  BooleanMatrix matrix;
};

std::ostream& operator <<(std::ostream& os, MatchingMatrix const& matrix);

}

#endif // MATCHINGMATRIX_H
