#include "matchingMatrix.h"

namespace CalibrationDepthPose
{

MatchingMatrix::MatchingMatrix(int n)
  : matrix(BooleanMatrix::Zero(n, n))
{

}

std::ostream &operator <<(std::ostream &os, const MatchingMatrix &matrix)
{
  return os << matrix.matrix;
}

}
