#include "matchingMatrix.h"

namespace CalibrationDepthPose
{

MatchingMatrix::MatchingMatrix(int n)
  : matrix(BooleanMatrix::Zero(n, n))
{

}

}
