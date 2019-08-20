#include "matchingMatrix.h"

namespace CalibrationDepthPose
{

MatchingMatrix::MatchingMatrix(int n)
  : matrix(BooleanMatrix::Zero(n, n))
{

}

void MatchingMatrix::setFromPredefinedMatchingStrategy(const PairsMatchingStrategy &strategy)
{
  if (strategy.strategy == PairsMatchingStrategy::Strategy::ALL)
  {
    this->setMatrix(BooleanMatrix::Ones(this->getSize(), this->getSize()));
  }
  else if (strategy.strategy == PairsMatchingStrategy::Strategy::N_CONSECUTIVES_WITH_LOOP)
  {
    for (size_t i = 0; i < this->getSize(); ++i)
    {
      for (int j = 1; j <= strategy.nbNeighbours; ++j)
      {
        this->addMatch(i, (i + j) % this->getSize(), true);
      }
    }
  }
  else if (strategy.strategy == PairsMatchingStrategy::Strategy::N_CONSECUTIVES_NO_LOOP)
  {
    for (size_t i = 0; i < this->getSize(); ++i)
    {
      for (int j = 1; j <= strategy.nbNeighbours && i + j < this->getSize(); ++j)
      {
        this->addMatch(i, i + j, true);
      }
    }
  }
}

std::ostream &operator <<(std::ostream &os, const MatchingMatrix &matrix)
{
  return os << matrix.matrix;
}

std::ostream &operator <<(std::ostream &os, const PairsMatchingStrategy &strategy)
{
  if (strategy.strategy == PairsMatchingStrategy::Strategy::ALL)
  {
    os << "all vs all";
  }
  else if (strategy.strategy == PairsMatchingStrategy::Strategy::N_CONSECUTIVES_WITH_LOOP)
  {
    os << strategy.nbNeighbours << " consecutive neighbours with loop";
  }
  else if (strategy.strategy == PairsMatchingStrategy::Strategy::N_CONSECUTIVES_NO_LOOP)
  {
    os << strategy.nbNeighbours << " consecutive neighbours without loop";
  }
  return os;
}

}
