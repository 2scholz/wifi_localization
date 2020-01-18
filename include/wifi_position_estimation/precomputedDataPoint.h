//
// Created by scholz on 07.02.17.
//

#ifndef PROJECT_PRECOMPUTEDDATAPOINT_H
#define PROJECT_PRECOMPUTEDDATAPOINT_H

class Process;

/**
 * PrecomputedDataPoint struct
 * Holds a gaussian process and the precomputed mean and variance.
 */
struct PrecomputedDataPoint
{
  Process* gp_;
  double mean_;
  double variance_;
};

#endif //PROJECT_PRECOMPUTEDDATAPOINT_H
