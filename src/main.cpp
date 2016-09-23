#include <iostream>
#include "path_finding.h"

int FindPath(const int nStartX, const int nStartY,
             const int nTargetX, const int nTargetY,
             const unsigned char *pMap, const int nMapWidth, const int nMapHeight,
             int *pOutBuffer, const int nOutBufferSize) {
  return pathfinding::PathFinding().FindPath(nStartX, nStartY,
                                             nTargetX, nTargetY,
                                             pMap, nMapWidth, nMapHeight,
                                             pOutBuffer, nOutBufferSize);
}

int main(int argc, char **argv) {
  std::cout << "TEST\n";

  return 0;
}