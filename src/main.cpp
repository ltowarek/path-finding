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
  unsigned char pMap[] = {1, 1, 1, 1, 0, 1, 0, 1, 0, 1, 1, 1};
  int pOutBuffer[12];
  auto path_size = FindPath(0, 0, 1, 2, pMap, 4, 3, pOutBuffer, 12);

  std::cout << "Path size: " << path_size << "\n";
  std::cout << "Path: ";
  for (int i = 0; i < path_size; ++i) {
    std::cout << pOutBuffer[i] << " ";
  }
  std::cout << "\n";

  return 0;
}