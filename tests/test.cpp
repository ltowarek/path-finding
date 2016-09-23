#include "gtest/gtest.h"
#include "path_finding.h"

TEST(PathFinding, FindPathPositive) {
  unsigned char pMap[] = {1, 1, 1, 1, 0, 1, 0, 1, 0, 1, 1, 1};
  int pOutBuffer[12];
  EXPECT_EQ(pathfinding::PathFinding().FindPath(0, 0, 1, 2, pMap, 4, 3, pOutBuffer, 12), 3);
  EXPECT_EQ(pOutBuffer[0], 1);
  EXPECT_EQ(pOutBuffer[1], 5);
  EXPECT_EQ(pOutBuffer[2], 9);
}

TEST(PathFinding, FindPathNegative) {
  unsigned char pMap[] = {0, 0, 1, 0, 1, 1, 1, 0, 1};
  int pOutBuffer[7];
  EXPECT_EQ(pathfinding::PathFinding().FindPath(2, 0, 0, 2, pMap, 3, 3, pOutBuffer, 7), -1);
}
