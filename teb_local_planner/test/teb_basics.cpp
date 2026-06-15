#include <gtest/gtest.h>

#include <teb_local_planner/distance_calculations.h>
#include <teb_local_planner/graph_search.h>
#include <teb_local_planner/homotopy_class_planner.h>
#include <teb_local_planner/obstacles.h>
#include <teb_local_planner/timed_elastic_band.h>

namespace
{

teb_local_planner::Point2dContainer makeSquare(double min_x, double min_y, double max_x, double max_y)
{
  teb_local_planner::Point2dContainer square;
  square.push_back(Eigen::Vector2d(min_x, min_y));
  square.push_back(Eigen::Vector2d(max_x, min_y));
  square.push_back(Eigen::Vector2d(max_x, max_y));
  square.push_back(Eigen::Vector2d(min_x, max_y));
  return square;
}

const Eigen::Vector2d& identityPosition(const Eigen::Vector2d& position)
{
  return position;
}

} // namespace

TEST(TEBDistance, separatedPolygonsHavePositiveDistance)
{
  const teb_local_planner::Point2dContainer first = makeSquare(0.0, 0.0, 1.0, 1.0);
  const teb_local_planner::Point2dContainer second = makeSquare(1.5, 0.0, 2.5, 1.0);

  EXPECT_NEAR(0.5, teb_local_planner::distance_polygon_to_polygon_2d(first, second), 1e-9);
}

TEST(TEBDistance, overlappingPolygonsHaveNegativePenetrationDistance)
{
  const teb_local_planner::Point2dContainer first = makeSquare(0.0, 0.0, 1.0, 1.0);
  const teb_local_planner::Point2dContainer second = makeSquare(0.75, 0.0, 1.75, 1.0);

  EXPECT_NEAR(-0.25, teb_local_planner::distance_polygon_to_polygon_2d(first, second), 1e-9);
}

TEST(TEBDistance, containedPolygonHasUsefulPenetrationGradient)
{
  const teb_local_planner::Point2dContainer outer = makeSquare(0.0, 0.0, 1.0, 1.0);
  const teb_local_planner::Point2dContainer centered = makeSquare(0.25, 0.25, 0.75, 0.75);
  const teb_local_planner::Point2dContainer shifted = makeSquare(0.35, 0.25, 0.85, 0.75);

  EXPECT_NEAR(-0.75, teb_local_planner::distance_polygon_to_polygon_2d(outer, centered), 1e-9);
  EXPECT_GT(teb_local_planner::distance_polygon_to_polygon_2d(outer, shifted),
            teb_local_planner::distance_polygon_to_polygon_2d(outer, centered));
}

TEST(TEBGraph, predictsOncomingPolygonEncounterInsideLocalHorizon)
{
  teb_local_planner::TebConfig cfg;
  cfg.robot.max_vel_x = 0.6;
  cfg.obstacles.min_obstacle_dist = 0.2;
  cfg.obstacles.dynamic_obstacle_inflation_dist = 0.6;
  cfg.hcp.max_number_classes = 3;
  cfg.hcp.obstacle_keypoint_offset = 0.1;
  cfg.robot_model.reset(new teb_local_planner::PolygonRobotFootprint(makeSquare(-0.4, -0.25, 0.4, 0.25)));

  teb_local_planner::ObstContainer obstacles;
  teb_local_planner::ObstaclePtr obstacle(
      new teb_local_planner::PolygonObstacle(makeSquare(3.1, -0.4, 3.9, 0.4)));
  obstacle->setCentroidVelocity(Eigen::Vector2d(-1.0, 0.0));
  obstacles.push_back(obstacle);

  teb_local_planner::HomotopyClassPlanner planner(cfg, &obstacles);
  teb_local_planner::lrKeyPointGraph graph(cfg, &planner);
  graph.createGraph(teb_local_planner::PoseSE2(-1.0, 0.0, 0.0),
                    teb_local_planner::PoseSE2(1.5, 0.0, 0.0),
                    cfg.obstacles.min_obstacle_dist, cfg.hcp.obstacle_heading_threshold, NULL);

  ASSERT_EQ(4u, boost::num_vertices(graph.graph_));
  EXPECT_NEAR(-0.2625, graph.graph_[1].pos.x(), 1e-9);
  EXPECT_NEAR(0.95, graph.graph_[1].pos.y(), 1e-9);
  EXPECT_NEAR(-0.2625, graph.graph_[2].pos.x(), 1e-9);
  EXPECT_NEAR(-0.95, graph.graph_[2].pos.y(), 1e-9);
}

TEST(TEBHcp, selectsForwardCandidateBeforeReverseCandidate)
{
  teb_local_planner::TebConfig cfg;
  cfg.obstacles.include_dynamic_obstacles = false;
  cfg.hcp.max_number_classes = 3;
  cfg.hcp.switching_blocking_period = 10.0;

  teb_local_planner::ObstContainer obstacles;
  obstacles.push_back(teb_local_planner::ObstaclePtr(new teb_local_planner::PointObstacle(1.0, 0.0)));
  teb_local_planner::HomotopyClassPlanner planner(cfg, &obstacles);

  std::vector<Eigen::Vector2d> reverse_path;
  reverse_path.push_back(Eigen::Vector2d(0.0, 0.0));
  reverse_path.push_back(Eigen::Vector2d(-0.2, 0.0));
  reverse_path.push_back(Eigen::Vector2d(1.0, 1.0));
  reverse_path.push_back(Eigen::Vector2d(2.0, 0.0));
  teb_local_planner::TebOptimalPlannerPtr reverse = planner.addAndInitNewTeb(
      reverse_path.begin(), reverse_path.end(), identityPosition, 0.0, 0.0, NULL);

  std::vector<Eigen::Vector2d> forward_path;
  forward_path.push_back(Eigen::Vector2d(0.0, 0.0));
  forward_path.push_back(Eigen::Vector2d(0.2, 0.0));
  forward_path.push_back(Eigen::Vector2d(1.0, -1.0));
  forward_path.push_back(Eigen::Vector2d(2.0, 0.0));
  teb_local_planner::TebOptimalPlannerPtr forward = planner.addAndInitNewTeb(
      forward_path.begin(), forward_path.end(), identityPosition, 0.0, 0.0, NULL);

  ASSERT_TRUE(reverse);
  ASSERT_TRUE(forward);
  EXPECT_EQ(forward, planner.selectBestTeb());
}

TEST(TEBBasic, autoResizeLargeValueAtEnd)
{
  double dt = 0.1;
  double dt_hysteresis = dt/3.;
  teb_local_planner::TimedElasticBand teb;
  
  teb.addPose(teb_local_planner::PoseSE2(0., 0., 0.));
  for (int i = 1; i < 10; ++i) {
    teb.addPoseAndTimeDiff(teb_local_planner::PoseSE2(i * 1., 0., 0.), dt);
  }
  // add a pose with a large timediff as the last one
  teb.addPoseAndTimeDiff(teb_local_planner::PoseSE2(10., 0., 0.), dt + 2*dt_hysteresis);

  // auto resize + test of the result
  teb.autoResize(dt, dt_hysteresis, 3, 100, false);
  for (int i = 0; i < teb.sizeTimeDiffs(); ++i) {
    ASSERT_LE(teb.TimeDiff(i), dt + dt_hysteresis + 1e-3) << "dt is greater than allowed: " << i;
    ASSERT_LE(dt - dt_hysteresis - 1e-3, teb.TimeDiff(i)) << "dt is less than allowed: " << i;
  }
}

TEST(TEBBasic, autoResizeSmallValueAtEnd)
{
  double dt = 0.1;
  double dt_hysteresis = dt/3.;
  teb_local_planner::TimedElasticBand teb;
  
  teb.addPose(teb_local_planner::PoseSE2(0., 0., 0.));
  for (int i = 1; i < 10; ++i) {
    teb.addPoseAndTimeDiff(teb_local_planner::PoseSE2(i * 1., 0., 0.), dt);
  }
  // add a pose with a small timediff as the last one
  teb.addPoseAndTimeDiff(teb_local_planner::PoseSE2(10., 0., 0.), dt - 2*dt_hysteresis);

  // auto resize + test of the result
  teb.autoResize(dt, dt_hysteresis, 3, 100, false);
  for (int i = 0; i < teb.sizeTimeDiffs(); ++i) {
    ASSERT_LE(teb.TimeDiff(i), dt + dt_hysteresis + 1e-3) << "dt is greater than allowed: " << i;
    ASSERT_LE(dt - dt_hysteresis - 1e-3, teb.TimeDiff(i)) << "dt is less than allowed: " << i;
  }
}

TEST(TEBBasic, autoResize)
{
  double dt = 0.1;
  double dt_hysteresis = dt/3.;
  teb_local_planner::TimedElasticBand teb;
  
  teb.addPose(teb_local_planner::PoseSE2(0., 0., 0.));
  for (int i = 1; i < 10; ++i) {
    teb.addPoseAndTimeDiff(teb_local_planner::PoseSE2(i * 1., 0., 0.), dt);
  }
  // modify the timediff in the middle and add a pose with a smaller timediff as the last one
  teb.TimeDiff(5) = dt + 2*dt_hysteresis;
  teb.addPoseAndTimeDiff(teb_local_planner::PoseSE2(10., 0., 0.), dt - 2*dt_hysteresis);

  // auto resize
  teb.autoResize(dt, dt_hysteresis, 3, 100, false);
  for (int i = 0; i < teb.sizeTimeDiffs(); ++i) {
    ASSERT_LE(teb.TimeDiff(i), dt + dt_hysteresis + 1e-3) << "dt is greater than allowed: " << i;
    ASSERT_LE(dt - dt_hysteresis - 1e-3, teb.TimeDiff(i)) << "dt is less than allowed: " << i;
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
