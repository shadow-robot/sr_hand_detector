/*
* Copyright 2020 Shadow Robot Company Ltd.
*
* This program is free software: you can redistribute it and/or modify it
* under the terms of the GNU General Public License as published by the Free
* Software Foundation version 2 of the License.
*
* This program is distributed in the hope that it will be useful, but WITHOUT
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
* more details.
*
* You should have received a copy of the GNU General Public License along
* with this program. If not, see <http://www.gnu.org/licenses/>.
*/

#include <gtest/gtest.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <utility>
#include <string>
#include "sr_hand_detector/sr_hand_detector.h"
#include "sr_hand_detector/sr_hand_autodetect.h"
#include "yaml-cpp/exceptions.h"


TEST(SrHandAutodetect, test_run_unimanual)
{
  std::string expected_command_suffix = " eth_port:=eth0 hand_serial:=1130 side:=right hand_type:=hand_e_plus";
  std::string hand_config_path = ros::package::getPath("sr_hand_detector") + "/test/config";
  std::string detected_hands_file = ros::package::getPath("sr_hand_detector") + "/test/config/test_unimanual.yaml";

  sr_hand_detector::SrHandAutodetect sr_hand_autodetect(detected_hands_file, hand_config_path);
  sr_hand_autodetect.run();
  ASSERT_EQ(sr_hand_autodetect.get_command_suffix(), expected_command_suffix);
}

TEST(SrHandAutodetect, test_run_unimanual_additional_params)
{
  std::string expected_command_suffix = " eth_port:=eth0 hand_serial:=634 side:=right hand_type:=hand_e";
  std::string expected_mapping_path = ros::package::getPath("sr_hand_detector") + "/rh_mapping.yaml";
  expected_command_suffix += " mapping_path:=" + expected_mapping_path;

  std::string hand_config_path = ros::package::getPath("sr_hand_detector") + "/test/config";
  std::string detected_hands_file = ros::package::getPath("sr_hand_detector") +
                                      "/test/config/test_unimanual_additional_params.yaml";

  sr_hand_detector::SrHandAutodetect sr_hand_autodetect(detected_hands_file, hand_config_path);
  sr_hand_autodetect.run();
  ASSERT_EQ(sr_hand_autodetect.get_command_suffix(), expected_command_suffix);
}

TEST(SrHandAutodetect, test_run_bimanual)
{
  std::string expected_command_suffix = " eth_port:=eth0_eth1 rh_serial:=634 lh_serial:=2346 hand_type:=hand_e";
  std::string expected_rh_mapping_path = ros::package::getPath("sr_hand_detector") + "/rh_mapping.yaml";
  std::string expected_lh_mapping_path = ros::package::getPath("sr_hand_detector") + "/lh_mapping.yaml";
  expected_command_suffix += " rh_mapping_path:=" + expected_rh_mapping_path;
  expected_command_suffix += " lh_mapping_path:=" + expected_lh_mapping_path;

  std::string hand_config_path = ros::package::getPath("sr_hand_detector") + "/test/config";
  std::string detected_hands_file = ros::package::getPath("sr_hand_detector") + "/test/config/test_bimanual.yaml";

  sr_hand_detector::SrHandAutodetect sr_hand_autodetect(detected_hands_file, hand_config_path);
  sr_hand_autodetect.run();
  ASSERT_EQ(sr_hand_autodetect.get_command_suffix(), expected_command_suffix);
}

TEST(SrHandAutodetect, test_run_no_hands)
{
  std::string expected_command_suffix = "";
  std::string hand_config_path = ros::package::getPath("sr_hand_detector") + "/test/config";
  std::string detected_hands_file = ros::package::getPath("sr_hand_detector") + "/test/config/test_no_hands.yaml";

  sr_hand_detector::SrHandAutodetect sr_hand_autodetect(detected_hands_file, hand_config_path);
  sr_hand_autodetect.run();
  ASSERT_EQ(sr_hand_autodetect.get_command_suffix(), expected_command_suffix);
}

TEST(SrHandAutodetect, test_run_non_existing_hand)
{
  std::string hand_config_path = ros::package::getPath("sr_hand_detector") + "/test/config";
  std::string detected_hands_file = ros::package::getPath("sr_hand_detector") +
    "/test/config/test_non_existing_hand.yaml";
  sr_hand_detector::SrHandAutodetect sr_hand_autodetect(detected_hands_file, hand_config_path);
  ASSERT_THROW(sr_hand_autodetect.run(), YAML::BadFile);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_sr_hand_autodetect");
  ros::NodeHandle nh;
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
