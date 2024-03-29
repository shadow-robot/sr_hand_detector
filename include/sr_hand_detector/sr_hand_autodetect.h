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

#ifndef SR_HAND_DETECTOR_SR_HAND_AUTODETECT_H
#define SR_HAND_DETECTOR_SR_HAND_AUTODETECT_H

#include <string>
#include <map>
#include <vector>

#include "sr_hand_detector/sr_hand_detector.h"
#include "yaml-cpp/yaml.h"

namespace sr_hand_detector
{

enum class ForcedHandSide {none, right, left};

class SrHandAutodetect
{
  public:
    explicit SrHandAutodetect(std::string detected_hands_file = "/tmp/sr_hand_detector.yaml",
                              std::string hand_config_path = "",
                              ForcedHandSide forced_hand_side = ForcedHandSide::none);
    ~SrHandAutodetect();
    void run();
    std::string get_command_suffix();

  private:
    void get_path_to_sr_hand_config();
    YAML::Node get_hand_general_info(int serial);
    void detect_hands();
    void filter_hands_if_side_forced();
    void compose_command_suffix();
    void compose_command_suffix_unimanual();
    void compose_command_suffix_bimanual();
    std::string vector_to_xacro_string(const std::vector<std::string> &vec);
    std::string map_to_xacro_string(const std::map<std::string, std::string> &map);
    std::vector<std::string> yaml_node_list_to_std_vector(const YAML::Node &node);
    std::map<std::string, std::string> yaml_node_map_to_std_map(const YAML::Node &node_map);
    std::string command_suffix_bimanual_per_hand(const YAML::Node &hand_info);
    std::string hand_side_to_prefix(const std::string &side);

    ForcedHandSide forced_hand_side_;
    int number_of_detected_hands_;
    std::string sr_hand_config_path_;
    std::string detected_hands_file_;
    std::string command_suffix_;
    std::map<int, std::string> hand_serial_and_port_map_;
};
}  // namespace sr_hand_detector

#endif  // SR_HAND_DETECTOR_SR_HAND_AUTODETECT_H
