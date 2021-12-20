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

#include <iostream>
#include <string>
#include <ros/package.h>
#include "sr_hand_detector/sr_hand_autodetect.h"

namespace
{
  const std::string DETECTED_HANDS_FILE = "/tmp/sr_hand_detector.yaml";
  const std::string HAND_CONFIG_PATH = "";
}

int main(int argc, char* argv[])
{
  int result = system("sr_hand_detector_node");
  if (result != 0) return result;

  int iter = 1;
  std::string first_arg = argv[iter];
  sr_hand_detector::ForcedHandSide forced_hand_side = sr_hand_detector::ForcedHandSide::none;

  if (("--right-only" == first_arg) || ("-r" == first_arg))
  {
    forced_hand_side = sr_hand_detector::ForcedHandSide::right;
    iter++;
  }
  else if (("--left-only" == first_arg) || ("-l" == first_arg))
  {
    forced_hand_side = sr_hand_detector::ForcedHandSide::left;
    iter++;
  }

  std::string command_string = "";
  for (int i = iter; i < argc; ++i)
  {
      command_string += argv[i];
      if (!(argc - 1 == i))
      {
          command_string += " ";
      }
  }

  sr_hand_detector::SrHandAutodetect sr_hand_autodetect(DETECTED_HANDS_FILE,
                                                        HAND_CONFIG_PATH,
                                                        forced_hand_side);
  sr_hand_autodetect.run();

  command_string += sr_hand_autodetect.get_command_suffix();
  std::cout << "Actual command run: " << command_string << std::endl;
  return system(command_string.c_str());
}
