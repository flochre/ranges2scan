// Copyright 2023 flochre
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "ranges2scan/range_info.hpp"

RangeInfo::RangeInfo(std::string frame_id)
: frame_id_(frame_id)
{}

RangeInfo::~RangeInfo(){}

// void set_frame_id(std::string frame_id);
void RangeInfo::set_distance(float distance){
  distance_ = distance;
}
void RangeInfo::set_index(int index){
  index_ = index;
}

std::string RangeInfo::get_frame_id(){
  return frame_id_;
}
float RangeInfo::get_distance(){
  return distance_;
}
int RangeInfo::get_index(){
  return index_;
}