/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#ifndef DRIVER_VELODYNE_INPUT_H
#define DRIVER_VELODYNE_INPUT_H

#include <stdio.h>
#include <unistd.h>
#include <string>

namespace driver {
namespace velodyne {

/** @brief Pure virtual Velodyne input base class */
class Input {
 public:
  Input() {}
  virtual ~Input() {}
  virtual int ReceivePacket(void* data, const int length, uint64_t* nsec) = 0;
  virtual int Init() = 0;
};

}  // namespace velodyne
}  // namespace driver

#endif  // DRIVER_VELODYNE_INPUT_H
