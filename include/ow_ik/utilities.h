/*! \file
 *
 * \author J. Rogelio Guadarrama-Olvera
 * \author Emmanuel Dean-Leon
 * \author Florian Bergner
 * \author Simon Armleder
 * \author Gordon Cheng
 *
 * \version 0.1
 * \date 03.05.2020
 *
 * \copyright Copyright 2020 Institute for Cognitive Systems (ICS),
 *    Technical University of Munich (TUM)
 *
 * #### Licence
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 *
 * #### Acknowledgment
 *  This project has received funding from the European Union‘s Horizon 2020
 *  research and innovation programme under grant agreement No 732287.
 */

#ifndef OPEN_WALKER_IK_UTILITIES_H
#define OPEN_WALKER_IK_UTILITIES_H

#include <kdl/frames.hpp>
#include <ow_core/types.h>

namespace ow_ik
{

/*!
 * \brief KDL::Frame to ow::HomogeneousTransformation
 */
inline ow::HomogeneousTransformation& convert(
  ow::HomogeneousTransformation& T,
  const KDL::Frame& T_kdl)
{
  for(int i = 0; i < T.pos().size(); ++i)
  {
    T.pos()[i] = T_kdl.p[i];
  }
  for(int i = 0; i < T.orien().size(); ++i)
  {
    T.orien()(i/3, i%3) = T_kdl.M.data[i];
  }
  return T;
}

/*!
 * \brief ow::HomogeneousTransformation to KDL::Frame
 */
inline KDL::Frame& convert(
  KDL::Frame& T_kdl,
  const ow::HomogeneousTransformation& T)
{
  for(int i = 0; i < T.pos().size(); ++i)
  {
    T_kdl.p[i] = T.pos()[i];
  }
  for(int i = 0; i < T.orien().size(); ++i)
  {
    T_kdl.M.data[i] = T.orien()(i/3, i%3);
  }
  return T_kdl;
}

/*!
 * \brief KDL::Wrench to ow::Wrench
 */
inline ow::Wrench& convert(
  ow::Wrench& wrench,
  const KDL::Wrench& wrench_kdl)
{
  for(int i = 0; i < 3; ++i)
  {
    wrench.force()[i] = wrench_kdl.force.data[i];
    wrench.moment()[i] = wrench_kdl.torque.data[i];
  }
  return wrench;
}

/*!
 * \brief ow::Wrench to KDL::Wrench
 */
inline KDL::Wrench& convert(
  KDL::Wrench& wrench_kdl,
  const ow::Wrench& wrench)
{
  for(int i = 0; i < 3; ++i)
  {
    wrench_kdl.force.data[i] = wrench.force()[i];
    wrench_kdl.torque.data[i] = wrench.moment()[i];
  }
  return wrench_kdl;
}

}

#endif
