/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/**
 * Copyright (c) 2014-2017,  The University of Memphis,
 *                           Regents of the University of California,
 *                           Arizona Board of Regents.
 *
 * This file is part of NLSR (Named-data Link State Routing).
 * See AUTHORS.md for complete list of NLSR authors and contributors.
 *
 * NLSR is free software: you can redistribute it and/or modify it under the terms
 * of the GNU General Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later version.
 *
 * NLSR is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
 * without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
 * PURPOSE.  See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * NLSR, e.g., in COPYING.md file.  If not, see <http://www.gnu.org/licenses/>.
 **/

#include "nfd-rib-commands.hpp"

namespace nlsr {
namespace update {

NfdRibRegisterCommand::NfdRibRegisterCommand()
  : ControlCommand("nlsr", "register")
{
  m_requestValidator.required(ndn::nfd::CONTROL_PARAMETER_NAME);
  m_requestValidator.optional(ndn::nfd::CONTROL_PARAMETER_ORIGIN);
}

NfdRibUnregisterCommand::NfdRibUnregisterCommand()
  : ControlCommand("nlsr", "unregister")
{
  m_requestValidator.required(ndn::nfd::CONTROL_PARAMETER_NAME);
  m_requestValidator.optional(ndn::nfd::CONTROL_PARAMETER_ORIGIN);
}

} // namespace update
} // namespace nlsr
