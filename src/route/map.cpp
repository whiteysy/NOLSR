/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/**
 * Copyright (c) 2014-2018,  The University of Memphis,
 *                           Regents of the University of California
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

#include "map.hpp"
#include "nlsr.hpp"
#include "adjacent.hpp"
#include "lsa.hpp"
#include "lsdb.hpp"
#include "logger.hpp"

namespace nlsr {

INIT_LOGGER(route.Map);

void
Map::addEntry(const ndn::Name& rtrName)
{
  MapEntry me(rtrName, m_mappingIndex);
  if (addEntry(me)) {
    m_mappingIndex++;
  }
}

bool
Map::addEntry(MapEntry& mpe)
{
  return m_entries.insert(mpe).second;
}

ndn::optional<ndn::Name>
Map::getRouterNameByMappingNo(int32_t mn) const
{
  auto&& mappingNumberView = m_entries.get<detail::byMappingNumber>();
  auto iterator = mappingNumberView.find(mn);
  if (iterator == mappingNumberView.end()) {
    return {};
  }
  else {
    return {iterator->getRouter()};
  }
}

ndn::optional<int32_t>
Map::getMappingNoByRouterName(const ndn::Name& rName)
{
  auto&& routerNameView = m_entries.get<detail::byRouterName>();
  auto iterator = routerNameView.find(rName);
  if (iterator == routerNameView.end()) {
    return {};
  }
  else {
    return {iterator->getMappingNumber()};
  }
}

void
Map::reset()
{
  m_entries = detail::entryContainer{};
  m_mappingIndex = 0;
}

void
Map::writeLog()
{
  NLSR_LOG_DEBUG("---------------Map----------------------");
  auto&& routerNameView = m_entries.get<detail::byRouterName>();
  for (auto entry = routerNameView.begin(); entry != routerNameView.end(); entry++) {
    NLSR_LOG_DEBUG("MapEntry: ( Router: " << entry->getRouter() << " Mapping No: "
               << entry->getMappingNumber() << " )");
  }
}

} // namespace nlsr
