/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/**
 * Copyright (c) 2014-2019,  The University of Memphis,
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

#ifndef NLSR_MAP_HPP
#define NLSR_MAP_HPP

#include "common.hpp"
#include "map-entry.hpp"

#include <iostream>
#include <list>
#include <boost/cstdint.hpp>

#include <boost/multi_index_container.hpp>
#include <boost/multi_index/hashed_index.hpp>
#include <boost/multi_index/mem_fun.hpp>
#include <boost/multi_index/tag.hpp>

namespace nlsr {

namespace detail {

  using namespace boost::multi_index;
  // Define tags so that we can search by different indices.
  struct byRouterName {};
  struct byMappingNumber{};
  using entryContainer = multi_index_container<
    MapEntry,
    indexed_by<
      hashed_unique<tag<byRouterName>,
                    const_mem_fun<MapEntry, const ndn::Name&, &MapEntry::getRouter>,
                    std::hash<ndn::Name>>,
      hashed_unique<tag<byMappingNumber>,
                    const_mem_fun<MapEntry, int32_t, &MapEntry::getMappingNumber>>
      >
    >;

} // namespace detail

class Map
{
public:
  Map()
    : m_mappingIndex(0)
  {
  }

  /*! \brief Add a map entry to this map.
    \param rtrName The name of the router.

    Adds a router to this map. Each entry is also given an arbitrary,
    ascending mappingNo (mapping number).
  */
  void
  addEntry(const ndn::Name& rtrName);

  /*! Populates the Map with AdjacencyLsas.

    \note IteratorType must an iterator type, and begin to end must represent a valid range.
  */
  template<typename IteratorType>
  void
  createFromAdjLsdb(IteratorType begin, IteratorType end)
  {
    BOOST_STATIC_ASSERT_MSG(is_iterator<IteratorType>::value, "IteratorType must be an iterator!");
    for (auto lsa = begin; lsa != end; lsa++) {
      addEntry(lsa->getOrigRouter());
      for (const auto& adjacent : lsa->getAdl().getAdjList()) {
        addEntry(adjacent.getName());
      }
    }
  }

  /*! Populates the Map with CoordinateLsas.

    \note IteratorType must an iterator type, and begin to end must represent a valid range.
  */
  template<typename IteratorType>
  void
  createFromCoordinateLsdb(IteratorType begin, IteratorType end)
  {
    BOOST_STATIC_ASSERT_MSG(is_iterator<IteratorType>::value, "IteratorType must be an iterator!");
    for (auto lsa = begin; lsa != end; lsa++) {
      addEntry(lsa->getOrigRouter());
    }
  }

  ndn::optional<ndn::Name>
  getRouterNameByMappingNo(int32_t mn) const;

  ndn::optional<int32_t>
  getMappingNoByRouterName(const ndn::Name& rName);

  void
  reset();

  size_t
  getMapSize() const
  {
    return m_entries.size();
  }

  void
  writeLog();

private:
  bool
  addEntry(MapEntry& mpe);

  int32_t m_mappingIndex;
  detail::entryContainer m_entries;
};

} // namespace nlsr

#endif // NLSR_MAP_HPP
