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
 *
 **/

#ifndef NLSR_ROUTING_TABLE_HPP
#define NLSR_ROUTING_TABLE_HPP

#include "conf-parameter.hpp"
#include "routing-table-entry.hpp"
#include "signals.hpp"
#include "lsdb.hpp"
#include "route/fib.hpp"
#include "tuple-state.hpp"

#include <iostream>
#include <utility>
#include <string>
#include <boost/cstdint.hpp>
#include <ndn-cxx/util/scheduler.hpp>

namespace nlsr {

class NextHop;

class RoutingTable : boost::noncopyable
{
public:
  explicit
  RoutingTable(ndn::Scheduler& scheduler, TupleState& tuplestate, Fib& fib, Lsdb& lsdb,
               NamePrefixTable& namePrefixTable, ConfParameter& confParam);

  /*! \brief Calculates a list of next hops for each router in the network.
   *
   *  Calculates the list of next hops to every other router in the network.
   */
  void
  calculate();

  /*! \brief Adds a next hop to a routing table entry.
   *  \param destRouter The destination router whose RTE we want to modify.
   *  \param nh The next hop to add to the RTE.
   */
  void
  addNextHop(const ndn::Name& destRouter, NextHop& nh);

  /*! \brief Adds a next hop to a routing table entry in a dry run scenario.
   *  \param destRouter The destination router whose RTE we want to modify.
   *  \param nh The next hop to add to the router.
   */
  void
  addNextHopToDryTable(const ndn::Name& destRouter, NextHop& nh);

  RoutingTableEntry*
  findRoutingTableEntry(const ndn::Name& destRouter);

  /*! \brief Schedules a calculation event in the event scheduler only
   *  if one isn't already scheduled.
   */
  void
  scheduleRoutingTableCalculation();

  int
  getNoNextHop()
  {
    return m_NO_NEXT_HOP;
  }

  void
  setRoutingCalcInterval(uint32_t interval)
  {
    m_routingCalcInterval = ndn::time::seconds(interval);
  }

  const ndn::time::seconds&
  getRoutingCalcInterval() const
  {
    return m_routingCalcInterval;
  }

  const std::list<RoutingTableEntry>&
  getRoutingTableEntry() const
  {
    return m_rTable;
  }

  const std::list<RoutingTableEntry>&
  getDryRoutingTableEntry() const
  {
    return m_dryTable;
  }

  uint64_t
  getRtSize()
  {
    return m_rTable.size();
  }

private:
  /*! \brief Calculates a link-state routing table. */
  void
  calculateLsRoutingTable();

  /*! \brief Calculates a HR routing table. */
  void
  calculateHypRoutingTable(bool isDryRun);

  void
  clearRoutingTable();

  void
  clearDryRoutingTable();

  void
  writeLog();

public:
  std::unique_ptr<AfterRoutingChange> afterRoutingChange;

PUBLIC_WITH_TESTS_ELSE_PRIVATE:
  std::list<RoutingTableEntry> m_rTable;

private:
  ndn::Scheduler& m_scheduler;
  TupleState& m_state;
  Fib& m_fib;
  Lsdb& m_lsdb;
  NamePrefixTable& m_namePrefixTable;

  const int m_NO_NEXT_HOP;

  std::list<RoutingTableEntry> m_dryTable;

  ndn::time::seconds m_routingCalcInterval;

  bool m_isRoutingTableCalculating;
  bool m_isRouteCalculationScheduled;

  ConfParameter& m_confParam;
};

} // namespace nlsr

#endif // NLSR_ROUTING_TABLE_HPP
