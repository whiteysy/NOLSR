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
 * \author Ashlesh Gawande <agawande@memphis.edu>
 *
 **/

#include "route/routing-table.hpp"
#include "nlsr.hpp"
#include "../test-common.hpp"
#include "route/routing-table-entry.hpp"
#include "route/nexthop.hpp"
#include <boost/test/unit_test.hpp>

namespace nlsr {
namespace test {

BOOST_FIXTURE_TEST_SUITE(TestRoutingTable, BaseFixture)

BOOST_AUTO_TEST_CASE(RoutingTableAddNextHop)
{
  ndn::util::DummyClientFace face;
  ConfParameter conf(face);
  ndn::KeyChain keyChain;
  Nlsr nlsr(face, keyChain, conf);

  RoutingTable rt1(m_scheduler, nlsr.m_fib, nlsr.m_lsdb,
                   nlsr.m_namePrefixTable, conf);

  NextHop nh1;
  const std::string DEST_ROUTER = "destRouter";
  rt1.addNextHop(DEST_ROUTER, nh1);

  BOOST_CHECK_EQUAL(rt1.findRoutingTableEntry(DEST_ROUTER)->getDestination(), DEST_ROUTER);
}

BOOST_AUTO_TEST_SUITE_END()

} // namespace test
} // namespace nlsr
