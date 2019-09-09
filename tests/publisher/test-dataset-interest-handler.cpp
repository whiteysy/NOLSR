/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/**
 * Copyright (c) 2014-2019,  The University of Memphis,
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

#include "publisher/dataset-interest-handler.hpp"
#include "tests/test-common.hpp"
#include "tlv/tlv-nlsr.hpp"

#include "publisher-fixture.hpp"
#include "../boost-test.hpp"

#include <ndn-cxx/mgmt/nfd/control-response.hpp>

#include <iostream>

namespace nlsr {
namespace test {

static void
processDatasetInterest(ndn::util::DummyClientFace& face,
                       std::function<bool(const ndn::Block&)> isSameType)
{
  face.processEvents(30_ms);

  BOOST_REQUIRE_EQUAL(face.sentData.size(), 1);

  ndn::Block parser(face.sentData[0].getContent());
  parser.parse();

  ndn::Block::element_const_iterator it = parser.elements_begin();
  BOOST_CHECK_EQUAL(isSameType(*it), true);
  ++it;

  BOOST_CHECK(it == parser.elements_end());

  face.sentData.clear();
}

BOOST_FIXTURE_TEST_SUITE(PublisherTestLsdbDatasetInterestHandler, PublisherFixture)

BOOST_AUTO_TEST_CASE(Localhost)
{
  checkPrefixRegistered(Nlsr::LOCALHOST_PREFIX);

  // Install adjacency LSA
  AdjLsa adjLsa;
  adjLsa.setOrigRouter("/RouterA");
  addAdjacency(adjLsa, "/RouterA/adjacency1", "udp://face-1", 10);
  lsdb.installAdjLsa(adjLsa);

  std::vector<double> angles = {20.00, 30.00};

  // Install coordinate LSA
  CoordinateLsa coordinateLsa = createCoordinateLsa("/RouterA", 10.0, angles);
  lsdb.installCoordinateLsa(coordinateLsa);

  // Install Name LSA
  NameLsa nameLsa;
  nameLsa.setOrigRouter("/RouterA");
  nameLsa.addName("/RouterA/name1");
  lsdb.installNameLsa(nameLsa);

  // Install routing table
  RoutingTableEntry rte1("desrouter1");
  const ndn::Name& DEST_ROUTER = rte1.getDestination();

  NextHop nh = createNextHop("udp://face-test1", 10);

  rt1.addNextHop(DEST_ROUTER, nh);

  // Request adjacency LSAs
  face.receive(ndn::Interest("/localhost/nlsr/lsdb/adjacencies").setCanBePrefix(true));
  processDatasetInterest(face,
    [] (const ndn::Block& block) { return block.type() == ndn::tlv::nlsr::AdjacencyLsa; });

  // Request coordinate LSAs
  face.receive(ndn::Interest("/localhost/nlsr/lsdb/coordinates").setCanBePrefix(true));
  processDatasetInterest(face,
    [] (const ndn::Block& block) { return block.type() == ndn::tlv::nlsr::CoordinateLsa; });

  // Request Name LSAs
  face.receive(ndn::Interest("/localhost/nlsr/lsdb/names").setCanBePrefix(true));
  processDatasetInterest(face,
    [] (const ndn::Block& block) { return block.type() == ndn::tlv::nlsr::NameLsa; });

  // Request Routing Table
  face.receive(ndn::Interest("/localhost/nlsr/routing-table").setCanBePrefix(true));
  processDatasetInterest(face,
    [] (const ndn::Block& block) {
      return block.type() == ndn::tlv::nlsr::RoutingTable; });
}

BOOST_AUTO_TEST_CASE(Routername)
{
  ndn::Name regRouterPrefix(conf.getRouterPrefix());
  regRouterPrefix.append("nlsr");
  // Should already be added to dispatcher
  BOOST_CHECK_THROW(nlsr.m_dispatcher.addTopPrefix(regRouterPrefix), std::out_of_range);

  checkPrefixRegistered(regRouterPrefix);

  // Install adjacencies LSA
  AdjLsa adjLsa;
  adjLsa.setOrigRouter("/RouterA");
  addAdjacency(adjLsa, "/RouterA/adjacency1", "udp://face-1", 10);
  lsdb.installAdjLsa(adjLsa);

  std::vector<double> angles = {20.00, 30.00};

  // Install coordinate LSA
  CoordinateLsa coordinateLsa = createCoordinateLsa("/RouterA", 10.0, angles);
  lsdb.installCoordinateLsa(coordinateLsa);

  // Install routing table
  RoutingTableEntry rte1("desrouter1");
  const ndn::Name& DEST_ROUTER = rte1.getDestination();

  NextHop nh = createNextHop("udp://face-test1", 10);

  rt1.addNextHop(DEST_ROUTER, nh);

  ndn::Name routerName(conf.getRouterPrefix());
  routerName.append("nlsr");

  // Request adjacency LSAs
  face.receive(ndn::Interest(ndn::Name(routerName).append("lsdb").append("adjacencies")).setCanBePrefix(true));
  processDatasetInterest(face,
    [] (const ndn::Block& block) { return block.type() == ndn::tlv::nlsr::AdjacencyLsa; });

  // Request coordinate LSAs
  face.receive(ndn::Interest(ndn::Name(routerName).append("lsdb").append("coordinates")).setCanBePrefix(true));
  processDatasetInterest(face,
    [] (const ndn::Block& block) { return block.type() == ndn::tlv::nlsr::CoordinateLsa; });

  // Request Name LSAs
  face.receive(ndn::Interest(ndn::Name(routerName).append("lsdb").append("names")).setCanBePrefix(true));
  processDatasetInterest(face,
    [] (const ndn::Block& block) { return block.type() == ndn::tlv::nlsr::NameLsa; });

  // Request Routing Table
  face.receive(ndn::Interest(ndn::Name(routerName).append("routing-table")));
  processDatasetInterest(face,
    [] (const ndn::Block& block) {
      return block.type() == ndn::tlv::nlsr::RoutingTable; });
}

BOOST_AUTO_TEST_SUITE_END()

} // namespace test
} // namespace nlsr
