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

#include "communication/sync-protocol-adapter.hpp"
#include "../test-common.hpp"
#include "../boost-test.hpp"

#include <ndn-cxx/util/dummy-client-face.hpp>

#include <boost/mpl/int.hpp>
#include <boost/mpl/vector.hpp>

namespace nlsr {
namespace test {

using namespace ndn;

class SyncProtocolAdapterFixture : public UnitTestTimeFixture
{
public:
  SyncProtocolAdapterFixture()
   : syncPrefix("/localhop/ndn/nlsr/sync/")
   , nameLsaUserPrefix("/localhop/ndn/nlsr/LSA/NAME")
   , syncInterestLifetime(time::seconds(60))
  {
  	syncPrefix.appendVersion(4);
  }

  template <int32_t T>
  void
  addNodes()
  {
    for (int i = 0; i < 2; i++) {
      faces[i] = std::make_shared<ndn::util::DummyClientFace>(m_ioService,
                                                              util::DummyClientFace::Options{true, true});
      userPrefixes[i] = Name(nameLsaUserPrefix).appendNumber(i);
      nodes[i] = std::make_shared<SyncProtocolAdapter>(*faces[i], T, syncPrefix,
      	                                               userPrefixes[i],
                                                       syncInterestLifetime,
                                                       [i, this] (const ndn::Name& updateName,
                                                                   uint64_t highSeq) {
                                                         prefixToSeq[i].emplace(updateName, highSeq);
                                                       });
    }

    faces[0]->linkTo(*faces[1]);
    advanceClocks(ndn::time::milliseconds(10), 10);
  }

public:
  Name syncPrefix, nameLsaUserPrefix;
  Name userPrefixes[2];
  time::milliseconds syncInterestLifetime;
  std::shared_ptr<ndn::util::DummyClientFace> faces[2];
  std::shared_ptr<SyncProtocolAdapter> nodes[2];
  std::map<ndn::Name, uint64_t> prefixToSeq[2];
};

using boost::mpl::int_;
using Protocols = boost::mpl::vector<int_<SYNC_PROTOCOL_CHRONOSYNC>, int_<SYNC_PROTOCOL_PSYNC>>;

BOOST_FIXTURE_TEST_SUITE(TestSyncProtocolAdapter, SyncProtocolAdapterFixture)

BOOST_AUTO_TEST_CASE_TEMPLATE(Sync, SyncProtocol, Protocols)
{
  addNodes<SyncProtocol::value>();

  nodes[0]->publishUpdate(userPrefixes[0], 10);
  advanceClocks(ndn::time::milliseconds(1000), 100);

  auto it = prefixToSeq[1].find(userPrefixes[0]);
  BOOST_CHECK(it != prefixToSeq[1].end());
  BOOST_CHECK_EQUAL(it->first, userPrefixes[0]);
  BOOST_CHECK_EQUAL(it->second, 10);

  nodes[1]->publishUpdate(userPrefixes[1], 100);
  advanceClocks(ndn::time::milliseconds(1000), 100);

  it = prefixToSeq[0].find(userPrefixes[1]);
  BOOST_CHECK(it != prefixToSeq[0].end());
  BOOST_CHECK_EQUAL(it->first, userPrefixes[1]);
  BOOST_CHECK_EQUAL(it->second, 100);

  Name adjLsaUserPrefix("/localhop/ndn/nlsr/LSA/ADJACENCY");
  nodes[0]->addUserNode(adjLsaUserPrefix);
  advanceClocks(ndn::time::milliseconds(1000), 100);
  nodes[0]->publishUpdate(adjLsaUserPrefix, 10);
  advanceClocks(ndn::time::milliseconds(1000), 100);

  it = prefixToSeq[1].find(adjLsaUserPrefix);
  BOOST_CHECK(it != prefixToSeq[1].end());
  BOOST_CHECK_EQUAL(it->first, adjLsaUserPrefix);
  BOOST_CHECK_EQUAL(it->second, 10);
}

BOOST_AUTO_TEST_SUITE_END()

} // namespace test
} // namespace nlsr