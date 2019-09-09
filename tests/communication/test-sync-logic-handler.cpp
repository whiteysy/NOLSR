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

#include "communication/sync-logic-handler.hpp"
#include "../test-common.hpp"
#include "common.hpp"
#include "nlsr.hpp"
#include "lsa.hpp"

#include <ndn-cxx/util/dummy-client-face.hpp>

namespace nlsr {
namespace test {

using std::shared_ptr;

template<int32_t Protocol>
class SyncLogicFixture : public UnitTestTimeFixture
{
public:
  SyncLogicFixture()
    : face(m_ioService, m_keyChain)
    , conf(face)
    , confProcessor(conf, Protocol)
    , testIsLsaNew([] (const ndn::Name& name, const Lsa::Type& lsaType,
                       const uint64_t sequenceNumber) {
                     return true;
                   })
    , sync(face, testIsLsaNew, conf)
    , updateNamePrefix(this->conf.getLsaPrefix().toUri() +
                       this->conf.getSiteName().toUri() +
                       "/%C1.Router/other-router/")
  {
    addIdentity(conf.getRouterPrefix());
  }

  void
  receiveUpdate(const std::string& prefix, uint64_t seqNo)
  {
    this->advanceClocks(ndn::time::milliseconds(1), 10);
    face.sentInterests.clear();

    if (Protocol == SYNC_PROTOCOL_CHRONOSYNC) {
      std::vector<chronosync::MissingDataInfo> updates;
      updates.push_back({ndn::Name(prefix).appendNumber(1), 0, seqNo});
      sync.m_syncLogic->onChronoSyncUpdate(updates);
    }
    else {
      std::vector<psync::MissingDataInfo> updates;
      updates.push_back({ndn::Name(prefix), 0, seqNo});
      sync.m_syncLogic->onPSyncUpdate(updates);
    }

    this->advanceClocks(ndn::time::milliseconds(1), 10);
  }

public:
  ndn::util::DummyClientFace face;
  ConfParameter conf;
  DummyConfFileProcessor confProcessor;
  SyncLogicHandler::IsLsaNew testIsLsaNew;
  SyncLogicHandler sync;

  const std::string updateNamePrefix;
  const std::vector<Lsa::Type> lsaTypes = {Lsa::Type::NAME, Lsa::Type::ADJACENCY,
                                             Lsa::Type::COORDINATE};
};

using mpl_::int_;
using Protocols = boost::mpl::vector<int_<SYNC_PROTOCOL_CHRONOSYNC>,
                                     int_<SYNC_PROTOCOL_PSYNC>>;

BOOST_AUTO_TEST_SUITE(TestSyncLogicHandler)

/* Tests that when SyncLogicHandler receives an LSA of either Name or
   Adjacency type that appears to be newer, it will emit to its signal
   with those LSA details.
 */
BOOST_FIXTURE_TEST_CASE_TEMPLATE(UpdateForOtherLS, T, Protocols, SyncLogicFixture<T::value>)
{
  std::vector<Lsa::Type> lsaTypes = {Lsa::Type::NAME, Lsa::Type::ADJACENCY};

  uint64_t syncSeqNo = 1;

  for (const Lsa::Type& lsaType : lsaTypes) {
    std::string updateName = this->updateNamePrefix + std::to_string(lsaType);

    // Actual testing done here -- signal function callback
    ndn::util::signal::ScopedConnection connection = this->sync.onNewLsa->connect(
      [&] (const ndn::Name& routerName, const uint64_t& sequenceNumber,
           const ndn::Name& originRouter) {
        BOOST_CHECK_EQUAL(ndn::Name{updateName}, routerName);
        BOOST_CHECK_EQUAL(sequenceNumber, syncSeqNo);
      });

    this->receiveUpdate(updateName, syncSeqNo);
  }
}

/* Tests that when SyncLogicHandler in HR mode receives an LSA of
   either Coordinate or Name type that appears to be newer, it will
   emit to its signal with those LSA details.
 */
BOOST_FIXTURE_TEST_CASE_TEMPLATE(UpdateForOtherHR, T, Protocols, SyncLogicFixture<T::value>)
{
  this->conf.setHyperbolicState(HYPERBOLIC_STATE_ON);

  uint64_t syncSeqNo = 1;
  std::vector<Lsa::Type> lsaTypes = {Lsa::Type::NAME, Lsa::Type::COORDINATE};

  for (const Lsa::Type& lsaType : lsaTypes) {
    std::string updateName = this->updateNamePrefix + std::to_string(lsaType);

    ndn::util::signal::ScopedConnection connection = this->sync.onNewLsa->connect(
      [&] (const ndn::Name& routerName, const uint64_t& sequenceNumber,
           const ndn::Name& originRouter) {
        BOOST_CHECK_EQUAL(ndn::Name{updateName}, routerName);
        BOOST_CHECK_EQUAL(sequenceNumber, syncSeqNo);
      });

    this->receiveUpdate(updateName, syncSeqNo);
  }
}

/* Tests that when SyncLogicHandler in HR-dry mode receives an LSA of
   any type that appears to be newer, it will emit to its signal with
   those LSA details.
 */
BOOST_FIXTURE_TEST_CASE_TEMPLATE(UpdateForOtherHRDry, T, Protocols, SyncLogicFixture<T::value>)
{
  this->conf.setHyperbolicState(HYPERBOLIC_STATE_DRY_RUN);

  uint64_t syncSeqNo = 1;

  for (const Lsa::Type& lsaType : this->lsaTypes) {
    std::string updateName = this->updateNamePrefix + std::to_string(lsaType);

    ndn::util::signal::ScopedConnection connection = this->sync.onNewLsa->connect(
      [&] (const ndn::Name& routerName, const uint64_t& sequenceNumber,
           const ndn::Name& originRouter) {
        BOOST_CHECK_EQUAL(ndn::Name{updateName}, routerName);
        BOOST_CHECK_EQUAL(sequenceNumber, syncSeqNo);
      });

    this->receiveUpdate(updateName, syncSeqNo);
  }
}

/* Tests that when SyncLogicHandler receives an update for an LSA with
   details matching this router's details, it will *not* emit to its
   signal those LSA details.
 */
BOOST_FIXTURE_TEST_CASE_TEMPLATE(NoUpdateForSelf, T, Protocols, SyncLogicFixture<T::value>)
{
  const uint64_t sequenceNumber = 1;

  for (const Lsa::Type& lsaType : this->lsaTypes) {
    // To ensure that we get correctly-separated components, create
    // and modify a Name to hand off.
    ndn::Name updateName = ndn::Name{this->conf.getLsaPrefix()};
    updateName.append(this->conf.getSiteName())
              .append(this->conf.getRouterName())
              .append(std::to_string(lsaType));

    ndn::util::signal::ScopedConnection connection = this->sync.onNewLsa->connect(
      [&] (const ndn::Name& routerName, const uint64_t& sequenceNumber,
           const ndn::Name& originRouter) {
        BOOST_FAIL("Updates for self should not be emitted!");
      });

    this->receiveUpdate(updateName.toUri(), sequenceNumber);
  }
}

/* Tests that when SyncLogicHandler receives an update for an LSA with
   details that do not match the expected format, it will *not* emit
   to its signal those LSA details.
 */
BOOST_FIXTURE_TEST_CASE_TEMPLATE(MalformedUpdate, T, Protocols, SyncLogicFixture<T::value>)
{
  const uint64_t sequenceNumber = 1;

  for (const Lsa::Type& lsaType : this->lsaTypes) {
    ndn::Name updateName{this->conf.getSiteName()};
    updateName.append(this->conf.getRouterName()).append(std::to_string(lsaType));

    ndn::util::signal::ScopedConnection connection = this->sync.onNewLsa->connect(
      [&] (const ndn::Name& routerName, const uint64_t& sequenceNumber,
           const ndn::Name& originRouter) {
        BOOST_FAIL("Malformed updates should not be emitted!");
      });

    this->receiveUpdate(updateName.toUri(), sequenceNumber);
  }
}

/* Tests that when SyncLogicHandler receives an update for an LSA with
   details that do not appear to be new, it will *not* emit to its
   signal those LSA details.
 */
BOOST_FIXTURE_TEST_CASE_TEMPLATE(LsaNotNew, T, Protocols, SyncLogicFixture<T::value>)
{
  auto testLsaAlwaysFalse = [] (const ndn::Name& routerName, const Lsa::Type& lsaType,
                                const uint64_t& sequenceNumber) {
    return false;
  };

  const uint64_t sequenceNumber = 1;
  SyncLogicHandler sync{this->face, testLsaAlwaysFalse, this->conf};
    ndn::util::signal::ScopedConnection connection = sync.onNewLsa->connect(
      [&] (const ndn::Name& routerName, const uint64_t& sequenceNumber,
           const ndn::Name& originRouter) {
        BOOST_FAIL("An update for an LSA with non-new sequence number should not emit!");
      });

  std::string updateName = this->updateNamePrefix + std::to_string(Lsa::Type::NAME);

  this->receiveUpdate(updateName, sequenceNumber);
}

/* Tests that SyncLogicHandler successfully concatenates configured
   variables together to form the necessary prefixes to advertise
   through ChronoSync.
 */
BOOST_FIXTURE_TEST_CASE_TEMPLATE(UpdatePrefix, T, Protocols, SyncLogicFixture<T::value>)
{
  ndn::Name expectedPrefix = this->conf.getLsaPrefix();
  expectedPrefix.append(this->conf.getSiteName());
  expectedPrefix.append(this->conf.getRouterName());

  this->sync.buildUpdatePrefix();

  BOOST_CHECK_EQUAL(this->sync.m_nameLsaUserPrefix,
                    ndn::Name(expectedPrefix).append(std::to_string(Lsa::Type::NAME)));
  BOOST_CHECK_EQUAL(this->sync.m_adjLsaUserPrefix,
                    ndn::Name(expectedPrefix).append(std::to_string(Lsa::Type::ADJACENCY)));
  BOOST_CHECK_EQUAL(this->sync.m_coorLsaUserPrefix,
                    ndn::Name(expectedPrefix).append(std::to_string(Lsa::Type::COORDINATE)));
}

/* Tests that SyncLogicHandler's socket will be created when
   Nlsr is initialized, preventing use of sync before the
   socket is created.

   NB: This test is as much an Nlsr class test as a
   SyncLogicHandler class test, but it rides the line and ends up here.
 */
BOOST_FIXTURE_TEST_CASE_TEMPLATE(createSyncLogicOnInitialization, T, Protocols,
                                 SyncLogicFixture<T::value>) // Bug #2649
{
  Nlsr nlsr(this->face, this->m_keyChain, this->conf);

  // Make sure an adjacency LSA has not been built yet
  ndn::Name key = ndn::Name(this->conf.getRouterPrefix()).append(std::to_string(Lsa::Type::ADJACENCY));
  AdjLsa* lsa = nlsr.m_lsdb.findAdjLsa(key);
  BOOST_REQUIRE(lsa == nullptr);

  // Publish a routing update before an Adjacency LSA is built
  BOOST_CHECK_NO_THROW(nlsr.m_lsdb.m_sync.publishRoutingUpdate(Lsa::Type::ADJACENCY, 0));
}

BOOST_AUTO_TEST_SUITE_END()

} // namespace test
} // namespace nlsr
