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

#include "lsdb.hpp"

#include "test-common.hpp"
#include "nlsr.hpp"
#include "lsa.hpp"
#include "name-prefix-list.hpp"
#include <boost/test/unit_test.hpp>

#include <ndn-cxx/util/dummy-client-face.hpp>
#include <ndn-cxx/util/segment-fetcher.hpp>

#include <unistd.h>

namespace nlsr {
namespace test {

using namespace ndn::time_literals;

class LsdbFixture : public UnitTestTimeFixture
{
public:
  LsdbFixture()
    : face(m_ioService, m_keyChain, {true, true})
    , conf(face)
    , confProcessor(conf)
    , nlsr(face, m_keyChain, conf)
    , lsdb(nlsr.m_lsdb)
    , REGISTER_COMMAND_PREFIX("/localhost/nfd/rib")
    , REGISTER_VERB("register")
  {
    addIdentity("/ndn/site/%C1.Router/this-router");

    nlsr.initialize();

    advanceClocks(10_ms);
    face.sentInterests.clear();
  }

  void
  extractParameters(ndn::Interest& interest, ndn::Name::Component& verb,
                    ndn::nfd::ControlParameters& extractedParameters)
  {
    const ndn::Name& name = interest.getName();
    verb = name[REGISTER_COMMAND_PREFIX.size()];
    const ndn::Name::Component& parameterComponent = name[REGISTER_COMMAND_PREFIX.size() + 1];

    ndn::Block rawParameters = parameterComponent.blockFromValue();
    extractedParameters.wireDecode(rawParameters);
  }

  void
  areNamePrefixListsEqual(NamePrefixList& lhs, NamePrefixList& rhs)
  {

    typedef std::list<ndn::Name> NameList;

    NameList lhsList = lhs.getNames();
    NameList rhsList = rhs.getNames();

    BOOST_REQUIRE_EQUAL(lhsList.size(), rhsList.size());

    NameList::iterator i = lhsList.begin();
    NameList::iterator j = rhsList.begin();

    for (; i != lhsList.end(); ++i, ++j) {
      BOOST_CHECK_EQUAL(*i, *j);
    }
  }

public:
  ndn::util::DummyClientFace face;
  ConfParameter conf;
  DummyConfFileProcessor confProcessor;
  Nlsr nlsr;
  Lsdb& lsdb;

  ndn::Name REGISTER_COMMAND_PREFIX;
  ndn::Name::Component REGISTER_VERB;
};

BOOST_FIXTURE_TEST_SUITE(TestLsdb, LsdbFixture)

BOOST_AUTO_TEST_CASE(LsdbSync)
{
  ndn::Name interestName("/ndn/NLSR/LSA/cs/%C1.Router/router2/name");
  uint64_t oldSeqNo = 82;

  ndn::Name oldInterestName = interestName;
  oldInterestName.appendNumber(oldSeqNo);

  lsdb.expressInterest(oldInterestName, 0);
  advanceClocks(10_ms);

  std::vector<ndn::Interest>& interests = face.sentInterests;

  BOOST_REQUIRE(interests.size() > 0);

  bool didFindInterest = false;
  for (const auto& interest : interests) {
    didFindInterest = didFindInterest || interest.getName() == oldInterestName;
  }

  BOOST_CHECK(didFindInterest);
  interests.clear();

  ndn::time::steady_clock::TimePoint deadline = ndn::time::steady_clock::now() +
    ndn::time::seconds(LSA_REFRESH_TIME_MAX);

  // Simulate an LSA interest timeout
  lsdb.onFetchLsaError(ndn::util::SegmentFetcher::ErrorCode::INTEREST_TIMEOUT, "Timeout",
                       oldInterestName, 0, deadline, interestName, oldSeqNo);
  advanceClocks(10_ms);

  BOOST_REQUIRE(interests.size() > 0);

  didFindInterest = false;
  for (const auto& interest : interests) {
    didFindInterest = didFindInterest || interest.getName() == oldInterestName;
  }

  BOOST_CHECK(didFindInterest);
  interests.clear();

  uint64_t newSeqNo = 83;

  ndn::Name newInterestName = interestName;
  newInterestName.appendNumber(newSeqNo);

  lsdb.expressInterest(newInterestName, 0);
  advanceClocks(10_ms);

  BOOST_REQUIRE(interests.size() > 0);

  didFindInterest = false;
  for (const auto& interest : interests) {
    didFindInterest = didFindInterest || interest.getName() == newInterestName;
  }

  BOOST_CHECK(didFindInterest);

  interests.clear();

  // Simulate an LSA interest timeout where the sequence number is outdated
  lsdb.onFetchLsaError(ndn::util::SegmentFetcher::ErrorCode::INTEREST_TIMEOUT, "Timeout",
                       oldInterestName, 0, deadline, interestName, oldSeqNo);
  advanceClocks(10_ms);

  // Interest should not be expressed for outdated sequence number
  BOOST_CHECK_EQUAL(interests.size(), 0);
}

BOOST_AUTO_TEST_CASE(LsdbSegmentedData)
{
  // Add a lot of NameLSAs to exceed max packet size
  ndn::Name lsaKey("/ndn/site/%C1.Router/this-router/NAME");

  NameLsa* nameLsa = lsdb.findNameLsa(lsaKey);
  uint64_t seqNo = nameLsa->getLsSeqNo();

  ndn::Name prefix("/ndn/edu/memphis/netlab/research/nlsr/test/prefix/");

  int nPrefixes = 0;
  while (nameLsa->serialize().size() < ndn::MAX_NDN_PACKET_SIZE) {
    nameLsa->addName(ndn::Name(prefix).appendNumber(++nPrefixes));
  }
  lsdb.installNameLsa(*nameLsa);

  // Create another Lsdb and expressInterest
  ndn::util::DummyClientFace face2(m_ioService, m_keyChain, {true, true});
  face.linkTo(face2);
  ConfParameter conf2(face2);
  Nlsr nlsr2(face2, m_keyChain, conf2);
  std::string config = R"CONF(
              trust-anchor
                {
                  type any
                }
            )CONF";
  conf2.getValidator().load(config, "config-file-from-string");

  Lsdb& lsdb2(nlsr2.m_lsdb);

  advanceClocks(ndn::time::milliseconds(1), 10);

  ndn::Name interestName("/localhop/ndn/nlsr/LSA/site/%C1.Router/this-router/NAME");
  interestName.appendNumber(seqNo);
  // 0 == timeout count
  lsdb2.expressInterest(interestName, 0);

  advanceClocks(ndn::time::milliseconds(1), 10);

  BOOST_CHECK_EQUAL(lsdb.getNameLsdb().front().getNpl(), lsdb2.getNameLsdb().front().getNpl());
}

BOOST_AUTO_TEST_CASE(SegmentLsaData)
{
  ndn::Name lsaKey("/ndn/site/%C1.Router/this-router/NAME");

  NameLsa* lsa = lsdb.findNameLsa(lsaKey);
  uint64_t seqNo = lsa->getLsSeqNo();

  ndn::Name prefix("/ndn/edu/memphis/netlab/research/nlsr/test/prefix/");

  int nPrefixes = 0;
  while (lsa->serialize().size() < ndn::MAX_NDN_PACKET_SIZE) {
    lsa->addName(ndn::Name(prefix).appendNumber(++nPrefixes));
  }
  lsdb.installNameLsa(*lsa);

  std::string expectedDataContent = lsa->serialize();

  ndn::Name interestName("/localhop/ndn/nlsr/LSA/site/%C1.Router/this-router/NAME/");
  interestName.appendNumber(seqNo);

  ndn::util::DummyClientFace face2(m_ioService, m_keyChain, {true, true});
  face.linkTo(face2);

  auto fetcher = ndn::util::SegmentFetcher::start(face2, ndn::Interest(interestName),
                                                  ndn::security::v2::getAcceptAllValidator());
  fetcher->onComplete.connect([&expectedDataContent] (ndn::ConstBufferPtr bufferPtr) {
                                ndn::Block block(bufferPtr);
                                BOOST_CHECK_EQUAL(expectedDataContent, readString(block));
                              });

  advanceClocks(ndn::time::milliseconds(1), 100);
  fetcher->stop();
}

BOOST_AUTO_TEST_CASE(ReceiveSegmentedLsaData)
{
  ndn::Name router("/ndn/cs/%C1.Router/router1");
  uint64_t seqNo = 12;
  NamePrefixList prefixList;

  NameLsa lsa(router, seqNo, ndn::time::system_clock::now(), prefixList);

  ndn::Name prefix("/prefix/");

  for (int nPrefixes = 0; nPrefixes < 3; ++nPrefixes) {
    lsa.addName(ndn::Name(prefix).appendNumber(nPrefixes));
  }

  ndn::Name interestName("/localhop/ndn/nlsr/LSA/cs/%C1.Router/router1/NAME/");
  interestName.appendNumber(seqNo);

  ndn::Block block = ndn::encoding::makeStringBlock(ndn::tlv::Content, lsa.serialize());
  lsdb.afterFetchLsa(block.getBuffer(), interestName);

  NameLsa* foundLsa = lsdb.findNameLsa(lsa.getKey());
  BOOST_REQUIRE(foundLsa != nullptr);

  BOOST_CHECK_EQUAL(foundLsa->serialize(), lsa.serialize());
}

BOOST_AUTO_TEST_CASE(LsdbRemoveAndExists)
{
  ndn::time::system_clock::TimePoint testTimePoint =  ndn::time::system_clock::now();
  NamePrefixList npl1;

  std::string s1 = "name1";
  std::string s2 = "name2";
  std::string router1 = "router1/1";

  npl1.insert(s1);
  npl1.insert(s2);

  // For NameLsa lsType is name.
  // 12 is seqNo, randomly generated.
  // 1800 seconds is the default life time.
  NameLsa nlsa1(ndn::Name("/router1/1"), 12, testTimePoint, npl1);

  Lsdb& lsdb1(nlsr.m_lsdb);

  lsdb1.installNameLsa(nlsa1);
  lsdb1.writeNameLsdbLog();

  BOOST_CHECK(lsdb1.doesLsaExist(ndn::Name("/router1/1/NAME"), Lsa::Type::NAME));

  lsdb1.removeNameLsa(router1);

  BOOST_CHECK_EQUAL(lsdb1.doesLsaExist(ndn::Name("/router1/1"), Lsa::Type::NAME), false);
}

BOOST_AUTO_TEST_CASE(InstallNameLsa)
{
  // Install lsa with name1 and name2
  ndn::Name name1("/ndn/name1");
  ndn::Name name2("/ndn/name2");

  NamePrefixList prefixes;
  prefixes.insert(name1);
  prefixes.insert(name2);

  std::string otherRouter("/ndn/site/%C1.router/other-router");
  ndn::time::system_clock::TimePoint MAX_TIME = ndn::time::system_clock::TimePoint::max();

  NameLsa lsa(otherRouter, 1, MAX_TIME, prefixes);
  lsdb.installNameLsa(lsa);

  BOOST_REQUIRE_EQUAL(lsdb.doesLsaExist(otherRouter + "/NAME", Lsa::Type::NAME), true);
  NamePrefixList& nameList = lsdb.findNameLsa(otherRouter + "/NAME")->getNpl();

  BOOST_CHECK_EQUAL(nameList, prefixes);
  //areNamePrefixListsEqual(nameList, prefixes);

  // Add a prefix: name3
  ndn::Name name3("/ndn/name3");
  prefixes.insert(name3);

  NameLsa addLsa(otherRouter, 2, MAX_TIME, prefixes);
  lsdb.installNameLsa(addLsa);

  // Lsa should include name1, name2, and name3
  BOOST_CHECK_EQUAL(nameList, prefixes);

  // Remove a prefix: name2
  prefixes.remove(name2);

  NameLsa removeLsa(otherRouter, 3, MAX_TIME, prefixes);
  lsdb.installNameLsa(removeLsa);

  // Lsa should include name1 and name3
  BOOST_CHECK_EQUAL(nameList, prefixes);

  // Add and remove a prefix: add name2, remove name3
  prefixes.insert(name2);
  prefixes.remove(name3);

  NameLsa addAndRemoveLsa(otherRouter, 4, MAX_TIME, prefixes);
  lsdb.installNameLsa(addAndRemoveLsa);

  // Lsa should include name1 and name2
  BOOST_CHECK_EQUAL(nameList, prefixes);

  // Install a completely new list of prefixes
  ndn::Name name4("/ndn/name4");
  ndn::Name name5("/ndn/name5");

  NamePrefixList newPrefixes;
  newPrefixes.insert(name4);
  newPrefixes.insert(name5);

  NameLsa newLsa(otherRouter, 5, MAX_TIME, newPrefixes);
  lsdb.installNameLsa(newLsa);

  // Lsa should include name4 and name5
  BOOST_CHECK_EQUAL(nameList, newPrefixes);
}

BOOST_AUTO_TEST_CASE(TestIsLsaNew)
{
  const ndn::Name::Component CONFIG_NETWORK{"/ndn"};
  const ndn::Name::Component CONFIG_SITE{"/memphis"};
  ndn::Name originRouter{};
  originRouter.append(CONFIG_NETWORK).append(CONFIG_SITE).append("/%C1.Router/other-router");

  // Install Name LSA
  NamePrefixList nameList;
  NameLsa lsa(originRouter, 999, ndn::time::system_clock::TimePoint::max(), nameList);

  lsdb.installNameLsa(lsa);

  // Lower NameLSA sequence number
  uint64_t lowerSeqNo = 998;
  BOOST_CHECK(!lsdb.isLsaNew(originRouter, Lsa::Type::NAME, lowerSeqNo));

  // Same NameLSA sequence number
  uint64_t sameSeqNo = 999;
  BOOST_CHECK(!lsdb.isLsaNew(originRouter, Lsa::Type::NAME, sameSeqNo));

  // Higher NameLSA sequence number
  uint64_t higherSeqNo = 1000;
  BOOST_CHECK(lsdb.isLsaNew(originRouter, Lsa::Type::NAME, higherSeqNo));
}

BOOST_AUTO_TEST_SUITE_END() // TestLsdb

} // namespace test
} // namespace nlsr
