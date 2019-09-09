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

#include "update/prefix-update-processor.hpp"
#include "nlsr.hpp"

#include "../control-commands.hpp"
#include "../test-common.hpp"

#include <ndn-cxx/mgmt/nfd/control-response.hpp>
#include <ndn-cxx/security/command-interest-signer.hpp>
#include <ndn-cxx/security/pib/identity.hpp>
#include <ndn-cxx/security/signing-helpers.hpp>

#include <boost/filesystem.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/info_parser.hpp>

using namespace ndn;

namespace nlsr {
namespace test {

class PrefixUpdateFixture : public nlsr::test::UnitTestTimeFixture
{
public:
  PrefixUpdateFixture()
    : face(m_ioService, m_keyChain, {true, true})
    , siteIdentityName(ndn::Name("site"))
    , opIdentityName(ndn::Name("site").append(ndn::Name("%C1.Operator")))
    , conf(face)
    , confProcessor(conf)
    , nlsr(face, m_keyChain, conf)
    , namePrefixList(conf.getNamePrefixList())
    , SITE_CERT_PATH(boost::filesystem::current_path() / std::string("site.cert"))
  {
    // Site cert
    siteIdentity = addIdentity(siteIdentityName);
    saveCertificate(siteIdentity, SITE_CERT_PATH.string());

    // Operator cert
    opIdentity = addSubCertificate(opIdentityName, siteIdentity);

    std::ifstream inputFile;
    inputFile.open(std::string("nlsr.conf"));

    BOOST_REQUIRE(inputFile.is_open());

    boost::property_tree::ptree pt;

    boost::property_tree::read_info(inputFile, pt);
    for (const auto& tn : pt) {
      if (tn.first == "security") {
        for (const auto& it : tn.second) {
          if (it.first == "prefix-update-validator") {
            conf.getPrefixUpdateValidator().load(it.second, std::string("nlsr.conf"));
          }
        }
      }
    }
    inputFile.close();

    nlsr.loadCertToPublish(opIdentity.getDefaultKey().getDefaultCertificate());

    addIdentity(conf.getRouterPrefix());

    // Initialize NLSR so a sync socket is created
    nlsr.initialize();

    this->advanceClocks(ndn::time::milliseconds(10));

    face.sentInterests.clear();
  }

  void sendInterestForPublishedData()
  {
    // Need to send an interest now since ChronoSync
    // no longer does face->put(*data) in publishData.
    // Instead it does it in onInterest
    ndn::Name lsaInterestName = conf.getLsaPrefix();
    lsaInterestName.append(conf.getSiteName());
    lsaInterestName.append(conf.getRouterName());
    lsaInterestName.append(std::to_string(Lsa::Type::NAME));

    lsaInterestName.appendNumber(nlsr.m_lsdb.m_sequencingManager.getNameLsaSeq());

    auto lsaInterest = std::make_shared<Interest>(lsaInterestName);
    lsaInterest->setCanBePrefix(true);
    face.receive(*lsaInterest);
    this->advanceClocks(ndn::time::milliseconds(100));
  }

  bool
  wasRoutingUpdatePublished()
  {
    sendInterestForPublishedData();

    const ndn::Name& lsaPrefix = conf.getLsaPrefix();

    const auto& it = std::find_if(face.sentData.begin(), face.sentData.end(),
      [lsaPrefix] (const ndn::Data& data) {
        return lsaPrefix.isPrefixOf(data.getName());
      }
    );

    return (it != face.sentData.end());
  }

public:
  ndn::util::DummyClientFace face;

  ndn::Name siteIdentityName;
  ndn::security::pib::Identity siteIdentity;

  ndn::Name opIdentityName;
  ndn::security::pib::Identity opIdentity;

  ConfParameter conf;
  DummyConfFileProcessor confProcessor;
  Nlsr nlsr;
  NamePrefixList& namePrefixList;

  const boost::filesystem::path SITE_CERT_PATH;
};

BOOST_FIXTURE_TEST_SUITE(TestPrefixUpdateProcessor, PrefixUpdateFixture)

BOOST_AUTO_TEST_CASE(Basic)
{
  uint64_t nameLsaSeqNoBeforeInterest = nlsr.m_lsdb.m_sequencingManager.getNameLsaSeq();

  ndn::nfd::ControlParameters parameters;
  parameters.setName("/prefix/to/advertise/");

  // Control Command format: /<prefix>/<management-module>/<command-verb>/<control-parameters>
  // /<timestamp>/<random-value>/<signed-interests-components>

  // Advertise
  ndn::Name advertiseCommand("/localhost/nlsr/prefix-update/advertise");

  // append /<control-parameters>
  advertiseCommand.append(parameters.wireEncode());

  ndn::security::CommandInterestSigner cis(m_keyChain);

  // CommandInterestSigner::makeCommandInterest() will append the last
  // three components: (<timestamp>/<random-value>/<signed-interests-components>)
  ndn::Interest advertiseInterest =
    cis.makeCommandInterest(advertiseCommand,
                            ndn::security::signingByIdentity(opIdentity));

  face.receive(advertiseInterest);

  this->advanceClocks(ndn::time::milliseconds(10));

  NamePrefixList& namePrefixList = conf.getNamePrefixList();

  BOOST_REQUIRE_EQUAL(namePrefixList.size(), 1);
  BOOST_CHECK_EQUAL(namePrefixList.getNames().front(), parameters.getName());

  BOOST_CHECK(wasRoutingUpdatePublished());
  BOOST_CHECK(nameLsaSeqNoBeforeInterest < nlsr.m_lsdb.m_sequencingManager.getNameLsaSeq());

  face.sentData.clear();
  nameLsaSeqNoBeforeInterest = nlsr.m_lsdb.m_sequencingManager.getNameLsaSeq();

  //Withdraw
  ndn::Name withdrawCommand("/localhost/nlsr/prefix-update/withdraw");
  withdrawCommand.append(parameters.wireEncode());

  ndn::Interest withdrawInterest
    = cis.makeCommandInterest(withdrawCommand,
                              ndn::security::signingByIdentity(opIdentity));

  face.receive(withdrawInterest);
  this->advanceClocks(ndn::time::milliseconds(10));

  BOOST_CHECK_EQUAL(namePrefixList.size(), 0);

  BOOST_CHECK(wasRoutingUpdatePublished());
  BOOST_CHECK(nameLsaSeqNoBeforeInterest < nlsr.m_lsdb.m_sequencingManager.getNameLsaSeq());
}

BOOST_AUTO_TEST_SUITE_END()

} // namespace test
} // namespace nlsr
