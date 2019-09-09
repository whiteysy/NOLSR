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

#ifndef NLSR_PUBLISHER_FIXTURE_HPP
#define NLSR_PUBLISHER_FIXTURE_HPP

#include "publisher/dataset-interest-handler.hpp"
#include "nlsr.hpp"

#include "../boost-test.hpp"
#include "../test-common.hpp"

#include <ndn-cxx/util/dummy-client-face.hpp>
#include <ndn-cxx/security/key-chain.hpp>
#include <ndn-cxx/security/pib/identity.hpp>

#include <ndn-cxx/util/io.hpp>

#include <boost/filesystem.hpp>

using namespace ndn;

namespace nlsr {
namespace test {

class PublisherFixture : public BaseFixture
{
public:
  PublisherFixture()
    : face(m_ioService, m_keyChain, {true, true})
    , conf(face)
    , confProcessor(conf)
    , nlsr(face, m_keyChain, conf)
    , lsdb(nlsr.m_lsdb)
    , rt1(nlsr.m_routingTable)
  {
    routerId = addIdentity(conf.getRouterPrefix());

    nlsr.initialize();
    face.processEvents(ndn::time::milliseconds(100));
  }

  void
  checkPrefixRegistered(const Name& prefix)
  {
    bool registerCommandEmitted = false;
    for (const auto& interest : face.sentInterests) {
      if (interest.getName().size() > 4 && interest.getName().get(3) == name::Component("register")) {
        name::Component test = interest.getName().get(4);
        ndn::nfd::ControlParameters params(test.blockFromValue());
        if (params.getName() == prefix) {
          registerCommandEmitted = true;
          break;
        }
      }
    }
    BOOST_CHECK(registerCommandEmitted);
  }

  void
  addAdjacency(AdjLsa& lsa, const std::string& name, const std::string& faceUri, double cost)
  {
    Adjacent adjacency(name, ndn::FaceUri(faceUri), cost, Adjacent::STATUS_ACTIVE, 0, 0);
    lsa.addAdjacent(std::move(adjacency));
  }

  void
  checkTlvLsaInfo(const tlv::LsaInfo& info, Lsa& lsa)
  {
    BOOST_CHECK_EQUAL(info.getOriginRouter(), lsa.getOrigRouter());
    BOOST_CHECK_EQUAL(info.getSequenceNumber(), lsa.getLsSeqNo());
    BOOST_CHECK_LE(info.getExpirationPeriod(), ndn::time::milliseconds(0));
  }

  void
  checkTlvAdjLsa(const ndn::Block& block, AdjLsa& lsa)
  {
    BOOST_CHECK_EQUAL(block.type(), ndn::tlv::nlsr::AdjacencyLsa);

    tlv::AdjacencyLsa tlvLsa;
    BOOST_REQUIRE_NO_THROW(tlvLsa.wireDecode(block));

    checkTlvAdjLsa(tlvLsa, lsa);
  }

  void
  checkTlvAdjLsa(const tlv::AdjacencyLsa& tlvLsa, AdjLsa& lsa)
  {
    checkTlvLsaInfo(tlvLsa.getLsaInfo(), lsa);

    std::list<tlv::Adjacency>::const_iterator it = tlvLsa.getAdjacencies().begin();

    for (const Adjacent& adjacency : lsa.getAdl().getAdjList()) {
      BOOST_CHECK_EQUAL(it->getName(), adjacency.getName());
      BOOST_CHECK_EQUAL(it->getUri(), adjacency.getFaceUri().toString());
      BOOST_CHECK_EQUAL(it->getCost(), adjacency.getLinkCost());
      ++it;
    }
  }

  NextHop
  createNextHop(const std::string& faceUri, double cost)
  {
    NextHop nexthop(faceUri, cost);
    return nexthop;
  }

  CoordinateLsa
  createCoordinateLsa(const std::string& origin, double radius, std::vector<double> angle)
  {
    CoordinateLsa lsa(origin, 1, ndn::time::system_clock::now(),
                      radius, angle);
    return lsa;
  }

  void
  checkTlvCoordinateLsa(const ndn::Block& block, CoordinateLsa& lsa)
  {
    BOOST_CHECK_EQUAL(block.type(), ndn::tlv::nlsr::CoordinateLsa);

    tlv::CoordinateLsa tlvLsa;
    BOOST_REQUIRE_NO_THROW(tlvLsa.wireDecode(block));

    checkTlvCoordinateLsa(tlvLsa, lsa);
  }

  void
  checkTlvCoordinateLsa(const tlv::CoordinateLsa& tlvLsa, CoordinateLsa& lsa)
  {
    checkTlvLsaInfo(tlvLsa.getLsaInfo(), lsa);

    BOOST_CHECK_EQUAL(tlvLsa.getHyperbolicRadius(), lsa.getCorRadius());
    BOOST_CHECK(tlvLsa.getHyperbolicAngle() == lsa.getCorTheta());
  }

  void
  checkTlvNameLsa(const ndn::Block& block, NameLsa& lsa)
  {
    BOOST_CHECK_EQUAL(block.type(), ndn::tlv::nlsr::NameLsa);

    tlv::NameLsa tlvLsa;
    BOOST_REQUIRE_NO_THROW(tlvLsa.wireDecode(block));

    checkTlvNameLsa(tlvLsa, lsa);
  }

  void
  checkTlvNameLsa(const tlv::NameLsa& tlvLsa, NameLsa& lsa)
  {
    checkTlvLsaInfo(tlvLsa.getLsaInfo(), lsa);

    std::list<ndn::Name>::const_iterator it = tlvLsa.getNames().begin();

    for (const ndn::Name& name : lsa.getNpl().getNames()) {
      BOOST_CHECK_EQUAL(*it, name);
      ++it;
    }
  }

public:
  ndn::util::DummyClientFace face;
  ConfParameter conf;
  DummyConfFileProcessor confProcessor;
  Nlsr nlsr;
  Lsdb& lsdb;

  ndn::security::pib::Identity routerId;
  RoutingTable& rt1;
};

} // namespace test
} // namespace nlsr

#endif // NLSR_PUBLISHER_FIXTURE_HPP
