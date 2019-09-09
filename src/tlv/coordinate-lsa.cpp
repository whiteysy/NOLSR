/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2014-2018,  The University of Memphis,
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
 */

#include "coordinate-lsa.hpp"
#include "tlv-nlsr.hpp"

#include <ndn-cxx/util/concepts.hpp>
#include <ndn-cxx/encoding/block-helpers.hpp>

#include <iostream>

namespace nlsr {
namespace tlv {

BOOST_CONCEPT_ASSERT((ndn::WireEncodable<CoordinateLsa>));
BOOST_CONCEPT_ASSERT((ndn::WireDecodable<CoordinateLsa>));
static_assert(std::is_base_of<ndn::tlv::Error, CoordinateLsa::Error>::value,
              "CoordinateLsa::Error must inherit from tlv::Error");

CoordinateLsa::CoordinateLsa()
  : m_hyperbolicRadius(0.0)
{
}

CoordinateLsa::CoordinateLsa(const ndn::Block& block)
{
  wireDecode(block);
}

template<ndn::encoding::Tag TAG>
size_t
CoordinateLsa::wireEncode(ndn::EncodingImpl<TAG>& block) const
{
  size_t totalLength = 0;

  for (auto it = m_hyperbolicAngle.rbegin(); it != m_hyperbolicAngle.rend(); ++it) {
    totalLength += prependDouble(block, ndn::tlv::nlsr::HyperbolicAngle, *it);
  }

  totalLength += prependDouble(block, ndn::tlv::nlsr::HyperbolicRadius, m_hyperbolicRadius);

  totalLength += m_lsaInfo.wireEncode(block);

  totalLength += block.prependVarNumber(totalLength);
  totalLength += block.prependVarNumber(ndn::tlv::nlsr::CoordinateLsa);

  return totalLength;
}

NDN_CXX_DEFINE_WIRE_ENCODE_INSTANTIATIONS(CoordinateLsa);

const ndn::Block&
CoordinateLsa::wireEncode() const
{
  if (m_wire.hasWire()) {
    return m_wire;
  }

  ndn::EncodingEstimator estimator;
  size_t estimatedSize = wireEncode(estimator);

  ndn::EncodingBuffer buffer(estimatedSize, 0);
  wireEncode(buffer);

  m_wire = buffer.block();

  return m_wire;
}

void
CoordinateLsa::wireDecode(const ndn::Block& wire)
{
  m_hyperbolicRadius = 0.0;
  m_hyperbolicAngle.clear();

  m_wire = wire;

  if (m_wire.type() != ndn::tlv::nlsr::CoordinateLsa) {
    std::stringstream error;
    error << "Expected CoordinateLsa Block, but Block is of a different type: #"
          << m_wire.type();
    BOOST_THROW_EXCEPTION(Error(error.str()));
  }

  m_wire.parse();

  ndn::Block::element_const_iterator val = m_wire.elements_begin();

  if (val != m_wire.elements_end() && val->type() == ndn::tlv::nlsr::LsaInfo) {
    m_lsaInfo.wireDecode(*val);
    ++val;
  }
  else {
    std::cout << "Missing required LsaInfo field" << std::endl;
    BOOST_THROW_EXCEPTION(Error("Missing required LsaInfo field"));
  }

  if (val != m_wire.elements_end() && val->type() == ndn::tlv::nlsr::HyperbolicRadius) {
    m_hyperbolicRadius = ndn::tlv::nlsr::readDouble(*val);
    ++val;
  }
  else {
    std::cout << "Missing required HyperbolicRadius field" << std::endl;
    BOOST_THROW_EXCEPTION(Error("Missing required HyperbolicRadius field"));
  }

  for (; val != m_wire.elements_end(); ++val) {
    if (val->type() == ndn::tlv::nlsr::HyperbolicAngle) {
      m_hyperbolicAngle.push_back(ndn::tlv::nlsr::readDouble(*val));
    }
  }
}

std::ostream&
operator<<(std::ostream& os, const CoordinateLsa& coordinateLsa)
{
  os << "CoordinateLsa("
     << coordinateLsa.getLsaInfo() << ", "
     << "HyperbolicRadius: " << coordinateLsa.getHyperbolicRadius() << ", ";

  os << "HyperbolicAngles: ";
  int i = 0;
  for (const auto& value: coordinateLsa.getHyperbolicAngle()) {
    if (i == 0) {
      os << value;
    }
    else {
      os << ", " << value;
    }
    ++i;
  }
  os << ")";

  return os;
}

} // namespace tlv
} // namespace nlsr
