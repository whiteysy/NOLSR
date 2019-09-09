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

#ifndef NLSR_TLV_LSDB_STATUS_HPP
#define NLSR_TLV_LSDB_STATUS_HPP

#include "adjacency-lsa.hpp"
#include "coordinate-lsa.hpp"
#include "name-lsa.hpp"

#include <ndn-cxx/util/time.hpp>
#include <ndn-cxx/encoding/block.hpp>
#include <ndn-cxx/encoding/encoding-buffer.hpp>
#include <ndn-cxx/encoding/tlv.hpp>
#include <ndn-cxx/name.hpp>

#include <list>

namespace nlsr {
namespace tlv {

/*! \brief Data abstraction for LsdbStatus
 *
 *  LsdbStatus := LSDB-STATUS-TYPE TLV-LENGTH
 *                  AdjacencyLsa*
 *                  CoordinateLsa*
 *                  NameLsa*
 *
 * \sa https://redmine.named-data.net/projects/nlsr/wiki/LSDB_DataSet
 */
class LsdbStatus
{
public:
  class Error : public ndn::tlv::Error
  {
  public:
    explicit
    Error(const std::string& what)
      : ndn::tlv::Error(what)
    {
    }
  };

  typedef std::list<AdjacencyLsa> AdjacencyLsaList;
  typedef std::list<CoordinateLsa> CoordinateLsaList;
  typedef std::list<NameLsa> NameLsaList;

  LsdbStatus();

  explicit
  LsdbStatus(const ndn::Block& block);

  const std::list<AdjacencyLsa>&
  getAdjacencyLsas() const
  {
    return m_adjacencyLsas;
  }

  LsdbStatus&
  addAdjacencyLsa(const AdjacencyLsa& adjacencyLsa);

  LsdbStatus&
  clearAdjacencyLsas();

  bool
  hasAdjacencyLsas()
  {
    return m_hasAdjacencyLsas;
  }

  const std::list<CoordinateLsa>&
  getCoordinateLsas() const
  {
    return m_coordinateLsas;
  }

  LsdbStatus&
  addCoordinateLsa(const CoordinateLsa& coordinateLsa);

  LsdbStatus&
  clearCoordinateLsas();

  bool
  hasCoordinateLsas()
  {
    return m_hasCoordinateLsas;
  }

  const std::list<NameLsa>&
  getNameLsas() const
  {
    return m_nameLsas;
  }

  LsdbStatus&
  addNameLsa(const NameLsa& nameLsa);

  LsdbStatus&
  clearNameLsas();

  bool
  hasNameLsas()
  {
    return m_hasNameLsas;
  }

  /*! \brief Encodes the LSA objects and some info for each LSA using
   * the method in TAG.
   *
   * This function will TLV-format the LSA objects and some LSA
   * info using the implementation specified by TAG. Usually this is
   * called with an estimator first to guess how long the buffer needs
   * to be, then with an encoder to do the real work. This process is
   * automated by the other wireEncode.
   * \sa LsdbStatus::wireEncode()
   */
  template<ndn::encoding::Tag TAG>
  size_t
  wireEncode(ndn::EncodingImpl<TAG>& block) const;

  /*! \brief Create a TLV encoding of this object.
   *
   * Create a block containing the TLV encoding of this object. That
   * involves two steps: estimating the size that the information will
   * take up, and then creating a buffer of that size and encoding the
   * information into it. Both steps are accomplished by
   * LsdbStatus::wireEncode(ndn::EncodingImpl<TAG>&)
   */
  const ndn::Block&
  wireEncode() const;

  /*! \brief Populate this object by decoding the one contained in the
   * given block.
   */
  void
  wireDecode(const ndn::Block& wire);

private:
  AdjacencyLsaList m_adjacencyLsas;
  CoordinateLsaList m_coordinateLsas;
  NameLsaList m_nameLsas;

  bool m_hasAdjacencyLsas;
  bool m_hasCoordinateLsas;
  bool m_hasNameLsas;

  mutable ndn::Block m_wire;
};

NDN_CXX_DECLARE_WIRE_ENCODE_INSTANTIATIONS(LsdbStatus);

std::ostream&
operator<<(std::ostream& os, const LsdbStatus& lsdbStatus);

} // namespace tlv
} // namespace nlsr

#endif // NLSR_TLV_LSDB_STATUS_HPP
