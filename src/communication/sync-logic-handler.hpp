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

#ifndef NLSR_SYNC_LOGIC_HANDLER_HPP
#define NLSR_SYNC_LOGIC_HANDLER_HPP

#include "conf-parameter.hpp"
#include "test-access-control.hpp"
#include "signals.hpp"
#include "lsa.hpp"
#include "sync-protocol-adapter.hpp"

#include <ndn-cxx/face.hpp>
#include <ndn-cxx/util/signal.hpp>
#include <boost/throw_exception.hpp>

class InterestManager;

namespace nlsr {

class ConfParameter;

/*! \brief NLSR-to-ChronoSync interaction point
 *
 * This class serves as the abstraction for the syncing portion of
 * NLSR and its components. NLSR has no particular reliance on
 * ChronoSync, except that the NLSR source would need to be modified
 * for use with other sync protocols.
 *
 */
class SyncLogicHandler
{
public:
  using IsLsaNew =
    std::function<bool(const ndn::Name&, const Lsa::Type& lsaType, const uint64_t&)>;

  class Error : public std::runtime_error
  {
  public:
    explicit
    Error(const std::string& what)
      : std::runtime_error(what)
    {
    }
  };

  SyncLogicHandler(ndn::Face& face, const IsLsaNew& isLsaNew, const ConfParameter& conf);

  /*! \brief Instruct ChronoSync to publish an update.
   *
   * This function instructs sync to push an update into the network,
   * based on whatever the state of the sequencing manager is when
   * this is called. Since each ChronoSync instance maintains its own
   * PIT, doing this satisfies those interests so that other routers
   * know a sync update is available.
   * \sa publishSyncUpdate
   */
  void
  publishRoutingUpdate(const Lsa::Type& type, const uint64_t& seqNo);

PUBLIC_WITH_TESTS_ELSE_PRIVATE:
  /*! \brief Create and configure a Logic object to enable Sync for this NLSR.
   *
   * In a typical situation this only needs to be called once, when NLSR starts.
   * \param syncPrefix The sync prefix you want this Sync to use
   * \param syncInterestLifetime ChronoSync/PSync sends a periodic sync interest every
   *        \p syncInterestLifetime / 2 ms
   * \sa Nlsr::initialize
   */
  void
  createSyncLogic(const ndn::Name& syncPrefix,
                  const ndn::time::milliseconds& syncInterestLifetime =
                    ndn::time::milliseconds(SYNC_INTEREST_LIFETIME_DEFAULT));

  /*! \brief Callback from Sync protocol
   *
   * In a typical situation this only needs to be called once, when NLSR starts.
   * \param updateName The prefix for which sync reports an update
   * \param highSeq The latest sequence number of the update
   */
  void
  processUpdate(const ndn::Name& updateName, uint64_t highSeq);

  /*! \brief Simple function to glue Name components together
   */
  void
  buildUpdatePrefix();

  /*! \brief Determine which kind of LSA was updated and fetch it.
   *
   * Checks that the received update is not from us, which can happen,
   * and then inspects the update to determine which kind of LSA the
   * update is for. Finally, it expresses interest for the correct LSA
   * type.
   * \throws SyncUpdate::Error If the sync update doesn't look like a sync LSA update.
   */
  void
  processUpdateFromSync(const ndn::Name& originRouter,
                        const ndn::Name& updateName, uint64_t seqNo);

public:
  std::unique_ptr<OnNewLsa> onNewLsa;

private:
  ndn::Face& m_syncFace;
PUBLIC_WITH_TESTS_ELSE_PRIVATE:
  std::shared_ptr<SyncProtocolAdapter> m_syncLogic;
private:
  IsLsaNew m_isLsaNew;
  const ConfParameter& m_confParam;

PUBLIC_WITH_TESTS_ELSE_PRIVATE:
  ndn::Name m_nameLsaUserPrefix;
  ndn::Name m_adjLsaUserPrefix;
  ndn::Name m_coorLsaUserPrefix;

private:
  static const std::string NLSR_COMPONENT;
  static const std::string LSA_COMPONENT;
};

} // namespace nlsr

#endif // NLSR_SYNC_LOGIC_HANDLER_HPP
