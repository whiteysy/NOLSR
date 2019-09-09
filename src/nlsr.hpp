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

#ifndef NLSR_NLSR_HPP
#define NLSR_NLSR_HPP

#include "adjacency-list.hpp"
#include "common.hpp"
#include "conf-parameter.hpp"
#include "hello-protocol.hpp"
#include "lsdb.hpp"
#include "name-prefix-list.hpp"
#include "test-access-control.hpp"
#include "publisher/dataset-interest-handler.hpp"
#include "route/fib.hpp"
#include "route/name-prefix-table.hpp"
#include "route/routing-table.hpp"
#include "security/certificate-store.hpp"
#include "update/prefix-update-processor.hpp"
#include "update/nfd-rib-command-processor.hpp"
#include "utility/name-helper.hpp"
#include "stats-collector.hpp"
#include "tuple-state.hpp"

#include <boost/cstdint.hpp>
#include <stdexcept>
#include <boost/throw_exception.hpp>

#include <ndn-cxx/face.hpp>
#include <ndn-cxx/security/key-chain.hpp>
#include <ndn-cxx/security/validator-config.hpp>
#include <ndn-cxx/security/v2/certificate-fetcher-direct-fetch.hpp>
#include <ndn-cxx/security/signing-helpers.hpp>
#include <ndn-cxx/security/signing-info.hpp>
#include <ndn-cxx/util/scheduler.hpp>
#include <ndn-cxx/mgmt/nfd/face-event-notification.hpp>
#include <ndn-cxx/mgmt/nfd/face-monitor.hpp>
#include <ndn-cxx/mgmt/dispatcher.hpp>
#include <ndn-cxx/mgmt/nfd/face-status.hpp>
#include <ndn-cxx/data.hpp>
#include <ndn-cxx/encoding/block.hpp>
#include <ndn-cxx/encoding/nfd-constants.hpp>
#include <ndn-cxx/mgmt/nfd/control-parameters.hpp>
#include <ndn-cxx/mgmt/nfd/control-response.hpp>

namespace nlsr {

static ndn::Name DEFAULT_BROADCAST_PREFIX("/ndn/broadcast");

class Nlsr
{
public:
  using FetchDatasetCallback = std::function<void(const std::vector<ndn::nfd::FaceStatus>&)>;
  using FetchDatasetTimeoutCallback = std::function<void(uint32_t, const std::string&)>;

  class Error : public std::runtime_error
  {
  public:
    explicit
    Error(const std::string& what)
      : std::runtime_error(what)
    {
    }
  };

  Nlsr(ndn::Face& face, ndn::KeyChain& keyChain, ConfParameter& confParam);

  void
  registerOriginRouter();

  void
  registerStrategyForCerts(const ndn::Name& originRouter);

  void
  registrationFailed(const ndn::Name& name);

  void
  onRegistrationSuccess(const ndn::Name& name);

  void
  setDataFilter();

  void
  setInfoInterestFilter();

  void
  setLsaInterestFilter();

  /*! \brief Add top level prefixes for Dispatcher
   *
   * All dispatcher-related sub-prefixes *must* be registered before sub-prefixes
   * must be added before adding top
   */
  void
  addDispatcherTopPrefix(const ndn::Name& topPrefix);

  Fib&
  getFib()
  {
    return m_fib;
  }

  void
  initialize();

  /*! \brief Initializes neighbors' Faces using information from NFD.
   * \sa Nlsr::initialize()
   * \sa Nlsr::processFaceDataset()
   *
   * This function serves as the entry-point for initializing the
   * neighbors listed in nlsr.conf during Nlsr::initialize(). NLSR
   * will attempt to fetch a dataset of Faces from NFD, and configure
   * each of its neighbors using information from that dataset. The
   * explicit callbacks allow for better testability.
   */
  void
  initializeFaces(const FetchDatasetCallback& onFetchSuccess,
                  const FetchDatasetTimeoutCallback& onFetchFailure);

  void
  onFaceDatasetFetchTimeout(uint32_t code,
                            const std::string& reason,
                            uint32_t nRetriesSoFar);

  /*! \brief Consumes a Face StatusDataset to configure NLSR neighbors.
   * \sa Nlsr::initializeFaces
   * \param faces A Face Dataset that should conform to FaceMgmt specifications.
   *
   * This function processes a Face StatusDataset that should conform
   * to the FaceMgmt specifications listed
   * [here](https://redmine.named-data.net/projects/nfd/wiki/FaceMgmt#Face-Dataset).
   * Any newly configured neighbors will have prefixes registered with NFD
   * and be sent Hello Interests as well.
   */
  void
  processFaceDataset(const std::vector<ndn::nfd::FaceStatus>& faces);

  /*! \brief Registers NLSR-specific prefixes for a neighbor (Adjacent)
   * \sa Nlsr::initializeFaces
   * \param adj A reference to the neighbor to register prefixes for
   * \param timeout The amount of time to give NFD to respond to *each* registration request.
   *
   * Registers the prefixes in NFD that NLSR needs to route with a
   * neighbor. The timeout given is how long to set the timeout for
   * *each* registration request that is made.
   */
  void
  registerAdjacencyPrefixes(const Adjacent& adj,
                            const ndn::time::milliseconds& timeout);

  /*! \brief Add a certificate NLSR claims to be authoritative for to the certificate store.
   *
   * \sa CertificateStore
   */
  void
  loadCertToPublish(const ndn::security::v2::Certificate& certificate);

  /*! \brief Callback when SegmentFetcher retrieves a segment.
   */
  void
  afterFetcherSignalEmitted(const ndn::Data& lsaSegment);

  /*! \brief Retrieves the chain of certificates from Validator's cache and
   *   store them in Nlsr's own CertificateStore.
   * \param keyName Name of the first key in the certificate chain.
   */
  void
  publishCertFromCache(const ndn::Name& keyName);

  void
  initializeKey();

  /*! \brief Find a certificate
   *
   * Find a certificate that NLSR has. First it checks against the
   * certificates this NLSR claims to be authoritative for, usually
   * something like this specific router's certificate, and then
   * checks the cache of certficates it has already fetched. If none
   * can be found, it will return an empty pointer.
   */
  const ndn::security::v2::Certificate*
  getCertificate(const ndn::Name& certificateKeyName)
  {
    const ndn::security::v2::Certificate* cert =
      m_certStore.find(certificateKeyName);

    return cert;
  }

  void
  setStrategies();

PUBLIC_WITH_TESTS_ELSE_PRIVATE:

  security::CertificateStore&
  getCertificateStore()
  {
    return m_certStore;
  }

private:
  /*! \brief Registers the prefix that NLSR will use for key/certificate interests.
   */
  void
  registerKeyPrefix();

  /*! \brief Registers the prefix that NLSR will consider to be the machine-local, secure prefix.
   */
  void
  registerLocalhostPrefix();

  /*! \brief Registers the <router-prefix>/nlsr so that NLSR can respond to status requests from remote routers.
   */
  void
  registerRouterPrefix();

  /*! \brief Attempts to satisfy an Interest for a certificate, and send it back.
   */
  void
  onKeyInterest(const ndn::Name& name, const ndn::Interest& interest);

  /*! \brief Do nothing.
   */
  void
  onKeyPrefixRegSuccess(const ndn::Name& name);

  /*! \brief Do nothing.
   */
  void
  onFaceEventNotification(const ndn::nfd::FaceEventNotification& faceEventNotification);

  void
  scheduleDatasetFetch();

  /*! \brief Enables NextHopFaceId indication in NFD for incoming data packet.
   *
   * After enabling, when NFD gets a data packet, it will put the incoming face id
   * of the data in NextHopFaceId field of the packet. The NextHopFaceId will be used
   * by DirectFetcher to fetch the certificates needed to validate the data packet.
   * \sa https://redmine.named-data.net/projects/nfd/wiki/NDNLPv2#Consumer-Controlled-Forwarding
   */
  void
  enableIncomingFaceIdIndication();

  void
  onFaceIdIndicationSuccess(const ndn::nfd::ControlParameters& cp);

  void
  onFaceIdIndicationFailure(const ndn::nfd::ControlResponse& cr);

public:
  static const ndn::Name LOCALHOST_PREFIX;

private:
  ndn::Face& m_face;
  TupleState m_state;
  ndn::Scheduler m_scheduler;
  ndn::security::v2::KeyChain& m_keyChain;
  ConfParameter& m_confParam;
  AdjacencyList& m_adjacencyList;
  NamePrefixList& m_namePrefixList;
  bool m_isDaemonProcess;
  ndn::security::ValidatorConfig& m_validator;
  std::vector<ndn::Name> m_strategySetOnRouters;

PUBLIC_WITH_TESTS_ELSE_PRIVATE:
  Fib m_fib;
  RoutingTable m_routingTable;
  NamePrefixTable m_namePrefixTable;
  Lsdb m_lsdb;

private:
  ndn::util::signal::ScopedConnection m_afterSegmentValidatedConnection;
//  ndn::util::signal::ScopedConnection m_onNewLsaConnection;

PUBLIC_WITH_TESTS_ELSE_PRIVATE:
  ndn::mgmt::Dispatcher m_dispatcher;
  DatasetInterestHandler m_datasetHandler;
  HelloProtocol m_helloProtocol;

private:
  /*! \brief Where NLSR stores certificates it claims to be
   * authoritative for. Usually the router certificate.
   */
  security::CertificateStore& m_certStore;

  ndn::nfd::Controller m_controller;
  ndn::nfd::Controller m_faceDatasetController;

PUBLIC_WITH_TESTS_ELSE_PRIVATE:
  ndn::security::SigningInfo m_signingInfo;
  update::PrefixUpdateProcessor m_prefixUpdateProcessor;
  update::NfdRibCommandProcessor m_nfdRibCommandProcessor;

  StatsCollector m_statsCollector;

private:
  ndn::nfd::FaceMonitor m_faceMonitor;

  friend class NlsrRunner;
};

} // namespace nlsr

#endif // NLSR_NLSR_HPP
