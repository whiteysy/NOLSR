/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/**
 * Copyright (c) 2014-2019,  The University of Memphis,
 *                           Regents of the University of California
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

#ifndef NLSR_CONF_FILE_PROCESSOR_HPP
#define NLSR_CONF_FILE_PROCESSOR_HPP

#include "common.hpp"
#include "conf-parameter.hpp"

#include <boost/algorithm/string.hpp>
#include <boost/property_tree/info_parser.hpp>
#include <boost/filesystem.hpp>

namespace nlsr {

/*! \brief A class containing methods to parse an NLSR configuration file
 *
 * This class contains methods to parse an NLSR configuration file and
 * set all the parameters in NLSR to the received values. There are
 * defaults for any unconfigured settings.
 *
 * This is currently called by the wrapper class NlsrRunner to
 * populate the NLSR object with its configuration before NLSR is
 * started.
 *
 * \sa nlsr::ConfParameter
 * \sa NlsrRunner::run
 */
class ConfFileProcessor
{
public:
  ConfFileProcessor(ConfParameter& confParam);

  /*! \brief Load and parse the configuration file, then populate NLSR.
   *
   * Entry-point function that chains all the necessary steps together
   * to configure an NLSR object.
   *
   * \return A boolean for whether configuration was successful.
   */
  bool
  processConfFile();

private:
  typedef boost::property_tree::ptree ConfigSection;

  /*! \brief Parse the configuration file into a tree and process the nodes.
   *
   * Reads the configuration file as a property tree, and then iterates
   * over them, attempting to parse each node. The nodes themselves
   * are passed to another function that determines what kind of
   * section it is and handles it appropriately. On any error in
   * reading the file, return false.
   *
   * \return Whether configuration was successful.
   */
  bool
  load(std::istream& input);

  /*! \brief A dispatcher-like function to send configuration tree nodes to the right subfunction.
   */
  bool
  processSection(const std::string& sectionName, const ConfigSection& section);

  /*! \brief Parse general options, including router name, LSA refresh.
   */
  bool
  processConfSectionGeneral(const ConfigSection& section);

  /*! \brief Configure options relating to neighbor configuration and detection.
   *
   * Parse options that control NLSR's behavior about neighbors. Such
   * things include how many hello interests are sent and what their
   * timeout is, as well as parsing neighbor specifications. Neighbor
   * Face URIs are parsed and confirmed as valid here by ndn-cxx.
   */
  bool
  processConfSectionNeighbors(const ConfigSection& section);

  /*! \brief Set the state of hyperbolic routing: off, on, dry-run.
   */
  bool
  processConfSectionHyperbolic(const ConfigSection& section);

  /*! \brief Set options for the FIB: nexthops per prefix, routing calculation interval.
   */
  bool
  processConfSectionFib(const ConfigSection& section);

  /*! \brief Set prefixes that NLSR is supposed to advertise immediately.
   */
  bool
  processConfSectionAdvertising(const ConfigSection& section);

  /*! \brief Parse and set rules for the validator.
   *
   * This section parses and sets rules for the validators, which
   * control what criteria Interest and Data need to follow to be
   * considered valid by this NLSR.
   */
  bool
  processConfSectionSecurity(const ConfigSection& section);

private:
  /*! m_confFileName The full path of the configuration file to parse. */
  std::string m_confFileName;
  /*! m_confParam The ConfFileProcessor object to configure as parsing is done. */
  ConfParameter& m_confParam;
  /*! m_io For canonization of faceUri. */
  boost::asio::io_service m_io;
};

} // namespace nlsr
#endif // NLSR_CONF_FILE_PROCESSOR_HPP
