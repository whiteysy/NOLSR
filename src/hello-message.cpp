/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/**
 * Copyright (c) 2014-2018,  The University of Memphis,
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

#include "hello-message.hpp"
#include "logger.hpp"
#include "common.hpp"

#include <iostream>
#include <sstream>
#include <string>
#include <cmath>
#include <limits>

namespace nlsr {

INIT_LOGGER(Neighbor);


Neighbor::Neighbor()
    : m_name()
    , m_mpr()
{
}

Neighbor::Neighbor(const ndn::Name& an, const std::string& mpr)
    : m_name(an)
    , m_mpr(mpr)
{
}

bool
Neighbor::operator==(const Neighbor& neighbor) const
{
  return (m_name == neighbor.getName()) &&
         (m_mpr == neighbor.getMpr());
}

std::ostream&
operator<<(std::ostream& os, const Neighbor& neighbor)
{
  os << "Neighbor: " << neighbor.m_name << "\n Mpr: " << neighbor.m_mpr << std::endl;
  return os;
}

void
Neighbor::writeLog()
{
  NLSR_LOG_DEBUG(*this);
}




//*******************************************************************************************//
//*******************************************************************************************//


HelloMessage::HelloMessage()
{
}

HelloMessage::HelloMessage(uint32_t willingness)
             :m_willingness(willingness)
             ,m_neighborList()
{
}

HelloMessage::~HelloMessage()
{
}

//format: <size()>|<this router's wiilingness>|<neighbor 1>|<mpr 1>|....|<neighbor n>|<mpr n>|
std::string
HelloMessage::serialize() const
{
  std::ostringstream os;
  os << m_neighborList.size() << "|" << m_willingness;
  if (m_neighborList.size() > 0)
  {
    for (const auto& neighbor : m_neighborList)
    {
      os << "|" << neighbor.getName() << "|" << neighbor.getMpr();
    }
  }
  os << "|";
  return os.str();
}

bool
HelloMessage::deserialize(const std::string& content)
{
  uint32_t numNeighbor = 0;
  uint32_t willingness = 0;
  boost::char_separator<char> sep("|");
  boost::tokenizer<boost::char_separator<char> >tokens(content, sep);
  boost::tokenizer<boost::char_separator<char> >::iterator tok_iter =
                                               tokens.begin();

  try {
    numNeighbor = boost::lexical_cast<uint32_t>(*tok_iter++);
    willingness = boost::lexical_cast<uint32_t>(*tok_iter++);
    setWillingness(willingness);
    for (uint32_t i = 0; i < numNeighbor; i++) {
      ndn::Name neighborName(*tok_iter++);
      std::string mpr(*tok_iter++);
      Neighbor neighbor(neighborName, mpr);
      insert(neighbor);
    }
  }
  catch (const std::exception& e) {
    NLSR_LOG_ERROR("Could not deserialize from content: " << e.what());
    return false;
  }
  return true;
}

int32_t
HelloMessage::insert(Neighbor& neighbor)
{
  std::list<Neighbor>::iterator it = find(neighbor.getName());
  if (it != m_neighborList.end()) {
    return -1;
  }
  m_neighborList.push_back(neighbor);
  return 0;
}

bool
HelloMessage::isNeighbor(const ndn::Name& neighborName) const
{
  std::list<Neighbor>::const_iterator it = find(neighborName);
  if (it == m_neighborList.end())
  {
    return false;
  }
  return true;
}

std::list<Neighbor>&
HelloMessage::getNeighborList()
{
  return m_neighborList;
}

const std::list<Neighbor>&
HelloMessage::getNeighborList() const
{
  return m_neighborList;
}

std::list<Neighbor>::iterator
HelloMessage::find(const ndn::Name& neighborName)
{
  std::list<Neighbor>::iterator it = std::find_if(m_neighborList.begin(),
                                                  m_neighborList.end(),
                                                  std::bind(&Neighbor::compareNeighborName,
                                                            _1, std::cref(neighborName)));
  return it;
}

std::list<Neighbor>::const_iterator
HelloMessage::find(const ndn::Name& neighborName) const
{
  std::list<Neighbor>::const_iterator it = std::find_if(m_neighborList.cbegin(),
                                                        m_neighborList.cend(),
                                                        std::bind(&Neighbor::compareNeighborName,
                                                                  _1, std::cref(neighborName)));
  return it;
}

//考虑是否留下这个函数，如果hello协议中用不到就删了这个函数
HelloMessage::iterator
HelloMessage::findNeighbor(const ndn::Name& neighborName)
{
  return std::find_if(m_neighborList.begin(),
                      m_neighborList.end(),
                      std::bind(&Neighbor::compareNeighborName,
                                _1, std::cref(neighborName)));
}

void
HelloMessage::writeLog()
{
  NLSR_LOG_DEBUG("-------Neigbor List--------");
  for (std::list<Neighbor>::iterator it = m_neighborList.begin();
       it != m_neighborList.end(); it++) {
    (*it).writeLog();
  }
}

} // namespace nlsr
