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
#include "common.hpp"

#include <list>
#include <string>
#include <cmath>
#include <boost/cstdint.hpp>
#include <boost/tokenizer.hpp>
#include <boost/lexical_cast.hpp>


#ifndef NLSR_HELLO_MESSAGE_HPP
#define NLSR_HELLO_MESSAGE_HPP

namespace nlsr {

/*! \brief A neighbor reachable over a Face.
 *
 * Represents another node that we expect to be running NLSR that we
 * should be able to reach over a direct Face connection.
 */
class Neighbor
{

public:

  Neighbor();

  Neighbor(const ndn::Name& an, const std::string& mpr);

  const ndn::Name&
  getName() const
  {
    return m_name;
  }

  void
  setName(const ndn::Name& an)
  {
    m_name = an;
  }

  const std::string&
  getMpr() const
  {
    return m_mpr;
  }

  void
  setMpr(const std::string& mpr)
  {
    m_mpr = mpr;
  }

  /*! \brief Equality is when name, mpr, and willingness are all equal. */
  bool
  operator==(const Neighbor& neighbor) const;

  bool
  operator!=(const Neighbor& neighbor) const
  {
    return !(*this == neighbor);
  }

  inline bool
  compareNeighborName(const ndn::Name& neighborName) const
  {
    return m_name == neighborName;
  }

  inline bool
  compareMpr(const std::string mpr) const
  {
    return m_mpr == mpr;
  }

  void
  writeLog();



private:
  /*! m_name The NLSR-configured router name of the neighbor */
  ndn::Name m_name;
  /*! m_faceUri The NFD-level specification of the Face*/
  std::string m_mpr;


  friend std::ostream&
  operator<<(std::ostream& os, const Neighbor& neighbor);
};

std::ostream&
operator<<(std::ostream& os, const Neighbor& neighbor);




//*******************************************************************************************//
//*******************************************************************************************//

class HelloMessage
{
public:
  typedef std::list<Neighbor>::const_iterator const_iterator;
  typedef std::list<Neighbor>::iterator iterator;

  HelloMessage();
  HelloMessage(uint32_t willingness);
  ~HelloMessage();

  void
  setWillingness(uint32_t willingness)
  {
    m_willingness = willingness;
  }

  uint32_t
  getWillingness()
  {
    return m_willingness;
  }
  
  std::string
  serialize() const;

  bool
  deserialize(const std::string& content);

  /*! \brief Inserts a neighbor  into the list.

    \param neighbor The neighbor that we want to add to this list.

    \retval 0 Indicates success.
    \retval 1 Indicates failure.

    This function attempts to insert the supplied neighbor into this
    object, which is an neighbor list.
   */
  int32_t
  insert(Neighbor& neighbor);

  std::list<Neighbor>&
  getNeighborList();

  const std::list<Neighbor>&
  getNeighborList() const;

  bool
  isNeighbor(const ndn::Name& neighborName) const;


  size_t
  size() const
  {
    return m_neighborList.size();
  }

  void
  reset()
  {
    if (m_neighborList.size() > 0) {
      m_neighborList.clear();
    }
  }

  HelloMessage::iterator
  findNeighbor(const ndn::Name& neighborName);

  void
  writeLog();

public:
  const_iterator
  begin() const
  {
    return m_neighborList.begin();
  }

  const_iterator
  end() const
  {
    return m_neighborList.end();
  }

private:
  iterator
  find(const ndn::Name& neighborName);

  const_iterator
  find(const ndn::Name& neighborName) const;

private:
  uint32_t m_willingness;
  std::list<Neighbor> m_neighborList;
};

} // namespace nlsr

#endif // NLSR_HELLO_MESSAGE_HPP
