/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2004 Francisco J. Ros
 * Copyright (c) 2007 INESC Porto
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Authors: Francisco J. Ros  <fjrm@dif.um.es>
 *          Gustavo J. A. M. Carneiro <gjc@inescporto.pt>
 */

///
/// \brief Here are defined all data structures needed by an OLSR node.
///

#ifndef NLSR_TUPLE_H
#define NLSR_TUPLE_H

#include <set>
#include <vector>
#include <list>

#include <ndn-cxx/name.hpp>
#include <ndn-cxx/util/scheduler.hpp>

namespace nlsr {

/// A Neighbor Tuple.
struct NeighborTuple
{
  public:
  void
  setExpiringEventId(ndn::scheduler::EventId eid)
  {
    expiringEventId = std::move(eid);
  }

  ndn::scheduler::EventId
  getExpiringEventId() const
  {
    return expiringEventId;
  }

  ndn::Name neighbor;
  ndn::scheduler::EventId expiringEventId;
  uint64_t willingness;
};

static inline bool
operator == (const NeighborTuple &a, const NeighborTuple &b)
{
  return (a.neighbor == b.neighbor);
}

static inline std::ostream&
operator << (std::ostream &os, const NeighborTuple &tuple)
{
  os << "NeighborTuple(neighbor=" << tuple.neighbor << ")";
  return os;
}


/// A 2-hop Tuple.
struct TwoHopNeighborTuple
{
  ndn::Name neighbor;
  ndn::Name twoHopNeighbor;
};

static inline std::ostream&
operator << (std::ostream &os, const TwoHopNeighborTuple &tuple)
{
  os << "TwoHopNeighborTuple(neighbor=" << tuple.neighbor
     << ", twoHopNeighbor=" << tuple.twoHopNeighbor
     << ")";
  return os;
}

static inline bool
operator == (const TwoHopNeighborTuple &a, const TwoHopNeighborTuple &b)
{
  return (a.neighbor == b.neighbor && a.twoHopNeighbor == b.twoHopNeighbor);
}


/// A Topology Tuple
struct TopologyTuple
{
  ndn::Name dest;
  ndn::Name last;
  /// Sequence number.
  uint16_t sequenceNumber;
};

static inline bool
operator == (const TopologyTuple &a, const TopologyTuple &b)
{
  return (a.dest == b.dest && a.last == b.last && a.sequenceNumber == b.sequenceNumber);
}

static inline std::ostream&
operator << (std::ostream &os, const TopologyTuple &tuple)
{
  os << "TopologyTuple(dest=" << tuple.dest
     << ", last=" << tuple.last
     << ", sequenceNumber=" << (int) tuple.sequenceNumber
     << ")";
  return os;
}


struct MprTuple
{
  ndn::Name mprNeighbor;
};


struct MprSelectorTuple
{
  ndn::Name mprSelector;
};



typedef std::list<NeighborTuple>              NeighborSet; //!< Neighbor Set type.
typedef std::list<TwoHopNeighborTuple>        TwoHopNeighborSet; //!< 2-hop Neighbor Set type.
typedef std::list<TopologyTuple>              TopologySet; //!< Topology Set type.
typedef std::list<MprTuple>                   MprSet; //!< Topology Set type.
typedef std::list<MprSelectorTuple>           MprSelectorSet; //!< Topology Set type.


} // namespace nlsr
#endif /* NLSR_TUPLE_H  */
