
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

/// \brief	This header file declares and defines internal state of an OLSR node.

#ifndef NLSR_TUPLESTATE_H
#define NLSR_TUPLESTATE_H

#include "tuple.hpp"


namespace nlsr {
using namespace std;
/// \ingroup olsr
/// This class encapsulates all data structures needed for maintaining internal state of an OLSR node.
class TupleState{
  //  friend class Olsr;

public:
  
  NeighborSet m_neighborSet;            //!< Neighbor Set (\RFC{3626}, section 4.3.1).
  TwoHopNeighborSet m_twoHopNeighborSet;        //!< 2-hop Neighbor Set (\RFC{3626}, section 4.3.2).
  TopologySet m_topologySet;    //!< Topology Set (\RFC{3626}, section 4.4).
  MprSet m_mprSet;
  MprSelectorSet m_mprSelectorSet; 
  uint32_t m_willingness;
  enum willingness
  {
    WILL_NEVER,WILL_LOW,WILL_DEFAULT,WILL_HIGH,WILL_ALWAYS
  };


public:
  TupleState ()
  {
    m_willingness = WILL_DEFAULT;
  }

  // Neighbor

  /**
   * Gets the neighbor set.
   * \returns The neighbor set.
   */
  void SetWillingness(uint32_t willingness)
  {
    m_willingness = willingness;
  }

  const NeighborSet & GetNeighbors () const
  {
    return m_neighborSet;
  }
  /**
   * Gets the neighbor set.
   * \returns The neighbor set.
   */
  NeighborSet & GetNeighbors ()
  {
    return m_neighborSet;
  }

  /**
   * Finds a neighbor tuple.
   * \param mainAddr The neighbor tuple main address.
   * \returns The neighbor tuple, if found. Else it returns a null pointer.
   */
  NeighborTuple* FindNeighborTuple (const ndn::Name &neighbor);

  /**
   * Erases a neighbor tuple.
   * \param neighborTuple The neighbor tuple.
   */
  void EraseNeighborTuple (const ndn::Name &neighbor);

  /**
   * Inserts a neighbor tuple.
   * \param tuple The neighbor tuple.
   */
  void InsertNeighborTuple (NeighborTuple const &tuple);

  // Two-hop neighbor

  /**
   * Gets the 2-hop neighbor set.
   * \returns The 2-hop neighbor set.
   */
  const TwoHopNeighborSet & GetTwoHopNeighbors () const
  {
    return m_twoHopNeighborSet;
  }
  /**
   * Gets the 2-hop neighbor set.
   * \returns The 2-hop neighbor set.
   */
  TwoHopNeighborSet & GetTwoHopNeighbors ()
  {
    return m_twoHopNeighborSet;
  }

  /**
   * Finds a 2-hop neighbor tuple.
   * \param neighbor The neighbor main address.
   * \param twoHopNeighbor The 2-hop neighbor main address.
   * \returns The 2-hop neighbor tuple, if found. Else it returns a null pointer.
   */
  TwoHopNeighborTuple* FindTwoHopNeighborTuple (const ndn::Name &neighbor,
                                                const ndn::Name &twoHopNeighbor);

  /**
   * Erases a 2-hop neighbor tuple.
   * \param tuple The 2-hop neighbor tuple.
   */
  void EraseTwoHopNeighborTuple (const TwoHopNeighborTuple &tuple);
  /**
   * Erases the 2-hop neighbor tuples with the same 1-hop neighbor.
   * \param neighbor The neighbor address.
   */
  void EraseTwoHopNeighborTuples (const ndn::Name &neighbor);
  /**
   * Erases the 2-hop neighbor tuples with matching predicates.
   * \param neighbor The neighbor address.
   * \param twoHopNeighbor The 2-hop neighbor main address.
   */
  void EraseTwoHopNeighborTuples (const ndn::Name &neighbor,
                                  const ndn::Name &twoHopNeighbor);
  /**
   * Inserts a 2-hop neighbor tuple.
   * \param tuple The 2-hop neighbor tuple.
   */
  void InsertTwoHopNeighborTuple (const TwoHopNeighborTuple &tuple);



  // Topology

  /**
   * Gets the topology set.
   * \returns The topology set.
   */
  const TopologySet & GetTopologySet () const
  {
    return m_topologySet;
  }
  /**
   * Finds a topology tuple.
   * \param destAddr The destination address.
   * \param lastAddr The address of the node previous to the destination.
   * \returns The topology tuple, or a null pointer if no match.
   */
  TopologyTuple* FindTopologyTuple (const ndn::Name &dest,
                                    const ndn::Name &last);
  /**
   * Finds a topology tuple.
   * \param lastAddr The address of the node previous to the destination.
   * \param ansn The Advertised Neighbor Sequence Number.
   * \returns The topology tuple, or a null pointer if no match.
   */
  TopologyTuple* FindNewerTopologyTuple (const ndn::Name &last,
                                         uint16_t ansn);
  /**
   * Erases a topology tuple.
   * \param tuple The tuple to erase.
   */
  void EraseTopologyTuple (const TopologyTuple &tuple);
  /**
   * Erases a topology tuple.
   * \param lastAddr The address of the node previous to the destination.
   * \param ansn The Advertised Neighbor Sequence Number.
   */
  void EraseOlderTopologyTuples (const ndn::Name &last,
                                 uint16_t ansn);

  void EraseTopologyTuples(const ndn::Name &last);
  /**
   * Inserts a topology tuple.
   * \param tuple The tuple to insert.
   */
  void InsertTopologyTuple (const TopologyTuple &tuple);



// Mpr
  void MprComputation();

  MprTuple* FindMprTuple (const ndn::Name &neighbor);


  void EraseMprTuple (const ndn::Name &neighbor);


  void InsertMprTuple (MprTuple const &tuple);

  void SetMprSet(MprSet computedMprSet)
  {
    m_mprSet = computedMprSet;
  }

  int Degree (NeighborTuple const &tuple);

  void CoverTwoHopNeighbors (ndn::Name neighbor, TwoHopNeighborSet & N2);




//MprSelector

  MprSelectorTuple* FindMprSelectorTuple (const ndn::Name &neighbor);

  void EraseMprSelectorTuple (const ndn::Name &neighbor);


  void InsertMprSelectorTuple (MprSelectorTuple const &tuple);


  void writeLog();
};

}  // namespace olsr,ns3

#endif /* NLSR_TUPLESTATE_H */
