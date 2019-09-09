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
/// \file	olsr-state.cc
/// \brief	Implementation of all functions needed for manipulating the internal
///		state of an OLSR node.

///

#include "tuple-state.hpp"



namespace nlsr {

/********** Neighbor Set Manipulation **********/

NeighborTuple*
TupleState::FindNeighborTuple (const ndn::Name &neighbor)
{
  for (NeighborSet::iterator it = m_neighborSet.begin ();
       it != m_neighborSet.end (); it++)
    {
      if (it->neighbor == neighbor)
        {
          return &(*it);
        }
    }
  return NULL;
}

void
TupleState::EraseNeighborTuple (const ndn::Name &neighbor)
{
  for (NeighborSet::iterator it = m_neighborSet.begin ();
       it != m_neighborSet.end (); it++)
    {
      if (it->neighbor == neighbor)
        {
          it = m_neighborSet.erase (it);
          break;
        }
    }
}

void
TupleState::InsertNeighborTuple (NeighborTuple const &tuple)
{
  for (NeighborSet::iterator it = m_neighborSet.begin();
       it != m_neighborSet.end (); it++)
    {
      if (it->neighbor == tuple.neighbor)
        {
          return;
        }
    }
  m_neighborSet.push_back (tuple);
}

/********** Neighbor 2 Hop Set Manipulation **********/

TwoHopNeighborTuple*
TupleState::FindTwoHopNeighborTuple (const ndn::Name &neighbor,
                                    const ndn::Name &twoHopNeighbor)
{
  for (TwoHopNeighborSet::iterator it = m_twoHopNeighborSet.begin ();
       it != m_twoHopNeighborSet.end (); it++)
    {
      if (it->neighbor == neighbor
          && it->twoHopNeighbor == twoHopNeighbor)
        {
          return &(*it);
        }
    }
  return NULL;
}

void
TupleState::EraseTwoHopNeighborTuple (const TwoHopNeighborTuple &tuple)
{
  for (TwoHopNeighborSet::iterator it = m_twoHopNeighborSet.begin ();
       it != m_twoHopNeighborSet.end (); it++)
    {
      if (*it == tuple)
        {
          m_twoHopNeighborSet.erase (it);
          break;
        }
    }
}

void
TupleState::EraseTwoHopNeighborTuples (const ndn::Name &neighbor,
                                      const ndn::Name &twoHopNeighbor)
{
  for (TwoHopNeighborSet::iterator it = m_twoHopNeighborSet.begin ();
       it != m_twoHopNeighborSet.end (); )
    {
      if (it->neighbor == neighbor
          && it->twoHopNeighbor == twoHopNeighbor)
        {
          it = m_twoHopNeighborSet.erase (it);
        }
      else
        {
          it++;
        }
    }
}

void
TupleState::EraseTwoHopNeighborTuples (const ndn::Name &neighbor)
{
  for (TwoHopNeighborSet::iterator it = m_twoHopNeighborSet.begin ();
       it != m_twoHopNeighborSet.end (); )
    {
      if (it->neighbor == neighbor)
        {
          it = m_twoHopNeighborSet.erase (it);
        }
      else
        {
          it++;
        }
    }
}

void
TupleState::InsertTwoHopNeighborTuple (const TwoHopNeighborTuple &tuple)
{
  m_twoHopNeighborSet.push_back (tuple);
}

/********** Topology Set Manipulation **********/

TopologyTuple*
TupleState::FindTopologyTuple (const ndn::Name &dest,
                              const ndn::Name &last)
{
  for (TopologySet::iterator it = m_topologySet.begin ();
       it != m_topologySet.end (); it++)
    {
      if (it->dest == dest && it->last == last)
        {
          return &(*it);
        }
    }
  return NULL;
}

TopologyTuple*
TupleState::FindNewerTopologyTuple (const ndn::Name &last, uint16_t ansn)
{
  for (TopologySet::iterator it = m_topologySet.begin ();
       it != m_topologySet.end (); it++)
    {
      if (it->last == last && it->sequenceNumber > ansn)
        {
          return &(*it);
        }
    }
  return NULL;
}

void
TupleState::EraseTopologyTuple (const TopologyTuple &tuple)
{
  for (TopologySet::iterator it = m_topologySet.begin ();
       it != m_topologySet.end (); it++)
    {
      if (*it == tuple)
        {
          m_topologySet.erase (it);
          break;
        }
    }
}

void
TupleState::EraseOlderTopologyTuples (const ndn::Name &last, uint16_t ansn)
{
  for (TopologySet::iterator it = m_topologySet.begin ();
       it != m_topologySet.end (); )
    {
      if (it->last == last && it->sequenceNumber < ansn)
        {
          it = m_topologySet.erase (it);
        }
      else
        {
          it++;
        }
    }
}

void
TupleState::EraseTopologyTuples(const ndn::Name &last)
{
  for (TopologySet::iterator it = m_topologySet.begin ();
       it != m_topologySet.end (); )
    {
      if (it->last == last)
        {
          it = m_topologySet.erase (it);
        }
      else
        {
          it++;
        }
    }

}

void
TupleState::InsertTopologyTuple (const TopologyTuple &tuple)
{
  m_topologySet.push_back (tuple);
}



/********** Mpr Set Manipulation **********/

MprTuple*
TupleState::FindMprTuple (const ndn::Name &neighbor)
{
  for (MprSet::iterator it = m_mprSet.begin ();
       it != m_mprSet.end (); it++)
    {
      if (it->mprNeighbor == neighbor)
        {
          return &(*it);
        }
    }
  return NULL;
}

void
TupleState::EraseMprTuple (const ndn::Name &neighbor)
{
  for (MprSet::iterator it = m_mprSet.begin ();
       it != m_mprSet.end (); it++)
    {
      if (it->mprNeighbor == neighbor)
        {
          it = m_mprSet.erase (it);
          break;
        }
    }
}

void
TupleState::InsertMprTuple (MprTuple const &tuple)
{
  for (MprSet::iterator it = m_mprSet.begin();
       it != m_mprSet.end (); it++)
    {
      if (it->mprNeighbor == tuple.mprNeighbor)
        {
          return;
        }
    }
  m_mprSet.push_back (tuple);
}

int
TupleState::Degree (NeighborTuple const &tuple)
{
  int degree = 0;
  for (TwoHopNeighborSet::const_iterator it = this->GetTwoHopNeighbors ().begin ();
       it != this->GetTwoHopNeighbors ().end (); it++)
    {
      TwoHopNeighborTuple const &nb2hop_tuple = *it;
      if (nb2hop_tuple.neighbor == tuple.neighbor)
        {

          const NeighborTuple *nb_tuple =
            this->FindNeighborTuple (nb2hop_tuple.twoHopNeighbor);
          if (nb_tuple == NULL)
            {
              degree++;
            }
        }
    }
  return degree;
}

///
/// \brief Remove all covered 2-hop neighbors from N2 set.
/// This is a helper function used by MprComputation algorithm.
///
/// \param neighborMainAddr Neighbor main address.
/// \param N2 Reference to the 2-hop neighbor set.
///
void
TupleState::CoverTwoHopNeighbors (ndn::Name neighbor, TwoHopNeighborSet & N2)
{
  // first gather all 2-hop neighbors to be removed
  std::set<ndn::Name> toRemove;
  for (TwoHopNeighborSet::iterator twoHopNeigh = N2.begin (); twoHopNeigh != N2.end (); twoHopNeigh++)
    {
      if (twoHopNeigh->neighbor == neighbor)
        {
          toRemove.insert ({twoHopNeigh->twoHopNeighbor});
        }
    }
  // Now remove all matching records from N2
  for (TwoHopNeighborSet::iterator twoHopNeigh = N2.begin (); twoHopNeigh != N2.end (); )
    {
      if (toRemove.find (twoHopNeigh->twoHopNeighbor) != toRemove.end ())
        {
          twoHopNeigh = N2.erase (twoHopNeigh);
        }
      else
        {
          twoHopNeigh++;
        }
    }

}


void
TupleState::MprComputation ()
{
  cout<<"Mpr computation begin"<<endl;
  // MPR computation should be done for each interface. See section 8.3.1
  // (RFC 3626) for details.
  MprSet mprSet;

  // N is the subset of neighbors of the node, which are
  // neighbor "of the interface I"
  NeighborSet N;
  for (NeighborSet::const_iterator neighbor = this->GetNeighbors ().begin ();
       neighbor != this->GetNeighbors ().end (); neighbor++)
    {
      //NOLSR中的NeighborTuple中都是确认过邻居关系的邻居，所以此步无需检查
      //if (neighbor->status == NeighborTuple::STATUS_SYM) // I think that we need this check
      //{
          N.push_back (*neighbor);
    //  }
    }

  // N2 is the set of 2-hop neighbors reachable from "the interface
  // I", excluding:
  // (i)   the nodes only reachable by members of N with willingness WILL_NEVER
  // (ii)  the node performing the computation
  // (iii) all the symmetric neighbors: the nodes for which there exists a symmetric
  //       link to this node on some interface.
  TwoHopNeighborSet N2;
  for (TwoHopNeighborSet::const_iterator twoHopNeigh = this->GetTwoHopNeighbors ().begin ();
       twoHopNeigh != this->GetTwoHopNeighbors ().end (); twoHopNeigh++)
    {
      //此步在lsbd-installAdjLsa已检查过，二跳邻居中不会出现这种情况
      // excluding:
      // (ii)  the node performing the computation
      //if (twoHopNeigh->twoHopNeighborAddr == m_mainAddress)
      //  {
      //    continue;
      //  }

      //  excluding:
      // (i)   the nodes only reachable by members of N with willingness WILL_NEVER
      bool ok = false;
      for (NeighborSet::const_iterator neigh = N.begin ();
           neigh != N.end (); neigh++)
        {
          if (neigh->neighbor == twoHopNeigh->neighbor)
            {
              if (neigh->willingness == WILL_NEVER)
                {
                  ok = false;
                  break;
                }
              else
                {
                  ok = true;
                  break;
                }
            }
        }
      if (!ok)
        {
          continue;
        }

      // excluding:
      // (iii) all the symmetric neighbors: the nodes for which there exists a symmetric
      //       link to this node on some interface.
      for (NeighborSet::const_iterator neigh = N.begin ();
           neigh != N.end (); neigh++)
        {
          if (neigh->neighbor == twoHopNeigh->twoHopNeighbor)
            {
              ok = false;
              break;
            }
        }

      if (ok)
        {
          N2.push_back (*twoHopNeigh);
        }
    }

  // 1. Start with an MPR set made of all members of N with
  // N_willingness equal to WILL_ALWAYS
  for (NeighborSet::const_iterator neighbor = N.begin (); neighbor != N.end (); neighbor++)
    {
      if (neighbor->willingness == WILL_ALWAYS)
        {
          MprTuple newMpr;
          newMpr.mprNeighbor = neighbor->neighbor;
          mprSet.push_back (newMpr);
          // (not in RFC but I think is needed: remove the 2-hop
          // neighbors reachable by the MPR from N2)
          CoverTwoHopNeighbors (neighbor->neighbor, N2);
          cout<<"First,WILL_ALWAYS node is set to MPR " << neighbor->neighbor<<endl;
        }
    }

  // 2. Calculate D(y), where y is a member of N, for all nodes in N.
  // (we do this later)

  // 3. Add to the MPR set those nodes in N, which are the *only*
  // nodes to provide reachability to a node in N2.
  std::set<ndn::Name> coveredTwoHopNeighbors;
  for (TwoHopNeighborSet::const_iterator twoHopNeigh = N2.begin (); twoHopNeigh != N2.end (); twoHopNeigh++)
    {
      bool onlyOne = true;
      // try to find another neighbor that can reach twoHopNeigh->twoHopNeighborAddr
      for (TwoHopNeighborSet::const_iterator otherTwoHopNeigh = N2.begin (); otherTwoHopNeigh != N2.end (); otherTwoHopNeigh++)
        {
          if (otherTwoHopNeigh->twoHopNeighbor == twoHopNeigh->twoHopNeighbor
              && otherTwoHopNeigh->neighbor != twoHopNeigh->neighbor)
            {
              onlyOne = false;
              break;
            }
        }
      if (onlyOne)
        {
          cout<<"Second ,the *only* node to provide reachability to a node in N2 is set to MPR: " <<twoHopNeigh->neighbor<<endl;
          MprTuple newMpr;
          newMpr.mprNeighbor = twoHopNeigh->neighbor;
          mprSet.push_back (newMpr);

          // take note of all the 2-hop neighbors reachable by the newly elected MPR
          for (TwoHopNeighborSet::const_iterator otherTwoHopNeigh = N2.begin ();
               otherTwoHopNeigh != N2.end (); otherTwoHopNeigh++)
            {
              if (otherTwoHopNeigh->neighbor == twoHopNeigh->neighbor)
                {
                  coveredTwoHopNeighbors.insert ({otherTwoHopNeigh->twoHopNeighbor});
                }
            }
        }
    }
  // Remove the nodes from N2 which are now covered by a node in the MPR set.
  for (TwoHopNeighborSet::iterator twoHopNeigh = N2.begin ();
       twoHopNeigh != N2.end (); )
    {
      if (coveredTwoHopNeighbors.find (twoHopNeigh->twoHopNeighbor) != coveredTwoHopNeighbors.end ())
        {
          // This works correctly only because it is known that twoHopNeigh is reachable by exactly one neighbor,
          // so only one record in N2 exists for each of them. This record is erased here.
          cout<<"Remove N2 nodes which are covered by MPR have been selected:" << twoHopNeigh->twoHopNeighbor<<endl;
          twoHopNeigh = N2.erase (twoHopNeigh);
        }
      else
        {
          twoHopNeigh++;
        }
    }

  // 4. While there exist nodes in N2 which are not covered by at
  // least one node in the MPR set:
  while (N2.begin () != N2.end ())
    {
      // 4.1. For each node in N, calculate the reachability, i.e., the
      // number of nodes in N2 which are not yet covered by at
      // least one node in the MPR set, and which are reachable
      // through this 1-hop neighbor
      std::map<int, std::vector<const NeighborTuple *> > reachability;
      std::set<int> rs;
      for (NeighborSet::iterator it = N.begin (); it != N.end (); it++)
        {
          NeighborTuple const &nb_tuple = *it;
          int r = 0;
          for (TwoHopNeighborSet::iterator it2 = N2.begin (); it2 != N2.end (); it2++)
            {
              TwoHopNeighborTuple const &nb2hop_tuple = *it2;
              if (nb_tuple.neighbor == nb2hop_tuple.neighbor)
                {
                  r++;
                }
            }
          rs.insert ({r});
          reachability[r].push_back (&nb_tuple);
        }

      // 4.2. Select as a MPR the node with highest N_willingness among
      // the nodes in N with non-zero reachability. In case of
      // multiple choice select the node which provides
      // reachability to the maximum number of nodes in N2. In
      // case of multiple nodes providing the same amount of
      // reachability, select the node as MPR whose D(y) is
      // greater. Remove the nodes from N2 which are now covered
      // by a node in the MPR set.
      NeighborTuple const *max = NULL;
      int max_r = 0;
      for (std::set<int>::iterator it = rs.begin (); it != rs.end (); it++)
        
        {
          int r = *it;
          if (r == 0)
            {
              continue;
            }
          for (std::vector<const NeighborTuple *>::iterator it2 = reachability[r].begin ();
               it2 != reachability[r].end (); it2++)
            {
              const NeighborTuple *nb_tuple = *it2;
              if (max == NULL || nb_tuple->willingness > max->willingness)
                {
                  max = nb_tuple;
                  max_r = r;
                }
              else if (nb_tuple->willingness == max->willingness)
                {
                  if (r > max_r)
                    {
                      max = nb_tuple;
                      max_r = r;
                    }
                  else if (r == max_r)
                    {
                      if (Degree (*nb_tuple) > Degree (*max))
                        {
                          max = nb_tuple;
                          max_r = r;
                        }
                    }
                }
            }
        }

      if (max != NULL)
        {
          MprTuple newMpr;
          newMpr.mprNeighbor = max->neighbor;
          mprSet.push_back (newMpr);
          CoverTwoHopNeighbors (max->neighbor, N2);
          cout<<"Second, the node with highest N_willingness amongthe nodes in N with non-zero reachability:" << max->neighbor<<endl;
        }
    }
    
  this->SetMprSet (mprSet);
  cout<<"Mpr computation end"<<endl;
  writeLog();
}

/********** MprSelector Set Manipulation **********/


MprSelectorTuple*
TupleState::FindMprSelectorTuple (const ndn::Name &neighbor)
{
  for (MprSelectorSet::iterator it = m_mprSelectorSet.begin ();
       it != m_mprSelectorSet.end (); it++)
    {
      if (it->mprSelector == neighbor)
        {
          return &(*it);
        }
    }
  return NULL;
}

void
TupleState::EraseMprSelectorTuple (const ndn::Name &neighbor)
{
  for (MprSelectorSet::iterator it = m_mprSelectorSet.begin ();
       it != m_mprSelectorSet.end (); it++)
    {
      if (it->mprSelector == neighbor)
        {
          it = m_mprSelectorSet.erase (it);
          break;
        }
    }
}

void
TupleState::InsertMprSelectorTuple (MprSelectorTuple const &tuple)
{
  for (MprSelectorSet::iterator it = m_mprSelectorSet.begin();
       it != m_mprSelectorSet.end (); it++)
    {
      if (it->mprSelector == tuple.mprSelector)
        {
          return;
        }
    }
  m_mprSelectorSet.push_back (tuple);
}


void
TupleState::writeLog()
{ 
  cout<<"log begin"<<endl;
  for (TopologySet::iterator it = m_topologySet.begin ();
       it != m_topologySet.end (); it++)
  {
    cout<<"TopologyTuple dest: " << it->dest<<endl;
    cout<<"TopologyTuple last: " << it->last<<endl;
  }
  
  for (TwoHopNeighborSet::iterator it = m_twoHopNeighborSet.begin ();
       it != m_twoHopNeighborSet.end (); it++)
  {
    cout<<"twoHopNeighborTuple neighbor: " << it->neighbor<<endl;;
    cout<<"twoHopNeighborTuple twoHopNeighbor: " << it->twoHopNeighbor<<endl;
  }
  
  for (NeighborSet::iterator it = m_neighborSet.begin ();
       it != m_neighborSet.end (); it++)
  {
    cout<<"NeighborTuple neighbor: " << it->neighbor<<endl;

  }

  for (MprSet::iterator it = m_mprSet.begin ();
       it != m_mprSet.end (); it++)
  {
    cout<<"Mpr neighbor: " << it->mprNeighbor<<endl;

  }

  for (MprSelectorSet::iterator it = m_mprSelectorSet.begin ();
       it != m_mprSelectorSet.end (); it++)
  {
    cout<<"MprSelector neighbor: " << it->mprSelector<<endl;

  }
  cout<<"log end"<<endl;
}

} // namespace olsr, ns3
