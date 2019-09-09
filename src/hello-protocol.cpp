/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
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

#include "hello-protocol.hpp"
#include "nlsr.hpp"
#include "lsdb.hpp"
#include "utility/name-helper.hpp"
#include "logger.hpp"

namespace nlsr {

INIT_LOGGER(HelloProtocol);

const std::string HelloProtocol::INFO_COMPONENT = "INFO";
const std::string HelloProtocol::NLSR_COMPONENT = "nlsr";
const std::string HelloProtocol::RECEIVE_COMPONENT = "received";
const std::string HelloProtocol::NOT_RECEIVE_COMPONENT = "notreceived";
const std::string HelloProtocol::MPR_COMPONENT = "mpr";
const std::string HelloProtocol::NOT_MPR_COMPONENT = "notmpr";

HelloProtocol::HelloProtocol(ndn::Face& face, TupleState& tuplestate, ndn::KeyChain& keyChain,
                             ndn::security::SigningInfo& signingInfo,
                             ConfParameter& confParam, RoutingTable& routingTable,
                             Lsdb& lsdb)
  : m_face(face)
  , m_state(tuplestate)
  , m_scheduler(m_face.getIoService())
  , m_keyChain(keyChain)
  , m_signingInfo(signingInfo)
  , m_confParam(confParam)
  , m_routingTable(routingTable)
  , m_lsdb(lsdb)
{
}

void
HelloProtocol::expressHello(const ndn::Name& helloDataName, uint32_t seconds)
{
  NLSR_LOG_DEBUG("Expressing Hello Data :" << helloDataName);
  std::shared_ptr<ndn::Data> data = std::make_shared<ndn::Data>();
  data->setName(helloDataName);
  data->setFreshnessPeriod(ndn::time::seconds(10)); // 10 sec
  
  //Put neighbor information into data
  HelloMessage helloMessage(m_state.m_willingness);
  for (const auto& adjacent : m_confParam.getAdjacencyList().getAdjList())
  {
    if (adjacent.getStatus() == Adjacent::STATUS_ACTIVE)
    {
      if (m_state.FindMprTuple(adjacent.getName()) != NULL)
      {
        Neighbor neighbor(adjacent.getName(), MPR_COMPONENT);
        helloMessage.insert(neighbor);
      }
      else
      {
        Neighbor neighbor(adjacent.getName(), NOT_MPR_COMPONENT);
        helloMessage.insert(neighbor);        
      }
    }
  }
  std::string content = helloMessage.serialize();
  data->setContent(ndn::encoding::makeStringBlock(ndn::tlv::Content, content));
  m_keyChain.sign(*data, m_signingInfo);
  m_face.put(*data);
  // increment SENT_HELLO_DATA
  hpIncrementSignal(Statistics::PacketType::SENT_HELLO_DATA);
}

void
HelloProtocol::sendScheduledHello()
{
  for (const auto& adjacent : m_confParam.getAdjacencyList().getAdjList()) {
    // If this adjacency has a Face, just proceed as usual.
    if(adjacent.getFaceId() != 0) {
      // hello data name: /<neighbor>/NLSR/INFO/<router>
     
      ndn::Name helloDataName = adjacent.getName() ;
      if (m_receivedNeighborSet.count(helloDataName))
      {
        helloDataName.append(NLSR_COMPONENT);
        helloDataName.append(INFO_COMPONENT);
        helloDataName.append(m_confParam.getRouterPrefix().wireEncode());
        helloDataName.append(RECEIVE_COMPONENT);
        expressHello(helloDataName, m_confParam.getInterestResendTime());
        NLSR_LOG_DEBUG("Sending scheduled hello: " << helloDataName);

      }
      else
      {
        helloDataName.append(NLSR_COMPONENT);
        helloDataName.append(INFO_COMPONENT);
        helloDataName.append(m_confParam.getRouterPrefix().wireEncode());
        helloDataName.append(NOT_RECEIVE_COMPONENT);
        expressHello(helloDataName, m_confParam.getInterestResendTime());
        NLSR_LOG_DEBUG("Sending scheduled hello: " << helloDataName);
      }
  }
 }
  scheduleSendHello(m_confParam.getInfoInterestInterval());

}

void
HelloProtocol::scheduleSendHello(uint32_t seconds)
{
  NLSR_LOG_DEBUG("Scheduling HELLO Datas in " << ndn::time::seconds(seconds));

  m_scheduler.schedule(ndn::time::seconds(seconds), [this] { sendScheduledHello(); });
}

//verify hello's validity, then give it to processHello() to process it.
void
HelloProtocol::afterFetchHello(const ndn::Data& data)
{
  m_confParam.getValidator().validate(data,
                                      std::bind(&HelloProtocol::processHello, this, _1),
                                      std::bind(&HelloProtocol::onHelloValidationFailed,
                                                this, _1, _2));

}

void
HelloProtocol::onHelloValidationFailed(const ndn::Data& data,
                                       const ndn::security::v2::ValidationError& ve)
{
  NLSR_LOG_DEBUG("Validation Error: " << ve);
}

void
HelloProtocol::processHello(const ndn::Data& data)
{
  // data name: /<neighbor>/NLSR/INFO/<router>/receiveComponent
  const ndn::Name dataName = data.getName();

  // increment RCV_HELLO_DATA
  hpIncrementSignal(Statistics::PacketType::RCV_HELLO_DATA);

  NLSR_LOG_DEBUG("Hello Data Received for Name: " << dataName);
  if (dataName.get(-3).toUri() != INFO_COMPONENT) {
    NLSR_LOG_DEBUG("INFO_COMPONENT not found or dataName: " << dataName
               << " does not match expression");
    return;
  }

  ndn::Name neighborName;
  neighborName.wireDecode(dataName.get(-2).blockFromValue());
  NLSR_LOG_DEBUG("Neighbor: " << neighborName);
  if (m_receivedNeighborSet.count(neighborName))
  {
    auto iterator = m_receivedNeighborMap.find(neighborName);
    iterator->second.cancel();
    NLSR_LOG_DEBUG("cancel Received Neighbor Expiring ");
    m_receivedNeighborMap.erase(neighborName);

    ndn::scheduler::EventId id = scheduleReceivedNeighborExpiration(neighborName, 
                                                         3*m_confParam.getInfoInterestInterval());
    m_receivedNeighborMap.insert({neighborName,id});

    if (dataName.get(-1).toUri() == RECEIVE_COMPONENT)
    {
      HelloMessage helloMessage;
      std::string dataContent(reinterpret_cast<const char*>(data.getContent().value()),
                              data.getContent().value_size());

      if (helloMessage.deserialize(dataContent))
      {
        NLSR_LOG_DEBUG("reading helloMessage ");
      }
      Adjacent::Status oldStatus = m_confParam.getAdjacencyList().getStatusOfNeighbor(neighborName);
      m_confParam.getAdjacencyList().setStatusOfNeighbor(neighborName, Adjacent::STATUS_ACTIVE);
      m_confParam.getAdjacencyList().setTimedOutInterestCount(neighborName, 0);
      Adjacent::Status newStatus = m_confParam.getAdjacencyList().getStatusOfNeighbor(neighborName);
  
      NLSR_LOG_DEBUG("Neighbor : " << neighborName);
      NLSR_LOG_DEBUG("Old Status: " << oldStatus << " New Status: " << newStatus);
      // change in Adjacency list
      if ((oldStatus - newStatus) != 0) 
      {
        if (m_confParam.getHyperbolicState() == HYPERBOLIC_STATE_ON)
         {
          m_routingTable.scheduleRoutingTableCalculation();
         }
        else 
        {
          m_lsdb.scheduleAdjLsaBuild();
          ProcessHelloMessageCaseOne(neighborName,helloMessage);
        }
      }
      else
      {
        ProcessHelloMessageCaseTwo(neighborName,helloMessage);
      }
      
    }
    else
    {
      NLSR_LOG_DEBUG("Maybe one-way link or network delay, so ignore it ");
      return;
    }
  }
  else
  {
    m_receivedNeighborSet.insert({neighborName});
    ndn::scheduler::EventId id = scheduleReceivedNeighborExpiration(neighborName, 
                                                         3*m_confParam.getInfoInterestInterval());
    m_receivedNeighborMap.insert({neighborName,id});
  }

}

//case 1: process hello message when neighbor status is : inactive->active 
void
HelloProtocol::ProcessHelloMessageCaseOne(ndn::Name& neighborName, HelloMessage& helloMessage)
{
  NeighborTuple *test_neighbortuple = m_state.FindNeighborTuple(neighborName);
  if (test_neighbortuple == NULL)
  {
    NeighborTuple newNeighborTuple;
    newNeighborTuple.neighbor = neighborName;
    newNeighborTuple.willingness = helloMessage.getWillingness();
    newNeighborTuple.setExpiringEventId(scheduleNeighborExpiration(neighborName,
                                        3*m_confParam.getInfoInterestInterval()));
    m_state.InsertNeighborTuple(newNeighborTuple);
  }

  //(1)put two hop neighbors into twoHopNeighbor tuple;
  //(2)fetch mpr-selector node
  for (const auto& twoHopNeighbor : helloMessage.getNeighborList())
  {
    if(twoHopNeighbor.getName() == m_confParam.getRouterPrefix().toUri() &&
        twoHopNeighbor.getMpr() == MPR_COMPONENT)
    {
      MprSelectorTuple newMprSelectorTuple;
      newMprSelectorTuple.mprSelector = neighborName;
      m_state.InsertMprSelectorTuple(newMprSelectorTuple);
    }
    else
    {
      if (twoHopNeighbor.getName() == m_confParam.getRouterPrefix().toUri())
      {
        continue;
      }
      TwoHopNeighborTuple newtuple;
      newtuple.neighbor = neighborName;
      newtuple.twoHopNeighbor = twoHopNeighbor.getName();
      m_state.InsertTwoHopNeighborTuple(newtuple);
    }
  }
  m_state.MprComputation();

}

//Case 2: process hello message when neighbor status is: active->active
void
HelloProtocol::ProcessHelloMessageCaseTwo(ndn::Name& neighborName, HelloMessage& helloMessage)
{
  NeighborTuple *test_neighbortuple = m_state.FindNeighborTuple(neighborName);
  test_neighbortuple->expiringEventId.cancel();
  NLSR_LOG_DEBUG("cancel Neighbor Expiring ");
  test_neighbortuple->setExpiringEventId(scheduleNeighborExpiration(neighborName,
                                                                    3*m_confParam.getInfoInterestInterval()));
  

  //fetch mpr selector neighbor firstly!
  for (const auto& twoHopNeighbor : helloMessage.getNeighborList())
  {
    if(twoHopNeighbor.getName() == m_confParam.getRouterPrefix().toUri() &&
       twoHopNeighbor.getMpr() == MPR_COMPONENT)
    {
      MprSelectorTuple newMprSelectorTuple;
      newMprSelectorTuple.mprSelector = neighborName;
      m_state.InsertMprSelectorTuple(newMprSelectorTuple);
    }
  }

  //then update twoHopNeighbor tuple by comparing new helloMessage's twoHopNeighbor with
  //old twoHopNeighbor tuple. If there is change, mpr and route computation will be triggered.
  bool needUpdateMprAndRoute = false;
  if (helloMessage.getWillingness() != test_neighbortuple->willingness)
  {
    needUpdateMprAndRoute = true;
  }

  for (const auto& twoHopNeighborTuple : m_state.m_twoHopNeighborSet)
  {
    if (twoHopNeighborTuple.neighbor == neighborName && 
        !helloMessage.isNeighbor(twoHopNeighborTuple.twoHopNeighbor))
    {

      m_state.EraseTwoHopNeighborTuple(twoHopNeighborTuple);
      needUpdateMprAndRoute = true;
    }
  }
        
  for (const auto& twoHopNeighbor : helloMessage.getNeighborList())
  {
    if (m_state.FindTwoHopNeighborTuple(neighborName, twoHopNeighbor.getName()) == NULL &&
        twoHopNeighbor.getName() != m_confParam.getRouterPrefix().toUri())
    {
      TwoHopNeighborTuple newtuple;
      newtuple.neighbor = neighborName;
      newtuple.twoHopNeighbor = twoHopNeighbor.getName();
      m_state.InsertTwoHopNeighborTuple(newtuple);
      needUpdateMprAndRoute = true;
    }
  }
  if (needUpdateMprAndRoute)
  {
    m_state.MprComputation();
    m_routingTable.scheduleRoutingTableCalculation();
  }

}

ndn::scheduler::EventId
HelloProtocol::scheduleNeighborExpiration(ndn::Name& neighbor, uint32_t seconds)
{  
  
  NLSR_LOG_DEBUG("Scheduling Neighbor Expires in " << ndn::time::seconds(seconds));

  return m_scheduler.schedule(ndn::time::seconds(seconds),
                                   [&] {
                                        neighborExpire(neighbor);
                                       });
}

void
HelloProtocol::neighborExpire(ndn::Name& neighbor)
{
  m_state.EraseNeighborTuple(neighbor);
  Adjacent::Status status = m_confParam.getAdjacencyList().getStatusOfNeighbor(neighbor);
  if (status == Adjacent::STATUS_ACTIVE)
  {
    m_confParam.getAdjacencyList().setStatusOfNeighbor(neighbor, Adjacent::STATUS_INACTIVE);

    NLSR_LOG_DEBUG("Neighbor: " << neighbor << " status changed to INACTIVE");

    m_lsdb.scheduleAdjLsaBuild();
  }
  m_state.MprComputation();
  m_state.EraseTwoHopNeighborTuples(neighbor);
}  

ndn::scheduler::EventId
HelloProtocol::scheduleReceivedNeighborExpiration(ndn::Name& neighbor, uint32_t seconds)
{

  ndn::Name neighbor1 = neighbor;
  NLSR_LOG_DEBUG("Scheduling Received Neighbor Expires in " << ndn::time::seconds(seconds));

  return m_scheduler.schedule(ndn::time::seconds(seconds),
                              [this,&neighbor1] {
                              receivedNeighborExpire(neighbor1);
                                                });

}

void
HelloProtocol::receivedNeighborExpire(ndn::Name& neighbor)
{

  NLSR_LOG_DEBUG("Received Neighbor:" << neighbor<< "Expires";);
  m_receivedNeighborMap.erase(neighbor);
  m_receivedNeighborSet.erase(neighbor);

}

void
HelloProtocol::processInterestTimedOut(const ndn::Interest& interest)
{/*
  // interest name: /<neighbor>/NLSR/INFO/<router>
  const ndn::Name interestName(interest.getName());
  NLSR_LOG_DEBUG("Interest timed out for Name: " << interestName);
  if (interestName.get(-2).toUri() != INFO_COMPONENT) {
    return;
  }
  ndn::Name neighbor = interestName.getPrefix(-3);
  NLSR_LOG_DEBUG("Neighbor: " << neighbor);
  m_confParam.getAdjacencyList().incrementTimedOutInterestCount(neighbor);

  Adjacent::Status status = m_confParam.getAdjacencyList().getStatusOfNeighbor(neighbor);

  uint32_t infoIntTimedOutCount =
    m_confParam.getAdjacencyList().getTimedOutInterestCount(neighbor);
  NLSR_LOG_DEBUG("Status: " << status);
  NLSR_LOG_DEBUG("Info Interest Timed out: " << infoIntTimedOutCount);
  if (infoIntTimedOutCount < m_confParam.getInterestRetryNumber()) {
    // interest name: /<neighbor>/NLSR/INFO/<router>
    ndn::Name interestName(neighbor);
    interestName.append(NLSR_COMPONENT);
    interestName.append(INFO_COMPONENT);
    interestName.append(m_confParam.getRouterPrefix().wireEncode());
    NLSR_LOG_DEBUG("Resending interest: " << interestName);
    expressInterest(interestName, m_confParam.getInterestResendTime());
  }
  else if ((status == Adjacent::STATUS_ACTIVE) &&
           (infoIntTimedOutCount == m_confParam.getInterestRetryNumber())) {
    m_confParam.getAdjacencyList().setStatusOfNeighbor(neighbor, Adjacent::STATUS_INACTIVE);

    NLSR_LOG_DEBUG("Neighbor: " << neighbor << " status changed to INACTIVE");

    m_lsdb.scheduleAdjLsaBuild();
  }*/
}

  // This is the first function that incoming Hello data will
  // see. This checks if the data appears to be signed, and passes it
  // on to validate the content of the data.
void
HelloProtocol::onContent(const ndn::Interest& interest, const ndn::Data& data)
{/*
  NLSR_LOG_DEBUG("Received data for INFO(name): " << data.getName());
  if (data.getSignature().hasKeyLocator()) {
    if (data.getSignature().getKeyLocator().getType() == ndn::KeyLocator::KeyLocator_Name) {
      NLSR_LOG_DEBUG("Data signed with: " << data.getSignature().getKeyLocator().getName());
    }
  }
  m_confParam.getValidator().validate(data,
                                      std::bind(&HelloProtocol::onContentValidated, this, _1),
                                      std::bind(&HelloProtocol::onContentValidationFailed,
                                                this, _1, _2));*/
}

void
HelloProtocol::onContentValidated(const ndn::Data& data)
{/*
  // data name: /<neighbor>/NLSR/INFO/<router>/<version>
  ndn::Name dataName = data.getName();
  NLSR_LOG_DEBUG("Data validation successful for INFO(name): " << dataName);

  if (dataName.get(-3).toUri() == INFO_COMPONENT) {
    ndn::Name neighbor = dataName.getPrefix(-4);

    Adjacent::Status oldStatus = m_confParam.getAdjacencyList().getStatusOfNeighbor(neighbor);
    m_confParam.getAdjacencyList().setStatusOfNeighbor(neighbor, Adjacent::STATUS_ACTIVE);
    m_confParam.getAdjacencyList().setTimedOutInterestCount(neighbor, 0);
    Adjacent::Status newStatus = m_confParam.getAdjacencyList().getStatusOfNeighbor(neighbor);

    NLSR_LOG_DEBUG("Neighbor : " << neighbor);
    NLSR_LOG_DEBUG("Old Status: " << oldStatus << " New Status: " << newStatus);
    // change in Adjacency list
    if ((oldStatus - newStatus) != 0) {
      if (m_confParam.getHyperbolicState() == HYPERBOLIC_STATE_ON) {
        m_routingTable.scheduleRoutingTableCalculation();
      }
      else {
        m_lsdb.scheduleAdjLsaBuild();
      }
    }
  }
  // increment RCV_HELLO_DATA
  hpIncrementSignal(Statistics::PacketType::RCV_HELLO_DATA);*/
}

void
HelloProtocol::onContentValidationFailed(const ndn::Data& data,
                                         const ndn::security::v2::ValidationError& ve)
{/*
  NLSR_LOG_DEBUG("Validation Error: " << ve);*/
}

void
HelloProtocol::processInterest(const ndn::Name& name,
                               const ndn::Interest& interest)
{

}

} // namespace nlsr
