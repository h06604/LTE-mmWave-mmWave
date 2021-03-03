/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
*   Copyright (c) 2011 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
*   Copyright (c) 2015, NYU WIRELESS, Tandon School of Engineering, New York University
*
*   This program is free software; you can redistribute it and/or modify
*   it under the terms of the GNU General Public License version 2 as
*   published by the Free Software Foundation;
*
*   This program is distributed in the hope that it will be useful,
*   but WITHOUT ANY WARRANTY; without even the implied warranty of
*   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*   GNU General Public License for more details.
*
*   You should have received a copy of the GNU General Public License
*   along with this program; if not, write to the Free Software
*   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
*
*   Author: Marco Miozzo <marco.miozzo@cttc.es>
*           Nicola Baldo  <nbaldo@cttc.es>
*
*   Modified by: Marco Mezzavilla < mezzavilla@nyu.edu>
*                         Sourjya Dutta <sdutta@nyu.edu>
*                         Russell Ford <russell.ford@nyu.edu>
*                         Menglei Zhang <menglei@nyu.edu>
*/

#include <iostream>
#include "ns3/mmwave-helper.h"
#include "ns3/epc-helper.h"
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/ipv4-global-routing-helper.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/applications-module.h"
#include "ns3/point-to-point-helper.h"
#include "ns3/config-store.h"
#include "ns3/mmwave-point-to-point-epc-helper.h"
//#include "ns3/gtk-config-store.h"

using namespace ns3;
using namespace mmwave;

/**
 * Sample simulation script for LTE+EPC. It instantiates several eNodeB,
 * attaches one UE per eNodeB starts a flow for each UE to  and from a remote host.
 * It also  starts yet another flow between each UE pair.
 */
NS_LOG_COMPONENT_DEFINE ("dcExample");
class MyApp : public Application
{
public:
  MyApp ();
  virtual ~MyApp ();

  void Setup (Ptr<Socket> socket, Address address, uint32_t packetSize, uint32_t nPackets, DataRate dataRate);

private:
  virtual void StartApplication (void);
  virtual void StopApplication (void);

  void ScheduleTx (void);
  void SendPacket (void);

  Ptr<Socket>     m_socket;
  Address         m_peer;
  uint32_t        m_packetSize;
  uint32_t        m_nPackets;
  DataRate        m_dataRate;
  EventId         m_sendEvent;
  bool            m_running;
  uint32_t        m_packetsSent;
};

MyApp::MyApp ()
  : m_socket (0),
    m_peer (),
    m_packetSize (0),
    m_nPackets (0),
    m_dataRate (0),
    m_sendEvent (),
    m_running (false),
    m_packetsSent (0)
{
}

MyApp::~MyApp ()
{
  m_socket = 0;
}

void
MyApp::Setup (Ptr<Socket> socket, Address address, uint32_t packetSize, uint32_t nPackets, DataRate dataRate)
{
  m_socket = socket;
  m_peer = address;
  m_packetSize = packetSize;
  m_nPackets = nPackets;
  m_dataRate = dataRate;
}

void
MyApp::StartApplication (void)
{
  m_running = true;
  m_packetsSent = 0;
  m_socket->Bind ();
  m_socket->Connect (m_peer);
  SendPacket ();
}

void
MyApp::StopApplication (void)
{
  m_running = false;

  if (m_sendEvent.IsRunning ())
    {
      Simulator::Cancel (m_sendEvent);
    }

  if (m_socket)
    {
      m_socket->Close ();
    }
}

void
MyApp::SendPacket (void)
{
  static int send_num = 1;
  Ptr<Packet> packet = Create<Packet> (m_packetSize);
  m_socket->Send (packet);
  NS_LOG_DEBUG ("Sending:    " << send_num++ << "\t" << Simulator::Now ().GetSeconds ());

  if (++m_packetsSent < m_nPackets)
    {
      ScheduleTx ();
    }
}

static void Rx (Ptr<OutputStreamWrapper> stream, Ptr<const Packet> packet, const Address &from)
{
  *stream->GetStream () << Simulator::Now ().GetSeconds () << "\t" << packet->GetSize () << std::endl;
}


void
MyApp::ScheduleTx (void)
{
  if (m_running)
    {
      Time tNext (Seconds (m_packetSize * 8 / static_cast<double> (m_dataRate.GetBitRate ())));
      m_sendEvent = Simulator::Schedule (tNext, &MyApp::SendPacket, this);
    }
}

static void
CwndChange (Ptr<OutputStreamWrapper> stream, uint32_t oldCwnd, uint32_t newCwnd)
{
  *stream->GetStream () << Simulator::Now ().GetSeconds () << "\t" << oldCwnd << "\t" << newCwnd << std::endl;
}

static void
Rtt (Ptr<OutputStreamWrapper> stream, Time oldValue, Time newValue)
{
  *stream->GetStream () << Simulator::Now ().GetSeconds () << "\t" << oldValue <<"\t" << newValue  << std::endl;
}

static void
BytesInFlight (Ptr<OutputStreamWrapper> stream, uint32_t oldValue, uint32_t newValue)
{
  *stream->GetStream () << Simulator::Now ().GetSeconds () << "\t" << oldValue <<"\t" << newValue  << std::endl;
}

//uint16_t pktsizeUDP = 1400;
uint16_t pktsizeUDP = 900;
uint16_t pktsizeTCP = 1400;

uint64_t lastTotalRx[100];
uint16_t c[10];
double t_total[10];

void
CalculateThroughput (Ptr<OutputStreamWrapper> stream, Ptr<UdpServer> sink, uint16_t i)
{
	Time now = Simulator::Now ();                                         /* Return the simulator's virtual time. */
  double cur = (sink->GetReceived () - lastTotalRx[i]) * (double) 8 * (double) pktsizeUDP * (double) 100 / 1e9;    /* Convert Application RX Packets to GBits. */
  //double cur = (sink->GetReceived () - lastTotalRx[i]) * (double) 8 * (double) pktsizeUDP  / 1e9; 
	c[i]++;
	t_total[i]+=cur;
	*stream->GetStream()  << now.GetSeconds () << "\t" << cur <<"\t"<<(double)(t_total[i]/c[i])<<std::endl;/* 瞬間tput與平均tput*/
	lastTotalRx[i] = sink->GetReceived ();
  
	Simulator::Schedule (MilliSeconds (10), &CalculateThroughput,stream,sink,i);
  //Simulator::Schedule (MilliSeconds (1000), &CalculateThroughput,stream,sink,i);
  
}

void
CalculateThroughput2 (Ptr<OutputStreamWrapper> stream, Ptr<PacketSink> sink, uint16_t i)
{
	Time now = Simulator::Now ();                                         /* Return the simulator's virtual time. */
  double cur = (sink->GetTotalRx () - lastTotalRx[i]) * (double) 8 * (double) 100 / 1e9; 
  //double cur = (sink->GetTotalRx () - lastTotalRx[i]) * (double) 8 / 1e9;    /* GetTotalRx : the total bytes received in this sink*/
	c[i]++;
	t_total[i]+=cur;
	*stream->GetStream()  << now.GetSeconds () << "\t" << cur <<"\t"<<(double)(t_total[i]/c[i])<<std::endl;/* 瞬間tput與平均tput*/
	lastTotalRx[i] = sink->GetTotalRx ();
  
  Simulator::Schedule (MilliSeconds (10), &CalculateThroughput2,stream,sink,i);
	//Simulator::Schedule (MilliSeconds (1000), &CalculateThroughput2,stream,sink,i);
}

void
enablelog(void)
{
  //LogComponentEnable ("TcpTxBuffer", LOG_LEVEL_ALL);
  //LogComponentEnable ("TcpSocketBase", LOG_LEVEL_DEBUG);
  //LogComponentEnable("SrncProcessor", LOG_LEVEL_DEBUG);
  //LogComponentEnable("McUePdcp", LOG_LEVEL_DEBUG);
  //LogComponentEnable("McEnbPdcp", LOG_LEVEL_DEBUG);
  //LogComponentEnable ("LteEnbRrc", LOG_LEVEL_ALL);
}

int
main (int argc, char *argv[])
{
  //LogComponentEnable ("McUeNetDevice", LOG_LEVEL_ALL);
  //LogComponentEnable ("MmWaveSpectrumPhy", LOG_LEVEL_ALL);
  //LogComponentEnable ("MmWaveUePhy", LOG_LEVEL_ALL);
  //LogComponentEnable ("LteRlcTm", LOG_LEVEL_ALL);
  //LogComponentEnable ("LteRlc", LOG_LEVEL_ALL);
  //LogComponentEnable ("LteEnbRrc", LOG_LEVEL_ALL);
  //LogComponentEnable ("LteUeRrc", LOG_LEVEL_ALL);
  //LogComponentEnable ("EpcUeNas", LOG_LEVEL_ALL);
  //LogComponentEnable ("MmWaveLteRrcProtocolReal", LOG_LEVEL_ALL);
  //LogComponentEnable ("MmWaveEnbMac", LOG_LEVEL_ALL);
  ///LogComponentEnable ("MmWaveUeMac", LOG_LEVEL_ALL);
  //LogComponentEnable ("MmWaveEnbPhy", LOG_LEVEL_ALL);
  //LogComponentEnable ("EpcX2", LOG_LEVEL_ALL);
  //LogComponentEnable ("LteRlcAm", LOG_LEVEL_ALL);
  //LogComponentEnable ("TcpSocketBase", LOG_LEVEL_ALL);
  //LogComponentEnable ("MmWavePhyRxTrace", LOG_LEVEL_DEBUG);
  //LogComponentEnable("MmWaveNoOpComponentCarrierManager", LOG_LEVEL_ALL);
  //LogComponentEnable("SrncTag", LOG_LEVEL_ALL);
  //LogComponentEnable("SrncHeader", LOG_LEVEL_ALL);
  //LogComponentEnable("SrncProcessor", LOG_LEVEL_ALL);
  //LogComponentEnable("Ipv4L3Protocol", LOG_LEVEL_ALL);
  //LogComponentEnable("TcpL4Protocol", LOG_LEVEL_ALL);
  //LogComponentEnable("dcExample", LOG_LEVEL_ALL);
  //LogComponentEnable ("EpcUeNas", LOG_LEVEL_ALL);
  //LogComponentEnable ("MmWaveHelper", LOG_LEVEL_ALL);
  //LogComponentEnable ("LteRlc", LOG_LEVEL_ALL);
  //LogComponentEnable("EpcX2", LOG_LEVEL_ALL);
  //LogComponentEnable("McEnbPdcp", LOG_LEVEL_DEBUG);
  //LogComponentEnable("McEnbPdcp", LOG_LEVEL_ALL);
  //LogComponentEnable("McUePdcp", LOG_LEVEL_ALL);
  //LogComponentEnable("McUeNetDevice", LOG_LEVEL_ALL);
  //LogComponentEnable("MmWavePointToPointEpcHelper",LOG_LEVEL_ALL);
  //LogComponentEnable ("LteSpectrumPhy", LOG_LEVEL_LOGIC);
  //LogComponentEnable ("MmWaveSpectrumPhy", LOG_LEVEL_LOGIC);
  //LogComponentEnable ("LteUePhy", LOG_LEVEL_DEBUG);
  //LogComponentEnable ("MmWaveUePhy", LOG_LEVEL_DEBUG);
  //LogComponentEnable ("MmWaveEnbPhy", LOG_LEVEL_DEBUG);
  //LogComponentEnable ("MmWaveUeMac", LOG_LEVEL_LOGIC);
  //LogComponentEnable ("UdpClient", LOG_LEVEL_ALL);
  //LogComponentEnable ("UdpServer", LOG_LEVEL_ALL);
  //LogComponentEnable("PropagationLossModel",LOG_LEVEL_ALL);
  //LogComponentEnable("MmWave3gppPropagationLossModel",LOG_LEVEL_ALL);
  uint32_t ReportTablePeriodicity = 1600;
  //bool onlyUsemmWaveDecoding = false;
  //bool alwaysLteUplink = false;
  //uint16_t timeval = 160;
  bool sack = false;
  bool isPacingEnabled = true;
  bool useUDP = true;
  bool use3GPPModel = false;
  //bool PdEnabled = false;
  //bool NcEnabled = false;
  uint16_t numEnb = 1;
  uint16_t numUe = 1;
  double simTime = 4.5;
  double interPacketInterval = 1;
  double minDistance = 10.0;        // eNB-UE distance in meters
  double maxDistance = 10.0;        // eNB-UE distance in meters
  //double maxDistance = 150.0;        // eNB-UE distance in meters
  bool harqEnabled = true;
  bool rlcAmEnabled = true;
  bool fixedTti = false;
  unsigned symPerSf = 24;
  double sfPeriod = 100.0;
  unsigned run = 0;
  //bool smallScale = true;
  //double speed = 3;
  DataRate maxPacingRate ("2.64Gbps");
  uint64_t PacingRate = maxPacingRate.GetBitRate()/(uint64_t)numUe;
  DataRate avgPacingRate (PacingRate);
  double speed = 3;
  // Command line arguments
  CommandLine cmd;
  cmd.AddValue ("numEnb", "Number of eNBs", numEnb);
  cmd.AddValue ("numUe", "Number of UEs per eNB", numUe);
  cmd.AddValue ("simTime", "Total duration of the simulation [s])", simTime);
  cmd.AddValue ("interPacketInterval", "Inter-packet interval [us])", interPacketInterval);
  cmd.AddValue ("harq", "Enable Hybrid ARQ", harqEnabled);
  cmd.AddValue ("rlcAm", "Enable RLC-AM", rlcAmEnabled);
  cmd.AddValue ("symPerSf", "OFDM symbols per subframe", symPerSf);
  cmd.AddValue ("sfPeriod", "Subframe period = 4.16 * symPerSf", sfPeriod);
  cmd.AddValue ("fixedTti", "Fixed TTI scheduler", fixedTti);
  cmd.AddValue ("run", "run for RNG (for generating different deterministic sequences for different drops)", fixedTti);
  cmd.Parse (argc, argv);

  Config::SetDefault ("ns3::TcpSocketBase::Sack", BooleanValue (sack));
  Config::SetDefault ("ns3::TcpSocketState::EnablePacing", BooleanValue (isPacingEnabled));
  Config::SetDefault ("ns3::TcpSocketState::MaxPacingRate", DataRateValue (avgPacingRate));
  Config::SetDefault ("ns3::TcpL4Protocol::SocketType", TypeIdValue (TcpCubic::GetTypeId ()));
  Config::SetDefault ("ns3::TcpSocket::SndBufSize", UintegerValue (30 * 1024 * 1024));//30Mbytes
  Config::SetDefault ("ns3::TcpSocket::RcvBufSize", UintegerValue (30 * 1024 * 1024));
  //Config::SetDefault ("ns3::TcpSocket::SndBufSize", UintegerValue (131072 * 50));//6.25Mbytes
  //Config::SetDefault ("ns3::TcpSocket::RcvBufSize", UintegerValue (131072 * 50));

  //要留時間設置mmWave RT 否則封包會直接從LTE stack往下傳
  //double transientDuration = 0.24;
  int windowForTransient = 150; 
  if (ReportTablePeriodicity == 1600)
    {
      windowForTransient = 150;
    }
  else if (ReportTablePeriodicity == 25600)
    {
      windowForTransient = 50;
    }
  else if (ReportTablePeriodicity == 12800)
    {
      windowForTransient = 100;
    }
  else
    {
      NS_ASSERT_MSG (false, "Unrecognized");
    }
  int vectorTransient = windowForTransient * ReportTablePeriodicity;
  double transientDuration = double(vectorTransient) / 1000000;
  /*
  Config::SetDefault ("ns3::TcpSocket::SegmentSize", UintegerValue (pktsizeTCP));
  if(useUDP){
    Config::SetDefault ("ns3::McEnbPdcp::NcPktSize", UintegerValue (pktsizeUDP));
  }else{
    Config::SetDefault ("ns3::McEnbPdcp::NcPktSize", UintegerValue (pktsizeTCP));
  }
  Config::SetDefault ("ns3::McEnbPdcp::NcUeNumber", UintegerValue (numUe));
  //Config::SetDefault ("ns3::McEnbPdcp::NcTimeVal", UintegerValue (100));
  Config::SetDefault ("ns3::McEnbPdcp::NcTimeVal", UintegerValue (timeval));
  Config::SetDefault ("ns3::McEnbPdcp::NcLabelK", UintegerValue (0));
  Config::SetDefault ("ns3::McEnbPdcp::NcTriggerClock", UintegerValue (1));//最多一次只能解碼110個封包, 所以time interval/clock < 110(rank計算上限)
  Config::SetDefault ("ns3::McEnbPdcp::EnableNetworkCoding", BooleanValue (NcEnabled));
  Config::SetDefault ("ns3::McEnbPdcp::EnablePacketDuplication", BooleanValue (PdEnabled));
  Config::SetDefault ("ns3::McEnbPdcp::useUdp", BooleanValue (useUDP));
  Config::SetDefault ("ns3::McUePdcp::useUdp", BooleanValue (useUDP));
  Config::SetDefault ("ns3::McUePdcp::EnableNetworkCoding", BooleanValue (NcEnabled));
  Config::SetDefault ("ns3::McUePdcp::NcTriggerClock", UintegerValue (30));//not use
  Config::SetDefault ("ns3::McUePdcp::NcBufferLimit", UintegerValue (100000));
  Config::SetDefault ("ns3::McUePdcp::LteUplink", BooleanValue (alwaysLteUplink));
  Config::SetDefault ("ns3::McUePdcp::onlyUsemmWaveDecoding", BooleanValue (onlyUsemmWaveDecoding));
  */
  Config::SetDefault ("ns3::LteRlcUm::MaxTxBufferSize", UintegerValue (40 * 1024 * 1024));//30Mbytes
  Config::SetDefault ("ns3::LteRlcUmLowLat::MaxTxBufferSize", UintegerValue (40 * 1024 * 1024));
  Config::SetDefault ("ns3::LteRlcAm::PollRetransmitTimer", TimeValue (MilliSeconds (4.0)));
  Config::SetDefault ("ns3::LteRlcAm::ReorderingTimer", TimeValue (MilliSeconds (2.0)));
  Config::SetDefault ("ns3::LteRlcAm::StatusProhibitTimer", TimeValue (MilliSeconds (1.0)));
  Config::SetDefault ("ns3::LteRlcAm::ReportBufferStatusTimer", TimeValue (MilliSeconds (4.0)));
  Config::SetDefault ("ns3::LteRlcAm::MaxTxBufferSize", UintegerValue (40 * 1024 * 1024));//30Mbytes

  Config::SetDefault ("ns3::MmWaveUeMac::UpdateUeSinrEstimatePeriod", DoubleValue (0));
  Config::SetDefault ("ns3::MmWaveEnbPhy::UpdateSinrEstimatePeriod", IntegerValue (ReportTablePeriodicity));
  Config::SetDefault ("ns3::MmWaveEnbPhy::Transient", IntegerValue (vectorTransient));
  Config::SetDefault ("ns3::LteEnbRrc::SecondaryCellHandoverMode", EnumValue (LteEnbRrc::THRESHOLD));
  Config::SetDefault ("ns3::LteEnbRrc::FixedTttValue", UintegerValue (150));
  Config::SetDefault ("ns3::LteEnbRrc::CrtPeriod", IntegerValue (ReportTablePeriodicity));
  Config::SetDefault ("ns3::LteEnbRrc::OutageThreshold", DoubleValue (-5));
  Config::SetDefault ("ns3::MmWaveEnbPhy::NoiseAndFilter", BooleanValue (false));

  Config::SetDefault ("ns3::MmWavePointToPointEpcHelper::S1uLinkDelay", TimeValue (MicroSeconds (1000)));
  Config::SetDefault ("ns3::MmWavePointToPointEpcHelper::S1apLinkDelay", TimeValue (MicroSeconds (10000)));
  Config::SetDefault ("ns3::MmWavePointToPointEpcHelper::X2LinkDelay", TimeValue (MicroSeconds (500)));
  Config::SetDefault ("ns3::MmWavePointToPointEpcHelper::X2LinkDataRate", DataRateValue (DataRate ("1000Gb/s")));
  Config::SetDefault ("ns3::MmWavePointToPointEpcHelper::X2LinkMtu",  UintegerValue (10000));
  Config::SetDefault ("ns3::MmWaveHelper::RlcAmEnabled", BooleanValue (rlcAmEnabled));
  Config::SetDefault ("ns3::MmWaveHelper::HarqEnabled", BooleanValue (harqEnabled));
  Config::SetDefault ("ns3::MmWaveFlexTtiMacScheduler::HarqEnabled", BooleanValue (harqEnabled));
  Config::SetDefault ("ns3::MmWaveFlexTtiMacScheduler::CqiTimerThreshold", UintegerValue (1000));
  Config::SetDefault ("ns3::MmWaveFlexTtiMaxWeightMacScheduler::HarqEnabled", BooleanValue (harqEnabled));
  Config::SetDefault ("ns3::MmWaveFlexTtiMaxWeightMacScheduler::FixedTti", BooleanValue (fixedTti));
  Config::SetDefault ("ns3::MmWaveFlexTtiMaxWeightMacScheduler::SymPerSlot", UintegerValue (6));
  Config::SetDefault ("ns3::MmWavePhyMacCommon::ResourceBlockNum", UintegerValue (1));
  Config::SetDefault ("ns3::MmWavePhyMacCommon::ChunkPerRB", UintegerValue (72));
  Config::SetDefault ("ns3::MmWavePhyMacCommon::SymbolsPerSubframe", UintegerValue (symPerSf));
  Config::SetDefault ("ns3::MmWavePhyMacCommon::SubframePeriod", DoubleValue (sfPeriod));
  Config::SetDefault ("ns3::MmWavePhyMacCommon::TbDecodeLatency", UintegerValue (200.0));
  Config::SetDefault ("ns3::MmWaveBeamforming::LongTermUpdatePeriod", TimeValue (MilliSeconds (100.0)));
  Config::SetDefault ("ns3::LteEnbRrc::SystemInformationPeriodicity", TimeValue (MilliSeconds (5.0)));
  Config::SetDefault ("ns3::LteRlcAm::ReportBufferStatusTimer", TimeValue (MicroSeconds (100.0)));
  Config::SetDefault ("ns3::LteRlcUmLowLat::ReportBufferStatusTimer", TimeValue (MicroSeconds (100.0)));
  Config::SetDefault ("ns3::LteEnbRrc::SrsPeriodicity", UintegerValue (320));
  Config::SetDefault ("ns3::LteEnbRrc::FirstSibTime", UintegerValue (2));
  //Config::SetDefault ("ns3::MmWaveBeamforming::SmallScaleFading", BooleanValue (smallScale));
  Config::SetDefault ("ns3::MmWaveBeamforming::FixSpeed", BooleanValue (true));
  Config::SetDefault ("ns3::MmWaveBeamforming::UeSpeed", DoubleValue (speed));
  Config::SetDefault ("ns3::McUeNetDevice::AntennaNum", UintegerValue(16));
  Config::SetDefault ("ns3::MmWaveEnbNetDevice::AntennaNum", UintegerValue(64));
  Config::SetDefault ("ns3::LteEnbRrc::SecondaryCellHandoverMode", EnumValue (LteEnbRrc::DYNAMIC_TTT));
  //Config::SetDefault ("ns3::MmWavePhyMacCommon::CenterFreq",DoubleValue (28e9));

  RngSeedManager::SetSeed (124);
  RngSeedManager::SetRun (run);

  Ptr<MmWaveHelper> mmwaveHelper = CreateObject<MmWaveHelper> ();
  mmwaveHelper->SetSchedulerType ("ns3::MmWaveFlexTtiMacScheduler");

  if(use3GPPModel){
    Config::SetDefault ("ns3::MmWave3gppPropagationLossModel::ChannelCondition", StringValue ("a"));
    Config::SetDefault ("ns3::MmWave3gppPropagationLossModel::Scenario", StringValue ("UMa"));
    //Config::SetDefault ("ns3::MmWave3gppPropagationLossModel::Scenario", StringValue ("UMi-StreetCanyon"));
    Config::SetDefault ("ns3::MmWave3gppPropagationLossModel::OptionalNlos", BooleanValue (false));
    Config::SetDefault ("ns3::MmWave3gppPropagationLossModel::Shadowing", BooleanValue (false)); 
    Config::SetDefault ("ns3::MmWave3gppBuildingsPropagationLossModel::UpdateCondition", BooleanValue (false));
    mmwaveHelper->SetAttribute ("ChannelModel", StringValue ("ns3::MmWave3gppChannel"));
    Config::SetDefault ("ns3::MmWave3gppChannel::UpdatePeriod", TimeValue (MilliSeconds (100))); 
    Config::SetDefault ("ns3::MmWave3gppChannel::DirectBeam", BooleanValue (true)); // Set true to perform the beam in the exact direction of receiver node.
    Config::SetDefault ("ns3::MmWave3gppChannel::Blockage", BooleanValue (false)); // use blockage or not
    Config::SetDefault ("ns3::MmWave3gppChannel::PortraitMode", BooleanValue (false)); // use blockage model with UT in portrait mode
    //Config::SetDefault ("ns3::MmWave3gppChannel::NumNonselfBlocking", IntegerValue (4)); // number of non-self blocking obstacles
    //mmwaveHelper->SetAttribute ("PathlossModel", StringValue ("ns3::MmWave3gppBuildingsPropagationLossModel"));
    mmwaveHelper->SetAttribute ("PathlossModel", StringValue ("ns3::MmWave3gppPropagationLossModel"));
  }else{
    //Config::SetDefault ("ns3::MmWavePropagationLossModel::ChannelStates", StringValue ("a"));
    Config::SetDefault ("ns3::MmWavePropagationLossModel::ChannelStates", StringValue ("l"));
    mmwaveHelper->SetAttribute ("ChannelModel", StringValue ("ns3::MmWaveBeamforming"));
    mmwaveHelper->SetAttribute ("PathlossModel", StringValue ("ns3::MmWavePropagationLossModel"));
  }
  
  Ptr<MmWavePointToPointEpcHelper>  epcHelper = CreateObject<MmWavePointToPointEpcHelper> ();
  mmwaveHelper->SetEpcHelper (epcHelper);
  mmwaveHelper->SetHarqEnabled (harqEnabled);
  mmwaveHelper->Initialize ();

  ConfigStore inputConfig;
  inputConfig.ConfigureDefaults ();

  // parse again so you can override default values from the command line
  cmd.Parse (argc, argv);

  // Create a single RemoteHost
  Ptr<Node> pgw = epcHelper->GetPgwNode ();
  NodeContainer remoteHostContainer;
  remoteHostContainer.Create (1);
  Ptr<Node> remoteHost = remoteHostContainer.Get (0);
  InternetStackHelper internet;
  internet.Install (remoteHostContainer);

  // Create the Internet
  PointToPointHelper p2ph;
  p2ph.SetDeviceAttribute ("DataRate", DataRateValue (DataRate ("100Gb/s")));
  p2ph.SetDeviceAttribute ("Mtu", UintegerValue (1500));
  p2ph.SetChannelAttribute ("Delay", TimeValue (Seconds (0.010)));
  NetDeviceContainer internetDevices = p2ph.Install (pgw, remoteHost);
  Ipv4AddressHelper ipv4h;
  ipv4h.SetBase ("1.0.0.0", "255.0.0.0");
  Ipv4InterfaceContainer internetIpIfaces = ipv4h.Assign (internetDevices);
  // interface 0 is localhost, 1 is the p2p device
  Ipv4Address remoteHostAddr = internetIpIfaces.GetAddress (1);
  Ipv4StaticRoutingHelper ipv4RoutingHelper;
  Ptr<Ipv4StaticRouting> remoteHostStaticRouting = ipv4RoutingHelper.GetStaticRouting (remoteHost->GetObject<Ipv4> ());
  remoteHostStaticRouting->AddNetworkRouteTo (Ipv4Address ("7.0.0.0"), Ipv4Mask ("255.0.0.0"), 1);

  NodeContainer ueNodes;
  NodeContainer lteEnbNodes;
  NodeContainer mmWaveEnbNodes;
  NodeContainer mmWaveEnbNodes28G;
  NodeContainer mmWaveEnbNodes73G;
  NodeContainer allEnbNodes;
  lteEnbNodes.Create (numEnb);
  mmWaveEnbNodes28G.Create (numEnb);
  mmWaveEnbNodes73G.Create (numEnb);
  mmWaveEnbNodes.Add(mmWaveEnbNodes28G);
  mmWaveEnbNodes.Add(mmWaveEnbNodes73G);
  ueNodes.Create (numUe);
  allEnbNodes.Add (lteEnbNodes);
  allEnbNodes.Add (mmWaveEnbNodes28G);
  allEnbNodes.Add (mmWaveEnbNodes73G);

  // Install Mobility Model
  Ptr<ListPositionAllocator> enbPositionAlloc = CreateObject<ListPositionAllocator> ();
  enbPositionAlloc->Add (Vector (0.0, 0.0, 3));
  enbPositionAlloc->Add (Vector (0.0, 0.0, 3));
  enbPositionAlloc->Add (Vector (0.0, 0.0, 3));
  MobilityHelper enbmobility;
  enbmobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  enbmobility.SetPositionAllocator (enbPositionAlloc);
  enbmobility.Install (allEnbNodes);

  MobilityHelper uemobility;
  Ptr<ListPositionAllocator> uePositionAlloc = CreateObject<ListPositionAllocator> ();
  Ptr<UniformRandomVariable> distRv = CreateObject<UniformRandomVariable> ();
  for (unsigned i = 0; i < numUe; i++)
    {
      double dist = distRv->GetValue (minDistance, maxDistance);
      uePositionAlloc->Add (Vector (dist, 0.0, 1.6));
      //uePositionAlloc->Add (Vector (dist, 0.0, 1.6)); 3GPP channel model
    }
  uemobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  uemobility.SetPositionAllocator (uePositionAlloc);
  uemobility.Install (ueNodes);

  // Install mmWave Devices to the nodes
  NetDeviceContainer lteEnbDevs = mmwaveHelper->InstallLteEnbDevice (lteEnbNodes);
  NetDeviceContainer mmWaveEnbDevs28G = mmwaveHelper->InstallEnbDevice (mmWaveEnbNodes28G);
  NetDeviceContainer mmWaveEnbDevs73G = mmwaveHelper->InstallFakeEnbDevice (mmWaveEnbNodes73G);
  NetDeviceContainer mcUeDevs;
  mcUeDevs = mmwaveHelper->InstallMcUeDevice (ueNodes);
  // Install the IP stack on the UEs
  internet.Install (ueNodes);
  Ipv4InterfaceContainer ueIpIface;
  ueIpIface = epcHelper->AssignUeIpv4Address (NetDeviceContainer (mcUeDevs));
  // Assign IP address to UEs, and install applications
  for (uint32_t u = 0; u < ueNodes.GetN (); ++u)
    {
      Ptr<Node> ueNode = ueNodes.Get (u);
      // Set the default gateway for the UE
      Ptr<Ipv4StaticRouting> ueStaticRouting = ipv4RoutingHelper.GetStaticRouting (ueNode->GetObject<Ipv4> ());
      ueStaticRouting->SetDefaultRoute (epcHelper->GetUeDefaultGatewayAddress (), 1);
    }
    
  mmwaveHelper->AddX2Interface (lteEnbNodes, mmWaveEnbNodes);
  mmwaveHelper->AttachToClosestEnb (mcUeDevs, mmWaveEnbDevs28G, mmWaveEnbDevs73G, lteEnbDevs);

  if(useUDP)
  {
    // Install and start applications on UEs and remote host
    uint16_t dlPort = 1234;
    uint16_t ulPort = 2000;
    uint16_t otherPort = 3000;
    ApplicationContainer clientApps;
    ApplicationContainer serverApps;
    for (uint32_t u = 0; u < ueNodes.GetN (); ++u)
      {
        ++ulPort;
        ++otherPort;
        
        UdpServerHelper dlPacketSinkHelper (dlPort);
        //dlPacketSinkHelper.SetAttribute ("PacketWindowSize", UintegerValue (256));
        serverApps.Add (dlPacketSinkHelper.Install (ueNodes.Get (u)));

        UdpClientHelper ulClient (remoteHostAddr, ulPort);
        UdpClientHelper dlClient (ueIpIface.GetAddress (u), dlPort);
        dlClient.SetAttribute ("Interval", TimeValue (MicroSeconds (interPacketInterval)));
        dlClient.SetAttribute ("MaxPackets", UintegerValue (1000000000));
        dlClient.SetAttribute ("PacketSize", UintegerValue (pktsizeUDP));

        clientApps.Add (dlClient.Install (remoteHost));

        std::ostringstream fileName_2;
        fileName_2<<"udp_throughput_ue" << u+1 <<".txt";
        AsciiTraceHelper asciiTraceHelper_2;
        Ptr<OutputStreamWrapper> stream_2 = asciiTraceHelper_2.CreateFileStream(fileName_2.str().c_str());
        Simulator::Schedule (Seconds (transientDuration), &CalculateThroughput,stream_2,serverApps.Get(u)->GetObject<UdpServer>(),u);

      }

    
    serverApps.Start (Seconds (transientDuration));
    clientApps.Start (Seconds (transientDuration));

    clientApps.Stop (Seconds(simTime - 0.02));
    serverApps.Stop (Seconds(simTime - 0.02));


  }else
  {
    std::string WindowFileName = "mmWave-tcp-window";
    std::string DataFileName = "mmWave-tcp-data";
    std::string RttFileName = "mmWave-tcp-rtt";
    std::string TputFileName = "tcp_throughput_ue";
    std::string BytesInFlightFileName = "mmWave-tcp-ByteInFlight";
    std::string FileName = ".txt";
    uint16_t sinkPort = 20000;
    for (uint32_t u = 0; u < ueNodes.GetN (); ++u)
    {
      
      Address sinkAddress (InetSocketAddress (ueIpIface.GetAddress (u), sinkPort));
      PacketSinkHelper packetSinkHelper ("ns3::TcpSocketFactory", InetSocketAddress (Ipv4Address::GetAny (), sinkPort));
      ApplicationContainer sinkApps = packetSinkHelper.Install (ueNodes.Get (u));
      sinkApps.Start (Seconds (transientDuration));
      sinkApps.Stop (Seconds (simTime - 0.02));
      Ptr<Socket> ns3TcpSocket = Socket::CreateSocket (remoteHostContainer.Get (0), TcpSocketFactory::GetTypeId ());

      Ptr<MyApp> app = CreateObject<MyApp> ();
      app->Setup (ns3TcpSocket, sinkAddress, pktsizeTCP, 9000000, DataRate ("3000Mb/s"));
      remoteHostContainer.Get (0)->AddApplication (app);

      AsciiTraceHelper asciiTraceHelper;
      Ptr<OutputStreamWrapper> stream1 = asciiTraceHelper.CreateFileStream (WindowFileName + std::to_string(u) + FileName);
      ns3TcpSocket->TraceConnectWithoutContext ("CongestionWindow", MakeBoundCallback (&CwndChange, stream1));

      Ptr<OutputStreamWrapper> stream2 = asciiTraceHelper.CreateFileStream (DataFileName + std::to_string(u) + FileName);
      sinkApps.Get (0)->TraceConnectWithoutContext ("Rx",MakeBoundCallback (&Rx, stream2));

      Ptr<OutputStreamWrapper> stream_3 = asciiTraceHelper.CreateFileStream(TputFileName + std::to_string(u) + FileName);
      Simulator::Schedule (Seconds (transientDuration), &CalculateThroughput2,stream_3,sinkApps.Get(0)->GetObject<PacketSink>(),u);
      
      Ptr<OutputStreamWrapper> stream4 = asciiTraceHelper.CreateFileStream (RttFileName + std::to_string(u) + FileName);
      ns3TcpSocket->TraceConnectWithoutContext ("RTT",MakeBoundCallback (&Rtt, stream4));

      Ptr<OutputStreamWrapper> stream5 = asciiTraceHelper.CreateFileStream (BytesInFlightFileName + std::to_string(u) + FileName);
      ns3TcpSocket->TraceConnectWithoutContext ("BytesInFlight",MakeBoundCallback (&BytesInFlight, stream5));

      app->SetStartTime (Seconds (transientDuration));
      app->SetStopTime (Seconds (simTime - 0.02));
    }
  }
  Simulator::Schedule (Seconds (1.24), &enablelog);

  mmwaveHelper->EnableTraces ();
  //p2ph.EnablePcapAll ("mmwave-epc-simple");

  Simulator::Stop (Seconds (simTime));
  Simulator::Run ();

  /*GtkConfigStore config;
  config.ConfigureAttributes();*/

  Simulator::Destroy ();
  return 0;

}

