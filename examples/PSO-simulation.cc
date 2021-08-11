#include "ns3/point-to-point-module.h"
#include "ns3/ipv4-global-routing-helper.h"
#include <fstream>
#include <string>
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/applications-module.h"
#include "ns3/mobility-module.h"
#include "ns3/config-store-module.h"
#include "ns3/wifi-module.h"
#include "ns3/wifi-radio-energy-model-helper.h"
#include "ns3/energy-module.h"
#include "ns3/aodv-helper.h"
#include "ns3/internet-module.h"
#include "ns3/netanim-module.h"
#include "ns3/pso-mobility-model.h"
#include "ns3/mobility-helper.h"

using namespace ns3;
NS_LOG_COMPONENT_DEFINE ("Mob");

void ReceivePacket (Ptr<Socket> socket)
{
  while (socket->Recv ())
    {
      NS_LOG_UNCOND ("Received one packet!");
    }
}

static void GenerateTraffic (Ptr<Socket> socket, uint32_t pktSize,
                             uint32_t pktCount, Time pktInterval )
{
  if (pktCount > 0)
    {
      socket->Send (Create<Packet> (pktSize));
      Simulator::Schedule (pktInterval, &GenerateTraffic,
                           socket, pktSize,pktCount - 1, pktInterval);
    }
  else
    {
      socket->Close ();
    }
}

// trace functions for energy model

// trace function for remaining energy at node
void
RemainingEnergy (double oldValue, double remainingEnergy)
{
  NS_LOG_UNCOND (Simulator::Now ().GetSeconds ()
  		  << "s Current remaining energy = " << remainingEnergy << "J");
}

// trace function for total energy consumption at node
void
TotalEnergy (double oldValue, double totalEnergy)
{
  NS_LOG_UNCOND (Simulator::Now ().GetSeconds ()
                 << "s Total energy consumed by radio = " << totalEnergy << "J");
}

int main (int argc, char *argv[])
{
  CommandLine cmd;
  cmd.Parse (argc, argv);
  NodeContainer c;
  c.Create (10); // 10 nodes
  Address serverAddress;
  WifiHelper wifi;
  wifi.SetStandard (WIFI_PHY_STANDARD_80211b);
  WifiMacHelper mac;
  mac.SetType ("ns3::AdhocWifiMac");
  wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
                                "DataMode", StringValue ("OfdmRate54Mbps"));
  YansWifiPhyHelper wifiPhy = YansWifiPhyHelper::Default ();
  YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default ();
  wifiChannel.SetPropagationDelay ("ns3::ConstantSpeedPropagationDelayModel");
  wifiChannel.AddPropagationLoss ("ns3::FriisPropagationLossModel");
  wifiPhy.SetChannel (wifiChannel.Create ());
  NetDeviceContainer cDevices = wifi.Install (wifiPhy, mac, c);
  NS_LOG_INFO ("Enabling AODV routing on all backbone nodes");
  AodvHelper aodv;
  InternetStackHelper internet;
  internet.SetRoutingHelper (aodv); // has effect on the next Install ()
  internet.Install (c);
  Ipv4AddressHelper ipAddrs;
  ipAddrs.SetBase ("192.168.0.0", "255.255.255.0");
  Ipv4InterfaceContainer cInterfaces;
  cInterfaces=ipAddrs.Assign (cDevices);
  serverAddress = Address(cInterfaces.GetAddress (0));
  TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");
  Ptr<Socket> recvSink = Socket::CreateSocket (c.Get (0), tid);
  InetSocketAddress local = InetSocketAddress (Ipv4Address::GetAny (), 80);
  recvSink->Bind (local);
  recvSink->SetRecvCallback (MakeCallback (&ReceivePacket));
  Ptr<Socket> source = Socket::CreateSocket (c.Get (1), tid);
  InetSocketAddress remote = InetSocketAddress (Ipv4Address ("255.255.255.255"), 80);
  source->SetAllowBroadcast (true);
  source->Connect (remote);
  
  /** Energy Model **/
  /***************************************************************************/
  /* energy source */
  BasicEnergySourceHelper basicSourceHelper;
  // configure energy source
  basicSourceHelper.Set ("BasicEnergySourceInitialEnergyJ", DoubleValue (100.0));
  // install source
  EnergySourceContainer sources = basicSourceHelper.Install (c);
  /* device energy model */
  WifiRadioEnergyModelHelper radioEnergyHelper;
  // configure radio energy model
  radioEnergyHelper.Set ("TxCurrentA", DoubleValue (0.0174));
  // install device model
  DeviceEnergyModelContainer deviceModels = radioEnergyHelper.Install (cDevices, sources);
  /***************************************************************************/
  
  // connect trace sources
  // all sources are connected to node 1
  Ptr<BasicEnergySource> basicSourcePtr = DynamicCast<BasicEnergySource> (sources.Get (1));
  basicSourcePtr->TraceConnectWithoutContext ("RemainingEnergy", MakeCallback (&RemainingEnergy));
  // device energy model
  Ptr<DeviceEnergyModel> basicRadioModelPtr =
  basicSourcePtr->FindDeviceEnergyModels ("ns3::WifiRadioEnergyModel").Get (0);
  NS_ASSERT (basicRadioModelPtr != NULL);
  basicRadioModelPtr->TraceConnectWithoutContext ("TotalEnergyConsumption",MakeCallback (&TotalEnergy));
  
  /** Mobility Model **/
  /*******************************************************************************/
  MobilityHelper mobility;
  mobility.SetMobilityModel ("ns3::PSOMobilityModel");
  mobility.SetPositionAllocator ("ns3::RandomBoxPositionAllocator",
    "X", StringValue ("ns3::UniformRandomVariable[Min=0|Max=100]"),
    "Y", StringValue ("ns3::UniformRandomVariable[Min=0|Max=100]"),
    "Z", StringValue ("ns3::UniformRandomVariable[Min=0|Max=100]"));
  mobility.Install (c); // tether nodes to mobility model
  wifiPhy.EnablePcap ("PSO", cDevices);
  //Ascii Trace Metrics can be processed using Tracemetrics Software.
  AnimationInterface anim("PSO.xml");
  AsciiTraceHelper ascii;
  wifiPhy.EnableAsciiAll(ascii.CreateFileStream("PSO.tr"));
  mobility.EnableAsciiAll(ascii.CreateFileStream("PSOm.tr"));
  //Simulator::Schedule (Seconds (1), &PSOMobilityModel::Update);
  Simulator::Stop (Seconds (500.0));
  Simulator::Run ();
  for (DeviceEnergyModelContainer::Iterator iter = deviceModels.Begin (); iter != deviceModels.End (); iter ++)
      {
        double energyConsumed = (*iter)->GetTotalEnergyConsumption ();
        NS_LOG_UNCOND ("End of simulation (" << Simulator::Now ().GetSeconds ()
                       << "s) Total energy consumed by radio = " << energyConsumed << "J");
        //NS_ASSERT (energyConsumed <= 1.0);
      }
  Simulator::Destroy ();
  return 0;
}
