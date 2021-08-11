#include "ns3/point-to-point-module.h"
#include "ns3/ipv4-global-routing-helper.h"
#include <fstream>
#include <string>
#include "ns3/core-module.h"
#include "ns3/applications-module.h"
#include "ns3/mobility-module.h"
#include "ns3/config-store-module.h"
#include "ns3/netanim-module.h"
#include "ns3/pso-mobility-model.h"
#include "ns3/mobility-helper.h"

using namespace ns3;

int main (int argc, char *argv[])
{
  CommandLine cmd;
  cmd.Parse (argc, argv);
  NodeContainer c;
  c.Create (10); // 10 nodes
  
  /** Mobility Model **/
  /*******************************************************************************/
  MobilityHelper mobility;
  mobility.SetMobilityModel ("ns3::PSOMobilityModel");
  mobility.SetPositionAllocator ("ns3::RandomBoxPositionAllocator",
    "X", StringValue ("ns3::UniformRandomVariable[Min=0|Max=100]"),
    "Y", StringValue ("ns3::UniformRandomVariable[Min=0|Max=100]"),
    "Z", StringValue ("ns3::UniformRandomVariable[Min=0|Max=100]"));
  mobility.Install (c);
  AnimationInterface anim("PSO.xml");
  AsciiTraceHelper ascii;
  mobility.EnableAsciiAll(ascii.CreateFileStream("PSOm.tr"));
  Simulator::Stop (Seconds (500.0));
  Simulator::Run ();
  Simulator::Destroy ();
  return 0;
}
