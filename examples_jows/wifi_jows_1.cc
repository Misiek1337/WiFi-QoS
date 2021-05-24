/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2016 AGH University of Science and Technology
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as 
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 * Author: Lukasz Prasnal <prasnal@kt.agh.edu.pl>
 */

#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/network-module.h"
#include "ns3/applications-module.h"
#include "ns3/wifi-module.h"
#include "ns3/propagation-module.h"
#include "ns3/mobility-module.h"
#include "ns3/rng-seed-manager.h"
#include "ns3/flow-monitor-helper.h"
#include "ns3/ipv4-flow-classifier.h"

using namespace ns3; 

NS_LOG_COMPONENT_DEFINE ("wifi-qos-test");

class SimulationHelper 
{
public:
	SimulationHelper ();
	static void PopulateArpCache ();
};

SimulationHelper::SimulationHelper () 
{
}

//fullfil the ARP cache prior to simulation run
void
SimulationHelper::PopulateArpCache () 
{
  Ptr<ArpCache> arp = CreateObject<ArpCache> ();
  arp->SetAliveTimeout (Seconds (3600 * 24 * 365) );
	
  for (NodeList::Iterator i = NodeList::Begin (); i != NodeList::End (); ++i) 
    {	
      Ptr<Ipv4L3Protocol> ip = (*i)->GetObject<Ipv4L3Protocol> ();
      NS_ASSERT (ip != 0);
      ObjectVectorValue interfaces;
      ip->GetAttribute ("InterfaceList", interfaces);

      for (ObjectVectorValue::Iterator j = interfaces.Begin (); j != interfaces.End (); j++) 
        {		
          Ptr<Ipv4Interface> ipIface = (*j).second->GetObject<Ipv4Interface> ();
          NS_ASSERT (ipIface != 0);
          Ptr<NetDevice> device = ipIface->GetDevice ();
          NS_ASSERT (device != 0);
          Mac48Address addr = Mac48Address::ConvertFrom (device->GetAddress () );
      
          for (uint32_t k = 0; k < ipIface->GetNAddresses (); k++) 
            {			
              Ipv4Address ipAddr = ipIface->GetAddress (k).GetLocal();		
              if (ipAddr == Ipv4Address::GetLoopback ()) 
                continue;

              ArpCache::Entry *entry = arp->Add (ipAddr);
              Ipv4Header ipv4Hdr;
              ipv4Hdr.SetDestination (ipAddr);
              Ptr<Packet> p = Create<Packet> (100);  
              entry->MarkWaitReply (ArpCache::Ipv4PayloadHeaderPair (p, ipv4Hdr) );
              entry->MarkAlive (addr);
            }
        }
    }

    for (NodeList::Iterator i = NodeList::Begin (); i != NodeList::End (); ++i) 
      {
        Ptr<Ipv4L3Protocol> ip = (*i)->GetObject<Ipv4L3Protocol> ();
		NS_ASSERT (ip != 0);
		ObjectVectorValue interfaces;
		ip->GetAttribute ("InterfaceList", interfaces);

        for (ObjectVectorValue::Iterator j = interfaces.Begin (); j != interfaces.End (); j ++)
          {
            Ptr<Ipv4Interface> ipIface = (*j).second->GetObject<Ipv4Interface> ();
            ipIface->SetAttribute ("ArpCache", PointerValue (arp) );
          }
      }
}



/* ===== main function ===== */

int main (int argc, char *argv[])
{
  uint32_t nSTA = 3;
  uint32_t packetSize = 1472;
  Time appsStart = Seconds (0);
  float simTime = 10;
  float calcStart = 0;
  double Mbps = 54;
  uint32_t seed = 1;


/* ===== Command Line parameters ===== */

  CommandLine cmd;
  cmd.AddValue ("nSTA",      "Number of stations",                 nSTA);
  cmd.AddValue ("pSize",     "Packet size [B]",                    packetSize);
  cmd.AddValue ("end",       "simulation time [s]",                simTime);
  cmd.AddValue ("calcStart", "start of results analysis [s]",      calcStart);
  cmd.AddValue ("Mbps",      "traffic generated per queue [Mbps]", Mbps);
  cmd.AddValue ("seed",      "Seed",                               seed);
  cmd.Parse (argc, argv);

  Time simulationTime = Seconds (simTime);
  ns3::RngSeedManager::SetSeed (seed);
 
  Packet::EnablePrinting ();

  NodeContainer sta;
  sta.Create (nSTA+1);



/* ======== Positioning / Mobility ======= */
  
  Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator> ();
  positionAlloc->Add (Vector (0.0, 0.0, 0.0));
  for (uint32_t i = 0; i < nSTA; i++)
    positionAlloc->Add (Vector (0.0, 0.0, 0.0));

  MobilityHelper mobility;
  mobility.SetPositionAllocator (positionAlloc);
  mobility.SetMobilityModel ("ns3::ConstantVelocityMobilityModel");

  mobility.Install (sta);



/* ===== Propagation Model configuration ===== */

  YansWifiChannelHelper channel = YansWifiChannelHelper::Default ();



/* ===== MAC and PHY configuration ===== */

  YansWifiPhyHelper phy = YansWifiPhyHelper::Default ();
  phy.SetChannel (channel.Create ());

  WifiHelper wifi;
  WifiMacHelper mac; //802.11a
  wifi.SetStandard (WIFI_PHY_STANDARD_80211a);

  //MAC parameters
  //for complete list of available parameters - see Attributes on https://www.nsnam.org/doxygen/classns3_1_1_adhoc_wifi_mac.html#pri-methods
  mac.SetType ("ns3::AdhocWifiMac",
               "QosSupported", BooleanValue (true),
               "Ssid", SsidValue (Ssid ("TEST")) );


  //WiFi Remote Station Manager parameters 

  //Constant Rate setting - see Attributes on https://www.nsnam.org/doxygen/classns3_1_1_constant_rate_wifi_manager.html#pri-attribs
  wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
							    "DataMode", StringValue ("OfdmRate54Mbps") ); 


  NetDeviceContainer staDevices = wifi.Install (phy, mac, sta);




/* ===== Internet stack ===== */

  InternetStackHelper stack;
  stack.Install (sta);

  Ipv4AddressHelper address;

  address.SetBase ("192.168.1.0", "255.255.255.0");
  Ipv4InterfaceContainer staIf;
  staIf = address.Assign (staDevices);



/* ===== Setting applications ===== */

  //Configure traffic destination (sink)
  uint32_t destinationSTANumber = nSTA; //for one common traffic destination
  Ipv4Address destination = staIf.GetAddress (destinationSTANumber);
  Ptr<Node> dest = sta.Get (destinationSTANumber);

  PacketSinkHelper sink ("ns3::UdpSocketFactory", InetSocketAddress (destination, 1000) );
  sink.Install (dest);


  //Configure CBR traffic sources
  DataRate dataRate = DataRate (1000000 * Mbps);

  for (uint32_t i = 0; i < nSTA; i++) 
    {
      Ptr<Node> node = sta.Get(i);

      OnOffHelper cbr  ("ns3::UdpSocketFactory", InetSocketAddress (destination, 1000) );
      cbr.SetConstantRate (dataRate, packetSize);
      cbr.SetAttribute ("StartTime",  TimeValue (appsStart) );
      cbr.SetAttribute ("StopTime",   TimeValue (simulationTime) );

      cbr.Install (node);

    }


/* ===== tracing configuration ====== */

  //phy.EnablePcap ("out", nSTA-1, 0); // sniffing to PCAP file

  //AsciiTraceHelper ascii;
  //phy.EnableAsciiAll (ascii.CreateFileStream ("out.tr"));
  //phy.EnableAscii (ascii.CreateFileStream ("out.tr"), sta.Get (0)->GetDevice (0));

  FlowMonitorHelper flowmon_helper;
  Ptr<FlowMonitor> monitor = flowmon_helper.InstallAll ();
  monitor->SetAttribute ("StartTime", TimeValue (Seconds (calcStart) ) ); //Time from which flowmonitor statistics are gathered.
  monitor->SetAttribute ("DelayBinWidth", DoubleValue (0.001));
  monitor->SetAttribute ("JitterBinWidth", DoubleValue (0.001));
  monitor->SetAttribute ("PacketSizeBinWidth", DoubleValue (20));



/* ===== running simulation ========= */

  SimulationHelper::PopulateArpCache ();
  Simulator::Stop (simulationTime);
  Simulator::Run ();
  Simulator::Destroy ();



/* ===== printing results ===== */

  monitor->CheckForLostPackets ();

  //monitor->SerializeToXmlFile ("out.xml", true, true); // sniffing to XML file
  
  std::string proto;
  //initialize variables for overall results calculation
  uint64_t txBytes = 0, rxBytes = 0, txPackets = 0, rxPackets = 0, lostPackets = 0;
  double throughput;
  Time delaySum = Seconds (0), jitterSum = Seconds (0);

  Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier> (flowmon_helper.GetClassifier ());
  //iterate over traffic flows
  std::map< FlowId, FlowMonitor::FlowStats > stats = monitor->GetFlowStats ();
  for (std::map<FlowId, FlowMonitor::FlowStats>::iterator flow = stats.begin (); flow != stats.end (); flow++)
    {
      Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow (flow->first);

      //print results for given traffic flow
      switch (t.protocol)
        {
          case (6):
            proto = "TCP";
            break;
          case (17):
            proto = "UDP";
            break;
          default:
            exit (1);
        }
      std::cout << "FlowID: " << flow->first << "(" << proto << " "
                << t.sourceAddress << "/" << t.sourcePort << " --> "
                << t.destinationAddress << "/" << t.destinationPort << ")" <<
      std::endl;

      std::cout << "  Tx bytes:\t"     << flow->second.txBytes << std::endl;
      std::cout << "  Rx bytes:\t"     << flow->second.rxBytes << std::endl;
      std::cout << "  Tx packets:\t"   << flow->second.txPackets << std::endl;
      std::cout << "  Rx packets:\t"   << flow->second.rxPackets << std::endl;
      std::cout << "  Lost packets:\t" << flow->second.lostPackets << std::endl;
      if (flow->second.rxPackets > 0)
        {
          //std::cout << "  Throughput:\t"   << flow->second.rxBytes * 8.0 / (flow->second.timeLastRxPacket.GetSeconds ()-flow->second.timeFirstTxPacket.GetSeconds ()) / 1000000  << " Mb/s" << std::endl;
          std::cout << "  Throughput:\t"   << flow->second.rxBytes * 8.0 / (simulationTime - Seconds (calcStart)).GetMicroSeconds ()  << " Mb/s" << std::endl;
          std::cout << "  Mean delay:\t"   << (double)(flow->second.delaySum / (flow->second.rxPackets)).GetMicroSeconds () / 1000 << " ms" << std::endl;    
          if (flow->second.rxPackets > 1)
            std::cout << "  Mean jitter:\t"  << (double)(flow->second.jitterSum / (flow->second.rxPackets - 1)).GetMicroSeconds () / 1000 << " ms" << std::endl;   
          else
            std::cout << "  Mean jitter:\t---"   << std::endl;
        }
      else
        {
          std::cout << "  Throughput:\t0 Mb/s" << std::endl;
          std::cout << "  Mean delay:\t---"    << std::endl;    
          std::cout << "  Mean jitter:\t---"   << std::endl;
        }

      //increase variables for overall results calculation
      txBytes     += flow->second.txBytes;
      rxBytes     += flow->second.rxBytes;
      txPackets   += flow->second.txPackets;
      rxPackets   += flow->second.rxPackets;
      lostPackets += flow->second.lostPackets;
      //throughput  += (flow->second.rxPackets > 0 ? flow->second.rxBytes * 8.0 / (flow->second.timeLastRxPacket.GetSeconds ()-flow->second.timeFirstTxPacket.GetSeconds ()) / 1000000 : 0);
      throughput  += (flow->second.rxPackets > 0 ? flow->second.rxBytes * 8.0 / (simulationTime - Seconds (calcStart)).GetMicroSeconds () : 0);
      delaySum    += flow->second.delaySum;
      jitterSum   += flow->second.jitterSum;
    }


  //print overall results
  std::cout << "=======================Total: =====================================" << std::endl;

  std::cout << "  Tx bytes:\t"     << txBytes     << std::endl;
  std::cout << "  Rx bytes:\t"     << rxBytes     << std::endl;
  std::cout << "  Tx packets:\t"   << txPackets   << std::endl;
  std::cout << "  Rx packets:\t"   << rxPackets   << std::endl;
  std::cout << "  Lost packets:\t" << lostPackets << std::endl;
  std::cout << "  Throughput:\t"   << throughput  << " Mb/s" << std::endl;
  if (rxPackets > 0)
    {
      std::cout << "  Mean delay:\t"   << (double)(delaySum / (rxPackets)).GetMicroSeconds () / 1000 << " ms" << std::endl;    
      if (rxPackets > 1)  
        std::cout << "  Mean jitter:\t"  << (double)(jitterSum / (rxPackets - 1)).GetMicroSeconds () / 1000  << " ms" << std::endl;   
      else
        std::cout << "  Mean jitter:\t---"   << std::endl;
    }
  else
    {
      std::cout << "  Mean delay:\t---"    << std::endl;    
      std::cout << "  Mean jitter:\t---"   << std::endl;
    }


  return 0;
}
