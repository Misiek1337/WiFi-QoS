/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2017
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
 * Author: Sebastien Deronne <sebastien.deronne@gmail.com>
 */

#include "ns3/command-line.h"
#include "ns3/config.h"
#include "ns3/uinteger.h"
#include "ns3/log.h"
#include "ns3/yans-wifi-helper.h"
#include "ns3/ssid.h"
#include "ns3/mobility-helper.h"
#include "ns3/internet-stack-helper.h"
#include "ns3/ipv4-address-helper.h"
#include "ns3/udp-client-server-helper.h"
#include "ns3/packet-sink-helper.h"
#include "ns3/packet-sink.h"
#include "ns3/ipv4-global-routing-helper.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/yans-wifi-channel.h"
#include "ns3/flow-monitor-helper.h"
#include "ns3/on-off-helper.h"
#include "ns3/core-module.h"
#include <iostream>
#include <iomanip>

// This is an example to show how to configure an IEEE 802.11 Wi-Fi
// network where the AP and the station use different 802.11 standards.
//
// It outputs the throughput for a given configuration: user can specify
// the 802.11 versions for the AT and the station as well as their rate
// adaptation algorithms. It also allows to decide whether the station,
// the AP or both has/have traffic to send.
//
// Example for an IEEE 802.11ac station sending traffic to an 802.11a AP using Ideal rate adaptation algorithm:
// ./waf --run "wifi-backward-compatibility --apVersion=80211a --staVersion=80211ac --staRaa=Ideal"

using namespace ns3;
using namespace std;

NS_LOG_COMPONENT_DEFINE ("wifi-backward-compatibility");

WifiStandard ConvertStringToStandard (std::string version)
{
  WifiStandard standard = WIFI_STANDARD_80211b;
  if (version == "80211b")
    {
      standard = WIFI_STANDARD_80211b;
    }
  else if (version == "80211g")
    {
      standard = WIFI_STANDARD_80211g;
    }
  else if (version == "80211n")
    {
      standard = WIFI_STANDARD_80211n_2_4GHZ;
    }
  else if (version == "80211ax")
    {
      standard = WIFI_STANDARD_80211ax_2_4GHZ;
    }
  return standard;
}

bool fileExists(const std::string& filename);


int main (int argc, char *argv[])
{
  double simulationTime = 10; //seconds
  double flowStart = 0;
  std::string apVersion = "80211n";
  std::string staVersion = "80211n";
  std::string legacyVersion = "80211b";
  int client_nr = 1;
  int legacyClients = 1;
  std::string outputFileName = "default";

  CommandLine cmd (__FILE__);
  cmd.AddValue ("simulationTime", "Simulation time in seconds", simulationTime);
  cmd.AddValue ("apVersion", "The standard version used by the AP: 80211b, 80211g, 80211n, 80211ax", apVersion);
  cmd.AddValue ("staVersion", "The standard version used by the station: 80211b, 80211g, 80211n, 80211ax", staVersion);
  cmd.AddValue ("clientNr", "Number of clients", client_nr);
  cmd.AddValue ("legacyClients", "Number of legacy clients", legacyClients);
  cmd.AddValue ("legacyVersion", "Legacy standard used by clients", legacyVersion);
  cmd.AddValue ("outputFileName", "Output filename", outputFileName);
  cmd.Parse (argc,argv);


  NodeContainer wifiLegacyNode;
  NetDeviceContainer legacyDevice;


  NodeContainer wifiStaNode;
  wifiStaNode.Create (client_nr);
  if (legacyClients > 0)
{
  wifiLegacyNode.Create (legacyClients);
}
  NodeContainer wifiApNode;
  wifiApNode.Create (1);

  YansWifiChannelHelper channel = YansWifiChannelHelper::Default ();
  YansWifiPhyHelper phy;
  phy.SetChannel (channel.Create ());

  WifiMacHelper mac;
  WifiHelper wifi;
  Ssid ssid = Ssid ("ns3");

InternetStackHelper stack;
//STA config
  wifi.SetStandard (ConvertStringToStandard (staVersion));
  wifi.SetRemoteStationManager ("ns3::IdealWifiManager");

  mac.SetType ("ns3::StaWifiMac",
               "Ssid", SsidValue (ssid));

  NetDeviceContainer staDevice;
  staDevice = wifi.Install (phy, mac, wifiStaNode);
//Node config
  wifi.SetStandard (ConvertStringToStandard (apVersion));
  wifi.SetRemoteStationManager ("ns3::IdealWifiManager");

  mac.SetType ("ns3::ApWifiMac",
               "Ssid", SsidValue (ssid));

  NetDeviceContainer apDevice;
  apDevice = wifi.Install (phy, mac, wifiApNode);

//Legacy node config
  if (legacyClients>0)
{
  wifi.SetStandard (ConvertStringToStandard (legacyVersion));
  wifi.SetRemoteStationManager ("ns3::IdealWifiManager");

  mac.SetType ("ns3::StaWifiMac",
               "Ssid", SsidValue (ssid));


  legacyDevice = wifi.Install (phy, mac, wifiLegacyNode);
}
//Location fo AP
  MobilityHelper mobility;
  Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator> ();
  positionAlloc->Add (Vector (0.0, 0.0, 0.0));
  mobility.SetPositionAllocator (positionAlloc);
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  stack.Install (wifiApNode);
  mobility.Install (wifiApNode);

//Location of nodes

  mobility.SetPositionAllocator ("ns3::RandomDiscPositionAllocator",
                                  "X", StringValue ("0.0"),
                                  "Y", StringValue ("0.0"),
                                  "Rho", StringValue ("ns3::UniformRandomVariable[Min=0|Max=20]"));
  mobility.Install (wifiStaNode);
  stack.Install (wifiStaNode);

//Legacy Nodes location
  if (legacyClients>0)
{
  mobility.SetPositionAllocator ("ns3::RandomDiscPositionAllocator",
                                  "X", StringValue ("0.0"),
                                  "Y", StringValue ("0.0"),
                                  "Rho", StringValue ("ns3::UniformRandomVariable[Min=0|Max=20]"));
  mobility.Install (wifiLegacyNode);
  stack.Install (wifiLegacyNode);
}

  Ipv4AddressHelper address;
  address.SetBase ("192.168.1.0", "255.255.255.0");
  Ipv4InterfaceContainer staNodeInterface;
  Ipv4InterfaceContainer apNodeInterface;

  staNodeInterface = address.Assign (staDevice);
  apNodeInterface = address.Assign (apDevice);


  if (legacyClients>0)
{
  Ipv4InterfaceContainer legacyNodeInterface;
  legacyNodeInterface = address.Assign (legacyDevice);
}

ApplicationContainer sourceApplications, sinkApplications;
  int portNumber = 9;
  for (int index = 0; index < client_nr; ++index) //Loop over all stations (which transmit to the AP)
    {
      auto ipv4 = wifiApNode.Get (0)->GetObject<Ipv4> (); //Get destination's IP interface
      const auto address = ipv4->GetAddress (1, 0).GetLocal (); //Get destination's IP address
      InetSocketAddress sinkSocket (address, portNumber++); //Configure destination socket
      OnOffHelper onOffHelper ("ns3::UdpSocketFactory", sinkSocket); //Configure traffic generator: UDP, destination socket
      onOffHelper.SetConstantRate (DataRate (100e6), 1000);  //Set data rate (150 Mb/s divided by no. of transmitting stations) and packet size [B]
      sourceApplications.Add (onOffHelper.Install (wifiStaNode.Get (index))); //Install traffic generator on station
      PacketSinkHelper packetSinkHelper ("ns3::UdpSocketFactory", sinkSocket); //Configure traffic sink
      sinkApplications.Add (packetSinkHelper.Install (wifiApNode.Get (0))); //Install traffic sink on AP
    }


  if (legacyClients>0)
{
  for (int index = 0; index < legacyClients; ++index) //Loop over all stations (which transmit to the AP)
    {
      auto ipv4 = wifiApNode.Get (0)->GetObject<Ipv4> (); //Get destination's IP interface
      const auto address = ipv4->GetAddress (1, 0).GetLocal (); //Get destination's IP address
      InetSocketAddress sinkSocket (address, portNumber++); //Configure destination socket
      OnOffHelper onOffHelper ("ns3::UdpSocketFactory", sinkSocket); //Configure traffic generator: UDP, destination socket
      onOffHelper.SetConstantRate (DataRate (100e6), 1000);  //Set data rate (150 Mb/s divided by no. of transmitting stations) and packet size [B]
      sourceApplications.Add (onOffHelper.Install (wifiLegacyNode.Get (index))); //Install traffic generator on station
      PacketSinkHelper packetSinkHelper ("ns3::UdpSocketFactory", sinkSocket); //Configure traffic sink
      sinkApplications.Add (packetSinkHelper.Install (wifiApNode.Get (0))); //Install traffic sink on AP
    }
}

  sinkApplications.Start (Seconds (0.5));
  sinkApplications.Stop (Seconds (simulationTime));
//flow monitor

  FlowMonitorHelper flowmon_helper;
  Ptr<FlowMonitor> monitor = flowmon_helper.InstallAll ();

  monitor->SetAttribute ("StartTime", TimeValue (Seconds (flowStart) ) );

  Ipv4GlobalRoutingHelper::PopulateRoutingTables ();

//Sim start
  Simulator::Stop (Seconds (simulationTime + 1));
  Simulator::Run ();

//Output
  if (true) {
    std::ofstream myfile;
    std::string outputCsv = outputFileName + ".csv";
    if (fileExists(outputCsv)) {
      // If the file exists, append to it
      myfile.open (outputCsv, std::ios::app); 
    }
    else {
      // If the file does not exist, create it and set the header line
      myfile.open (outputCsv, std::ios::app);  
      myfile << "Timestamp,client_nr,RngRun,FlowSrc,Throughput" << std::endl;
    }

    //Get timestamp
    auto t = std::time(nullptr);
    auto tm = *std::localtime(&t);

    // Calculate per-flow throughput and print results to file
    double flowThr=0;
    Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier> (flowmon_helper.GetClassifier ());
    std::map<FlowId, FlowMonitor::FlowStats> stats = monitor->GetFlowStats ();
    for (std::map<FlowId, FlowMonitor::FlowStats>::const_iterator i = stats.begin (); i != stats.end (); ++i) {
      Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow (i->first);
      flowThr=i->second.rxBytes * 8.0 / (i->second.timeLastRxPacket.GetSeconds () - i->second.timeFirstTxPacket.GetSeconds ()) / 1e6;
      myfile << std::put_time(&tm, "%Y-%m-%d %H:%M") << "," << client_nr << "," << RngSeedManager::GetRun() << "," << t.sourceAddress << "," << flowThr << std::endl;
    }
    myfile.close();
  }

  double throughput = 0;
  for (uint32_t index = 0; index < sinkApplications.GetN (); ++index) //Loop over all traffic sinks
  {
    uint64_t totalBytesThrough = DynamicCast<PacketSink> (sinkApplications.Get (index))->GetTotalRx (); //Get amount of bytes received
    // std::cout << "Bytes received: " << totalBytesThrough << std::endl;
    throughput += ((totalBytesThrough * 8) / (simulationTime * 1000000.0)); //Mbit/s 
  }
  std::cout << "- network throughput: " << throughput << " Mbit/s" << std::endl;

  Simulator::Destroy ();
}
bool fileExists(const std::string& filename)
{
    std::ifstream f(filename.c_str());
    return f.good();   
}
