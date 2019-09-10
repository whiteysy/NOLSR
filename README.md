NOLSR - Named Data Optimized Link State Routing Protocol
=============================================
## Overview
NOLSR is a routing protocol designed for NDN-MANET based on OLSR. The main function of NOLSR is to provide a routing protocol to populate NFD's FIB.  

It is a proactive routing protocol based on link state. Neighbors periodically send hello-data to discover and maintain neighbor relationships. Only multipiont relay(MPR) will forward LSA to reduce flooding of LSAs. Each node uses the Dijkstra algorithm to calculate the route.

Source code structure
---------------

The implementation of NOLSR leverages the framework of NLSR source code framework. In order to realize the NOLSR's own functions, it has been greatly modified. Details are as follows:

- New file: (1) hello-message.cpp/hpp: Provide the format of hello-data and its serialization and deserialization.
            (2) tuple-state.cpp/hpp: Provide the addition, deletion and alteration of network information.
            (3) tuple.hpp:Provide the storage of network information(such as neighbor, two-hop neighbor, mpr, mpr-selector).
            
- Modified file: nlsr.cpp/hpp, lsbd.cpp/hpp, hello-protocol.cpp/hpp, routing-calculator.cpp/hpp...

Installation
----------------------

Execute the following commands to build NOLSR:

 (1)./waf configure
 (2)./waf
 (3)sudo ./waf install

After building NOLSR, it will generate an executable file of c++ in the path "usr/local/bin". The running of NOLSR also need NFD, ndn-cxx.


