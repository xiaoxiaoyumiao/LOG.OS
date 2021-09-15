# Miscellaneous

TODO - reorganize these things

* iperf3
  * set up server: `iperf3 -s -p [port_number]` 
  * connect to a server `iperf3 -c [ip] -p [port_number] -t [time]` 
    * ref: [https://support.cumulusnetworks.com/hc/en-us/articles/216509388-Throughput-Testing-and-Troubleshooting](https://support.cumulusnetworks.com/hc/en-us/articles/216509388-Throughput-Testing-and-Troubleshooting)
  * How to interpret the throughput values in the result?
    * ref: [https://unix.stackexchange.com/questions/257143/how-do-i-interpret-iperf3-results](https://unix.stackexchange.com/questions/257143/how-do-i-interpret-iperf3-results)
  * 传输时收端收到的数据显著少于发端发出的数据：可能是因为其测试原理导致最后一段发出的数据未被接收
    * ref: [https://github.com/esnet/iperf/issues/382](https://github.com/esnet/iperf/issues/382)
* mininet
  * walkthrough & cheatsheet
    * [https://github.com/mininet/mininet/wiki/Introduction-to-Mininet\#what](https://github.com/mininet/mininet/wiki/Introduction-to-Mininet#what)
    * [http://mininet.org/walkthrough/](http://mininet.org/walkthrough/)
    * show nodes, ports, port numbers and net topology: [https://stackoverflow.com/questions/30989366/how-to-figure-out-port-information-in-mininet](https://stackoverflow.com/questions/30989366/how-to-figure-out-port-information-in-mininet)
  * trace a packet with wireshark
    * `mininet> h1 wireshark &` 
    * ref: [https://github.com/mininet/mininet/issues/341](https://github.com/mininet/mininet/issues/341)
* open vswitch
  * OpenFlow spec
    * [https://opennetworking.org/wp-content/uploads/2013/04/openflow-spec-v1.3.1.pdf](https://opennetworking.org/wp-content/uploads/2013/04/openflow-spec-v1.3.1.pdf)
  * ovs-vsctl, ovs-ofctl, ovs-appctl
  * cheatsheet
    * [https://randomsecurity.dev/posts/openvswitch-cheat-sheet/](https://randomsecurity.dev/posts/openvswitch-cheat-sheet/)
    * [https://gist.github.com/djoreilly/c5ea44663c133b246dd9d42b921f7646](https://gist.github.com/djoreilly/c5ea44663c133b246dd9d42b921f7646)
    * [https://docs.pica8.com/pages/viewpage.action?pageId=3083175\#:~:text=ovs%2Dofctl%20add%2Dflow%20dl\_vlan%3D%3C,ranges%20from%200%20to%204095.](https://docs.pica8.com/pages/viewpage.action?pageId=3083175#:~:text=ovs%2Dofctl%20add%2Dflow%20dl_vlan%3D%3C,ranges%20from%200%20to%204095.)
    * queue: [https://docs.pica8.com/display/picos292cg/Configuring+QoS+Queue](https://docs.pica8.com/display/picos292cg/Configuring+QoS+Queue)
  * version negotiation failed
    * 使用 --protocol 参数
    * ref: [https://bugzilla.redhat.com/show\_bug.cgi?id=1278136](https://bugzilla.redhat.com/show_bug.cgi?id=1278136)

