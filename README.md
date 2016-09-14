## Predictive-OLSR

Predictive-OLSR was developed at [École Polytechnique Fédérale de Lausanne](http://epfl.ch) for flying fixed-wing drones.

Details: [http://smavnet.epfl.ch/](http://smavnet.epfl.ch/)


### Predictive-OLSR

In the [SMAVNET](http://smavnet.epfl.ch/) project we showed that due to the high mobility of the nodes,
sometimes the existing network routing algorithms, which have been designed for mobile ad-hoc networks
(MANETs) fail to provide a reliable communication. Therefore we proposed Predictive-OLSR an extension to
the Optimized Link-State Routing (OLSR) protocol to enable efficient routing in even very dynamic conditions.
The key idea is to exploit GPS information to aid the routing protocol. [Predictive-OLSR](http://smavnet.epfl.ch/)
weights the expected transmission count (ETX) metric, taking into account the relative speed between the nodes.


![alt text](http://smavnet.epfl.ch/images/ebee_little2.png "eBee")


Originally Predictive-OLSR was prepared based on OLSRd ver. 0.6.5.2.
This branch provides Predictive-OLSR based on the recent version of OLSR/olsrd.

### Acknowledgments

This work is supported by [armasuisse](http://www.ar.admin.ch/en/home.html) competence sector.
Science+Technology for the Swiss Federal Department of Defense, Civil Protection and Sport.
We would also like to thank [SenseFly](https://www.sensefly.com) for providing the robotic platforms and support.


