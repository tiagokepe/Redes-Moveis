/* 
 *  Copyright (C) 2007 Dip. Ing. dell'Informazione, University of Pisa, Italy
 *  http://info.iet.unipi.it/~cng/ns2mesh80216/
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA, USA
 */

#ifndef __NS2_WIMSH_MAC_H
#define __NS2_WIMSH_MAC_H

#include <wimax_common.h>
#include <t_timers.h>
#include <wimsh_phy.h>
#include <rng.h>

#include <vector>
#include <map>
#include <set>

class WimshPhyMib;
class WimshMacMib;
class WimshChannel;
class WimshPhy;
class WimshScheduler;
class WimaxSdu;
class WimshMshDsch;
class WimshMshNcfg;
class WimshMshNent;
class WimshTopology;
class WimshFragmentationBuffer;
class WimaxReassemblyBuffer;
class WimshBwManager;
class WimshForwarding;
class WimshCoordinator;
class WimshPhyMib;
class WimshBurst;

class Packet;
class LL;

//! WiMAX MAC management information base.
class WimshMacMib : public TclObject {
	//! Timer to count frames.
	TTimer<WimshMacMib> timer_;
	//! Pointer to the PHY MIB.
	WimshPhyMib* phyMib_;
	//! Frame counter. There should be no wrap-around (1 ms -> 49 days).
	unsigned int frame_;
	//! Map from ns2 packet flow ID to CRC indicator.
	/*!
	  This allows the user to specify via TCL the presence of CRC
	  at the 802.16 MAC layer on a flow by flow basis.

	  The CRC is disabled by default.
	  */
	std::map<int, bool> flow2crc_;
	//! Map from ns2 packet flow ID to 802.16 MAC priority.
	/*!
	  This allows the user to specify via TCL the 802.16 MAC priority
	  of PDUs on a flow by flow basis.

	  The priority if zero by default
	  */
	std::map<int, unsigned char> flow2prio_;
	//! Map from ns2 packet flow ID to 802.16 MAC precedence.
	/*!
	  This allows the user to specify via TCL the 802.16 MAC precedence
	  of PDUs on a flow by flow basis.

	  The precedence is zero by default.
	  */
	std::map<int, unsigned char> flow2drop_;


	//! Link structure
	/*!
	  Link structure that holds the source node and destination node.
	  */
	struct Link {
		//! Source node
		WimaxNodeId src_;
		//! Destination node
		WimaxNodeId dst_;
		//! Current burst profile
		wimax::BurstProfile profile_;
		//! Constructor
		Link( WimaxNodeId src = 0, WimaxNodeId dst = 0, 
			wimax::BurstProfile p = wimax::QPSK_1_2 ) {
			src_ = src; dst_ = dst; profile_ = p;
		}
	};
	
	//! Link list
	std::list< Link > linkList_;
public:
	//! Create an empty MAC MIB.
	WimshMacMib () : timer_(this), phyMib_(0), frame_ (0) { }
	//! Do nothing.
	virtual ~WimshMacMib () { }

	//! Handle the next frame timer.
	void handle ();

	//! Return the current frame number.
	unsigned int frame () { return frame_; }

	//! Get the CRC indicator for a given flow number.
	bool flow2crc (int flowId) {
		return ( flow2crc_.count(flowId) == 1 ) ? flow2crc_[flowId] : false; }
	//! Get the priority for a given flow number.
	unsigned char flow2prio (int flowId) {
		return ( flow2prio_.count(flowId) == 1 ) ? flow2prio_[flowId] : 0; }
	//! Get the drop precedence for a given flow number.
	unsigned char flow2drop (int flowId) {
		return ( flow2drop_.count(flowId) == 1 ) ? flow2drop_[flowId] : 0; }
	//! Update burst profile
	void updateBurstProfile ( WimaxNodeId src, WimaxNodeId dst, wimax::BurstProfile p );
	//! Get burst profile
	wimax::BurstProfile getBurstProfile ( WimaxNodeId src, WimaxNodeId dst );

protected:
	//! Tcl interface.
	/*!
	  This function is also used as a demultiplex for sending commands
	  to all member data structures of MAC that do not have a Tcl
	  shadow object (eg. scheduler, bandwidth manager).
	  
	  The PHY MIB must be initialized via the tcl command "phymib", which
	  also starts the frame counter timer.
	  The first next-frame event is scheduled a few us before the actual
	  frame start time. This is to make sure that all the subsequent
	  frame counters are updated before other events (such as MSH-DSCH
	  messages scheduling).
	  */
	virtual int command (int argc, const char*const* argv);
};

//! WiMAX MAC layer.
class WimshMac : public TclObject {

	//! Sponsor channel open procedure finite state machine.
	enum SponsorState { SS_IDLE, SS_SEND_REQ, SS_SEND_ACK, SS_SEND_OPEN };

	//! Link establishment procedure finite state machine.
	enum LinkEstState { LE_IDLE, LE_SEND_CHALLENGE, LE_WAIT_RESPONSE,
                       LE_SEND_RESPONSE, LE_SEND_ACK };

	//! Link quality descriptor.
	struct LinkQuality {
		//! True if the link is good, false otherwise.
		bool good_;
		//! Number of messages before status transition.
		unsigned int frames_;
		//! Creates a good link.
		LinkQuality () {
			good_ = true;
			frames_ = 0;
		}
	};

	//! PHY MIB. Initialized via tcl command.
	WimshPhyMib* phyMib_;
	//! MAC MIB. Initialized via tcl command.
	WimshMacMib* macMib_;

	//! Link layer. Initialized via tcl command.
	LL* ll_;
	//! Network topology. Initialized via tcl command.
	WimshTopology* topology_;

	//! Packet scheduler. Initialized via tcl command.
	WimshScheduler* scheduler_;

	//! Map each neighbor to a local numerical identifier for other structures.
	/*!
	  Initialized via a the initialization() function from the topology.

	  The 'index' stored by this data structure can be seen as the LinkID
	  in a real 802.16 system, which is chosen randomly (between 0 and 255)
	  during the link establishment negotation phase, which is not
	  simulated.
	  */
	std::map<WimaxNodeId, unsigned int> neigh2ndx_;
	//! Map each neighbor local numerical identifier to the NodeID.
	std::vector<WimaxNodeId> ndx2neigh_;

	//! Array of alpha values (one for each neighbor).
	/*!
	  This data structure is initialized via the "profile" tcl command.
	  The entry i of this array stores the number of coded bytes that
	  are conveyed by each OFDM symbol to transmit data from this node to
	  the neighbor mapper to the index i via neigh2ndx_.

	  We assume that the burst profile is bi-directional.
	  */
	std::vector<unsigned int> alpha_;
	//! Array of burst profiles (one for each neighbor).
	/*!
	  This data structure is initialized via the "profile" tcl command.
	  The entry i of this array stores the burst profile used
	  to transmit data from this node to
	  the neighbor mapper to the index i via neigh2ndx_.
	  */
	std::vector<wimax::BurstProfile> profile_;

	//! Array of fragmentation buffers (one for each neighbor).
	/*!
	  This data structure is created by the initialize() function.
	  */
	std::vector<WimshFragmentationBuffer*> fragbuf_;

	//! Array of reassembly buffers (one for each neighbor).
	/*!
	  This data structure is created by the initialize() function.
	  */
	std::vector<WimaxReassemblyBuffer*> reasbuf_;

	//! Array of estimated interval times between two consecutive opportunities.
	std::vector<double> hNeigh_;

	//! Last time that the neighbors sent an MSH-DSCH message.
	std::vector<double> hNeighLast_;

	//! My estimated interval time between two consecutive opportunities.
	double hSelf_;

	//! Last time that this node sent an MSH-DSCH message.
	double hLast_;
	//! Last time that this node sent an MSH-NCFG message.
	double hLastNcfg_;
	//! Last time that this node had an opportunity to send an MSH-NENT message.
	double hLastNent_;

	//! Weight of the current sample for estimating H values.
	double hEstCurr_;
	//! Weight of the past estimated value for estimating H values.
	double hEstPast_;

	//! State of the sponsor channel open procedure. Started via Tcl.
	SponsorState sponsorState_;
	//! Sponsor channel open procedure start time.
	double sponsorStart_;
	//! Sponsor node ID. Set via Tcl.
	WimaxNodeId sponsorId_;

	//! State of the link establishment procedure. Started via Tcl.
	LinkEstState linkEstState_;
	//! Link establishment procedure start time.
	double linkEstStart_;
	//! Current index of the neighbor with which a link is being established.
	unsigned int linkEstCurrent_;
	//! ID of the new node.
	WimaxNodeId linkEstId_;

	//! True if the node is in scan mode.
	bool scanMode_;
	//! Scan procedure start time.
	double scanStart_;
	//! Set of neighbors from which an MSH-NCFG message has been received.
	std::set<WimaxNodeId> scanSet_;

	//! Array of PHY (if > 1 then the node is multiradio). Initialized via tcl.
	/*!
	  During control frames, only one PHY can be used, even though there are
	  more than one. We assume that phy_[0] is always used for that purpose.
	  */
	std::vector<WimshPhy*> phy_;

	//! Array of the available channels. Initialized via tcl.
	/*!
	  The list of available channels is the same for all the 802.16 nodes in
	  the same network. Channels are identified IRL using a numerical
	  identifier from 0 to 15 (4 bits), which are used (for instance) in
	  bandwidth availabilities/grants. We do not map the channel pointers
	  to the numerical identifiers, since we assume that all nodes have
	  the same list of channels. Thus, the channel identifier is simply
	  the array entry index.
	  
	  If there are multiple channels, then the control channel is iterated
	  at the start of each frame.
	  */
	std::vector<WimshChannel*> channel_;

	//! Bandwidth request manager. Created by initialize().
	WimshBwManager* bwmanager_;

	//! Forwarding module. Created by initialize().
	WimshForwarding* forwarding_;

	//! Distributed election coordination module. Created by initialize().
	WimshCoordinator* coordinator_;

	//! NodeID (equal to the ns2 node ID). Initialized via tcl.
	WimaxNodeId nodeId_;

	//! Number of neighbors of this node.
	unsigned int nneighs_;

	//! Used to enforce the initialize function to be called only once.
	bool initialized_;

	//! True if this node has to measure the estimation accuracy of H values.
	bool hErrorTagged_;

	//! This index is only used for statistical purposes. Set via Tcl.
	unsigned int index_;

	//! Average number of consecutive corrupted MSH-DSCH messages.
	double mshDschAvgError_;
	//! Average number of consecutive good MSH-DSCH messages.
	double mshDschAvgGood_;
	//! Random number generator to select the bad period duration.
	RNG mshDschRngError_;
	//! Random number generator to select the good period duration.
	RNG mshDschRngGood_;
	//! Array of link quality indicators, one for each neighbor.
	std::vector<LinkQuality> mshDschLinkQuality_;
	
public:
	//! Build an empty MAC.
	WimshMac ();
	//! We probably do not want to ever destroy MAC objects. Thus, do nothing.
	virtual ~WimshMac () { }

	//! Receive an IP datagram from LL.
	void recvPacket (Packet* pkt);
	//! Receive a MAC SDU.
	/*!
	  SDUs are either IP datagrams received from LL (funcion recvPacket)
	  encapsulated by the MAC, or SDUs received from the reassembly buffers.
	  */
	void recvSdu (WimaxSdu* sdu);
	//! Receive an MSH-DSCH.
	void recvMshDsch (WimshMshDsch* dsch, double txtime);
	//! Receive an MSH-NCFG
	void recvMshNcfg (WimshMshNcfg* ncfg, double txtime);
	//! Receive an MSH-NENT.
	void recvMshNent (WimshMshNent* nent, double txtime);
	//! Receive a burst, either control or data, from PHY.
	void recvBurst (WimshBurst* burst);

	//! Receive a partial MSH-DSCH from the coordinator and send it to the PHY.
	/*!
	  The coordinator has just allocated a new MSH-DSCH, containing the
	  information about the distributed election algorithms of neighbors.
	  We now pass this MSH-DSCH message to the bandwidth manager, whic
	  fills the information elements about bandwidth request/grants, and
	  then send it to the PHY layer.
	  */
	void opportunity (WimshMshDsch* dsch);
	//! Receive a partial MSH-NCFG from the coordinator and send it to the PHY.
	void opportunity (WimshMshNcfg* ncfg);
	//! Receive a partial MSH-NENT from the coordinator and send it to the PHY.
	void opportunity (WimshMshNent* nent);

	//! Get the NodeID.
	WimaxNodeId nodeId () { return nodeId_; }

	//! Set the control PHY/channel to the specified mode.
	/*!
	  The standard specifies that all nodes in the network use the same
	  channel to transmit control messages, so that all nodes in the
	  neighborhood can hear them. However it does not specify which of the
	  available channel is used to this purpose.

	  Every node uses channel number 0, and transmits using the only
	  available physical interface (at the moment, the implementation
	  of this MAC layer does not support multiple radios).
	  */
	void setControlChannel (wimax::ChannelStatus s);

	//! Return the current frame number.
	unsigned int frame () { return macMib_->frame(); }
	//! Return the MAC MIB used by this MAC.
	WimshMacMib* macMib () { return macMib_; }
	//! Return the PHY MIB used by this MAC.
	WimshPhyMib* phyMib () { return phyMib_; }
	//! Return the topology.
	WimshTopology* topology () { return topology_; }
	//! Return the scheduler.
	WimshScheduler* scheduler () { return scheduler_; }
	//! Return the bandwidth manager.
	WimshBwManager* bwmanager () { return bwmanager_; }

	//! Return the internal 'index' of a neighbor.
	/*!
	  We assume that n is a neighbor of this node. If it is not, then
	  the neigh2ndx_ will contain a spurious entry which refers to the
	  index 0, which is definitely not what we want to happen.
	  */
	unsigned int neigh2ndx (WimaxNodeId n) { return neigh2ndx_[n]; }

	//! Return the NodeID of the local identifier of a neighbor.
	WimaxNodeId ndx2neigh (unsigned int n) { return ndx2neigh_[n]; }

	//! Return the alpha associated to a local identifier.
	unsigned int alpha (unsigned int n) { return alpha_[n]; }

	//! Convert bytes to minislot for a given neighbor.
	/*!
	  Data bursts must pre-pended by a physical preamble, which consists
	  of one OFDM symbol. The presence of the preamble is indicated through
	  the function argument preamble, which defaults to true.
	 */
	/*
    *     __                       __
    *    |      _           _        |
	 *    |     |    bytes	  |       |
    *    |     |  ---------  | + P   |    P == 1 if preamble is present
	 *    |     |  alpha_[n]  |       |    P == 0 otherwise
	 *    |  -----------------------  |
	 *    |   phyMib_->symPerSlot()   |
	 *    |                           |
	 *    |                           |
	 */
	unsigned int bytes2slots (unsigned int n,
			                    unsigned int bytes,
									  bool preamble = false) {
		if ( bytes == 0 ) return 0;
		int p = ( preamble == true ) ? phyMib_->symShortPreamble() : 0;
		return 1 + ( p + ( bytes - 1 ) / alpha_[n] ) / phyMib_->symPerSlot(); }
		// return 1 + ( bytes - 1 ) / (phyMib_->symPerSlot() * alpha_[n]); }

	//! Convert minislots to bytes for a given neighbor.
	/*!
	  Data bursts must pre-pended by a physical preamble, which consists
	  of one OFDM symbol. The presence of the preamble is indicated through
	  the function argument preamble, which defaults to true.
	 */
	unsigned int slots2bytes (unsigned int n,
			                    unsigned int slots,
									  bool preamble = false ) {
		int p = ( preamble == true ) ? phyMib_->symShortPreamble() : 0;
		return alpha_[n] * ( slots * phyMib_->symPerSlot() - p ); }

	//! Return the estimated interval between consecutive opportunities.
	double h (WimaxNodeId x) {
		return ( x == nodeId_ ) ? hSelf_ : hNeigh_[neigh2ndx_[x]]; }

	//! Receive data from a given channel. Single-radio only.
	void receive (unsigned int channel);

	//! Transmit data over a given channel. Single-radio only.
	void transmit (unsigned int range, WimaxNodeId dst, unsigned int channel);

	//! Return the number of neighbors of this node.
	unsigned int nneighs () { return nneighs_; }

	//! Return the number of available channels.
	unsigned int nchannels () { return channel_.size(); }

	//! Return the index for statistical purposes.
	unsigned int index () { return index_; }

protected:
	//! Tcl interface.
	virtual int command (int argc, const char*const* argv);

private:
	//! Initialize MAC data structures, provided that all objects are defined.
	void initialize ();

	//! Utility function to update H.
	void updateH (double& h, double& last);

	//! Draws a value from a geometric distribution.
	unsigned int geometric (RNG& rng, double p) {
		return ( p == 1.0 ) ? 1 :
			(unsigned int) ceil ( log (rng.uniform()) / log (1 - p));
	}
};

#endif // __NS2_WIMSH_MAC_H
