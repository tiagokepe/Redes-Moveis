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

#ifndef __NS2_WIMSH_BW_MANAGER_FRR_H
#define __NS2_WIMSH_BW_MANAGER_FRR_H

#include <wimsh_weight_manager.h>
#include <wimsh_bwmanager.h>
#include <wimsh_packet.h>
#include <wimsh_mac.h>

#include <rng.h>

//! Fair round robin bandwidth manager for 802.16. Single-radio only.
/*!
  :TODO: more documentation. Much more.
  */
class WimshBwManagerFairRR : public WimshBwManager {
	//! Typedef for an array of bitsets (representing slots within frames).
	typedef std::vector< std::bitset<MAX_SLOTS> > Bitmap;

protected:
	//! Descriptor of the internal status used by grantFit().
	struct grantFitDesc {
		//! True if this is the first time that it is called during granting.
		bool first;
		//! Last channel to be considered.
		unsigned int chSum;
		//! Current channel.
		unsigned int ch;
		//! Create an uninitialized descriptor.
		grantFitDesc () : first (true) { }
	};

	//! Descriptor of neighbor information for bandwidth requesting/granting.
	struct NeighDesc {
		//! Cumulative size, in bytes, of the bandwidth requests received.
		/*!
		  This data structure is used when granting bandwidth to neighbors.
		  */
		unsigned int req_in_;
		//! Cumulative size, in bytes, of the bandwidth grants sent.
		/*!
		  This data structure is used when granting bandwidth to neighbors.
		  */
		unsigned int gnt_in_;
		//! Cumulative size, in bytes, of the bandwidth confirmations received.
		/*!
		  This data structure is used when granting bandwidth to neighbors.
		  */
		unsigned int cnf_in_;

		//! Cumulative size, in bytes, of the bandwidth requests sent.
		/*!
		  :TODO: documentation
		  */
		unsigned int req_out_;
		//! Cumulative size, in bytes, of the bandwidth grants received.
		/*!
		  :TODO: documentation
		  */
		unsigned int gnt_out_;
		//! Cumulative size, in bytes, of the bandwidth confirmations sent.
		/*!
		  :TODO: documentation
		  */
		unsigned int cnf_out_;

		//! True if we received a conf. from a node before we sent it a grant.
		bool rcvCnf_;

		//! Backlog for which no request has been issued, in bytes.
		/*!
		  This value is continuosly incremented by the scheduler.
		  On the other hand, it is decremented when this nodes sends
		  a bandwidth request.
		  */
		unsigned int backlog_;

		//! Deficit counter of the input link, in bytes.
		/*!
		  During bandwidth request, we increment the deficit counter of
		  any encountered input link, up to a maximum value, so that
		  this input link will lag some bandwidth granting.
		  */
		unsigned int def_in_;

		//! Deficit counter of the output link, in bytes.
		/*!
		  During bandwidth granting, we increment the deficit counter of
		  any encountered output link, up to a maximum value, so that
		  this output link will lag some bandwidth request.
		  */
		unsigned int def_out_;

		//! Create an empty descriptor.
		NeighDesc () {
			req_in_  = 0;
			req_out_ = 0;
			gnt_in_  = 0;
			gnt_out_ = 0;
			cnf_in_  = 0;
			cnf_out_ = 0;
			rcvCnf_  = false;
			backlog_ = 0;
			def_in_  = 0;
			def_out_ = 0;
		}
	};

	//! Array of neighbor descriptors for bandwidth requesting/granting.
	std::vector<NeighDesc> neigh_;

	//! Active-list of neighbor descriptors for bandwidth granting/requesting.
	CircularList<wimax::LinkId> activeList_;

	//! Active-list of neighbors descriptors for bandwidth regranting.
	CircularList<unsigned int> regntActiveList_;

	//! Neighbors's unavailabilities to transmit.
	/*!
	  This is a four-dimension bitmap.

	  The first dimension is the neighbor index.
	  The second dimension is the channel index.
	  The third dimension is the frame index (modulo HORIZON).
	  The four dimension is the slot index within a frame.

	  The last two dimensions are embedded into the Bitmap type,
	  which is a vector of bitsets.

	  This data structure is used when granting bandwidth to neighbors.

	  All the slots of the F-th frame of each neighbor for all channels
	  are reset by handle(), where F is the frame number (modulo HORIZON)
	  of the current data frame, at the end of each frame.
	  */
	std::vector< std::vector< Bitmap > > neigh_tx_unavl_;

	//! Combination of <channel, frame, slot> where this node cannot receive.
	/*!
	  Updated when a confirmation from a node which is not in this node's
	  neighborhood is received.

	  Used when granting bandwidth to neighbors.

	  All the slots of the F-th frame for all channels
	  are reset by handle(), where F is the frame number (modulo HORIZON)
	  of the current data frame, at the end of each frame.
	  */
	std::vector< Bitmap > self_rx_unavl_;

	//! Combination of <channel, frame, slot> where this node cannot transmit.
	/*!
	  Updated when a grant which is not addressed to this node is received.

	  Used when confirming bandwidth to neighbors.

	  All the slots of the F-th frame for all channels
	  are reset by handle(), where F is the frame number (modulo HORIZON)
	  of the current data frame, at the end of each frame.
	  */
	std::vector< Bitmap > self_tx_unavl_;

	//! Two-dimension bitmap representing this node unavailabilities.
	/*!
	  The bit busy_[F][S] is set (ie. true) if this node cannot
	  receive/transmit in the minislot S of frame F.
	  
	  This data structure is used when granting/confirming bandwidth.

	  The F-th entry of this data structure is reset by handle(), where
	  F is the frame number (modulo HORIZON) of the current data frame
	  at the end of each frame.
	  */
	Bitmap busy_;

	//! Two-dimension bitmap representing the slots unconfirmed by this node.
	/*!
	  Updated when a granted addressed to this node is received.
	  Used to confirm only minislots that have not been reserved
	  by other nodes. As soon as slot is confirmed, it is marked as
	  busy (ie. the corresponding bit in busy_ is set to true) so that
	  the same slot can never be confirmed twice.

	  The F-th entry of this data structure is reset by handle(), where
	  F is the frame number (modulo HORIZON) of the current data frame
	  at the end of each frame.
	  */
	Bitmap unconfirmedSlots_;

	//! List of unconfirmed grants directed to this node.
	std::list<WimshMshDsch::GntIE> unconfirmed_;

	//! List of pending availabilities to send out.
	std::list<WimshMshDsch::AvlIE> availabilities_;

   //! True if availabilities have to be advertised. Configured via Tcl.
   bool avlAdvertise_;

	//! Regrant horizon offset, in frames. Set via Tcl. Default = 1.
	unsigned int regrantOffset_;
	//! Regrant horizon duration, in handshake periods. Set via Tcl. Default = 1.
	unsigned int regrantDuration_;
	//! True if the same horizon for granting should be used. Default = no.
	bool sameRegrantHorizon_;

	//! True if unconfirmed bandwidth is regranted. Configured via Tcl.
	bool regrantEnabled_;

	//! True if we want to be fair while regranting bandwidth.
	bool fairRegrant_;

	//! True if we want to be fair while granting bandwidth.
	bool fairGrant_;

	//! True if we want to be fair while requesting bandwidth.
	bool fairRequest_;

	//! Maximum deficit, in bytes. Set via Tcl command. Zero means no maximum.
	unsigned int maxDeficit_;

	//! Maximum backlog, in bytes. Set via Tcl command. Zero means no maximum.
	unsigned int maxBacklog_;

	//! True if the last bandwidth/request grant terminated due to maxDeficit_.
	bool deficitOverflow_;

	//! Weight manager.
	WimshWeightManager wm_;

	//! Sum of the quanta, in bytes.
	/*!
	  Each quantum for bandwidth requesting/granting is computed as this
	  value times the weight of the input/output link.

	  This value is set via a Tcl command.
	  */
	unsigned int roundDuration_;

	//! Random number generator to pick up channel/frame/slot when granting.
	RNG grantFitRng;

	//! True if the starting channel is picked up randomly when granting.
	bool grantFitRandomChannel_;

	//! Deadlock detection timeout, in units of MSH-DSCH opportunities.
	/*!
	  Zero means disabled.
	  */
	unsigned int ddTimeout_;

	//! Number of consecutive rounds in which deficitOverflow_ is true.
	unsigned int ddTimer_;

	//! Minimum grant size, in OFDM symbols, preamble not included. Default = 1.
	unsigned int minGrant_;

public:
	//! Create an empty bandwidth manager.
	WimshBwManagerFairRR (WimshMac* m);
	//! Do nothing.
	~WimshBwManagerFairRR () { }

	//! Get an MSH-DSCH message from a neighbor, which is not deallocated.
	void recvMshDsch (WimshMshDsch* dsch);

	//! Schedule bandwidth into the following MSH-DSCH message.
	/*!
	  Some fieds have been already filled by the coordinator.
	  */
	void schedule (WimshMshDsch* dsch);

	//! Resize the internal data structures based on the number of neighbors.
	void initialize ();

	//! We have some new data to send out on an end-to-end flow.
	void backlog (WimaxNodeId src, WimaxNodeId dst, unsigned char prio,
			WimaxNodeId nexthop, unsigned int bytes);

	//! We have some new data to send out on a link.
	void backlog (WimaxNodeId nexthop, unsigned int bytes);

	//! We sent out some data on a link (i.e. negative backlog).
	void sent (WimaxNodeId nexthop, unsigned int bytes);

	//! We received some new data addressed to this node.
	void received (WimaxNodeId src, WimaxNodeId dst, unsigned char prio,
			WimaxNodeId source, unsigned int bytes) {
		wm_.flow (src, dst, prio, source, wimax::IN); }

	//! Tcl interface from the MAC layer.
	/*!
	  Tcl commands:
	  - $mac bwmanager availabilities [on|off]\n
	    Turn on/off availabilities advertising.
     - $mac bwmanager regrant [on|off]\n
	    Turn on/off the regranting procedure.
     - $mac bwmanager regrant-horizon x\n
	    Set the regrant-horizon to x frames.
     - $mac bwmanager dd-timeout x\n
	    Set the deadlock detection timeout to x MSH-DSCH opportunities.
     - $mac bwmanager round-duration x\n
	    Set the FEBA target round duration to x bytes.
     - $mac bwmanager max-deficit x\n
	    Set the maximum deficit to x bytes. Zero means no maximum.
     - $mac bwmanager min-grant x\n
	    Set the minimum grant to x slots. Zero means no maximum.
     - $mac bwmanager max-backlog x\n
	    Set the maximum backlog to x bytes. Zero means no maximum.
     - $mac bwmanager fairness [grant|regrant|request|no]\n
	    Activate the fairness procedure when granting, or regranting,
		 or requesting. When 'no' is specified, all of them are turned off.
     - $mac bwmanager grant-fit channel [random|first]\n
	    Choose the algorithm to fit the grant into the forthcoming frames.
		 */
	int command (int argc, const char*const* argv);

protected:
	//! Invalidate the data structures' entries for the current frame.
	void invalidate (unsigned int F);

private:
	//! Decode grants/confirmations from an incoming MSH-DSCH message.
	/*!
	  There the following cases:

	  - for each grant addressed to this node, a confirmation is added
	    to the pending list of confirmations (managed by confirm()),
		 the granted minislots are marked as unconfirmed unavailable
		 (in the unconfirmedSlots_ bitmap) and the amount of bandwidth
		 granted by a neighbor is updated

	  - for each grant not addressed to this node, an unavailability is
	    added to the pending list of availabilites (managed by availabilities())

	  - for each confirmation addressed to this node, update the cnf_in_
	    data structure

     - for each confirmation addressed to a node which is not in this node's
	    first-hop neighborhood, mark the confirmed minislots as unavailable
		 for reception from this node
	  */
	void rcvGrants (WimshMshDsch* dsch);
	//! Decode availabilities from an incoming MSH-DSCH message.
	/*!
	  We update the status of neigh_tx_unavail_ based on the received
	  availabilities.
	  */
	void rcvAvailabilities (WimshMshDsch* dsch);
	//! Decode requests from an incoming MSH-DSCH message.
	/*!
	  For each request addressed to this node, add the number of minislots
	  requested to the req_in_ data structure.
	  */
	void rcvRequests (WimshMshDsch* dsch);

   //! Confirm pending grants.
	/*!
	  For each confirmation in the unconfirmed list, we:

	  - try to send as many confirmation as possible, provided that
	    the slots that have been granted are still available for
		 transmission by this node (via the self_tx_unaval_ bitmap)
	  - update the status of the cnf_out_ data structure
	  - set the minislots reserved for transmission at this node, which
	    will be used by the handle() function to trigger the packet
		 scheduler at the MAC layer. Both self_tx_unavl_ and self_tx_unavl_
		 are updated

     Note that the cnf_out_ data structure is updated with the number
	  of minislots actually confirmed which will be used for transmission.
	  */
   void confirm (WimshMshDsch* dsch);
   //! Advertise pending availabilities.
   void availabilities (WimshMshDsch* dsch);
   //! Request/grant bandwidth.
	/*!
	  :TODO: more documentation (come on, this is a critical function!)

	  Let H be the average number of frames between two consecutive
	  transmission opportunities of this node, and H' the same measure
	  for the node to which we are currently granting bandwidth.
	  The time window over which we grant bandwidth is NOW + [H',H + 2H'].

	  The granted minislots are marked as unavailable for reception.
	  The amount of granted minislots are udpated.
	  */
   void requestGrant (WimshMshDsch* dsch);

	//! Regrant as much as possible unconfirmed bandwidth.
	/*!
	  Bandwidth is regranted on a round-robin fashion. If it is not possible
	  to regrant bandwidth to a neighbor, then the behavior depends on the
	  someFairness_ flag. If true, regranting stops immediately, and
	  next regranting will begin from the current neighbor. On the other
	  hand, if it is false, the neighbor is skipped and regranting continues
	  over the remaining neighbors until either none of them can be
	  served or there is not any more spare room in the MSH-DSCH message.

	  In any case, the gnt_in_ counter for this neighbor is not updated.
	  In other words, the latter counts the bytes that have been granted
	  the first time only.
	  */
	void regrant (WimshMshDsch* dsch);

	//! Return the real number of frames for which the persistence is relevant.
	/*!
	  Since the frame number never wraps around, but the frame index into
	  the data structures does, if a stale message is received, we have to
	  update only the frames that are actually relevant. In other words, we
	  do not want to update frames in the past, because we would overwrite
	  information that will be used in the future (in a circular manner).
	  */
	void realPersistence (unsigned int start, WimshMshDsch::Persistence pers,
			unsigned int& realStart, unsigned int& range);

	//! First-fit to grant bandwidth to a neighbor.
	/*!
	  The first fit is searched starting from the first minislot of
	  the first frame in the first available channel.
	  Persistence is always 1-frame.

	  A slot in a frame on a channel is eligible to be granted to
	  the requester if the corresponding entry in the following
	  data structures are false: busy_, neigh_tx_unavl_, unconfirmed_.
	  
	  If it is not possible to schedule bandiwdth to ndx in the
	  specified time window, then a grant with an empty minislot
	  range is returned.
	  */
	WimshMshDsch::GntIE grantFit (unsigned int ndx, unsigned int bytes,
			unsigned int minFrame, unsigned int maxFrame, grantFitDesc& status);

	//! First-fit to confirm bandwidth (similar to grantFit).
	/*!
	  A slot in a frame on a channel is eligible to be confirmed
	  if the corresponding entry of busy_ is false.
	  */
	void confFit ( unsigned int fstart, unsigned int frange,
			unsigned int mstart, unsigned int mrange,
			WimshMshDsch::GntIE& gnt);

	//! Get the interval between two consecutive control opportunities in frames.
	unsigned int handshake (WimaxNodeId x) {
		return (unsigned int) ceil (
				  (fabs ( mac_->h (x)  - mac_->phyMib()->controlDuration() ))
				/ mac_->phyMib()->frameDuration()); }

	//! Return the quantum value of a given input/output link, in bytes.
	unsigned int quantum (unsigned int ndx, wimax::LinkDirection dir) {
		return (unsigned int) (ceil(wm_.weight (ndx, dir) * roundDuration_)); }

	//! Debug function. Print out  RR data structures.
	void printDataStructures (FILE* os);
};

#endif // __NS2_WIMSH_BW_MANAGER_FRR_H
