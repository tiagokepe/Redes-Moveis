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

#ifndef __NS2_WIMSH_SCHEDULER_FRR_H
#define __NS2_WIMSH_SCHEDULER_FRR_H

#include <wimsh_scheduler.h>

//! FairRR packet scheduler.
/*!
  :TODO: documentation
  */
class WimshSchedulerFairRR : public WimshScheduler {

public:
	//! Buffer sharing strategy.
	enum BufferSharingMode { SHARED, PER_LINK, PER_FLOW };

protected:
	//! Flow descriptor.
	/*!
	  Used for packet scheduling. Contains the packet queue and related
	  information used to identify a traffic flow (source and destination
	  NodeID's) and schedule (deficit counter).

	  The quantum is computed as the roundDuration_ weighted on the
	  number of traffic flows in the round-robin list of this link.
	  */
	struct FlowDesc {
		//! Source NodeID.
		WimaxNodeId src_;
		//! Destination NodeID.
		WimaxNodeId dst_;
		//! Priority.
		unsigned char prio_;

		//! Last time a packet was received.
		double last_;

		//! Quantum value, in bytes. Set by the recompute() function.
		unsigned int quantum_;
		//! Deficit counter, in bytes.
		unsigned int deficit_;

		//! Buffer occupacy, in bytes.
		unsigned int size_;

		//! Packet queue to go to the destination dst_.
		std::queue<WimaxPdu*> queue_;

		//! Build an empty flow descriptor.
		FlowDesc (WimaxNodeId src=0, WimaxNodeId dst=0, unsigned char prio=0) :
			src_ (src), dst_ (dst), prio_(prio),
			last_(0), quantum_(0), deficit_ (0), size_ (0) { }

		//! Returns true if same source, destination, and priority.
		bool operator== (const FlowDesc& x) const {
			return ( dst_ == x.dst_ && src_ == x.src_ && prio_ == x.prio_ ); }

		//! Returns false if two descriptors do not have the same src and dst.
		bool operator!= (const FlowDesc& x) const {
			return ! ( *this == x ); }
	};

	//! Link descriptor.
	/*!
	  Contains the packet queues, one for each end-to-end destination
	  and related data structures.
	  */
	struct LinkDesc {
		//! Buffer occupancy of this link, in bytes.
		unsigned int size_;

		//! Round robin list of flow descriptors, ie. packet queues.
		CircularList<FlowDesc> rr_;

		//! Build an empty link descriptor.
		LinkDesc () { size_ = 0; }
	};

	//! Factors to be used to compute weights according to priorities.
	/*!
		 Default values are all equal to 1.
	 */
	double prioWeights_[WimaxMeshCid::MAX_PRIO];
	//! Interval after which a flow is considered to be inactive.
	double interval_;
	//! Timer to remove stale flows.
	TTimer<WimshSchedulerFairRR> timer_;

	//! Round-robin duration, in bytes. Set via Tcl.
	unsigned int roundDuration_;

	//! Array of link descriptors, one for each neighbor. Initialized by the MAC.
	std::vector<LinkDesc> link_;

	//! True if there is a pending DRR round.
	/*!
	  If a flow cannot make up its deficit because there is no more spare
	  room into the burst of PDUs under service, not due to the packet size
	  being larger than its deficit, then the DRR service of this flow
	  is not complete. In this case, unfinishedRound_ is true, and the
	  current flow is pointed by the current round-robin pointer in
	  the active list.
	  */
	std::vector<bool> unfinishedRound_;

	//! Buffer sharing mode. Default is SHARED.
	BufferSharingMode bufferSharingMode_;

public:
	//! Create an  empty scheduler.
	WimshSchedulerFairRR (WimshMac* m);
	//! Do nothing.
	~WimshSchedulerFairRR () { }

	//! Resize the number of queues based on the number of neighbors.
	void initialize ();

	//! Remove stale flows.
	void handle ();

	//! Add a MAC PDU to this object.
	void addPdu (WimaxPdu* pdu);

	//! Schedule a new data burst to a neighbor.
	void schedule (WimshFragmentationBuffer& frag, WimaxNodeId dst);

	//! Return the size, in bytes, of the queue to a neighbor (by index).
	unsigned int neighbor (unsigned ndx) { return link_[ndx].size_; }
	
	//! Tcl interface via MAC.
	int command (int argc, const char*const* argv);

private:
	//! Drop a PDU (by deallocating PDU/SDU/IP).
	void drop (WimaxPdu* pdu);

	//! Serve a flow until its deficit or backlog are exhausted.
	bool serve (WimshFragmentationBuffer& frag,
			unsigned int ndx, bool unfinished);

	//! Recompute the quanta values of a given list of flow descriptors.
	void recompute (CircularList<FlowDesc>& rr);
};

#endif // __NS2_WIMSH_SCHEDULER_FRR_H
