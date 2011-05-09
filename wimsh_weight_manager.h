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

#ifndef __NS2_WIMSH_WEIGHT_MANAGER_H
#define __NS2_WIMSH_WEIGHT_MANAGER_H

#include <wimsh_packet.h>
#include <wimsh_mac.h>

#include <vector>

//! Object which computes/stores he weight of end-to-end flows.
/*!
  We define an end-to-end flow as a stream of IP datagrams from a
  source to a destination.
  For instance, FTP flows (even though uni-directional in ns2) consist
  of two flows each.
  On the other hand, many application sending data to the same
  destination node (i.e. final destination, not to the same next-hop
  neighbor of the source node) are treated as a single flow.
  */
class WimshWeightManager {
	typedef std::list<unsigned int> NdxList;

	//! Flow descriptor.
	/*!
	  This descriptor is for:
	  - any outgoing flows, for which we always know both source and destination
	  - an incoming flow, provided that we received some packets
	  */
	struct FlowDesc {
		//! Source NodeID (i.e. the flow source).
		WimaxNodeId src_;
		//! Destination NodeID (i.e. the flow destination).
		WimaxNodeId dst_;
		//! Flow priority, as specified into the Mesh CID.
		unsigned char prio_;
		//! List of neighbors to which we send (from which we receive) data.
		/*!
		  With single-path routing the list holds at most one element.
		  
		  Also, incomplete elements only have one element.
		  */
		NdxList ndx_;
		//! Size of the list of neighbors.
		unsigned int ndxSize_;
		//! True if the information is not complete.
		/*!
		  When a bandwidth request is received, then the node does not know
		  the destination/source of the packets that will be conveyed.
		  However, if we assigned a weight to nodes that have already
		  sent packets, then we would end up never assigning bandwidth
		  to new flows (i.e. deadlock at startup).

		  However, we mark this descriptor as "incomplete", so that we
		  will remove it as soon as the first packet from the sending
		  node is received.
		  */
		bool incomplete_;
		//! Last time a packet has been received, used to invalidate flows.
		/*!
		  This field is not meaningful for incomplete flow descriptors.
		  */
		double lastRcvd_;

		//! Return true if same source, destination and priority.
		bool operator== (const FlowDesc& desc) {
			return ( src_ == desc.src_ && dst_ == desc.dst_ &&
					prio_ == desc.prio_ ) ? true : false; }
		
		//! Build an incomplete flow descriptor by default.
		/*!
		  Fields that are not meaningful for an incomplete flow descriptor
		  are not set.
		  */
		FlowDesc (unsigned int ndx = 0) {
			prio_ = 0; ndxSize_ = 1; ndx_.push_back(ndx); incomplete_ = true; }
	};

	typedef std::list<FlowDesc> DescList;

	//! List of flow descriptors for incoming flows.
	DescList in_;
	//! List of flow descriptors for outgoing flows.
	DescList out_;

	//! Factors to be used to compute weights according to priorities.
	/*!
	  Default values are all equal to 1.
	  */
	double prioWeights_[WimaxMeshCid::MAX_PRIO];

	//! Sum of the number of elements in the flow descriptors' lists.
	unsigned int nFlowDesc_;

	//! Array of weights (input links).
	std::vector<double> weightIn_;
	//! Array of weights (output links).
	std::vector<double> weightOut_;

	//! Pointer to the MAC layer.
	WimshMac* mac_;

	//! Timer to clean up stale weights.
	TTimer<WimshWeightManager> timer_;
	//! Stale weights cleaning interval.
	/*!
	  Set via the interval() function. If this function is not called, or
	  if it sets the interval to zero, then stale weights are never
	  removed.

	  If the timer has not been started, then subsequent calls to the
	  interval() to change the interval value will have no effect.
	  Furthermore, if the timer is stopped (i.e. by setting a value
	  smaller than or equal to zero, then it cannot be restarted).
	  */
	double interval_;

	//! True if weights are normalized based on the number of end-to-end flows.
	bool normalizeFlow_;

public:
	//! Build an empty weight manager bound to a given MAC layer.
	WimshWeightManager (WimshMac* m);
	//! Do nothing.
	virtual ~WimshWeightManager () { }

	//! Tcl interface via the bandwidth manager.
	int command (int argc, const char*const* argv);

	//! Initialize internal data structures.
	void initialize ();

	//! Get/set the stale detection interval.
	double& interval () { return interval_; }

	//! Timer handler.
	void handle ();

	//! Get the weight of a link (identified via the neighbor index).
	double weight (unsigned int ndx, wimax::LinkDirection dir) {
		return ( dir == wimax::IN ) ? weightIn_[ndx] : weightOut_[ndx]; }

	//! Set a, perhaphs new, flow as active.
	/*!
	  Adding a flow descriptor also removes the corresponding incomplete
	  flow descriptor into the list, if any.

	  If a new element is added to the list of descriptors, then
	  weights are recomputed.
	  */
	void flow (WimaxNodeId src, WimaxNodeId dst, unsigned char prio,
			unsigned int ndx, wimax::LinkDirection dir);
	//! Set a, perhaphs new, incomplete flow as active.
	/*!
	  If an element, either complete or incomplete, associated to the same
	  link already exists into the list, then the incomplete flow is not added.

	  If a new element is added to the list of descriptors, then
	  weights are recomputed.
	  */
	void flow (unsigned int ndx, wimax::LinkDirection dir);

	//! Recompute weights.
	void recompute ();

	//! Get/set the normalizeFlow_ weight.
	bool& normalizeFlow () { return normalizeFlow_; }

	//! Get/set the i-th priority weight.
	double& prioWeight (unsigned int i) {
		if ( i >= WimaxMeshCid::MAX_PRIO ) abort();
		return prioWeights_[i]; }
};

#endif // __NS2_WIMSH_WEIGHT_MANAGER_H
