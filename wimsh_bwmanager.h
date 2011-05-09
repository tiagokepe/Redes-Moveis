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

#ifndef __NS2_WIMSH_BW_MANAGER_H
#define __NS2_WIMSH_BW_MANAGER_H

#include <wimax_common.h>
#include <t_timers.h>

#include <bitset>
#include <vector>

#include <math.h>

class WimshMac;
class WimshMshDsch;

/*
 *
 * class WimshBwManager
 *
 */

//! Abstract class for a bandwidth manager in 802.16. Single-radio only.
class WimshBwManager {
protected:
	//! Maximum time horizon (in terms of frames) for granting bandwidth.
	enum { HORIZON = 128 };

	//! Maximum number of minislots per frame.
	enum { MAX_SLOTS = 256 };

	//! Pointer to the MAC layer.
	WimshMac* mac_;

	//! Timer to schedule bandwidth manager events.
	TTimer<WimshBwManager> timer_;

   //! Array of bit maps representing granted minislots within frames.
   /*!
     When the minislot is not granted for transmission, the node
	  moves to receive mode. The channel used for both reception and
	  transmission is specified in the channel_ array.

	  Note that there is the following (bad) trick. The elements in
	  the channel_ array are initialized to 'zero' and those in the
	  grants_ array to 'receive'. Therefore, whenever a node does not
	  send a confirmation (ie. it does not use that slot for transmission)
	  and does not receive a confirmation addressed to itself (ie. it does
	  not receive from a neighbor in that slot) then the slot is
	  automatically used for reception from the 'zero' channel, which is
	  the control channel. This allows for uncoordinated scheduling to
	  be implemented without any modification, *but* only works provided
	  that the control channel used by the MAC is always 'zero'.

     The F-th entry of this data structure is reset by handle(), where
     F is the frame number (modulo HORIZON) of the current data frame
     at the end of each frame.
     */
	std::vector< std::bitset<MAX_SLOTS> > grants_;
	//! Array of vectors representing the grants' destinations.
	std::vector< std::vector<WimaxNodeId> > dst_;
	//! Array of vectors representing the channel identifiers.
	std::vector< std::vector<unsigned int> > channel_;

	//! Next slot to be served.
	unsigned int lastSlot_;
	
public:
	//! Create an empty bandwidth manager.
	WimshBwManager (WimshMac* m);
	//! Do nothing.
	virtual ~WimshBwManager () { }

	//! Get an MSH-DSCH message from a neighbor, which is not deallocated.
	virtual void recvMshDsch (WimshMshDsch* dsch) = 0;

	//! Schedule bandwidth into the following MSH-DSCH message.
	/*!
	  Some fieds have been already filled by the coordinator.
	  */
	virtual void schedule (WimshMshDsch* dsch) = 0;

	//! Timer handler.
	virtual void handle ();

	//! Initialize the internal data structures. Called by the MAC layer.
	virtual void initialize () = 0;

	//! Indicate some additional backlog of an end-to-end flow.
	virtual void backlog (WimaxNodeId src, WimaxNodeId dst, unsigned char prio,
			WimaxNodeId nexthop, unsigned int bytes ) = 0;

	//! Indicate some additional on a link towards a neighbor.
	virtual void backlog (WimaxNodeId nexthop, unsigned int bytes ) = 0;

	//! Indicate some data was received.
	virtual void received (WimaxNodeId src, WimaxNodeId dst, unsigned char prio,
			WimaxNodeId source, unsigned int bytes) = 0;

	//! We sent out some data on a link (i.e. negative backlog).
	virtual void sent (WimaxNodeId nexthop, unsigned int bytes) = 0;

	//! Tcl interface from the MAC layer.
	virtual int command (int argc, const char*const* argv) = 0;

protected:
	//! Invalidate any data structure of frame F (modulo HORIZON).
	/*!
	  This function must be called by any derived class after invalidating
	  its own data structures.
	  */
	virtual void invalidate (unsigned int F);

	//! Utility function to set a range of elements to a given value.
	template<typename T, typename V>
	void setSlots ( T& map,
			unsigned int fstart, unsigned int frange,
			unsigned int mstart, unsigned int mrange, const V& value);
};

template<typename T, typename V>
void
WimshBwManager::setSlots (
      T& map,
      unsigned int fstart, unsigned int frange,
      unsigned int mstart, unsigned int mrange, const V& value)
{
   // for each frame
   for ( unsigned int f = 0 ; f < frange ; f++ ) {
      unsigned int F = ( fstart + f ) % HORIZON;

      // for each minislot
      for ( unsigned int s = 0 ; s < mrange ; s++ ) {
         unsigned int S = mstart + s;

         // mark the minislot
         map[F][S] = value;
      }
   }
}

/*
 *
 * class WimshBwManagerDummy
 *
 */

//! Dummy bandwidth manager in 802.16 (debug/test only). Single channel only.
class WimshBwManagerDummy : public WimshBwManager {
	//! Descriptor for a static minislot allocation.
	struct AllocationDesc {
		//! Destination node.
		WimaxNodeId node_;
		//! Minislot start.
		unsigned char start_;
		//! Minislot range size.
		unsigned char range_;
		//! Create an allocation descriptor.
		AllocationDesc (WimaxNodeId node,
				unsigned char start,
				unsigned char range) {
			node_ = node; start_ = start; range_ = range; }
	};

	//! Array of static allocations.
	std::vector<AllocationDesc> alloc_;

public:
	//! Create an empty bandwidth manager.
	WimshBwManagerDummy (WimshMac* m);
	//! Do nothing.
	~WimshBwManagerDummy () { }

	//! Get an MSH-DSCH message from a neighbor, which is not deallocated.
	void recvMshDsch (WimshMshDsch* dsch);

	//! Schedule bandwidth into the following MSH-DSCH message.
	/*!
	  Some fieds have been already filled by the coordinator.
	  */
	void schedule (WimshMshDsch* dsch);

	//! Do nothing.
	void initialize () { }

	//! Do nothing.
	void backlog (WimaxNodeId, WimaxNodeId, unsigned char,
			WimaxNodeId, unsigned int) { }
	//! Do nothing.
	void backlog (WimaxNodeId nexthop, unsigned int bytes ) { }
	//! Do nothing.
	void received (WimaxNodeId src, WimaxNodeId dst, unsigned char,
			WimaxNodeId source, unsigned int bytes) { }
	//! Do nothing.
	void sent (WimaxNodeId nexthop, unsigned int bytes) { }

	//! Tcl interface from the MAC layer.
	int command (int argc, const char*const* argv);

protected:
	//! Invalidate any data structure of frame F (modulo HORIZON).
	void invalidate (unsigned int F) { WimshBwManager::invalidate(F); }

private:
	//! Set the fixed start and range for confirmation messages.
	void setRange (WimaxNodeId node, unsigned char start, unsigned char range) {
		alloc_.push_back (AllocationDesc (node, start, range)); }
};

#endif // __NS2_WIMSH_BW_MANAGER_H
