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

#include <wimsh_coordinator.h>
#include <t_timers.h>
#include <wimsh_packet.h>
#include <wimsh_topology.h>
#include <wimsh_mac.h>

#include <math.h>

/*
 *
 * class WimshCoordinator
 *
 */

WimshCoordinator::WimshCoordinator (WimshMac* m) : mac_(m), timer_ (this)
{
	nextDschSlot_  = 0;
	nextDschFrame_ = NEVER;
	nextNcfgSlot_  = 0;
	nextNcfgFrame_ = NEVER;
	nextNentSlot_  = 0;      // never changed
	nextNentFrame_ = NEVER;
}


void
WimshCoordinator::handle ()
{
	// get a pointer to the PHY MIB
	WimshPhyMib* phyMib = mac_->phyMib();

	// number of control slots per frame
	const unsigned int C = phyMib->controlSlots();

	// compute the current slot
	const unsigned int curslot = (unsigned int) round (
			( NOW  - mac_->frame() * phyMib->frameDuration() ) /
			phyMib->controlSlotDuration() );

	if ( WimaxDebug::trace ("WCRD::handle") ) fprintf (stderr,
			"%.9f WCRD::handle     [%d] "
			"(slot, frame) current (%d, %d) nextDsch (%d, %d) nextNcfg (%d, %d) "
			"nextNent (%d, %d)\n",
			NOW, mac_->nodeId(),
			curslot, mac_->frame(), nextDschSlot_, nextDschFrame_,
			nextNcfgSlot_, nextNcfgFrame_, nextNentSlot_, nextNentFrame_);
	
	// this same handler is used for different even types
	// thus, we have to check what happened that triggered this event
	// 1. it is time to transmit an MSH-DSCH message
	// 2. it is time to transmit an MSH-NCFG message
	// 3. it is time to transmit an MSH-NENT message
	// 4. the control frame ended
	// 5. any other event

	// 1. it is time to transmit an MSH-DSCH message
	if ( curslot == nextDschSlot_ && mac_->frame() == nextDschFrame_ ) {

		// call the election procedure to set nextDschFrame_ and nextDschSlot_
		electionDsch ();

		// set the timer to expire at the end of this opportunity to
		// restore the RX status on the control channel
		timer_.start (phyMib->controlSlotDuration());

		// create a new (empty) MSH-DSCH message
		WimshMshDsch* dsch = new WimshMshDsch;

		// fill the IEs about myself and my neighborhood
		fillSelf (dsch);
		fillNeighbors (dsch);

		// pass it to the MAC layer
		mac_->opportunity (dsch);

	// 2. it is time to transmit an MSH-NCFG message
	} else if ( curslot == nextNcfgSlot_ && mac_->frame() == nextNcfgFrame_ ) {

		// call the election procedure to set nextNcfgFrame_ and nextNcfgSlot_
		electionNcfg ();

		// set the timer to expire at the end of this opportunity to
		// restore the RX status on the control channel
		timer_.start (phyMib->controlSlotDuration());

		// create a new (empty) MSH-NCFG message
		WimshMshNcfg* ncfg = new WimshMshNcfg;

		// fill the IEs about myself and my neighborhood
		fillSelf (ncfg);
		fillNeighbors (ncfg);

		// pass it to the MAC layer
		mac_->opportunity (ncfg);

	// 3. it is time to transmit an MSH-NENT message
	} else if ( curslot == nextNentSlot_ && mac_->frame() == nextNentFrame_ ) {

		// call the election procedure to set nextNentFrame_ and nextNcfgSlot_
		electionNent ();

		// set the timer to expire at the end of this opportunity to
		// restore the RX status on the control channel
		timer_.start (phyMib->controlSlotDuration());

		// create a new (empty) MSH-NENT message
		WimshMshNent* nent = new WimshMshNent;

		// pass it to the MAC layer
		mac_->opportunity (nent);

	// 4. the control frame ended
	} else  if ( curslot == C ) {                                        
		// set the timer to expire at the beginning of the next frame
		timer_.start (
				  phyMib->frameDuration()
				- C * phyMib->controlSlotDuration());

		// in any case, tell the MAC to listen for control messages
		mac_->setControlChannel (wimax::RX);

	// 5. otherwise set the next handle to the earliest event among:
	//    + the next MSH-DSCH opportunity
	//    + the next MSH-NCFG opportunity
	//    + the next MSH-NENT opportunity
	//    + the end of the control sub-frame
	} else {
		// if the next opportunity is within this frame, set the
		// timer to expire at that slot

		int slot = -1;
		if ( nextDschFrame_ == mac_->frame() ) {
			slot = nextDschSlot_;
		} else if ( nextNentFrame_ == mac_->frame() ) {
			slot = nextNentSlot_;
		} else if ( nextNcfgFrame_ == mac_->frame() ) {
			slot = nextNcfgSlot_;
		}

		// set the timer to expire at the end of this control frame
		if ( slot >= 0 ) slot -= curslot;
		else slot = C - curslot;

		// start the timer
		timer_.start ( slot * phyMib->controlSlotDuration() );

		// in any case, tell the MAC to listen for control messages
		mac_->setControlChannel (wimax::RX);
	}
}

/*
 *
 * class WimshCoordinatorDummy
 *
 */

WimshCoordinatorDummy::WimshCoordinatorDummy (WimshMac* m, WimshPhyMib* p) :
	WimshCoordinator (m)
{
	phyMib_ = p;
	slot_ = 0;
	frame_ = 1;
	period_ = 1;
	periodNumber_ = 0;
	firstFrame_ = 0;
	type_ = FIXED;
}

int
WimshCoordinatorDummy::command (int argc, const char*const* argv)
{
	if ( argc == 2 && strcmp (argv[0], "mode") == 0 ) {
		if ( strcmp (argv[1], "fixed") == 0 ) {
			type_ = FIXED;
		} else if ( strcmp (argv[1], "moving") == 0 ) {
			type_ = MOVING;
		} else {
			fprintf (stderr, "invalid dummy coordinator type '%s'. "
					"Choose 'fixed' or 'moving'.\n", argv[1]);
			return TCL_ERROR;
		}
		start ();
		return TCL_OK;
	} else if ( argc == 2 && strcmp (argv[0], "slot") == 0 ) {
		slot_ = (unsigned int) atoi (argv[1]);
		return TCL_OK;
	} else if ( argc == 2 && strcmp (argv[0], "frame") == 0 ) {
		frame_ = (unsigned int) atoi (argv[1]);
		return TCL_OK;
	} else if ( argc == 2 && strcmp (argv[0], "first") == 0 ) {
		firstFrame_ = (unsigned int) atoi (argv[1]);
		return TCL_OK;
	} else if ( argc == 2 && strcmp (argv[0], "period") == 0 ) {
		period_ = (unsigned int) atoi (argv[1]);
		return TCL_OK;
	}
	return TCL_ERROR;
}

void
WimshCoordinatorDummy::start ()
{
	// compute the first transmission opportunity
	if ( type_ == FIXED ) {
		nextDschSlot_ = slot_;
		nextDschFrame_ = firstFrame_;
	} else  { // type == MOVING
		nextDschSlot_ = slot_ % phyMib_->controlSlots();
		nextDschFrame_ = slot_ / phyMib_->controlSlots();
	}

	timer_.start (
			  nextDschSlot_ * phyMib_->controlSlotDuration()
			+ nextDschFrame_ * phyMib_->frameDuration() );
}

void
WimshCoordinatorDummy::fillSelf (WimshMshDsch* dsch)
{
	// fill the MAC header and meshsubheader fields
	dsch->src() = mac_->nodeId();
	dsch->hdr().crc() = true;
	// dsch->hdr().fragmentation() is false by default
	// dsch->hdr().length() is automatically computed
	// dsch->hdr().meshCid() is set to management, no ARQ, by default

	// fill the neighbor information about myself
	dsch->myself().nodeId_ = mac_->nodeId();
	dsch->myself().nextXmtMx_ = 0;
	dsch->myself().xmtHoldoffExponent_ = 0;
}

void
WimshCoordinatorDummy::fillNeighbors (WimshMshDsch* dsch)
{
	// get the list of neighbors of this node
	std::vector<WimaxNodeId> neigh;
	mac_->topology()->neighbors (mac_->nodeId(), neigh);

	std::vector<WimaxNodeId>::iterator it;
	for ( it = neigh.begin() ; it != neigh.end() ; ++it ) {

		// check whether there is enough room to add another IE
		if ( dsch->remaining() < WimshMshDsch::NghIE::size() ) break;

		WimshMshDsch::NghIE ie; // new information element

		// fill the IE with junk information
		ie.nodeId_ = *it;
		ie.nextXmtMx_ = 0;
		ie.xmtHoldoffExponent_ = 0;

		// add the IE to those in MSH-DSCH
		dsch->add (ie);
	}
}

void
WimshCoordinatorDummy::election ()
{
	// number of control slots per frame
	const unsigned int C = phyMib_->controlSlots();

	// compute the next opportunity
	if ( type_ == FIXED ) {
		nextDschSlot_ = slot_;
		nextDschFrame_ = mac_->frame () + frame_;

	} else {  // type_ == MOVING
		// update the period number
		++periodNumber_;

		// set the slot within the period
		slot_ = ( slot_ + 1 ) % period_;

		// absolute number of control slot since the system start
		unsigned int abs = periodNumber_ * period_ + slot_;

		// set the frame
		nextDschSlot_ = abs % C;
		nextDschFrame_ = abs / C;
	}
}
