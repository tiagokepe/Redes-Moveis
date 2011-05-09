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

#ifndef __NS2_WIMSH_COORDINATOR_H
#define __NS2_WIMSH_COORDINATOR_H

#include <wimax_common.h>
#include <t_timers.h>
#include <wimax_defs.h>

class WimshMac;
class WimshMshDsch;
class WimshMshNcfg;
class WimshMshNent;
class WimshPhyMib;

/*
 *
 * class WimshCoordinator
 *
 */

//! Abstract class for a distributed election coordinator.
/*!
  Note that slots are number from 0 to (MSH-CTRL-LEN - 1).
  */
class WimshCoordinator {
	//! This is used to set an opportunity that never expires.
	enum { NEVER = 0xFFFFFFFF };
protected:
	//! Pointer to the MAC object.
	WimshMac* mac_;

	//! Expire each new transmission opportunity.
	TTimer<WimshCoordinator> timer_;

	//! Slot of the next opportunity to send an MSH-DSCH message.
	unsigned int nextDschSlot_;
	//! Frame number of the next opportunity to send an MSH-DSCH message.
	unsigned int nextDschFrame_;

	//! Slot of the next opportunity to send an MSH-NCFG message.
	unsigned int nextNcfgSlot_;
	//! Frame number of the next opportunity to send an MSH-NCFG message.
	unsigned int nextNcfgFrame_;

	//! Slot of the next opportunity to send an MSH-NENT message.
	/*!
	 * According to the IEEE 802.16d standard, this is always set to 0.
	 */
	unsigned int nextNentSlot_;
	//! Frame number of the next opportunity to send an MSH-NENT message.
	unsigned int nextNentFrame_;

public:
	//! Create the coordinator.
	WimshCoordinator (WimshMac* m);
	//! Do nothing.
	virtual ~WimshCoordinator () { }

	//! Get an MSH-DSCH message from the MAC. The message is not freed.
	virtual void recvMshDsch (WimshMshDsch* dsch, double txtime = 0) = 0;
	virtual void recvMshNcfg (WimshMshNcfg* ncfg, double txtime = 0) { }

	//! Timer handler.
	/*!
	  This handler is called in any of the following events:
	  - frame start time
	  - end of the control subframe
	  - beginning of the transmission opportunity
	  - end of the transmission opportunity
	  */
	virtual void handle ();

	//! Initialiaze the internal data structure.
	virtual void initialize() = 0;

	//! Tcl interface.
	virtual int command (int argc, const char*const* argv) = 0;

protected:
	//! Election procedure called by handle().
	virtual void electionDsch () = 0;
	//! Election procedure called by handle().
	virtual void electionNcfg () = 0;
	//! Election procedure called by handle().
	virtual void electionNent () = 0;

	//! Fill the IE about myself in an MSH-DSCH message.
	virtual void fillSelf (WimshMshDsch* dsch) = 0;
	//! Fill the IEs about my neighbors in an MSH-DSCH message.
	virtual void fillNeighbors (WimshMshDsch* dsch) = 0;

	//! Fill the IE about myself in an MSH-NCFG message.
	virtual void fillSelf (WimshMshNcfg* ncfg) = 0;
	//! Fill the IEs about my neighbors in an MSH-NCFG message.
	virtual void fillNeighbors (WimshMshNcfg* ncfg) = 0;
};

/*
 *
 * class WimshCoordinatorDummy
 *
 */

//! Dummy coordinator, which ignores the MSH-DSCH messages (debug/test only).
/*!
  It can be used in two modes: fixed and moving.

  In 'fixed' mode, each opportunity is scheduled at the i-th opportunity
  (where i = slot_) of frame f = frame_ * k, k = 0,1,...

  In 'moving' mode, there is a period of period_ control slots, not
  necessarily aligned with the frame boundaries. The first opportunity
  is set to the value of slot_. The subsequent opportunities are
  k + slot_ modulo period_.
  */
class WimshCoordinatorDummy : public WimshCoordinator {
	//! Coordination type (fixed or moving).
	enum { FIXED = 0, MOVING } type_;

	//! Fixed control slot used on each frame.
	unsigned int slot_;
	//! Fixed frame interval between two consecutive opportunities.
	unsigned int frame_;
	//! First frame in which to send an MSH-DSCH, used with 'fixed' coordination.
	unsigned int firstFrame_;

	//! Shifting period, used with 'moving' coordination.
	unsigned int period_;
	//! Period numbed, used with 'moving' coordination.
	unsigned int periodNumber_;

	//! PHY MIB.
	WimshPhyMib* phyMib_;
public:
	//! Create the coordinator.
	WimshCoordinatorDummy (WimshMac* m, WimshPhyMib* p);
		
	//! Ignore MSH-DSCH message from the MAC.
	void recvMshDsch (WimshMshDsch* dsch, double txtime = 0) { }
	//! Ignore MSH-NCFG message from the MAC.
	void recvMshNcfg (WimshMshNcfg* ncfg, double txtime = 0) { }

	//! Initialiaze the internal data structure. Do nothing
	void initialize() { };

	// Tcl interface.
	/*!
	  Includes the command 'mode X', where X is either 'fixed' or
	  'moving'. This command *must* be issued exactly once for
	  each coordinator. Otherwise, the behaviour is unspecified.
	  */
	int command (int argc, const char*const* argv);

protected:
	//! Election procedure called by handle().
	void election ();
	// :TODO:
	//! Election procedure called by handle().
	void electionDsch () {};
	//! Election procedure called by handle().
	void electionNcfg () {};
	//! Election procedure called by handle().
	void electionNent () {};
		
	//! Fill the IE about myself with junk information.
	void fillSelf (WimshMshDsch* dsch);

	//! Fill the IEs about my neighbors with junk information.
	void fillNeighbors (WimshMshDsch* dsch);

	//! Fill the IE about myself in an MSH-NCFG message.
	void fillSelf (WimshMshNcfg* ncfg) {}; // :TODO:
	//! Fill the IEs about my neighbors in an MSH-NCFG message.
	void fillNeighbors (WimshMshNcfg* ncfg) {}; //:TODO:

private:
	//! Start the timer the first time.
	void start ();
};

#endif // __NS2_WIMSH_COORDINATOR_H
