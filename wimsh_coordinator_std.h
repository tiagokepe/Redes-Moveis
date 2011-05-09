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

#ifndef __NS2_WIMAX_COORDINATOR_STD_H
#define __NS2_WIMAX_COORDINATOR_STD_H

#include <list>

#include <wimsh_coordinator.h>

/*
 *
 * class WimshCoordinatorStandard
 *
 */

//! 802.16 standard coordinator
/*!
  :TODO: update the documentation

  Implements the 802.16 standard election mechanism modified to avoid 
  collision of requests. Two modification are implemented

  In 'XMTAWARE' mode, the standard 802.16 MAC protocol is modified to
  make SSs transmit the NextXmtTime information 

  In 'XMTUNAWARE' mode, the election mechanism is performed in the worst
  case:
  	i) the EarliestSubsequentXmtTime = holdOffTime + xmtTimeWindow lower
  	bound
  	ii) two-hop neighbors are always considered competing nodes 
  */
class WimshCoordinatorStandard : public WimshCoordinator {
	//! Coordination type (fixed or moving).
	enum { XMTAWARE = 0, XMTUNAWARE } type_;

	//! PHY MIB.
	WimshPhyMib* phyMib_;

	//! This node information data structure.
	struct MyInfo {
		//! Next transmit time multiplier.
		unsigned short nextXmtMx_;
		//! Holdoff exponent.
		unsigned short holdOffExp_;
		//! Next transmit time, in slots.
		unsigned int nextXmtTime_;
		//! Next transmit time, in seconds.
		double nextXmtTimeSec_;

		//! Create an empty data structure.
		MyInfo () {
			nextXmtMx_ = 0;
			holdOffExp_ = 0;
			nextXmtTime_ = 0;
			nextXmtTimeSec_ = 0;
		}
	};

	//! Maximum number of advertised neighbors.
	/*!
	 * A negative value means that the maximum number is limited
	 * by the control slot size only.
	 */
	int maxAdvertisedNeighbors_;

	//! Neighbors information data structure.
	struct NeighInfo{
		//! Next trasmit time 
		unsigned nextXmtTime_;
		//! Next Transmit mx
		unsigned short nextXmtMx_;
		//! Holdoff exponent
		unsigned short holdOffExp_;
		//! Earliest subsequent xmt time
		unsigned int earlSubXmtTime_;
		//! Node ID
		WimaxNodeId nodeID_;
		//! First hop or second hop neighbor
		unsigned short nhop_;
		//! Tell if the node is competing for the current slot
		bool competing_;
		//! Time stamp update
		unsigned int timeStamp_;
		
		//! Constructor
		NeighInfo(unsigned nextXmtTime, unsigned short nextXmtMx, 
			WimaxNodeId nodeID, unsigned short nhop){
			nextXmtTime_ = nextXmtTime; nextXmtMx_ = nextXmtMx;
			nodeID_ = nodeID; nhop_ = nhop; earlSubXmtTime_ = 0;
			competing_ = true; timeStamp_ = 0;
		}

		//! Operator redefinition for sorting the list of NeighInfo
		bool operator < (const NeighInfo right) { 
			return nextXmtTime_ < right.nextXmtTime_; }	
	};

	//! List of neighbors for the purpose of sending MSH-DSCH messages.
	/*!
	  This list is used during the mesh election procedure. Neighbors must be
	  sorted in increasing nextXmtTime_. The < operator in the struct NeighInfo
	  has been redefined for that purpose.
	  */
	std::list<NeighInfo> nghListDsch_;
	//! Map of neighbors to optimize the access to nghListSch_ data.
	std::map<WimaxNodeId, NeighInfo*> nghMapDsch_;

	//! List of neighbors for the purpose of sending MSH-NCFG messages.
	std::list<NeighInfo> nghListNcfg_;
	//! Map of neighbors to optimize the access to nghListNcfg_ data.
	std::map<WimaxNodeId, NeighInfo*> nghMapNcfg_;

	//! My information regarding MSH-DSCH messages.
	MyInfo myDsch_;
	//! My information regarding MSH-NCFG messages.
	MyInfo myNcfg_;

	//! True if this node belongs to the maximum clique in the conflit graph.
	bool maxClique_;

public:
	//! Create the coordinator.
	WimshCoordinatorStandard (WimshMac* m, WimshPhyMib* p);
		
	//! Manage MSH-DSCH message from the MAC.
	void recvMshDsch (WimshMshDsch* dsch, double txtime = 0);
	//! Manage MSH-NCFG message from the MAC.
	void recvMshNcfg (WimshMshNcfg* ncfg, double txtime = 0);

	//! Initialiaze the internal data structure.
	void initialize();

	// Tcl interface.
	/*!
	  Includes the command 'mode X', where X is either 'fixed' or
	  'moving'. This command *must* be issued exactly once for
	  each coordinator. Otherwise, the behaviour is unspecified.
	  */
	int command (int argc, const char*const* argv);

protected:
	//! Election procedure for Dsch called by handle().
	void electionDsch ();
	//! Election procedure called by handle().
	void electionNcfg ();
	//! Election procedure called by handle().
	void electionNent ();
	
	//! Fill the IE about myself with scheduling information.
	void fillSelf (WimshMshDsch* dsch);

	//! Fill the IEs about my neighbors with scheduling information.
	void fillNeighbors (WimshMshDsch* dsch);

	//! Fill the IE about myself with scheduling information. 
	void fillSelf (WimshMshNcfg* ncfg);

	//! Fill the IEs about my neighbors with scheduling information.	
	void fillNeighbors (WimshMshNcfg* ncfg);

private:
	//! Start the timer the first time.
	void start ();

	//! Competition procedure
	/*!
	  Run the standard mesh election procedure as specified in 
	  IEEE 802.16-2004 standard, Section 6.3.7.5.5.6 pp. 159-160
	  nextXmtTime_ is filled with the slot number relative to
	  the node's next Xmt Time
	  */
	void competition (std::list<NeighInfo>& nghList,MyInfo& my,
			wimax::BurstType type);

	//! Find the competing nodes given a certain XmtTime.
	/*!
	  Each neighbor that is considered to be a competitor has the competing_
	  flag set to one into the nghList vector.
	  Return the number of competitors.
	  */
	unsigned int competingNodes (unsigned int TempXmtTime,
			std::list<NeighInfo>& nghList);
	
	//! Execute the mesh election procedure
	/*!
	  This function is identical to that in the standard p. 160
	  */
	bool meshElection (unsigned int TempXmtTime,
			short unsigned int nodeID,
			std::list<NeighInfo>& nghList,
			wimax::BurstType type);

	//! Return the holdoff time.
	unsigned int computeHoldoffTime(unsigned holdOffExp);

	//! Return the XmtTimeMx.
	/*!
	  Find x in the following formula given NextXmtTime and holdOffExp:
	  2^holdoffExp*x < NextXmtTime <= 2^holdoffExp*(x+1)
	  */
	unsigned int computeXmtTimeMx(unsigned holdOffExp, unsigned NextXmtTime);

	//! Return true if a node is eligible.
	bool eligible (unsigned xmtmx, unsigned TempXmtTime, unsigned holdexp);
	//! Return the hash for nodeID; see IEEE 802.16 std pp. 160.
	unsigned int inline_smear(unsigned short int val);
	//! Compute the control slot from the dawn of time.
	unsigned int currentCtrlSlot(double txtime = 0);
	//! Compute the MSH-DSCH slot from the dawn of time.
	unsigned int currentCtrlSlotDsch(double txtime = 0);
	//! Compute the MSH-NCFG slot from the dawn of time.
	unsigned int currentCtrlSlotNcfg(double txtime = 0);

};

#endif // __NS2_WIMAX_COORDINATOR_STD_H
