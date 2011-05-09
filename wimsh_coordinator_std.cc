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

#include <wimsh_coordinator_std.h>
#include <t_timers.h>
#include <wimsh_packet.h>
#include <wimsh_topology.h>
#include <wimsh_mac.h>
#include <stat.h>

#include <math.h>

/*
 *
 * class WimaxStandardCoordinator
 *
 */
 
WimshCoordinatorStandard::WimshCoordinatorStandard(WimshMac* m, WimshPhyMib* p):
   WimshCoordinator (m)
{
   phyMib_ = p;
   type_ = XMTUNAWARE;
   maxClique_ = false;
   maxAdvertisedNeighbors_ = -1;
}

int
WimshCoordinatorStandard::command (int argc, const char*const* argv)
{
   if ( argc == 2 && strcmp (argv[0], "mode") == 0 ) {
      if ( strcmp (argv[1], "xmtaware") == 0 ) {
         type_ = XMTAWARE;
      } else if ( strcmp (argv[1], "xmtunaware") == 0 ) {
         type_ = XMTUNAWARE;
      } else {
         fprintf (stderr, "invalid standard coordinator type '%s'. "
               "Choose 'xmtaware' or 'xmtunaware'.\n", argv[1]);
         return TCL_ERROR;
      }
      start ();
      return TCL_OK;
   } else if ( argc == 2 && strcmp (argv[0], "nextxmttime") == 0 ) {
      myDsch_.nextXmtTime_ = (unsigned int) atoi (argv[1]);
      myNcfg_.nextXmtTime_ = (unsigned int) atoi (argv[1]);
      return TCL_OK;
   } else if ( argc == 2 && strcmp (argv[0], "max-advertised") == 0 ) {
      maxAdvertisedNeighbors_ = (int) atoi (argv[1]);
      return TCL_OK;
   } else if ( argc == 2 && strcmp (argv[0], "holdoffexp-dsch") == 0 ) {
      myDsch_.holdOffExp_ = (unsigned int) atoi (argv[1]);
      return TCL_OK;
   } else if ( argc == 2 && strcmp (argv[0], "holdoffexp-ncfg") == 0 ) {
		myNcfg_.holdOffExp_ = (unsigned int) atoi (argv[1]);
      return TCL_OK;
   }
   return TCL_ERROR;
}

void
WimshCoordinatorStandard::initialize ()
{
   myDsch_.nextXmtMx_ = 0;
   myNcfg_.nextXmtMx_ = 0;
   std::vector<WimaxNodeId> neigh;
   
   // get the first hop neighbors
   mac_->topology()->neighbors( mac_->nodeId(), neigh, 1 );
   // update the neighbor list
   for ( unsigned int i = 0; i < neigh.size(); i++ ){
      nghListDsch_.push_back( NeighInfo(0, 0, neigh.at(i), 1) ); 
      nghListNcfg_.push_back( NeighInfo(0, 0, neigh.at(i), 1) ); 
   }
   neigh.clear();

   // get the two-hop neighbors
   mac_->topology()->neighbors( mac_->nodeId(), neigh, 2 );
   // update the neighbor list
   for ( unsigned int i = 0; i < neigh.size(); i++ ){
      nghListDsch_.push_back( NeighInfo(0, 0, neigh.at(i), 2) ); 
      nghListNcfg_.push_back( NeighInfo(0, 0, neigh.at(i), 2) ); 
   }

   // set the maxClique_ flag (false by default)
   /*std::vector<WimaxNodeId> maxClique =
      mac_->topology()->getMaxClique ();

   // check whether I belong to the above set
   for ( unsigned int i = 0 ; i < maxClique.size() ; i++ ) {
      if ( maxClique[i] == mac_->nodeId() ) {
         maxClique_ = true;
         break;
      }
   }*/

   // initialize the neighbors map
   std::list<NeighInfo>::iterator it;
   for ( it = nghListDsch_.begin(); it != nghListDsch_.end(); ++it ){
      nghMapDsch_[ it->nodeID_ ] = &(*it);
   }
   for ( it = nghListNcfg_.begin(); it != nghListNcfg_.end(); ++it ){
      nghMapNcfg_[ it->nodeID_ ] = &(*it);
   }
}

void
WimshCoordinatorStandard::start ()
{
   // compute the first transmission opportunity
	const unsigned int C = phyMib_->controlSlots();
	const unsigned int T = phyMib_->cfgInterval ();
   
   nextDschSlot_   = myDsch_.nextXmtTime_ % C;
   nextDschFrame_  = myDsch_.nextXmtTime_ / C;
	nextDschFrame_ += 1 + nextDschFrame_ / T;

	nextNcfgSlot_   = 1 + myNcfg_.nextXmtTime_ % ( C - 1 );
	nextNcfgFrame_  = myNcfg_.nextXmtTime_ / ( C - 1 );
	nextNcfgFrame_ += T * nextNcfgFrame_;

	nextNentSlot_   = 0;
	nextNentFrame_  = 0;

	timer_.start (0);
}
 

void
WimshCoordinatorStandard::fillSelf (WimshMshDsch* dsch)
{
   // fill the MAC header and meshsubheader fields
   dsch->src() = mac_->nodeId();
   dsch->hdr().crc() = true;
   // dsch->hdr().fragmentation() is false by default
   // dsch->hdr().length() is automatically computed
   // dsch->hdr().meshCid() is set to management, no ARQ, by default

   // fill the neighbor information about myself
   dsch->myself().nodeId_ = mac_->nodeId();

   dsch->myself().nextXmtTimeSec_ = myDsch_.nextXmtTimeSec_;
   dsch->myself().nextXmtTime_ = myDsch_.nextXmtTime_;
   dsch->myself().nextXmtMx_ = myDsch_.nextXmtMx_;
   dsch->myself().xmtHoldoffExponent_ = myDsch_.holdOffExp_;
}

void
WimshCoordinatorStandard::fillNeighbors (WimshMshDsch* dsch)
{
   if ( type_ == XMTUNAWARE ) return;

   std::list<NeighInfo>::iterator it;
   unsigned int advertisedNeighbors = 0;
   for ( it = nghListDsch_.begin(); it != nghListDsch_.end(); ++it ){

      // check whether there is enough room to add another IE
      // or whether the maximum number of advertised neighbors
      // is reached
      if ( ( maxAdvertisedNeighbors_ >= 0 &&
             (int)advertisedNeighbors >= maxAdvertisedNeighbors_ ) ||
           dsch->remaining() < WimshMshDsch::NghIE::size() ) break;
      
      WimshMshDsch::NghIE ie; // new information element

      // check if the xmtTime info is updated
      if ( it->nextXmtTime_ > 0 ) {
         // fill the information element
         ie.nextXmtTime_ = it->nextXmtTime_;
         ie.nextXmtMx_ = it->nextXmtMx_;
         ie.xmtHoldoffExponent_ = it->holdOffExp_;
         ie.nodeId_ = it->nodeID_;

         // chech the condition to add to the dsch 
         if ( it->nhop_ == 1 ) {
            dsch->add (ie);
            ++advertisedNeighbors;
	 }
      }
   }
}

void
WimshCoordinatorStandard::fillSelf (WimshMshNcfg* ncfg)
{
   // fill the MAC header and meshsubheader fields
   ncfg->src() = mac_->nodeId();
   ncfg->hdr().crc() = true;
   // ncfg->hdr().fragmentation() is false by default
   // ncfg->hdr().length() is automatically computed
   // ncfg->hdr().meshCid() is set to management, no ARQ, by default

   // fill the neighbor information about myself
   ncfg->myself().nodeId_ = mac_->nodeId();

   ncfg->myself().nextXmtTime_ = myNcfg_.nextXmtTime_;
   ncfg->myself().nextXmtMx_ = myNcfg_.nextXmtMx_;
   ncfg->myself().xmtHoldoffExponent_ = myNcfg_.holdOffExp_;
}

void
WimshCoordinatorStandard::fillNeighbors (WimshMshNcfg* ncfg)
{
   if ( type_ == XMTUNAWARE ) return;
	abort();  // :TODO: fix
}

void
WimshCoordinatorStandard::recvMshDsch (WimshMshDsch *dsch, double txtime)
{
   assert ( initialized ); 
   if ( WimaxDebug::trace("WCRD::recvMshDsch") ) fprintf (stderr,
      "%.9f WMAC::recvMshDsch[%d]\n", NOW, mac_->nodeId());
   
   // compute the current slot from the beginning of time
   const unsigned int currentSlot = currentCtrlSlotDsch(txtime);
   
   std::map< WimaxNodeId, NeighInfo* >::iterator neighbor_it;
   // list scan is optimized by means of the neighbors map
   neighbor_it = nghMapDsch_.find( dsch->myself().nodeId_ );

   // update sender node info
   // Note that neighbor MUST be != nghMapDsch_.end()
   if ( type_ == XMTAWARE ){
      neighbor_it->second->nextXmtTime_ = dsch->myself().nextXmtTime_;
   } else {
      // we do *not* know the next transmit time, but we store the
      // lower bound of the advertised range, for the purpose of
      // deriving the earliest subsequent transmit time
      neighbor_it->second->nextXmtTime_ =
         1 + dsch->myself().nextXmtMx_ *
         ( 1 << dsch->myself().xmtHoldoffExponent_ );
   }
   neighbor_it->second->nextXmtMx_ = dsch->myself().nextXmtMx_;
   neighbor_it->second->holdOffExp_ = dsch->myself().xmtHoldoffExponent_;
   neighbor_it->second->timeStamp_ = currentSlot;

   if ( type_ == XMTAWARE ){
      // update neighbor infos
      std::list< WimshMshDsch::NghIE > nghList = dsch->ngh();
      std::list< WimshMshDsch::NghIE >::iterator it;
   
      for ( it = nghList.begin(); it != nghList.end(); ++it ){

         neighbor_it = nghMapDsch_.find( it->nodeId_ );

         if( neighbor_it != nghMapDsch_.end() && 
            neighbor_it->second->nhop_ == 2  ) {
            // update
            neighbor_it->second->nextXmtTime_ = it->nextXmtTime_;
            neighbor_it->second->nextXmtMx_ = it->nextXmtMx_;
            neighbor_it->second->holdOffExp_ = it->xmtHoldoffExponent_;
            neighbor_it->second->timeStamp_ = currentSlot;
            break;
         }
      }
   }
}

void
WimshCoordinatorStandard::recvMshNcfg (WimshMshNcfg *ncfg, double txtime)
{
   assert ( initialized ); 

   if ( WimaxDebug::trace("WCRD::recvMshNcfg") ) fprintf (stderr,
      "%.9f WMAC::recvMshNcfg[%d]\n", NOW, mac_->nodeId());

   // compute the current slot from the beginning of time
   const unsigned int currentSlot = currentCtrlSlotNcfg(txtime);
   
   std::map< WimaxNodeId, NeighInfo* >::iterator neighbor_it;
   // list scan is optimized by means of the neighbors map
   neighbor_it = nghMapNcfg_.find( ncfg->myself().nodeId_ );

   // update sender node info
   // Note that neighbor MUST be != nghMapDsch_.end()
   if ( type_ == XMTAWARE ){
      // :TODO:
		abort();
   } else {
      // we do *not* know the next transmit time, but we store the
      // lower bound of the advertised range, for the purpose of
      // deriving the earliest subsequent transmit time
      neighbor_it->second->nextXmtTime_ =
         1 + ncfg->myself().nextXmtMx_ *
         ( 1 << ncfg->myself().xmtHoldoffExponent_ );
   }
   neighbor_it->second->nextXmtMx_ = ncfg->myself().nextXmtMx_;
   neighbor_it->second->holdOffExp_ = ncfg->myself().xmtHoldoffExponent_;
   neighbor_it->second->timeStamp_ = currentSlot;
}

void
WimshCoordinatorStandard::electionDsch ()
{   
   // number of control slots per frame
   const unsigned int C = phyMib_->controlSlots ();
	const unsigned int T = phyMib_->cfgInterval ();

   // run the mesh election procedure
   competition (nghListDsch_, myDsch_, wimax::MSHDSCH);

   // compute the next opportunity since the system start
   unsigned int slot = currentCtrlSlotDsch();
	// fprintf (stderr,
	// "current %u relativeSlot %u, absCurFrame %u nextXmtTime %u ",
	// currentCtrlSlot(), slot, absCurFrame, myDsch_.nextXmtTime_);
	slot += myDsch_.nextXmtTime_;

   // statistic collection
   if ( maxClique_ ) Stat::put ("wimsh_election_util", 0, 1);
   Stat::put ("wimsh_dsch_election_random", mac_->index(),
		( myDsch_.nextXmtTime_ - computeHoldoffTime(myDsch_.holdOffExp_) ) );
   Stat::put ("wimsh_dsch_election_slots", mac_->index(), myDsch_.nextXmtTime_);
   Stat::put ("wimsh_dsch_election_slots_d", mac_->index(), myDsch_.nextXmtTime_);

	slot += ( 1 + ( slot / C ) / T ) * C;

	// fprintf(stderr, "nextabsslot %u ", absCurSlot);
   nextDschSlot_ = slot % C;
   nextDschFrame_ = slot / C;
	// fprintf(stderr, "%u %u\n", nextDschSlot_, nextDschFrame_);

	// set the next transmit time, in seconds
	myDsch_.nextXmtTimeSec_ =
			  phyMib_->controlSlotDuration() * nextDschSlot_
			+ phyMib_->frameDuration() * nextDschFrame_
			- NOW;
}

void
WimshCoordinatorStandard::electionNcfg ()
{   
   // number of control slots per frame
   const unsigned int C = phyMib_->controlSlots ();
	const unsigned int T = phyMib_->cfgInterval ();

   // run the mesh election procedure
   competition (nghListNcfg_, myNcfg_, wimax::MSHNCFG);

   // compute the next opportunity since the system start
   unsigned int slot = currentCtrlSlotNcfg();
	// fprintf (stderr,
	// "current %u relativeSlot %u, absCurFrame %u nextXmtTime %u\n",
	// currentCtrlSlotNcfg(), slot, slot / C, myNcfg_.nextXmtTime_);
	slot += myNcfg_.nextXmtTime_;

   // statistic collection
   Stat::put ("wimsh_ncfg_election_random", mac_->index(),
		( myNcfg_.nextXmtTime_ - computeHoldoffTime(myNcfg_.holdOffExp_) ) );
   Stat::put ("wimsh_ncfg_election_slots", mac_->index(), myNcfg_.nextXmtTime_);

	slot += ( 1 + T * C ) * ( slot / ( C - 1 ) );

	// fprintf(stderr, "nextabsslot %u ", absCurSlot);
   nextNcfgSlot_ = 1 + slot % ( C - 1 );
   nextNcfgFrame_ = slot / C;
	// fprintf(stderr, "%u %u\n", nextNcfgSlot_, nextNcfgFrame_);
}

void
WimshCoordinatorStandard::electionNent ()
{
	const unsigned int T = phyMib_->cfgInterval ();
	nextNentFrame_ += T + 1;
}

void 
WimshCoordinatorStandard::competition (
		std::list<NeighInfo>& nghList,
		MyInfo& my, wimax::BurstType type) 
{   
   // compute the current slot from the beginning of time
   unsigned int currentSlot = 0;
	if ( type == wimax::MSHDSCH ) currentSlot = currentCtrlSlotDsch();
	else if ( type == wimax::MSHNCFG ) currentSlot = currentCtrlSlotNcfg();
	else abort();

   std::list<NeighInfo>::iterator it;
   for( it = nghList.begin(); it != nghList.end(); ++it ){
      unsigned int elapsed = currentSlot - it->timeStamp_;
      it->timeStamp_ = currentSlot;

      it->nextXmtTime_ = ( it->nextXmtTime_ > elapsed ) ?
         it->nextXmtTime_ - elapsed : 0;
      
      // set the earliest subsequent xmt time
      it->earlSubXmtTime_ = it->nextXmtTime_ 
         + computeHoldoffTime( it->holdOffExp_ );
   }
   // sort the neighbor list
   nghList.sort ();

   // compute the relative xmt
   unsigned int TempXmtTime = computeHoldoffTime(my.holdOffExp_);
   // mesh election procedure
   bool success = false;
   while (!success){
      // compute the competing nodes' set
		Stat::put ("wimsh_dsch_competing", mac_->index(),
				(double) competingNodes( TempXmtTime, nghList ));

      if ( meshElection (TempXmtTime, mac_->nodeId(), nghList, type) == false )
         ++TempXmtTime;
      else {
         success = true;
         my.nextXmtTime_ = TempXmtTime;
         my.nextXmtMx_ = computeXmtTimeMx( my.holdOffExp_, my.nextXmtTime_ );
      }   
   }
}

bool 
WimshCoordinatorStandard::meshElection (unsigned int TempXmtTime, 
                               short unsigned int nodeID,
										 std::list<NeighInfo>& nghList,
										 wimax::BurstType type)
{
   // compute the current slot from the beginning of time
   unsigned int currentSlot = 0;
	if ( type == wimax::MSHDSCH ) currentSlot = currentCtrlSlotDsch();
	else if ( type == wimax::MSHNCFG ) currentSlot = currentCtrlSlotNcfg();
	else abort();

   // IEEE 802.16 standard procedure
   unsigned int nbr_smear_val, smear_val1, smear_val2;
   smear_val1 = inline_smear( nodeID ^ ( TempXmtTime + currentSlot ) );
   smear_val2 = inline_smear( nodeID + ( TempXmtTime + currentSlot ) );
   
   std::list<NeighInfo>::iterator it;
   for( it = nghList.begin(); it != nghList.end(); ++it ){
      if( it->competing_ ){
         nbr_smear_val = inline_smear( it->nodeID_ ^ 
            ( TempXmtTime + currentSlot ) );

         if( nbr_smear_val > smear_val1 ){
            return false;
         } else if( nbr_smear_val == smear_val1 ) {
            nbr_smear_val = inline_smear( it->nodeID_ + 
               ( TempXmtTime + currentSlot ) );

            if( nbr_smear_val > smear_val2 ) {
               return false;
            }
         } else if( nbr_smear_val == smear_val2 ) {
            if( ( ( ( TempXmtTime + currentSlot )%2 == 0 ) && 
               ( it->nodeID_ > nodeID ) ) ||
               ( ( ( TempXmtTime + currentSlot )%2 == 1 ) && 
               ( it->nodeID_ < nodeID ) ) ) {
               return false;
            }
         }
      }
   }
   return true;
}


unsigned int
WimshCoordinatorStandard::competingNodes (unsigned int TempXmtTime,
		std::list<NeighInfo>& nghList)
{
	unsigned int competingNodes = 0;
   std::list<NeighInfo>::iterator it;
   for( it = nghList.begin(); it != nghList.end(); ++it ){
      it->competing_ = false;
      unsigned int holdexp = it->holdOffExp_;
      // information not updated
      // it is considered a competing node
      if( it->nextXmtTime_ == 0 ){ 
         it->competing_ = true;
			++competingNodes;
      } else { 
         if (
              ( type_ == XMTAWARE && it->nextXmtTime_ == TempXmtTime ) ||
              ( type_ == XMTUNAWARE &&
                   ( TempXmtTime >= it->nextXmtTime_ &&
		/*	XXX			TempXmtTime < it->nextXmtTime_ + ( 1 << holdexp ) ) ) ||*/
                     TempXmtTime <= it->nextXmtTime_ + ( 1 << holdexp ) ) ) ||
              ( it->earlSubXmtTime_ <= TempXmtTime )
			) {
            it->competing_ = true;
				++competingNodes;
			}
      }
   }
	return competingNodes;
}

// utility functions for the mesh election procedure


unsigned int
WimshCoordinatorStandard::computeHoldoffTime(unsigned holdOffExp)
{
   return 1 << ( holdOffExp + 4 );
}


unsigned int
WimshCoordinatorStandard::computeXmtTimeMx
   ( unsigned int holdOffExp, unsigned int NextXmtTime )
{
   // implements the research of x in the function:
   // 2^holdoffExp*x < NextXmtTime <= 2^holdoffExp*(x+1)
   unsigned int hold = 1 << holdOffExp;
   bool found = false;
   unsigned int x = 0;
   while( !found ) {
      if( ( NextXmtTime > x * hold ) && 
         ( NextXmtTime <= (x+1) * hold ) ) found = true;
      else x++;
   }
   return x;
}



bool 
WimshCoordinatorStandard::eligible 
   ( unsigned xmtmx, unsigned TempXmtTime, unsigned holdexp )
{
   return    ( ( xmtmx < TempXmtTime ) && 
      ( TempXmtTime <= ( ( xmtmx + 1 ) * ( 1 << holdexp ) ) ) );
}

unsigned int
WimshCoordinatorStandard::inline_smear(unsigned short int val)
{
   val += ( val << 12 );
   val ^= ( val >> 22 );
   val += ( val << 4 );
   val ^= ( val >> 9 );
   val += ( val << 10 );
   val ^= ( val >> 2 );
   val += ( val << 7 );
   val ^= ( val >> 12 );
   return val;
}

unsigned int 
WimshCoordinatorStandard::currentCtrlSlot (double txtime)
{
   // number of control slots per frame
   const unsigned int C = phyMib_->controlSlots();

   // compute the current slot in the current frame
   const unsigned int curslotFrame = (unsigned int) round (
         ( NOW  - mac_->frame() * phyMib_->frameDuration() - txtime ) /
         phyMib_->controlSlotDuration() );
   
   // return the current slot from the beginning of time
   return ( C * mac_->frame() + curslotFrame );
}

unsigned int 
WimshCoordinatorStandard::currentCtrlSlotDsch (double txtime)
{
   const unsigned int C = phyMib_->controlSlots();
	const unsigned int T = phyMib_->cfgInterval ();

   unsigned int slot = currentCtrlSlot();
	unsigned int frame = slot / C;
	slot -= C * ( 1 + ( frame - 1 ) / ( T + 1 ) );
	return slot;
}

unsigned int 
WimshCoordinatorStandard::currentCtrlSlotNcfg (double txtime)
{
   const unsigned int C = phyMib_->controlSlots();
	const unsigned int T = phyMib_->cfgInterval ();

   unsigned int slot = currentCtrlSlot(txtime);
	unsigned int frame = slot / C;
	slot -= ( 1 + T * C ) * ( frame / ( T + 1 ) );
	return slot;
}
