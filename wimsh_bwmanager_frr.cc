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

#include <wimsh_bwmanager_frr.h>

#include <wimsh_mac.h>
#include <wimsh_topology.h>
#include <wimsh_packet.h>

#include <random.h>
#include <stat.h>

WimshBwManagerFairRR::WimshBwManagerFairRR (WimshMac* m) :
	WimshBwManager (m), wm_ (m)
{
	// resize the unavailabilities to receive of this node, confirmed or not
	busy_.resize (HORIZON);
	unconfirmedSlots_.resize (HORIZON);

	for ( unsigned int i = 0 ; i < HORIZON ; i++ ) {
		// clear all the bits of the busy_ and unconfirmedSlots_ bitmap
		busy_[i].reset();
		unconfirmedSlots_[i].reset();
	}

	regrantOffset_         = 1;
	regrantDuration_       = 1;
	avlAdvertise_          = true;
	regrantEnabled_        = true;
	fairGrant_             = true;
	fairRequest_           = true;
	fairRegrant_           = true;
	deficitOverflow_       = false;
	grantFitRandomChannel_ = false;
	sameRegrantHorizon_    = false;
	maxDeficit_            = 0;
	maxBacklog_            = 0;
	roundDuration_         = 0;
	ddTimeout_             = 0;
	ddTimer_               = 0;
	minGrant_              = 1;
}

int
WimshBwManagerFairRR::command (int argc, const char*const* argv)
{
	if ( argc == 2 && strcmp (argv[0], "availabilities") == 0 ) {
		if ( strcmp (argv[1], "on") == 0 ) {
			avlAdvertise_ = true;
		} else if ( strcmp (argv[1], "off") == 0 ) {
			avlAdvertise_ = false;
		} else {
			fprintf (stderr, "invalid availabilities '%s' command. "
					"Choose either 'on' or 'off'", argv[1]);
			return TCL_ERROR;
		}
      return TCL_OK;
	} else if ( argc == 2 && strcmp (argv[0], "regrant") == 0 ) {
		if ( strcmp (argv[1], "on") == 0 ) {
			regrantEnabled_ = true;
		} else if ( strcmp (argv[1], "off") == 0 ) {
			regrantEnabled_ = false;
		} else {
			fprintf (stderr, "invalid regrant '%s' command. "
					"Choose either 'on' or 'off'", argv[1]);
			return TCL_ERROR;
		}
      return TCL_OK;
	} else if ( argc == 2 && strcmp (argv[0], "regrant-same-horizon") == 0 ) {
		if ( strcmp (argv[1], "on") == 0 ) {
			sameRegrantHorizon_ = true;
		} else if ( strcmp (argv[1], "off") == 0 ) {
			sameRegrantHorizon_ = false;
		} else {
			fprintf (stderr, "invalid regrant-same-horizon '%s' command. "
					"Choose either 'on' or 'off'", argv[1]);
			return TCL_ERROR;
		}
      return TCL_OK;
	} else if ( argc == 2 && strcmp (argv[0], "regrant-offset") == 0 ) {
		regrantOffset_ = (unsigned int) atoi (argv[1]);
		return TCL_OK;
	} else if ( argc == 2 && strcmp (argv[0], "regrant-duration") == 0 ) {
		regrantDuration_ = (unsigned int) atoi (argv[1]);
		return TCL_OK;
	} else if ( argc == 2 && strcmp (argv[0], "dd-timeout") == 0 ) {
		if ( atoi (argv[1]) < 0 ) {
			fprintf (stderr, "Invalid deadlock detection timeout '%d'. "
					"Choose a number greater than or equal to zero\n",
					atoi (argv[1]));
			return TCL_ERROR;
		}
		ddTimeout_ = atoi (argv[1]);
		return TCL_OK;
	} else if ( argc == 2 && strcmp (argv[0], "round-duration") == 0 ) {
		if ( atoi (argv[1]) <= 0 ) {
			fprintf (stderr, "Invalid round duration '%d'. "
					"Choose a number greater than zero (in bytes)\n",
					atoi (argv[1]));
			return TCL_ERROR;
		}
		roundDuration_ = (unsigned int) atoi (argv[1]);
		return TCL_OK;
	} else if ( argc == 2 && strcmp (argv[0], "max-deficit") == 0 ) {
		if ( atoi (argv[1]) < 0 ) {
			fprintf (stderr, "Invalid maximum deficit amount '%d'. "
					"Choose a number greater than or equal to zero (in bytes)\n",
					atoi (argv[1]));
			return TCL_ERROR;
		}

		maxDeficit_ = (unsigned int) atoi (argv[1]);
		return TCL_OK;
	} else if ( argc == 2 && strcmp (argv[0], "min-grant") == 0 ) {
		if ( atoi (argv[1]) < 1 ) {
			fprintf (stderr, "Invalid minimum grant size '%d'. Choose "
					"a number greater than or equal to one (in OFDM symbols)\n",
					atoi (argv[1]));
			return TCL_ERROR;
		}

		minGrant_ = (unsigned int) atoi (argv[1]);
		return TCL_OK;
	} else if ( argc == 2 && strcmp (argv[0], "max-backlog") == 0 ) {
		if ( atoi (argv[1]) < 0 ) {
			fprintf (stderr, "Invalid maximum backlog amount '%d'. "
					"Choose a number greater than or equal to zero (in bytes)\n",
					atoi (argv[1]));
			return TCL_ERROR;
		}

		maxBacklog_ = (unsigned int) atoi (argv[1]);
		return TCL_OK;
	} else if ( argc == 2 && strcmp (argv[0], "fairness") == 0 ) {
		if ( strcmp (argv[1], "grant") == 0 ) {
			fairGrant_ = true;
		} else if ( strcmp (argv[1], "regrant") == 0 ) {
			fairRegrant_ = true;
		} else if ( strcmp (argv[1], "request") == 0 ) {
			fairRequest_ = true;
		} else if ( strcmp (argv[1], "no") == 0 ) {
			fairRequest_ = false;
			fairRegrant_ = false;
			fairGrant_ = false;
		} else {
			fprintf (stderr, "unknown fairness specifier '%s'. "
					"Choose 'grant', 'regrant' or 'no'\n", argv[1]);
			return TCL_ERROR;
		}
		return TCL_OK;
	} else if ( argc == 3 && strcmp (argv[0], "grant-fit") == 0 ) {
		if ( strcmp (argv[1], "channel" ) == 0 ) {
			if ( strcmp (argv[2], "random") == 0 ) {
				grantFitRandomChannel_ = true;
			} else if ( strcmp (argv[2], "first") == 0 ) {
				grantFitRandomChannel_ = false;
			}
		} else {
			fprintf (stderr, "unknown grant-fit specifier '%s %s'\n",
					argv[1], argv[2]);
			return TCL_ERROR;
		}
		return TCL_OK;
	} else if ( strcmp (argv[0], "wm") == 0 ) {
		return wm_.command (argc - 1, argv + 1);
	}

	return TCL_ERROR;
}

void
WimshBwManagerFairRR::initialize ()
{
	const unsigned int neighbors = mac_->nneighs();
	
	// resize and clear the bw request/grant data structure
	neigh_.resize (neighbors);

	// resize and clear the neighbors' unavailabilites bitmaps
	neigh_tx_unavl_.resize (neighbors);
	for ( unsigned int ngh = 0 ; ngh < neighbors ; ngh++ ) {
		neigh_tx_unavl_[ngh].resize (mac_->nchannels());
		for ( unsigned int ch = 0 ; ch < mac_->nchannels() ; ch++ ) {
			neigh_tx_unavl_[ngh][ch].resize (HORIZON);
			for ( unsigned int f = 0 ; f < HORIZON ; f++ )
				neigh_tx_unavl_[ngh][ch][f].reset();
		}
	}

	self_rx_unavl_.resize (mac_->nchannels());
	self_tx_unavl_.resize (mac_->nchannels());
	for ( unsigned int ch = 0 ; ch < mac_->nchannels() ; ch++ ) {
		self_rx_unavl_[ch].resize (HORIZON);
		self_tx_unavl_[ch].resize (HORIZON);
		for ( unsigned int f = 0 ; f < HORIZON ; f++ ) {
			self_rx_unavl_[ch][f].reset();
			self_tx_unavl_[ch][f].reset();
		}
	}

	// initialize the weight manager
	wm_.initialize ();
}

void
WimshBwManagerFairRR::recvMshDsch (WimshMshDsch* dsch)
{
	if ( WimaxDebug::trace("WBWM::recvMshDsch") ) fprintf (stderr,
			"%.9f WBWM::recvMshDsch[%d]\n", NOW, mac_->nodeId());

	rcvGrants(dsch);
	rcvAvailabilities(dsch);
	rcvRequests(dsch);
}

void
WimshBwManagerFairRR::rcvGrants (WimshMshDsch* dsch)
{
	// get the local identifier of the node who sent this MSH-DSCH
	unsigned int ndx = mac_->neigh2ndx (dsch->src());

	// in any case we set the rcvCnf_ flag to true, so that the grant()
	// function will grant any unconfirmed bandwidth with a higher
	// priority than that of "fresh" bandwidth grants
	neigh_[ndx].rcvCnf_ = true;

	// get the list of grants/confirmations
	std::list<WimshMshDsch::GntIE>& gnt = dsch->gnt();

	std::list<WimshMshDsch::GntIE>::iterator it;
	for ( it = gnt.begin() ; it != gnt.end() ; ++it ) {

		//
		// grant
		//

		// if this grant is addressed to us, we first schedule a confirmation
		// message to be advertised as soon as possible by this node,
		// then mark the slots as unconfirmed unavailable
		// moreover, we update the amount of granted bandwidth
		if ( it->fromRequester_ == false && it->nodeId_ == mac_->nodeId() ) {

			// schedule the confirmation to be sent asap
			// modify the destination NodeID and the grant direction
			it->nodeId_ = dsch->src();
			it->fromRequester_ = true;

			// add the IE to the list of unconfirmed grants
			unconfirmed_.push_back (*it);

			// number of frames over which the grant spans
			// we assume that bandwidth is never granted in the past
			unsigned int frange = WimshMshDsch::pers2frames (it->persistence_);

			setSlots (unconfirmedSlots_, it->frame_, frange,
					it->start_, it->range_, true);

			// update the amount of bytes granted from this node
			neigh_[ndx].gnt_out_ +=
				frange * mac_->slots2bytes (ndx, it->range_, true);
			Stat::put ( "wimsh_gnt_out", mac_->index(),
				frange * mac_->slots2bytes (ndx, it->range_, true) );

			// we enforce the number of granted bytes to be smaller than
			// that of requested bytes
			neigh_[ndx].gnt_out_ =
				( neigh_[ndx].gnt_out_ < neigh_[ndx].req_out_ ) ?
				neigh_[ndx].gnt_out_ : neigh_[ndx].req_out_;      // XXX
		}

		// if this grant is not addressed to us, then add a pending availability
		// and update the bandwidth grant/confirm data structures so that:
		// - we will not grant bandwidth to the requester on any channel
		// - we will not grant bandwidth to the granter's neighbors on the
		//   same channel of this grant
		// - we will not confirm bandwidth on the same channel of this grant
		if ( it->fromRequester_ == false && it->nodeId_ != mac_->nodeId() ) {
			
			//
			// create a new availability and add it to the pending list
			//

			WimshMshDsch::AvlIE avl;
			avl.frame_ = it->frame_;
			avl.start_ = it->start_;
			avl.range_ = it->range_;
			avl.direction_ = WimshMshDsch::UNAVAILABLE;
			avl.persistence_ = it->persistence_;
			avl.channel_ = it->channel_;

			// push the new availability into the pending list
			availabilities_.push_back (avl);

			//
			// the granter is able to receive data from this node while
			// it is receiving data from any of its neighbors.
			// Thus, we mark the granted slots as unavailable on all channels
			//

			// number of frames over which the grant spans
			// we assume that bandwidth is never granted in the past
			unsigned int frange = WimshMshDsch::pers2frames (it->persistence_);

			// set the minislots as unavailable for the requester to transmit
			for ( unsigned int ch = 0 ; ch < mac_->nchannels() ; ch++ ) {
				setSlots (neigh_tx_unavl_[ndx][ch], it->frame_, frange,
						it->start_, it->range_, true);
			}

			//
			// if the requester is one of our neighbors, then we will not
			// be able to grant bandwidth to it (if requested) into the
			// set of granted slots on any channel
			//

			if ( mac_->topology()->neighbors (it->nodeId_, mac_->nodeId()) ) {
				// index of the requester
				const unsigned int ndx = mac_->neigh2ndx (it->nodeId_);

				// set the minislots as unavailable for the requester to transmit
				for ( unsigned int ch = 0 ; ch < mac_->nchannels() ; ch++ ) {
					setSlots (neigh_tx_unavl_[ndx][ch], it->frame_, frange,
							it->start_, it->range_, true);
				}
			}

			//
			// all neighbors of the granter will not be able to transmit
			// in the set of granted slots on the specified channel, which
			// is thus set as unavailable for transmission for all the
			// neighbors of the granter which are also our neighbors
			//

			std::vector<WimaxNodeId> gntNeigh;  // array of the granter's neighbors
			mac_->topology()->neighbors (dsch->src(), gntNeigh); // retrieve them
			for ( unsigned int ngh = 0 ; ngh < gntNeigh.size() ; ngh++ ) {

				// skip the requester, which has been already managed,
				// and nodes which are not our neighbors
				if ( gntNeigh[ngh] == it->nodeId_ ||
					  ! mac_->topology()->neighbors (gntNeigh[ngh], mac_->nodeId()) )
					continue;

				// otherwise, set the granted slots as unavailable
				const unsigned int ndx = mac_->neigh2ndx (gntNeigh[ngh]); // index
				setSlots (neigh_tx_unavl_[ndx][it->channel_],
						it->frame_, frange, it->start_, it->range_, true);
			}

			//
			// we are not able to transmit in the granted slots on the specified
			// channel (ie. to confirm bandwidth, even though it has been granted)
			//
			setSlots (self_tx_unavl_[it->channel_],
					it->frame_, frange, it->start_, it->range_, true);
		}

		//
		// confirmation
		//
		
		// if the confirmation is addressed to a node which is not in
		// our first-hop neighborhood (nor to this node itself), then
		// that node cannot transmit in the confirmed minislots on all channels
		// and we cannot receive in the confirmed minislots on the
		// specified channel
		if ( it->fromRequester_ == true &&
				! mac_->topology()->neighbors (it->nodeId_, mac_->nodeId()) ) {

			// convert the <frame, persistence> pair to the actual <frame, range>
			unsigned int fstart;   // start frame number
			unsigned int frange;   // frame range
			realPersistence (it->frame_, it->persistence_, fstart, frange);

			// set the minislots as unavailable for reception on all channels
			for ( unsigned int ch = 0 ; ch < mac_->nchannels() ; ch++ ) {
				setSlots (neigh_tx_unavl_[ndx][ch], fstart, frange,
						it->start_, it->range_, true);
			}

			// set the minislots as unavailable for reception at this node
			setSlots (self_rx_unavl_[it->channel_], fstart, frange,
					it->start_, it->range_, true);
		}

		// if the confirmation is addressed to this node, then update
		// the counter of the incoming confirmed bandwidth 
		// and set the (only) radio to listen to the confirmed channel
		if ( it->fromRequester_ == true && it->nodeId_ == mac_->nodeId() ) {

			// get the local identifier of the node who sent this MSH-DSCH
			unsigned int ndx = mac_->neigh2ndx (dsch->src());

			// get number of frames over which the confirmation spans
			// we assume that the persistence_ is not 'forever'
			// we assume that bandwidth requests are not canceled
			unsigned int frange = WimshMshDsch::pers2frames (it->persistence_);

			// listen to the specified channel in the confirmed set of slots
			setSlots (channel_, it->frame_, frange,
					it->start_, it->range_, it->channel_);

			// update the number of bytes confirmed
			neigh_[ndx].cnf_in_ +=
				frange * mac_->slots2bytes (ndx, it->range_, true);
			Stat::put ( "wimsh_cnf_in", mac_->index(),
				frange *	mac_->slots2bytes (ndx, it->range_, true) );
		}
	}

}

void
WimshBwManagerFairRR::rcvAvailabilities (WimshMshDsch* dsch)
{
	// get the list of availabilities
	std::list<WimshMshDsch::AvlIE>& avl = dsch->avl();

	// get the index of the neighbor that sent this MSH-DSCH message
	unsigned int ndx = mac_->neigh2ndx (dsch->src());

	std::list<WimshMshDsch::AvlIE>::iterator it;
	for ( it = avl.begin() ; it != avl.end() ; ++it ) {

		// we ignore unavailabilities to receive
		if ( it->direction_ == WimshMshDsch::TX_ONLY ) continue;

		// convert the <frame, persistence> pair to the actual <frame, range>
		unsigned int fstart;   // start frame number
		unsigned int frange;   // frame range
		realPersistence (it->frame_, it->persistence_, fstart, frange);

		// mark the minislots
		setSlots (neigh_tx_unavl_[ndx][it->channel_],
				fstart, frange, it->start_, it->range_,
				( it->direction_ == WimshMshDsch::AVAILABLE ) ? false : true);
	}
}

void
WimshBwManagerFairRR::rcvRequests (WimshMshDsch* dsch)
{
	// get the list of bandwidth requests
	std::list<WimshMshDsch::ReqIE>& req = dsch->req();

	// get the index of the neighbor who sent this MSH-DSCH message
	unsigned int ndx = mac_->neigh2ndx (dsch->src());

	std::list<WimshMshDsch::ReqIE>::iterator it;
	for ( it = req.begin() ; it != req.end() ; ++it ) {

		// if this request is not addressed to this node, ignore it
		if ( it->nodeId_ != mac_->nodeId() ) continue;

		// indicate that there may be a new flow to the weight manager
		wm_.flow (ndx, wimax::IN);
		
		// number of bytes requested
		unsigned requested =
			  WimshMshDsch::pers2frames (it->persistence_)
			* mac_->slots2bytes (ndx, it->level_, false);

		// otherwise, update the status of the req_in_, in bytes
		// we assume that the persistence_ is not 'forever'
		// we assume that bandwidth requests are not canceled
		neigh_[ndx].req_in_ += requested;
		Stat::put ("wimsh_req_in", mac_->index(), requested);

		// if this node is not already in the active list, add it
		if ( neigh_[ndx].req_in_ > neigh_[ndx].gnt_in_ &&
				! activeList_.find (wimax::LinkId(ndx, wimax::IN)) )
			activeList_.insert (wimax::LinkId(ndx, wimax::IN));
	}
}

void
WimshBwManagerFairRR::schedule (WimshMshDsch* dsch)
{
	if ( WimaxDebug::trace("WBWM::schedule") ) {
		fprintf (stderr, "%.9f WBWM::schedule   [%d]\n", NOW, mac_->nodeId());
		printDataStructures (stderr);
	}

   // schedule availabilities into the MSH-DSCH message
   if ( avlAdvertise_ ) availabilities (dsch);

   // confirm granted minislot ranges into the MSH-DSCH message
   confirm (dsch);

	// regrant unconfirmed bandwidth
	if ( regrantEnabled_ ) regrant (dsch);

   // add bandwidth grants and requests into the MSH-DSCH message
   requestGrant (dsch);
}

void
WimshBwManagerFairRR::availabilities (WimshMshDsch* dsch)
{
	// add as many availabilities as possible

	while ( ! availabilities_.empty() ) {

		// if there is not enough space to add an availability, stop now
		if ( dsch->remaining() < WimshMshDsch::AvlIE::size() ) break;
		WimshMshDsch::AvlIE avl = availabilities_.front();
		availabilities_.pop_front();

		// schedule the availability, unless it contains stale information
		// we assume that the persistence is not 'forever'
		// we ignore cancellations (ie. persistence = 'cancel')
		if ( mac_->frame() <=
				avl.frame_ + WimshMshDsch::pers2frames (avl.persistence_) ) {
			dsch->add (avl);
		}
	}
}

void
WimshBwManagerFairRR::requestGrant (WimshMshDsch* dsch)
{
	// get my handshake time, used to compute the grant horizon
	unsigned int Hslf = handshake (mac_->nodeId());

	//
	// we do not want to add request IE directly to the MSH-DSCH message
	// because doing so may unnecessarily increase the overhead (i.e.
	// the MSH-DSCH size) and the approximation error to bytes into
	// minislots multiples
	//
	// thus, we keep a counter of the bytes that we want to request
	// and an explicit variable which stores the remaining amount of
	// bytes in the MSH-DSCH message. The latter is incremented only
	// when a request to a new neighbor is added (checked via the
	// neighReq array)
	//
	// to do so, we also need to compute the maximum amount of bytes
	// that can be requested by a neighbor, which depends on the
	// burst profile currently used. If the amount of bytes requested
	// overflows this value, then we immediately add a new request
	// IE to the MSH-DSCH and update the remaining number of bytes
	// accordingly
	//

	// number of bytes that can still be allocated into the MSH-DSCH message
	unsigned int reqIeOccupancy = 0;

	// the i-th element is true if we requested bandwidth to neighbor i
	std::vector<bool> neighReq (mac_->nneighs(), false);

	// the i-th element stores the number of bytes requested to neighbor i
	std::vector<unsigned int> neighReqBytes (mac_->nneighs(), 0);

	// the i-th element stores the maximum number of bytes that can be
	// requested to neighbor i in a single IE
	std::vector<unsigned int> neighReqMax (mac_->nneighs());
	for ( unsigned int i = 0 ; i < mac_->nneighs() ; i++ ) {
		neighReqMax[i] =
			  mac_->phyMib()->slotPerFrame()
			* mac_->phyMib()->symPerSlot ()
			* WimshMshDsch::pers2frames (WimshMshDsch::FRAME128)
			* mac_->alpha (i);
	}

	// create an array of grantFitDescriptor's to be used during the
	// grant procedure to keep the current channel/frame/slot to analyze
	std::vector<grantFitDesc> grantFitStatus (mac_->nneighs());

	//
	// request bandwidth on a round robin manner
	// stop when one of the following occurs:
	// 1. the active list is empty
	// 2. there is not enough room in the MSH-DSCH to add a request/grant
	// 3. none of the links (both input and output) are eligible for service
	// 4. the round robin iterator points to a link descriptor
	//    whose deficit cannot be incremented, because doing so would
	//    overflow the maximum deficit amount, if specified
	//
	// to check 3. we use an integer number storing the index of the
	// first neighbor that was not eligible for service; if an eligible
	// link is encountered by the round-robin pointer through its walk,
	// that variable is set as not valid, to indicate that at least one link
	// is eligible
	//
	// NOTE: condition 4. is only enabled if fairGrant_ is trued
	//       otherwise, the maxDeficit_ acts as a threshold (if > 0)
	//
	unsigned int ineligible = 0;    // only meaningful with ineligibleValid true
	bool ineligibleValid = false;

	// update the deadlock detection timer
	if ( deficitOverflow_ ) ++ddTimer_;

	while ( ! activeList_.empty() ) {                                // 1.

		// get current link information
		const unsigned int ndx         = activeList_.current().ndx_;  // index
		const unsigned int dst         = mac_->ndx2neigh (ndx);       // NodeID
		const wimax::LinkDirection dir = activeList_.current().dir_;  // direction

		// check if there are not any more eligible links
		if ( ineligibleValid && ineligible == ndx ) break;                 // 3.

		//----------------------------------------------//
		// if this is an input link, we GRANT bandwidth //
		//----------------------------------------------//

		if ( dir == wimax::IN ) {

			// stop if there is not enough room in this MSH-DSCH to add a grant
			if ( dsch->remaining() - reqIeOccupancy < WimshMshDsch::GntIE::size() )
				break;                                                       // 2.

			// get the handshake time of the neighbor currently served
			unsigned int Hdst = handshake (dst);

			// alias for the deficit counter and other variables
			unsigned int& deficit   = neigh_[ndx].def_in_;
			unsigned int& granted   = neigh_[ndx].gnt_in_;
			unsigned int& requested = neigh_[ndx].req_in_;

			// update the deficit counter, unless we are resuming last round
			if ( ! deficitOverflow_ ) deficit += quantum (ndx, wimax::IN);
			else deficitOverflow_ = false;

			// the deficit counter is smaller than the maxDeficit_ threshold
			deficit =
				( ! fairGrant_ && maxDeficit_ > 0 && deficit > maxDeficit_ )
				? maxDeficit_ : deficit;

			// get the number of pending minislots
			unsigned int pending = requested - granted;

			// the deficit counter is smaller than the number of pending minislots
			deficit = ( deficit > pending ) ? pending : deficit;

			//
			// grant until one of the following occurs:
			// a. there is not any more spare room in the MSH-DSCH message
			// b. all the pending bytes have been made up
			// c. it is not possible to grant bandwidth over the time horizon
			//
			while ( dsch->remaining() - reqIeOccupancy
					       >= WimshMshDsch::GntIE::size() &&                // a.
					deficit > 0 ) {                                         // b.

				// get a new grant information element
				WimshMshDsch::GntIE gnt;

				// grant up to 'deficit' bytes within the time horizon
				gnt = grantFit (ndx,
						deficit,                         // max number of bytes
						mac_->frame() + Hdst,            // first eligible frame
                  mac_->frame() + 2 * Hdst + Hslf, // last eligible frame
						grantFitStatus[ndx]);            // current grant status

				// if the minislot range is empty, then it was not possible
				// to grant bandwidth to this node => break from this loop
				if ( gnt.range_ == 0 ) break;                              // c.

				// collect the average grant size, in minislots
				Stat::put ("wimsh_gnt_size", mac_->index(), gnt.range_);

				// add the grant to the MSH-DSCH message
				dsch->add (gnt);

				// convert the grant's persistence into the number of frames
				unsigned int frange = WimshMshDsch::pers2frames (gnt.persistence_);

				// number of bytes granted
				unsigned int bgnt =
					frange * mac_->slots2bytes (ndx, gnt.range_, true);

				// update the number of bytes still needed
				// since bandwidth is granted in terms of minislots, then it
				// is possible that are granted more bytes than needed
				// in this case, we do not count the surplus allocation
				// and just reset the needed variable to zero
				deficit = ( deficit > bgnt ) ? ( deficit - bgnt ) : 0;

				// update the granted counter
				granted += bgnt;
				Stat::put ("wimsh_gnt_in", mac_->index(), bgnt);

				// set the granted slots as unavailable for reception
				setSlots (busy_, gnt.frame_, frange,
						gnt.start_, gnt.range_, true);
			}

			// if fairGrant_ in enabled and the deficit overflows
			// the maximum allowed deficit, then we exit immediately from
			// the request/grant process
			if ( fairGrant_ && ( deficit > maxDeficit_ ) ) {
				deficitOverflow_ = true;
				break;
			}

			// if needed is not zero, then this link has not been granted
			// as much bandwidth as it was entitled => mark as ineligible
			if ( ! ineligibleValid && deficit > 0 ) {
				// otherwise, we just set this queue an ineligible for service
				ineligibleValid = true;
				ineligible = ndx;
			}

			// if we granted as much as the node requested, remove this element
			// if ( granted == requested ) {  // :XXX: check this
			if ( granted >= requested ) {
				deficit = 0;
				activeList_.erase();

			// otherwise, move the active list pointer to the next element
			} else {
				activeList_.move ();
			}

		//-------------------------------------------------//
		// if this is an output link, we REQUEST bandwidth //
		//-------------------------------------------------//

		} else {   // dir == wimax::OUT

			// alias for the deficit counter and the backlog
			unsigned int& deficit   = neigh_[ndx].def_out_;
			unsigned int& backlog   = neigh_[ndx].backlog_;
			unsigned int& confirmed = neigh_[ndx].cnf_out_;
			unsigned int& requested = neigh_[ndx].req_out_;
			unsigned int& granted   = neigh_[ndx].gnt_out_;

			/*  XXX
			// number of pending bytes (ie. requested but not confirmed)
			unsigned int pending =
				(requested > confirmed ) ? ( requested - confirmed ) : 0;
			*/

			// number of pending bytes (ie. backlogged but not requested)
			unsigned int pending =
				( backlog > requested ) ? ( backlog - requested ) : 0;

			// number of ungranted bytes (ie. requested but not granted)
			unsigned int ungranted =
				( requested > granted ) ? ( requested - granted ) : 0;

			// stop if there is not enough room in this MSH-DSCH to add a request
			if ( dsch->remaining() - reqIeOccupancy < WimshMshDsch::ReqIE::size() )
				break;                                                        // 2.

			// update the deficit counter, unless we are resuming last round
			if ( ! deficitOverflow_ ) deficit += quantum (ndx, wimax::OUT);
			else deficitOverflow_ = false;

			// the deficit counter is bounded by the maxDeficit_ value
			deficit =
				( ! fairRequest_ && maxDeficit_ > 0 && deficit > maxDeficit_ )
				? maxDeficit_ : deficit;

			// the deficit counter is bounded by the backlog
			// deficit = ( deficit > backlog ) ? backlog : deficit;
			deficit = ( deficit > pending ) ? pending : deficit;    // XXX

			// if there is too much bandwidth pending, do not request anymore
			// if ( maxBacklog_ > 0 && pending > maxBacklog_ ) {
			if ( maxBacklog_ > 0 && ungranted > maxBacklog_ ) {// XXX

				// if the deadlock timer is enabled and deadlock is detected
				// when we reset the deficit of the current descriptor
				// and move it to the end of the active list
				if ( ddTimeout_ > 0 && ddTimer_ >= ddTimeout_ ) {

					// :XXX: debug
					Stat::put ("wimsh_dd_timeout", mac_->index(), 1.0);

					// reset deadlock detection timer status
					ddTimer_ = 0;
					deficitOverflow_ = false;

					// reset queue status variables
					// backlog += ( requested > confirmed ) ? requested - confirmed : 0; // XXX
					granted   = 0;
					deficit   = 0;
					confirmed = 0;
					requested = 0;
				}

				// if fairRequest_ is enabled and the deficit overflows
				// the maximum allowed deficit, then we exit immediately from
				// the request/grant process
				if ( fairRequest_ && deficit > maxDeficit_ ) {
					deficitOverflow_ = true;
					break;
				}

				// otherwise we mark the link as ineligible and skip it
				if ( ! ineligibleValid ) {
					ineligibleValid = true;
					ineligible = ndx;
				}

				// move the round-robin pointer to the next element
				activeList_.move();
				continue;
			}

			// if we are here, when we have to stop the deadlock detection timer
			if ( ddTimeout_ > 0 ) ddTimer_ = 0;

			// if we did not send a bandwidth request to this neighbor,
			// increase the occupancy of request IEs
			if ( neighReq[ndx] == false )
				reqIeOccupancy += WimshMshDsch::ReqIE::size();

			// in any case, we are now sending a bandwidth request to it
			neighReq[ndx] = true;

			// update the number of bytes that we are requesting
			neighReqBytes[ndx] += deficit;

			// the only purpose of the loop below is to add request IE
			// for bandwidth requests that overflow the maximum amount
			// of bytes that can be requested in a single request IE

			while ( neighReqBytes[ndx] > neighReqMax[ndx] ) {
				
				// create a request IE and
				// fill the level and persistence fields of the request IE
				// with the maximum amount that can be requested
				WimshMshDsch::ReqIE ie;
				ie.nodeId_ = mac_->ndx2neigh (ndx);
				ie.level_ = WimshMshDsch::FRAME128;

				// update the number of bytes still to be requested
				neighReqBytes[ndx] -= neighReqMax[ndx];
				
				// insert the IE into the MSH-DSCH message
				dsch->add (ie);
			}

			// update the requested counter
			requested += deficit;
			Stat::put ("wimsh_req_out", mac_->index(), deficit);

			// update the backlog value
			// backlog = ( backlog > deficit ) ? ( backlog - deficit ) : 0;

			// reset the deficit counter
			deficit = 0;

			// if there are no pending requests, remove the node
			// from the active list
			if ( pending == 0 ) {    // XXX
			// if ( backlog == 0 ) {
				// confirmed = requested = granted = 0; // XXX
				activeList_.erase();

			// otherwise, move the active list pointer to the next element
			} else {
				activeList_.move();
			}
		}

	}

	// add to the MSH-DSCH message the request IEs that have been
	// accounted for during the request/grant process above

	for ( unsigned int ndx = 0 ; ndx < mac_->nneighs() ; ndx++ ) {
		if ( neighReqBytes[ndx] == 0 ) continue;

		// create a request IE
		WimshMshDsch::ReqIE ie;
		ie.nodeId_ = mac_->ndx2neigh (ndx);

		// fill the level and persistence fields of the request IE
		// we are sure that the number of bytes never overflows the
		// maximum number of bytes that can be requested in a single
		// IE, since this case has been managed during the request/grant
		// process itself
		WimshMshDsch::slots2level (
				mac_->phyMib()->slotPerFrame(),
				mac_->bytes2slots (ndx, neighReqBytes[ndx], false),
				ie.level_, ie.persistence_);

		// insert the IE into the MSH-DSCH message
		dsch->add (ie);
	}
}

void
WimshBwManagerFairRR::regrant (WimshMshDsch* dsch)
{
	// get my handshake time
	unsigned int Hslf = handshake (mac_->nodeId());

	//
	// compute the unconfirmed bytes for each neighbor, if any
	// and we add to the regntActiveList_ data structure the index of all
	// neighbors that have unconfirmed minislots, unless the node already
	// is in the active list (the only for a node to be in the list already
	// is that it was not possible to regrant its number of unconfirmed
	// minislots during the previous round)
	//
	std::vector<unsigned int> unconfBytes;
	unconfBytes.resize (mac_->nneighs());

	for ( unsigned int i = 0 ; i < mac_->nneighs() ; i++ ) {
		// note that
		// - it should be always gnt_in_[i] >= cnf_in_[i], however it happens
		//   that sometimes it is not :TODO: check why
		// - if we did not receive a confirmation yet, then we set
		//   the number of unconfirmed minislots to zero, since our
		//   neighbor did not have time to respond, and hence we
		//   do not want to grant it new bandwidth now
		unconfBytes[i] =
			( neigh_[i].rcvCnf_ && neigh_[i].gnt_in_ >= neigh_[i].cnf_in_ )
			? neigh_[i].gnt_in_ - neigh_[i].cnf_in_ : 0;

		if ( unconfBytes[i] > 0 && ! regntActiveList_.find (i) )
			regntActiveList_.insert (i);

		// clear the confirmation received flag
		neigh_[i].rcvCnf_ = false;
	}

	// create an array of grantFitDescriptor's to be used during the
	// grant procedure to keep the current channel/frame/slot to analyze
	std::vector<grantFitDesc> grantFitStatus (mac_->nneighs());

	//
	// regrant as much as possible unconfirmed bandwidth 
	//
	// exit if there is not enough room in MSH-DSCH message to grant bw
	// or if there are not anymore unconfirmed slots
	//

	while ( dsch->remaining() >= WimshMshDsch::GntIE::size() &&
			  ! regntActiveList_.empty() ) {

		// get the index of the current node
		const unsigned int ndx = regntActiveList_.current();

		// get the NodeID of the current node
		const WimaxNodeId dst = mac_->ndx2neigh (ndx);

		// compute the handshake time of the neighbor currently served
		unsigned int Hdst = handshake (dst);

		// create a new grant information element
		WimshMshDsch::GntIE gnt;

		// select the regrant horizon
		unsigned int horizonLow  = mac_->frame();
		unsigned int horizonHigh;
		if ( sameRegrantHorizon_ ) {
			horizonLow += Hdst;
			horizonHigh = horizonLow + Hdst + Hslf;
		} else {
			horizonLow += 2 * Hdst + Hslf + regrantOffset_;
			horizonHigh = horizonLow + ( Hdst + Hslf ) * regrantDuration_;
		}

		// regrant unconfirmed bandwidth
		gnt = grantFit (ndx,
				unconfBytes[ndx],          // max bytes
				horizonLow,                // 1st frame
				horizonHigh,               // Lst frame
				grantFitStatus[ndx]);      // fit status

		// if the minislot range is empty, then it was not possible
		// to grant bandwidth to this node
		if ( gnt.range_ == 0 ) {
			// to keep some fairness, we may want to stop
			// granting bandwidth as soon as we cannot serve a node
			if ( fairRegrant_ ) break;

			// otherwise, we just remove this node from the active list
			regntActiveList_.erase ();
			continue;
		}

		// convert the grant's persistence into the number of frames
		unsigned int frange = WimshMshDsch::pers2frames (gnt.persistence_);

		// add the grant to the MSH-DSCH message
		dsch->add (gnt);

		// update the number of unconfirmed bytes still pending, if any
		unconfBytes[ndx] -= frange * mac_->slots2bytes (ndx, gnt.range_, true);
		Stat::put ( "wimsh_regnt_in", mac_->index(),
			frange * mac_->slots2bytes (ndx, gnt.range_, true) );

		// remove the current element if there are not anymore unconfirmed slots
		if ( unconfBytes[ndx] == 0 ) {
			regntActiveList_.erase();
			continue;
		}

		// set the granted slots as unavailable for reception
		setSlots (busy_, gnt.frame_, frange,
				gnt.start_, gnt.range_, true);

		// advance the current pointer to the next element in the active list
		regntActiveList_.move();
	}
}

void
WimshBwManagerFairRR::confirm (WimshMshDsch* dsch)
{
	// confirm as many grants as possible

	while ( ! unconfirmed_.empty() ) {

		// if there is not enough space to add a confirmation to this
		// grant, then stop now
		if ( dsch->remaining() < WimshMshDsch::GntIE::size() ) break;

		// get the first unconfirmed grant
		WimshMshDsch::GntIE gnt = unconfirmed_.front();
		unconfirmed_.pop_front();

		//
		// we assume that the persistence is not 'forever'
		// we ignore cancellations (ie. persistence = 'cancel')
		//

		// get the start frame number and range
		unsigned int fstart;  // frame start number
		unsigned int frange;  // frame range

		// convert the <frame, persistence> into the actual values
		realPersistence (gnt.frame_, gnt.persistence_, fstart, frange);

		// get the start minislot number and range
		unsigned int mstart = gnt.start_;  // minislot start number
		unsigned int mrange = gnt.range_;  // minislot range

		// granted/confirmed bytes
		unsigned int confirmed = 0;

		// get the index of this neighbor
		unsigned int ndx = mac_->neigh2ndx (gnt.nodeId_);

		// confirm as many slots as possible
		while ( dsch->remaining() >= WimshMshDsch::GntIE::size() ) {

			// find the first block of available slots
			confFit (fstart, frange, mstart, mrange, gnt);

			// if there are not anymore blocks, terminate
			if ( gnt.range_ == 0 ) break;

			// collect the average confirmed grant size, in minislots
			Stat::put ("wimsh_cnf_size", mac_->index(), gnt.range_);

			// schedule the grant as a confirmation
			dsch->add (gnt);

			// convert to the actual values of <frame, range>
			unsigned int fs;  // frame start
			unsigned int fr;  // frame range
			realPersistence (gnt.frame_, gnt.persistence_, fs, fr);

			// compute the number bytes confirmed
			confirmed += fr * mac_->slots2bytes (ndx, gnt.range_, true);

			// mark the minislots
			setSlots (busy_, fs, fr, gnt.start_, gnt.range_, true);
			setSlots (grants_, fs, fr, gnt.start_, gnt.range_, true);
			setSlots (dst_, fs, fr, gnt.start_, gnt.range_, gnt.nodeId_);
			setSlots (channel_, fs, fr, gnt.start_, gnt.range_, gnt.channel_);
		}

		// update the number of bytes confirmed
		neigh_[ndx].cnf_out_ += confirmed;
		Stat::put ("wimsh_cnf_out", mac_->index(), confirmed);

		// we enforce the number of granted bytes to be smaller than
		// that of granted bytes
		neigh_[ndx].cnf_out_ =
			( neigh_[ndx].cnf_out_ < neigh_[ndx].gnt_out_ ) ?
			neigh_[ndx].cnf_out_ : neigh_[ndx].gnt_out_;              // XXX

		// remove the confirmed bytes from the data structures
		if ( neigh_[ndx].req_out_ < neigh_[ndx].cnf_out_ ) abort();  // XXX
		neigh_[ndx].gnt_out_ -= neigh_[ndx].cnf_out_;                // XXX
		neigh_[ndx].req_out_ -= neigh_[ndx].cnf_out_;                // XXX
		neigh_[ndx].cnf_out_ = 0;                                    // XXX

	} // for each unconfirmed grant
}

void
WimshBwManagerFairRR::invalidate (unsigned int F)
{
	// compute the number of slots in the current frame
	// that could have been used to transmit date for measurement purposes
	const unsigned int unused =
		mac_->phyMib()->slotPerFrame() - busy_[F].count();
	Stat::put ("wimsh_unused_a", mac_->index(), unused);
	Stat::put ("wimsh_unused_d", mac_->index(), unused);

	if ( WimaxDebug::trace("WBWM::invalidate") ) fprintf (stderr,
			"%.9f WBWM::invalidate [%d] unused %d\n", NOW, mac_->nodeId(), unused);

	// reset to default values all data structures of the last frame
	for ( unsigned int ngh = 0 ; ngh < mac_->nneighs() ; ngh++) {
		for ( unsigned int ch = 0 ; ch < mac_->nchannels() ; ch++ ) {
			neigh_tx_unavl_[ngh][ch][F].reset();
		}
	}
	for ( unsigned int ch = 0 ; ch < mac_->nchannels() ; ch++ ) {
		self_rx_unavl_[ch][F].reset();
		self_tx_unavl_[ch][F].reset();
	}
	busy_[F].reset();
	unconfirmedSlots_[F].reset();
	WimshBwManager::invalidate (F);
}

void
WimshBwManagerFairRR::realPersistence (
		unsigned int start, WimshMshDsch::Persistence pers,
		unsigned int& realStart, unsigned int& range)
{
	range = WimshMshDsch::pers2frames (pers);

	// if the start frame of the grant is smaller than the
	// current frame number, then some (or all) the information is stale
	range -= ( start >= mac_->frame() ) ? 0 : ( mac_->frame() - start );

	if ( range < 0 ) range = 0;

	// the start time is the largest value between start and the current frame
	realStart = ( start >= mac_->frame() ) ? start : mac_->frame();
}

//
// :TODO: this function can be optimized significantly: at each iteration
//        the grant search restarts from the beginning
//
WimshMshDsch::GntIE
WimshBwManagerFairRR::grantFit (
		unsigned int ndx, unsigned int bytes,
		unsigned int minFrame, unsigned int maxFrame,
		grantFitDesc& status)
{
	// new grant to be returned (and then added to the MSH-DSCH)
	WimshMshDsch::GntIE gnt;
	gnt.nodeId_ = mac_->ndx2neigh (ndx);

	// number of minislots per frame
	unsigned int N = mac_->phyMib()->slotPerFrame();

	// number of channels
	unsigned int C = mac_->nchannels();

	// :TODO: ouch!
	/*
	
	// if this is the first time that this neighbor is granted bandwidth,
	// then we may have to pick the first channel/frame/slot randomly
	if ( status.first ) {

		// set the channel
		status.chSum = mac_->nchannels();
		if ( grantFitRandomChannel_ ) {
			status.ch = grantFitRng.uniform ((int)status.chSum);
		} else {
			status.ch = 0;
		}

		// set the first flag to false, that further searches during the
		// same request/grant process will start from the last allocated slot
		status.first = false;
	}

	// for each channel
	// for ( unsigned int ch = 0 ; ch < mac_->nchannels() ; ch++ ) {
	while ( status.chSum > 0 ) {

		// for each frame in the time window
		for ( unsigned int f = minFrame ; f <= maxFrame ; f++ ) {
			unsigned int F = f % HORIZON;

			// get a bitset which represents the grant unavailabilities
			std::bitset<MAX_SLOTS> map =
			  unconfirmedSlots_[F] | busy_[F] |
			  self_rx_unavl_[status.ch][F] | neigh_tx_unavl_[ndx][status.ch][F];

			// for each minislot in the current frame
			for ( unsigned int s = 0 ; s < N ; s++ ) {
				
				// as soon as a free minislot is found, start the grant allocation
				if ( map[s] == false ) {
					gnt.frame_ = f;
					gnt.start_ = s;
					gnt.persistence_ = WimshMshDsch::FRAME1;
					gnt.fromRequester_ = false;
					gnt.channel_ = status.ch;

					// search for the largest minislot range
					for ( ; s < N &&
							  map[s] == false &&
							  ( s - gnt.start_ ) < maxSlots; s++ ) { }

					gnt.range_ = s - gnt.start_;
					return gnt;
				}
			} // for each minislot
		} // for each frame

		// update the grantFit channel status variables
		status.ch = ( status.ch + 1 < mac_->nchannels() ) ? ( status.ch + 1 ) : 0;
		status.chSum--;
	} // for each channel

	*/


	// for each frame in the time window
	for ( unsigned int f = minFrame ; f <= maxFrame ; f++ ) {
		// set the actual frame number within the frame horizon
		unsigned int F = f % HORIZON;

		// pick up a random channel, if required
		unsigned int ch = 0;
		if ( grantFitRandomChannel_ ) ch = grantFitRng.uniform ((int)C);

		// for each channel
		for ( unsigned int c = 0 ; c < C ; c++ ) {

			// get a bitset which represents the grant unavailabilities
			std::bitset<MAX_SLOTS> map =
			  unconfirmedSlots_[F] | busy_[F] |
			  self_rx_unavl_[ch][F] | neigh_tx_unavl_[ndx][ch][F];

			// for each minislot in the current frame
			for ( unsigned int s = 0 ; s < N ; s++ ) {
				
				// as soon as a free minislot is found, start the grant allocation
				if ( map[s] == false ) {
					gnt.frame_ = f;
					gnt.start_ = s;
					gnt.persistence_ = WimshMshDsch::FRAME1;
					gnt.fromRequester_ = false;
					gnt.channel_ = ch;

					// search for the largest minislot range
					unsigned int maxSlots = mac_->bytes2slots (ndx, bytes, true);
					for ( ; s < N && map[s] == false &&
						( s - gnt.start_ ) < maxSlots ; s++ ) { }

					gnt.range_ = s - gnt.start_;

					// check the minimum number of OFDM symbols per grant
					// unless the number of slots requested is smaller than that
					unsigned int symbols =
						  gnt.range_ * mac_->phyMib()->symPerSlot()
						- mac_->phyMib()->symShortPreamble();

					if ( gnt.range_ != maxSlots && symbols < minGrant_ )
						continue;

					return gnt;
				}
			} // for each minislot

			// set the actual channel number
			ch = ( ch + 1 ) % C;

		} // for each channel
	} // for each frame

	// if we reach this point, then it is not possible to grant bandwidth
	gnt.range_ = 0;  // in this case, the other fields are not meaningful
	return gnt;
}

//
// :TODO: this function can be optimized significantly: at each iteration
//        the grant search restarts from the beginning
//
void
WimshBwManagerFairRR::confFit (
		unsigned int fstart, unsigned int frange,
		unsigned int mstart, unsigned int mrange,
		WimshMshDsch::GntIE& gnt)
{
	// for each frame in the time window
	for ( unsigned int f = fstart ; f < fstart + frange ; f++ ) {
		unsigned int F = f % HORIZON;

		const std::bitset<MAX_SLOTS> map =
			busy_[F] | self_tx_unavl_[gnt.channel_][F];

		// for each minislot in the current frame
		for ( unsigned int s = mstart ; s < mstart + mrange ; s++ ) {
			
			// as soon as a free minislot is found, start the grant allocation
			if ( map[s] == false ) {
				gnt.frame_ = f;
				gnt.start_ = s;
				gnt.persistence_ = WimshMshDsch::FRAME1;

				// search for the largest minislot range
				for ( ; s < ( mstart + mrange ) && map[s] == false ; s++ ) { }

				gnt.range_ = s - gnt.start_;
				return;
			}
		} // for each minislot
	} // for each frame

	// if we reach this point, then it is not possible to grant bandwidth
	gnt.range_ = 0;  // in this case, the other fields are not meaningful
}

void
WimshBwManagerFairRR::backlog (WimaxNodeId src, WimaxNodeId dst,
		unsigned char prio, WimaxNodeId nexthop, unsigned int bytes)
{
	// add the current flow to the weight manager data structure
	wm_.flow (src, dst, prio, mac_->neigh2ndx(nexthop), wimax::OUT);

	// get the index of the nexthop neighbor (ie. the link identifier)
	const unsigned int ndx = mac_->neigh2ndx(nexthop);

	// add the amount of received bytes to the backlog of this output link
	neigh_[ndx].backlog_ += bytes;

	// if the output link is not alreay in the active list, push it in
	if ( ! activeList_.find (wimax::LinkId(ndx, wimax::OUT)) )
		activeList_.insert (wimax::LinkId(ndx, wimax::OUT));
}

void
WimshBwManagerFairRR::backlog (WimaxNodeId nexthop, unsigned int bytes)
{
	// add the current flow to the weight manager data structure
	wm_.flow (mac_->neigh2ndx(nexthop), wimax::OUT);

	// get the index of the nexthop neighbor (ie. the link identifier)
	const unsigned int ndx = mac_->neigh2ndx(nexthop);

	// add the amount of received bytes to the backlog of this output link
	neigh_[ndx].backlog_ += bytes;

	// if the output link is not already in the active list, push it in
	if ( ! activeList_.find (wimax::LinkId(ndx, wimax::OUT)) )
		activeList_.insert (wimax::LinkId(ndx, wimax::OUT));
}

void
WimshBwManagerFairRR::sent (WimaxNodeId nexthop, unsigned int bytes)
{
	// get the index of the nexthop neighbor (ie. the link identifier)
	const unsigned int ndx = mac_->neigh2ndx(nexthop);

	// remove the amount of received bytes from the backlog of this output link
	neigh_[ndx].backlog_ -= bytes;
}

void
WimshBwManagerFairRR::printDataStructures (FILE* os)
{
	fprintf (os, "\tDATA STRUCTURES\n");
	for ( unsigned int i = 0 ; i < mac_->nneighs() ; i++ ) {
		fprintf (os, "\t%d in  %d req %d gnt %d cnf %d def %d w %f q %d\n",
				i, mac_->ndx2neigh(i),
				neigh_[i].req_in_, neigh_[i].gnt_in_, neigh_[i].cnf_in_,
				neigh_[i].def_in_, wm_.weight (i, wimax::IN),
				quantum (i, wimax::IN) );
		fprintf (os, "\t%d out %d req %d gnt %d cnf %d def %d w %f "
				"q %d backlog %d \n",
				i, mac_->ndx2neigh(i),
				neigh_[i].req_out_, neigh_[i].gnt_out_, neigh_[i].cnf_out_,
				neigh_[i].def_out_, wm_.weight (i, wimax::OUT),
				quantum (i, wimax::OUT), neigh_[i].backlog_);
	}
	fprintf (os, "\tREQUEST/GRANTING ACTIVE-LIST\n");
	WimaxDebug::print (activeList_, os, "\t");
	fprintf (os, "\tUNCONFIRMED ACTIVE-LIST\n");
	WimaxDebug::print (regntActiveList_, os, "\t");
}
