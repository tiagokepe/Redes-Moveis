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

#include <math.h>

#include <wimsh_phy.h>
#include <wimsh_packet.h>
#include <wimsh_channel.h>
#include <wimsh_mac.h>
#include <wimsh_topology.h>

/*
 *
 * class WimshPhyMib
 *
 */

static class WimshPhyMibClass : public TclClass {
public:
   WimshPhyMibClass() : TclClass("WimshPhyMib") {}
   TclObject* create(int, const char*const*) {
      return (new WimshPhyMib);
   }
} class_wimsh_phy_mib;

const unsigned int WimshPhyMib::alpha[]  = {
	24, 36, 48, 72, 96, 108 };

WimshPhyMib::WimshPhyMib ()
{
	frameDuration_ = 0;
	symDuration_ = 0;
	symPerFrame_ = 0;
	controlSlots_ = 0;
}

int
WimshPhyMib::command (int argc, const char*const* argv)
{
	if ( argc == 3 && strcmp (argv[1], "symDuration") == 0 ) {
		symDuration_ = 1.0e-6 * atof (argv[2]);  // in us
		return TCL_OK;
	} else if ( argc == 3 && strcmp (argv[1], "frameDuration") == 0 ) {
		frameDuration_ = 1.0e-3 * atof (argv[2]);
		return TCL_OK;
	} else if ( argc == 3 && strcmp (argv[1], "controlSlots") == 0 ) {
		controlSlots_ = (unsigned int) atoi (argv[2]);
		return TCL_OK;
	} else if ( argc == 3 && strcmp (argv[1], "cfg-interval") == 0 ) {
		cfgInterval_ = (unsigned int) atoi (argv[2]);
		return TCL_OK;
	} else if ( argc == 2 && strcmp (argv[1], "recompute") == 0 ) {
		if ( recompute() == true ) return TCL_OK;
		fprintf (stderr, "Invalid PHY parameters selection\n");
		return TCL_ERROR;
	} else if ( argc == 2 && strcmp (argv[1], "dump") == 0 ) {
		dump (stderr);
		return TCL_OK;
	}
	return TCL_ERROR;
}

void
WimshPhyMib::dump (FILE* os)
{
	fprintf (os, "frame duration           = %f ms\n"
			       "OFDM symbol duration     = %f us\n"
					 "OFDM symbols per frame   = %d\n"
					 "OFDM symbols per slot    = %d\n"
					 "slots per frame          = %d\n"
					 "control slots per frame  = %d\n"
					 "MSH-NCFG frames interval = %d\n",
					 1.0e3 * frameDuration(),
					 1.0e6 * symDuration(),
					 symPerFrame(),
					 symPerSlot(),
					 slotPerFrame(),
					 controlSlots(),
					 cfgInterval());
}

bool
WimshPhyMib::recompute ()
{
	symPerFrame_ = (unsigned int) (frameDuration_ / symDuration_);
	if ( 7 * controlSlots_ > symPerFrame_ - 1 ) return false;
	symPerSlot_ = 1 + ( symPerFrame_ - 7 * controlSlots_ - 1 ) / 256;
	slotPerFrame_ = ( symPerFrame_ - 7 * controlSlots_ ) / symPerSlot_;
	if ( slotPerFrame_ == 0 ) return false;
	return true;
}

double
WimshPhyMib::nextFrame ()
{
	return frameDuration() * ceil ( NOW / frameDuration() );
}

/*
 *
 * class WimshPhy
 *
 */

static class WimshPhyClass : public TclClass {
public:
   WimshPhyClass() : TclClass("WimshPhy") {}
   TclObject* create(int, const char*const*) {
      return (new WimshPhy);
   }
} class_wimax_phy;

WimshPhy::WimshPhy () : rxFinishTimes_ (this)
{
	channel_ = 0;
	phyMib_ = 0;
	epsilon_ = 0;
}

int
WimshPhy::command(int argc, const char*const* argv)
{
	if ( argc == 3 && strcmp (argv[1], "channel") == 0 ) {
		channel_ = (WimshChannel*) TclObject::lookup (argv[2]);
		return TCL_OK;
	} else if ( argc == 3 && strcmp (argv[1], "phymib") == 0 ) {
		phyMib_ = (WimshPhyMib*) TclObject::lookup (argv[2]);
		return TCL_OK;
	} else if ( argc == 3 && strcmp (argv[1], "mac") == 0 ) {
		mac_ = (WimshMac*) TclObject::lookup (argv[2]);
		return TCL_OK;
	} else if ( argc == 3 && strcmp (argv[1], "topology") == 0 ) {
		topology_ = (WimshTopologySimple*) TclObject::lookup (argv[2]);
		return TCL_OK;
	} else if ( argc == 3 && strcmp (argv[1], "epsilon") == 0 ) {
		epsilon_ = 1.0e-6 * atof (argv[2]);   // in us
		if ( epsilon_ < 0 ) {
			fprintf (stderr, "the epsilon value '%f' is not valid. "
					"Choose a positive (small) value\n", epsilon_);
			return TCL_ERROR;
		}
		return TCL_OK;
	}
	return TCL_ERROR;
}

void
WimshPhy::setMode (wimax::ChannelStatus s, WimshChannel* channel)
{
	if ( WimaxDebug::trace("WPHY::setMode") ) fprintf (stderr,
			"%.9f WPHY::setMode    [%d] phy %p channel %d mode %s\n",
			NOW, mac_->nodeId(), this, channel->uid(),
			( s == wimax::TX ) ? "TX" : ( s == wimax::RX ) ? "RX" : "NN");

	// if there are undispatched bursts to myself, we mark then as corrupted
	// in case we are switching from RX to TX or we are switching channel
	//
	// note that the descriptor is not removed from the list, since it
	// can still interfere with other bursts

	if ( ( channel && channel != channel_ ) || 
			( channel_->getMode (this) == wimax::RX && s == wimax::TX ) ) {
		std::list<BurstDesc>::iterator it;
		for ( it = rxBursts_.begin() ; it != rxBursts_.end() ; it++ ) {
			if ( gt (it->finish_, NOW) && it->burst_ != 0 ) {
				it->burst_->error () = true;
			}
		}
	}

	// if there is a channel switch, then the old channel status becomes NONE
	if ( channel && channel_ && channel != channel_ )
		channel_->setMode (this, wimax::NONE);

	// switch the channel
	if ( channel ) channel_ = channel;
	channel_->setMode (this, s);
}

void
WimshPhy::sendBurst (WimshBurst* burst)
{
	// set the transceiver to transmit mode
	channel_->setMode (this, wimax::TX);

	// compute the burst duration (in seconds) aligned to OFDM symbol boundaries
	burst->txtime() =
		phyMib_->symDuration() *
		( 1 + ( burst->size() - 1 ) / phyMib_->alpha[burst->profile()] );

	// add a physical preamble
	// for DATA bursts, a preamble consists of one OFDM symbol
	// while, it is two OFDM symbols for CONTROL messages
	if ( burst->type() == wimax::DATA ) {
		burst->txtime() += phyMib_->symShortPreamble() * phyMib_->symDuration ();
	} else if ( burst->type() == wimax::MSHDSCH ) {
		burst->txtime() += phyMib_->symShortPreamble() * phyMib_->symDuration ();
	} else if ( burst->type() == wimax::MSHNCFG ) {
		burst->txtime() += phyMib_->symShortPreamble() * phyMib_->symDuration ();
	} else if ( burst->type() == wimax::MSHNENT ) {
		burst->txtime() += phyMib_->symShortPreamble() * phyMib_->symDuration ();
	} else {
		abort();
	}

	// set the source field of the burst
	burst->source() = mac_->nodeId();

	// send the burst to channel
	channel_->recvBurst (burst);
}

void
WimshPhy::handle (WimshBurst* burst)
{
	// if there is no burst into the earliest element, just ignore it
	if ( burst == 0 ) return;

	if ( WimaxDebug::trace("WPHY::handle") ) fprintf (stderr,
			"%.9f WPHY::handle     [%d] src %d txtime %f \n", NOW, mac_->nodeId(),
			burst->source(), burst->txtime());

	// otherwise, check whether this burst can be received

	// first, find the element with that burst into the list
	std::list<BurstDesc>::iterator cur;
	for ( cur = rxBursts_.begin() ; cur != rxBursts_.end() ; ++cur ) {
		if ( cur->burst_ == burst ) break;
	}

	assert ( cur != rxBursts_.end() );
	
	// :XXX: this is debug code, remove it
	/*
	if ( cur->burst_ != burst ) {
		fprintf (stderr, "%x != %x\n", cur->burst_, burst);
		for ( cur = rxBursts_.begin() ; cur != rxBursts_.end() ; ++cur ) {
			fprintf (stderr, "src %d dst %d start %f finish %f burst %x\n",
					cur->src_, cur->dst_, cur->start_, cur->finish_,
					cur->burst_);
		}
		abort ();
	}
	*/

	// then, check all the elements into the rx lists

	std::list<BurstDesc>::iterator it;
	for ( it = rxBursts_.begin() ; it != rxBursts_.end() ; ++it ) {

		// skip the burst itself
		if ( it == cur ) continue;

		// check if the channels interfere
		if ( it->channel_ != cur->channel_ )
			continue;

		// check if the transmission times overlap
		if ( ! ( gt (it->finish_, cur->start_) &&
					gt (cur->finish_, it->start_ ) ) )
			continue;

		// check if the links interfere
		if ( ! topology_->neighbors (it->src_, mac_->nodeId()) ||
				! topology_->neighbors (cur->src_, mac_->nodeId()) )
			continue;

		// if we are here, then the current transmission collides
		cur->burst_->error() = true;
		break;
	}

	// send the burst to the MAC layer
	mac_->recvBurst (cur->burst_);
	cur->burst_ = 0;

	// check if there are elements in the list that can be removed
	// 1. find the latest undispatched start time
	// 2. remove all elements that have been dispatched before that time

	double latestStart = -1;
	for ( it = rxBursts_.begin() ; it != rxBursts_.end() ; ++it ) {
		if ( ! gt (NOW, it->finish_) )   // undispatched
			if ( latestStart < 0 || it->start_ > latestStart)
				latestStart = it->start_;

	}
	
	if ( latestStart < 0 ) {  // ie. no undispatched elements
		rxBursts_.clear ();
	} else {
		for ( it = rxBursts_.begin() ; it != rxBursts_.end() ; ) {
			if ( it->finish_ < latestStart ) {
				std::list<BurstDesc>::iterator drop = it;
				++it;
				rxBursts_.erase (drop);
			} else {
				++it;
			}
		}
	}
}

void
WimshPhy::recvBurst (WimshBurst* burst)
{
	if ( WimaxDebug::trace("WPHY::recvBurst") ) fprintf (stderr,
			"%.9f WPHY::recvBurst  [%d] phy %p src %d type %s txtime %f\n",
			NOW, mac_->nodeId(), this, burst->source(),
			( burst->type() == wimax::MSHDSCH ) ? "dsch" :
			( burst->type() == wimax::MSHNCFG ) ? "ncfg" :
			( burst->type() == wimax::MSHNENT ) ? "nent" :
			( burst->type() == wimax::DATA ) ? "data" :
			"unkn",
			burst->txtime());

	// create a new descriptor
	BurstDesc desc;
	desc.src_ = burst->source();
	desc.start_ = NOW;
	desc.finish_ = NOW + burst->txtime();
	desc.channel_ = channel_;

	// if the transmitting node is a neighbor, then store this burst
	// to pass it to the MAC layer, if it will not collide
	if ( topology_->neighbors (mac_->nodeId(), burst->source()) ) {
		desc.burst_ = new WimshBurst (*burst);
		desc.dst_ = mac_->nodeId();

	// if the transmitting node is not a neighbor, then we store
	// information on the burst to check that it will not collide
	// with forthcoming bursts, but we are not able to decode the
	// PDU burst anyway, so we do not store it
	} else {
		desc.burst_ = 0;
		desc.dst_ = topology_->nextHop (burst->source(), mac_->nodeId());
	}

	// set the timer to the nearest finish time event
	rxFinishTimes_.add (burst->txtime(), desc.burst_);

	// add the burst descriptor to the pending list
	rxBursts_.push_back (desc);
} 
