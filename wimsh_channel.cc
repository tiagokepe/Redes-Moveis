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

#include <wimsh_channel.h>
#include <wimsh_topology.h>
#include <wimsh_phy.h>
#include <wimsh_packet.h>

#include <stat.h>

static class WimshChannelClass : public TclClass {
public:
   WimshChannelClass() : TclClass("WimshChannel") {}
   TclObject* create(int, const char*const*) {
      return (new WimshChannel);
   }
} class_wimsh_channel;

WimshChannel::WimshChannel () : timer_ (this)
{
	topology_ = 0;
	errorData_ = 0;
	errorCtrl_ = 0;
	rngErrorData_ = 0;
	rngErrorCtrl_ = 0;
	uid_ = 0;
}

int
WimshChannel::command(int argc, const char*const* argv)
{
	if ( argc == 3 && strcmp (argv[1], "topology") == 0 ) {
		topology_ = (WimshTopologySimple*) TclObject::lookup(argv[2]);
		return TCL_OK;
	} else if ( argc == 3 && strcmp (argv[1], "propagation") == 0 ) {
		propagation_ = 1.0e-6 * atof (argv[2]);   // in us
		return TCL_OK;
	} else if ( argc == 3 && strcmp (argv[1], "id") == 0 ) {
		uid_ = (unsigned int) atoi (argv[2]);
		return TCL_OK;
	} else if ( argc == 4 && strcmp (argv[1], "error") == 0 ) {
		if ( strcmp(argv[2], "data") == 0 ) {
			errorData_ = atof (argv[3]);
			rngErrorData_ = new RNG;
			if ( errorData_ < 0 || errorData_ > 1.0 ) {
				fprintf (stderr, "invalid error rate '%f' for data bursts. "
						"Choose a number in [0, 1]\n", errorData_);
				return TCL_ERROR;
			}
		} else if ( strcmp(argv[2], "control") == 0 ) {
			errorCtrl_ = atof (argv[3]);
			rngErrorCtrl_ = new RNG;
			if ( errorCtrl_ < 0 || errorCtrl_ > 1.0 ) {
				fprintf (stderr, "invalid error rate '%f' for control bursts. "
						"Choose a number in [0, 1]\n", errorCtrl_);
				return TCL_ERROR;
			}
		} else {
			fprintf (stderr, "invalid error type '%s'. "
					"Choose either 'data' or 'control'\n", argv[2]);
			return TCL_ERROR;
		}
			
		return TCL_OK;
	}
	return TCL_ERROR;
}

void
WimshChannel::recvBurst (WimshBurst* burst)
{
	if ( WimaxDebug::trace("WCHN::recvBurst") ) fprintf (stderr,
			"%.9f WCHN::recvBurst  [%d] src %d type %s txtime %f\n",
			NOW, uid_, burst->source(),
			( burst->type() == wimax::MSHDSCH ) ? "dsch" :
			( burst->type() == wimax::MSHNCFG ) ? "ncfg" :
			( burst->type() == wimax::MSHNENT ) ? "nent" :
			( burst->type() == wimax::DATA ) ? "data" :"unkn", burst->txtime());

	// corrupt the burst according to the uniform r.v.'s and settings
	if ( (
				( burst->type() == wimax::MSHDSCH  ||
				  burst->type() == wimax::MSHNCFG  || 
				  burst->type() == wimax::MSHNENT ) &&
			rngErrorCtrl_ &&
			rngErrorCtrl_->uniform() < errorCtrl_ ) ||
	     ( burst->type() == wimax::DATA &&
			rngErrorData_ &&
			rngErrorData_->uniform() < errorData_ ) )
		burst->error() = true;

	if ( burst->type() == wimax::DATA ) {
		Stat::put ("wimsh_chn_data_tpt", 0, burst->size());    // all channels
		Stat::put ("wimsh_chn_data_tpt", uid_, burst->size()); // this channel
	}
	else
		Stat::put ("wimsh_chn_ctrl_tpt", uid_, burst->size());

	// dispatch this burst a propagation time later
	timer_.add (propagation_, burst);
}

void
WimshChannel::handle (WimshBurst* burst)
{
	if ( WimaxDebug::trace("WCHN::handle") ) fprintf (stderr,
			"%.9f WCHN::handle     [%d] src %d type %s txtime %f error %d\n",
			NOW, uid_, burst->source(),
			( burst->type() == wimax::MSHDSCH ) ? "dsch" :
			( burst->type() == wimax::MSHNCFG ) ? "ncfg" :
			( burst->type() == wimax::MSHNENT ) ? "nent" :
			( burst->type() == wimax::DATA ) ? "data" : "unkn",
			burst->txtime(),
			burst->error());

	std::map<WimshPhy*, wimax::ChannelStatus>::iterator it = phymap_.begin();

	// dispatch the burst to all listening PHYs
	for ( ; it != phymap_.end() ; ++it ) {
		if ( it->second == wimax::RX ) it->first->recvBurst (burst);
	}

	// destroy the burst
	delete burst;
}
