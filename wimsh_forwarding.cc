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

#include <wimsh_forwarding.h>
#include <wimsh_topology.h>
#include <wimsh_mac.h>

#include <packet.h>
#include <ip.h>

/*
 *
 * class WimshForwardingSpf
 *
 */

WimaxNodeId
WimshForwardingSpf::nextHop (WimaxSdu* sdu)
{
	return topology_->nextHop (
			mac_->nodeId(), HDR_IP(sdu->ip())->daddr());
}
