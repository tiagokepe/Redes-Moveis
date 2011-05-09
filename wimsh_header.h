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

#ifndef __NS2_WIMSH_HEADER_H
#define __NS2_WIMSH_HEADER_H

//! WiMAX Mesh CID.
struct WimaxMeshCid {
private:
	//! Destination neighbor NodeID, instead of the LinkID.
	/*!
	  IRL the CID does NOT contain the destination neighbor NodeID. On the
	  other hand, it contains the LinkID, which has been negotiated during
	  the initial network configuration (via MSH-NCFG messages). During
	  normal operation, then, each node uses the mesh subheader + LinkID
	  information to derive the destination NodeID.
	  
	  Since, we do not simulate network configuration, there is no point in
	  using the LinkID, which only incurs additional simulation overhead
	  due to the lookup of (LinkID, src-NodeID) -> dst-NodeID.

	  Management broadcast messages IRL are identified through a 0xFF LinkID.
	  We simply ignore this, and assume that all management CID are broadcast.
	  This is safe, since the only management message that we simulate is
	  MSH-DSCH, which is broadcast.
	  */
	WimaxNodeId dst_;
	//! PDU type: management, or IP. 2 bits (1 is reserved).
	unsigned char type_;
	//! ARQ enabled: 0x0 = no, 0x2 = up to four retransmissions allowed. 1 bit.
	unsigned char reliability_;
	//! Priority/class. 3 bits.
	unsigned char priority_;
	//! Drop precedence. 2 bits.
	unsigned char drop_;

public:
	//! CID types.
	enum { MANAGEMENT = 0x0, DATA = 0x1 };

	//! ARQ enabled flag.
	enum { NO_ARQ = 0x0, ARQ = 0x1 };

	//! Number of priorities (= eight since the field has three bits).
	enum { MAX_PRIO = 8 };

	//! Number of precedences (= four since the field has two bits).
	enum { MAX_PREC = 4 };
	
	//! Build an empty MeshCID. Management by default.
	WimaxMeshCid () {
		dst_ = 0;
		type_ = MANAGEMENT;
		reliability_ = NO_ARQ;
		priority_ = 0;
		drop_ = 0;
	}

	//! Get/set the destination NodeID.
	WimaxNodeId& dst () { return dst_; }
	//! Return true if this is a management CID.
	unsigned char& type () { return type_; }
	//! Get/set the reliability.
	unsigned char& reliability () { return reliability_; }
	//! Get/set the priority/class field.
	unsigned char& priority () { return priority_; }
	//! Get/set the drop precedence.
	unsigned char& drop () { return drop_; }
};

#endif // __NS2_WIMSH_HEADER_H
