#ifndef __NS2_WIMSH_BW_MANAGER_FEBA_H
#define __NS2_WIMSH_BW_MANAGER_FEBA_H


#include <wimsh_weight_manager.h>
#include <wimsh_bwmanager.h>
#include <wimsh_packet.h>
#include <wimsh_mac.h>




class WimshBwManagerFeba : public WimshBwManager {
	
	typedef std::vector< std::bitset<MAX_SLOTS> > Bitmap;


	struct NeighbourDesc {
		
		unsigned int req_in_;

		unsigned int gnt_in_;

		unsigned int cnf_in_;

		unsigned int lag_in_;

		unsigned int req_out_;

		unsigned int gnt_out_;

		unsigned int cnf_out_;

	}



public:
	/* Construtor */
	WimshBwManagerFeba (WimshMac* m);
	~WimshBwManagerFeba () { }

	/* Weight Manager */
	WimshWeightManager wm_;


	/* Interface com o Tcl */
	int command (int argc, const char*const* argv);


	void recvMshDsch (WimshMshDsch* dsch);

	void schedule (WimshMshDsch* dsch);

	void initialize ();
	
	void backlog (WimaxNodeId src, WimaxNodeId dst, unsigned char prio, WimaxNodeId nexthop, unsigned int bytes);
	
	void backlog (WimaxNodeId nexthop, unsigned int bytes);

	void sent (WimaxNodeId nexthop, unsigned int bytes);
	
	void received (WimaxNodeId src, WimaxNodeId dst, unsigned char prio, WimaxNodeId source, unsigned int bytes) {
		wm_.flow (src, dst, prio, source, wimax::IN); 
	}
			
};

#endif

