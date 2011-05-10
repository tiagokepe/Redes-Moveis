#ifndef __NS2_WIMSH_BW_MANAGER_FEBA_H
#define __NS2_WIMSH_BW_MANAGER_FEBA_H


#include <wimsh_weight_manager.h>
#include <wimsh_bwmanager.h>
#include <wimsh_packet.h>
#include <wimsh_mac.h>




class WimshBwManagerFeba : public WimshBwManager {
	
	typedef std::vector< std::bitset<MAX_SLOTS> > Bitmap;

protected:
	struct NeighbourDesc {
	
		/* Número de bytes requisitados ao vizinho. */

		unsigned int req_in_;

		/* Concedidos. */

		unsigned int gnt_in_;

		/* Confirmados. */

		unsigned int cnf_in_;

		/* Compensação. */

		unsigned int lag_in_;

		
		unsigned int req_out_;

		unsigned int gnt_out_;

		unsigned int cnf_out_;
		
		unsigned int lag_out_;

		NeighbourDesc() {
			req_in_=0;
			req_out_=0;
			gnt_in_=0;
			gnt_out_=0;
			lag_in_=0;
			lag_out_=0;
		}

		std::vector<NeighbourDesc> neigh_;

	};



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

