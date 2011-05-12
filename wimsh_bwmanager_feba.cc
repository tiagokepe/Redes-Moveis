#include <wimsh_bwmanager_feba.h>
#include <iostream>
using namespace std;

WimshBwManagerFeba::WimshBwManagerFeba ( WimshMac* m) :
	WimshBwManager (m), wm_ (m)
{
}


int WimshBwManagerFeba::command (int argc, const char*const* argv) {
	return TCL_OK;
}


void WimshBwManagerFeba::recvMshDsch (WimshMshDsch* dsch){


}

void WimshBwManagerFeba::schedule (WimshMshDsch* dsch){
}

void WimshBwManagerFeba::initialize (){
	
	int nneighbors = mac_->nneighs();
	neigh_.resize(nneighbors);

	neigh_tx_unavl_.resize(nneighbors);

	busy_.resize(HORIZON);

	// Para cada vizinho
	for(int ngh_index=0; ngh_index < nneighbors; ngh_index++)
	{
		neigh_tx_unavl_[ngh_index].resize(mac_->nchannels());
		//Para cada canal
		for(int ch_index=0; ch_index < mac_->nchannels(); ch_index++)
		{
			neigh_tx_unavl_[ngh_index][ch_index].resize(HORIZON);
			self_rx_unavl_[ch_index].resize(HORIZON);
			self_tx_unavl_[ch_index].resize(HORIZON);

			// Para cada frame
			for(int fr_index; fr_index < HORIZON; fr_index++)
			{
				neigh_tx_unavl_[ngh_index][ch_index][fr_index].reset();
				self_rx_unavl_[ch_index][fr_index].reset();
				self_tx_unavl_[ch_index][fr_index].reset();
			}
		}
	}

	//Para cada canal
	for(int ch_index=0; ch_index < mac_->nchannels(); ch_index++)
	{
		self_rx_unavl_[ch_index].resize(HORIZON);
		self_tx_unavl_[ch_index].resize(HORIZON);

		// Para cada frame
		for(int fr_index; fr_index < HORIZON; fr_index++)
		{
			self_rx_unavl_[ch_index][fr_index].reset();
			self_tx_unavl_[ch_index][fr_index].reset();
		}
	}


}
	
void WimshBwManagerFeba::backlog (WimaxNodeId src, WimaxNodeId dst, unsigned char prio, WimaxNodeId nexthop, unsigned int bytes){
}
	
void WimshBwManagerFeba::backlog (WimaxNodeId nexthop, unsigned int bytes){
}

void WimshBwManagerFeba::sent (WimaxNodeId nexthop, unsigned int bytes){
}
