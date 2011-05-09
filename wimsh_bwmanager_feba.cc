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
	    for ( unsigned int i = 0 ; i < mac_->nneighs() ; i++ ) {
			cout << "Estou iniciando" <<  mac_->alpha (i) << endl;
		}
}
	
void WimshBwManagerFeba::backlog (WimaxNodeId src, WimaxNodeId dst, unsigned char prio, WimaxNodeId nexthop, unsigned int bytes){
}
	
void WimshBwManagerFeba::backlog (WimaxNodeId nexthop, unsigned int bytes){
}

void WimshBwManagerFeba::sent (WimaxNodeId nexthop, unsigned int bytes){
}
