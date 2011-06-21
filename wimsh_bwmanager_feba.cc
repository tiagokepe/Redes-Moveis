#include <wimsh_bwmanager_feba.h>

#include <wimsh_mac.h>
#include <wimsh_topology.h>
#include <wimsh_packet.h>

#include <random.h>
#include <stat.h>

#include <iostream>
using namespace std;

WimshBwManagerFeba::WimshBwManagerFeba (WimshMac* m) :
	WimshBwManager (m), wm_ (m)
{

	regrantOffset_         = 1;
	regrantDuration_       = 1;
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
WimshBwManagerFeba::command (int argc, const char*const* argv)
{
	if ( argc == 2 && strcmp (argv[0], "availabilities") == 0 ) {

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

}

void WimshBwManagerFeba::initialize (){

	int nneighbors = mac_->nneighs();
	neigh_.resize(nneighbors);

	neigh_tx_unavl_.resize(nneighbors);

	busy_.resize(HORIZON);
	unconfirmedSlots_.resize(HORIZON);

	for(int fr_index = 0; fr_index < HORIZON; fr_index++)
	{
		busy_[fr_index].reset();
		unconfirmedSlots_[fr_index].reset();
	}


	// Para cada vizinho
	for(int ngh_index=0; ngh_index < nneighbors; ngh_index++)
	{
		neigh_tx_unavl_[ngh_index].resize(mac_->nchannels());
		//Para cada canal
		for(unsigned int ch_index=0; ch_index < mac_->nchannels(); ch_index++)
		{
			neigh_tx_unavl_[ngh_index][ch_index].resize(HORIZON);

			// Para cada frame
			for( int fr_index = 0; fr_index < HORIZON; fr_index++)
			{
				neigh_tx_unavl_[ngh_index][ch_index][fr_index].reset();
			}
		}
	}

	self_rx_unavl_.resize(mac_->nchannels());
	self_tx_unavl_.resize(mac_->nchannels());
	//Para cada canal
	for(unsigned int ch_index=0; ch_index < mac_->nchannels(); ch_index++)
	{
		self_rx_unavl_[ch_index].resize(HORIZON);
		self_tx_unavl_[ch_index].resize(HORIZON);
		// Para cada frame
		for(int fr_index = 0; fr_index < HORIZON; fr_index++)
		{
			self_rx_unavl_[ch_index][fr_index].reset();
			self_tx_unavl_[ch_index][fr_index].reset();
		}
	}

	wm_.initialize();


}

void
WimshBwManagerFeba::recvMshDsch (WimshMshDsch* dsch)
{

	cout << "gnt:" << dsch->gnt().size() << "req:" << dsch->req().size() << endl;

	rcvGrants(dsch);
	rcvAvailabilities(dsch);
	rcvRequests(dsch);
}

void
WimshBwManagerFeba::rcvGrants (WimshMshDsch* dsch) {

	// identificação de quem enviou a mensagem
	unsigned int ndx = mac_->neigh2ndx (dsch->src());


	std::list<WimshMshDsch::GntIE>& gnt = dsch->gnt();

	std::list<WimshMshDsch::GntIE>::iterator it;
	for ( it = gnt.begin() ; it != gnt.end() ; ++it ) {

		// Grant destinado a este nodo.

		if ( it->fromRequester_ == false && it->nodeId_ == mac_->nodeId() ) {

			it->nodeId_ = dsch->src();
			it->fromRequester_ = true;

			unconfirmed_.push_back (*it);


			unsigned int frange = WimshMshDsch::pers2frames (it->persistence_);

			setSlots (unconfirmedSlots_, it->frame_, frange,
					it->start_, it->range_, true);

			// atualize a quantidade de bytes concedidos por este nodo
			neigh_[ndx].gnt_out_ +=
				frange * mac_->slots2bytes (ndx, it->range_, true);
			Stat::put ( "wimsh_gnt_out", mac_->index(),
				frange * mac_->slots2bytes (ndx, it->range_, true) );


			neigh_[ndx].gnt_out_ =
				( neigh_[ndx].gnt_out_ < neigh_[ndx].req_out_ ) ?
				neigh_[ndx].gnt_out_ : neigh_[ndx].req_out_;
		}

		// Grant endereçado a outro nodo
		if ( it->fromRequester_ == false && it->nodeId_ != mac_->nodeId() ) {

			// Estaremos indisponiveis durante o tempo concedido, no canal a ser usado.

			WimshMshDsch::AvlIE avl;
			avl.frame_ = it->frame_;
			avl.start_ = it->start_;
			avl.range_ = it->range_;
			avl.direction_ = WimshMshDsch::UNAVAILABLE;
			avl.persistence_ = it->persistence_;
			avl.channel_ = it->channel_;

			availabilities_.push_back (avl);

			unsigned int frange = WimshMshDsch::pers2frames (it->persistence_);

			for ( unsigned int ch = 0 ; ch < mac_->nchannels() ; ch++ ) {
				setSlots (neigh_tx_unavl_[ndx][ch], it->frame_, frange,
						it->start_, it->range_, true);
			}

			// Se o nodo que requisitou for nosso vizinho, não poderemos conceder nada a ele.

			if ( mac_->topology()->neighbors (it->nodeId_, mac_->nodeId()) ) {
				// index of the requester
				const unsigned int ndx = mac_->neigh2ndx (it->nodeId_);

				for ( unsigned int ch = 0 ; ch < mac_->nchannels() ; ch++ ) {
					setSlots (neigh_tx_unavl_[ndx][ch], it->frame_, frange,
							it->start_, it->range_, true);
				}
			}


			std::vector<WimaxNodeId> gntNeigh;
			mac_->topology()->neighbors (dsch->src(), gntNeigh); // retrieve them
			for ( unsigned int ngh = 0 ; ngh < gntNeigh.size() ; ngh++ ) {

				// pule o requisitor e nodos que não são nossos vizinhos
				if ( gntNeigh[ngh] == it->nodeId_ ||
					  ! mac_->topology()->neighbors (gntNeigh[ngh], mac_->nodeId()) )
					continue;

				const unsigned int ndx = mac_->neigh2ndx (gntNeigh[ngh]); // index
				setSlots (neigh_tx_unavl_[ndx][it->channel_],
						it->frame_, frange, it->start_, it->range_, true);
			}

			//
			setSlots (self_tx_unavl_[it->channel_],
					it->frame_, frange, it->start_, it->range_, true);
		}

		// Confirmação endereçada a outro nodo
		if ( it->fromRequester_ == true &&
				! mac_->topology()->neighbors (it->nodeId_, mac_->nodeId()) ) {
			// O destinatário desta confirmação não pode ser nosso vizinho.
			// Neste caso já teriamos cuidado de seu grant.


			unsigned int fstart;
			unsigned int frange;
			realPersistence (it->frame_, it->persistence_, fstart, frange);

			for ( unsigned int ch = 0 ; ch < mac_->nchannels() ; ch++ ) {
				setSlots (neigh_tx_unavl_[ndx][ch], fstart, frange,
						it->start_, it->range_, true);
			}

			setSlots (self_rx_unavl_[it->channel_], fstart, frange,
					it->start_, it->range_, true);
		}

		// confirmação endereçada a este nodo
		if ( it->fromRequester_ == true && it->nodeId_ == mac_->nodeId() ) {

			unsigned int ndx = mac_->neigh2ndx (dsch->src());


			unsigned int frange = WimshMshDsch::pers2frames (it->persistence_);

			// Após a confirmação, precisaremos escutar no canal definido.
			setSlots (channel_, it->frame_, frange,
					it->start_, it->range_, it->channel_);

			// Incrementamos o número de bytes de entrada confirmados.
			neigh_[ndx].cnf_in_ +=
				frange * mac_->slots2bytes (ndx, it->range_, true);
		}
	}

}

void
WimshBwManagerFeba::rcvAvailabilities (WimshMshDsch* dsch) {
	/* Lista de indisponibilidades */
	std::list< WimshMshDsch::AvlIE >avl = dsch->avl();

	std::list< WimshMshDsch::AvlIE >::iterator it;

	// Índice do meu vizinho.
	int ngh_index;

	// Dados atualizados do meu frame
	unsigned int frame_start;
	unsigned int frame_range;

	// Atualizamos nossa estrutura de dados de acordo com as disponibilidades recebidas.
	for( it = avl.begin(); it != avl.end(); it++ )
	{
		if ( ( it->direction_ == WimshMshDsch::RX_ONLY) || (it->direction_ == WimshMshDsch::UNAVAILABLE ) )
		{
			ngh_index = mac_->neigh2ndx(dsch->src());
			realPersistence(it->frame_,it->persistence_,frame_start,frame_range);
			setSlots(neigh_tx_unavl_[ngh_index][it->channel_],frame_start,frame_range,it->start_,it->range_,true);
		}

	}
}

void
WimshBwManagerFeba::rcvRequests (WimshMshDsch* dsch) {

	/* Lista de requisições */
	std::list< WimshMshDsch::ReqIE >req = dsch->req();

	std::list< WimshMshDsch::ReqIE >::iterator it;

	int ngh_index = mac_->neigh2ndx(dsch->src());

	for( it = req.begin(); it != req.end(); it++)
	{
		if( it->nodeId_ == mac_->nodeId()  )
		{

			// Indica ao weightManager que pode haver novo fluxo
			wm_.flow (ngh_index, wimax::IN);


			 neigh_[ngh_index].req_in_+= WimshMshDsch::pers2frames(it->persistence_) * mac_->slots2bytes(ngh_index, it->level_, false);
		//	 cout << "Recebi" << endl;
			 // Inserimos este vizinho na lista de conexões ativas,
			 // caso ele ainda não esteja lá
			 if (   neigh_[ngh_index].req_in_ > neigh_[ngh_index].gnt_in_ && !( activeList_.find(wimax::LinkId(ngh_index,wimax::IN)) )   )
				 activeList_.insert(wimax::LinkId(ngh_index,wimax::IN));
		}
	}
}

void WimshBwManagerFeba::schedule (WimshMshDsch* dsch){

	//  Encaixar as disponibilidades
	 availabilities(dsch);

	// Confirmar concessões
	confirm(dsch);

	// Varre a lista de ativos para tratar requisições e concessões.
	requestGrant (dsch);
}


void WimshBwManagerFeba::availabilities(WimshMshDsch* dsch) {


	// Adicione o maior número de disponibilidades possíveis
	while ( ! availabilities_.empty() ) {

		// Se não há mais espaço, paramos
		if ( dsch->remaining() < WimshMshDsch::AvlIE::size() ) break;

		WimshMshDsch::AvlIE avl = availabilities_.front();
		availabilities_.pop_front();

		// Incluimos a disponibilidade na mensagem, a não ser que esta informação não seja mais válida.
		// Corresponda a um frame que não exite mais.
		if ( mac_->frame() <=
				avl.frame_ + WimshMshDsch::pers2frames (avl.persistence_) ) {
			dsch->add (avl);
		}
	}
}


void
WimshBwManagerFeba::requestGrant (WimshMshDsch* dsch)
{
	// Usado para calcular o horizonte de concessão de banda.
	unsigned int Hslf = handshake(mac_->nodeId());


	// número de bytes que serão requisitados em uma requisição
	unsigned int reqIeOccupancy = 0;

	// Para quais vizinhos requisitamos banda
	std::vector<bool> neighReq (mac_->nneighs(), false);
	// Quantos bytes foram requisitados a cada vizinho.
	std::vector<unsigned int> neighReqBytes (mac_->nneighs(), 0);

	// Guardamos o número máximo de bytes que podem ser requeridos a um vizinho
	std::vector<unsigned int> neighReqMax (mac_->nneighs());
	for ( unsigned int i = 0 ; i < mac_->nneighs() ; i++ ) {
		neighReqMax[i] = maxReq(i);
	}


	// Usado quando não foi possível alocar banda suficiente a um vizinho. Evita um loop infinito na lista ativa.
	unsigned int ineligible = 0;
	bool ineligibleValid = false;

	if ( deficitOverflow_ ) ++ddTimer_;


	// Enquanto a lista de conexões ativas ainda existir, continue
	while ( ! activeList_.empty() ) {

		// Capturamos informações sobre esta conexão
		const unsigned int ndx         = activeList_.current().ndx_;
		const unsigned int dst         = mac_->ndx2neigh (ndx);
		const wimax::LinkDirection dir = activeList_.current().dir_;

		// Se não foi possível alocar banda suficiente a este nodo no loop anterior, saimos
		if ( ineligibleValid && ineligible == ndx ) break;



		// Se for um fluxo de entrada, eu devo conceder
		// Ver grant(i) no artigo do FEBA
		if ( dir == wimax::IN ) {

			// Se não há mais espaço na mensagem, saimos.
			if ( dsch->remaining() - reqIeOccupancy < WimshMshDsch::GntIE::size() )
				break;

			unsigned int Hdst = handshake (dst);

			unsigned int& deficit   = neigh_[ndx].def_in_;
			unsigned int& granted   = neigh_[ndx].gnt_in_;
			unsigned int& requested = neigh_[ndx].req_in_;

			// lag_i_in <- min{lag_i_in + quantum, req_i_in - gnt_i_in}
			if ( ! deficitOverflow_ ) deficit += quantum (ndx, wimax::IN);
			else deficitOverflow_ = false;
			deficit =
				( ! fairGrant_ && maxDeficit_ > 0 && deficit > maxDeficit_ )
				? maxDeficit_ : deficit;
			unsigned int pending = requested - granted;
			deficit = ( deficit > pending ) ? pending : deficit;

			// Nós iremos conceder banda até que o deficit tenha sido contemplado, ou que não haja mais espaço na mensagem
			// ou não há mais opções de alocação do horizonte.
			while ( dsch->remaining() - reqIeOccupancy
					       >= WimshMshDsch::GntIE::size() &&
					deficit > 0 ) {

				WimshMshDsch::GntIE gnt;
			//	cout << "bytes gnt:" << deficit << endl;

				// Tentaremos realizar um grant de deficit bytes
				gnt = grantFit (ndx, deficit, mac_->frame() + Hdst,mac_->frame() + 2 * Hdst + Hslf);

				// Se não conseguimos alocar nenhuma banda, saimos do loop
				if ( gnt.range_ == 0 ) break;

//				Stat::put ("wimsh_gnt_size", mac_->index(), gnt.range_);

				// adicionamos o nosso grant
				dsch->add (gnt);

				unsigned int frange = WimshMshDsch::pers2frames (gnt.persistence_);

				unsigned int bgnt =
					frange * mac_->slots2bytes (ndx, gnt.range_, true);

				// Atualizamos o deficit com o valor que foi alocado
				deficit = ( deficit > bgnt ) ? ( deficit - bgnt ) : 0;

				// Número de bytes alocados aumentou
				granted += bgnt;

				// Durante o período alocação, não poderemos escutar em nenhum outro canal
				setSlots (busy_, gnt.frame_, frange,
						gnt.start_, gnt.range_, true);
			}

			if ( fairGrant_ && ( deficit > maxDeficit_ ) ) {
				deficitOverflow_ = true;
				break;
			}

			// Se não foi possível alocar banda suficiente neste turno, marcamos isto
			if ( ! ineligibleValid && deficit > 0 ) {
				ineligibleValid = true;
				ineligible = ndx;
			}

			// Se não há mais nada a ser alocado para este link, tiramos ele da lista ativa.
			if ( granted >= requested ) {
				deficit = 0;
				activeList_.erase();
				// senão apenas movemos o ponteiro por causa do round robin.
			} else {
				activeList_.move ();
			}


		} else {   // Se for um fluxo de saída, eu devo requisitar. wimax::OUT
			// Implementa o request(i) do artigo FEBA

			unsigned int& deficit   = neigh_[ndx].def_out_;
			unsigned int& backlog   = neigh_[ndx].backlog_;
			unsigned int& confirmed = neigh_[ndx].cnf_out_;
			unsigned int& requested = neigh_[ndx].req_out_;
			unsigned int& granted   = neigh_[ndx].gnt_out_;


			// número de bytes a serem requisitados
			unsigned int pending =
				( backlog > requested ) ? ( backlog - requested ) : 0;

			// número de bytes que ainda não nos foram concedidos
			unsigned int ungranted =
				( requested > granted ) ? ( requested - granted ) : 0;

			// Se não há mais espaço na mensagem, saimos
			if ( dsch->remaining() - reqIeOccupancy < WimshMshDsch::ReqIE::size() )
				break;


			// lag_i_out = min{ lag_i_out + quantum, blog_i_out}
			if ( ! deficitOverflow_ ) deficit += quantum (ndx, wimax::OUT);
			else deficitOverflow_ = false;
			deficit =
				( ! fairRequest_ && maxDeficit_ > 0 && deficit > maxDeficit_ )
				? maxDeficit_ : deficit;
			deficit = ( deficit > pending ) ? pending : deficit;

			// Se há muitos bytes pendentes.
			if ( maxBacklog_ > 0 && ungranted > maxBacklog_ ) {


				if ( ddTimeout_ > 0 && ddTimer_ >= ddTimeout_ ) {


					ddTimer_ = 0;
					deficitOverflow_ = false;


					granted   = 0;
					deficit   = 0;
					confirmed = 0;
					requested = 0;
				}

				// Saimos se o deficit ultrapassou o limite máximo
				if ( fairRequest_ && deficit > maxDeficit_ ) {
					deficitOverflow_ = true;
					break;
				}

				if ( ! ineligibleValid ) {
					ineligibleValid = true;
					ineligible = ndx;
				}

				// Movemos o apontador do round-robin para o próximo elemento
				activeList_.move();
				continue;
			}

			// if we are here, when we have to stop the deadlock detection timer
			if ( ddTimeout_ > 0 ) ddTimer_ = 0;

			// Se ainda não requisitamos banda a este vizinho, adicionamos no calcúlo da MSH-DSCH um request.
			// A intenção é que eu junte várias requisições a este nodo em uma somente.
			if ( neighReq[ndx] == false )
				reqIeOccupancy += WimshMshDsch::ReqIE::size();

			// Marcamos que desejamos requisitar banda a um nodo
			neighReq[ndx] = true;

			// atualize o número de bytes que estamos requisitando
			neighReqBytes[ndx] += deficit;

			// Se as requisições a este nodo ultrapassaram o limite de tamanho, enviamos ela imediatamente
			while ( neighReqBytes[ndx] > neighReqMax[ndx] ) {

				WimshMshDsch::ReqIE ie;
				ie.nodeId_ = mac_->ndx2neigh (ndx);
				ie.level_ = WimshMshDsch::FRAME128;


				neighReqBytes[ndx] -= neighReqMax[ndx];
				
				dsch->add (ie);
			}

			// atualiza o número de bytes requisitados
			requested += deficit;


			deficit = 0;

			// se não há mais bytes pendentes, removemos este link da lista de conexões ativas.
			if ( pending == 0 ) {

				activeList_.erase();

			// senão vamos para o próximo link, de acordo com o DRR
			} else {
				activeList_.move();
			}
		}

	}

	// Agora realizamos as requisições que foram consideradas no loop anterior

	for ( unsigned int ndx = 0 ; ndx < mac_->nneighs() ; ndx++ ) {
		if ( neighReqBytes[ndx] == 0 ) continue;

		WimshMshDsch::ReqIE ie;
		ie.nodeId_ = mac_->ndx2neigh (ndx);


		WimshMshDsch::slots2level (
				mac_->phyMib()->slotPerFrame(),
				mac_->bytes2slots (ndx, neighReqBytes[ndx], false),
				ie.level_, ie.persistence_);

		dsch->add (ie);
	}
}


void WimshBwManagerFeba::confirm (WimshMshDsch * dsch  ){

	// Devemos confirmar o maior numero possível de concessões
	while( !unconfirmed_.empty() ) {

		// Se não há mais espaço na mensagem, terminamos
		if ( dsch->remaining() < WimshMshDsch::GntIE::size() ) break;


		WimshMshDsch::GntIE gnt;


		// Número de bytes confirmados
		unsigned int confirmed = 0;

		gnt = unconfirmed_.front();
		unconfirmed_.pop_front();

		unsigned int frame_start;
		unsigned int frame_range;

		realPersistence(gnt.frame_,gnt.persistence_,frame_start, frame_range);

		unsigned int ngh_index = mac_->neigh2ndx(gnt.nodeId_);


		while ( dsch->remaining() >= WimshMshDsch::GntIE::size() ) {

			confFit(frame_start, frame_range, gnt.start_, gnt.range_, gnt);


			// Se não há mais bytes a serem confirmados, termine
			if ( gnt.range_ == 0 )
				break;

			dsch->add(gnt);

			unsigned int actual_frame_start;
			unsigned int actual_frame_range;

			realPersistence(gnt.frame_,gnt.persistence_,actual_frame_start,actual_frame_range);

			// Computamos o valor dos bytes confirmados
			confirmed += actual_frame_range * mac_->slots2bytes (ngh_index, gnt.range_, true);
			// Marcamos as estruturas de dados. Para avisar onde transmitiremos.
			setSlots(busy_,actual_frame_start,actual_frame_range,gnt.start_,gnt.range_,true);
			setSlots (grants_, actual_frame_start, actual_frame_range, gnt.start_, gnt.range_, true);
			setSlots (dst_, actual_frame_start, actual_frame_range, gnt.start_, gnt.range_, gnt.nodeId_);
			setSlots (channel_, actual_frame_start, actual_frame_range, gnt.start_, gnt.range_, gnt.channel_);
		}


		neigh_[ngh_index].cnf_out_ += confirmed;
		neigh_[ngh_index].cnf_out_ =	( neigh_[ngh_index].cnf_out_ < neigh_[ngh_index].gnt_out_ ) ?
			neigh_[ngh_index].cnf_out_ : neigh_[ngh_index].gnt_out_;



		// Se o número de requisições e menor que o de confirmações, a simulação é abortada.
		if ( neigh_[ngh_index].req_out_ < neigh_[ngh_index].cnf_out_ ) abort();

		// Os dados sobre os bytes desta confirmação são esquecidos.
		neigh_[ngh_index].gnt_out_ -= neigh_[ngh_index].cnf_out_;
		neigh_[ngh_index].req_out_ -= neigh_[ngh_index].cnf_out_;
		neigh_[ngh_index].cnf_out_ = 0;



	}

}


void WimshBwManagerFeba::realPersistence(unsigned int frame_start, WimshMshDsch::Persistence frame_range, unsigned int &actual_frame_start, unsigned int &actual_frame_range){
	actual_frame_range = WimshMshDsch::pers2frames(frame_range);

	actual_frame_range -= ( frame_start > mac_->frame() )? 0 : mac_->frame() - frame_start  ;

	if ( actual_frame_range < 0  ) actual_frame_range = 0;

	actual_frame_start = ( frame_start >= mac_->frame() )? frame_start : mac_->frame();
}


WimshMshDsch::GntIE
WimshBwManagerFeba::grantFit (
		unsigned int ndx, unsigned int bytes,
		unsigned int minFrame, unsigned int maxFrame)
{
	// grant a ser retornado
	WimshMshDsch::GntIE gnt;
	gnt.nodeId_ = mac_->ndx2neigh (ndx);


	// Número de slots por frame
	unsigned int N = mac_->phyMib()->slotPerFrame();

	// número de canais
	unsigned int C = mac_->nchannels();
	// para cada frame
	for ( unsigned int f = minFrame ; f <= maxFrame ; f++ ) {
		unsigned int F = f % HORIZON;

		// pegamos um canal aleatório
		unsigned int ch = 0;
		ch = grantFitRng.uniform ((int)C);
		// para cada canal
		for ( unsigned int c = 0 ; c < C ; c++ ) {

			// Marcamos os slots que não podemos conceder.
			std::bitset<MAX_SLOTS> map =
			  unconfirmedSlots_[F] | busy_[F] |
			  self_rx_unavl_[ch][F] | neigh_tx_unavl_[ndx][ch][F];

			// Para cada minislot
			for ( unsigned int s = 0 ; s < N ; s++ ) {

				// Se achou um slot livre, pode tentar alocar.
				if ( map[s] == false ) {
					gnt.frame_ = f;
					gnt.start_ = s;
					gnt.persistence_ = WimshMshDsch::FRAME1;
					gnt.fromRequester_ = false;
					gnt.channel_ = ch;


					unsigned int maxSlots = mac_->bytes2slots (ndx, bytes, true);
					for ( ; s < N && map[s] == false &&
						( s - gnt.start_ ) < maxSlots ; s++ ) { }

					gnt.range_ = s - gnt.start_;

					// Verifica se o valor não é nulo, a não ser que isso tenha sido pedido.
					if ( gnt.range_ != maxSlots && gnt.range_ <= 0 )
						continue;

					return gnt;
				}
			}

			ch = ( ch + 1 ) % C;

		}
	}

	// Se chegamos até aqui não foi possível alocar banda.
	gnt.range_ = 0;
	return gnt;
}



void WimshBwManagerFeba::confFit(unsigned int frame_start, unsigned int frame_range, unsigned int slot_start, unsigned int slot_range, WimshMshDsch::GntIE& gnt) {

	// Para cada frame
	for( unsigned int f = frame_start; f < frame_start + frame_range; f++) {
		unsigned int actual_frame = f % HORIZON;

		// Verificamos as impossibilidades de transmissão.
		const std::bitset<MAX_SLOTS> map = busy_[actual_frame] | self_tx_unavl_[gnt.channel_][actual_frame];

		// Para cada minislot
		for ( unsigned int slot = slot_start; slot < slot_start + slot_range; slot++ ) {

			if ( map[slot] == false ) {
				gnt.frame_ = f;
				gnt.start_ = slot;
				gnt.persistence_ = WimshMshDsch::FRAME1;

				// Procure o maior número de minislots
				for ( ; slot < ( slot_start + slot_range ) && map[slot] == false ; slot++ ) { }

				gnt.range_ = slot - gnt.start_;
				return;
			}
		}
	}

	// Se chegamos até aqui, retornamos um grant vazio. Não é possível conceder mais nada.
	gnt.range_ = 0;
}

void WimshBwManagerFeba::backlog (WimaxNodeId src, WimaxNodeId dst, unsigned char prio, WimaxNodeId nexthop, unsigned int bytes){

	// Índice do meu vizinho
	const unsigned int ndx = mac_->neigh2ndx(nexthop);

	// Adicionamos este fluxo ao pesador de fluxos
	wm_.flow(src,dst,prio,ndx,wimax::OUT);


	neigh_[ndx].backlog_+=bytes;

	// Este fluxo precisa ser adicionado se não existir, para que o alocador em Deficit Round Robin possa ser usado.
	if (  !activeList_.find(wimax::LinkId(ndx, wimax::OUT))  )
		activeList_.insert(wimax::LinkId(ndx,wimax::OUT));
}

void WimshBwManagerFeba::backlog (WimaxNodeId nexthop, unsigned int bytes){

	// Índice do meu vizinho
	const unsigned int ndx = mac_->neigh2ndx(nexthop);

	wm_.flow(ndx, wimax::OUT);

	neigh_[ndx].backlog_+=bytes;

	if (  !activeList_.find(wimax::LinkId(ndx, wimax::OUT))  )
		activeList_.insert(wimax::LinkId(ndx,wimax::OUT));
}

void WimshBwManagerFeba::sent (WimaxNodeId nexthop, unsigned int bytes){

	const unsigned int ngh_index = mac_->neigh2ndx(nexthop);

	// Atualizamos o backlog aqui ( ver requestAndgrant) .
	neigh_[ngh_index].backlog_ -= bytes;
}
