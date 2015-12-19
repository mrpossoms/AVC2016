/*
typedef enum{
	MSG_GREETING = 0,
	MSG_STATE,
	MSG_TRACKING,
	MSG_VIDEO,
} msgType_e;

typedef struct{
	uint32_t  signature;
	msgType_e type;
} msgHeader_t;

typedef int(rxProc*)(int socket, struct sockaddr* peer) rxProc_t;
*/

static rxProc_t RX_PROCESSORS[MSG_COUNT];

static int                SOCK;
static struct sockaddr_in HOST_ADDR;

static const uint32_t SIG = 0xDEADBEEF;

int commInitClient(const char* hostname)
{			
	struct hostent* he;

	HOST_ADDR.sin_family = AF_INET;
	HOST_ADDR.sin_port   = htons(port);	

	// try to resolve the host's name
	if(!(he = gethostbyname(hostname))){
		return -1;
	}

	// open the socket
	if(!(SOCK = socket(AF_INET, SOCK_DGRAM, 0))){
		return -2;
	}

	return 0;
}

int commInitHost(uint16_t port)
{
	struct sockaddr_in addr = {};

	addr.sin_family = AF_INET;
	addr.sin_port   = htons(port);
	addr.sin_addr   = htonl(INADDR_ANY);
	
	// open the socket
	if(!(SOCK = socket(AF_INET, SOCK_DGRAM, 0))){
		return -1;
	}

	// bind to the port
	if(bind(SOCK, (const struct sockaddr*)&addr, sizeof(addr) < 0)){
		return -2;
	}

	return 0;
}

int commSend(msgType_e type, const void* payload, size_t payloadSize, struct sockaddr* peer)
{
	msgHeader_t header = {};

	header.signature = SIG;
	header.type      = type;

	if(!payload) return -1;
	if(!peer) return -2;

	return sendto(
		SOCK,
		payload,
		payloadSize,
		0,
		peer,
		sizeof(*peer)
	);
}

void commRegisterRxProc(msgType_e type, rxProc_t processor)
{
	RX_PROCESSORS[type] = processor;
}

int commListen()
{
	return 0;
}

