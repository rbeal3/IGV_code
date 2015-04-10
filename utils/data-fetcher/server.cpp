/* This code is adapted from the sample provided by Prof. Kredo
 * 
 * Author: Sterling Baldwin
 * Also: Naomi Miller
 */
#include <sys/types.h>
#include <sys/socket.h>
#include <netdb.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fstream>
using namespace std;

#define SERVER_PORT "8000"
#define MAX_LINE 256
#define MAX_PENDING 5


int
main(int argc, char *argv[])
{
	struct addrinfo hints;
	struct addrinfo *rp, *result;
	char buf[MAX_LINE];
	int s, new_s;
	int len;
	int debug = false;
	if(argc == 2) debug = true;


	if( debug ) printf("[+] Launching server in debug mode\n");
	/* Build address data structure */
	memset(&hints, 0, sizeof(struct addrinfo));
	hints.ai_family = AF_INET;
	hints.ai_socktype = SOCK_STREAM;
	hints.ai_flags = AI_PASSIVE;
	hints.ai_protocol = 0;
	hints.ai_canonname = NULL;
	hints.ai_addr = NULL;
	hints.ai_next = NULL;

	/* Get local address info */
	if ((s = getaddrinfo(NULL, SERVER_PORT, &hints, &result)) != 0 )
	{
		fprintf(stderr, "%s: getaddrinfo: %s\n", argv[0], gai_strerror(s));
		exit(1);
	}

	/* Iterate through the address list and try to perform passive open */
	for (rp = result; rp != NULL; rp = rp->ai_next)
	{
		if ((s = socket(rp->ai_family, rp->ai_socktype, rp->ai_protocol)) == -1 )
		{
			continue;
		}

		if (!bind(s, rp->ai_addr, rp->ai_addrlen))
		{
			break;
		}

		close(s);
	}
	if (rp == NULL)
	{
		perror("project 2 server: bind");
		exit(1);
	}
	if (listen(s, MAX_PENDING) == -1)
	{
		perror("project 2 server: listen");
		close(s);
		exit(1);
	}

	/* Wait for connection, then receive and print text */
	while(1)
	{
		if ((new_s = accept(s, rp->ai_addr, &(rp->ai_addrlen))) < 0)
		{
			perror("project 2 server: accept");
			close(s);
			exit(1);
		} else {
			if( debug ) printf("new connection\n");
		}
		while ((len = recv(new_s, buf, sizeof(buf), 0)))
		{
			if( debug ) printf("[+] revced message \"%s\"\n", buf);
			ifstream file;
			file.open(buf, ios::binary);
			if(file.is_open()) {
				if(debug) printf("[+] Found the file\n");
				while( !file.eof() ) 
				{
					char buffer[1024] = "";
					int BytesSent = 0;
					int BytesIndex = 0;
					file.read(buffer, 1024);
					int BytesLeft = file.gcount();
					while(BytesLeft != 0)
					{
						BytesSent = send(new_s, &buffer[BytesIndex], BytesLeft, 0);
						BytesLeft -= BytesSent;
						BytesIndex +=BytesSent;
					}
				}
				char quit[] = "q\0";
				int qlen = strlen(quit)+1;
				send(new_s, quit, qlen, 0);
				if(debug) printf("[+] Sending quit signal to client\n");
			} else {
				if(debug) printf("[-] No file found\n");
				char noFile[] = "No such file";
				int len = strlen(noFile)+1;
				send(new_s, noFile, len, 0);
			}

		}
		if( debug ) printf("[+] Client closed connection\n");
		close(new_s);
	}

	freeaddrinfo(result);
	close(s);

	return 0;
}
