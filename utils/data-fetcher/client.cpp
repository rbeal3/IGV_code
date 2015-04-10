/* This code is adopted from the sample provided by Prof. Kredo 
 * 
 * Author: Sterling Baldwin
 * Also: Naomi Miller
 */


#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netdb.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h> 
#include <errno.h> 

#define SERVER_PORT "8000"
#define MAX_LINE 256

int
main(int argc, char *argv[])
{
	struct addrinfo hints;
	struct addrinfo *rp, *result;
	char* host;
	char* file_path;
	char* port;
	int debug = false;
	if (argc>3)
	{
		host = argv[1];
		port = argv[2];
		file_path = argv[3] + '\0';
		if( argc == 5 &&  (strcmp(argv[4], "-d") == 0) ) debug = true;
	}
	else
	{
		fprintf(stderr, "usage: %s <host> <port> <file_path>\n", argv[0]);
		exit(1);
	}
	if( debug ) printf("[+] Launching client in debug mode\n");
	int s;
	
	int buflen;
	char buf[MAX_LINE];
	
	int reqlen;
	ssize_t recvlen;
	
	int fd;
	int writelen;
	
	memset(&hints, 0, sizeof(hints));
	hints.ai_family = AF_INET;
	hints.ai_socktype = SOCK_STREAM;
	hints.ai_flags = 0;
	hints.ai_protocol = 0;

	if ((s = getaddrinfo(host, port, &hints, &result)) != 0 )
	{
		printf("Client Error: Unable to find host ‘%s'\n", host);
		exit(1);
	}

	/* Iterate through the address list and try to connect */
	for (rp = result; rp != NULL; rp = rp->ai_next)
	{
		if ((s = socket(rp->ai_family, rp->ai_socktype, rp->ai_protocol)) == -1 )
		{
			continue;
		}

		if (connect(s, rp->ai_addr, rp->ai_addrlen) != -1)
		{
			break;
		}

		close(s);
	}
	if (rp == NULL)
	{
		perror("project2 client: connect\n");
		exit(1);
	}
	freeaddrinfo(result);
	

	reqlen = strlen(file_path)+1;
	if( debug ) printf("[+] Sending message: %s\n", file_path);
	char noFile[] = "No such file";
	send(s, file_path, reqlen, 0); //send the requested file_path to the server

	memset(buf,0,strlen(buf));
	buflen = MAX_LINE;
	recvlen = recv(s, buf, buflen,0); // wait until the server responds 
	if(debug) printf("[+] got message from server %s", buf);
		for (int i =0; i < 10; i++) {
			printf("[+] thing at buf %d is %c \n", i , buf[i]); 
	}
	FILE * gpsFile;
	if(strcmp(noFile, buf) == 0) {
		printf("Server Error: Unable to access file ‘%s'\n", file_path);
		return 0;
	}
	else {
		//Open file for the returned data from server
		 printf("opening file \n"); 
    gpsFile = fopen( "gpsData.txt", "w+" );
		if( ( fd = open(file_path, O_WRONLY | O_CREAT | O_EXCL, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH) ) == -1)
			{fd = open(file_path, O_WRONLY | O_TRUNC, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);}
		else
		{ printf("fd failed with this return value: %d \n", fd); }

		 printf("You should know fd=%d \n", fd); 
			if(recvlen > 0)
				writelen = fwrite(buf, sizeof(char), recvlen, gpsFile);
			recvlen = recv(s, buf, buflen,0); // wait until the server responds 
			if( debug ) printf("got respose from the server '%s'\n", buf);
			if(debug){
				for (int i =0; i < 10; i++) {
					printf("[+] thing at buf %d is %c \n", i , buf[i]); }
			}
	}
	 
	
	if(recvlen == 0)
		printf("recv closed normally");
	else if(recvlen < 0)
		printf("recv err %i", errno);
	else if(writelen < 0) 
		printf("recv err %i", errno);
	
	
	fclose(gpsFile);
	close(fd);
	close(s);

	return 0;
}
