/*
 * utils.cpp
 *
 *  Created on: 17/08/2011
 *      Author: dennislui
 */
#include "utils.h"
#include <sys/sysinfo.h>
#include <sys/stat.h>
#include <sys/types.h>

unsigned long utils::memoryUsableQuery()
{
	struct sysinfo myinfo;
	unsigned long total_bytes;

	sysinfo(&myinfo);
	total_bytes = (myinfo.freeram + myinfo.bufferram) * myinfo.mem_unit;
	printf("total available main memory is %lu B, %lu MB\n",
		         total_bytes, total_bytes/1024/1024);

	return total_bytes;
}

//check whether directory exists, if it doesn't, create a new directory if boolean set to true
bool utils::checkDirectory(string path, bool create)
{
	struct stat statbuf;
	bool exist = true;
	if (stat(path.c_str(), &statbuf) == -1) {
		if(create)
			mkdir(path.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
		exist = false;
	}

	return exist;
}
