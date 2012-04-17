/*
 * utils.h
 *
 *  Created on: 17/08/2011
 *      Author: dennislui
 */

#include <iostream>
#include <stdio.h>
#include <stdlib.h>

using namespace std;

#ifndef UTILS_H_
#define UTILS_H_

namespace utils{
unsigned long memoryUsableQuery();
bool checkDirectory(string path, bool create);
}


#endif /* UTILS_H_ */
