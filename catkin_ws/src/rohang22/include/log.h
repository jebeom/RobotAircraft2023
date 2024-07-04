#ifndef __LOG_H__
#define __LOG_H__

#include <mavros_msgs/FileOpen.h>
#include <mavros_msgs/FileClose.h>
#include <mavros_msgs/FileWrite.h>

#include <stdio.h>

#define MODE_READ 0
#define MODE_WRITE 1
#define MODE_CREATE 2

int create_file(const char *path);
void save_timestamp(const char *path, int wpt, unsigned long time);


#endif