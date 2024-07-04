#include "log.h"

int create_file(const char *path)
{
    FILE *fp = fopen(path, "w");
    fclose(fp);
}

void save_timestamp(const char *path, int wpt, unsigned long time)
{
    FILE *fp = fopen(path, "a+");
    fprintf(fp, "%d,%ld\n", wpt, time);
    fclose(fp);
}