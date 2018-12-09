#include <stdio.h>
#include <time.h>
#include <stdlib.h>

int main (int argc, char **argv)
{
    time_t rawtime;
    struct tm * timeinfo;
    FILE *outfile;

    if(argc < 2) {
        printf("Usage:\n\t%s <file name>\n", argv[0]);
        exit(1);
    }

    // open file for writing
    outfile = fopen (argv[1], "w");
    if (outfile == NULL)
    {
        fprintf(stderr, "\nError opening file\n"); 
        exit (1); 
    }
    time (&rawtime);
    timeinfo = localtime (&rawtime);

    // write struct to file
    fprintf(outfile, "SET_DATE");
    fwrite (&rawtime, sizeof(rawtime), 1, outfile);
    printf ("Current local time and date: %s", asctime(timeinfo));

    // close file
    fclose (outfile);
    return 0;
}
