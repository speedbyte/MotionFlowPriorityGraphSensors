// A simple program that computes the square root of a number
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "Tutorial.hpp"
#include "TutorialConfig.h"
#include "mysqrt.hpp"

int main (int argc, char *argv[])
{
  if (argc < 2)
    {
    fprintf(stdout,"%s Version number is : %d\n",argv[0], VERSION_NUMBER);
    fprintf(stdout,"Usage: %s number\n",argv[0]);
    return 1;
    }
  double inputValue = atof(argv[1]);
#ifdef USE_MYMATH
  double outputValue = mysqrt(inputValue);
  fprintf(stdout,"Using my library");
#else
  double outputValue = sqrt(inputValue);
  fprintf(stdout,"Using math library");
#endif
 
  fprintf(stdout,"The square root of %g is %g\n",
          inputValue, outputValue);
  return 0;
}
