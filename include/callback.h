#ifndef _CALLBACK_H
#define _CALLBACK_H

#include "gurobi_c++.h"
#include <fstream>
#include <cmath>
#include "parameter.h"

class print_callback: public GRBCallback{
  public:
    std::ofstream* logfile;
    double runtime;
    print_callback(std::ofstream* xlogfile);
  protected:
    void callback();
};

#endif

