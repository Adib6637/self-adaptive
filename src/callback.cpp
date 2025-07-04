#include "callback.h"
print_callback::print_callback(std::ofstream* xlogfile) {
  logfile = xlogfile;
  runtime = 0.0;
}

void print_callback::callback() {
#ifdef OPTIMIZATION_TIMING_LOG
  try {
    if (where == GRB_CB_MIPSOL){
        runtime = getDoubleInfo(GRB_CB_RUNTIME);
        //std::cout << "Callback called at runtime: " << runtime << std::endl;
        *logfile << runtime << "\n";
    }
  } catch (GRBException e) {
    std::cout << "Error number: " << e.getErrorCode() << std::endl;
    std::cout << e.getMessage() << std::endl;
  } catch (...) {
    std::cout << "Error during callback" << std::endl;
  }
#endif
}

