#include "globals.h"
#include "transport_pipe.h"

std::map<std::string, std::unique_ptr<AbstractPipe> > pipe_by_name;
std::mutex global_mutex;