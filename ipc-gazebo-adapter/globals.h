#include <string>
#include <map>
#include <memory>
#include <mutex>

class AbstractPipe;

extern std::map<std::string, std::unique_ptr<AbstractPipe> > pipe_by_name;
extern std::mutex global_mutex;