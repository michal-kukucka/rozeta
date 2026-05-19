#include <rozeta/logging.hpp>
#include <chrono>
#include <iostream>
namespace rozeta::logging {
static std::shared_ptr<Logger>& active(){ static auto l=std::shared_ptr<Logger>(new ConsoleLogger()); return l; }
const char* levelName(Level level){ switch(level){case Level::Debug:return "debug";case Level::Info:return "info";case Level::Warning:return "warning";case Level::Error:return "error";} return "unknown"; }
void ConsoleLogger::log(Level level,const std::string& channel,const std::string& message){ std::cerr << levelName(level) << "," << channel << "," << message << "\n"; }
CsvFileLogger::CsvFileLogger(const std::string& path):file_(path, std::ios::app){ if(file_.tellp()==0) file_ << "level,channel,message\n"; }
void CsvFileLogger::log(Level level,const std::string& channel,const std::string& message){ std::lock_guard<std::mutex> lock(mutex_); file_ << levelName(level) << ',' << channel << ',' << '"' << message << '"' << "\n"; file_.flush(); }
void setLogger(std::shared_ptr<Logger> logger){ active()=std::move(logger); }
std::shared_ptr<Logger> getLogger(){ return active(); }
void log(Level level,const std::string& channel,const std::string& message){ if(active()) active()->log(level,channel,message); }
}