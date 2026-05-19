#include <rozeta/gps.hpp>
#include <cstdlib>
#include <sstream>
#include <vector>
namespace rozeta::gps {
static std::vector<std::string> split(const std::string& s){ std::vector<std::string> out; std::stringstream ss(s); std::string item; while(std::getline(ss,item,',')) out.push_back(item); return out; }
static double coord(const std::string& raw,const std::string& hemi){ if(raw.empty()) return 0; double v=std::stod(raw); int deg=int(v/100); double min=v-deg*100; double dec=deg+min/60.0; if(hemi=="S"||hemi=="W") dec=-dec; return dec; }
GpsFix NmeaParser::parseLine(const std::string& line) const { GpsFix f; auto star=line.find('*'); auto clean=line.substr(0, star==std::string::npos?line.size():star); auto p=split(clean); if(p.empty()) return f; auto type=p[0]; if(type.size()>=6 && type.substr(type.size()-3)=="GGA" && p.size()>9){ f.latitude=coord(p[2],p[3]); f.longitude=coord(p[4],p[5]); f.fix_quality=p[6].empty()?0:std::stoi(p[6]); f.satellite_count=p[7].empty()?0:std::stoi(p[7]); f.altitude_m=p[9].empty()?0:std::stod(p[9]); f.valid=f.fix_quality>0; } else if(type.size()>=6 && type.substr(type.size()-3)=="RMC" && p.size()>8){ f.valid=(p[2]=="A"); f.latitude=coord(p[3],p[4]); f.longitude=coord(p[5],p[6]); f.speed_mps=(p[7].empty()?0:std::stod(p[7]))*0.514444; f.course_deg=p[8].empty()?0:std::stod(p[8]); f.fix_quality=f.valid?1:0; } return f; }
LocalCoordinate toLocal(const GeoCoordinate& origin, const GpsFix& fix){ return geoToLocal(origin, {fix.latitude, fix.longitude, fix.altitude_m}); }
}