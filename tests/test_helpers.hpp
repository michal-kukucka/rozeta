#pragma once
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <string>

#define REQUIRE_TRUE(expr) do { if(!(expr)) throw std::runtime_error(std::string("Requirement failed: ") + #expr); } while(false)
#define REQUIRE_EQ(a,b) do { auto _a=(a); auto _b=(b); if(!(_a==_b)) { std::cerr << "Expected equality: " << #a << " == " << #b << " got " << _a << " vs " << _b << "\n"; throw std::runtime_error("equality failed"); } } while(false)
#define REQUIRE_NEAR(a,b,eps) do { double _a=(a); double _b=(b); if(std::fabs(_a-_b)>(eps)) { std::cerr << "Expected near: " << #a << " ~= " << #b << " got " << _a << " vs " << _b << "\n"; throw std::runtime_error("near failed"); } } while(false)
