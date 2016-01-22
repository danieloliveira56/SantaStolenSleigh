CPXLDIR = /opt/ibm/ILOG/CPLEX_Studio1251/cplex/lib/x86-64_sles10_4.1/static_pic/
CPXIDIR = /opt/ibm/ILOG/CPLEX_Studio1251/cplex/include/
CNCRTDIR = /opt/ibm/ILOG/CPLEX_Studio1251/concert/include/
CNCRTLIB = /opt/ibm/ILOG/CPLEX_Studio1251/concert/lib/x86-64_sles10_4.1/static_pic/
CXX = g++
MY_LIBS   = -lilocplex -lcplex -lpthread -lconcert -L $(CPXLDIR) -L $(CNCRTLIB) 
MY_CFLAGS = -DCPX -DIL_STD
CXXFLAGS= -O3 -o santastolensleigh SantaStolenSleigh.cpp solution.cpp instance.cpp CPUTimer.cpp trip.cpp
#CXXFLAGS= -O0 -g -o  bctimeindexed BCTimeIndexed.cpp
CPPFLAGS  = -Wall -I $(OSIDIR)include -I $(CPXIDIR) -I $(CNCRTDIR) -std=c++11 -fopenmp -fpermissive
LINK.cxx = $(CXX) $(MY_CFLAGS) $(CXXFLAGS) $(CPPFLAGS)

all:
	$(LINK.cxx) $(MY_LIBS)