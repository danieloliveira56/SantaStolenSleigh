#ifdef _WIN32
#include "stdafx.h"
#endif

#include "trip.h"
#include "solution.h"
#include "instance.h"
#include "CPUTimer.h"

#include <algorithm>
#include <iostream>
#include <iomanip>
#include <sstream> 
#include <omp.h>

#include <fstream>

using namespace std;

void studyCompatibity(Instance & inst);
bool distanciaNP(const Gift& gift1, const Gift& gift2);
bool reverseDistanciaNP(const Gift& gift1, const Gift& gift2);

void searchInterExchange(Instance & inst, Trip & trip1, Trip & trip2, bool & improveFlag, double & account, bool showLog = false);
void searchSegregation(Instance & inst, Trip & trip, vector<Trip> & newTrips, bool & improveFlag, double & account, bool showLog = false);
void searchInterRelocation(Instance & inst, Trip & tripFrom, Trip & tripTo, bool & improveFlag, double & account, bool showLog = false);
void searchCrossExchange(Instance & inst, Trip & trip1, Trip & trip2, bool & improveFlag, double & account, int index1, int index2, bool endOfRoute, bool showLog = false);
void search2OptStar(Instance & inst, Trip & trip1, Trip & trip2, bool & improveFlag, double & account, bool showLog = false);
void search2InterRelocation(Instance & inst, Trip & tripFrom, Trip & tripTo, bool & improveFlag, double & account, bool showLog = false);
void searchInterChainRelocation(Instance & inst, Trip & tripFrom, Trip & tripTo, bool & improveFlag, double & account, bool showLog = false);
void SearchMerge2(Instance & inst, Trip & trip1, Trip & trip2, Trip & newTrip, bool & improveFlag, double & account, bool showLog = false);
void SearchMerge3(Instance & inst, Trip & trip1, Trip & trip2, Trip & trip3, Trip & newTrip, bool & improveFlag, double & account, bool showLog = false);
void SearchNet1to1Relocation(Instance & inst, Trip & trip1, Trip & trip2, bool & improveFlag, double & account, bool showLog = false);
void SearchNetRelocation(Instance & inst, Trip & tripFrom, vector<Trip*>& ptr_kCloseTrips, bool & improveFlag, double & account, bool showLog = false);

void searchCrossExchangeArtur(Instance & inst, Trip & trip1, Trip & trip2, bool & improveFlag, double & account, int index1, int index2, bool showLog=false);

void distanceComplexityStudy(Instance & inst);

void MoveMultiExchange(Instance& inst, Solution& sol,vector<GiftChain>& exchangePath, double costDiff);
void SearchMultiExchange(Instance &inst, Solution& sol, ExchangeGraph& solExchangeGraph, vector<vector<GiftChain>>& exchangePaths, vector<double>& costDiffs, ofstream& log, double & account);
void printExchangePath(vector<GiftChain> exchangePath);

void searchBiCrossExchange(Instance & inst, Trip & trip1, Trip & trip2, bool & improveFlag, double & account, bool showLog = false);

void SearchNetCrossExchange(Instance & inst, Trip & tripFrom, vector<Trip*>& ptr_kCloseTrips, bool & improveFlag, double & account, bool showLog = false);
void SearchAllNetCrossExchange(Instance& inst, vector<Trip*>& ptr_Trips, bool& improveFlag, double& account, bool showLog=false);

void SearchMerge2Polar(Instance & inst, Trip & trip1, Trip & trip2, bool & improveFlag, double & account, bool & feasible, bool force = false, bool showLog=false);

//perturbations
void PerturbRandomCrossExchange(Instance& inst, Solution& sol, int cxLevel, bool& hasPerturbation);
void PerturbExplode(Instance& inst, Solution& sol, bool& hasPerturbation);
void PerturbMerge2Polar(Instance& inst, Solution& sol, bool& hasPerturbation);
void PerturbMerge3(Instance& inst, Solution& sol, bool& hasPerturbation);
void PerturbMerge2(Instance& inst, Solution& sol, bool& hasPerturbation);
void PerturbSplitLongest(Instance& inst, Solution& sol, bool& hasPerturbation);
void PerturbHoldSP(Instance& inst, Solution& sol, bool& hasPerturbation);
void PerturbRandomShift(Instance& inst, Solution& sol, bool& hasPerturbation);
void PerturbRandomExchange(Instance& inst, Solution& sol, bool& hasPerturbation);
void PerturbSplitCostlier(Instance& inst, Solution& sol, bool& hasPerturbation);


void searchIntra_Controller(Instance& inst, Solution& sol, vector<double>& candidateSearchAccount, ofstream& log);
void searchNetRelocation_Controller(Instance& inst, Solution& sol, bool& overallImproveFlag, vector<double>& searchAccount, ofstream& log);
void searchCrossExchange_Controller(Instance& inst, Solution& sol, bool& overallImproveFlag, vector<double>& searchAccount, ofstream& log, bool permitsNewTrips = true, bool sloopy=false);
void searchNetCrossExchange_Controller(Instance& inst, Solution& sol, bool& overallImproveFlag, vector<double>& searchAccount, ofstream& log);
void searchAllNetCrossExchange_Controller(Instance& inst, Solution& sol, bool& overallImproveFlag, vector<double>& searchAccount, ofstream& log);
void searchMerge2Polar_Controller(Instance& inst, Solution& sol, bool& overallImproveFlag, vector<double>& searchAccount, ofstream& log);
void searchMerge2_Controller(Instance& inst, Solution& sol, bool& overallImproveFlag, vector<double>& searchAccount, ofstream& log);
void searchMerge3_Controller(Instance& inst, Solution& sol, bool& overallImproveFlag, vector<double>& searchAccount, ofstream& log);
void searchMerge4_Controller(Instance& inst, Solution& sol, bool& overallImproveFlag, vector<double>& searchAccount, ofstream& log);
void searchMerge5_Controller(Instance& inst, Solution& sol, bool& overallImproveFlag, vector<double>& searchAccount, ofstream& log);
void SearchCondense(Instance & inst, vector<Trip*>& ptr_Trips, vector<Trip*>& ptr_newTrips, bool & improveFlag, vector<double> & account, bool showLog);
void searchCondense3_Controller(Instance& inst, Solution& sol, bool& overallImproveFlag, vector<double>& searchAccount, ofstream& log);

void setPertubationProbabilities();