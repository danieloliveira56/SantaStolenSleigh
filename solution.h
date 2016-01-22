#pragma once

#include "solution.h"

#include <list>
#include <vector>
#include <string>
#include <fstream>

using namespace std;

class Instance;
class Trip;
class Gift;

class PairIdDistance
{
public:
	PairIdDistance(int id_, double distance_);
	int id;
	double distance;

	bool operator < (const PairIdDistance& comp) {
		return (distance < comp.distance);    
    }
};

#define NUM_PERTURBATIONS 10
	#define PERTURBATION_2MERGE 0
	#define PERTURBATION_3MERGE 1
	#define PERTURBATION_RANDOM_CROSS_EXCHANGE 2
	#define PERTURBATION_EXPLODE 3
	#define PERTURBATION_HOLD_SOUTH_POLE 4
	#define PERTURBATION_MERGE_2POLAR 5
	#define PERTURBATION_SPLIT_LONGEST 6
	#define PERTURBATION_RANDOM_SHIFT 7
	#define PERTURBATION_RANDOM_EXCHANGE 8
	#define PERTURBATION_SPLIT_COSTLIER 9


class CandidatePerturbation
{
public:
	CandidatePerturbation(int Perturbation_Code_, int arg1 = 0, int arg2 = 0, int arg3 = 0, int arg4 = 0, int arg5 = 0, int arg6 = 0, double priority_ = 0);
	int Perturbation_Code;
	double priority;
	int index1;
	int index2;
	int index3;

	int i;
	int j;
	int k;
	int l;

	bool operator < (const CandidatePerturbation& comp) {
		return (priority < comp.priority);    //sort ascending  
    }
};


class Solution
{
public:
	Solution();
	Solution(Instance & inst);
	Solution(Instance & inst, int criteria, double criteriaArg1, double criteriaArg2, string strArg);
	Solution (const Solution &copySolution);

    void SolutionBySeedGreedy(Instance & inst, double threeshold);
    void SolutionKytojoki2007(Instance & inst, int kLSFrequency, double deltaEmptyness);
	void SolutionBySeedGreedyWL(Instance & inst, double maxWeightLengthIncrease);
	void SolutionBySeedGreedyCostPercent(Instance & inst, double maxPercentual);
	void SolutionBySeedGreedyTripsLimit(Instance & inst, int tripsLimit);
    void SolutionBySeedGreedyExtensive(Instance & inst, double threeshold);	
	void ReadOldSolution(Instance & inst, string solutionFileName);
	void ClearEmptyTrips();
	void ValidateWeights(Instance & inst);
	void ValidateCost(Instance & inst);
	void ValidateGifts(Instance & inst, int initialNumber);
	void LocalSearch(Instance & inst);
	void resetInterNeighborhoods();
	void setNeighborhoodsAllCleared();

	void clearTabu();
	double getCost();
	
	void SortCost();
	void SortDescendingCost();
	void SortLongitude();
	void SortMCLongitude();
	void RandomSort();
	void MarkPositions();
	double getTripPairOffset(int index1, int index2);

	int getNumGifts();

	string toCSV();
	string toPolarCSV(bool onlyPolars);
	void toCSVFile(string fileName);
	void toPolarCSVFile(string fileName, bool onlyPolars);

	void toRoutesFolder(string solutionsFolder);
	
	//data
	std::vector<Trip> Trips;
	Instance* ptr_Instance;
};

class ExchangeArc
{
public:
	ExchangeArc(int t, double c): tail(t), cost(c) {}
	int getTail() { return tail; }
	double getCost() { return cost; }

private:
	int tail;
	double cost;
};

class GiftChain
{
public:
	//used to create an atificial node
	GiftChain();
	GiftChain(Solution& sol, int tripIndex, int l1_, int l2_);
	double get_q();
	int get_k();
	int get_l1();
	int get_l2();
	double get_routeWeight();
	void addEastArc(int tail, double cost);
	void addWestArc(int tail, double cost);
	int getEastDegree();
	int getWestDegree();
	ExchangeArc& getEastArc(int index);
	ExchangeArc& getWestArc(int index);
	string print();

	bool operator== ( GiftChain& other );

private:
	//index of the trip in the solution
	int k;

	//index of gift before first gift in the chain
	int l1;

	//index of last gift of the chain
	//if l1 == l2, empty chain
	int l2;

	//weight of the gift chain
	double q;

	//current route weight, excluding the sleigh
	double routeWeight;

	//list of adjacent arcs to the east and west
	vector<ExchangeArc> eastArcs;
	vector<ExchangeArc> westArcs;
};



class ExchangeGraph
{
public:
	ExchangeGraph(Instance& inst, Solution& sol, int movementReach, int numChainLimits, ofstream& log);

	vector<GiftChain> V;
	vector<int> tripStarts;

private:
};




