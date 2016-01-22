#pragma once
#include <vector>
#include <string>
#include <unordered_map>
 
using namespace std;

//#define PRE_PROCESSED_DISTANCES 250

#ifdef WIN32
#define LIMITGIFTS 100000
#else
#define LIMITGIFTS 100000
#endif
//100000
//1000

#define CRITERIA_DISTANCE 0
#define CRITERIA_ANGLE 1
#define CRITERIA_ANGLE_DISTANCE 2
#define CRITERIA_FAR_SEED_GREEDY 3
#define CRITERIA_FAR_SEED_GREEDY_WD 4
#define CRITERIA_SolutionKytojoki2007 5
#define CRITERIA_FAR_SEED_GREEDY_COST_PERCENT 6
#define CRITERIA_FAR_SEED_GREEDY_LIMIT_TRIPS 7
#define CRITERIA_FAR_SEED_GREEDY_EXTENSIVE 8
#define CRITERIA_OLD_SOLUTION 9

class Solution;
class Trip;

class Gift
{
public:
	Gift(int id_, double latitude_, double longitude_, double weight_);
	double Distance(Gift gift);
	int id;
	double latitude;
	double longitude;
	double weight;
	double npDistance;

	double latitudeInRads;
	double longitudeInRads;
	double x;
	double y;
	double z;

	bool isPolar();


	//#ifdef _WIN32
	bool operator < (const Gift& gift) {
		return (longitude < gift.longitude);    
    }
	/*#else
	bool operator < (const Gift& gif1, const Gift& gift2) {		
		return (gif1.longitude < gift2.longitude);   
    }
	#endif*/
};

class HugeDirectedGraph
{
public:
	HugeDirectedGraph();
	//void addVertice(int id);
	void setVertices(int size);
	void addArc(int indexFrom, int indexTo, double distance);
	int numVertices;
	int numArcs;

	//a container containing only distances for reasonably distanced vertices
	vector<unordered_map<int,double>> idDistance;

private:

};

class Instance
{
public:
	Instance(std::string instanceFilename, double weightLimit, double benchmark, double limitAdjacent);
	double EvaluateSol(Solution &sol);
	double CalculateTripCost(Trip &trip);
	double EvaluateTripForceLast(Trip &trip, int giftLast);
	
	void optKara2007();

	bool Compatible(Gift & gift1, Gift & gift2, int criteria, double threeshold);
	double howAligned(Gift & gift1, Gift & gift2);
	double angle(Gift & gift1, Gift & gift2);
	double weightLimit;
	int n; //number of gifts
	double sleighWeight;

	int getGiftId(int index);
	double getWeight(int index);

	HugeDirectedGraph world;

	std::vector<Gift> Gifts;
	std::vector<Gift> UnsortedGifts;

	//long int distanceCalculations;
	double getDistance(int idGift1, int idGift2);
	double getHaversineDistance(double lat1, double lon1, double lat2, double lon2);
	
private:

};



