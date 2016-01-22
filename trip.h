#pragma once

#include <list>
#include <vector>
#include <string>

//used to avoid trips around -180 to 180 longitudes not interacting
//200 puts the -180 to 180 line over the atlantic sea
#define LONGITUDE_OFFSET 0
#define RELEVANT_INCREASE 1

#define NUM_NEIGHBORHOOD 29
#define NEIGHBORHOOD_FIRST_INTER 9

	//intra
	#define NEIGHBORHOOD_2Opt 0
	#define NEIGHBORHOOD_3Opt 1
	#define NEIGHBORHOOD_4Opt 2
	#define NEIGHBORHOOD_5Opt 3
	#define NEIGHBORHOOD_IntraRelocation  4
	#define NEIGHBORHOOD_IntraExchange  5
	#define NEIGHBORHOOD_IntraChainRelocation  6
	#define NEIGHBORHOOD_Segregation  7
	#define NEIGHBORHOOD_KARA  8

	//inter
	#define NEIGHBORHOOD_2OptStar  9
	#define NEIGHBORHOOD_InterRelocation  10
	#define NEIGHBORHOOD_2InterRelocation  11
	#define NEIGHBORHOOD_InterChainRelocation  12
	#define NEIGHBORHOOD_CrossExchange  13
	#define NEIGHBORHOOD_IntenseInterChainRelocation  14
	#define NEIGHBORHOOD_IntenseCrossExchange  15
	#define NEIGHBORHOOD_Merge2  16
	#define NEIGHBORHOOD_Merge3  17
	#define NEIGHBORHOOD_Merge4  18
	#define NEIGHBORHOOD_Merge5  19
	#define NEIGHBORHOOD_Net1to1Relocation  20
	#define NEIGHBORHOOD_NetRelocation  21
	#define NEIGHBORHOOD_BiCrossExchange  22
	#define NEIGHBORHOOD_NetCrossExchange  23
	#define NEIGHBORHOOD_AllNetCrossExchange 24
	#define NEIGHBORHOOD_Merge2Polar 25
	#define NEIGHBORHOOD_Condense3  26
	#define NEIGHBORHOOD_Condense4  27
	#define NEIGHBORHOOD_Condense5  28

class Instance;
class Trip;
class Gift;

using namespace std;

class Trip
{
public:
	int longitudeOrder;

	//constructors
	Trip(Instance & inst, Gift & initialGift);
	Trip(Instance & inst, int initialGiftId);
	Trip(Instance & inst);
	Trip (const Trip &copyTrip);

	//optimization
	void optTSP(Instance & inst);
	void optKara2007(Instance & inst, bool & improveFlag);
	double gapKara2007(Instance & inst);

	//local search procedures
	void search2Opt(Instance & inst, bool & improveFlag, double & account, bool showLog = false);
	void search3Opt(Instance & inst, bool & improveFlag, double & account, bool showLog = false);
	void search4Opt(Instance & inst, bool & improveFlag, double & account, bool showLog = false);
	void searchIntraRelocation(Instance & inst, bool & improveFlag, double & account, bool showLog = false);
	void searchIntraChainRelocation(Instance & inst, bool & improveFlag, double & account, bool showLog = false);
	void searchIntraExchange(Instance & inst, bool & improveFlag, double & account, bool showLog = false);
	double costOfChain(int startGiftIndex, int endGiftIndex, double& weightAtStart, bool inverted=false);
	double costOfArc(Instance & inst, int fromGiftIndex, int toGiftIndex, double& weightAtStart);

	//setting
	void setCost(double cost_);
	void setGift(Instance & inst, int index, Gift & gift, bool update = true);
	void UpdateAuxiliaryDataStructures(Instance & inst);
	void cloneTrip(Instance & inst, Trip & newTrip);
	void CalculateSmallestWeight();
	void CalculateMassCenter();
	void setNeighborhoodCleared(int NeighborhoodCode);
	void setNeighborhoodPending(int NeighborhoodCode);
	void setNeighborhoodsAllCleared();
	void resetNeighborhoodStatuses();
	void resetInterNeighborhoods();
	void CalculateCoordinateRange();
	void setTabu();
	void clearTabu();
	void setMark(int position);

	//evaluation
	bool LastCompatible(Gift & newGift, Instance & inst, int criteria, double threeshold);
	bool FullyCompatible(Gift & newGift, Instance & inst, int criteria, double threeshold);
	bool SomeCompatible(Gift & newGift, Instance & inst, int criteria, double threeshold);

	//movements evaluations
	double evaluateSetGift(Instance & inst, int index, Gift & gift);
	double evaluateInsertion(Instance & inst, int index, Gift & gift);
	double evaluateEraseGift(Instance & inst, int index);
	double evaluateInsertionLength(Instance & inst, int index, Gift & gift);
	double evaluate2Opt(Instance & inst, int i, int j);
	double evaluate3Opt(Instance & inst, int i, int j, int k, int & identifier);
	double evaluate4Opt(Instance & inst, int i, int j, int k, int l, int & identifier);
	double evaluate5Opt(Instance & inst, int i, int j, int k, int l, int m, int & identifier);
	double evaluateIntraChainRelocation(Instance & inst, int i, int k, int j);

	//gifts manipulation
	void eraseGift(Instance & inst, int index, bool update = true);
	void appendGift(Instance & inst, Gift & gift);
	void appendGift(Instance & inst, int giftId);
	void swapGifts(Instance & inst, int index1, int index2);
	void insertGift(Instance & inst, int indexBefore, Gift & gift, bool update = true);
	void popBackGift(Instance & inst);
	void move2Opt(Instance & inst, int i, int j);
	void move3Opt(Instance & inst, int i, int j, int k, int identifier);
	void move4Opt(Instance & inst, int i, int j, int k, int l, int identifier);
	void move5Opt(Instance & inst, int i, int j, int k, int l, int m, int identifier);
	void moveIntraChainRelocation(Instance & inst, int i, int k, int j);
	void SortDistanciaNP();

	//queries
	Gift & getGift(int index);
	Gift & getBackGift();
	Gift & getFrontGift();
	double getLength();
	bool isTabu();	
	int getMark();

	//return the weight of the chain after delivering the gift given by the index
	//first gift index = 0
	//last gift index = getSize() - 1
	//use excludeSleigh = true to return only the gifts weight
	double getWeightAt(int index, bool excludeSleigh = false);

	double getCostFrom(int index);
	double getDistanceUntil(int index);
	double getDistanceFrom(int index);
	double getDistanceAhead(int index);
	double getCostUntil(int index);
	double getCost() const;
	double getCostAhead(int index);
	double avgLongitude() const;
	double avgMCLongitude() const;
	double getGiftLatitude(int index) const;
	double getGiftLongitude(int index) const;
	double getSmallestWeight();
	double getLatitude_MassCenter();
	double getLongitude_MassCenter();
	bool getNeighborhoodStatus(int NeighborhoodCode);	
	double getMinLatitude();
	double getMaxLatitude();
	double getMaxLongitude();
	double getMinLongitude();
	double getGiftBadness(int index);
	bool isPolar();

	//a trip has size gifts
	//size + 1 positions from 0 (north pole) to size (last gift)
	//and size + 1 arcs from (0,1) to (size,0)
	int getSize();
	int getSize() const;
	double getCost();
	double getWeight();
	double getPolarWeight();
	//gift weight
	double getWeight(int index); 
	double getRealWeight(); 
	int getGiftId(int index);

	//output
	std::string routeToGeoJson(int id);
	std::string pointsToGeoJson();
	std::string printTrip();
	string printTripDebug();
	string printTripQuality();

	//operators for comparison
	bool operator < (const Trip& trip) {
		return (cost > trip.cost);    //descending 
    }

	bool improvedByPerturbation;
private:
	bool tabu;
	std::vector<Gift> Gifts;
	
	vector<bool> NeighborhoodStatus;	
	
	//auxiliary data structures (ADS's)

	//number of gifts
	//if number of gifts is n
	//the number of point of the trip is n+1
	//and number of arcs is also n+1
	int size;
	double weight;
	double smallestWeight;
	double cost;

	//stores the weight at each trip position
	//position 0 stores the full weight of the trip
	//position 1 stores decrements the weight of the first gift
	//...
	//postion size-1 (last) is the weight of the sleigh
	vector<double> weightAt;

	//stores the distance from point i
	//to point i+1 of the trip
	//position 0 stores the distance from north pole to first gift
	//position 1 stores the distance from gift 1 to gift 2
	//...
	//position size stores the distance from last gift to north pole
	vector<double> distanceFrom;

	//stores the cost of going from point i
	//to point i+1 of the trip
	//position 0 stores the cost from north pole to first gift
	//position 1 stores the cost from gift 1 to gift 2
	//...
	//position size stores the cost from last gift to north pole
	vector<double> costFrom;

	//stores the distance of getting to point i
	//from the North Pole
	//position 0 stores the total distance from north pole to first gift
	//position 1 stores the distance total from north pole to second gift
	//...	
	//position size stores the distance total from north pole to north pole
	vector<double> distanceUntil;

	//stores the distance of getting from point i to the North Pole
	//position 0 stores the total distance from north pole to north pole
	//position 1 stores the distance total from first gift to north pole
	//...	
	//position size stores the distance total from last gift to north pole
	vector<double> distanceAhead;

	//stores the cost of getting to point i
	//from the North Pole
	//position 0 stores the total cost from north pole to first gift
	//position 1 stores the total cost from north pole to second gift
	//...	
	//position size stores the distance total from north pole to north pole
	vector<double> costUntil;

	//stores the cost of getting from point i to the North Pole
	//position 0 stores the total cost from north pole to north pole
	//position 1 stores the total cost from first gift to north pole
	//...	
	//position size stores the distance total from last gift to north pole
	vector<double> costAhead;

	//not used
	double maxLatitude;
	double minLatitude;
	double maxLongitude;
	double minLongitude;
	
	double latitude_MassCenter;
	double longitude_MassCenter;
};

double evaluateCrossExchange(Instance & inst, Trip & trip1, Trip & trip2, int i, int k, int j, int l, int& sense);
double evaluateCrossExchangeArtur(Instance & inst, Trip & trip1, Trip & trip2, int i, int k, int j, int l, bool& over1, bool& over2);
void moveCrossExchange(Instance & inst, Trip & trip1, Trip & trip2, int i, int k, int j, int l, int sense);

double evaluate2OptStar(Instance & inst,  Trip & trip1, Trip & trip2, int i, int j);
void move2OptStar(Instance & inst,  Trip & trip1, Trip & trip2, int i, int j);

double evaluate2InterRelocation(Instance & inst,  Trip & tripFrom, Trip & tripTo, int i, int j);
void move2InterRelocation(Instance & inst,  Trip & tripFrom, Trip & tripTo, int i, int j);

double evaluateInterChainRelocation(Instance & inst, Trip & trip1, Trip & trip2, int i, int k, int j);
void moveInterChainRelocation(Instance & inst, Trip & trip1, Trip & trip2, int i, int k, int j);

double evaluateIntraChainRelocation(Instance & inst, Trip & trip1, int i, int k, int j);
void moveIntraChainRelocation(Instance & inst, Trip & trip1, int i, int k, int j);


double InterMassCenterDistance(Instance & inst, Trip & trip1, Trip & trip2);

double evaluateBiCrossExchange(Instance & inst, Trip & trip1, Trip & trip2, int i, int k, int m, int o, int j, int l, int n, int p, int& sense1, int& sense2);
void moveBiCrossExchange(Instance & inst, Trip & trip1, Trip & trip2, int i, int k, int m, int o, int j, int l, int n, int p, int sense1, int sense2);