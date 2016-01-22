#ifndef EC2
#include "ilcplex/cplex.h"
#include "ilcplex/ilocplex.h"

typedef IloArray<IloNumVarArray> NumVarMatrix;
typedef IloArray<IloNumArray> NumMatrix;
#endif

#include "trip.h"
#include "instance.h"
#include "CPUTimer.h"

#include <iterator>
#include <set>
#include <list>
#include <vector>
#include <string>
#include <iostream>
#include <sstream>
#include <algorithm> 
#include <iomanip>

//search inverted chains in Cross Exchange
//#define REVERSE_CX

using namespace std;

Trip::Trip(Instance & inst)
{
	size = 0;
	cost = 0;
	weight = inst.sleighWeight;
	smallestWeight = 0;
	weightAt.push_back(weight);
	distanceFrom.push_back(0);
	costFrom.push_back(0);
	distanceUntil.push_back(0);
	distanceAhead.push_back(0);
	costUntil.push_back(0);
	costAhead.push_back(0);
	NeighborhoodStatus.resize(NUM_NEIGHBORHOOD);
	tabu = false;
}


Trip::Trip (const Trip &copyTrip)
{
	Gifts = copyTrip.Gifts;
	size = copyTrip.size;
	cost = copyTrip.cost;
	weight = copyTrip.weight;
	smallestWeight = copyTrip.smallestWeight;

	weightAt = copyTrip.weightAt;
	distanceFrom = copyTrip.distanceFrom;
	costFrom = copyTrip.costFrom;
	distanceUntil = copyTrip.distanceUntil;
	distanceAhead = copyTrip.distanceAhead;
	costUntil = copyTrip.costUntil;
	costAhead = copyTrip.costAhead;

	NeighborhoodStatus = copyTrip.NeighborhoodStatus; 
	tabu = copyTrip.tabu;
	improvedByPerturbation = copyTrip.improvedByPerturbation;
	
	maxLatitude = copyTrip.maxLatitude;	
	minLatitude = copyTrip.minLatitude;	
	maxLongitude = copyTrip.maxLongitude;	
	minLongitude = copyTrip.minLongitude;	
	latitude_MassCenter = copyTrip.latitude_MassCenter;
	longitude_MassCenter = copyTrip.longitude_MassCenter;
}


bool Trip::getNeighborhoodStatus(int NeighborhoodCode)
{
	return NeighborhoodStatus[NeighborhoodCode];
}

void Trip::resetNeighborhoodStatuses()
{
	for (int i = 0; i < (int)NeighborhoodStatus.size(); i++)
	{
		NeighborhoodStatus[i] = false;
	}
}

void Trip::resetInterNeighborhoods()
{
	for (int i = NEIGHBORHOOD_2OptStar+1; i < (int)NeighborhoodStatus.size(); i++)
	{
		NeighborhoodStatus[i] = false;
	}
}

void Trip::setNeighborhoodCleared(int NeighborhoodCode)
{
	NeighborhoodStatus[NeighborhoodCode] = true;
}

void Trip::setNeighborhoodsAllCleared()
{
	for (int i = 0; i < (int)NeighborhoodStatus.size(); i++)
	{
		NeighborhoodStatus[i] = true;
	}
}

void Trip::setNeighborhoodPending(int NeighborhoodCode)
{
	NeighborhoodStatus[NeighborhoodCode] = false;
}

Trip::Trip(Instance & inst, Gift & initialGift)
{
	size = 0;
	cost = 0;
	weight = inst.sleighWeight;
	weightAt.push_back(weight);
	distanceFrom.push_back(0);
	costFrom.push_back(0);
	distanceUntil.push_back(0);
	distanceAhead.push_back(0);
	costUntil.push_back(0);
	costAhead.push_back(0);
	smallestWeight = initialGift.weight; 
	appendGift(inst, initialGift);
	NeighborhoodStatus.resize(NUM_NEIGHBORHOOD);
	tabu = false;
}

Trip::Trip(Instance & inst, int initialGiftId)
{
	size = 0;
	cost = 0;
	weight = inst.sleighWeight;
	weightAt.push_back(weight);
	distanceFrom.push_back(0);
	costFrom.push_back(0);
	distanceUntil.push_back(0);
	distanceAhead.push_back(0);
	costUntil.push_back(0);
	costAhead.push_back(0);
	smallestWeight = inst.UnsortedGifts[initialGiftId-1].weight; 
	appendGift(inst, inst.UnsortedGifts[initialGiftId-1]);
	NeighborhoodStatus.resize(NUM_NEIGHBORHOOD);
	tabu = false;
}

void Trip::eraseGift(Instance & inst, int index, bool update)
{
	Gifts.erase(Gifts.begin()+index);

	if (update)
	{
		UpdateAuxiliaryDataStructures(inst);
	}
	
}

//retorn a diferenca resultado de se colocar o novo presente no indice index
//se negativa , melhor
double Trip::evaluateEraseGift(Instance & inst, int index)
{
	return - getDistanceUntil(index-1)*(getWeight(index)) //diferenca que o novo peso faz na rota do polo norte até o presente anterior
		- getCostFrom(index-1) - getCostFrom(index)   //diferenca nos dois arcos que morrem
		+ inst.getDistance(getGiftId(index-1), getGiftId(index+1))*(getWeightAt(index-1)-getWeight(index));			
}

//retorn a diferenca resultado de se colocar o novo presente no indice index
//se negativa , melhor
double Trip::evaluate2Opt(Instance & inst, int i, int j)
{
	double totalCost = 0;
	totalCost += getCostUntil(i);

	double  currWeight = getWeightAt(i);
	totalCost += currWeight * inst.getDistance(getGiftId(i), getGiftId(j));


	for (int k = j; k > i+1; k--)
	{
		currWeight -= getWeight(k);
		totalCost += currWeight * getDistanceFrom(k-1);
	}
	currWeight -= getWeight(i+1);
		
	totalCost += currWeight * inst.getDistance(getGiftId(i+1), getGiftId(j+1));

	totalCost += getCostAhead(j+1);

	return totalCost - getCost();
}

//on 3opt, 2 chains are changed
double Trip::evaluate3Opt(Instance & inst, int i, int j, int k, int & identifier)
{
	double aux_weight = 0;
	//1 0...(i-1)
	//2 i...(j-1)
	//3 j...(k-1)
	//4 k...0
	
	int nVariations = 2*2*2; //8
	vector<double> cost(nVariations);

	vector<vector<int>> chainIndex(nVariations);
	for (int i = 0; i < nVariations; i++)
	{
		chainIndex[i].resize(4);
	}
	vector<vector<bool>> chainInverted(nVariations);
	for (int i = 0; i < nVariations; i++)
	{
		chainInverted[i].resize(2);
	}

	int v = 0;
	for (int v1 = 0; v1 < 2*2; v1++)
	{
		chainIndex[v][0] = i;
		chainIndex[v][1] = j-1;
		chainIndex[v][2] = j;
		chainIndex[v][3] = k-1;
		v++;
	}
	for (int v1 = 0; v1 < 2*2; v1++)
	{
		chainIndex[v][0] = j;
		chainIndex[v][1] = k-1;
		chainIndex[v][2] = i;
		chainIndex[v][3] = j-1;
		v++;
	}

	for (int v = 1; v < nVariations; v++)
	{
		if (v % 2 == 1)
		{
			chainInverted[v][0] = true; //not inverted
		}
		if ((1+(v/2)) % 2 == 0) //2,3,6,7 encyclopedia of integer sequences
		{
			chainInverted[v][1] = true; //not inverted
		}
	}
	
	////0: 1->3->2->4
	////0...(i-1)->j...(k-1)->i...(j-1)->k...0
	//chainIndex[0][0] = j;
	//chainIndex[0][1] = k-1;
	//chainInverted[0][0] = false; //not inverted
	//chainIndex[0][2] = i;
	//chainIndex[0][3] = j-1;
	//chainInverted[0][1] = false; //not inverted
	//
	////1: 1->inv(2)->inv(3)->4
	////0...(i-1)->(j-1)...i->(k-1)...j->k...0
	//chainIndex[1][0] = i;
	//chainIndex[1][1] = j-1;
	//chainInverted[1][0] = true; //inverted
	//chainIndex[1][2] = j;
	//chainIndex[1][3] = k-1;
	//chainInverted[1][1] = true; //inverted

	//
	////2: 1->inv(3)->2->4
	////0...(i-1)->(k-1)...j->i...(j-1)->k...0
	//chainIndex[2][0] = j;
	//chainIndex[2][1] = k-1;
	//chainInverted[2][0] = true; //inverted
	//chainIndex[2][2] = i;
	//chainIndex[2][3] = j-1;
	//chainInverted[2][1] = false; //not inverted

	//
	////3: 1->3->inv(2)->4
	////0...(i-1)->j...(k-1)->(j-1)...i->k...0
	//chainIndex[3][0] = j;
	//chainIndex[3][1] = k-1;
	//chainInverted[3][0] = false; //not inverted
	//chainIndex[3][2] = i;
	//chainIndex[3][3] = j-1;
	//chainInverted[3][1] = true; //inverted


	////4: 1->2->inv(3)->4
	////0...(i-1)->i...(j-1)->(k-1)...j->k...0
	//chainIndex[4][0] = i;
	//chainIndex[4][1] = j-1;
	//chainInverted[4][0] = false; //not inverted
	//chainIndex[4][2] = j;
	//chainIndex[4][3] = k-1;
	//chainInverted[4][1] = true; //inverted

	////5: 1->inv(2)->3->4
	////0...(i-1)->(j-1)...i->j...(k-1)->k...0	
	//chainIndex[5][0] = i;
	//chainIndex[5][1] = j-1;
	//chainInverted[5][0] = true; //inverted
	//chainIndex[5][2] = j;
	//chainIndex[5][3] = k-1;
	//chainInverted[5][1] = false; //not inverted

	cost[0] = getCost();
	for (int c = 1; c < nVariations; c++)
	{
		//start of the route
		cost[c] = getCostUntil(i-1);
		aux_weight = getWeightAt(i-1);

		//(i-1)->(chain 1 start)		
		if (!chainInverted[c][0])
		{
			cost[c] += costOfArc(inst, i-1, chainIndex[c][0], aux_weight);	
		}	
		else
		{
			cost[c] += costOfArc(inst, i-1, chainIndex[c][1], aux_weight);	
		}

		//(chain 1 start) to (chain 1 finish)
		cost[c] += costOfChain(chainIndex[c][0], chainIndex[c][1], aux_weight, chainInverted[c][0]);

		//(chain 1 finish) to (chain 2 start)
		if (!chainInverted[c][0])
		{
			if (!chainInverted[c][1])
			{
				cost[c] += costOfArc(inst, chainIndex[c][1], chainIndex[c][2], aux_weight);	
			}	
			else
			{
				cost[c] += costOfArc(inst, chainIndex[c][1], chainIndex[c][3], aux_weight);	
			}
		}	
		else
		{
			if (!chainInverted[c][1])
			{
				cost[c] += costOfArc(inst, chainIndex[c][0], chainIndex[c][2], aux_weight);	
			}	
			else
			{
				cost[c] += costOfArc(inst, chainIndex[c][0], chainIndex[c][3], aux_weight);	
			}
		}

		//(chain 2 start) to (chain 2 finish)
		cost[c] += costOfChain(chainIndex[c][2], chainIndex[c][3], aux_weight, chainInverted[c][1]);

		//(chain 2 finish)->rest of the route	
		if (!chainInverted[c][1])
		{
			cost[c] += costOfArc(inst, chainIndex[c][3], k, aux_weight);	
		}	
		else
		{
			cost[c] += costOfArc(inst, chainIndex[c][2], k, aux_weight);	
		}

		//rest of the route
		cost[c] += getCostAhead(k);
	}
	
	double bestCost = cost[0];
	identifier = 0;

	for (int c = 1; c < nVariations; c++)
	{
		if (cost[c] < bestCost)
		{
			bestCost = cost[c];
			identifier = c;
		}
	}

	return bestCost - getCost();
}

/// <summary>
/// Move3s the opt.
/// </summary>
/// <param name="inst">The inst.</param>
/// <param name="i">The i.</param>
/// <param name="j">The j.</param>
/// <param name="k">The k.</param>
/// <param name="identifier">The identifier.</param>
void Trip::move3Opt(Instance & inst, int i, int j, int k, int identifier)
{
	//contains ordered gifts of chains 2 and 3 
	vector<Gift> bagOfGifts;
	vector<Gift>::iterator it;

	int nVariations = 2*2*2; //8
	vector<double> cost(nVariations);

	vector<vector<int>> chainIndex(nVariations);
	for (int i = 0; i < nVariations; i++)
	{
		chainIndex[i].resize(4);
	}
	vector<vector<bool>> chainInverted(nVariations);
	for (int i = 0; i < nVariations; i++)
	{
		chainInverted[i].resize(2);
	}

	int v = 0;
	for (int v1 = 0; v1 < 2*2; v1++)
	{
		chainIndex[v][0] = i;
		chainIndex[v][1] = j-1;
		chainIndex[v][2] = j;
		chainIndex[v][3] = k-1;
		v++;
	}
	for (int v1 = 0; v1 < 2*2; v1++)
	{
		chainIndex[v][0] = j;
		chainIndex[v][1] = k-1;
		chainIndex[v][2] = i;
		chainIndex[v][3] = j-1;
		v++;
	}

	for (int v = 1; v < nVariations; v++)
	{
		if (v % 2 == 1)
		{
			chainInverted[v][0] = true; //not inverted
		}
		if ((1+(v/2)) % 2 == 0) //2,3,6,7 encyclopedia of integer sequences
		{
			chainInverted[v][1] = true; //not inverted
		}
	}

	//get first chain
	if (!chainInverted[identifier][0])
	{
		for (int g = chainIndex[identifier][0]; g <= chainIndex[identifier][1]; g++)
		{
			bagOfGifts.push_back(getGift(g));
		}
	}
	else
	{
		for (int g = chainIndex[identifier][1]; g >= chainIndex[identifier][0]; g--)
		{
			bagOfGifts.push_back(getGift(g));
		}
	}

	//get second chain
	if (!chainInverted[identifier][1])
	{
		for (int g = chainIndex[identifier][2]; g <= chainIndex[identifier][3]; g++)
		{
			bagOfGifts.push_back(getGift(g));
		}
	}
	else
	{
		for (int g = chainIndex[identifier][3]; g >= chainIndex[identifier][2]; g--)
		{
			bagOfGifts.push_back(getGift(g));
		}
	}
	
	it = bagOfGifts.begin();
	for (int g = i; g <= k-1; g++)
	{
		setGift(inst, g, *it, false);
		++it;
	}
	
	UpdateAuxiliaryDataStructures(inst);
}

void Trip::move4Opt(Instance & inst, int i, int j, int k, int l, int identifier)
{
	//contains ordered gifts of chains 2 and 3 
	vector<Gift> bagOfGifts;
	vector<Gift>::iterator it;

	int nVariations = 3*2*2*2*2; //48
	vector<vector<int>> chainOrder();

	vector<double> cost(nVariations);

	vector<vector<int>> chainIndex(nVariations);
	for (int i = 0; i < nVariations; i++)
	{
		chainIndex[i].resize(6);
	}
	vector<vector<bool>> chainInverted(nVariations);
	for (int i = 0; i < nVariations; i++)
	{
		chainInverted[i].resize(3);
	}

	int v = 0;
	//234
	for (int v1 = 0; v1 < 2*2*2; v1++)
	{
		chainIndex[v][0] = i;
		chainIndex[v][1] = j-1;
		chainIndex[v][2] = j;
		chainIndex[v][3] = k-1;
		chainIndex[v][4] = k;
		chainIndex[v][5] = l-1;
		v++;
	}
	//243
	for (int v1 = 0; v1 < 2*2*2; v1++)
	{
		chainIndex[v][0] = i;
		chainIndex[v][1] = j-1;
		chainIndex[v][2] = k;
		chainIndex[v][3] = l-1;
		chainIndex[v][4] = j;
		chainIndex[v][5] = k-1;
		v++;
	}
	//324
	for (int v1 = 0; v1 < 2*2*2; v1++)
	{		
		chainIndex[v][0] = j;
		chainIndex[v][1] = k-1;
		chainIndex[v][2] = i;
		chainIndex[v][3] = j-1;
		chainIndex[v][4] = k;
		chainIndex[v][5] = l-1;
		v++;
	}
	//342
	for (int v1 = 0; v1 < 2*2*2; v1++)
	{		
		chainIndex[v][0] = j;
		chainIndex[v][1] = k-1;
		chainIndex[v][2] = k;
		chainIndex[v][3] = l-1;
		chainIndex[v][4] = i;
		chainIndex[v][5] = j-1;
		v++;
	}
	//423
	for (int v1 = 0; v1 < 2*2*2; v1++)
	{		
		chainIndex[v][0] = k;
		chainIndex[v][1] = l-1;
		chainIndex[v][2] = i;
		chainIndex[v][3] = j-1;
		chainIndex[v][4] = j;
		chainIndex[v][5] = k-1;
		v++;
	}
	//432
	for (int v1 = 0; v1 < 2*2*2; v1++)
	{		
		chainIndex[v][0] = k;
		chainIndex[v][1] = l-1;
		chainIndex[v][2] = j;
		chainIndex[v][3] = k-1;
		chainIndex[v][4] = i;
		chainIndex[v][5] = j-1;
		v++;
	}

	for (int v = 1; v < nVariations; v++)
	{
		if ( (v%8 == 1) || (v%8 == 4) || (v%8 == 5) || (v%8 == 7) )
		{
			chainInverted[v][0] = true; // inverted
		}
		if ( (v%8 == 2) || (v%8 == 4) || (v%8 == 6) || (v%8 == 7) )
		{
			chainInverted[v][1] = true; // inverted
		}
		if ( (v%8 == 3) || (v%8 == 5) || (v%8 == 6) || (v%8 == 7) )
		{
			chainInverted[v][2] = true; // inverted
		}
	}
	//end - populate search structures
	//end

	//get first chain
	if (!chainInverted[identifier][0])
	{
		for (int g = chainIndex[identifier][0]; g <= chainIndex[identifier][1]; g++)
		{
			bagOfGifts.push_back(getGift(g));
		}
	}
	else
	{
		for (int g = chainIndex[identifier][1]; g >= chainIndex[identifier][0]; g--)
		{
			bagOfGifts.push_back(getGift(g));
		}
	}


	//get second chain
	if (!chainInverted[identifier][1])
	{
		for (int g = chainIndex[identifier][2]; g <= chainIndex[identifier][3]; g++)
		{
			bagOfGifts.push_back(getGift(g));
		}
	}
	else
	{
		for (int g = chainIndex[identifier][3]; g >= chainIndex[identifier][2]; g--)
		{
			bagOfGifts.push_back(getGift(g));
		}
	}
	
	//get third chain
	if (!chainInverted[identifier][2])
	{
		for (int g = chainIndex[identifier][4]; g <= chainIndex[identifier][5]; g++)
		{
			bagOfGifts.push_back(getGift(g));
		}
	}
	else
	{
		for (int g = chainIndex[identifier][5]; g >= chainIndex[identifier][4]; g--)
		{
			bagOfGifts.push_back(getGift(g));
		}
	}

	it = bagOfGifts.begin();
	for (int g = i; g <= l-1; g++)
	{
		setGift(inst, g, *it, false);
		++it;
	}
	
	UpdateAuxiliaryDataStructures(inst);
}


void Trip::move5Opt(Instance & inst, int i, int j, int k, int l, int m, int identifier)
{
	//contains ordered gifts of chains 2 and 3 
	vector<Gift> bagOfGifts;
	vector<Gift>::iterator it;

	switch (identifier)
	{
	case 0:
		//1->3->2->4
		//0...(i-1)->j...(k-1)->i...(j-1)->k...0

		//3
		for (int g = j; g <= k-1; g++)
		{
			bagOfGifts.push_back(getGift(g));
		}	
		//2
		for (int g = i; g <= j-1; g++)
		{
			bagOfGifts.push_back(getGift(g));
		}

		break;
	case 1:
		//1->inv(2)->inv(3)->4
		//0...(i-1)->(j-1)...i->(k-1)...j->k...0

		//inv(2)
		for (int g = j-1; g >= i; g--)
		{
			bagOfGifts.push_back(getGift(g));
		}
		//inv(3)
		for (int g = k-1; g >= j; g--)
		{
			bagOfGifts.push_back(getGift(g));
		}

		break;
	case 2:
		//1->inv(3)->2->4
		//0...(i-1)->(k-1)...j->i...(j-1)->k...0

		//inv(3)
		for (int g = k-1; g >= j; g--)
		{
			bagOfGifts.push_back(getGift(g));
		}
		//2
		for (int g = i; g <= j-1; g++)
		{
			bagOfGifts.push_back(getGift(g));
		}

		break;
	case 3:
		//1->3->inv(2)->4
		//0...(i-1)->j...(k-1)->(j-1)...i->k...0
		
		//3
		for (int g = j; g <= k-1; g++)
		{
			bagOfGifts.push_back(getGift(g));
		}	
		//inv(2)
		for (int g = j-1; g >= i; g--)
		{
			bagOfGifts.push_back(getGift(g));
		}

		break;
	}

	it = bagOfGifts.begin();
	for (int g = i; g <= k-1; g++)
	{
		setGift(inst, g, *it, false);
		++it;
	}
	
	UpdateAuxiliaryDataStructures(inst);
}

double Trip::costOfArc(Instance & inst, int fromGiftIndex, int toGiftIndex, double& weightAtStart)
{
	//cost[5] += aux_weight * inst.getDistance(getGiftId(i-1), getGiftId(j-1));	
	//aux_weight -= getWeight(j-1);	

	double cost = weightAtStart * inst.getDistance(getGiftId(fromGiftIndex), getGiftId(toGiftIndex));
	weightAtStart -= getWeight(toGiftIndex);	

	return cost;
}

double Trip::costOfChain(int startGiftIndex, int endGiftIndex, double& weightAtStart, bool inverted)
{
	double cost = 0;
	if (!inverted)
	{
		for (int g = startGiftIndex; g < endGiftIndex; g++)
		{
			cost += weightAtStart * getDistanceFrom(g);
			weightAtStart -= getWeight(g+1);
		}
	}
	else
	{
		for (int g = endGiftIndex; g > startGiftIndex; g--)
		{
			cost += weightAtStart * getDistanceFrom(g-1);
			weightAtStart -= getWeight(g-1);
		}
	}	
	return cost;
}


//on 3opt, 2 chains are changed
double Trip::evaluate4Opt(Instance & inst, int i, int j, int k, int l, int & identifier)
{
	double aux_weight = 0;
	//1 0...(i-1)
	//2 i...(j-1)
	//3 j...(k-1)
	//4 k...0
	
	//populate search structures
	int nVariations = 3*2*2*2*2; //48
	vector<vector<int>> chainOrder();

	vector<double> cost(nVariations);

	vector<vector<int>> chainIndex(nVariations);
	for (int i = 0; i < nVariations; i++)
	{
		chainIndex[i].resize(6);
	}
	vector<vector<bool>> chainInverted(nVariations);
	for (int i = 0; i < nVariations; i++)
	{
		chainInverted[i].resize(3);
	}

	int v = 0;
	//234
	for (int v1 = 0; v1 < 2*2*2; v1++)
	{
		chainIndex[v][0] = i;
		chainIndex[v][1] = j-1;
		chainIndex[v][2] = j;
		chainIndex[v][3] = k-1;
		chainIndex[v][4] = k;
		chainIndex[v][5] = l-1;
		v++;
	}
	//243
	for (int v1 = 0; v1 < 2*2*2; v1++)
	{
		chainIndex[v][0] = i;
		chainIndex[v][1] = j-1;
		chainIndex[v][2] = k;
		chainIndex[v][3] = l-1;
		chainIndex[v][4] = j;
		chainIndex[v][5] = k-1;
		v++;
	}
	//324
	for (int v1 = 0; v1 < 2*2*2; v1++)
	{		
		chainIndex[v][0] = j;
		chainIndex[v][1] = k-1;
		chainIndex[v][2] = i;
		chainIndex[v][3] = j-1;
		chainIndex[v][4] = k;
		chainIndex[v][5] = l-1;
		v++;
	}
	//342
	for (int v1 = 0; v1 < 2*2*2; v1++)
	{		
		chainIndex[v][0] = j;
		chainIndex[v][1] = k-1;
		chainIndex[v][2] = k;
		chainIndex[v][3] = l-1;
		chainIndex[v][4] = i;
		chainIndex[v][5] = j-1;
		v++;
	}
	//423
	for (int v1 = 0; v1 < 2*2*2; v1++)
	{		
		chainIndex[v][0] = k;
		chainIndex[v][1] = l-1;
		chainIndex[v][2] = i;
		chainIndex[v][3] = j-1;
		chainIndex[v][4] = j;
		chainIndex[v][5] = k-1;
		v++;
	}
	//432
	for (int v1 = 0; v1 < 2*2*2; v1++)
	{		
		chainIndex[v][0] = k;
		chainIndex[v][1] = l-1;
		chainIndex[v][2] = j;
		chainIndex[v][3] = k-1;
		chainIndex[v][4] = i;
		chainIndex[v][5] = j-1;
		v++;
	}

	for (int v = 1; v < nVariations; v++)
	{
		if ( (v%8 == 1) || (v%8 == 4) || (v%8 == 5) || (v%8 == 7) )
		{
			chainInverted[v][0] = true; // inverted
		}
		if ( (v%8 == 2) || (v%8 == 4) || (v%8 == 6) || (v%8 == 7) )
		{
			chainInverted[v][1] = true; // inverted
		}
		if ( (v%8 == 3) || (v%8 == 5) || (v%8 == 6) || (v%8 == 7) )
		{
			chainInverted[v][2] = true; // inverted
		}
	}
	//end - populate search structures

	cost[0] = getCost();
	for (int c = 1; c < nVariations; c++)
	{
		//start of the route
		cost[c] = getCostUntil(i-1);
		aux_weight = getWeightAt(i-1);

		//(i-1)->(chain 1 start)		
		if (!chainInverted[c][0])
		{
			cost[c] += costOfArc(inst, i-1, chainIndex[c][0], aux_weight);	
		}	
		else
		{
			cost[c] += costOfArc(inst, i-1, chainIndex[c][1], aux_weight);	
		}

		//(chain 1 start) to (chain 1 finish)
		cost[c] += costOfChain(chainIndex[c][0], chainIndex[c][1], aux_weight, chainInverted[c][0]);

		//(chain 1 finish) to (chain 2 start)
		if (!chainInverted[c][0])
		{
			if (!chainInverted[c][1])
			{
				cost[c] += costOfArc(inst, chainIndex[c][1], chainIndex[c][2], aux_weight);	
			}	
			else
			{
				cost[c] += costOfArc(inst, chainIndex[c][1], chainIndex[c][3], aux_weight);	
			}
		}	
		else
		{
			if (!chainInverted[c][1])
			{
				cost[c] += costOfArc(inst, chainIndex[c][0], chainIndex[c][2], aux_weight);	
			}	
			else
			{
				cost[c] += costOfArc(inst, chainIndex[c][0], chainIndex[c][3], aux_weight);	
			}
		}

		//(chain 2 start) to (chain 2 finish)
		cost[c] += costOfChain(chainIndex[c][2], chainIndex[c][3], aux_weight, chainInverted[c][1]);

		//(chain 2 finish) to (chain 3 start)
		if (!chainInverted[c][1])
		{
			if (!chainInverted[c][2])
			{
				cost[c] += costOfArc(inst, chainIndex[c][3], chainIndex[c][4], aux_weight);	
			}	
			else
			{
				cost[c] += costOfArc(inst, chainIndex[c][3], chainIndex[c][5], aux_weight);	
			}
		}	
		else
		{
			if (!chainInverted[c][2])
			{
				cost[c] += costOfArc(inst, chainIndex[c][2], chainIndex[c][4], aux_weight);	
			}	
			else
			{
				cost[c] += costOfArc(inst, chainIndex[c][2], chainIndex[c][5], aux_weight);	
			}
		}

		//(chain 3 start) to (chain 3 finish)
		cost[c] += costOfChain(chainIndex[c][4], chainIndex[c][5], aux_weight, chainInverted[c][2]);

		//(chain 3 finish) to rest of the route 
		if (!chainInverted[c][2])
		{
			cost[c] += costOfArc(inst, chainIndex[c][5], l, aux_weight);	
		}	
		else
		{
			cost[c] += costOfArc(inst, chainIndex[c][4], l, aux_weight);	
		}

		//rest of the route
		cost[c] += getCostAhead(l);
	}
	
	double bestCost = cost[0];
	identifier = 0;

	for (int c = 1; c < nVariations; c++)
	{
		if (cost[c] < bestCost)
		{
			bestCost = cost[c];
			identifier = c;
		}
	}

	return bestCost - getCost();
}
	

//retorn a diferenca resultado de se colocar o novo presente no indice index
//se negativa , melhor
double Trip::evaluateSetGift(Instance & inst, int index, Gift & gift)
{
	return getDistanceUntil(index-1)*(gift.weight - getWeight(index)) //diferenca que o novo peso faz na rota do polo norte até o presente anterior
		- getCostFrom(index-1) + inst.getDistance(gift.id, getGiftId(index-1))*(getWeightAt(index-1)+gift.weight-getWeight(index)) //diferenca no arco que entra
	    - getCostFrom(index)   + inst.getDistance(gift.id, getGiftId(index+1))*(getWeightAt(index));			//diferenca no arco que sai
}

//retorn a diferenca resultado de se inserir um novo presente no indice index
//se negativa , melhor
double Trip::evaluateInsertion(Instance & inst, int index, Gift & gift)
{
	return getDistanceUntil(index-1)*gift.weight //diferenca que o novo peso faz na rota do polo norte até o presente anterior
		- getCostFrom(index-1) 
		+ inst.getDistance(gift.id, getGiftId(index-1))*(getWeightAt(index-1)+gift.weight) //diferenca no arco que entra
	    + inst.getDistance(gift.id, getGiftId(index))*(getWeightAt(index-1));			//diferenca no arco que sai
}

double Trip::evaluateInsertionLength(Instance & inst, int index, Gift & gift)
{
	return - getDistanceFrom(index-1) 
		+ inst.getDistance(getGiftId(index-1), gift.id) //diferenca no arco que entra
	    + inst.getDistance(gift.id, getGiftId(index));			//diferenca no arco que sai
}

void Trip::setGift(Instance & inst, int index, Gift & gift, bool update)
{
	Gifts[index] = gift;
	
	if (update)
	{
		UpdateAuxiliaryDataStructures(inst);
	}
}

void Trip::appendGift(Instance & inst, Gift & gift)
{
	Gifts.push_back(gift);
	
	UpdateAuxiliaryDataStructures(inst);
}

void Trip::appendGift(Instance & inst, int giftId)
{
	appendGift(inst, inst.UnsortedGifts[giftId-1]);
}

void Trip::popBackGift(Instance & inst)
{
	Gifts.pop_back();
	
	UpdateAuxiliaryDataStructures(inst);
}

void Trip::insertGift(Instance & inst, int indexBefore, Gift & gift, bool update)
{
	Gifts.insert(Gifts.begin()+indexBefore, gift);
	
	if (update)
	{
		UpdateAuxiliaryDataStructures(inst);
	}
}

void Trip::swapGifts(Instance & inst, int index1, int index2)
{
	//index1 should always be the first
	if (index1 > index2)
	{
		int aux = index1;
		index1 = index2;
		index2 = aux;
	}

	//switch gifts at trip's gift vector
	Gift auxGift = Gifts[index1];

	Gifts[index1] = Gifts[index2];
	Gifts[index2] = auxGift;
		
	UpdateAuxiliaryDataStructures(inst);
}

std::string Trip::printTrip() 
{
	stringstream aux;
	aux << "NP->";

	for (int i = 0; i < (int)Gifts.size(); i++)
	{
		aux << Gifts[i].id << "->";
	}
	aux << "NP";

	return aux.str();
}

bool Trip::LastCompatible(Gift & newGift, Instance & inst, int criteria, double threeshold)
{
	if (weight + newGift.weight > inst.weightLimit)
	{
		return false;
	}

	//verifica se é compativel com o ultimo gift
	if (Gifts.size() > 0)
	{
		if (!inst.Compatible(newGift,Gifts.back(), criteria, threeshold))
		{
			return false;
		}
	}
	
	return true;
}

bool Trip::FullyCompatible(Gift & newGift, Instance & inst, int criteria, double threeshold)
{
	if (weight + newGift.weight > inst.weightLimit)
	{
		return false;
	}

	//verifica se é compativel com todos os gifts
	for (int i = 0; i < (int)Gifts.size(); i++)
	{
		if (!inst.Compatible(newGift,Gifts[i], criteria, threeshold))
		{
			return false;
		}
	}
	
	return true;
}

bool Trip::SomeCompatible(Gift & newGift, Instance & inst, int criteria, double threeshold)
{
	if (weight + newGift.weight > inst.weightLimit)
	{
		return false;
	}

	if (Gifts.size() == 0)
	{
		return true;
	}

	//verifica se é compativel com algum os gifts
	for (int i = 0; i < (int)Gifts.size(); i++)
	{
		if (inst.Compatible(newGift,Gifts[i], criteria, threeshold))
		{
			return true;
		}
	}
	
	return false;
}

string Trip::routeToGeoJson(int id)
{
	//sample
	//{"type":"FeatureCollection","features":[
	//{"type":"Feature","properties":{"name":"Route0"},"geometry":{"type":"LineString","coordinates":[[0,90],[0,80],[10,80],[20,70],[10,70],[0,90]]},"id":"route0"}
	//]}

	stringstream aux;

	aux << "{\"type\":\"FeatureCollection\",\"features\":[" << endl;
	aux << "{\"type\":\"Feature\",\"properties\":{\"name\":\"Route" << id << "\",\"weight\":" << weight << "},\"geometry\":{\"type\":\"LineString\",\"coordinates\":[[0,90],";
	for (int i = 0; i < (int)Gifts.size(); i++)
	{
		aux << "[" << Gifts[i].longitude << "," << Gifts[i].latitude << "],";
	}	
	aux << "[0,90]]},\"id\":\"route" << id << "\"}" << endl;
	aux << "]}";

	return aux.str();
}

string Trip::pointsToGeoJson()
{
	//sample
	//{"type":"FeatureCollection","features":[
	//{"type":"Feature","properties":{"name":"NorthPole"},"geometry":{"type":"Point","coordinates":[0,90]},"id":"p0"}
	//]}

	stringstream aux;

	aux << "{\"type\":\"FeatureCollection\",\"features\":[" << endl;
	for (int i = 0; i < (int)Gifts.size(); i++)
	{
		aux << "{\"type\":\"Feature\",\"properties\":{\"name\":\"Gift" << Gifts[i].id << "\",\"weight\":" << Gifts[i].weight << "},\"geometry\":{\"type\":\"Point\",\"coordinates\":[" << Gifts[i].longitude << "," << Gifts[i].latitude << "]},\"id\":\"g" << Gifts[i].id << "\"},";
	}	
	aux << "{\"type\":\"Feature\",\"properties\":{\"name\":\"NorthPole\"},\"geometry\":{\"type\":\"Point\",\"coordinates\":[0,90]},\"id\":\"np\"}" << endl;
	aux << "]}";

	return aux.str();
}

//double Trip::Weight(double sleighWeight)
//{
//	double tripWeight = sleighWeight; //ranges from 1 to m, first trip index = 1
//
//	for (int i = 0; i < (int)Gifts.size(); i++)
//	{
//		tripWeight += Gifts[i].weight;
//	}
//	return tripWeight;
//}

//sorting procedures
bool distanciaNP(const Gift& gift1, const Gift& gift2) {
   return gift1.npDistance < gift2.npDistance;
}

void Trip::SortDistanciaNP()
{
	sort(Gifts.begin(),Gifts.end(),distanciaNP);
}

bool reverseDistanciaNP(const Gift& gift1, const Gift& gift2) {
   return gift1.npDistance > gift2.npDistance;
}

void Trip::cloneTrip(Instance & inst, Trip & newTrip)
{
	size = newTrip.size;
	weight = newTrip.weight;
	cost = newTrip.cost;
	Gifts = newTrip.Gifts;
	weightAt = newTrip.weightAt;
	distanceFrom = newTrip.distanceFrom;
	costFrom = newTrip.costFrom;
	distanceUntil = newTrip.distanceUntil;
	distanceAhead = newTrip.distanceAhead;
}

//traditional CrossExchange sense
//both chains exchanged in original orientation
#define CX_SENSE_ORIGINAL_ORIGINAL 0

//First chain exchanged in original orientation
//Second chain exchanged in reverse orientation
#define CX_SENSE_ORIGINAL_REVERSE 1 

//First chain exchanged in reverse orientation
//Second chain exchanged in original orientation
#define CX_SENSE_REVERSE_ORIGINAL 2

//First chain exchanged in reverse orientation
//Second chain exchanged in reverse orientation
#define CX_SENSE_REVERSE_REVERSE 3


//movements and evalution procedures

#ifdef REVERSE_CX
double evaluateCrossExchange(Instance & inst, Trip & trip1, Trip & trip2, int i, int k, int j, int l, int& sense)
{	
	double weightChain1 = trip1.getWeightAt(i) - trip1.getWeightAt(k);
	double weightChain2 = trip2.getWeightAt(j) - trip2.getWeightAt(l);

	//check if weight exchange is feasible
	if ( (trip1.getWeight() - weightChain1 + weightChain2 > inst.weightLimit) ||
		(trip2.getWeight() - weightChain2 + weightChain1 > inst.weightLimit) )
	{
		return 1e15;
	}

	//cost of getting to gift i + cost of carrying the weight difference until gift i
	double cost1 = trip1.getCostUntil(i) + (weightChain2 - weightChain1) * trip1.getDistanceUntil(i);
	double reverseCost1 = cost1;

	//weight before crossing
	double weight1 = trip1.getWeightAt(i) - weightChain1 + weightChain2;
	double reverseWeight1 = weight1;
	if (j < l)
	{
		cost1 += inst.getDistance(trip1.getGiftId(i), trip2.getGiftId(j+1))*weight1;
		for (int g = j+1; g < l; g++)
		{
			weight1 -= trip2.getWeight(g);
			cost1 += trip2.getDistanceFrom(g)*weight1;
		}		
		weight1 -= trip2.getWeight(l);
		cost1 += inst.getDistance(trip2.getGiftId(l), trip1.getGiftId(k+1))*weight1;

		if (l != j+1)
		{
			reverseCost1 += inst.getDistance(trip1.getGiftId(i), trip2.getGiftId(l))*reverseWeight1;
			for (int g = l; g > j+1; g--)
			{
				reverseWeight1 -= trip2.getWeight(g);
				reverseCost1 += trip2.getDistanceFrom(g-1)*reverseWeight1;
			}		
			reverseWeight1 -= trip2.getWeight(j+1);
			reverseCost1 += inst.getDistance(trip2.getGiftId(j+1), trip1.getGiftId(k+1))*reverseWeight1;
		}
		else
		{
			reverseCost1 = cost1;
		}
	}
	else
	{
		cost1 += inst.getDistance(trip1.getGiftId(i), trip1.getGiftId(k+1))*weight1;
		reverseCost1 = cost1;
	}
	cost1 += trip1.getCostAhead(k+1);
	reverseCost1 += trip1.getCostAhead(k+1);
		
	//cost of getting to gift j + cost of carrying the weight difference until gift j
	double cost2 = trip2.getCostUntil(j) + (weightChain1 - weightChain2) * trip2.getDistanceUntil(j);
	double reverseCost2 = cost2;

	//weight before crossing
	double weight2 = trip2.getWeightAt(j) - weightChain2 + weightChain1;
	double reverseWeight2 = weight2;
	if (i < k)
	{
		cost2 += inst.getDistance(trip2.getGiftId(j), trip1.getGiftId(i+1))*weight2;
		for (int g = i+1; g < k; g++)
		{
			weight2 -= trip1.getWeight(g);
			cost2 += trip1.getDistanceFrom(g)*weight2;
		}
		weight2 -= trip1.getWeight(k);
		cost2 += inst.getDistance(trip1.getGiftId(k), trip2.getGiftId(l+1))*weight2;

		if (i+1 != k)
		{
			reverseCost2 += inst.getDistance(trip2.getGiftId(j), trip1.getGiftId(k))*reverseWeight2;
			for (int g = k; g > i+1; g--)
			{
				reverseWeight2 -= trip1.getWeight(g);
				reverseCost2 += trip1.getDistanceFrom(g-1)*reverseWeight2;
			}		
			reverseWeight2 -= trip1.getWeight(i+1);
			reverseCost2 += inst.getDistance(trip1.getGiftId(i+1), trip2.getGiftId(l+1))*reverseWeight2;
		}
		else
		{
			reverseCost2 = cost2;
		}
	}
	else
	{
		cost2 += inst.getDistance(trip2.getGiftId(j), trip2.getGiftId(l+1))*weight2;
		reverseCost2 = cost2;
	}
	cost2 += trip2.getCostAhead(l+1);	
	reverseCost2 += trip2.getCostAhead(l+1);

	double bestIncrease;

	//if trip1 receives reversed chain2
	if (reverseCost1 < cost1 - RELEVANT_INCREASE)
	{
		//if trip2 receives reversed chain1
		if (reverseCost2 < cost2 - RELEVANT_INCREASE)
		{
			sense = CX_SENSE_REVERSE_REVERSE;
			bestIncrease = ( reverseCost1 + reverseCost2 ) - ( trip1.getCost() + trip2.getCost() );
		}
		else
		{
			sense = CX_SENSE_ORIGINAL_REVERSE;
			bestIncrease = ( reverseCost1 + cost2 ) - ( trip1.getCost() + trip2.getCost() );
		}
	}
	else
	{
		//if trip2 receives reversed chain1
		if (reverseCost2 < cost2 - RELEVANT_INCREASE)
		{
			sense = CX_SENSE_REVERSE_ORIGINAL;
			bestIncrease = ( cost1 + reverseCost2 ) - ( trip1.getCost() + trip2.getCost() );
		}
		else
		{
			sense = CX_SENSE_ORIGINAL_ORIGINAL;
			bestIncrease = ( cost1 + cost2 ) - ( trip1.getCost() + trip2.getCost() );
		}
	}

	return bestIncrease;
}
#else
double evaluateCrossExchange(Instance & inst, Trip & trip1, Trip & trip2, int i, int k, int j, int l, int& sense)
{	
	double weightChain1 = trip1.getWeightAt(i) - trip1.getWeightAt(k);
	double weightChain2 = trip2.getWeightAt(j) - trip2.getWeightAt(l);

	//check if weight exchange is feasible
	if ( (trip1.getWeight() - weightChain1 + weightChain2 > inst.weightLimit) ||
		(trip2.getWeight() - weightChain2 + weightChain1 > inst.weightLimit) )
	{
		return 1e15;
	}

	//cost of getting to gift i + cost of carrying the weight difference until gift i
	double cost1 = trip1.getCostUntil(i) + (weightChain2 - weightChain1) * trip1.getDistanceUntil(i);

	//weight before crossing
	double weight1 = trip1.getWeightAt(i) - weightChain1 + weightChain2;
	if (j < l)
	{
		cost1 += inst.getDistance(trip1.getGiftId(i), trip2.getGiftId(j+1))*weight1;
		for (int g = j+1; g < l; g++)
		{
			weight1 -= trip2.getWeight(g);
			cost1 += trip2.getDistanceFrom(g)*weight1;
		}		
		weight1 -= trip2.getWeight(l);
		cost1 += inst.getDistance(trip2.getGiftId(l), trip1.getGiftId(k+1))*weight1;

	}
	else
	{
		cost1 += inst.getDistance(trip1.getGiftId(i), trip1.getGiftId(k+1))*weight1;
	}
	cost1 += trip1.getCostAhead(k+1);
		
	//cost of getting to gift j + cost of carrying the weight difference until gift j
	double cost2 = trip2.getCostUntil(j) + (weightChain1 - weightChain2) * trip2.getDistanceUntil(j);

	//weight before crossing
	double weight2 = trip2.getWeightAt(j) - weightChain2 + weightChain1;
	if (i < k)
	{
		cost2 += inst.getDistance(trip2.getGiftId(j), trip1.getGiftId(i+1))*weight2;
		for (int g = i+1; g < k; g++)
		{
			weight2 -= trip1.getWeight(g);
			cost2 += trip1.getDistanceFrom(g)*weight2;
		}
		weight2 -= trip1.getWeight(k);
		cost2 += inst.getDistance(trip1.getGiftId(k), trip2.getGiftId(l+1))*weight2;
	}
	else
	{
		cost2 += inst.getDistance(trip2.getGiftId(j), trip2.getGiftId(l+1))*weight2;
	}
	cost2 += trip2.getCostAhead(l+1);	
	
	sense = CX_SENSE_ORIGINAL_ORIGINAL;
	return ( cost1 + cost2 ) - ( trip1.getCost() + trip2.getCost() );
}
#endif

double evaluateCrossExchangeArtur(Instance & inst, Trip & trip1, Trip & trip2, int i, int k, int j, int l, bool& over1, bool& over2)
{
	double weightChain1 = trip1.getWeightAt(i) - trip1.getWeightAt(k);
	double weightChain2 = trip2.getWeightAt(j) - trip2.getWeightAt(l);

	if (trip1.getWeight() - weightChain1 + weightChain2 > inst.weightLimit)
	{
	over1 = true;
	over2 = false;
	return 1e15;
	}
	if (trip2.getWeight() - weightChain2 + weightChain1 > inst.weightLimit)
	{
	over2 = true;
	over1 = false;
	return 1e15;
	}

	double cost1 = trip1.getCostUntil(i) + (weightChain2 - weightChain1) * trip1.getDistanceUntil(i);
	double weight1 = trip1.getWeightAt(i) - weightChain1 + weightChain2;
	if (j < l)
	{
		cost1 += inst.getDistance(trip1.getGiftId(i), trip2.getGiftId(j+1))*weight1;
		for (int g = j+1; g < l; g++)
		{
			weight1 -= trip2.getWeight(g);
			cost1 += trip2.getDistanceFrom(g)*weight1;
		}
		weight1 -= trip2.getWeight(l);
		cost1 += inst.getDistance(trip2.getGiftId(l), trip1.getGiftId(k+1))*weight1;
	}
	else
	{
		cost1 += inst.getDistance(trip1.getGiftId(i), trip1.getGiftId(k+1))*weight1;
	}
	cost1 += trip1.getCostAhead(k+1);
		
	double cost2 = trip2.getCostUntil(j) + (weightChain1 - weightChain2) * trip2.getDistanceUntil(j);
	double weight2 = trip2.getWeightAt(j) - weightChain2 + weightChain1;
	if (i < k)
	{
		cost2 += inst.getDistance(trip2.getGiftId(j), trip1.getGiftId(i+1))*weight2;
		for (int g = i+1; g < k; g++)
		{
			weight2 -= trip1.getWeight(g);
			cost2 += trip1.getDistanceFrom(g)*weight2;
		}
		weight2 -= trip1.getWeight(k);
		cost2 += inst.getDistance(trip1.getGiftId(k), trip2.getGiftId(l+1))*weight2;
	}
	else
	{
		cost2 += inst.getDistance(trip2.getGiftId(j), trip2.getGiftId(l+1))*weight2;
	}
	cost2 += trip2.getCostAhead(l+1);	

	//cout << cost1 << "\t" << cost2 << endl;
	over1 = false;
	over2 = false;
	return ( cost1 + cost2 ) - ( trip1.getCost() + trip2.getCost() );
}

void moveCrossExchange(Instance & inst, Trip & trip1, Trip & trip2, int i, int k, int j, int l, int sense)
{
	vector<Gift> bag1;
	vector<Gift> bag2;
	
	//debugging
	/*cout << "trip1 size: " << trip1.getSize() << endl;
	cout << "trip2 size: " << trip2.getSize() << endl;		
	cout << "trip1: " << trip1.printTrip() << endl;
	cout << "trip2: " << trip2.printTrip() << endl;	
	cout << "i: " << i << endl;
	cout << "j: " << j << endl;
	cout << "k: " << k << endl;
	cout << "l: " << l << endl;*/

	for (int g = i+1; g <= k; g++)
	{
		bag1.push_back(trip1.getGift(i+1));
		trip1.eraseGift(inst,i+1, false);
	}
	for (int g = j+1; g <= l; g++)
	{
		bag2.push_back(trip2.getGift(j+1));
		trip2.eraseGift(inst,j+1, false);
	}

	if ( (sense == CX_SENSE_ORIGINAL_ORIGINAL) || (sense == CX_SENSE_REVERSE_ORIGINAL) )
	{
		for (int g = bag2.size()-1; g > -1; g--)
		{
			trip1.insertGift(inst,i+1,bag2[g], false);
		}
	}
	else
	{
		for (int g = 0; g < (int)bag2.size(); g++)
		{
			trip1.insertGift(inst,i+1,bag2[g], false);
		}
	}
	
	if ( (sense == CX_SENSE_ORIGINAL_ORIGINAL) || (sense == CX_SENSE_ORIGINAL_REVERSE) )
	{
		for (int g = bag1.size()-1; g > -1; g--)
		{
			trip2.insertGift(inst,j+1,bag1[g], false);
		}
	}
	else
	{
		for (int g = 0; g < (int)bag1.size(); g++)
		{
			trip2.insertGift(inst,j+1,bag1[g], false);
		}
	}

	trip1.UpdateAuxiliaryDataStructures(inst);
	trip2.UpdateAuxiliaryDataStructures(inst);

	//cout << "trip1: " << trip1.printTrip() << endl;
	//cout << "trip2: " << trip2.printTrip() << endl;
}

double evaluateInterChainRelocation(Instance & inst, Trip & trip1, Trip & trip2, int i, int k, int j)
{	
	double weightChain = trip1.getWeightAt(i-1) - trip1.getWeightAt(k);

	if ( (trip2.getWeight() + weightChain > inst.weightLimit) )
	{
		return 1e20;
	}

	double cost1 = trip1.getCostUntil(i-1) - weightChain * trip1.getDistanceUntil(i-1);
	double weight1 = trip1.getWeightAt(i-1) - weightChain;
	cost1 += inst.getDistance(trip1.getGiftId(i-1), trip1.getGiftId(k+1))*weight1;
	cost1 += trip1.getCostAhead(k+1);
		
	double cost2 = trip2.getCostUntil(j-1) + weightChain * trip2.getDistanceUntil(j-1);
	double weight2 = trip2.getWeightAt(j-1) + weightChain;
	cost2 += inst.getDistance(trip2.getGiftId(j-1), trip1.getGiftId(i))*weight2;
	for (int g = i; g < k; g++)
	{
		weight2 -= trip1.getWeight(g);
		cost2 += trip1.getDistanceFrom(g)*weight2;
	}
	weight2 -= trip1.getWeight(k);
	cost2 += inst.getDistance(trip1.getGiftId(k), trip2.getGiftId(j))*weight2;
	cost2 += trip2.getCostAhead(j);

	return ( cost1 + cost2 ) - ( trip1.getCost() + trip2.getCost() );
}

void moveInterChainRelocation(Instance & inst, Trip & trip1, Trip & trip2, int i, int k, int j)
{
	vector<Gift> bag;

	for (int g = i; g <= k; g++)
	{
		bag.push_back(trip1.getGift(i));
		trip1.eraseGift(inst,i);
	}

	for (int g = bag.size()-1; g > -1; g--)
	{
		trip2.insertGift(inst,j,bag[g]);
	}
}

double Trip::evaluateIntraChainRelocation(Instance & inst, int i, int k, int j)
{	
	double cost = 0;

	if (k < j)
	{
		cost = getCostUntil(i-1);
		double weight = getWeightAt(i-1);
		cost += inst.getDistance(getGiftId(i-1), getGiftId(k+1))*weight;
		for (int g = k+1; g < j-1; g++)
		{
			weight -= getWeight(g);
			cost += getDistanceFrom(g)*weight;
		}
		weight -= getWeight(j-1);
		cost += inst.getDistance(getGiftId(j-1), getGiftId(i))*weight;
		for (int g = i; g < k; g++)
		{
			weight -= getWeight(g);
			cost += getDistanceFrom(g)*weight;
		}
		weight -= getWeight(k);
		cost += inst.getDistance(getGiftId(k), getGiftId(j))*weight;
		cost += getCostAhead(j);
	}
	else
	{
		cost = getCostUntil(j-1);
		double weight = getWeightAt(j-1);
		cost += inst.getDistance(getGiftId(j-1), getGiftId(i))*weight;
		for (int g = i; g < k; g++)
		{
			weight -= getWeight(g);
			cost += getDistanceFrom(g)*weight;
		}
		weight -= getWeight(k);
		cost += inst.getDistance(getGiftId(k), getGiftId(j))*weight;
		for (int g = j; g < i-1; g++)
		{
			weight -= getWeight(g);
			cost += getDistanceFrom(g)*weight;
		}
		weight -= getWeight(i-1);
		cost += inst.getDistance(getGiftId(i-1), getGiftId(k+1))*weight;
		cost += getCostAhead(k+1);
	}	

	return cost - getCost();
}

void Trip::moveIntraChainRelocation(Instance & inst, int i, int k, int j)
{
	vector<Gift> bag;
	int new_j = j;

	for (int g = i; g <= k; g++)
	{
		bag.push_back(getGift(i));
		eraseGift(inst,i,false);
		if (j > k)
		{
			new_j--;
		}
	}

	//insert before j in reverse order to yield right ordering
	for (int g = bag.size()-1; g > -1; g--)
	{
		insertGift(inst, new_j, bag[g], false);
	}

	UpdateAuxiliaryDataStructures(inst);
}

double evaluate2OptStar(Instance & inst,  Trip & trip1, Trip & trip2, int i, int j)
{
	double weightChain1 = trip1.getWeightAt(i-1);
	double weightChain2 = trip2.getWeightAt(j-1);

	if ( (trip1.getWeight() - weightChain1 + weightChain2 > inst.weightLimit) ||
		(trip2.getWeight() - weightChain2 + weightChain1 > inst.weightLimit) )
	{
		return 1e15;
	}

	double cost1 = trip1.getCostUntil(i-1) + (weightChain2 - weightChain1) * trip1.getDistanceUntil(i-1);
	double weight1 = trip1.getWeightAt(i-1) - weightChain1 + weightChain2;
	cost1 += inst.getDistance(trip1.getGiftId(i-1), trip2.getGiftId(j))*weight1;
	cost1 += trip2.getCostAhead(j);
		
	double cost2 = trip2.getCostUntil(j-1) + (weightChain1 - weightChain2) * trip2.getDistanceUntil(j-1);
	double weight2 = trip2.getWeightAt(j-1) - weightChain2 + weightChain1;
	cost2 += inst.getDistance(trip2.getGiftId(j-1), trip1.getGiftId(i))*weight2;
	cost2 += trip1.getCostAhead(i);

	return ( cost1 + cost2 ) - ( trip1.getCost() + trip2.getCost() );
}

void move2OptStar(Instance & inst,  Trip & trip1, Trip & trip2, int i, int j)
{
	vector<Gift> bag1;
	vector<Gift> bag2;
	int size1 = trip1.getSize();
	int size2 = trip2.getSize();

	for (int g = i; g < size1; g++)
	{
		bag1.push_back(trip1.getGift(i));
		trip1.eraseGift(inst,i);
	}
	for (int g = j; g < size2; g++)
	{
		bag2.push_back(trip2.getGift(j));
		trip2.eraseGift(inst,j);
	}

	for (int g = bag2.size()-1; g > -1; g--)
	{
		trip1.insertGift(inst,i,bag2[g]);
	}
	for (int g = bag1.size()-1; g > -1; g--)
	{
		trip2.insertGift(inst,j,bag1[g]);
	}
}

double evaluate2InterRelocation(Instance & inst,  Trip & tripFrom, Trip & tripTo, int i, int j)
{
	double cost1 = tripFrom.getCostUntil(i-1) - (tripFrom.getWeight(i) + tripFrom.getWeight(i+1)) * tripFrom.getDistanceUntil(i-1); //custo até presente i-1 - perda dos 2 que sairam
	cost1 +=  inst.getDistance(tripFrom.getGiftId(i-1), tripFrom.getGiftId(i+2))*tripFrom.getWeightAt(i+1); //custo do presente i-1 ao i+2, devido ao erase dos i e i+1
	cost1 += tripFrom.getCostAhead(i+2); //custo alem do presente i+2
	
	double cost2 = tripTo.getCostUntil(j-1) + (tripFrom.getWeight(i) + tripFrom.getWeight(i+1)) * tripTo.getDistanceUntil(j-1);
	double weight2 = tripTo.getWeightAt(j-1) + tripFrom.getWeight(i) + tripFrom.getWeight(i+1);

	cost2 +=  inst.getDistance(tripTo.getGiftId(j-1), tripFrom.getGiftId(i))*weight2;
	weight2 -= tripFrom.getWeight(i);
	cost2 +=  tripFrom.getDistanceFrom(i)*weight2;
	weight2 -= tripFrom.getWeight(i+1);
	cost2 +=  inst.getDistance(tripFrom.getGiftId(i+1), tripTo.getGiftId(j))*weight2;
	cost2 += tripTo.getCostAhead(j);
	
	return ( cost1 + cost2 ) - ( tripFrom.getCost() + tripTo.getCost() );
}

void move2InterRelocation(Instance & inst,  Trip & tripFrom, Trip & tripTo, int i, int j)
{
	Gift gift1 = tripFrom.getGift(i);
	Gift gift2 = tripFrom.getGift(i+1);

	tripFrom.eraseGift(inst,i);
	tripFrom.eraseGift(inst,i);

	tripTo.insertGift(inst, j, gift2);
	tripTo.insertGift(inst, j, gift1);
}

//queries
double Trip::getWeight()
{
	return weight;
}

double Trip::getPolarWeight()
{
	double polarWeight = 0;
	for (int i = 0; i < (int)Gifts.size(); i++)
	{
		if (Gifts[i].isPolar())
		{
			polarWeight += Gifts[i].weight;
		}
	}
	return polarWeight;
}


double Trip::getRealWeight()
{
	double w = 0;
	for (int i = 0; i < getSize(); i++)
	{
		w += getWeight(i);
	}
	return w + 10;
}

double Trip::getSmallestWeight()
{
	return smallestWeight;
}

double Trip::getWeight(int index)
{
	if ((index == -1) || (index == size))
	{
		return 0;
	}

	return Gifts[index].weight;
}

int Trip::getSize()
{
	return size;
}

int Trip::getSize() const
{
	return size;
}

int Trip::getGiftId(int index)
{
	if (index == -1)
	{
		return 0; //north pole
	}
	if (index == size)
	{
		return 0; //north pole
	}
	else
	{
		return Gifts[index].id;
	}
}

double Trip::getWeightAt(int index, bool excludeSleigh)
{
	if (index == -1)
	{
		return weight;
	}
	else if (index == size)
	{
		return 0;
	}
	else
	{
		if (excludeSleigh)
		{
			return  weightAt[index+1] - 10;
		}
		else
		{
			return  weightAt[index+1];
		}
	}
}

double Trip::getCostFrom(int index)
{
	//if (index == -1)
	//{
	//	return costFrom[0]; //north pole
	//}
	if (index == size)
	{
		return 0;
	}
	else
	{		
		return  costFrom[index+1];
	}

}

double Trip::getDistanceFrom(int index)
{
	if (index == size)
	{
		return 0;
	}
	else
	{
		return distanceFrom[index+1];
	}
}

double Trip::getDistanceAhead(int index)
{
	if (index == size)
	{
		return 0;
	}

	return  distanceAhead[index+1];	
}

double Trip::getDistanceUntil(int index)
{
	if (index == size)
	{
		return distanceAhead[0];
	}

	return  distanceUntil[index+1];	
}

double Trip::getCostUntil(int index)
{
	if (index == size)
	{
		return costFrom[0];
	}
	else
	{
		return costUntil[index+1];
	} 
}

double Trip::getCostAhead(int index)
{
	if (index == -1)
	{
		return cost;
	}
	else if (index == size)
	{
		return 0;
	}
	else
	{
		return  costAhead[index+1];
	}
}

Gift & Trip::getGift(int index)
{
	return Gifts[index];
}

Gift & Trip::getBackGift()
{
	return Gifts.back();
}

Gift & Trip::getFrontGift()
{
	return Gifts.front();
}

double Trip::getCost()
{
	return cost;
}

string Trip::printTripDebug()
{
	stringstream msg;

	msg << printTrip() << endl;

	msg << "distanceFrom:" << endl; 
	for (int i = 0; i < (int)distanceFrom.size(); i++)
	{
		msg << "[" << i << "]: " << distanceFrom[i] << endl;
	}

	msg << "weightAt:" << endl; 
	for (int i = 0; i < (int)weightAt.size(); i++)
	{
		msg << "[" << i << "]: " << weightAt[i] << endl;
	}

	msg << "costFrom:" << endl; 
	for (int i = 0; i < (int)costFrom.size(); i++)
	{
		msg << "[" << i << "]: " << costFrom[i] << endl;
	}

	return msg.str();
}

double Trip::getLength()
{
	return distanceAhead[0];
}

double Trip::getMinLatitude()
{
	 return minLatitude;
}

double Trip::getMaxLatitude()
{
	 return maxLatitude;
}

double Trip::getMaxLongitude()
{
	 return maxLongitude;
}

double Trip::getMinLongitude()
{
	 return minLongitude;
}

double Trip::getGiftLatitude(int index) const
{
	return Gifts[index].latitude;
}

double Trip::getGiftLongitude(int index) const
{
	return Gifts[index].longitude;
}

double Trip::getCost() const
{
	return cost;
}

double Trip::getLatitude_MassCenter()
{
	return latitude_MassCenter;
}

double Trip::getLongitude_MassCenter()
{
	return longitude_MassCenter;
}

double Trip::avgLongitude() const
{
	double avg = 0;

	for (int i = 0; i < (int)Gifts.size(); i++)
	{
		double giftLongitude = Gifts[i].longitude+LONGITUDE_OFFSET;
		if (giftLongitude > 180)
		{
			giftLongitude -= 360;
		}
		avg += giftLongitude;
	}
	avg = avg/Gifts.size();

	return avg;
}

double Trip::avgMCLongitude() const
{
	double avg = 0;
	double totalWeight = 0;
	for (int i = 0; i < (int)Gifts.size(); i++)
	{
		double giftLongitude = Gifts[i].longitude+LONGITUDE_OFFSET;
		if (giftLongitude > 180)
		{
			giftLongitude -= 360;
		}
		avg += giftLongitude * Gifts[i].weight;
		totalWeight +=  Gifts[i].weight;
	}
	avg = avg/totalWeight;
	double alternativeAvg = avg + 180;

	if (alternativeAvg > 180)
	{
		alternativeAvg -= 360;
	}

	double avgAcumDev = 0;
	double alternativeAvgAcumDev = 0;

	for (int i = 0; i < (int)Gifts.size(); i++)
	{
		double dev1 = abs(avg - Gifts[i].longitude);
		if (dev1 > 180)
		{
			dev1 = 360 - dev1;
		}

		double dev2 = abs(alternativeAvg - Gifts[i].longitude);
		if (dev2 > 180)
		{
			dev2 = 360 - dev2;
		}

		avgAcumDev += dev1;
		alternativeAvgAcumDev += dev2;
	}

	if (avgAcumDev < alternativeAvgAcumDev)
	{		
		return alternativeAvg;
	}
	else
	{		
		return avg;
	}
}

double InterMassCenterDistance(Instance & inst, Trip & trip1, Trip & trip2)
{
	return inst.getHaversineDistance(trip1.getLatitude_MassCenter(), trip1.getLongitude_MassCenter(),
			trip2.getLatitude_MassCenter(), trip2.getLongitude_MassCenter());
}



//updating procedures
void Trip::setCost(double cost_)
{
	cost = cost_;
}

void Trip::UpdateAuxiliaryDataStructures(Instance & inst)
{
	//updates the following data

		//size; ok

		//weight; ok
		//weightAt; ok
		//smallestWeight; ok

		//distanceFrom; ok
		//distanceUntil; ok
		//distanceAhead; ok
	
		//cost; ok
		//costFrom; ok
		//costUntil; ok
		//costAhead; ok
	
	size = Gifts.size();

	weightAt.resize(size+1);
	distanceFrom.resize(size+1);
	distanceUntil.resize(size+1);
	distanceAhead.resize(size+1);
	costFrom.resize(size+1);
	costUntil.resize(size+1);
	costAhead.resize(size+1);
	
	if (size == 0)
	{
		cost = 0;
		weight = inst.sleighWeight; 
		smallestWeight = 0;

		weightAt[0] = weight;
		distanceFrom[0] = 0;
		distanceUntil[0] = 0;
		distanceAhead[0] = 0;
		costFrom[0] = 0;
		costUntil[0] = 0;
		costAhead[0] = 0;
				
		return;
	}

	//calculates weigh and smallestWeight
	weight = inst.sleighWeight;
	smallestWeight = 51;
	for (int i = 0; i < size; i++)
	{
		weight += Gifts[i].weight; 

		double weight_i = getWeight(i);
		if (weight_i < smallestWeight)
		{
			smallestWeight = weight_i;
		}
	}

	//calculates weightAt
	weightAt[0] = weight;
	for (int i = 1; i < (int)weightAt.size(); i++)
	{
		weightAt[i] = weightAt[i-1] - Gifts[i-1].weight; 
	}

	//calculates distanceFrom
	distanceFrom.front() = Gifts.front().npDistance;
	distanceFrom.back() = Gifts.back().npDistance;
	for (int i = 0; i < (int)Gifts.size()-1; i++)
	{
		distanceFrom[i+1] = inst.getDistance(Gifts[i].id, Gifts[i+1].id); 
	}

	//calculates cost and costFrom
	cost = 0;
	for (int i = 0; i < (int)weightAt.size(); i++)
	{
		double arcCost = weightAt[i]*distanceFrom[i];
		cost += arcCost;
		costFrom[i] = arcCost; 
	}

	//calculates distanceUntil and costUntil
	distanceUntil.front() = 0;
	costUntil.front() = 0;
	for (int i = 1; i < (int)distanceFrom.size(); i++)
	{
		distanceUntil[i] = distanceUntil[i-1] + distanceFrom[i-1];
		costUntil[i] = costUntil[i-1] + costFrom[i-1];
	}

	//calculates distanceAhead and costAhead
	distanceAhead.back() = distanceFrom.back();
	costAhead.back() = costFrom.back();
	for (int i = (int)distanceFrom.size()-2; i > -1; i--)
	{
		distanceAhead[i] = distanceAhead[i+1] + distanceFrom[i];
		costAhead[i] = costAhead[i+1] + costFrom[i];
	}
}

void Trip::CalculateCoordinateRange()
{
	 maxLatitude = getGiftLatitude(0);
	 minLatitude = getGiftLatitude(0);
	 maxLongitude = getGiftLongitude(0);
	 minLongitude = getGiftLongitude(0);

	 for (int i = 1; i < size; i++)
	 {
		 if (getGiftLatitude(i) > maxLatitude)
		 {
			 maxLatitude = getGiftLatitude(i);
		 }
		 if (getGiftLatitude(i) < minLatitude)
		 {
			 minLatitude = getGiftLatitude(i);
		 }
		 if (getGiftLongitude(i) > maxLongitude)
		 {
			 maxLongitude = getGiftLongitude(i);
		 }
		 if (getGiftLongitude(i) < minLongitude)
		 {
			 minLongitude = getGiftLongitude(i);
		 }
	 }
}

void Trip::CalculateMassCenter()
{
	latitude_MassCenter = 0;
	longitude_MassCenter = 0;

	for (int i = 0; i < getSize(); i++)
	{
		latitude_MassCenter += getGiftLatitude(i)*getWeight(i)/getWeight();
		longitude_MassCenter += getGiftLongitude(i)*getWeight(i)/getWeight();
	}
}



//local search procedures
void Trip::search2Opt(Instance & inst, bool & improveFlag, double & account, bool showLog)
{
	if ( getNeighborhoodStatus(NEIGHBORHOOD_2Opt) )
	{
		return;
	}

	double initialCost = getCost();
	
	stringstream msg;
	msg << "search2Opt Initial  cost:\t" << setprecision(17) << initialCost << endl;

	int best_i = 0;
	int best_j = 0;
	bool hasSwap = true;

	while (hasSwap)
	{
		hasSwap = false;
		double bestIncrease = 0;
		for (int i = 0; i < getSize()-2; i++)
		{
			for (int j = i+2; j < getSize(); j++)
			{	
				double increase = evaluate2Opt(inst, i, j);

				/*cout << printTrip() << endl;
				double a = -getCost();
				set2OPT(inst, i, j);
				a += getCost();

				if (abs(a - increase) > 0.01)
				{
					cerr << "error in evaluate2OPT";
					throw -1;
				}*/

				if (increase < bestIncrease - RELEVANT_INCREASE)
				{
					bestIncrease = increase;
					best_i = i;
					best_j = j;
					hasSwap = true;
				}	
				//set2OPT(inst, i, j);
			}
		}

		if (hasSwap)
		{
			improveFlag = true;

			move2Opt(inst, best_i, best_j);
		}
	}

	double finalCost = getCost();
	msg << "search2Opt Final cost:\t" << std::setprecision(17) << finalCost << endl;

	if (finalCost < initialCost - RELEVANT_INCREASE)
	{
		if (showLog)
		{
			cout << msg.str();
		}

		account += initialCost - finalCost;
		resetNeighborhoodStatuses();
	}
	setNeighborhoodCleared(NEIGHBORHOOD_2Opt);
}

void Trip::move2Opt(Instance & inst, int i, int j)
{
	vector<Gift> bagOfGifts;

	for (int k = j; k > i; k--)
	{
		bagOfGifts.push_back(getGift(k));
	}
	
	auto it = bagOfGifts.begin();
	for (int k = i+1; k <= j; k++)
	{
		setGift(inst, k, *it, false);
		++it;
	}

	UpdateAuxiliaryDataStructures(inst);
}

void Trip::search3Opt(Instance & inst, bool & improveFlag, double & account, bool showLog )
{
	if ( getNeighborhoodStatus(NEIGHBORHOOD_3Opt) )
	{
		return;
	}

	double initialCost = getCost();
	
	stringstream msg;
	msg << "search3Opt Initial  cost:\t" << std::setprecision(17) << initialCost << endl;

	int best_i = 0;
	int best_j = 0;
	int best_k = 0;
	int best_identifier = 0;
	bool hasSwap = true;

	while (hasSwap)
	{
		hasSwap = false;
		double bestIncrease = 0;
		for (int i = 0; i < getSize()-4; i++)
		{
			for (int j = i+2; j < getSize()-2; j++)
			{	
				for (int k = j+2; k < getSize(); k++)
				{
					int identifier = 0;
					double increase = evaluate3Opt(inst, i, j, k, identifier);

					if (increase < bestIncrease - RELEVANT_INCREASE)
					{
						bestIncrease = increase;
						best_i = i;
						best_j = j;
						best_k = k;
						best_identifier = identifier;
						hasSwap = true;
					}	
				}
			}
		}

		if (hasSwap)
		{
			improveFlag = true;
			double oldCost = getCost(); 
			move3Opt(inst, best_i, best_j, best_k, best_identifier);
			double newCost = getCost(); 

			if (abs(newCost - oldCost - bestIncrease) > 0.01)
			{
				cout << oldCost << "\t" << newCost << endl; 
				cout << newCost - oldCost << "\t" << bestIncrease << endl; 
				cerr << "error in 3Opt";
				throw -1;
			}
		}
	}

	double finalCost = getCost();
	msg << "search3Opt Final cost:\t" << std::setprecision(17) << finalCost << endl;

	if (finalCost < initialCost - RELEVANT_INCREASE)
	{
		if (showLog)
		{
			cout << msg.str();
		}
		account += initialCost - finalCost;		
		resetNeighborhoodStatuses();
	}
	setNeighborhoodCleared(NEIGHBORHOOD_3Opt);
}

void Trip::search4Opt(Instance & inst, bool & improveFlag, double & account, bool showLog )
{
	if ( getNeighborhoodStatus(NEIGHBORHOOD_4Opt) )
	{
		return;
	}

	double initialCost = getCost();
	
	stringstream msg;
	msg << "search4Opt Initial  cost:\t" << std::setprecision(17) << initialCost << endl;

	int best_i = 0;
	int best_j = 0;
	int best_k = 0;
	int best_l = 0;
	int best_identifier = 0;
	bool hasSwap = true;

	while (hasSwap)
	{
		hasSwap = false;
		double bestIncrease = 0;
		for (int i = 0; i < getSize()-6; i++)
		{
			for (int j = i+2; j < getSize()-4; j++)
			{	
				for (int k = j+2; k < getSize()-2; k++)
				{
					for (int l = k+2; l < getSize(); l++)
					{
						int identifier = 0;
						double increase = evaluate4Opt(inst, i, j, k, l, identifier);

						if (increase < bestIncrease - RELEVANT_INCREASE)
						{
							bestIncrease = increase;
							best_i = i;
							best_j = j;
							best_k = k;
							best_l = l;
							best_identifier = identifier;
							hasSwap = true;
						}	
					}
				}
			}
		}

		if (hasSwap)
		{
			improveFlag = true;
			double oldCost = getCost(); 
			move4Opt(inst, best_i, best_j, best_k, best_l, best_identifier);
			double newCost = getCost(); 

			if (abs(newCost - oldCost - bestIncrease) > 0.01)
			{
				cout << oldCost << "\t" << newCost << endl; 
				cout << newCost - oldCost << "\t" << bestIncrease << endl; 
				cerr << "error in 4Opt";
				throw -1;
			}
		}
	}

	double finalCost = getCost();
	msg << "search4Opt Final cost:\t" << std::setprecision(17) << finalCost << endl;

	if (finalCost < initialCost - RELEVANT_INCREASE)
	{
		if (showLog)
		{
			cout << msg.str();
		}
		account += initialCost - finalCost;		
		resetNeighborhoodStatuses();
	}
	setNeighborhoodCleared(NEIGHBORHOOD_4Opt);
}

void Trip::searchIntraRelocation(Instance & inst, bool & improveFlag, double & account, bool showLog)
{
	if ( getNeighborhoodStatus(NEIGHBORHOOD_IntraRelocation) )
	{
		return;
	}
		
	double initialCost = getCost();
	
	stringstream msg;
	msg << "searchIntraRelocation Initial  cost:\t" << std::setprecision(17) << initialCost << endl;

	int best_i = 0;
	int best_j = 0;
	bool hasSwap = true;

	while (hasSwap)
	{
		hasSwap = false;
		double bestCost = getCost();
		for (int i = 0; i < getSize(); i++)
		{
			Gift gift = getGift(i);
			eraseGift(inst,i); //to refactor
			for (int j = 0; j <= getSize(); j++)
			{	
				if (i !=j)
				{	
					double newCost = getCost() + evaluateInsertion(inst, j, gift);				

					if (newCost < bestCost - RELEVANT_INCREASE)
					{
						bestCost = newCost;
						best_i = i;
						best_j = j;
						hasSwap = true;
					}					
				}
			}
			insertGift(inst,i,gift);
		}

		if (hasSwap)
		{
			improveFlag = true;

			Gift gift = getGift(best_i);
			eraseGift(inst,best_i,false);
			insertGift(inst,best_j,gift,false);
			UpdateAuxiliaryDataStructures(inst);
		}
	}

	double finalCost = getCost();
	msg << "searchIntraRelocation Final cost:\t" << std::setprecision(17) << finalCost << endl;
	if (finalCost < initialCost - RELEVANT_INCREASE)
	{
		if (showLog)
		{
			cout << msg.str();
		}
		account += initialCost-finalCost;
		resetNeighborhoodStatuses();
	}
	setNeighborhoodCleared(NEIGHBORHOOD_IntraRelocation);
}

void Trip::searchIntraChainRelocation(Instance & inst, bool & improveFlag, double & account, bool showLog)
{
	if ( getNeighborhoodStatus(NEIGHBORHOOD_IntraChainRelocation) )
	{
		return;
	}
	if (getSize() < 4)
	{
		return;
	}
	double initialCost = getCost();
	
	stringstream msg;
	msg << "searchIntraChainRelocation Initial  cost:\t" << std::setprecision(17) << initialCost << endl;

	bool hasSwap = true;
	while (hasSwap)
	{
		double bestIncrease = 0;
		int best_i = 0;
		int best_j = 0;
		int best_k = 0;
		hasSwap = false;
		for (int i = 0; i < (int)getSize()-1; i++)
		{
			for (int k = i+1; k < (int)getSize(); k++)
			{	
				for (int j = 0; j <= (int)getSize(); j++)
				{
					if ( (j < i-1) || (j > k+1) )
					{
						double increase = evaluateIntraChainRelocation(inst, i, k, j);
						
						if (increase < bestIncrease - RELEVANT_INCREASE)
						{
							bestIncrease = increase;
							hasSwap = true;
							improveFlag = true;
							best_i = i;
							best_k = k;
							best_j = j;
						}	
					}								
				}
			}
		}
		if (hasSwap)
		{
			//cout << evaluateCrossExchange(inst, trip, best_i, best_k, best_j) << endl;
			//cout << printTrip() << "\t" << getCost() << endl << endl; 
			double oldCost = getCost();
			moveIntraChainRelocation(inst, best_i, best_k, best_j);
			double newCost = getCost();
			//cout << printTrip() << "\t" << getCost() << endl << endl;   
			if (abs(newCost-oldCost-bestIncrease) > 0.01)
			{
				cout << "Error on IntraChainRelocation trips"<< endl;
				cout << printTrip() << "\t" << getCost() << endl; 
				cout << best_i  << "\t" << best_k  << "\t" << best_j << endl;
				cout << newCost << endl;				
				cout << oldCost << endl;
				cout << newCost - oldCost << "\t" << bestIncrease << endl;
				throw -1;
			}
		}
	}
	
	double finalCost = getCost();
	msg << "searchIntraChainRelocation Final cost:\t" << std::setprecision(17) << finalCost << endl;
	if (finalCost < initialCost - RELEVANT_INCREASE)
	{
		if (showLog)
		{
			cout << msg.str();
		}
		account += initialCost-finalCost;
		resetNeighborhoodStatuses();
	}
	setNeighborhoodCleared(NEIGHBORHOOD_IntraChainRelocation);	
}	

void Trip::searchIntraExchange(Instance & inst, bool & improveFlag, double & account, bool showLog)
{
	if ( getNeighborhoodStatus(NEIGHBORHOOD_IntraExchange) )
	{
		return;
	}
	double initialCost = getCost();
	double bestCost = initialCost;

	stringstream msg;
	msg << "searchIntraExchange Initial cost: " << std::setprecision(17) << initialCost << endl;
	
	bool hasSwap = true;

	while (hasSwap)
	{
		int best_i = 0;
		int best_j = 0;
		hasSwap = false;
		for (int i = 0; i < (int)getSize() - 1; i++)
		{
			for (int j = i+1; j < (int)getSize(); j++)
			{
				swapGifts(inst,i,j);
				double newCost = getCost();				
				if (newCost < bestCost - RELEVANT_INCREASE)
				{
					bestCost = newCost;
					hasSwap = true;
					improveFlag = true;
					best_i = i;
					best_j = j;
				}
				swapGifts(inst,j,i);
			}
		}
		if (hasSwap)
		{
			swapGifts(inst,best_i,best_j);
		}
	}

	double finalCost = getCost();
	msg << "searchIntraExchange Final cost: " << std::setprecision(17) << finalCost << endl;
	if (finalCost < initialCost - RELEVANT_INCREASE)
	{
		if (showLog)
		{
			cout << msg.str();
		}
		account += initialCost-finalCost;
		resetNeighborhoodStatuses();
	}
	setNeighborhoodCleared(NEIGHBORHOOD_IntraExchange);
}


//optimization procedures
#ifndef EC2
void Trip::optTSP(Instance & inst)
{
	CPUTimer t;
	//t.start();

	cout  << "Optimizing Trip by TSP Integer Model to CPLEX..." << endl;

	vector<vector<double> > c(size+1);

	for (int i = 0; i < size+1; i++)
	{
		c[i].resize(size+1);
	}

	for (int i = 0; i < size+1; i++)
	{
		for (int j = i+1; j < size+1; j++)
		{
			c[i][j] = inst.getDistance(getGiftId(i), getGiftId(j));			
			c[j][i] = c[i][j];
		} 
	}

	IloEnv env;
	IloModel model(env);		
	IloObjective obj = IloMinimize(env);
	NumVarMatrix var_x(env);
	string varName;

	for (int i = 0; i < size+1; i++)
	{
		var_x.add(IloNumVarArray(env));
	}

	for (int i = 0; i < size+1; i++)
	{
		for (int j = 0; j < size+1; j++)
		{
			varName = "x" + to_string(getGiftId(i)) + "_" + to_string(getGiftId(j));
			var_x[i].add( IloNumVar(env, 0, 1, IloNumVar::Int, (char*)varName.c_str()) );
		}
	}

	for (int i = 0; i < size+1; i++)
	{
		for (int j = 0; j < size+1; j++)
		{
			obj.setLinearCoef(var_x[i][j], c[i][j]);
		}
	}
	model.add(obj);

	// (2) - tudo que sai de todo i
	for (int i = 0; i < size+1; i++)
	{
		IloRange c2(env, 1, 1);

		for (int j = 0; j < size+1; j++)
		{
			if (c[i][j] > 0)
			{
				c2.setLinearCoef(var_x[i][j], 1);
			}
		}
		model.add(c2);
	}

	// (3) - tudo que entra em todo j
	for (int j = 0; j < size+1; j++)
	{
		IloRange c3(env, 1, 1);

		for (int i = 0; i < size+1; i++)
		{
			if (c[i][j] > 0)
			{
				c3.setLinearCoef(var_x[i][j], 1);
			}
		}
		model.add(c3);
	}
	
	
	// Optimize the problem and obtain solution.
	set<int> M;
	vector<vector<int> > adjacencyMatrix (size+1);
	for (int i = 0; i < size+1; i++)
	{
		adjacencyMatrix[i].resize(size + 1);
	}

	bool cplexFail = false;
	int subTourCount = 0;

	while ( ( (int) M.size() < size +1) && (!cplexFail) )
	{	
		t.start();
		cout << subTourCount << " subtour elimination constraints..." << endl;
		cout << "Time: " << t.getCPUTotalSecs() << endl;
		//clear adjacencyMatrix
		for (int i = 0; i < size; i++)
		{
			for (int j = i+1; j < size+1; j++)
			{
				adjacencyMatrix[i][j] = 0;
				adjacencyMatrix[j][i] = 0;
			}			
		}

		IloCplex cplex(model);
		#ifdef _WIN32
			cplex.setParam(IloCplex::Threads,1);
		#endif	

		try {
			cplex.exportModel("trip_tsp.lp"); 
		}
		catch (IloException e) 
		{
			cerr << e;
		}
		
		
		try {
			 if ( cplex.solve() ) {   
				cout << "Solution status = " << cplex.getStatus() << endl;
				double objValue = cplex.getObjValue();
    
				int i = 0;
				for (i = 0; i < size+1; i++)
				{
					for (int j = 0; j < size+1; j++)
					{	
						if (c[i][j] > 0)
						{				
							double val = cplex.getValue(var_x[i][j]);
							if (val != 0)
							{
								cout << var_x[i][j].getName() << ": " << val << endl;
								adjacencyMatrix[i][j] = 1;
								adjacencyMatrix[j][i] = 1;
							}
						}				
					}	
				}
		
				M.clear();
				M.insert(size);

				int last = size;
				i = 0;

				while ((int) M.size() > i)
				{
					for (int j = 0; j < size+1; j++)
					{
						if ( ( adjacencyMatrix[last][j] == 1) && ( M.find(j) == M.end()) )
						{
							M.insert(j);
							last = j;
							break;
						}
					}
					i++;
				}

				IloRange subTourElimination(env, 2, IloInfinity);

				for (auto it = M.begin(); it != M.end(); ++it)
				{
					for (int j = 0; j < size + 1; j++)
					{
						if ( ( *it != j ) && ( M.find(j) == M.end() ) )
						{
							if (c[*it][j] > 0)
							{
								subTourElimination.setLinearCoef(var_x[*it][j], 1);
							}
							if (c[j][*it] > 0)
							{
								subTourElimination.setLinearCoef(var_x[j][*it], 1);
							}
						}
					}
				}
				model.add(subTourElimination);
				subTourCount++;

				cout << "M= {";
				for (set<int>::iterator it = M.begin(); it != M.end(); it++)
				{
					cout << getGiftId(*it) << ",";
				}
				cout << "}" << endl;		
			}	
			else
			{
				cout << "No integer inequality found" << endl;
			}
		 }
		catch (IloException e) 
		{
			cerr << e;
			cplexFail = true;
			break;
		}
		
		t.stop();
		if (t.getCPUTotalSecs() > 60)
		{
			cplexFail = true;
		}
		t.start();
	}
		
	if (!cplexFail)
	{
		vector<bool> visited(size+1);

		//starts at north pole
		visited[size] = true;
		int last = size;
	
		Trip TSP_Trip(inst);

		while (TSP_Trip.getSize() < size)
		{
			for (int j = 0; j < size+1; j++)
			{
				if ( ( adjacencyMatrix[last][j] == 1) && (!visited[j] ) )
				{
					visited[j] = true;	
					TSP_Trip.appendGift(inst, getGift(j));
					last = j;
					break;
				}
			}
		}

	
		vector<bool> visited2(size+1);

		//starts at north pole
		visited2[size] = true;
		last = size;

		Trip TSP_ReverseTrip(inst);

		while (TSP_ReverseTrip.getSize() < size)
		{
			for (int j = size; j > -1; j--)
			{
				if ( ( adjacencyMatrix[last][j] == 1) && (!visited2[j] ) )
				{
					visited2[j] = true;	
					TSP_ReverseTrip.appendGift(inst, getGift(j));
					last = j;
					break;
				}
			}
		}
		
		cout << "Original trip: " << printTrip() << endl;	
		cout << "Original cost: " << getCost() << endl;	

		cout << "TSP Optimal tour: " << TSP_Trip.printTrip() << endl;		
		cout << "TSP Optimal cost: " << TSP_Trip.getCost() << endl;

		cout << "Reverse TSP Optimal tour: " << TSP_ReverseTrip.printTrip() << endl;		
		cout << "ReverseTSP Optimal cost: " << TSP_ReverseTrip.getCost() << endl;

		if ( ( TSP_Trip.getCost() < getCost() ) || (TSP_ReverseTrip.getCost() < getCost()) )
		{
			if (TSP_Trip.getCost() < TSP_ReverseTrip.getCost())
			{		
				cloneTrip(inst, TSP_Trip);
			}
			else
			{
				cloneTrip(inst, TSP_ReverseTrip);
			}
		}
	}

	env.end();
}

void Trip::optKara2007(Instance & inst, bool & improveFlag)
{
	if ( getNeighborhoodStatus(NEIGHBORHOOD_KARA) )
	{
		return;
	}

	CPUTimer t;
	t.start();

	cout  << "Optimizing Trip by Kara2007 Two-Index One-Commodity Flow Integer Model to CPLEX..." << endl;
	cout  << "Number of Gifts:\t" << getSize() << endl;

	vector<vector<double> > c(size+1);

	for (int i = 0; i < size+1; i++)
	{
		c[i].resize(size+1);
	}

	for (int i = 0; i < size+1; i++)
	{
		for (int j = i+1; j < size+1; j++)
		{
			c[i][j] = inst.getDistance(getGiftId(i), getGiftId(j));			
			c[j][i] = c[i][j];
		} 
	}

	IloEnv env;
	IloModel model(env);		
	IloObjective obj = IloMinimize(env);
	NumVarMatrix var_x(env);
	NumVarMatrix var_f(env);
	string varName;
	string consName;

	for (int i = 0; i < size+1; i++)
	{
		var_x.add(IloNumVarArray(env));
		var_f.add(IloNumVarArray(env));
	}

	for (int i = 0; i < size+1; i++)
	{
		for (int j = 0; j < size+1; j++)
		{
			varName = "x" + to_string(getGiftId(i)) + "_" + to_string(getGiftId(j));
			var_x[i].add( IloNumVar(env, 0, 1, IloNumVar::Int, (char*)varName.c_str()) );

			varName = "f" + to_string(getGiftId(i)) + "_" + to_string(getGiftId(j));
			var_f[i].add( IloNumVar(env, 0, 1010, IloNumVar::Float, (char*)varName.c_str()) );
		}
	}

	for (int i = 0; i < size+1; i++)
	{
		for (int j = 0; j < size+1; j++)
		{
			obj.setLinearCoef(var_x[i][j], 10*c[i][j]);
			obj.setLinearCoef(var_f[i][j], c[i][j]);
		}
	}
	model.add(obj);

	//numeracao segundo Fukasawa 2015
	// (1a) - tudo que sai de todo i
	for (int i = 0; i < size+1; i++)
	{
		consName = "c1a_" + to_string(getGiftId(i));
		IloRange c1a(env, 1, 1, (char*)consName.c_str());

		for (int j = 0; j < size+1; j++)
		{
			if (c[i][j] > 0)
			{
				c1a.setLinearCoef(var_x[i][j], 1);
			}
		}
		model.add(c1a);
	}

	// (1b) - tudo que entra em todo i
	for (int i = 0; i < size+1; i++)
	{
		consName = "c1b_" + to_string(getGiftId(i));
		IloRange c1b( env, 1, 1, (char*)consName.c_str() );

		for (int j = 0; j < size+1; j++)
		{
			if (c[i][j] > 0)
			{
				c1b.setLinearCoef(var_x[j][i], 1);
			}
		}
		model.add(c1b);
	}
	
	//(1c) ensures that q_i units are delivered
	//tudo que sai de i deve ser menor w_i de tudo que entra
	for (int i = 0; i < size; i++)
	{
		consName = "c1c_" + to_string(getGiftId(i));
		IloRange c1c( env, getWeight(i), getWeight(i), (char*)consName.c_str() );

		for (int j = 0; j < size+1; j++)
		{
			if (c[j][i] > 0) //tudo que entra
			{
				c1c.setLinearCoef(var_f[j][i], 1);
			}
			if (c[i][j] > 0) //tudo que sai
			{
				c1c.setLinearCoef(var_f[i][j], -1);
			}
			
		}
		model.add(c1c);
	}

	consName = "c1c_0";
	IloRange c1c( env, getWeight()-inst.sleighWeight, getWeight()-inst.sleighWeight, (char*)consName.c_str() );

	for (int j = 0; j < size; j++)
	{
		if (c[size][j] > 0)
		{
			c1c.setLinearCoef(var_f[size][j], 1);
		}
		if (c[j][size] > 0)
		{
			c1c.setLinearCoef(var_f[j][size], -1);
		}
	}
	model.add(c1c);

	//(1d) f bounds
	for (int i = 0; i < size+1; i++)
	{
		for (int j = 0; j < size+1; j++)
		{		
			if (c[i][j] > 0)
			{
				model.add(IloConstraint( getWeight(j) * var_x[i][j] <= var_f[i][j] )); //o que entra em j deve pelo menos carregar o peso de j
				model.add(IloConstraint( var_f[i][j] <= (getWeight() - inst.sleighWeight - getWeight(i)) * var_x[i][j] )); //o que sai de i nao pode carregar o peso de i
			}			
		}
	}

	// (1e) - tudo que entra em 0	
	consName = "c1e_in";
	IloRange c1e_in( env, 1, 1, (char*)consName.c_str() );
	consName = "c1e_out";
	IloRange c1e_out( env, 1, 1, (char*)consName.c_str() );

	for (int i = 0; i < size; i++)
	{
		if (c[i][size] > 0)
		{
			c1e_in.setLinearCoef(var_x[i][size], 1);
		}
		if (c[size][i] > 0)
		{
			c1e_out.setLinearCoef(var_x[size][i], 1);
		}
	}
	model.add(c1e_in);
	model.add(c1e_out);
	
		
	IloCplex cplex(model);

	#ifdef _WIN32
		//cplex.setParam(IloCplex::Threads,1);
		//cplex.setParam(IloCplex::TiLim,10);
		//cplex.setParam(IloCplex::VarSel,3);
	#else
		cplex.setParam(IloCplex::TiLim,60);
	#endif	
		
	cplex.setParam(IloCplex::CutUp, getCost());

	try {
		cplex.exportModel("trip_kara2007.lp"); 
	}
	catch (IloException e) 
	{
		cerr << e;
	}		
		
	try {
		if ( cplex.solve() ) 
		{   
			cout << "Solution status = " << cplex.getStatus() << endl;
			double objValue = cplex.getObjValue();
			cout << "Best Solution Found:\t" << objValue << endl;

			vector<vector<int> > adjacencyMatrix (size+1);
			for (int i = 0; i < size+1; i++)
			{
				adjacencyMatrix[i].resize(size + 1);
			}
    
			for (int i = 0; i < size+1; i++)
			{
				for (int j = 0; j < size+1; j++)
				{	
					if (c[i][j] > 0)
					{				
						double val = cplex.getValue(var_x[i][j]);
						double valf = cplex.getValue(var_f[i][j]);
						if (val != 0)
						{
							cout << var_x[i][j].getName() << ": " << val << endl;
							cout << var_f[i][j].getName() << ": " << valf << endl;

							adjacencyMatrix[i][j] = 1;
							//adjacencyMatrix[j][i] = 1;
						}
					}				
				}	
			}

			vector<bool> visited(size+1);

			//starts at north pole
			visited[size] = true;
			int last = size;
	
			Trip Kara_Trip(inst);

			while (Kara_Trip.getSize() < size)
			{
				for (int j = 0; j < size+1; j++)
				{
					if ( ( adjacencyMatrix[last][j] == 1) && (!visited[j] ) )
					{
						visited[j] = true;	
						Kara_Trip.appendGift(inst, getGift(j));
						last = j;
						break;
					}
				}
			}

			cout << "OriginalTripCost:\t" << std::setprecision(17) << getCost() << endl;
			cout << "OptimalKaraTripCost:\t" << std::setprecision(17) << Kara_Trip.getCost() << endl;

			if ( abs(getCost() - Kara_Trip.getCost()) > 0.01 )
			{
				cout << "Kara Improvement" << endl;
			}

			if (Kara_Trip.getCost() < getCost()-0.01)
			{
				improveFlag = true;
				Gifts = Kara_Trip.Gifts;
				UpdateAuxiliaryDataStructures(inst);
				resetNeighborhoodStatuses();
				setNeighborhoodCleared(NEIGHBORHOOD_KARA);
			}
		}
	}
	catch (IloException e) 
	{
		cerr << e;
	}

	env.end();
}

double Trip::gapKara2007(Instance & inst)
{
	vector<vector<double> > c(size+1);

	for (int i = 0; i < size+1; i++)
	{
		c[i].resize(size+1);
	}

	for (int i = 0; i < size+1; i++)
	{
		for (int j = i+1; j < size+1; j++)
		{
			c[i][j] = inst.getDistance(getGiftId(i), getGiftId(j));			
			c[j][i] = c[i][j];
		} 
	}

	IloEnv env;
	IloModel model(env);		
	IloObjective obj = IloMinimize(env);
	NumVarMatrix var_x(env);
	NumVarMatrix var_f(env);
	string varName;
	string consName;

	for (int i = 0; i < size+1; i++)
	{
		var_x.add(IloNumVarArray(env));
		var_f.add(IloNumVarArray(env));
	}

	for (int i = 0; i < size+1; i++)
	{
		for (int j = 0; j < size+1; j++)
		{
			varName = "x" + to_string(getGiftId(i)) + "_" + to_string(getGiftId(j));
			var_x[i].add( IloNumVar(env, 0, 1, IloNumVar::Float, (char*)varName.c_str()) );

			varName = "f" + to_string(getGiftId(i)) + "_" + to_string(getGiftId(j));
			var_f[i].add( IloNumVar(env, 0, 1010, IloNumVar::Float, (char*)varName.c_str()) );
		}
	}

	for (int i = 0; i < size+1; i++)
	{
		for (int j = 0; j < size+1; j++)
		{
			obj.setLinearCoef(var_x[i][j], 10*c[i][j]);
			obj.setLinearCoef(var_f[i][j], c[i][j]);
		}
	}
	model.add(obj);

	//numeracao segundo Fukasawa 2015
	// (1a) - tudo que sai de todo i
	for (int i = 0; i < size+1; i++)
	{
		consName = "c1a_" + to_string(getGiftId(i));
		IloRange c1a(env, 1, 1, (char*)consName.c_str());

		for (int j = 0; j < size+1; j++)
		{
			if (c[i][j] > 0)
			{
				c1a.setLinearCoef(var_x[i][j], 1);
			}
		}
		model.add(c1a);
	}

	// (1b) - tudo que entra em todo i
	for (int i = 0; i < size+1; i++)
	{
		consName = "c1b_" + to_string(getGiftId(i));
		IloRange c1b( env, 1, 1, (char*)consName.c_str() );

		for (int j = 0; j < size+1; j++)
		{
			if (c[i][j] > 0)
			{
				c1b.setLinearCoef(var_x[j][i], 1);
			}
		}
		model.add(c1b);
	}
	
	//(1c) ensures that q_i units are delivered
	//tudo que sai de i deve ser menor w_i de tudo que entra
	for (int i = 0; i < size; i++)
	{
		consName = "c1c_" + to_string(getGiftId(i));
		IloRange c1c( env, getWeight(i), getWeight(i), (char*)consName.c_str() );

		for (int j = 0; j < size+1; j++)
		{
			if (c[j][i] > 0) //tudo que entra
			{
				c1c.setLinearCoef(var_f[j][i], 1);
			}
			if (c[i][j] > 0) //tudo que sai
			{
				c1c.setLinearCoef(var_f[i][j], -1);
			}
			
		}
		model.add(c1c);
	}

	consName = "c1c_0";
	IloRange c1c( env, getWeight()-inst.sleighWeight, getWeight()-inst.sleighWeight, (char*)consName.c_str() );

	for (int j = 0; j < size; j++)
	{
		if (c[size][j] > 0)
		{
			c1c.setLinearCoef(var_f[size][j], 1);
		}
		if (c[j][size] > 0)
		{
			c1c.setLinearCoef(var_f[j][size], -1);
		}
	}
	model.add(c1c);

	//(1d) f bounds
	for (int i = 0; i < size+1; i++)
	{
		for (int j = 0; j < size+1; j++)
		{		
			if (c[i][j] > 0)
			{
				model.add(IloConstraint( getWeight(j) * var_x[i][j] <= var_f[i][j] )); //o que entra em j deve pelo menos carregar o peso de j
				model.add(IloConstraint( var_f[i][j] <= (getWeight() - inst.sleighWeight - getWeight(i)) * var_x[i][j] )); //o que sai de i nao pode carregar o peso de i
			}			
		}
	}

	// (1e) - tudo que entra em 0	
	consName = "c1e_in";
	IloRange c1e_in( env, 1, 1, (char*)consName.c_str() );
	consName = "c1e_out";
	IloRange c1e_out( env, 1, 1, (char*)consName.c_str() );

	for (int i = 0; i < size; i++)
	{
		if (c[i][size] > 0)
		{
			c1e_in.setLinearCoef(var_x[i][size], 1);
		}
		if (c[size][i] > 0)
		{
			c1e_out.setLinearCoef(var_x[size][i], 1);
		}
	}
	model.add(c1e_in);
	model.add(c1e_out);
	
	IloCplex cplex(model);

	cplex.setOut(env.getNullStream());

	#ifdef _WIN32
		cplex.setParam(IloCplex::Threads,1);
		//cplex.setParam(IloCplex::TiLim,10);
		//cplex.setParam(IloCplex::VarSel,3);
	#else
		//cplex.setParam(IloCplex::TiLim,60);
		cplex.setParam(IloCplex::VarSel,3);
	#endif	
		
	//cplex.setParam(IloCplex::CutUp, getCost());

	try {
		cplex.exportModel("trip_kara2007.lp"); 
	}
	catch (IloException e) 
	{
		cerr << e;
	}		
		
	try {
		if ( cplex.solve() ) 
		{   
			double objValue = cplex.getObjValue();			
			return (getCost()-objValue)/objValue;
		}
	}
	catch (IloException e) 
	{
		cerr << e;
		return -1;
	}

	env.end();
}
#endif

double evaluateBiCrossExchange(Instance & inst, Trip & trip1, Trip & trip2, int i, int k, int m, int o, int j, int l, int n, int p, int& sense1, int& sense2)
{	
	double weightChain1_1 = trip1.getWeightAt(i) - trip1.getWeightAt(k);
	double weightChain1_2 = trip1.getWeightAt(m) - trip1.getWeightAt(o);
	double weightChain2_1 = trip2.getWeightAt(j) - trip2.getWeightAt(l);
	double weightChain2_2 = trip2.getWeightAt(n) - trip2.getWeightAt(p);

	//if simply exchange north pole
	if (
		( ( (i == -1) && (k == trip1.getSize() - 1) ) || ( (m == -1) && (o == trip1.getSize() - 1) ) ) &&
		( ( (j == -1) && (l == trip2.getSize() - 1) ) || ( (n == -1) && (p == trip2.getSize() - 1) ) ) 
		)
	{
		return 0;
	}

	//check if weight exchange is feasible
	if ( (trip1.getWeight() - weightChain1_1 - weightChain1_2 + weightChain2_1 + weightChain2_2 > inst.weightLimit) ||
		(trip2.getWeight() - weightChain2_1 - weightChain2_2 + weightChain1_1 + weightChain1_2 > inst.weightLimit) )
	{
		return 1e15;
	}

	//cost of getting to gift i + cost of carrying the weight difference until gift i
	double cost1 = trip1.getCostUntil(i) + (weightChain2_1 + weightChain2_2 - weightChain1_1 - weightChain1_2) * trip1.getDistanceUntil(i);
	double cost1_1 = 0;
	double cost1_2 = 0;
	double reverseCost1_1 = 0;
	double reverseCost1_2 = 0;

	//first chain	
	//weight before first crossing
	double weight1 = trip1.getWeightAt(i) - weightChain1_1 - weightChain1_2 + weightChain2_1 + weightChain2_2;
	double reverseWeight1 = weight1;	
	if (j < l)
	{
		cost1_1 += inst.getDistance(trip1.getGiftId(i), trip2.getGiftId(j+1))*weight1;
		for (int g = j+1; g < l; g++)
		{
			weight1 -= trip2.getWeight(g);
			cost1_1 += trip2.getDistanceFrom(g)*weight1;
		}		
		weight1 -= trip2.getWeight(l);
		cost1_1 += inst.getDistance(trip2.getGiftId(l), trip1.getGiftId(k+1))*weight1;

		reverseCost1_1 += inst.getDistance(trip1.getGiftId(i), trip2.getGiftId(l))*reverseWeight1;
		for (int g = l; g > j+1; g--)
		{
			reverseWeight1 -= trip2.getWeight(g);
			reverseCost1_1 += trip2.getDistanceFrom(g-1)*reverseWeight1;
		}		
		reverseWeight1 -= trip2.getWeight(j+1);
		reverseCost1_1 += inst.getDistance(trip2.getGiftId(j+1), trip1.getGiftId(k+1))*reverseWeight1;
	}
	else
	{
		cost1 += inst.getDistance(trip1.getGiftId(i), trip1.getGiftId(k+1))*weight1;
	}
	if (k+1 < m)
	{
		for (int g = k+1; g < m; g++)
		{
			cost1 += trip1.getCostFrom(g) + trip1.getDistanceFrom(g)*(weightChain2_2 - weightChain1_2);
		}
	}
	//second chain
	//weight before second crossing
	weight1 = trip1.getWeightAt(m) - weightChain1_2 + weightChain2_2;
	reverseWeight1 = weight1;
	if (n < p)
	{
		cost1_2 += inst.getDistance(trip1.getGiftId(m), trip2.getGiftId(n+1))*weight1;
		for (int g = n+1; g < p; g++)
		{
			weight1 -= trip2.getWeight(g);
			cost1_2 += trip2.getDistanceFrom(g)*weight1;
		}		
		weight1 -= trip2.getWeight(p);
		cost1_2 += inst.getDistance(trip2.getGiftId(p), trip1.getGiftId(o+1))*weight1;

		reverseCost1_2 += inst.getDistance(trip1.getGiftId(m), trip2.getGiftId(p))*reverseWeight1;
		for (int g = p; g > n+1; g--)
		{
			reverseWeight1 -= trip2.getWeight(g);
			reverseCost1_2 += trip2.getDistanceFrom(g-1)*reverseWeight1;
		}		
		reverseWeight1 -= trip2.getWeight(n+1);
		reverseCost1_2 += inst.getDistance(trip2.getGiftId(n+1), trip1.getGiftId(o+1))*reverseWeight1;
	}
	else
	{
		if (k < o)
		{
			cost1 += inst.getDistance(trip1.getGiftId(m), trip1.getGiftId(o+1))*weight1;
		}
	}
	cost1 += trip1.getCostAhead(o+1);
		
	//cost of getting to gift j + cost of carrying the weight difference until gift j
	double cost2 = trip2.getCostUntil(j) + (weightChain1_1 + weightChain1_2 - weightChain2_1 - weightChain2_2) * trip2.getDistanceUntil(j);
	double cost2_1 = 0;
	double cost2_2 = 0;
	double reverseCost2_1 = 0;
	double reverseCost2_2 = 0;

	//first chain
	//weight before first crossing
	double weight2 = trip2.getWeightAt(j) - weightChain2_1 - weightChain2_2 + weightChain1_1 + weightChain1_2;
	double reverseWeight2 = weight2;
	if (i < k)
	{
		cost2_1 += inst.getDistance(trip2.getGiftId(j), trip1.getGiftId(i+1))*weight2;
		for (int g = i+1; g < k; g++)
		{
			weight2 -= trip1.getWeight(g);
			cost2_1 += trip1.getDistanceFrom(g)*weight2;
		}
		weight2 -= trip1.getWeight(k);
		cost2_1 += inst.getDistance(trip1.getGiftId(k), trip2.getGiftId(l+1))*weight2;

		reverseCost2_1 += inst.getDistance(trip2.getGiftId(j), trip1.getGiftId(k))*reverseWeight2;
		for (int g = k; g > i+1; g--)
		{
			reverseWeight2 -= trip1.getWeight(g);
			reverseCost2_1 += trip1.getDistanceFrom(g-1)*reverseWeight2;
		}		
		reverseWeight2 -= trip1.getWeight(i+1);
		reverseCost2_1 += inst.getDistance(trip1.getGiftId(i+1), trip2.getGiftId(l+1))*reverseWeight2;
	}
	else
	{
		cost2 += inst.getDistance(trip2.getGiftId(j), trip2.getGiftId(l+1))*weight2;
	}
	if (l+1 < n)
	{
		for (int g = l+1; g < n; g++)
		{
			cost2 += trip2.getCostFrom(g) + trip2.getDistanceFrom(g)*(weightChain1_2 - weightChain1_2);
		}
	}
	//second chain
	//weight before second crossing
	weight2 = trip2.getWeightAt(n) - weightChain2_2 + weightChain1_2;
	reverseWeight2 = weight2;
	if (m < o)
	{
		cost2_1 += inst.getDistance(trip2.getGiftId(n), trip1.getGiftId(m+1))*weight2;
		for (int g = m+1; g < o; g++)
		{
			weight2 -= trip1.getWeight(g);
			cost2_1 += trip1.getDistanceFrom(g)*weight2;
		}
		weight2 -= trip1.getWeight(o);
		cost2_1 += inst.getDistance(trip1.getGiftId(o), trip2.getGiftId(p+1))*weight2;

		reverseCost2_1 += inst.getDistance(trip2.getGiftId(n), trip1.getGiftId(o))*reverseWeight2;
		for (int g = o; g > m+1; g--)
		{
			reverseWeight2 -= trip1.getWeight(g);
			reverseCost2_1 += trip1.getDistanceFrom(g-1)*reverseWeight2;
		}		
		reverseWeight2 -= trip1.getWeight(o+1);
		reverseCost2_1 += inst.getDistance(trip1.getGiftId(m+1), trip2.getGiftId(p+1))*reverseWeight2;
	}
	else
	{		
		if (l < p)
		{
			cost2 += inst.getDistance(trip2.getGiftId(n), trip2.getGiftId(p+1))*weight2;
		}
	}
	cost2 += trip2.getCostAhead(p+1);	

	double bestIncrease = cost1 + cost2 - trip1.getCost() - trip2.getCost();

	//if trip1 receives reversed chain2_1
	if (reverseCost1_1 < cost1_1 - RELEVANT_INCREASE)
	{
		//if trip2 receives reversed chain1
		if (reverseCost2_1 < cost2_1 - RELEVANT_INCREASE)
		{
			sense1 = CX_SENSE_REVERSE_REVERSE;
			bestIncrease += ( reverseCost1_1 + reverseCost2_1 ) ;
		}
		else
		{
			sense1 = CX_SENSE_ORIGINAL_REVERSE;
			bestIncrease += ( reverseCost1_1 + cost2_1 );
		}
	}
	else
	{
		//if trip2 receives reversed chain1_1
		if (reverseCost2_1 < cost2_1 - RELEVANT_INCREASE)
		{
			sense1 = CX_SENSE_REVERSE_ORIGINAL;
			bestIncrease += ( cost1_1 + reverseCost2_1 );
		}
		else
		{
			sense1 = CX_SENSE_ORIGINAL_ORIGINAL;
			bestIncrease += ( cost1_1 + cost2_1 );
		}
	}

	//if trip1 receives reversed chain2_2
	if (reverseCost1_2 < cost1_2 - RELEVANT_INCREASE)
	{
		//if trip2 receives reversed chain1
		if (reverseCost2_2 < cost2_2 - RELEVANT_INCREASE)
		{
			sense2 = CX_SENSE_REVERSE_REVERSE;
			bestIncrease += ( reverseCost1_2 + reverseCost2_2 ) ;
		}
		else
		{
			sense2 = CX_SENSE_ORIGINAL_REVERSE;
			bestIncrease += ( reverseCost1_2 + cost2_2 );
		}
	}
	else
	{
		//if trip2 receives reversed chain1
		if (reverseCost2_2 < cost2_2 - RELEVANT_INCREASE)
		{
			sense2 = CX_SENSE_REVERSE_ORIGINAL;
			bestIncrease += ( cost1_2 + reverseCost2_2 );
		}
		else
		{
			sense2 = CX_SENSE_ORIGINAL_ORIGINAL;
			bestIncrease += ( cost1_2 + cost2_2 );
		}
	}

	return bestIncrease;
}

void moveBiCrossExchange(Instance & inst, Trip & trip1, Trip & trip2, int i, int k, int m, int o, int j, int l, int n, int p, int sense1, int sense2)
{
	vector<Gift> bag1_1;
	vector<Gift> bag1_2;
	vector<Gift> bag2_1;
	vector<Gift> bag2_2;
	
	//debugging
	/*cout << "trip1 size: " << trip1.getSize() << endl;
	cout << "trip2 size: " << trip2.getSize() << endl;		
	cout << "trip1: " << trip1.printTrip() << endl;
	cout << "trip2: " << trip2.printTrip() << endl;	
	cout << "i: " << i << endl;
	cout << "j: " << j << endl;
	cout << "k: " << k << endl;
	cout << "l: " << l << endl;*/

	for (int g = m+1; g <= o; g++)
	{
		bag1_2.push_back(trip1.getGift(m+1));
		trip1.eraseGift(inst,m+1, false);
	}
	for (int g = n+1; g <= p; g++)
	{
		bag2_2.push_back(trip2.getGift(n+1));
		trip2.eraseGift(inst,n+1, false);
	}

	for (int g = i+1; g <= k; g++)
	{
		bag1_1.push_back(trip1.getGift(i+1));
		trip1.eraseGift(inst,i+1, false);
	}
	for (int g = j+1; g <= l; g++)
	{
		bag2_1.push_back(trip2.getGift(j+1));
		trip2.eraseGift(inst,j+1, false);
	}

	if ( (sense1 == CX_SENSE_ORIGINAL_ORIGINAL) || (sense1 == CX_SENSE_REVERSE_ORIGINAL) )
	{
		for (int g = bag2_1.size()-1; g > -1; g--)
		{
			trip1.insertGift(inst,i+1,bag2_1[g], false);
		}
	}
	else
	{
		for (int g = 0; g < (int)bag2_1.size(); g++)
		{
			trip1.insertGift(inst,i+1,bag2_1[g], false);
		}
	}
	
	if ( (sense1 == CX_SENSE_ORIGINAL_ORIGINAL) || (sense1 == CX_SENSE_ORIGINAL_REVERSE) )
	{
		for (int g = bag1_1.size()-1; g > -1; g--)
		{
			trip2.insertGift(inst,j+1,bag1_1[g], false);
		}
	}
	else
	{
		for (int g = 0; g < (int)bag1_1.size(); g++)
		{
			trip2.insertGift(inst,j+1,bag1_1[g], false);
		}
	}

	//second CrossExchange
	int pos1 = m-bag1_1.size()+bag2_1.size()+1;
	int pos2 = n-bag2_1.size()+bag1_1.size()+1;
	if ( (sense2 == CX_SENSE_ORIGINAL_ORIGINAL) || (sense2 == CX_SENSE_REVERSE_ORIGINAL) )
	{
		for (int g = bag2_2.size()-1; g > -1; g--)
		{
			trip1.insertGift(inst,pos1,bag2_2[g], false);
		}
	}
	else
	{
		for (int g = 0; g < (int)bag2_2.size(); g++)
		{
			trip1.insertGift(inst,pos1,bag2_2[g], false);
		}
	}
	
	if ( (sense2 == CX_SENSE_ORIGINAL_ORIGINAL) || (sense2 == CX_SENSE_ORIGINAL_REVERSE) )
	{
		for (int g = bag1_2.size()-1; g > -1; g--)
		{
			trip2.insertGift(inst,pos2,bag1_2[g], false);
		}
	}
	else
	{
		for (int g = 0; g < (int)bag1_2.size(); g++)
		{
			trip2.insertGift(inst,pos2,bag1_2[g], false);
		}
	}

	trip1.UpdateAuxiliaryDataStructures(inst);
	trip2.UpdateAuxiliaryDataStructures(inst);

	//cout << "trip1: " << trip1.printTrip() << endl;
	//cout << "trip2: " << trip2.printTrip() << endl;
}


//return how bad the gift is positioned
//measured as % of distance greater than NP distance 
double Trip::getGiftBadness(int index)
{
	double npDistance = getGift(index).npDistance;
	return (getDistanceUntil(index) - npDistance) / npDistance;
}

string Trip::printTripQuality()
{
	stringstream msg;

	for (int i = 0; i < size; i++)
	{
		msg << "Gift " << i << ": " << getGiftId(i) << "\tBadness: " << setprecision(2) <<  getGiftBadness(i)*100 << "%" << endl;
	}
	return msg.str();
}

void Trip::setTabu()
{
	tabu = true;
}
void Trip::clearTabu()
{
	tabu = false;
}

bool Trip::isTabu()
{
	return tabu;
}

bool Trip::isPolar()
{		
	for (int i = 0; i < (int)Gifts.size(); i++)
	{
		if (Gifts[i].isPolar())
		{
			return true;
		}
	}
	return false;
}

void Trip::setMark(int position)
{
	longitudeOrder = position;
}

int Trip::getMark()
{
	return longitudeOrder;
}


