#include "trip.h"
#include "solution.h"
#include "instance.h"

#include <iomanip>
#include <sstream> 
#include <fstream>
#include <iostream>
#include <vector>
#include <algorithm>    // std::sort

#ifdef WIN32
#include "dirent.h"
#include "direct.h"
#endif

using namespace std;

bool distanciaNP(const Gift& gift1, const Gift& gift2);
double CalculateExchangeCost(Instance& inst, Solution& sol, GiftChain& chain_i, GiftChain& chain_j);
bool GiftChainCompatible(GiftChain chain_i, GiftChain chain_j);

PairIdDistance::PairIdDistance(int id_, double distance_)
{
	id = id_;
	distance = distance_;
}

CandidatePerturbation::CandidatePerturbation(int Perturbation_Code_, int arg1, int arg2, int arg3, int arg4, int arg5, int arg6, double priority_)
{
	Perturbation_Code = Perturbation_Code_;
	priority = priority_;
	if (Perturbation_Code == PERTURBATION_2MERGE)
	{
		index1 = arg1;
		index2 = arg2;
	}
	if (Perturbation_Code == PERTURBATION_3MERGE)
	{
		index1 = arg1;
		index2 = arg2;
		index3 = arg3;
	}
	if (Perturbation_Code == PERTURBATION_RANDOM_CROSS_EXCHANGE)
	{
		index1 = arg1;
		index2 = arg2;
		i = arg3;
		j = arg4;
		k = arg5;
		l = arg6;
	}
	if (Perturbation_Code == PERTURBATION_EXPLODE)
	{
		index1 = arg1;
	}	
	if (Perturbation_Code == PERTURBATION_HOLD_SOUTH_POLE)
	{
		index1 = arg1;
	}	
}

Solution::Solution(Instance & inst)
{

}

Solution::Solution()
{

}

Solution::Solution (const Solution &copySolution)
{
	Trips.resize(copySolution.Trips.size(), *ptr_Instance);
	for (int i = 0; i < copySolution.Trips.size(); i++)
	{
		Trips[i] = copySolution.Trips[i];
	}
	ptr_Instance = copySolution.ptr_Instance;
}


Solution::Solution(Instance & inst, int criteria, double criteriaArg1, double criteriaArg2, string strArg)
{
	//cout << "Generating initial solution..." << endl;

	if ( criteria == CRITERIA_FAR_SEED_GREEDY )
	{
		SolutionBySeedGreedy(inst,criteriaArg1);			
		return;
	}
	if ( criteria == CRITERIA_FAR_SEED_GREEDY_WD )
	{
		SolutionBySeedGreedyWL(inst,criteriaArg1);			
		return;
	}

	if (criteria == CRITERIA_SolutionKytojoki2007)
	{
		SolutionKytojoki2007(inst,(int)criteriaArg1,criteriaArg2);			
		return;
	}

	if (criteria == CRITERIA_FAR_SEED_GREEDY_COST_PERCENT)
	{
		SolutionBySeedGreedyCostPercent(inst,criteriaArg1);			
		return;
	}

	if (criteria == CRITERIA_FAR_SEED_GREEDY_LIMIT_TRIPS)
	{
		SolutionBySeedGreedyTripsLimit(inst, (int)criteriaArg1);			
		return;
	}
	if ( criteria == CRITERIA_FAR_SEED_GREEDY_EXTENSIVE )
	{
		SolutionBySeedGreedyExtensive(inst,criteriaArg1);			
		return;
	}
	if ( criteria == CRITERIA_OLD_SOLUTION )
	{
		ReadOldSolution(inst,strArg);			
		return;
	}

	vector<int> tripList;
	vector<int> giftList;

	for (int i = 0; i  < (int)inst.n; i++)
	{
		giftList.push_back(i);
	}
	//std::random_shuffle(giftList.begin(), giftList.end()); 

	Trips.push_back(Trip(inst));
	tripList.push_back(0);

	for (int i = 0; i < inst.n; i++)
	{
		//std::random_shuffle(tripList.begin(), tripList.end()); 
		//cout << i << endl;
		bool hasTrip = false;
		for (int t = 0; t < (int)Trips.size(); t++)
		{
			if (Trips[tripList[t]].FullyCompatible(inst.Gifts[giftList[i]], inst, criteria, criteriaArg1))
			{
				hasTrip = true;
				Trips[tripList[t]].appendGift(inst, inst.Gifts[giftList[i]]);
				break;
			}
		}
		if (!hasTrip)
		{
			Trips.push_back(Trip(inst, inst.Gifts[giftList[i]]));
			tripList.push_back(Trips.size()-1);
		}
	}

	for (int i = 0; i < (int)Trips.size(); i++)
	{
		Trips[i].SortDistanciaNP();
		Trips[i].UpdateAuxiliaryDataStructures(inst);
	}
	
	//for (int i = 0; i < 10; i++)
	//{
	//	Trips[i].optTSP(inst);
	//	cout << endl << Trips[i].printTrip() << endl << Trips[i].getCost() << endl;
	//}

}

void Solution::ReadOldSolution(Instance & inst, string solutionFileName)
{
	std::ifstream solutionFile;
	std::string aux;
	int n = 0;

	solutionFile.open(solutionFileName.c_str());	
		
	solutionFile >> aux;
	int lastTripId = 0;
	while (solutionFile >> aux) {
		//search for 3 separators
		int s1 = aux.find(",");

		int id = stoi(aux.substr(0,s1));
		int tripId = stoi(aux.substr(s1+1));
		
		if (lastTripId == tripId)
		{
			Trips.back().appendGift(inst,id);
		}
		else
		{
			lastTripId = tripId;
			Trips.push_back(Trip(inst,id));
		}

		n++;	

		if (n == LIMITGIFTS)
		{
			break;
		}
	}
}

bool AscendingLatitude(const Gift& gift1, const Gift& gift2) {
   return gift1.latitude < gift2.latitude;
}

void Solution::SolutionBySeedGreedy(Instance & inst, double maxLengthIncrease)
{
	sort(inst.Gifts.begin(), inst.Gifts.end(), AscendingLatitude);
	
	//Trips.push_back(Trip(inst));

	for (int i = 0; i < inst.n; i++)
	{
		bool hasTrip = false;
		int bestTrip = 0;
		double bestIncrease = 1e15;
		
		for (int t = 0; t < (int)Trips.size(); t++)
		{
			if (Trips[t].getWeight() + inst.Gifts[i].weight <= inst.weightLimit)
			{
				double increase = Trips[t].evaluateInsertionLength(inst, 0, inst.Gifts[i]); //a
				/*double initialLength = Trips[t].getLength();
				Trips[t].insertGift(inst, 0, inst.Gifts[i]);

				double newLength  = Trips[t].getLength(); 

				double increase = newLength - initialLength;

				if (abs(a - increase) > 0.01)
				{
					cerr << "error in evaluateevaluateInsertionLengthSetGift";
					throw -1;
				}*/

				if (increase < maxLengthIncrease)
				{
					if (increase < bestIncrease)
					{
						bestIncrease = increase;
						bestTrip = t;
						hasTrip = true;
					}
				}
				//Trips[t].eraseGift(inst, 0);
			}
		}
		if (hasTrip)
		{
			/*double deltaLatExp = abs(Trips[bestTrip].getFrontGift().latitude-inst.Gifts[i].latitude);
			double deltaLongExp = abs(Trips[bestTrip].getFrontGift().longitude-inst.Gifts[i].longitude);
			cout << "dd\t" << deltaLatExp*deltaLongExp << endl;
			if (deltaLatExp*deltaLongExp > 50)
			{
				cout << bestIncrease << endl;
			}*/
			Trips[bestTrip].insertGift(inst, 0, inst.Gifts[i]);
		}
		else
		{
			Trips.push_back(Trip(inst, inst.Gifts[i]));
		}
	}

}


void Solution::SolutionBySeedGreedyWL(Instance & inst, double maxWeightLengthIncrease)
{
	sort(inst.Gifts.begin(), inst.Gifts.end(), AscendingLatitude);
	
	Trips.push_back(Trip(inst));

	for (int i = 0; i < inst.n; i++)
	{
		bool hasTrip = false;
		int bestTrip = 0;
		double bestIncrease = 1e15;

		for (int t = 0; t < (int)Trips.size(); t++)
		{
			if (Trips[t].getWeight() + inst.Gifts[i].weight <= inst.weightLimit)
			{
				
				double increase = Trips[t].evaluateInsertionLength(inst, 0, inst.Gifts[i])*inst.Gifts[i].weight; //a

				/*double initialLength = Trips[t].getLength();
				Trips[t].insertGift(inst, 0, inst.Gifts[i]);

				double newLength  = Trips[t].getLength(); 

				double increase = (newLength - initialLength)*inst.Gifts[i].weight;*/

				if (increase < maxWeightLengthIncrease)
				{
					if (increase < bestIncrease)
					{
						bestIncrease = increase;
						bestTrip = t;
						hasTrip = true;
					}
				}
				//Trips[t].eraseGift(inst, 0);
			}
		}
		if (hasTrip)
		{
			Trips[bestTrip].insertGift(inst, 0, inst.Gifts[i]);
		}
		else
		{
			Trips.push_back(Trip(inst, inst.Gifts[i]));
		}
	}

}

//#define CRITERIA_SolutionKytojoki2007 5
void Solution::SolutionKytojoki2007(Instance & inst, int kLSFrequency, double deltaEmptyness)
{
	sort(inst.Gifts.begin(), inst.Gifts.end(), AscendingLatitude);
	
	//number of gifts to insert before performing local search
	vector<bool> routed(LIMITGIFTS);
	int routedGifts = 0;

	while (routedGifts < LIMITGIFTS)
	{
		//choose farthest gift as the seed
		int i = 0;
		while (routed[i])
		{
			i++;
		}

		cout << "Incumbent Trip " << Trips.size() + 1 << endl;
		//cout << "Seed Gift " << inst.Gifts[i].id << endl;

		//create new trip from seed gift
		Trip incumbentTrip(inst, inst.Gifts[i]);

		routed[i] = true;
		routedGifts++;
		vector<PairIdDistance> candidates;
		candidates.reserve((int)inst.Gifts.size());

		for (int i = 0; i < (int)inst.Gifts.size(); i++)
		{
			if (!routed[i])
			{
				double increase = incumbentTrip.evaluateInsertionLength(inst, 0, inst.Gifts[i]); 
				candidates.push_back( PairIdDistance(i, increase) );
			}
		}
		if (candidates.size() > 1000)
		{
			sort(candidates.begin(),candidates.end());
			candidates.erase(candidates.begin()+1000,candidates.end());
		}

		bool hasGift = true;
		while (hasGift)
		{
			hasGift = false;
			double bestIncrease = 1e10;
			
			int best_i = 0;
			int best_j = 0;
			for (int i = 0; i < (int)candidates.size(); i++)
			{
				if ( (!routed[candidates[i].id]) && ( incumbentTrip.getWeight() + inst.Gifts[candidates[i].id].weight <= inst.weightLimit*(1-deltaEmptyness) ) )
				{
					for (int j = 0; j <= incumbentTrip.getSize(); j++)
					{
						//double increase = incumbentTrip.evaluateInsertionLength(inst, j, inst.Gifts[candidates[i].id]); //a
						double increase = incumbentTrip.evaluateInsertion(inst, j, inst.Gifts[candidates[i].id]); //a
						if (increase < bestIncrease)
						{
							hasGift = true;
							bestIncrease = increase;
							best_i = candidates[i].id;
							best_j = j;
						}
					}
				}				
			}
			if (hasGift)
			{
				//cout << "Inserting Gift " << inst.Gifts[best_i].id << "\t" << bestIncrease << endl;
				incumbentTrip.insertGift(inst, best_j, inst.Gifts[best_i]);
				routed[best_i] = true;
				routedGifts++;
				if ( ( kLSFrequency != 0 ) && ( routedGifts % kLSFrequency == 0 ) )
				{					
					double dummyAccount = 0;
					bool intraImproveFlag = true;
					while (intraImproveFlag)
					{
						intraImproveFlag = false;
					
						incumbentTrip.search2Opt(inst, intraImproveFlag, dummyAccount, false);	
						incumbentTrip.search3Opt(inst, intraImproveFlag, dummyAccount, false);						
						incumbentTrip.searchIntraRelocation(inst, intraImproveFlag, dummyAccount, false);					
						incumbentTrip.searchIntraChainRelocation(inst, intraImproveFlag, dummyAccount, false);					
						incumbentTrip.searchIntraExchange(inst, intraImproveFlag, dummyAccount, false);						
					}
				}
				//cout << routedGifts << endl;
			}
		}
		Trips.push_back(incumbentTrip);
	}
}

void Solution::SolutionBySeedGreedyCostPercent(Instance & inst, double maxPercentIncrease)
{
	sort(inst.Gifts.begin(), inst.Gifts.end(), AscendingLatitude);
	
	Trips.push_back(Trip(inst));

	for (int i = 0; i < inst.n; i++)
	{
		bool hasTrip = false;
		int bestTrip = 0;
		double bestIncrease = 1e15;

		for (int t = 0; t < (int)Trips.size(); t++)
		{
			if (Trips[t].getWeight() + inst.Gifts[i].weight <= inst.weightLimit)
			{				
				double increase = Trips[t].evaluateInsertion(inst, 0, inst.Gifts[i]); //a
				double percentualIncrease = increase/Trips[t].getLength();

				if (percentualIncrease < maxPercentIncrease)
				{
					if (increase < bestIncrease)
					{
						bestIncrease = increase;
						bestTrip = t;
						hasTrip = true;
					}
				}
			}
		}
		if (hasTrip)
		{
			Trips[bestTrip].insertGift(inst, 0, inst.Gifts[i]);
		}
		else
		{
			Trips.push_back(Trip(inst, inst.Gifts[i]));
		}
	}
}

void Solution::SolutionBySeedGreedyTripsLimit(Instance & inst, int tripsLimit)
{
	sort(inst.Gifts.begin(), inst.Gifts.end(), AscendingLatitude);
	
	for (int i = 0; i < tripsLimit; i++)
	{
		Trips.push_back(Trip(inst, inst.Gifts[i]));
	}

	for (int i = tripsLimit; i < inst.n; i++)
	{
		bool hasTrip = false;
		int bestTrip = 0;
		double bestIncrease = 1e15;

		for (int t = 0; t < (int)Trips.size(); t++)
		{
			if (Trips[t].getWeight() + inst.Gifts[i].weight <= inst.weightLimit)
			{
				double increase = Trips[t].evaluateInsertionLength(inst, 0, inst.Gifts[i]); //a

				if (increase < bestIncrease)
				{
					bestIncrease = increase;
					bestTrip = t;
					hasTrip = true;
				}
			}
		}
		Trips[bestTrip].insertGift(inst, 0, inst.Gifts[i]);
	}
}


void Solution::SolutionBySeedGreedyExtensive(Instance & inst, double maxLengthIncrease)
{
	sort(inst.Gifts.begin(), inst.Gifts.end(), AscendingLatitude);
	
	for (int i = 0; i < inst.n; i++)
	{
		bool hasTrip = false;
		int bestTrip = 0;
		int best_j = 0;
		double bestIncrease = 1e15;

		for (int t = 0; t < (int)Trips.size(); t++)
		{
			if (Trips[t].getWeight() + inst.Gifts[i].weight <= inst.weightLimit)
			{
				for (int j = 0; j <= Trips[t].getSize(); j++)
				{
					double increase = Trips[t].evaluateInsertionLength(inst, j, inst.Gifts[i]);	

					if (increase < maxLengthIncrease)
					{
						if (increase < bestIncrease)
						{
							bestIncrease = increase;
							bestTrip = t;
							best_j = j;
							hasTrip = true;
						}
					}
				}
				
			}
		}
		if (hasTrip)
		{
			Trips[bestTrip].insertGift(inst, best_j, inst.Gifts[i]);
		}
		else
		{
			Trips.push_back(Trip(inst, inst.Gifts[i]));
		}
	}
}

bool avgLongitude(const Trip& trip1, const Trip& trip2) {
   return trip1.avgLongitude() < trip2.avgLongitude();
}

bool avgMCLongitude(const Trip& trip1, const Trip& trip2) {
   return trip1.avgMCLongitude() < trip2.avgMCLongitude();
}

bool ascendingCost(const Trip& trip1, const Trip& trip2) {
   return trip1.getCost() < trip2.getCost();
}

bool descendingCost(const Trip& trip1, const Trip& trip2) {
   return trip1.getCost() > trip2.getCost();
}

void Solution::SortLongitude()
{
	sort(Trips.begin(), Trips.end(), avgLongitude);
}

class cmpAvgMCLongitude {
    int param;
public:
    cmpAvgMCLongitude(int p) : param(p) {}

    bool operator()(const Trip& trip1, const Trip& trip2) {
		return (trip1.avgMCLongitude() + param) < (trip2.avgMCLongitude()+param);
    }
};

void Solution::MarkPositions()
{
	for (int i = 0; i < (int)Trips.size(); i++)
	{
		Trips[i].setMark(i);
	}
}

double Solution::getTripPairOffset(int index1, int index2)
{
	int halfTrips = Trips.size()/2;

	int offset = abs(Trips[index1].getMark() - Trips[index2].getMark());

	if (offset > halfTrips)
	{
		offset -= halfTrips;
	}

	if (index2 > index1)
	{
		return offset;
	}
	else
	{
		return -offset;
	}
}


void Solution::SortMCLongitude()
{
	sort(Trips.begin(), Trips.end(), avgMCLongitude);
}

void Solution::RandomSort()
{
	random_shuffle(Trips.begin(), Trips.end());
}

void Solution::SortCost()
{
	sort(Trips.begin(), Trips.end(), ascendingCost);

}
void Solution::SortDescendingCost()
{
	sort(Trips.begin(), Trips.end(), descendingCost);

}

std::string Solution::toCSV()
{
	std::stringstream aux;

	aux << "GiftId,TripId" << std::endl;

	for (int i = 0; i < (int)Trips.size(); i++)
	{
		for (int j = 0; j < (int)Trips[i].getSize(); j++)
		{				
			aux << Trips[i].getGiftId(j) << "," << i+1 << std::endl;
		}
	}

	return aux.str();
}

std::string Solution::toPolarCSV(bool onlyPolars)
{
	std::stringstream aux;

	aux << "GiftId,TripId" << std::endl;
	int tripId = 1;

	if (onlyPolars)
	{
		for (int i = 0; i < (int)Trips.size(); i++)
		{
			if (Trips[i].isPolar())
			{
				for (int j = 0; j < (int)Trips[i].getSize(); j++)
				{				
					aux << Trips[i].getGiftId(j) << "," << tripId << std::endl;
				}
				tripId++;
			}			
		}
	}
	else
	{
		for (int i = 0; i < (int)Trips.size(); i++)
		{
			if (!Trips[i].isPolar())
			{
				for (int j = 0; j < (int)Trips[i].getSize(); j++)
				{				
					aux << Trips[i].getGiftId(j) << "," << tripId << std::endl;
				}
				tripId++;
			}			
		}
	}

	return aux.str();
}

void Solution::toCSVFile(std::string fileName)
{
	ofstream myfile;
	myfile.open (fileName);
	myfile << toCSV();
	myfile.close();
}

void Solution::toPolarCSVFile(std::string fileName, bool onlyPolar)
{
	ofstream myfile;
	myfile.open (fileName);
	myfile << toPolarCSV(onlyPolar);
	myfile.close();
}


int iterNum(string solutionsFolder);

#ifdef WIN32
string DataAgora()
{
	time_t now = time(0);
	tm ltm;
	localtime_s(&ltm,&now);

	stringstream min;
	stringstream hr;
	
	stringstream dia;
	stringstream mes;

	dia << setw(2) << setfill('0') << ltm.tm_mday;
	mes << setw(2) << setfill('0') << ltm.tm_mon + 1;
	hr << setw(2) << setfill('0') << ltm.tm_hour;
	min << setw(2) << setfill('0') << ltm.tm_min;
	string data = dia.str() + "/" + mes.str() + "/" + to_string(ltm.tm_year + 1900) + " " + hr.str() + ":" + min.str();

	return data;	
}

void Solution::toRoutesFolder(std::string solutionsFolder)
{
	string nomeIter = "sol_" + to_string(iterNum(solutionsFolder));

	string solutionsFolderName = solutionsFolder + nomeIter + "\\";
	_mkdir(solutionsFolderName.c_str());

	string routesFolderName = solutionsFolder + nomeIter + "\\Trips\\";
	_mkdir(routesFolderName.c_str());

	string pointsFolderName = solutionsFolder + nomeIter + "\\Points\\";;
	_mkdir(pointsFolderName.c_str());

	
	ofstream mySolFile;
	stringstream solutionDataFile;
	solutionDataFile << solutionsFolderName << "solutiondata.json";
	mySolFile.open (solutionDataFile.str());

	mySolFile << "{" << endl;
	mySolFile << "\"name\":\"" + nomeIter + "\",";
	mySolFile << "\"date\":\"" + DataAgora() + "\",";	
	mySolFile << "\"sol_cost\":" + to_string(getCost()) + ",";
	mySolFile << "\"num_trips\":" + to_string(Trips.size()) << endl;
	mySolFile << "}" << endl;

	mySolFile.close();

	for (int i = 0; i < (int)Trips.size(); i++)
	{
		ofstream myfile;
		stringstream routesFileName;
		routesFileName << routesFolderName << "trip" << i+1 << ".json";
		myfile.open (routesFileName.str());
		myfile << Trips[i].routeToGeoJson(i);
		myfile.close();

		stringstream tripDataFile;
		tripDataFile << routesFolderName << "trip" << i+1 << "_data.json";
		myfile.open (tripDataFile.str());
		myfile << "{" << endl;
		myfile << "\"name\":\"trip" + to_string(i+1) + "\",";
		myfile << "\"trip_cost\":" + to_string(Trips[i].getCost()) + ",";
		myfile << "\"trip_weight\":" + to_string(Trips[i].getWeight()) + ",";
		myfile << "\"trip_length\":" + to_string(Trips[i].getLength()) + ",";
		myfile << "\"trip_size\":" + to_string(Trips[i].getSize()) << endl;
		myfile << "}" << endl;
		myfile.close();

		stringstream fileNamePoints;
		fileNamePoints << pointsFolderName << "points_trip" << i+1 << ".json";
		myfile.open (fileNamePoints.str());
		myfile << Trips[i].pointsToGeoJson();
		myfile.close();
	}

}

int iterNum(string solutionsFolder)
{
	int iMaior = 0;
	DIR *dir;
	struct dirent *ent;
	if ((dir = opendir (solutionsFolder.c_str())) != NULL) {
		/* print all the files and directories within directory */
		ent = readdir(dir);
		while (ent != NULL) {
			if (ent->d_type == DT_DIR) {
				if (string(ent->d_name).size()>3) {
		/*			cout << string(ent->d_name).size() << endl;
					cout << string(ent->d_name).replace(0,9,"") << endl;*/
					int i = stoi(string(ent->d_name).replace(0,4,""));
					//cout << i;
					if (i>iMaior) {
						iMaior = i;
					}
				}		
			}
			ent = readdir(dir);
		}
		closedir (dir);
	}
	iMaior++;

	return iMaior;
}
#endif


void Solution::ClearEmptyTrips()
{
	sort(Trips.begin(),Trips.end());
	int i = 0;
	while ( (i < (int)Trips.size() ) && ( Trips[i].getSize() > 0 )  )
	{
		i++;
	}
	Trips.erase(Trips.begin() + i, Trips.end());
}


void Solution::ValidateWeights(Instance & inst)
{
	for (int i = 0; i < (int)Trips.size(); i++)
	{
		if ( ( abs(Trips[i].getRealWeight() - Trips[i].getWeight() ) > 0.01 ) || (Trips[i].getWeight() > inst.weightLimit) )
		{
			cout << "Weight Violation" << endl;
			cout << Trips[i].getRealWeight() << "\t" << Trips[i].getWeight() << endl;
			//throw -1;
		}
	}
}

void Solution::ValidateCost(Instance & inst)
{	
	double realCost = 0;
	for (int i = 0; i < (int)Trips.size(); i++)
	{		
		realCost += inst.CalculateTripCost(Trips[i]);
		if ( abs( Trips[i].getCost() - inst.CalculateTripCost(Trips[i]) ) > 1 ) 
		{
			cout << "Trip Cost Miscalculated" << endl;
			cout << "Trips[" << i << "].getCost()\tinst.CalculateTripCost(Trips[" << i << " ])" << endl;
			cout << Trips[i].getCost()  << "\t" <<  inst.CalculateTripCost(Trips[i]) << endl;
			//throw -1;
		}
	}
	if ( abs(getCost() - realCost) > 1 ) 
	{
		cout << "Solution Cost Miscalculated" << endl;
		cout << getCost() << "\t" << realCost << endl;
		//throw -1;
	}
}

int Solution::getNumGifts()
{
	int numGifts = 0;
	for (int i = 0; i < (int)Trips.size(); i++)
	{
		numGifts += Trips[i].getSize();
	}
	return numGifts;
}

void Solution::ValidateGifts(Instance & inst, int initialNumber)
{
	int numGifts = getNumGifts();
	if (numGifts != initialNumber)
	{
		cout << "Current number of gifts (" << numGifts << ") is less than the initial number(" << initialNumber << ")!" << endl;
		//throw -1;
	}
}

//always right
double Solution::getCost()
{
	double c = 0;
	for (int i = 0; i < (int)Trips.size(); i++)
	{
		c += Trips[i].getCost();
	}
	return c;
}

void Solution::resetInterNeighborhoods()
{
	for (int i = 0; i < (int)Trips.size(); i++)
	{
		Trips[i].resetInterNeighborhoods();
	}
}

void Solution::setNeighborhoodsAllCleared()
{
	for (int i = 0; i < (int)Trips.size(); i++)
	{
		Trips[i].setNeighborhoodsAllCleared();
	}
}

struct ChainLimit
{
	int gift;
	double cost;
	ChainLimit() : gift(-1), cost(0) {}
	ChainLimit(int g, double c) : gift(g), cost(c) {}
};

struct ChainLimitComp
{
	bool byCost;
	bool operator() (const ChainLimit& i, const ChainLimit& j)
	{
		if (byCost)
			return (i.cost > j.cost);
		else
			return (i.gift < j.gift);
	}
} chainLimitComp;

ExchangeGraph::ExchangeGraph(Instance& inst, Solution& sol, int movementReach, int numChainLimits, ofstream& log)
{
	for (int i = 0; i < (int)sol.Trips.size(); i++)
	{
		// Pega os limites de cadeia como os arcos de maior custo de cada rota
		vector<ChainLimit> limits;
		for (int j = -1; j < sol.Trips[i].getSize(); j++)
			limits.push_back(ChainLimit(j, sol.Trips[i].getCostAhead(j) - sol.Trips[i].getCostAhead(j+1)));
		chainLimitComp.byCost = true;
		sort(limits.begin(), limits.end(), chainLimitComp);
		int numLimits = numChainLimits;
		if (numLimits > (int)limits.size()) numLimits = limits.size();
		limits.resize(numLimits);
		chainLimitComp.byCost = false;
		sort(limits.begin(), limits.end(), chainLimitComp);

		//criacao de todos os arcos
		tripStarts.push_back(V.size());
		for (int j = 0; j < numLimits; j++)
		{
			for (int k = j; k < numLimits; k++)
			{
				V.push_back( GiftChain(sol, i, limits[j].gift, limits[k].gift) );
			}
		}
	}
	tripStarts.push_back(V.size());

	//for all pair of vertices
	for (int i = 0; i < (int)V.size(); i++)
	{
		int count = 0;
		for (int j = tripStarts[V[i].get_k()+1]; j < (int)V.size(); j++)
		{
			if ( (V[j].get_k() - V[i].get_k()) > movementReach ) break;
			if ( GiftChainCompatible(V[i], V[j]) && (V[j].get_l1() != V[j].get_l2()) )
			{
				double cost = CalculateExchangeCost(inst, sol, V[i], V[j]);
				V[i].addEastArc(j, cost);
				count ++;
			}
			if ( GiftChainCompatible(V[j], V[i]) && (V[i].get_l1() != V[i].get_l2()) )
			{
				double cost = CalculateExchangeCost(inst, sol, V[j], V[i]);
				V[i].addWestArc(j, cost);
				count++;
			}
		}
		if ((i + 1) == tripStarts[V[i].get_k()+1])
			log << "MultiExchange: built " << count << " arcs for nodes " << tripStarts[V[i].get_k()]
					<< "-" << (tripStarts[V[i].get_k()+1]-1) << " of route " << V[i].get_k() << endl; 
	}
}

double CalculateExchangeCost(Instance& inst, Solution& sol, GiftChain& chain_i, GiftChain& chain_j)
{
	//k
	int k1 = chain_i.get_k();

	//k'
	int k2 = chain_j.get_k();

	//l
	int l1 = chain_i.get_l1();

	//l'
	int l2 = chain_i.get_l2();
	
	//t
	int t1 = chain_j.get_l1();

	//t'
	int t2 = chain_j.get_l2();

	//c_a
	double ca = 0;

	//linha 1
	ca += (chain_j.get_q() + sol.Trips[k1].getWeightAt(l2)) * inst.getDistance(sol.Trips[k1].getGiftId(l1), sol.Trips[k2].getGiftId(t1+1));

	//linha 2
	ca += sol.Trips[k1].getWeightAt(l2) * inst.getDistance(sol.Trips[k2].getGiftId(t2), sol.Trips[k1].getGiftId(l2+1));

	//linha 3
	ca += ( sol.Trips[k1].getWeightAt(l2) - sol.Trips[k2].getWeightAt(t2) ) * ( sol.Trips[k2].getDistanceAhead(t1+1) - sol.Trips[k2].getDistanceAhead(t2) );

	//linha 4
	ca -= sol.Trips[k2].getWeightAt(t1) * sol.Trips[k2].getDistanceFrom(t1);

	//linha 5
	ca -= sol.Trips[k2].getWeightAt(t2) * sol.Trips[k2].getDistanceFrom(t2);

	//linha 6
	ca +=  chain_j.get_q() * ( sol.Trips[k1].getDistanceUntil(l1) - sol.Trips[k2].getDistanceUntil(t1) );

	return ca;
}

#define C 1000

GiftChain::GiftChain(Solution& sol, int tripIndex, int l1_, int l2_)
{
	k = tripIndex;
	routeWeight = sol.Trips[k].getWeight() - 10;
	l1 = l1_;
	l2 = l2_;

	q = sol.Trips[k].getWeightAt(l1) - sol.Trips[k].getWeightAt(l2);
}

GiftChain::GiftChain()
{
	k = -2;
	routeWeight = 0;
	l1 = -1;
	l2 = -1;
	q = 0;
}

bool GiftChainCompatible(GiftChain chain_i, GiftChain chain_j)
{
	//weight compatibility
	if ( chain_j.get_q() < C - chain_i.get_routeWeight() + chain_i.get_q() )
		return true;
	else
		return false;
}

double GiftChain::get_q()
{
	return q;
}

int GiftChain::get_k()
{
	return k;
}

int GiftChain::get_l1()
{
	return l1;
}

int GiftChain::get_l2()
{
	return l2;
}

double GiftChain::get_routeWeight()
{
	return routeWeight;
}

bool GiftChain::operator== ( GiftChain& other )
{
    return (get_k() == other.get_k() &&
            get_l1() == other.get_l1() &&
            get_l2() == other.get_l2());
}

void GiftChain::addEastArc(int tail, double cost)
{
	eastArcs.push_back(ExchangeArc(tail, cost));
}

void GiftChain::addWestArc(int tail, double cost)
{
	westArcs.push_back(ExchangeArc(tail, cost));
}

int GiftChain::getEastDegree()
{
	return eastArcs.size();
}

int GiftChain::getWestDegree()
{
	return westArcs.size();
}

ExchangeArc& GiftChain::getEastArc(int index)
{
	return eastArcs[index];
}

ExchangeArc& GiftChain::getWestArc(int index)
{
	return westArcs[index];
}

string GiftChain::print()
{
	stringstream aux;
	aux << "(" << k << ", " << l1 << ", " << l2 << ")";
	return aux.str();
}

void Solution::clearTabu()
{
	for (int i = 0; i < (int)Trips.size(); i++)
	{
		Trips[i].clearTabu();
	}
}
