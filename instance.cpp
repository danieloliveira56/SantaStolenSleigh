#ifndef EC2
#include "ilcplex/cplex.h"
#include "ilcplex/ilocplex.h"

typedef IloArray<IloNumVarArray> NumVarMatrix;
typedef IloArray<IloNumArray> NumMatrix;
#endif

#include "instance.h"
#include "solution.h"
#include "trip.h"
#include "CPUTimer.h"

#define _USE_MATH_DEFINES
#include <math.h>
#include <string>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <algorithm> 

using namespace std;

double haversine(double lat1, double lon1, double lat2, double lon2);
double haversine(double lat1, double lon1, Gift & gift2);
double haversine( Gift & gift1, double lat2, double lon2);
double haversine( Gift & gift1, Gift & gift2);
double fast_haversine( Gift & gift1, Gift & gift2); 
double fast_haversine(double x1, double y1, double z1, double x2, double y2, double z2);

HugeDirectedGraph::HugeDirectedGraph()
{
	numVertices = 0;
	numArcs = 0;;
}

void HugeDirectedGraph::setVertices(int size)
{
	numVertices = size;
	idDistance.resize(size);
}

void HugeDirectedGraph::addArc(int indexFrom, int indexTo, double distance)
{
	numArcs++;
	idDistance[indexFrom][indexTo] = distance;
	idDistance[indexTo][indexFrom] = distance;
}

Instance::Instance (std::string instanceFilename, double weightLimit_, double benchmark, double limitAdjacent)
{
	//distanceCalculations=0;
	weightLimit = weightLimit_;
	//std::cout << std::setprecision(15) << haversine(90,0,88,88);

	std::ifstream instanceFile;
	std::string aux;
	n = 0;

	instanceFile.open(instanceFilename.c_str());	

	sleighWeight = 10; // sledge Weight
	
	instanceFile >> aux;

	while (instanceFile >> aux) {
		//search for 3 separators
		int s1 = aux.find(",");
		int s2 = aux.find(",",s1+1);
		int s3 = aux.find(",",s2+1);

		int id = stoi(aux.substr(0,s1));
		double latitude = stod(aux.substr(s1+1,s2-s1-1));
		double longitude = stod(aux.substr(s2+1,s3-s2-1));
		double weight = stod(aux.substr(s3+1));
		
		Gifts.push_back(Gift(id,latitude,longitude,weight));
		UnsortedGifts.push_back(Gift(id,latitude,longitude,weight));

		n++;	

		if (n == LIMITGIFTS)
		{
			break;
		}
	}

#ifdef PRE_PROCESSED_DISTANCES
	world.setVertices(Gifts.size()+1);

	//north to south pole distance = 20,015.1
	//cout << haversine(90,0,-90,0) << endl;

	//map by number of best adjacents
	for (int i = 0; i < (int)Gifts.size()-1; i++)
	{
		vector<PairIdDistance> candidateAdjacent;
		for (int j = i+1; j < (int)Gifts.size(); j++)
		{			
			double distance = haversine(Gifts[i],Gifts[j]);
			
			if (distance < PRE_PROCESSED_DISTANCES)
			{
				candidateAdjacent.push_back(PairIdDistance(Gifts[j].id, distance));
			}
		}
		
		//sort(candidateAdjacent.begin(), candidateAdjacent.end());

		//for (int j = 0; j < limitAdjacent; j++)
		for (int j = 0; j < candidateAdjacent.size(); j++)
		{
			world.addArc(i, candidateAdjacent[j].id, candidateAdjacent[j].distance);
			//world.addArc(candidateAdjacent[j].id, i, candidateAdjacent[j].distance);
		}
		cout << "Mapped\t" << i+1 << "\t" << world.numArcs <<  endl;
	}
#endif

	std::sort(Gifts.begin(),Gifts.end());
}

//
// Gift constructor.
//
Gift::Gift(int id_, double latitude_, double longitude_, double weight_)
{
	id = id_;
	latitude = latitude_;
	longitude = longitude_;
	weight = weight_;
	npDistance = haversine(90,0,latitude,longitude);
		
	latitudeInRads  = (M_PI/180) * latitude;
    longitudeInRads = (M_PI/180) * longitude;

    // Cartesian coordinates, normalized for a sphere of diameter 1.0
    x = 0.5 * cos(latitudeInRads) * sin(longitudeInRads);
    y = 0.5 * cos(latitudeInRads) * cos(longitudeInRads);
    z = 0.5 * sin(latitudeInRads);
}

bool Gift::isPolar()
{
	return (latitude < (-58));
}

double Instance::EvaluateSol(Solution &sol)
{
	double totalCost = 0;
	for (int i = 0; i < (int)sol.Trips.size(); i++)
	{
		totalCost += CalculateTripCost(sol.Trips[i]);
	}
	return totalCost;
}

//trips should be evaluated at each modification
//full evaluation kept for debugging purposes
double Instance::CalculateTripCost(Trip &trip) {

	if (trip.getSize() == 0)
	{
		return 0;
	}

	double tripWeight = trip.getWeight(); //ranges from 1 to m, first trip index = 1

	if (tripWeight > weightLimit)
	{
		return 1e15;
	}
	
	double calculatedCost = 0;

	calculatedCost += trip.getGift(0).npDistance*tripWeight;
	tripWeight -= trip.getWeight(0);
	for (int i = 1; i < (int)trip.getSize(); i++)
	{
		calculatedCost += fast_haversine(trip.getGift(i-1), trip.getGift(i))*tripWeight;
		tripWeight -= trip.getWeight(i);
	}
	calculatedCost +=  trip.getBackGift().npDistance*tripWeight; 

	if (abs(calculatedCost - trip.getCost()) > 1)
	{
		cout << "Calculated Cost " << calculatedCost << endl;
		cout << "trip.getCost() "<< trip.getCost() << endl;
		cout << "Erro" << endl;
		//throw -1;	
	}

	return calculatedCost;
}

double Instance::EvaluateTripForceLast(Trip &trip, int giftLast) {

	double obj = 0;
	double tripWeight = sleighWeight; //ranges from 1 to m, first trip index = 1

	for (int i = 0; i < (int)trip.getSize(); i++)
	{
		tripWeight += trip.getGift(i).weight;
	}

	
	int initialGift = 1;
	if (giftLast != 0)
	{
		obj += haversine(90,0, trip.getGift(0))*tripWeight;
		tripWeight -= trip.getGift(0).weight;
	}
	else
	{
		obj += haversine(90,0, trip.getGift(1))*tripWeight;
		tripWeight -= trip.getGift(1).weight;
		initialGift = 2;
	}


	for (int i = initialGift; i < trip.getSize(); i++)
	{
		if ( trip.getGiftId(i-1) == trip.getGiftId(giftLast) )
		{
			obj += haversine(trip.getGift(i-2), trip.getGift(i))*tripWeight;
			tripWeight -= trip.getGift(i).weight;
		}
		else
		{
			if (trip.getGiftId(i) != trip.getGiftId(giftLast))
			{
				obj += haversine(trip.getGift(i-1), trip.getGift(i))*tripWeight;
				tripWeight -= trip.getGift(i).weight;
			}
		}
		
	}
	obj += haversine(trip.getBackGift(), trip.getGift(giftLast))*tripWeight;
	tripWeight -= trip.getGift(giftLast).weight;

	obj += haversine(90,0, trip.getGift(giftLast))*tripWeight; //last trip from north pole

	if (obj == trip.getCost())
	{
		cout << "obj " << obj << endl;
		cout << "trip.getCost() "<< trip.getCost() << endl;
		cout << "Erro" << endl;
	}
	return obj;
}

bool Instance::Compatible(Gift & gift1, Gift & gift2, int criteria, double threeshold)
{
	switch (criteria)
	{
		case CRITERIA_DISTANCE:
			return abs(getDistance(gift1.id,gift2.id)) < threeshold;
		case CRITERIA_ANGLE:
			return abs(angle(gift1,gift2)) < threeshold;
		case CRITERIA_ANGLE_DISTANCE:
			return abs(getDistance(gift1.id,gift2.id)*angle(gift1,gift2)) < threeshold;
		default:
			break;
	}
	
	double w1 = gift1.weight;
	double w2 = gift2.weight;
	
	double a = haversine(90,0,gift1);
	double b = haversine(90,0,gift2);
	double c = haversine(gift1, gift2);

	return ((w2*(a+b-c) < sleighWeight*(a-b+c)) && (w2*(-a+b+c) < sleighWeight*(a-b+c)));

	//delivering gift1 then gift2 then back must be better than delivering gift1 then back then gift2 then back
	//and	
	//delivering gift2 then gift1 then back must be better than delivering gift1 then back then gift2 then back
	//in order for then to be compatible, i.e. for it to reasonable that both are in the same trip
}

double Instance::howAligned(Gift & gift1, Gift & gift2)
{
    // (y1z2 - z1y2, z1x2 - x1z2, x1y2 - y1x2)

	double z = (gift1.latitude - 90)*(gift2.longitude-0) - 
				(gift1.longitude- 0)*(gift2.latitude-90);

    return z;
}


double Instance::angle(Gift & gift1, Gift & gift2)
{
    // (y1z2 - z1y2, z1x2 - x1z2, x1y2 - y1x2)
	double a1 = gift1.longitude > 0 ? gift1.longitude : 360 + gift1.longitude;
	double a2 = gift2.longitude > 0 ? gift2.longitude : 360 + gift2.longitude;
	
	return abs(a1-a2);
}


double Instance::getDistance(int idGift1, int idGift2)
{
	//distanceCalculations++;

#ifdef PRE_PROCESSED_DISTANCES
	if ( (idGift1 != 0) && (idGift2 != 0) )
	{
		if (idGift1 < idGift2)
		{
			unordered_map<int,double>::const_iterator it = world.idDistance[idGift1-1].find (idGift2-1);

			if ( it == world.idDistance[idGift1-1].end() )
			{
				return haversine(UnsortedGifts[idGift1-1], UnsortedGifts[idGift2-1]);
			}
			else
			{
				return world.idDistance[idGift1-1][idGift2-1];
			}
		}
		else
		{
			unordered_map<int,double>::const_iterator it = world.idDistance[idGift2-1].find (idGift1-1);

			if ( it == world.idDistance[idGift2-1].end() )
			{
				return haversine(UnsortedGifts[idGift2-1], UnsortedGifts[idGift1-1]);
			}
			else
			{
				return world.idDistance[idGift2-1][idGift1-1];
			}
		}	
	}
#else
	if ( (idGift1 != 0) && (idGift2 != 0) )
	{		
		//return haversine(UnsortedGifts[idGift1-1], UnsortedGifts[idGift2-1]);
		return fast_haversine(UnsortedGifts[idGift1-1], UnsortedGifts[idGift2-1]);
	}
#endif

	if ( (idGift1 == 0) && (idGift2 == 0) )
	{
		return 0;
	}
	else
	{
		if (idGift1 == 0)
		{
			return UnsortedGifts[idGift2-1].npDistance;
		}
		else
		{
			return UnsortedGifts[idGift1-1].npDistance;
		}
	}
	if ( idGift1 == idGift2) 
	{
		return 0;
	}
}


double Instance::getHaversineDistance(double lat1, double lon1, double lat2, double lon2) 
{
	return haversine(lat1, lon1, lat2, lon2);
}

double haversine(double lat1, double lon1, double lat2, double lon2) 
{
	//experimenting
		//double deltaLatExp = abs(lat2-lat1);
		//double deltaLongExp = abs(lon2-lon1);

		//if (deltaLatExp*deltaLongExp > 3000)
		//{
		//	return 1e15;
		//}
	//end - experimenting

	long R = 6371; // kilometers

	double latrad1 = lat1*M_PI/180;
	double latrad2 = lat2*M_PI/180;
	double deltaLat = (lat2-lat1)*M_PI/180;
	double deltaLong = (lon2-lon1)*M_PI/180;

	double a = sin(deltaLat/2) * sin(deltaLat/2) +
			cos(latrad1) * cos(latrad2) *
			sin(deltaLong/2) * sin(deltaLong/2);

	double c = 2 * atan2(sqrt(a), sqrt(1-a));

	return R * c;
}

double fast_haversine( Gift & gift1, Gift & gift2) 
{
	return fast_haversine(gift1.x, gift1.y, gift1.z, gift2.x, gift2.y, gift2.z);
}

double fast_haversine(double x1, double y1, double z1, double x2, double y2, double z2) 
{
    double dX = x1 - x2;
    double dY = y1 - y2;
    double dZ = z1 - z2;

    double r = sqrt(dX*dX + dY*dY + dZ*dZ);

    return 12742 * asin(r); //12742 = 2 * 6371
}

double haversine(double lat1, double lon1, Gift & gift2) 
{
	return haversine(lat1, lon1, gift2.latitude, gift2.longitude);
}

double haversine( Gift & gift1, double lat2, double lon2) 
{
	return haversine(gift1.latitude, gift1.longitude, lat2, lon2);
}

double haversine( Gift & gift1, Gift & gift2) 
{
	return haversine(gift1.latitude, gift1.longitude, gift2.latitude, gift2.longitude);
}

//double haversine( Gift gift1, Gift gift2) 
//{
//	return haversine(gift1.latitude, gift1.longitude, gift2.latitude, gift2.longitude);
//}
//
//double haversine( double lat1, double lon1, Gift gift2) 
//{
//	return haversine(lat1, lon1, gift2.latitude, gift2.longitude);
//}


#ifndef EC2
void Instance::optKara2007()
{
	CPUTimer t;
	t.start();

	int size = Gifts.size();
	double totalWeight = 0;
	for (int i = 0; i < (int)Gifts.size(); i++)
	{
		totalWeight += Gifts[i].weight;
	}

	cout  << "Optimizing Instance by Kara2007 Two-Index One-Commodity Flow Integer Model to CPLEX..." << endl;
	cout  << "Number of Gifts:\t" << size << endl;

	vector<vector<double> > c(size+1);

	for (int i = 0; i < size+1; i++)
	{
		c[i].resize(size+1);
	}

	for (int i = 0; i < size+1; i++)
	{
		for (int j = i+1; j < size+1; j++)
		{
			c[i][j] = getDistance(getGiftId(i), getGiftId(j));			
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

	varName = "K";
	IloNumVar var_K(env, 1, size, IloNumVar::Int, (char*)varName.c_str());

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
	for (int i = 0; i < size; i++)
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
	for (int i = 0; i < size; i++)
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
	IloRange c1c( env, totalWeight, totalWeight, (char*)consName.c_str() );

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
				model.add(IloConstraint( getWeight(j)* var_x[i][j] <= var_f[i][j] )); //o que entra em j deve pelo menos carregar o peso de j
				model.add(IloConstraint( var_f[i][j] <= (totalWeight - getWeight(i)) * var_x[i][j] )); //o que sai de i nao pode carregar o peso de i
			}			
		}
	}

	// (1e) - tudo que entra em 0	
	consName = "c1e_in";
	IloRange c1e_in( env, 0, 0, (char*)consName.c_str() );
	consName = "c1e_out";
	IloRange c1e_out( env, 0, 0, (char*)consName.c_str() );

	c1e_in.setLinearCoef(var_K, -1);
	c1e_out.setLinearCoef(var_K, -1);

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
		cplex.setParam(IloCplex::Threads,1);
		//cplex.setParam(IloCplex::TiLim,10);
		//cplex.setParam(IloCplex::VarSel,3);
	#else
		//cplex.setParam(IloCplex::TiLim,60);
	#endif	
		
	cplex.setParam(IloCplex::CutUp, 13632670.39);

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
			cout << "Best Solution Found:\t" << setprecision(17) << objValue << endl;
			cout << "Num Trips:\t" << setprecision(17) << cplex.getValue(var_K) << endl;

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
	
			/*Trip Kara_Trip();

			while (Kara_Trip.getSize() < size)
			{
				for (int j = 0; j < size+1; j++)
				{
					if ( ( adjacencyMatrix[last][j] == 1) && (!visited[j] ) )
					{
						visited[j] = true;	
						Kara_Trip.appendGift(inst, Gifts[j-1]);
						last = j;
						break;
					}
				}
			}

			cout << "OriginalTripCost:\t" << std::setprecision(17) << getCost() << endl;
			cout << "OptimalKaraTripCost:\t" << std::setprecision(17) << Kara_Trip.getCost() << endl;*/
		}
	}
	catch (IloException e) 
	{
		cerr << e;
	}

	env.end();
}

#endif

int Instance::getGiftId(int index)
{
	if (index == -1)
	{
		return 0; //north pole
	}
	if (index == Gifts.size())
	{
		return 0; //north pole
	}
	else
	{
		return Gifts[index].id;
	}
}

double Instance::getWeight(int index)
{
	if (index == Gifts.size())
	{
		return 0;
	}

	return Gifts[index].weight;
}