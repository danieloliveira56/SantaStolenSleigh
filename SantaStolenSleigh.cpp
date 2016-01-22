#include "prototypes.h"

#ifdef WIN32
	//#define TEST_INITIAL_SOL_CRITERIA
	#define NUM_THREADS 1
#endif
//#define PRINT_TRIP_DETAILS
//#define SAVE_JSON
//#define PRINT_GAP_KARA
//#define CROSSEXCHANGE_ARTUR
#define SET_OLD_SOLUTION_CLEAR
//#define SLOPPY_CROSS_EXCHANGE
#define PARALLEL_CROSS_EXCHANGE
//#define PARALLEL_ILS
#define PARALLEL_INTRA_SEARCH
#define PARALLEL_NET_RELOCATION
#define PARALLEL_SEARCH_CONDENSE

//used for saving only "useful" solutions
double currentBest = 12406500000;

//set searches reach
int searchCrossExchange_reach = 50;
int searchNetRelocation_reach = 50;
int searchNetCX_reach = 2;
int searchAllNetCX_reach = 2;
int searchMultiExchange_reach = 20;
int searchMerge2_reach = 10;
int searchCondense_reach = 2;

int numChainLimits_MultiExchange = 15;

//limit chain size in all Cross Exchange based Seaches
#define L_CX 5

//limits how many successive bad movements the NetCrossExchange can perform before bing terminated
#define searchNetCX_badLimit 10

//not only moves the last gift on the net relocation search
#define FULL_NET_RELOCATION

//set VNS order (0 if search not performed)
#define K_CrossExchange 1
#define K_NetRelocation 2
#define K_NetCrossExchange 0
#define K_AllNetCrossExchange 0
#define K_BiCrossExchange 0
#define K_IntenseCrossExchange 0
#define K_Merge2 0
#define K_Merge3 0
#define K_Merge4 0
#define K_Merge5 0
#define K_Merge2Polar 0
#define K_MultiExchange 0
#define K_Condense3 0
#define K_Condense4 0
#define K_Condense5 0
#define K_Perturbation 3

#define K_Last 3

//perturbations	config
vector<double> totalPerturbationCount(NUM_PERTURBATIONS,0);
vector<double> perturbationAccount(NUM_PERTURBATIONS,0);
vector<int> pertubationProbability(NUM_PERTURBATIONS,0);
vector<string> pertubationName(NUM_PERTURBATIONS);

void setPertubationProbabilities()
{
	pertubationProbability[PERTURBATION_2MERGE] = 0;
	pertubationProbability[PERTURBATION_3MERGE] = 0;
	pertubationProbability[PERTURBATION_RANDOM_CROSS_EXCHANGE] = 10;
	pertubationProbability[PERTURBATION_EXPLODE] = 0;
	pertubationProbability[PERTURBATION_HOLD_SOUTH_POLE] = 0;
	pertubationProbability[PERTURBATION_MERGE_2POLAR] = 0;
	pertubationProbability[PERTURBATION_RANDOM_SHIFT] = 50;
	pertubationProbability[PERTURBATION_RANDOM_EXCHANGE] = 50;
	pertubationProbability[PERTURBATION_SPLIT_LONGEST] = 50;
	pertubationProbability[PERTURBATION_SPLIT_COSTLIER] = 50;


	pertubationName[PERTURBATION_2MERGE] = "MERGE2";
	pertubationName[PERTURBATION_3MERGE] = "MERGE3";
	pertubationName[PERTURBATION_RANDOM_CROSS_EXCHANGE] = "RANDOM_CROSS_EXCHANGE";
	pertubationName[PERTURBATION_EXPLODE] = "EXPLODE";
	pertubationName[PERTURBATION_HOLD_SOUTH_POLE] = "SOUTH_POLE_HOLDING";
	pertubationName[PERTURBATION_MERGE_2POLAR] = "MERGE_2POLAR";
	pertubationName[PERTURBATION_SPLIT_LONGEST] = "SPLIT_LONGEST_ARC";
	pertubationName[PERTURBATION_RANDOM_SHIFT] = "RANDOM_SHIFT";
	pertubationName[PERTURBATION_RANDOM_EXCHANGE] = "RANDOM_EXCHANGE";
	pertubationName[PERTURBATION_SPLIT_COSTLIER] = "SPLIT_COSTLIER_ARC";
}

//faster iterations for not losing solutions with EC2 spot instances
#ifdef EC2
	//limit of performed perturbations 
	int limitFarReachLoop = 5; //30	 
	
	int offsetStep = 5;

	//limit of generated Random CrossExhange Perturbations per Trip
	int limitRndCrossExPert = 10;  
	int limitMerge = 2; //10

	int badILSLimit = NUM_THREADS;
#else

	int limitFarReachLoop = 5;

	//limit of generated Random CrossExhange Perturbations per Trip
	int limitRndCrossExPert = 10; 

	int offsetStep = 10;

	//limit of generated Merge Perturbation (limitMerge each Merge2 and Merge3)
	int limitMerge = 10; //10

	int badILSLimit = 5;
#endif

#ifdef TEST_INITIAL_SOL_CRITERIA
	double localSearch = false;
#else
	double localSearch = true;
#endif

int main( int argc, const char* argv[] )
{
	int seed = (int)time(0);
	srand(seed);
	
	if (argc != 7)
	{
		cout << "use santastolensleigh <criteria code> <criteria main parameter> <partition size> <initial procedure>" <<
			" <perturbation limit per loop on partition> <bonus perturbation limit per found improve>" << endl;
		cout << "CRITERIA_DISTANCE 0 (Main Parameter=breaking distance)" << endl;
		cout << "CRITERIA_ANGLE 1 (Main Parameter=breaking distance)" << endl;
		cout << "CRITERIA_ANGLE_DISTANCE 2 (Main Parameter=breaking distance)" << endl;
		cout << "CRITERIA_FAR_SEED_GREEDY 3 (Main Parameter=breaking distance)" << endl;
		cout << "CRITERIA_FAR_SEED_GREEDY_WD 4 (Main Parameter=breaking weighted distance)" << endl;
		cout << "CRITERIA_SolutionKytojoki2007 5 (Main Parameter=breaking distance)" << endl;
		cout << "CRITERIA_FAR_SEED_GREEDY_COST_PERCENT 6 (Main Parameter=breaking percentage increase)" << endl;
		cout << "CRITERIA_FAR_SEED_GREEDY_LIMIT_TRIPS 7 (Main Parameter=limit of trips)" << endl;
		cout << "CRITERIA_FAR_SEED_GREEDY_EXTENSIVE 8 (Main Parameter=)" << endl;
		cout << "CRITERIA_OLD_SOLUTION 9 (Main Parameter=old solution path)" << endl;

		cout <<  "initial procedure 1 Partitioned Perturbation" << endl;
		cout <<  "initial procedure 2 Full Local Search" << endl;
		
		cout <<  "initial procedure Partitioned Perturbation" << endl;
		cout <<  "initial procedure Full Local Search" << endl;
		return -1;
	}

	int criteria = stoi(argv[1]);
	double threshold = 0; //stod(argv[2]);
	string strArg;
	
	//partitioning to speed perturbation
	int partition_size = stod(argv[3]);
	
	//
	int ILS_loop = stod(argv[4]);
	
	//how man perturbation should be performed by partition
	int perturbationsLimit = stoi(argv[5]);

	
	int perturbation_bonus = stoi(argv[6]);
	
	
	double previousSolution = 0;
	int reverseCount = 0;
	stringstream csvName;
	bool improveFlag = false;
	CPUTimer t;
	CPUTimer t_initial_sol;

	t.start();
	t_initial_sol.start();

	#ifdef _WIN32
		Instance inst("gifts.csv", 1010, 100, 20);
	#else
		Instance inst("gifts.csv", 1010, 100, 20);
	#endif

	switch (criteria)
	{
		case CRITERIA_DISTANCE:
			cout << "Initial Solution Criteria: DISTANCE (" << CRITERIA_DISTANCE << ")";
			threshold = stod(argv[2]);
			break;
		case CRITERIA_ANGLE:
			cout << "Initial Solution Criteria: ANGLE (" << CRITERIA_ANGLE << ")";
			threshold = stod(argv[2]);
			break;
		case CRITERIA_ANGLE_DISTANCE:
			cout << "Initial Solution Criteria: ANGLExDISTANCE (" << CRITERIA_ANGLE_DISTANCE << ")";
			threshold = stod(argv[2]);
			break;
		case CRITERIA_FAR_SEED_GREEDY:
			cout << "Initial Solution Criteria: FAR_SEED_GREEDY (" << CRITERIA_FAR_SEED_GREEDY << ")";
			threshold = stod(argv[2]);
			break;
		case CRITERIA_FAR_SEED_GREEDY_WD:
			cout << "Initial Solution Criteria: FAR_SEED_GREEDY_WL (" << CRITERIA_FAR_SEED_GREEDY_WD << ")";
			threshold = stod(argv[2]);
			break;
		case CRITERIA_SolutionKytojoki2007:
			cout << "Initial Solution Criteria: SolutionKytojoki2007 (" << CRITERIA_SolutionKytojoki2007 << ")";
			threshold = stod(argv[2]);
			break;
		case CRITERIA_FAR_SEED_GREEDY_COST_PERCENT:
			cout << "Initial Solution Criteria: FAR_SEED_GREEDY_COST_PERCENT (" << CRITERIA_FAR_SEED_GREEDY_COST_PERCENT << ")";
			threshold = stod(argv[2]);
			break;
		case CRITERIA_FAR_SEED_GREEDY_LIMIT_TRIPS:
			cout << "Initial Solution Criteria: FAR_SEED_GREEDY_LIMIT_TRIPS (" << CRITERIA_FAR_SEED_GREEDY_LIMIT_TRIPS << ")";
			threshold = stod(argv[2]);
			break;
		case CRITERIA_FAR_SEED_GREEDY_EXTENSIVE:
			cout << "Initial Solution Criteria:  FAR_SEED_GREEDY_EXTENSIVE (" << CRITERIA_FAR_SEED_GREEDY_EXTENSIVE << ")";
			threshold = stod(argv[2]);
			break;
		case CRITERIA_OLD_SOLUTION:
			cout << "Initial Solution Criteria:  OLD_SOLUTION (" << CRITERIA_OLD_SOLUTION << ")";
			strArg = argv[2];
			break;

		default:
			break;
	}
	cout << endl << "Threeshold: " << threshold<< endl;
	
	Solution sol(inst, criteria, threshold, 0, strArg);

	sol.Trips.front().optKara2007(inst, improveFlag);
	return 0;
	
	int initialGiftNumber = sol.getNumGifts();
	
	inst.weightLimit = 1010;

	t_initial_sol.stop();

	cout << "Initial Sol Cost: " << std::setprecision(17) << inst.EvaluateSol(sol) << endl;
	cout << "Initial Number of Trips: " << sol.Trips.size() << endl;	
	cout << "Initial Sol Time: " << t_initial_sol.getCPUTotalSecs() << endl;

	setPertubationProbabilities();
	vector<int> pertubationProbabilities;
	for (int p = 0; p < NUM_PERTURBATIONS; p++)
	{
		for (int i = 0; i < pertubationProbability[p]; i++)
		{
			pertubationProbabilities.push_back(p);
		}
	}	
	
	vector<double> searchAccount(NUM_NEIGHBORHOOD);
	vector<double> candidateSearchAccount(NUM_NEIGHBORHOOD);	
	vector<string> neighborhoodName(NUM_NEIGHBORHOOD);

	neighborhoodName[NEIGHBORHOOD_2Opt] = "2Opt";
	neighborhoodName[NEIGHBORHOOD_3Opt] = "3Opt";
	neighborhoodName[NEIGHBORHOOD_4Opt] = "4Opt";
	neighborhoodName[NEIGHBORHOOD_5Opt] = "5Opt";
	neighborhoodName[NEIGHBORHOOD_IntraRelocation] = "IntraRelocation";
	neighborhoodName[NEIGHBORHOOD_IntraExchange] = "IntraExchange";
	neighborhoodName[NEIGHBORHOOD_IntraChainRelocation] = "IntraChainRelocation";
	neighborhoodName[NEIGHBORHOOD_Segregation] = "Segregation";
	neighborhoodName[NEIGHBORHOOD_KARA] = "KARA";

	neighborhoodName[NEIGHBORHOOD_2OptStar] = "2OptStar";
	neighborhoodName[NEIGHBORHOOD_InterRelocation] = "InterRelocation";
	neighborhoodName[NEIGHBORHOOD_2InterRelocation] = "2InterRelocation";
	neighborhoodName[NEIGHBORHOOD_InterChainRelocation] = "InterChainRelocation";
	neighborhoodName[NEIGHBORHOOD_CrossExchange] = "CrossExchange";
	neighborhoodName[NEIGHBORHOOD_IntenseInterChainRelocation] = "IntenseInterChainRelocation";
	neighborhoodName[NEIGHBORHOOD_IntenseCrossExchange] = "IntenseCrossExchange";
	neighborhoodName[NEIGHBORHOOD_Merge2] = "Merge2";
	neighborhoodName[NEIGHBORHOOD_Merge3] = "Merge3";
	neighborhoodName[NEIGHBORHOOD_Merge4] = "Merge4";
	neighborhoodName[NEIGHBORHOOD_Merge5] = "Merge5";
	neighborhoodName[NEIGHBORHOOD_Net1to1Relocation] = "Net1to1Relocation";
	neighborhoodName[NEIGHBORHOOD_NetRelocation] = "NetRelocation";
	neighborhoodName[NEIGHBORHOOD_BiCrossExchange] = "BiCrossExchange";
	neighborhoodName[NEIGHBORHOOD_NetCrossExchange] = "NetCrossExchange";
	neighborhoodName[NEIGHBORHOOD_AllNetCrossExchange] = "AllNetCrossExchange";
	neighborhoodName[NEIGHBORHOOD_Merge2Polar] = "Merge2Polar";	
	neighborhoodName[NEIGHBORHOOD_Condense3] = "Condense3";
	neighborhoodName[NEIGHBORHOOD_Condense4] = "Condense4";
	neighborhoodName[NEIGHBORHOOD_Condense5] = "Condense5";

	int despairCount = 0;
	
	//prepare for ILS parallelization	
	omp_set_dynamic(0);
	omp_set_num_threads(NUM_THREADS);
	cout << omp_get_num_threads() << endl << endl;
			
	if (localSearch)
	{		
		int badILSCount = 0;

		cout << endl << "Performing Partitioned-ILS..." << endl;
		cout << "Partition size: " << partition_size << endl;
		while (true) //never stop the fun
		{	
			double initialILSCost = sol.getCost();

			cout << endl << "ILS-Loop\t" << ILS_loop << endl;
			bool perturbLoop = false;
			bool resetNeighboorhoods = false;

			//partion Trips every other ILS_loop			
			int nPartitions = 0;

			if (partition_size >= sol.Trips.size())
			{
				perturbLoop = true;
				nPartitions = 1;
				for (int i = 0; i < (int)candidateSearchAccount.size(); i++)
				{
					candidateSearchAccount[i] = 0;
				}
			}
			else
			{
				if (ILS_loop%2 == 0)
				{
					nPartitions = 1;
					perturbLoop = false;
					resetNeighboorhoods = true;
				}
				else
				{
					perturbLoop = true;
					//no trip can stand just by itself
					if (sol.Trips.size() % partition_size < 2)
					{
						nPartitions = sol.Trips.size()/partition_size;
					}
					else
					{
						nPartitions = sol.Trips.size()/partition_size + 1;						
					}
					for (int i = 0; i < (int)candidateSearchAccount.size(); i++)
					{
						candidateSearchAccount[i] = 0;
					}
				}
			}
			
			vector<ofstream*> partitionLog(nPartitions);
			#ifdef PARALLEL_ILS
			for (int i = 0; i < nPartitions; i++)
			{
				stringstream logName;
				logName << "thread_" << i << "_log.txt";
				ofstream auxStream;
				auxStream.open(logName.str());
				partitionLog[i] = &auxStream;
			}
			#else
				stringstream logName;
				logName << "log.txt";
				ofstream auxStream;
				auxStream.open(logName.str());
				for (int i = 0; i < nPartitions; i++)
				{
					partitionLog[i] = &auxStream;
				}
			#endif

			//partitioning of trips
			sol.SortMCLongitude(); 
			vector<Solution> sols(nPartitions);
			vector<Solution> candidate_sols(nPartitions);
				
			int tripsCount = 0;
			{
				int i = 0;
				for (int t = 0; t < (int)sol.Trips.size(); t++)
				{
					if ( (tripsCount >= (partition_size * (i+1) ) ) && (i < (nPartitions - 1) ) )
					{
						i++;						
					}
										
					sols[i].Trips.push_back(sol.Trips[t]);
					tripsCount++;
				}
			}

			double solsCost = 0;
			for (int i = 0; i < nPartitions; i++)
			{
				solsCost += sols[i].getCost();
				candidate_sols[i] = sols[i];		
				
				#ifdef SET_OLD_SOLUTION_CLEAR
				if ( (criteria == CRITERIA_OLD_SOLUTION) && (ILS_loop == 1) )
				{
					candidate_sols[i].setNeighborhoodsAllCleared();
				}
				#endif				
			}	

			if (resetNeighboorhoods)
			{
				for (int i = 0; i < nPartitions; i++)
				{
					for (int t = 0; t < candidate_sols[i].Trips.size(); t++)
					{
						if (candidate_sols[i].Trips[i].improvedByPerturbation)
						{						
							candidate_sols[i].resetInterNeighborhoods();
							candidate_sols[i].Trips[i].improvedByPerturbation = false;
						}
					}					
				}
			}
			//reset pertubation counters
			for (int i = 0; i < NUM_PERTURBATIONS; i++)
			{
				perturbationAccount[i] = 0;
				totalPerturbationCount[i] = 0;
			}

		#ifdef PARALLEL_ILS
			#pragma omp parallel for 	
		#endif
			for (int partition = 0; partition < nPartitions; partition++)
			{		
				#pragma omp critical(output)
				{					
					if (perturbLoop)
					{
						cout << "Partition " << partition + 1 << "/" << nPartitions << " Perturbation loop started, Cost: " << candidate_sols[partition].getCost() << endl;
					}
					else
					{						
						cout << "Full solution local search loop started, Cost: " << candidate_sols[partition].getCost() << endl;
					}
				}
				int perturbationCount = 0;
				int currentPerturbation = -1;			

				//VNS-ILS
				
				int k = 1;
				if (perturbLoop)
				{
					//go straight to perturbing
					k = K_Perturbation;
				}
				while ( perturbationCount < perturbationsLimit ) 
				{
					improveFlag = false;

					//intra route local search
					int intraCount = 0;
												
					*partitionLog[partition] << "Intra Full-Optimization Trip (" << candidate_sols[partition].Trips.size() << " trips)" << endl;
					*partitionLog[partition] << "Current Cost: " << std::setprecision(17) << inst.EvaluateSol(candidate_sols[partition]) << endl;
					
					//intra local	
					searchIntra_Controller(inst, candidate_sols[partition], candidateSearchAccount, *partitionLog[partition]); 

					candidate_sols[partition].ValidateWeights(inst);
					candidate_sols[partition].ValidateCost(inst);	
															
					//inter route local search																	
					if (k == K_CrossExchange)
					{				
						bool overallImproveFlag = false;

						candidate_sols[partition].SortMCLongitude();
						candidate_sols[partition].MarkPositions();

						searchCrossExchange_Controller(inst, candidate_sols[partition], overallImproveFlag, 
							candidateSearchAccount, *partitionLog[partition]);

						candidate_sols[partition].ValidateWeights(inst);
						candidate_sols[partition].ClearEmptyTrips();				
												
						k = overallImproveFlag ? 1 : k+1;						
					}
					
					if (k == K_NetRelocation)
					{		
						bool overallImproveFlag = false;

						candidate_sols[partition].SortMCLongitude();
						candidate_sols[partition].MarkPositions();
												
						searchNetRelocation_Controller(inst, candidate_sols[partition], overallImproveFlag, 
						candidateSearchAccount, *partitionLog[partition]);

						candidate_sols[partition].ValidateWeights(inst);
						candidate_sols[partition].ClearEmptyTrips();			
												
						k = overallImproveFlag ? 1 : k+1;					
					}

					if (k == K_NetCrossExchange)
					{	
						bool overallImproveFlag = false;

						searchNetCrossExchange_Controller(inst, candidate_sols[partition], overallImproveFlag, 
						candidateSearchAccount, *partitionLog[partition]);

						k = overallImproveFlag ? 1 : k+1;				
					}		
	
					if (k == K_AllNetCrossExchange)
					{		
						bool overallImproveFlag = false;

						candidate_sols[partition].SortMCLongitude();
						candidate_sols[partition].MarkPositions();
						
						searchAllNetCrossExchange_Controller(inst, candidate_sols[partition], overallImproveFlag,
						candidateSearchAccount, *partitionLog[partition]);

						candidate_sols[partition].ValidateWeights(inst);
						candidate_sols[partition].ClearEmptyTrips();	
						
						k = overallImproveFlag ? 1 : k+1;					
					}
				
					if (k == K_Merge2Polar)
					{
						bool overallImproveFlag = false;

						candidate_sols[partition].SortMCLongitude();
						candidate_sols[partition].MarkPositions();

						searchMerge2Polar_Controller(inst, candidate_sols[partition], overallImproveFlag,
						candidateSearchAccount, *partitionLog[partition]);

						candidate_sols[partition].ValidateWeights(inst);
						candidate_sols[partition].ClearEmptyTrips();												
												
						k = overallImproveFlag ? 1 : k+1;						
					}

					if (k == K_Merge2)
					{
						bool overallImproveFlag = false;

						candidate_sols[partition].SortMCLongitude();
						candidate_sols[partition].MarkPositions();

						searchMerge2_Controller(inst, candidate_sols[partition], overallImproveFlag,
						candidateSearchAccount, *partitionLog[partition]);

						candidate_sols[partition].ValidateWeights(inst);
						candidate_sols[partition].ClearEmptyTrips();												
												
						k = overallImproveFlag ? 1 : k+1;						
					}

					if (k == K_Merge3)
					{
						bool overallImproveFlag = false;

						candidate_sols[partition].SortMCLongitude();
						candidate_sols[partition].MarkPositions();

						searchMerge3_Controller(inst, candidate_sols[partition], overallImproveFlag,
						candidateSearchAccount, *partitionLog[partition]);

						candidate_sols[partition].ValidateWeights(inst);
						candidate_sols[partition].ClearEmptyTrips();												
												
						k = overallImproveFlag ? 1 : k+1;						
					}

					if (k == K_Merge4)
					{
						bool overallImproveFlag = false;

						candidate_sols[partition].SortMCLongitude();
						candidate_sols[partition].MarkPositions();

						searchMerge4_Controller(inst, candidate_sols[partition], overallImproveFlag,
						candidateSearchAccount, *partitionLog[partition]);

						candidate_sols[partition].ValidateWeights(inst);
						candidate_sols[partition].ClearEmptyTrips();												
												
						k = overallImproveFlag ? 1 : k+1;						
					}

					if (k == K_Merge5)
					{
						bool overallImproveFlag = false;

						candidate_sols[partition].SortMCLongitude();
						candidate_sols[partition].MarkPositions();

						searchMerge5_Controller(inst, candidate_sols[partition], overallImproveFlag,
						candidateSearchAccount, *partitionLog[partition]);

						candidate_sols[partition].ValidateWeights(inst);
						candidate_sols[partition].ClearEmptyTrips();												
												
						k = overallImproveFlag ? 1 : k+1;						
					}

					if (k == K_Condense3)
					{
						bool overallImproveFlag = false;

						candidate_sols[partition].SortMCLongitude();
						candidate_sols[partition].MarkPositions();

						searchCondense3_Controller(inst, candidate_sols[partition], overallImproveFlag,
						candidateSearchAccount, *partitionLog[partition]);

						candidate_sols[partition].ValidateWeights(inst);
						candidate_sols[partition].ClearEmptyTrips();												
												
						k = overallImproveFlag ? 1 : k+1;						
					}

					if ( (k == K_Perturbation) && (!perturbLoop) )
					{
						for (int i = 0; i < NUM_NEIGHBORHOOD; i++)
						{
							searchAccount[i] += candidateSearchAccount[i];
							candidateSearchAccount[i] = 0;
						}

						candidate_sols[partition].ClearEmptyTrips();
						long long aux1 = candidate_sols[partition].getCost();
						stringstream csvName1;
						csvName1 << "solution" << aux1 << ".csv";
						sol.toCSVFile(csvName1.str());
							
						break;
					}
										
					if (k == K_Perturbation)
					{					
						bool improveFlag = false;			
						*partitionLog[partition] << "Inter(" << k << "/" << K_Last << ") Perturbation"<< endl;
						
						//check effective of last perturbation
						if ( ( candidate_sols[partition].getCost() < sols[partition].getCost() - RELEVANT_INCREASE) )
						{
							if (currentPerturbation != -1)
							{
								*partitionLog[partition] << "Previous Perturbation (" << pertubationName[currentPerturbation] <<  ")" << perturbationCount << "/" << perturbationsLimit << " Accepted!"<< endl;
							
								//update impacts								
								perturbationAccount[currentPerturbation] += sols[partition].getCost() - candidate_sols[partition].getCost();	

								for (int i = 0; i < candidate_sols[partition].Trips.size(); i++)
								{
									candidate_sols[partition].Trips[i].improvedByPerturbation = true;
								}
								perturbationCount -= perturbation_bonus; //bonus time
							}															

							#pragma omp critical(sols)
							{
								sols[partition] = candidate_sols[partition];

								sol.Trips.clear();
								for (int i = 0; i < nPartitions; i++)
								{
									for (int t = 0; t < (int)sols[i].Trips.size(); t++)
									{
										sol.Trips.push_back(sols[i].Trips[t]);
									}
								}
								//save sol
								if (sol.getCost() < currentBest)
								{
									sol.ClearEmptyTrips();
									currentBest = sol.getCost();
									long long aux1 = currentBest;
									stringstream csvName1;
									csvName1 << "solution" << aux1 << ".csv";
									sol.toCSVFile(csvName1.str());
								}	
							}
							*partitionLog[partition] << "Current Sol\t"<< sols[partition].getCost() << endl;																		
						}
						else
						{
							*partitionLog[partition] << "Previous Perturbation " << perturbationCount << "/" << perturbationsLimit << " Rejected!"<< endl;
							*partitionLog[partition] << "Current Sol\t"<< sols[partition].getCost() << endl;
							
							candidate_sols[partition] = sols[partition];
						}				
					
						//performs next perturbation
						perturbationCount++;
						
						candidate_sols[partition].ClearEmptyTrips();
							
						random_shuffle(pertubationProbabilities.begin(), pertubationProbabilities.end());
						currentPerturbation = pertubationProbabilities.front();																		
						bool hasPerturbation = false;
						while (!hasPerturbation)
						{							
							switch (currentPerturbation)
							{								
								break;
							case PERTURBATION_2MERGE:
								PerturbMerge2(inst, candidate_sols[partition], hasPerturbation);									
								break;
							case PERTURBATION_3MERGE:						
								PerturbMerge3(inst, candidate_sols[partition], hasPerturbation);								
								break;
							case PERTURBATION_RANDOM_CROSS_EXCHANGE:
								PerturbRandomCrossExchange(inst, candidate_sols[partition], rand()%5, hasPerturbation);									
								break;
							case PERTURBATION_EXPLODE:
								PerturbExplode(inst, candidate_sols[partition], hasPerturbation);							
								break;
							case PERTURBATION_HOLD_SOUTH_POLE:
								PerturbHoldSP(inst, candidate_sols[partition], hasPerturbation);								
								break;
							case PERTURBATION_MERGE_2POLAR:
								PerturbMerge2Polar(inst, candidate_sols[partition], hasPerturbation);								
								break;
							case PERTURBATION_SPLIT_LONGEST:
								PerturbSplitLongest(inst, candidate_sols[partition], hasPerturbation);								
								break;
							case PERTURBATION_SPLIT_COSTLIER:
								PerturbSplitCostlier(inst, candidate_sols[partition], hasPerturbation);								
								break;								
							case PERTURBATION_RANDOM_SHIFT:
								PerturbRandomShift(inst, candidate_sols[partition], hasPerturbation);									
								break;
							case PERTURBATION_RANDOM_EXCHANGE:
								PerturbRandomExchange(inst, candidate_sols[partition], hasPerturbation);									
								break;
							default:
								break;
							}			

							if (!hasPerturbation)
							{
								random_shuffle(pertubationProbabilities.begin(), pertubationProbabilities.end());
								currentPerturbation = pertubationProbabilities.front();
							}
						}

						totalPerturbationCount[currentPerturbation]++;

						*partitionLog[partition] << "pert " << perturbationCount << "/" << perturbationsLimit <<  " ("<< pertubationName[currentPerturbation] << ")";
						
						double solsCost = 0;
						for (int i = 0; i < nPartitions; i++)
						{
							solsCost += sols[i].getCost();	
						}

						#pragma omp critical(output)
						{
							cout << "Partition " << partition + 1 << " pert " << perturbationCount << "/" << perturbationsLimit << " ("<< pertubationName[currentPerturbation] << ")"
								<< "\tCurrent sol: " << solsCost << endl;
						}

						//restart VNS
						k = 1;															
					} //end - perturbation phase (k == K_)					
				} //end - VNS-ILS	
				
				//avoid losing an improving perturbation
				if ( ( candidate_sols[partition].getCost() < sols[partition].getCost() - RELEVANT_INCREASE) )
				{
					sols[partition] = candidate_sols[partition];
				}

				#pragma omp critical(output)
				{
					cout << "Partition " << partition + 1 << " ended, Cost: " << sols[partition].getCost() << endl;
				}

				//changing thread, print status here
				cout << "Intra Movements impact" << endl;
				for (int i = 0; i < NEIGHBORHOOD_FIRST_INTER; i++)
				{		
					cout << "\t" << neighborhoodName[i] << ":\t" << std::setprecision(17) << searchAccount[i] << endl;
				}
				cout << endl;
	
				cout << "Inter Movements impact:\t" << endl;	
				for (int i = NEIGHBORHOOD_FIRST_INTER; i < NUM_NEIGHBORHOOD; i++)
				{		
					cout << "\t" << neighborhoodName[i] << ":\t" << std::setprecision(17) << searchAccount[i] << endl;
				}
				cout << endl;
	
				cout << "Perturbation impact:\t" << std::setprecision(17) << endl;
				for (int i = 0; i < NUM_PERTURBATIONS; i++)
				{		
					cout << "\t" << pertubationName[i] << ":\t" << std::setprecision(17) << perturbationAccount[i] << "(" << totalPerturbationCount[i] << ")" << endl;
				}
				cout << endl;
			} //end for (int thread ...

			#ifdef PARALLEL_ILS
			for (int i = 0; i < nPartitions; i++)
			{
				partitionLog[i]->close();
			}
			#else
				partitionLog[0]->close();
			#endif

				sol.Trips.clear();
				for (int i = 0; i < sols.size(); i++)
				{
					for (int t = 0; t < (int)sols[i].Trips.size(); t++)
					{
						sol.Trips.push_back(sols[i].Trips[t]);
					}
				}
				
			if (sol.getCost() > initialILSCost - RELEVANT_INCREASE)
			{
				badILSCount++;
			}					

			ILS_loop++;
		} //end - while badReshuffle	
	} //end if (localSearch)
		
	double finalSolValue = inst.EvaluateSol(sol);
	cout << endl << "Final Sol Value:\t" << std::setprecision(17) << finalSolValue << endl;
	cout << "Final Number of Trips: " << sol.Trips.size() << endl << endl;
	
	cout << "Intra Movements impact" << endl;
	for (int i = 0; i < NEIGHBORHOOD_FIRST_INTER; i++)
	{		
		cout << "\t" << neighborhoodName[i] << ":\t" << std::setprecision(17) << searchAccount[i] << endl;
	}
	cout << endl;
	
	cout << "Inter Movements impact:\t" << endl;	
	for (int i = NEIGHBORHOOD_FIRST_INTER; i < NUM_NEIGHBORHOOD; i++)
	{		
		cout << "\t" << neighborhoodName[i] << ":\t" << std::setprecision(17) << searchAccount[i] << endl;
	}
	cout << endl;
	
	cout << "Perturbation impact:\t" << std::setprecision(17) << endl;
	for (int i = 0; i < NUM_PERTURBATIONS; i++)
	{		
		cout << "\t" << pertubationName[i] << ":\t" << std::setprecision(17) << perturbationAccount[i] << endl;
	}
	cout << endl;

	t.stop();
	cout << "Ellapsed Time: " << t.getCPUTotalSecs() << endl;
	
#ifndef TEST_INITIAL_SOL_CRITERIA
	long long aux = (long long)finalSolValue;
	csvName << "solution" << aux << "_" <<criteria << "_" << threshold << "_" << partition_size << ".csv";
	sol.toCSVFile(csvName.str());
	
#else	
	sol.SortMCLongitude();
	long long aux = finalSolValue;
	csvName << "solution" << aux << "_" <<criteria << "_" << threshold << "_" << partition_size << ".csv";
	sol.toCSVFile(csvName.str());
	
	csvName << "_polar.csv";
	sol.toPolarCSVFile(csvName.str(), true);
	csvName << "_notpolar.csv";
	sol.toPolarCSVFile(csvName.str(), false);

	cout << "***\tCriteria\tThreesold\tpartition_size\tNumber of Trips\tInitial Solution Value" << endl;
	cout << "***\t" << criteria << "\t" << threshold << "\t" << partition_size << "\t" << sol.Trips.size() << "\t" << std::setprecision(17) << finalSolValue << endl;
#endif

#ifdef SAVE_JSON
	sol.SortMCLongitude();
	#ifdef _WIN32
		//system("erase /F /Q D:\\Dropbox\\Dropbox\\www\\azimuthal\\routes\\*");
		//system("erase /F /Q D:\\Dropbox\\Dropbox\\www\\azimuthal\\routes\\points\\*");
		sol.toRoutesFolder("D:\\www2\\santasstolenmap\\solutions\\");
	#else
		//system("rm routes/*");
		//system("rm routes/points/*");
		//sol.toRoutesFolder("routes/", "routes/points/" );
	#endif
#endif

#ifdef PRINT_GAP_KARA
	for (int i = 0; i < (int)sol.Trips.size(); i++)
	{	
		cout << "gapKara2007 trip " << i+1 << ":\t" << std::setprecision(4) << sol.Trips[i].gapKara2007(inst)*100 << "%" << endl;
	}
#endif

#ifdef PRINT_TRIP_DETAILS
	double totalReturnCost = 0;
	int totalGifts = 0;
	for (int i = 0; i < (int)sol.Trips.size(); i++)
	{
		cout << "Trip\t" << i+1 << "\tweight\t" << sol.Trips[i].getWeight() << "\tcost\t" << sol.Trips[i].getCost() << "\tgifts\t" << sol.Trips[i].getSize() << endl;
		totalGifts += sol.Trips[i].getSize();
		totalReturnCost += sol.Trips[i].getBackGift().npDistance*10;
	}
	cout << "Total return cost" << "\t" << totalReturnCost << "\t" << totalReturnCost/a << endl;
	for (int i = 0; i < (int)sol.Trips.size(); i++)
	{
		for (int j = 0; j < sol.Trips[i].getSize(); j++)
		{
			cout << "Trip" << "\t" << i+1 << "\t"
				"Gift" << "\t" << j+1 << "\t"
				"GiftId" << "\t" << sol.Trips[i].getGiftId(j) << "\t"
				"Weight" << "\t" << sol.Trips[i].getWeight(j) <<  "\t"
				"Distance" << "\t" << sol.Trips[i].getDistanceUntil(j) <<  "\t"
				"Cost" << "\t" << sol.Trips[i].getDistanceUntil(j) * sol.Trips[i].getWeight(j) <<  "\t"
				"Min Distance" << "\t" << sol.Trips[i].getGift(j).npDistance  <<  "\t"
				"Min Cost" << "\t" << sol.Trips[i].getGift(j).npDistance * sol.Trips[i].getWeight(j) <<  "\t"
				<< endl;
		}
	}

	cout << "Number of gifts " << totalGifts << endl;

	vector<int> giftIds;

	for (int i = 0; i < (int)sol.Trips.size(); i++)
	{
		for (int j = 0; j < (int)sol.Trips[i].getSize(); j++)
		{
			giftIds.push_back(sol.Trips[i].getGiftId(j));
		}
	}
	sort(giftIds.begin(), giftIds.end());
	for (int i = 1; i < (int)giftIds.size(); i++)
	{
		if (giftIds[i] == giftIds[i-1])
		{
			cout << i << " erro" << endl;
		}
	}
#endif

#ifdef PRE_PROCESSED_DISTANCES
	cout << "Number of arcs: " << inst.world.numArcs << " of " << inst.n*inst.n*2 << " max" << endl;
#endif
		
	//cout << "Number of Distances Contultations " << inst.distanceCalculations << endl;

	#ifdef _WIN32
	system("pause");
	#endif

	return 0;
}

void studyCompatibity(Instance & inst, int criteria, double threeshold)
{
	for (int g = 1; g <= inst.n; g++)
	{
		int nComp = 0;
		for (int i = 1; i <= inst.n; i++)
		{
			if (i != g && inst.Compatible(inst.Gifts[g],inst.Gifts[i], criteria, threeshold))
			{
				nComp++;
				//cout << "Compatible with Gift " << i << endl;
			}
		}
		cout << "Gift " << inst.Gifts[g].id <<  " compatible with " << nComp << " gifts." << endl;
	}
}


//generalizado pelo searchCrossExchange
void searchInterExchange(Instance & inst, Trip & trip1, Trip & trip2, bool & improveFlag, double & account, bool showLog)
{
	double initialCost = inst.CalculateTripCost(trip1) + inst.CalculateTripCost(trip2);
	double bestCost = initialCost;

	stringstream msg;
	msg << "searchInterExchange Initial cost: " << std::setprecision(17) << initialCost << endl;
	
	bool hasSwap = true;
	while (hasSwap)
	{
		int best_i = 0;
		int best_j = 0;
		hasSwap = false;
		for (int i = 0; i < (int)trip1.getSize(); i++)
		{
			for (int j = 0; j < (int)trip2.getSize(); j++)
			{				
				if ( ( trip1.getWeight() - trip1.getWeight(i) +  trip2.getWeight(j) <= inst.weightLimit )  &&
					( trip2.getWeight() - trip2.getWeight(j) +  trip1.getWeight(i) <= inst.weightLimit ) )
				{
					Gift gift1 = trip1.getGift(i);
					Gift gift2 = trip2.getGift(j);

					double newCost =  inst.CalculateTripCost(trip1) + inst.CalculateTripCost(trip2) + trip1.evaluateSetGift(inst,i,gift2) + trip2.evaluateSetGift(inst,j,gift1);

					if (newCost < bestCost - RELEVANT_INCREASE)
					{
						bestCost = newCost;
						hasSwap = true;
						improveFlag = true;
						best_i = i;
						best_j = j;
					}
					
				}		
			}
		}
		if (hasSwap)
		{
			Gift gift1 = trip1.getGift(best_i);
			Gift gift2 = trip2.getGift(best_j);

			trip1.setGift(inst, best_i, gift2);
			trip2.setGift(inst, best_j, gift1);
		}
	}

	double finalCost = inst.CalculateTripCost(trip1) + inst.CalculateTripCost(trip2);	
	msg << "searchInterExchange Initial cost: " << std::setprecision(17) << finalCost << endl;
	if (finalCost < initialCost)
	{
		cout << msg.str();
		account += initialCost-finalCost;
		trip1.resetNeighborhoodStatuses();
		trip2.resetNeighborhoodStatuses();
	}
}

//generalizado pelo searchInterRelocation to empty trip
void searchSegregation(Instance & inst, Trip & trip, vector<Trip> & newTrips, bool & improveFlag, double & account, bool showLog)
{
	stringstream msg;
	if (trip.getSize() == 1)
	{
		return;
	}

	double initialCost = inst.CalculateTripCost(trip);
	double bestCost = initialCost;
	bool hasSwap = true;
	int best_i = 0;
	
	msg << "searchSegregation Initial cost: " << std::setprecision(17) << initialCost << endl;

	while ((hasSwap) && (trip.getSize()>1))
	{
		bestCost = inst.CalculateTripCost(trip);
		hasSwap = false;
		
		//cout << "Trip1: " << trip.printTrip() << endl;
		for (int i = 0; i < (int)trip.getSize(); i++)
		{
			Gift gift = trip.getGift(i);
			
			//cout << "Checking gift index " << i << ", id " << trip.Gifts[i].id;

			//cout << trip.printTripDebug();
			trip.eraseGift(inst, i);
			//cout << trip.printTripDebug();
			Trip newTrip(inst, gift);
			
			double newCost = inst.CalculateTripCost(trip) +  inst.CalculateTripCost(newTrip);
			//cout << " new cost " << newCost;

			if (newCost < bestCost - RELEVANT_INCREASE)
			{
				bestCost = newCost;
				best_i = i;
				hasSwap = true;
				improveFlag = true;
			}
			double test = inst.CalculateTripCost(trip);
			trip.insertGift(inst, i, gift);
			test = inst.CalculateTripCost(trip);
		}
		if (hasSwap)
		{
			//cout << "Found best gift worth segregating, index " << best_i << ", id " << trip.Gifts[best_i].id << endl;
			Gift gift = trip.getGift(best_i);
			trip.eraseGift(inst, best_i);
			newTrips.push_back(Trip(inst, gift));
			//cout << "Trip1*: " << trip.printTrip() << endl;
			//cout << "Trip2*: " << newTrips.back().printTrip() << endl;
		}
	}
	
	double finalCost = inst.CalculateTripCost(trip);	
	msg << "searchSegregation Initial cost: " << std::setprecision(17) << finalCost << endl;
	if (finalCost < initialCost)
	{
		cout << msg.str();
		account += initialCost-finalCost;
		trip.resetNeighborhoodStatuses();
	}
}

void search2OptStar(Instance & inst, Trip & trip1, Trip & trip2, bool & improveFlag, double & account, bool showLog)
{	
	double initialCost = inst.CalculateTripCost(trip1) + inst.CalculateTripCost(trip2);
	stringstream msg;
	msg << "search2OptStar Initial  cost:\t" << std::setprecision(17) << initialCost << endl;

	bool hasSwap = true;
	while (hasSwap)
	{
		double bestIncrease = 0;
		int best_i = 0;
		int best_j = 0;
		hasSwap = false;
		for (int i = 0; i < (int)trip1.getSize()-1; i++)
		{	
			for (int j = 0; j < (int)trip2.getSize()-1; j++)
			{
				double increase =  evaluate2OptStar(inst, trip1, trip2, i, j);
						
				if (increase < bestIncrease - RELEVANT_INCREASE)
				{
					bestIncrease = increase;
					hasSwap = true;
					improveFlag = true;
					best_i = i;
					best_j = j;
				}
			}
		}
		if (hasSwap)
		{
			//cout << evaluateCrossExchange(inst, trip1, trip2, best_i, best_k, best_j, best_l) << endl;
			//cout << trip1.printTrip() << "\t" << trip1.getCost() << endl << endl; 
			//cout << trip2.printTrip() << "\t" << trip2.getCost() << endl << endl;  
			//double oldCost = inst.CalculateTripCost(trip1) + inst.CalculateTripCost(trip2);
			move2OptStar(inst, trip1, trip2, best_i, best_j);
			//double newCost = inst.CalculateTripCost(trip1) + inst.CalculateTripCost(trip2);
			//cout << trip1.printTrip() << "\t" << trip1.getCost() << endl << endl;  
			//cout << trip2.printTrip() << "\t" << trip2.getCost() << endl << endl;   
			//if (abs(newCost-oldCost-bestIncrease) > 0.01)
			//{
			//	cout << "Error on search2OptStar"<< endl;
			//	cout << trip1.printTrip() << "\t" << trip1.getCost() << endl; 
			//	cout << trip2.printTrip() << "\t" << trip2.getCost() << endl; 
			//	cout << best_i  << "\t" << best_j << endl;
			//	cout << newCost << endl;				
			//	cout << oldCost << endl;
			//	cout << newCost - oldCost << "\t" << bestIncrease << endl;
			//	throw -1;
			//}
		}
	}
	
	
	double finalCost = inst.CalculateTripCost(trip1) + inst.CalculateTripCost(trip2);
	msg << "search2OptStar Final cost:\t" << std::setprecision(17) << finalCost << endl;
	if (finalCost < initialCost)
	{
		cout << msg.str();
		account += initialCost-finalCost;
		trip1.resetNeighborhoodStatuses();
		trip2.resetNeighborhoodStatuses();
	}
}

void searchInterRelocation(Instance & inst, Trip & tripFrom, Trip & tripTo, bool & improveFlag, double & account, bool showLog)
{
	
	double bestCost = inst.CalculateTripCost(tripFrom) + inst.CalculateTripCost(tripTo);
	double initialCost = bestCost;
	bool hasSwap = true;
	int best_i = 0;
	int best_j = 0;
	
	stringstream msg;
	msg << "searchInterRelocation Initial cost:\t" << std::setprecision(17) << initialCost << endl;

	while ((hasSwap) & (tripFrom.getSize() > 0) & (tripTo.getSize() > 0))
	{
		//bestCost = inst.CalculateTripCost(tripFrom) + inst.CalculateTripCost(tripTo);
		double bestIncrease = 0;
		hasSwap = false;
		
		//cout << "Trip1: " << tripFrom.printTrip() << endl;
		//cout << "Trip2: " << tripTo.printTrip() << endl;
		for (int i = 0; i < (int)tripFrom.getSize(); i++)
		{			
			if (tripFrom.getWeight(i) + tripTo.getWeight() <= inst.weightLimit)
			{
				Gift gift = tripFrom.getGift(i);
			
				//cout << "Checking gift index " << i << ", id " << trip.Gifts[i].id;
				//cout << tripFrom.getSize() << endl;

				double tripFromIncrease = tripFrom.evaluateEraseGift(inst, i); //deletionIncrease
				/*double tripFromIncrease = inst.CalculateTripCost(tripFrom);
				tripFrom.eraseGift(inst, i);
				tripFromIncrease = inst.CalculateTripCost(tripFrom) - tripFromIncrease;

				if (abs(deletionIncrease - tripFromIncrease) > 0.01)
				{
					cerr << "error in evaluateEraseGift";
					throw -1;
				}*/

				for (int j = 0; j <= (int)tripTo.getSize(); j++)
				{
					double increase = tripFromIncrease + tripTo.evaluateInsertion(inst, j, gift);
					//tripTo.insertGift(inst, j, gift);
					//double newCost = inst.CalculateTripCost(tripFrom) +  inst.CalculateTripCost(tripTo);

					//if (newCost < bestCost)
					if (increase < bestIncrease - RELEVANT_INCREASE)
					{
						//bestCost = newCost;
						bestIncrease = increase;
						best_i = i;
						best_j = j;
						hasSwap = true;
						improveFlag = true;
					}				
					//tripTo.eraseGift(inst, j);				
				}
				//tripFrom.insertGift(inst, i, gift);
			}
		}
		if (hasSwap)
		{
			Gift gift = tripFrom.getGift(best_i);
			tripFrom.eraseGift(inst, best_i);
			if (best_j >= 0) 
			{				
				tripTo.insertGift(inst, best_j, gift);
			}
			else
			{
				tripTo.appendGift(inst, gift);
			}
		}
	}

	double finalCost = inst.CalculateTripCost(tripFrom) + inst.CalculateTripCost(tripTo);
	msg << "searchInterRelocation Final cost:\t" << std::setprecision(17) << finalCost << endl;
	if (finalCost < initialCost)
	{
		cout << msg.str();
		account += initialCost-finalCost;
		tripFrom.resetNeighborhoodStatuses();
		tripTo.resetNeighborhoodStatuses();
	}

}

void search2InterRelocation(Instance & inst, Trip & tripFrom, Trip & tripTo, bool & improveFlag, double & account, bool showLog)
{
	stringstream msg;
	
	double bestCost = inst.CalculateTripCost(tripFrom) + inst.CalculateTripCost(tripTo);
	double initialCost = bestCost;
	bool hasSwap = true;
	int best_i = 0;
	int best_j = 0;
	
	msg << "search2InterRelocation Initial cost:\t" << std::setprecision(17) << bestCost << endl;

	while ((hasSwap) & (tripFrom.getSize() > 1))
	{
		double bestIncrease = 0;
		hasSwap = false;
		
		for (int i = 0; i < (int)tripFrom.getSize()-1; i++)
		{			
			if (tripFrom.getWeight(i) + tripFrom.getWeight(i+1) + tripTo.getWeight() <= inst.weightLimit)
			{
				for (int j = 0; j <= (int)tripTo.getSize(); j++)
				{
					double increase = evaluate2InterRelocation(inst, tripFrom, tripTo, i, j);

					if (increase < bestIncrease - RELEVANT_INCREASE)
					{
						bestIncrease = increase;
						best_i = i;
						best_j = j;
						hasSwap = true;
						improveFlag = true;
					}						
				}
			}
		}
		if (hasSwap)
		{
			double oldCost = inst.CalculateTripCost(tripFrom) + inst.CalculateTripCost(tripTo);
			//cout << tripFrom.printTrip() << "\t" << tripFrom.getCost() << endl; 
			//cout << tripTo.printTrip() << "\t" << tripTo.getCost() << endl; 
			move2InterRelocation(inst, tripFrom, tripTo, best_i, best_j);
			//cout << tripFrom.printTrip() << "\t" << tripFrom.getCost() << endl; 
			//cout << tripTo.printTrip() << "\t" << tripTo.getCost() << endl; 
			double newCost = inst.CalculateTripCost(tripFrom) + inst.CalculateTripCost(tripTo);
			if (abs(newCost-oldCost-bestIncrease) > 0.01)
			{
				cout << "Error on search2OptStar"<< endl;
				cout << tripFrom.printTrip() << "\t" << tripFrom.getCost() << endl; 
				cout << tripTo.printTrip() << "\t" << tripTo.getCost() << endl; 
				cout << best_i  << "\t" << best_j << endl;
				cout << newCost << endl;				
				cout << oldCost << endl;
				cout << newCost - oldCost << "\t" << bestIncrease << endl;
				throw -1;
			}
		}
	}
	
	double finalCost = inst.CalculateTripCost(tripFrom) + inst.CalculateTripCost(tripTo);
	msg << "search2InterRelocation Final cost:\t" << std::setprecision(17) << finalCost << endl;
	if (finalCost < initialCost)
	{
		cout << msg.str();
		account += initialCost-finalCost;
		tripFrom.resetNeighborhoodStatuses();
		tripTo.resetNeighborhoodStatuses();
	}
}

void searchInterChainRelocation(Instance & inst, Trip & tripFrom, Trip & tripTo, bool & improveFlag, double & account, bool showLog)
{
	double initialCost = inst.CalculateTripCost(tripFrom) + inst.CalculateTripCost(tripTo);
	
	stringstream msg;
	msg << "searchInterChainRelocation Initial  cost:\t" << std::setprecision(17) << initialCost << endl;

	bool hasSwap = true;
	while (hasSwap)
	{
		double bestIncrease = 0;
		int best_i = 0;
		int best_j = 0;
		int best_k = 0;
		hasSwap = false;
		for (int i = 0; i < (int)tripFrom.getSize()-1; i++)
		{
			for (int k = i; k < (int)tripFrom.getSize(); k++)
			{	
				for (int j = 0; j <= (int)tripTo.getSize(); j++)
				{
					double increase = evaluateInterChainRelocation(inst, tripFrom, tripTo, i, k, j);
						
					if (increase < bestIncrease - RELEVANT_INCREASE)
					{
						bestIncrease = increase;
						hasSwap = true;
						improveFlag = true;
						best_i = i;
						best_k= k;
						best_j = j;
					}				
				}
			}
		}
		if (hasSwap)
		{
			//cout << evaluateCrossExchange(inst, tripFrom, tripTo, best_i, best_k, best_j, best_l) << endl;
			//cout << tripFrom.printTrip() << "\t" << tripFrom.getCost() << endl << endl; 
			//cout << tripTo.printTrip() << "\t" << tripTo.getCost() << endl << endl;  
			double oldCost = inst.CalculateTripCost(tripFrom) + inst.CalculateTripCost(tripTo);
			moveInterChainRelocation(inst, tripFrom, tripTo, best_i, best_k, best_j);
			double newCost = inst.CalculateTripCost(tripFrom) + inst.CalculateTripCost(tripTo);
			//cout << tripFrom.printTrip() << "\t" << tripFrom.getCost() << endl << endl;  
			//cout << tripTo.printTrip() << "\t" << tripTo.getCost() << endl << endl;   
			if (abs(newCost-oldCost-bestIncrease) > 0.01)
			{
				cout << "Error on InterChainRelocation trips"<< endl;
				cout << tripFrom.printTrip() << "\t" << tripFrom.getCost() << endl; 
				cout << tripTo.printTrip() << "\t" << tripTo.getCost() << endl; 
				cout << best_i  << "\t" << best_k  << "\t" << best_j << endl;
				cout << newCost << endl;				
				cout << oldCost << endl;
				cout << newCost - oldCost << "\t" << bestIncrease << endl;
				throw -1;
			}
		}
	}
	
	double finalCost = inst.CalculateTripCost(tripFrom) + inst.CalculateTripCost(tripTo);
	msg << "searchInterChainRelocation Final cost:\t" << std::setprecision(17) << finalCost << endl;
	if (finalCost < initialCost)
	{
		cout << msg.str();
		account += initialCost-finalCost;
		tripFrom.resetNeighborhoodStatuses();
		tripTo.resetNeighborhoodStatuses();
	}
}


void searchCrossExchange(Instance & inst, Trip & trip1, Trip & trip2, bool & improveFlag, double & account, int index1, int index2, bool endOfRoute=false, bool showLog)
{	
	double initialCost = inst.CalculateTripCost(trip1) + inst.CalculateTripCost(trip2);
	
	stringstream msg;
	msg << "searchCrossExchange Initial  cost:\t" << std::setprecision(17) << initialCost << endl;

	bool hasSwap = true;
	while (hasSwap)
	{
		double bestIncrease = 0;
		double lastBestIncrease = 0;
		int best_i = 0;
		int best_j = 0;
		int best_k = 0;
		int best_l = 0;
		int best_sense = 0;
		hasSwap = false;

#ifdef PARALLEL_CROSS_EXCHANGE
		#pragma omp parallel for 	
#endif
		for (int i = -1; i < (int)trip1.getSize(); i++)
		{
		#ifdef L_CX
			for (int k = i; ( (k < (int)trip1.getSize()) && (k < i + L_CX) ); k++)
		#else
			for (int k = i; k < (int)trip1.getSize(); k++)
		#endif
			{	
				if (endOfRoute) k = trip1.getSize() - 1;
				for (int j = -1; j < (int)trip2.getSize(); j++)
				{			
				#ifdef L_CX
					for (int l = j; ( (l < (int)trip2.getSize()) && (l < j + L_CX) ); l++)
				#else
					for (int l = j; l < (int)trip2.getSize(); l++)
				#endif
					{
						if (endOfRoute) l = trip2.getSize() - 1;
						//if chain1 or chain2 is not empty
						if ((i<k) || (j<l))
						{
							int sense = 0;
							double increase =  evaluateCrossExchange(inst, trip1, trip2, i, k, j, l, sense);						
							
							if (increase < lastBestIncrease - RELEVANT_INCREASE)
							{
								#pragma omp critical(really_check)
								{
									if (increase < bestIncrease - RELEVANT_INCREASE)
									{
										bestIncrease = increase;
										lastBestIncrease = increase;
										hasSwap = true;
										improveFlag = true;
										best_i = i;
										best_k= k;
										best_j = j;
										best_l = l;
										best_sense = sense;
									}		
								}
							}				
						}
					}				
				}
			}
		}
		if (hasSwap)
		{
			//cout << evaluateCrossExchange(inst, trip1, trip2, best_i, best_k, best_j, best_l) << endl;
			//cout << trip1.printTrip() << "\t" << trip1.getCost() << endl << endl; 
			//cout << trip2.printTrip() << "\t" << trip2.getCost() << endl << endl;  
			double oldCost = inst.CalculateTripCost(trip1) + inst.CalculateTripCost(trip2);
			moveCrossExchange(inst, trip1, trip2, best_i, best_k, best_j, best_l, best_sense);
			double newCost = inst.CalculateTripCost(trip1) + inst.CalculateTripCost(trip2);
			//cout << trip1.printTrip() << "\t" << trip1.getCost() << endl << endl;  
			//cout << trip2.printTrip() << "\t" << trip2.getCost() << endl << endl;   
			if (abs(newCost-oldCost-bestIncrease) > 0.01)
			{
				cout << "Error on CrossExchanging trips"<< endl;
				cout << trip1.printTrip() << "\t" << trip1.getCost() << endl; 
				cout << trip2.printTrip() << "\t" << trip2.getCost() << endl; 
				cout << best_i  << "\t" << best_k  << "\t" << best_j  << "\t" << best_l << "\t" << best_sense << endl;
				cout << newCost << endl;				
				cout << oldCost << endl;
				cout << newCost - oldCost << "\t" << bestIncrease << endl;
				throw -1;
			}
		}
	}
	
	double finalCost = inst.CalculateTripCost(trip1) + inst.CalculateTripCost(trip2);
	msg << "searchCrossExchange Final cost:\t" << std::setprecision(17) << finalCost << endl;
	if (finalCost < initialCost)
	{
		if (showLog)
		{
			cout << msg.str();
		}
		account += initialCost-finalCost;		
		trip1.resetNeighborhoodStatuses();
		trip2.resetNeighborhoodStatuses();
		//cout << "{Crossexchange made\t" << index1 << "\t" << index2 << "\t" << initialCost - finalCost << endl;
	}
	else
	{		
		//cout << "InterMassCenterDistance\t" << InterMassCenterDistance(inst,trip1,trip2) << "\tNo CrossExchange Improve" << endl;
	}
}

void SearchNetCrossExchange(Instance & inst, Trip & tripFrom, vector<Trip*>& ptr_kCloseTrips, bool & improveFlag, double & account, bool showLog)
{		
	Trip auxtripFrom = tripFrom;
	Trip besttripFrom = tripFrom;

	int badCount = 0;

	vector<Trip> aux_kCloseTrips;
	vector<Trip> best_kCloseTrips;

	double initialCost = inst.CalculateTripCost(tripFrom);
	for (int t = 0; t < (int)ptr_kCloseTrips.size(); t++)
	{
		initialCost += inst.CalculateTripCost(*ptr_kCloseTrips[t]);
		aux_kCloseTrips.push_back(*ptr_kCloseTrips[t]);
		best_kCloseTrips.push_back(*ptr_kCloseTrips[t]);
	}
		
	stringstream msg;
	msg << "SearchNetCrossExchange Initial  cost:\t" << std::setprecision(17) << initialCost << endl;
	
	double bestNet = - RELEVANT_INCREASE;
	double currentNet = 0;
	double lastMove = 0;
	bool hasCX = true;
	while ( (hasCX) && (badCount < searchNetCX_badLimit) )
	{
		double bestIncrease = 1e15;
		double lastBestIncrease = 1e15;
		int best_t = 0;
		int best_i = 0;
		int best_j = 0;
		int best_k = 0;
		int best_l = 0;
		int best_sense = 0;
		hasCX = false;
		//for every possible destiny trip
#ifdef PARALLEL_CROSS_EXCHANGE
		#pragma omp parallel for 	
#endif
		for (int t = 0; t < (int)aux_kCloseTrips.size(); t++)
		{
			for (int i = -1; i < (int)auxtripFrom.getSize(); i++)
			{
			#ifdef L_CX
				for (int k = i; ( (k < (int)auxtripFrom.getSize()) && (k < i + L_CX) ); k++)
			#else
				for (int k = i; k < (int)auxtripFrom.getSize(); k++)
			#endif
				{	
					for (int j = -1; j < (int)aux_kCloseTrips[t].getSize(); j++)
					{			
					#ifdef L_CX
						for (int l = j; ( (l < (int)aux_kCloseTrips[t].getSize()) && (l < j + L_CX) ); l++)
					#else
						for (int l = j; l < (int)aux_kCloseTrips[t].getSize(); l++)
					#endif
						{
							//if chain1 or chain2 is not empty
							if ((i<k) || (j<l))
							{
								int sense = 0;
								double increase =  evaluateCrossExchange(inst, auxtripFrom, aux_kCloseTrips[t], i, k, j, l, sense);
						
								if (increase < lastBestIncrease - RELEVANT_INCREASE)
								{
									#pragma omp critical(really_check)
									{
										if ( (increase < bestIncrease - RELEVANT_INCREASE) && (abs(increase)>0.01) && (abs(increase+lastMove)>0.01) ) //&& (abs(increase+currentNet)>0.01)
										{
											bestIncrease = increase;
											lastBestIncrease = increase;
											hasCX = true;
											best_i = i;
											best_k= k;
											best_j = j;
											best_l = l;
											best_t = t;
											best_sense = sense;
										}	
									}
								}
							}
						}				
					}
				}
			}		
		}
		if (hasCX)
		{
			//cout << "NetCX move increase = " << bestIncrease << " , Current Net: " << currentNet+bestIncrease << endl;
			lastMove = bestIncrease;
			//cout << evaluateCrossExchange(inst, auxtripFrom, aux_kCloseTrips[t], best_i, best_k, best_j, best_l) << endl;
			//cout << auxtripFrom.printTrip() << "\t" << auxtripFrom.getCost() << endl << endl; 
			//cout << aux_kCloseTrips[t].printTrip() << "\t" << aux_kCloseTrips[t].getCost() << endl << endl;  
			double oldCost = inst.CalculateTripCost(auxtripFrom) + inst.CalculateTripCost(aux_kCloseTrips[best_t]);
			moveCrossExchange(inst, auxtripFrom, aux_kCloseTrips[best_t], best_i, best_k, best_j, best_l, best_sense);
			double newCost = inst.CalculateTripCost(auxtripFrom) + inst.CalculateTripCost(aux_kCloseTrips[best_t]);
			//cout << auxtripFrom.printTrip() << "\t" << auxtripFrom.getCost() << endl << endl;  
			//cout << aux_kCloseTrips[t].printTrip() << "\t" << aux_kCloseTrips[t].getCost() << endl << endl;   
			if (abs(newCost-oldCost-bestIncrease) > 0.01)
			{
				cout << "Error on NetCrossExchanging trips"<< endl;
				cout << auxtripFrom.printTrip() << "\t" << auxtripFrom.getCost() << endl; 
				cout << aux_kCloseTrips[best_t].printTrip() << "\t" << aux_kCloseTrips[best_t].getCost() << endl; 
				cout << best_i  << "\t" << best_k  << "\t" << best_j  << "\t" << best_l << "\t" << best_sense << endl;
				cout << newCost << endl;				
				cout << oldCost << endl;
				cout << newCost - oldCost << "\t" << bestIncrease << endl;
				throw -1;
			}
			currentNet += bestIncrease;
		}
		if (currentNet < bestNet -RELEVANT_INCREASE)
		{
			bestNet = currentNet;
			besttripFrom = auxtripFrom;
			for (int t = 0; t < (int)ptr_kCloseTrips.size(); t++)
			{
				best_kCloseTrips[t] = aux_kCloseTrips[t];
			}
			badCount = 0;
		}
		else
		{			
			badCount++;
		}
	}
	
	double finalCost = inst.CalculateTripCost(besttripFrom);
	for (int t = 0; t < (int)best_kCloseTrips.size(); t++)
	{
		finalCost += inst.CalculateTripCost(best_kCloseTrips[t]);
	}

	msg << "SearchNetCrossExchange Final cost:\t" << std::setprecision(17) << finalCost << endl;
	if (finalCost < initialCost - RELEVANT_INCREASE)
	{
		if (showLog)
		{
			cout << msg.str();
		}
		tripFrom = besttripFrom;
		tripFrom.resetNeighborhoodStatuses();
		for (int t = 0; t < (int)best_kCloseTrips.size(); t++)
		{
			*ptr_kCloseTrips[t] = best_kCloseTrips[t];
			ptr_kCloseTrips[t]->resetNeighborhoodStatuses();
		}

		account += initialCost-finalCost;	
		
		improveFlag = true;
	}
}


void SearchAllNetCrossExchange(Instance& inst, vector<Trip*>& ptr_Trips, bool& improveFlag, double& account, bool showLog)
{	
	int badCount = 0;

	vector<Trip> aux_Trips;
	vector<Trip> best_Trips;

	double initialCost = 0;
	for (int t = 0; t < (int)ptr_Trips.size(); t++)
	{
		initialCost += inst.CalculateTripCost(*ptr_Trips[t]);
		aux_Trips.push_back(*ptr_Trips[t]);
		best_Trips.push_back(*ptr_Trips[t]);
	}
		
	stringstream msg;
	msg << "SearchAllNetCrossExchange Initial  cost:\t" << std::setprecision(17) << initialCost << endl;
	
	double bestNet = - RELEVANT_INCREASE;
	double currentNet = 0;
	double lastMove = 0;
	bool hasCX = true;
	while ( (hasCX) && (badCount < searchNetCX_badLimit) )
	{
		double bestIncrease = 1e15;
		double lastBestIncrease = 1e15;
		int best_t1 = 0;
		int best_t2 = 0;
		int best_i = 0;
		int best_j = 0;
		int best_k = 0;
		int best_l = 0;
		int best_sense = 0;
		hasCX = false;
		//for every possible destiny trip
#ifdef PARALLEL_CROSS_EXCHANGE
		#pragma omp parallel for 	
#endif
		for (int t1 = 0; t1 < (int)ptr_Trips.size()-1; t1++)
		{
			for (int t2 = t1+1; t2 < (int)ptr_Trips.size(); t2++)
			{
				for (int i = -1; i < (int)aux_Trips[t1].getSize(); i++)
				{
				#ifdef L_CX
					for (int k = i; ( (k < (int)aux_Trips[t1].getSize()) && (k < i + L_CX) ); k++)
				#else
					for (int k = i; k < (int)aux_Trips[t1].getSize(); k++)
				#endif
					{	
						for (int j = -1; j < (int)aux_Trips[t2].getSize(); j++)
						{			
						#ifdef L_CX
							for (int l = j; ( (l < (int)aux_Trips[t2].getSize()) && (l < j + L_CX) ); l++)
						#else
							for (int l = j; l < (int)aux_Trips[t2].getSize(); l++)
						#endif
							{
								//if chain1 or chain2 is not empty
								if ((i<k) || (j<l))
								{
									int sense = 0;
									double increase =  evaluateCrossExchange(inst, aux_Trips[t1], aux_Trips[t2], i, k, j, l, sense);
									if (increase < lastBestIncrease - RELEVANT_INCREASE)
									{
										#pragma omp critical(really_check)
										{
											if ( (increase < bestIncrease - RELEVANT_INCREASE) && (abs(increase)>0.01) && (abs(increase+lastMove)>0.01) ) //&& (abs(increase+currentNet)>0.01)
											{
												bestIncrease = increase;
												lastBestIncrease = increase;
												hasCX = true;
												best_i = i;
												best_k= k;
												best_j = j;
												best_l = l;
												best_t1 = t1;
												best_t2 = t2;
												best_sense = sense;
											}	
										}
									}
								}
							}				
						}
					}
				}
			}		
		}
		if (hasCX)
		{
			//cout << "NetCX move increase = " << bestIncrease << " , Current Net: " << currentNet+bestIncrease << endl;
			lastMove = bestIncrease;
			//cout << evaluateCrossExchange(inst, aux_Trips[t1], aux_Trips[t2], best_i, best_k, best_j, best_l) << endl;
			//cout << aux_Trips[t1].printTrip() << "\t" << aux_Trips[t1].getCost() << endl << endl; 
			//cout << aux_Trips[t2].printTrip() << "\t" << aux_Trips[t2].getCost() << endl << endl;  
			double oldCost = inst.CalculateTripCost(aux_Trips[best_t1]) + inst.CalculateTripCost(aux_Trips[best_t2]);
			moveCrossExchange(inst, aux_Trips[best_t1], aux_Trips[best_t2], best_i, best_k, best_j, best_l, best_sense);
			double newCost = inst.CalculateTripCost(aux_Trips[best_t1]) + inst.CalculateTripCost(aux_Trips[best_t2]);
			//cout << aux_Trips[t1].printTrip() << "\t" << aux_Trips[t1].getCost() << endl << endl;  
			//cout << aux_Trips[t2].printTrip() << "\t" << aux_Trips[t2].getCost() << endl << endl;   
			if (abs(newCost-oldCost-bestIncrease) > 0.01)
			{
				cout << "Error on AllNetCrossExchanging trips"<< endl;
				cout << aux_Trips[best_t1].printTrip() << "\t" << aux_Trips[best_t1].getCost() << endl; 
				cout << aux_Trips[best_t2].printTrip() << "\t" << aux_Trips[best_t2].getCost() << endl; 
				cout << best_i  << "\t" << best_k  << "\t" << best_j  << "\t" << best_l << "\t" << best_sense << endl;
				cout << newCost << endl;				
				cout << oldCost << endl;
				cout << newCost - oldCost << "\t" << bestIncrease << endl;
				throw -1;
			}
			currentNet += bestIncrease;
		}
		if (currentNet < bestNet -RELEVANT_INCREASE)
		{
			bestNet = currentNet;
			for (int t = 0; t < (int)aux_Trips.size(); t++)
			{
				best_Trips[t] = aux_Trips[t];
			}
			badCount = 0;
		}
		else
		{			
			badCount++;
		}
	}
	
	double finalCost = 0;
	for (int t = 0; t < (int)best_Trips.size(); t++)
	{
		finalCost += inst.CalculateTripCost(best_Trips[t]);
	}

	msg << "SearchAllNetCrossExchange Final cost:\t" << std::setprecision(17) << finalCost << endl;
	if (finalCost < initialCost - RELEVANT_INCREASE)
	{
		if (showLog)
		{
			cout << msg.str();
		}
		for (int t = 0; t < (int)best_Trips.size(); t++)
		{
			*ptr_Trips[t] = best_Trips[t];
			ptr_Trips[t]->resetNeighborhoodStatuses();
		}

		account += initialCost-finalCost;	
		
		improveFlag = true;
	}
}

void searchCrossExchangeArtur(Instance & inst, Trip & trip1, Trip & trip2, bool & improveFlag, double & account, int index1, int index2, bool showLog)
{	
	double initialCost = inst.CalculateTripCost(trip1) + inst.CalculateTripCost(trip2);
	
	stringstream msg;
	msg << "searchCrossExchange Initial  cost:\t" << std::setprecision(17) << initialCost << endl;

	bool hasSwap = true;
	while (hasSwap)
	{
		double bestIncrease = 0;
		int best_i = 0;
		int best_j = 0;
		int best_k = 0;
		int best_l = 0;
		int best_sense = 0;
		hasSwap = false;
		
		for (int i = -1; i < (int)trip1.getSize(); i++)
		{
			//
			bool over1 = false;
			for (int k = i; k < (int)trip1.getSize(); k++)
			{
				double weightChain1 = trip1.getWeightAt(i) - trip1.getWeightAt(k);	

				for (int j = -1; j < (int)trip2.getSize(); j++)
				{				
					double maxWeightChain2 = trip2.getWeightAt(j) - trip2.getWeightAt(trip2.getSize());

					//check if impossible to fit chain1 on trip2
					if ( trip2.getWeight() + weightChain1 - maxWeightChain2 > inst.weightLimit ) 
					{	
						break;
					}
					
					for (int l = j; l < (int)trip2.getSize(); l++)
					{
						int sense = 0;
						double increase;
						increase = evaluateCrossExchange(inst, trip1, trip2, i, k, j, l, sense);
						 
						if (increase < bestIncrease - RELEVANT_INCREASE)
						{
							bestIncrease = increase;
							hasSwap = true;
							improveFlag = true;
							best_i = i;
							best_k = k;
							best_j = j;
							best_l = l;
							best_sense = sense;
						}
					}
				}
			}
		}
		if (hasSwap)
		{
			//cout << evaluateCrossExchange(inst, trip1, trip2, best_i, best_k, best_j, best_l) << endl;
			//cout << trip1.printTrip() << "\t" << trip1.getCost() << endl << endl; 
			//cout << trip2.printTrip() << "\t" << trip2.getCost() << endl << endl;  
			double oldCost = inst.CalculateTripCost(trip1) + inst.CalculateTripCost(trip2);
			moveCrossExchange(inst, trip1, trip2, best_i, best_k, best_j, best_l, best_sense);
			double newCost = inst.CalculateTripCost(trip1) + inst.CalculateTripCost(trip2);
			//cout << trip1.printTrip() << "\t" << trip1.getCost() << endl << endl;  
			//cout << trip2.printTrip() << "\t" << trip2.getCost() << endl << endl;   
			if (abs(newCost-oldCost-bestIncrease) > 0.01)
			{
				cout << "Error on CrossExchanging trips"<< endl;
				cout << trip1.printTrip() << "\t" << trip1.getCost() << endl; 
				cout << trip2.printTrip() << "\t" << trip2.getCost() << endl; 
				cout << best_i  << "\t" << best_k  << "\t" << best_j  << "\t" << best_l << endl;
				cout << newCost << endl;				
				cout << oldCost << endl;
				cout << newCost - oldCost << "\t" << bestIncrease << endl;
				throw -1;
			}
		}
	}
	
	double finalCost = inst.CalculateTripCost(trip1) + inst.CalculateTripCost(trip2);
	msg << "searchCrossExchange Final cost:\t" << std::setprecision(17) << finalCost << endl;
	if (finalCost < initialCost)
	{
		if (showLog)
		{
			cout << msg.str();
		}
		account += initialCost-finalCost;		
		trip1.resetNeighborhoodStatuses();
		trip2.resetNeighborhoodStatuses();
		//cout << "{Crossexchange made\t" << index1 << "\t" << index2 << "\t" << initialCost - finalCost << endl;
	}
	else
	{		
		//cout << "InterMassCenterDistance\t" << InterMassCenterDistance(inst,trip1,trip2) << "\tNo CrossExchange Improve" << endl;
	}
}

void SearchMerge(Instance & inst, vector<Trip*>& ptr_Trips, Trip & newTrip, bool & improveFlag, double & account, bool showLog)
{
	double initialCost = 0;
	for (int i = 0; i < (int)ptr_Trips.size(); i++)
	{
		 initialCost += inst.CalculateTripCost(*ptr_Trips[i]);
	}
	
	stringstream msg;
	msg << "SearchMerge Initial  cost:\t" << std::setprecision(17) << initialCost << endl;

	for (int i = 0; i < (int)ptr_Trips.size(); i++)
	{
		 for (int g = 0; g < ptr_Trips[i]->getSize(); g++)
		{
			newTrip.appendGift(inst, ptr_Trips[i]->getGift(g));
		}
	}
	
	double account_search2opt = 0;
	double account_search3opt = 0;
	double account_searchIntraRelocation = 0;
	double account_searchIntraChainRelocation = 0;
	double account_searchIntraExchange = 0;

	//perform all single trip movements on newTrip
	bool improveFlag2 = true;
	while (improveFlag2)
	{
		improveFlag2 = false;
		//try search2Opt (2Opt)					
		newTrip.search2Opt(inst, improveFlag2, account_search2opt, false);

		//try  search3Opt (3Opt)		
		newTrip.search3Opt(inst, improveFlag2, account_search3opt, false);

		//try  searchIntraRelocation (relocates a single gift to a new place in the trip)
		newTrip.searchIntraRelocation(inst, improveFlag2, account_searchIntraRelocation, false);

		//try searchIntraChainRelocation (relocates a chain of gifts to a new place into the trip		
		newTrip.searchIntraChainRelocation(inst, improveFlag2, account_searchIntraChainRelocation, false);
							
		//try searchIntraExchange (intra trip 2 gift swap)
		newTrip.searchIntraExchange(inst, improveFlag2, account_searchIntraExchange, false);		
	}

	double finalCost = inst.CalculateTripCost(newTrip);
	msg << "SearchMerge Final cost:\t" << std::setprecision(17) << finalCost << "\t" << finalCost - initialCost << endl;
	if (finalCost < initialCost - RELEVANT_INCREASE)
	{
		if (showLog)
		{
			cout << msg.str();
		}
		account += initialCost-finalCost;
		improveFlag = true;
	}
	else
	{
		//cout << msg.str();
		newTrip = Trip(inst); //erase newTrip
	}
}

//tries to lower the number of trips by reseeding and greedy insertion
void SearchCondense(Instance & inst, vector<Trip*>& ptr_Trips, vector<Trip*>& ptr_newTrips, bool & improveFlag, double & account, bool showLog)
{
	double initialCost = 0;
	for (int i = 0; i < (int)ptr_Trips.size(); i++)
	{
		 initialCost += inst.CalculateTripCost(*ptr_Trips[i]);
	}
	stringstream msg;
	msg << "SearchCondense Initial  cost:\t" << std::setprecision(17) << initialCost << endl;
	
	int originalNumber = ptr_Trips.size();
	int targetNumber = originalNumber-1;

	vector<Trip> newTrips(targetNumber, inst);
	vector<Gift> bagOfGifts;

	for (int i = 0; i < ptr_Trips.size(); i++)
	{
		for (int j = 0; j < ptr_Trips[i]->getSize(); j++)
		{
			bagOfGifts.push_back(ptr_Trips[i]->getGift(j));
		}
	}

	//find furthest from NP to seed
	for (int i = 0; i < targetNumber; i++)
	{
		double farthest_g = 0;
		double farthestDistance = 0;
		for (int g = 0; g < bagOfGifts.size(); g++)
		{
			if (bagOfGifts[g].npDistance > farthestDistance)
			{
				farthest_g = g;
			}
		}
		Gift auxGift = bagOfGifts[farthest_g];
		bagOfGifts.erase(bagOfGifts.begin()+farthest_g);
		newTrips[i].appendGift(inst, auxGift);
	}

	for (int g = 0; g < bagOfGifts.size(); g++)
	{
		bool hasTrip = false;
		int bestTrip = 0;
		double bestIncrease = 1e15;
		
		for (int t = 0; t < (int)newTrips.size(); t++)
		{
			if (newTrips[t].getWeight() + bagOfGifts[g].weight <= inst.weightLimit)
			{
				double increase = newTrips[t].evaluateInsertionLength(inst, 0, bagOfGifts[g]); //a
							
				if (increase < bestIncrease)
				{
					bestIncrease = increase;
					bestTrip = t;
					hasTrip = true;
				}				
			}
		}
		if (hasTrip)
		{
			newTrips[bestTrip].insertGift(inst, 0, bagOfGifts[g]);
		}
		else
		{
			//not used - left here just in case i decide to change the criteria later
			newTrips.push_back(Trip(inst, bagOfGifts[g])); 
		}
	}	
	
	double auxAccount = 0;
	vector<double> auxAccountVec(NUM_NEIGHBORHOOD);

	ofstream auxLog;
	Solution auxSol(inst);
	for (int i = 0; i < (int)newTrips.size(); i++)
	{
		auxSol.Trips.push_back(newTrips[i]);
	}

	bool improveFlag3 = true;
	while (improveFlag3)
	{
		improveFlag = false;
		//perform all single trip movements on newTrips
		#ifdef PARALLEL_SEARCH_CONDENSE
		#pragma omp parallel for
		#endif
		for (int i = 0; i < auxSol.Trips.size(); i++)
		{
			bool improveFlag2 = true;
			while (improveFlag2)
			{
				improveFlag2 = false;
				//try search2Opt (2Opt)					
				auxSol.Trips[i].search2Opt(inst, improveFlag2, auxAccount, false);

				//try  search3Opt (3Opt)		
				auxSol.Trips[i].search3Opt(inst, improveFlag2, auxAccount, false);

				//try  searchIntraRelocation (relocates a single gift to a new place in the trip)
				auxSol.Trips[i].searchIntraRelocation(inst, improveFlag2, auxAccount, false);

				//try searchIntraChainRelocation (relocates a chain of gifts to a new place into the trip		
				auxSol.Trips[i].searchIntraChainRelocation(inst, improveFlag2, auxAccount, false);
							
				//try searchIntraExchange (intra trip 2 gift swap)
				auxSol.Trips[i].searchIntraExchange(inst, improveFlag2, auxAccount, false);		
			}
		}

		//inter search
		auxSol.SortMCLongitude();
		auxSol.MarkPositions();

		searchCrossExchange_Controller(inst, auxSol, improveFlag3, 
			auxAccountVec, auxLog, false);
		searchAllNetCrossExchange_Controller(inst, auxSol, improveFlag3,
			auxAccountVec, auxLog);
		searchNetCrossExchange_Controller(inst, auxSol, improveFlag3, 
			auxAccountVec, auxLog);
		searchNetRelocation_Controller(inst, auxSol, improveFlag3, 
			auxAccountVec, auxLog);		
	}
	
	double finalCost = 0;
	for (int i = 0; i < (int)auxSol.Trips.size(); i++)
	{
		 finalCost += inst.CalculateTripCost(auxSol.Trips[i]);
	}

	msg << "SearchCondense Final cost:\t" << std::setprecision(17) << finalCost << "\t" << finalCost - initialCost << endl;
	if (finalCost < initialCost - RELEVANT_INCREASE)
	{
		if (showLog)
		{
			cout << msg.str();
		}
		account += initialCost-finalCost;
		improveFlag = true;
		for (int i = 0; i < auxSol.Trips.size(); i++)
		{
			ptr_newTrips.push_back(&auxSol.Trips[i]);
		}
	}
	else
	{
		cout << msg.str();
	}
}


void distanceComplexityStudy(Instance & inst)
{
	long N =  746390850;
	CPUTimer t;
	t.start();
	double sum = 0.0;
	for (int i = 0; i < N; i++)
	{
		int id1 = 1 + rand() %100000;
		int id2 = 1 + rand() %100000;

		sum += inst.getDistance(id1,id2);
	}
	t.stop();

	cout << N << " distance readings done in " << t.getCPUTotalSecs() << " CPU secs" << endl;
}


void SearchNet1to1Relocation(Instance & inst, Trip & trip1, Trip & trip2, bool & improveFlag, double & account, bool showLog)
{
	double initialCost = inst.CalculateTripCost(trip1) + inst.CalculateTripCost(trip2);

	Trip auxtrip1 = trip1;
	Trip auxtrip2 = trip2;

	Trip besttrip1 = trip1;
	Trip besttrip2 = trip2;
	
	stringstream msg;
	msg << "SearchNet1to1Relocation Initial  cost:\t" << std::setprecision(17) << initialCost << endl;
	
	double bestNet = - RELEVANT_INCREASE;
	double currentNet = 0;

	bool hasSwap = true;
	while ((hasSwap) & (auxtrip1.getSize() > 0))
	{
		double bestMoveNet = 1e10;
		double reverseDirection = false;
		hasSwap = false;
		int best_i = 0;
		int best_j = 0;
		
		//for every gift i of first trip
		for (int i = 0; i < (int)auxtrip1.getSize(); i++)
		{			
			//if it fits in the second trip
			if (auxtrip1.getWeight(i) + auxtrip2.getWeight() <= inst.weightLimit)
			{
				//calculate the gain of erasing gift i from the first trip
				double eraseGain = auxtrip1.evaluateEraseGift(inst, i);

				//for every insertion position j of trip 2
				for (int j = 0; j <= (int)auxtrip2.getSize(); j++)
				{					
					//calculate the loss of insert gift i from the first trip on position j of second trip
					double insertLoss = auxtrip2.evaluateInsertion(inst, j, auxtrip1.getGift(i));

					//calculate the net gain/loss of the movement
					double moveNet = eraseGain + insertLoss;

					//set as movement to be made, if is current best
					if (moveNet < bestMoveNet - RELEVANT_INCREASE)
					{
						bestMoveNet = moveNet;
						best_i = i;
						best_j = j;
						hasSwap = true;
					}						
				}
			}
		}

		if (hasSwap)
		{
			auxtrip2.insertGift(inst, best_j, auxtrip1.getGift(best_i));		
			auxtrip1.eraseGift(inst, best_i);	
			currentNet += bestMoveNet;
		}
		if (currentNet < bestNet)
		{
			besttrip1 = auxtrip1;
			besttrip2 = auxtrip2;
		}
	}
	
	double finalCost = inst.CalculateTripCost(besttrip1) + inst.CalculateTripCost(besttrip2);
	msg << "SearchNet1to1Relocation Final cost:\t" << std::setprecision(17) << finalCost << endl;
	if (finalCost < initialCost - RELEVANT_INCREASE)
	{
		if (showLog)
		{
			cout << msg.str();
		}
		trip1 = besttrip1;
		trip2 = besttrip2;

		account += initialCost-finalCost;	
		
		trip1.resetNeighborhoodStatuses();
		trip2.resetNeighborhoodStatuses();

		improveFlag = true;
	}
}

void SearchNetRelocation(Instance & inst, Trip & tripFrom, vector<Trip*>& ptr_kCloseTrips, bool & improveFlag, double & account, bool showLog)
{	
	Trip auxtripFrom = tripFrom;
	Trip besttripFrom = tripFrom;

	vector<Trip> aux_kCloseTrips;
	vector<Trip> best_kCloseTrips;

	double initialCost = inst.CalculateTripCost(tripFrom);
	for (int t = 0; t < (int)ptr_kCloseTrips.size(); t++)
	{
		initialCost += inst.CalculateTripCost(*ptr_kCloseTrips[t]);
		aux_kCloseTrips.push_back(*ptr_kCloseTrips[t]);
		best_kCloseTrips.push_back(*ptr_kCloseTrips[t]);
	}
		
	stringstream msg;
	msg << "SearchNetRelocation Initial  cost:\t" << std::setprecision(17) << initialCost << endl;
	
	double bestNet = - RELEVANT_INCREASE;
	double currentNet = 0;

	bool hasSwap = true;
	while ((hasSwap) & (auxtripFrom.getSize() > 0))
	{
		double bestMoveNet = 1e10;
		double lastBestMoveNet = 1e10;
		
		hasSwap = false; 
		
		//trip where gift is going
		int best_t = 0; 

		//position where gift is going
		int best_j = 0;
		
		int best_i  = 0;
#ifdef FULL_NET_RELOCATION
		double bestEraseGain = 1000;
		for (int i = 0; i < auxtripFrom.getSize(); i++)
		{
			double eraseGain = auxtripFrom.evaluateEraseGift(inst, best_i);

			if (eraseGain < bestEraseGain - RELEVANT_INCREASE)
			{
				bestEraseGain = eraseGain;
				best_i = i;
			}
		}
#else		
		best_i = auxtripFrom.getSize()-1;
#endif

		//calculate the gain of erasing last gift of tripFrom
		double eraseGain = auxtripFrom.evaluateEraseGift(inst, best_i);

		//for every possible destiny trip
#ifdef PARALLEL_NET_RELOCATION
		#pragma omp parallel for
#endif
		for (int t = 0; t < (int)aux_kCloseTrips.size(); t++)
		{
			//if it fits in the second trip
			if (auxtripFrom.getWeight(best_i) + aux_kCloseTrips[t].getWeight() <= inst.weightLimit)
			{
				//for every insertion position j on trip t
				for (int j = 0; j <= (int)aux_kCloseTrips[t].getSize(); j++)
				{					
					//calculate the loss of insert gift i from the first trip on position j of second trip
					double insertLoss = aux_kCloseTrips[t].evaluateInsertion( inst, j, auxtripFrom.getGift(best_i) );

					//calculate the net gain/loss of the movement
					double moveNet = eraseGain + insertLoss;

					if (moveNet < lastBestMoveNet - RELEVANT_INCREASE)
					{
						#pragma omp critical(really_check)
						{
							//set as movement to be made, if is current best
							if (moveNet < bestMoveNet - RELEVANT_INCREASE)
							{
								bestMoveNet = moveNet;
								lastBestMoveNet = moveNet;
								best_t = t;
								best_j = j;
								hasSwap = true;
							}			
						}
					}
				}
			}
		}					

		if (hasSwap)
		{
			aux_kCloseTrips[best_t].insertGift(inst, best_j, auxtripFrom.getGift(best_i) );		
			auxtripFrom.eraseGift(inst, best_i);	
			currentNet += bestMoveNet;
		}
		if (currentNet < bestNet)
		{
			besttripFrom = auxtripFrom;
			for (int t = 0; t < (int)ptr_kCloseTrips.size(); t++)
			{
				best_kCloseTrips[t] = aux_kCloseTrips[t];
			}
		}
	}
	
	double finalCost = inst.CalculateTripCost(besttripFrom);
	for (int t = 0; t < (int)best_kCloseTrips.size(); t++)
	{
		finalCost += inst.CalculateTripCost(best_kCloseTrips[t]);
	}

	msg << "SearchNetRelocation Final cost:\t" << std::setprecision(17) << finalCost << endl;
	if (finalCost < initialCost - RELEVANT_INCREASE)
	{
		if (showLog)
		{
			cout << msg.str();
		}
		tripFrom = besttripFrom;
		tripFrom.resetNeighborhoodStatuses();
		for (int t = 0; t < (int)best_kCloseTrips.size(); t++)
		{
			*ptr_kCloseTrips[t] = best_kCloseTrips[t];
			ptr_kCloseTrips[t]->resetNeighborhoodStatuses();
		}

		account += initialCost-finalCost;	
		
		improveFlag = true;
	}
}

void MoveMultiExchange(Instance& inst, Solution& sol,vector<GiftChain>& exchangePath, double costDiff)
{
	//cout << "Performing MultiExchange move, ExchangePath:" << endl;
	//printExchangePath(exchangePath);

	/* cout << "Initial Routes:" << endl;
	for (int i = 0; i < (int)exchangePath.size(); i++)
	{
		cout << i << ": " << sol.Trips[exchangePath[i].get_k()].printTrip() << endl;
	} */

	//exchange involves exchangePath size - 1 trips
	//bagsOfGifts[i] stores gifts to enter route i
	vector<vector<Gift>> bagsOfGifts(exchangePath.size()-1);
	vector<int> routeIndex(exchangePath.size()-1);
	vector<int> routeInsertionIndex(exchangePath.size()-1);		

	for (int i = 0; i < (int)exchangePath.size()-1; i++)
	{
		routeIndex[i] = exchangePath[i].get_k();
		routeInsertionIndex[i] = exchangePath[i].get_l1() + 1;

		int k = exchangePath[i+1].get_k();
		int l1  = exchangePath[i+1].get_l1();
		int l2  = exchangePath[i+1].get_l2();

		for (int g = l1+1; g <= l2; g++)
		{
			bagsOfGifts[i].push_back( sol.Trips[k].getGift(l1+1) );
			sol.Trips[k].eraseGift(inst,l1+1, false);
		}
	}

	for (int i = 0; i < (int)routeIndex.size(); i++)
	{
		for (int g = bagsOfGifts[i].size()-1; g > -1; g--)
		{
			sol.Trips[routeIndex[i]].insertGift(inst, routeInsertionIndex[i], bagsOfGifts[i][g], false);
		}
	}
	
	double trueDiff = 0.0;
	for (int i = 0; i < (int)exchangePath.size(); i++)
	{		
		trueDiff -= sol.Trips[exchangePath[i].get_k()].getCost();
		sol.Trips[exchangePath[i].get_k()].UpdateAuxiliaryDataStructures(inst);
		sol.Trips[exchangePath[i].get_k()].resetNeighborhoodStatuses();
		trueDiff += sol.Trips[exchangePath[i].get_k()].getCost();
	}

	/* cout << "Final Routes:" << endl;
	for (int i = 0; i < (int)exchangePath.size(); i++)
	{
		cout << i << ": " << sol.Trips[exchangePath[i].get_k()].printTrip() << endl;
	} */
	if (fabs(trueDiff - costDiff) > 0.1)
		cout << "ERROR: cost difference of " << trueDiff << " differs from expected " << costDiff << endl;
}

void SearchMultiExchange(Instance &inst, Solution& sol, ExchangeGraph& solExchangeGraph, vector<vector<GiftChain>>& exchangePaths, vector<double>& costDiffs, ofstream& log, double & account)
{
	bool hasNegativeCostPath = true;
	int currTrip = 0;
	vector<double> bestCostsE(solExchangeGraph.V.size(), 1e15);
	vector<int> previousE(solExchangeGraph.V.size(), -1);
	vector<double> bestCostsW(solExchangeGraph.V.size(), 1e15);
	vector<int> previousW(solExchangeGraph.V.size(), -1);

	while (hasNegativeCostPath)
	{
		hasNegativeCostPath = false;

		double bestPathCost = 1e15;
		vector<int> bestPath;

		// process the vertices from the current trip on
		for (int k = currTrip; k < (int)sol.Trips.size(); k++)
		{
			// check if any negative reduced cost path is present
			for (int i = solExchangeGraph.tripStarts[k]; i < solExchangeGraph.tripStarts[k+1]; i++)
			{
				double checkCost = bestCostsE[i] + sol.Trips[k].getWeightAt(solExchangeGraph.V[i].get_l2()) *
						inst.getDistance(sol.Trips[k].getGiftId(solExchangeGraph.V[i].get_l1()),
						sol.Trips[k].getGiftId(solExchangeGraph.V[i].get_l2()+1));
				if ( (checkCost < -RELEVANT_INCREASE) && (checkCost < bestPathCost) )
				{
					bestPath.clear();
					int j = i;
					while (j != -1) 
					{
						bestPath.push_back(j);
						j = previousE[j];
					}
					for (int l = 0; l < (int(bestPath.size())/2); l++)
					{
						j = bestPath[l];
						bestPath[l] = bestPath[bestPath.size() - l - 1];
						bestPath[bestPath.size() - l - 1] = j;
					}
					bestPathCost = checkCost;
				}
				if (solExchangeGraph.V[i].get_l1() == solExchangeGraph.V[i].get_l2())
				{
					checkCost = bestCostsW[i] - sol.Trips[k].getWeightAt(solExchangeGraph.V[i].get_l1()) *
							inst.getDistance(sol.Trips[k].getGiftId(solExchangeGraph.V[i].get_l1()),
							sol.Trips[k].getGiftId(solExchangeGraph.V[i].get_l1()+1));
					if ( (checkCost < -RELEVANT_INCREASE) && (checkCost < bestPathCost) )
					{
						bestPath.clear();
						int j = i;
						while (j != -1) 
						{
							bestPath.push_back(j);
							j = previousW[j];
						}
						bestPathCost = checkCost;
					}
				}
			}

			//builds shortest path and reset the dynamic programing table
			if (bestPathCost < - RELEVANT_INCREASE)
			{
				log << "MultiExchange: found a move path improving " << bestPathCost << " until trip " << k << endl;
				vector<GiftChain> auxGiftChain;
				for (int i = 0; i < (int)bestPath.size(); i++)
					auxGiftChain.push_back(solExchangeGraph.V[bestPath[i]]);				
				exchangePaths.push_back(auxGiftChain);
				costDiffs.push_back(bestPathCost);

				bestCostsE.clear();
				bestCostsE.resize(solExchangeGraph.V.size(), 1e15);
				previousE.clear();
				previousE.resize(solExchangeGraph.V.size(), -1);
				bestCostsW.clear();
				bestCostsW.resize(solExchangeGraph.V.size(), 1e15);
				previousW.clear();
				previousW.resize(solExchangeGraph.V.size(), -1);

				hasNegativeCostPath = true;
				currTrip = k+1;
				break;
			}
			log << "MultiExchange: found no improving move path until trip " << k << endl;

			// extend the best paths of the current trip to the next ones
			for (int i = solExchangeGraph.tripStarts[k]; i < solExchangeGraph.tripStarts[k+1]; i++)
			{
				// update if the best is to start a path here
				double initialCost;
				if (solExchangeGraph.V[i].get_l1() == solExchangeGraph.V[i].get_l2())
				{
					initialCost = -sol.Trips[k].getWeightAt(solExchangeGraph.V[i].get_l1()) *
							inst.getDistance(sol.Trips[k].getGiftId(solExchangeGraph.V[i].get_l1()),
							sol.Trips[k].getGiftId(solExchangeGraph.V[i].get_l1()+1));
					if (bestCostsE[i] > initialCost)
					{
						bestCostsE[i] = initialCost;
						previousE[i] = -1;
					}
				}
				initialCost = sol.Trips[k].getWeightAt(solExchangeGraph.V[i].get_l2()) *
						inst.getDistance(sol.Trips[k].getGiftId(solExchangeGraph.V[i].get_l1()),
						sol.Trips[k].getGiftId(solExchangeGraph.V[i].get_l2()+1));
				if (bestCostsW[i] > initialCost)
				{
					bestCostsW[i] = initialCost;
					previousW[i] = -1;
				}

				// extend the current vertex to all its successors
				for (int j = 0; j < solExchangeGraph.V[i].getEastDegree(); j++)
				{
					int l = solExchangeGraph.V[i].getEastArc(j).getTail();
					if (bestCostsE[l] > bestCostsE[i] + solExchangeGraph.V[i].getEastArc(j).getCost())
					{
						bestCostsE[l] = bestCostsE[i] + solExchangeGraph.V[i].getEastArc(j).getCost();
						previousE[l] = i;
					}
				}
				for (int j = 0; j < solExchangeGraph.V[i].getWestDegree(); j++)
				{
					int l = solExchangeGraph.V[i].getWestArc(j).getTail();
					if (bestCostsW[l] > bestCostsW[i] + solExchangeGraph.V[i].getWestArc(j).getCost())
					{
						bestCostsW[l] = bestCostsW[i] + solExchangeGraph.V[i].getWestArc(j).getCost();
						previousW[l] = i;
					}
				}
			}
		}
	}

	for (int i = 0; i < (int)costDiffs.size(); i++)
	{
		account -= costDiffs[i];
	}
}

void printExchangePath(vector<GiftChain> exchangePath)
{	
	cout << exchangePath[0].print();
	for (int i = 1; i < (int)exchangePath.size(); i++)
	{
		cout << "->" << exchangePath[i].print();
	}
	cout << endl;
}


void searchBiCrossExchange(Instance & inst, Trip & trip1, Trip & trip2, bool & improveFlag, double & account, bool showLog)
{	
	double initialCost = inst.CalculateTripCost(trip1) + inst.CalculateTripCost(trip2);
	
	stringstream msg;
	msg << "searchBiCrossExchange Initial  cost:\t" << std::setprecision(17) << initialCost << endl;

	bool hasSwap = true;
	while (hasSwap)
	{
		double bestIncrease = 0;
		int best_i = 0;
		int best_j = 0;
		int best_k = 0;
		int best_l = 0;
		int best_m = 0;
		int best_n = 0;
		int best_o = 0;
		int best_p = 0;
		int best_sense1 = 0;
		int best_sense2 = 0;
		hasSwap = false;
		for (int i = -1; i < (int)trip1.getSize(); i++)
		{
			for (int k = i; k < (int)trip1.getSize(); k++)
			{	
				for (int m = k; m < (int)trip1.getSize(); m++)
				{					
					for (int o = m; o < (int)trip1.getSize(); o++)
					{
						for (int j = -1; j < (int)trip2.getSize(); j++)
						{
							for (int l = j; l < (int)trip2.getSize(); l++)
							{	
								for (int n = l; n < (int)trip2.getSize(); n++)
								{					
									for (int p = n; p < (int)trip2.getSize(); p++)
									{
										int sense1 = 0;
										int sense2 = 0;
										double increase =  evaluateBiCrossExchange(inst, trip1, trip2, i, k, m, o, j, l, n, p, sense1, sense2);
						
										if (increase < bestIncrease - RELEVANT_INCREASE)
										{
											bestIncrease = increase;
											hasSwap = true;
											improveFlag = true;
											best_i = i;
											best_k= k;
											best_m = m;
											best_o= o;

											best_j = j;
											best_l = l;
											best_n = n;
											best_p = p;

											best_sense1 = sense1;
											best_sense2 = sense2;
										}											
									}				
								}
							}
						}										
					}				
				}
			}
		}
		if (hasSwap)
		{			
			int sense1 = 0;
			int sense2 = 0;
			cout << evaluateBiCrossExchange(inst, trip1, trip2, best_i, best_k, best_m, best_o, best_j, best_l, best_n, best_p, sense1, sense2) << endl;
			cout << trip1.printTrip() << "\t" << trip1.getCost() << endl << endl; 
			cout << trip2.printTrip() << "\t" << trip2.getCost() << endl << endl;  
			double oldCost = inst.CalculateTripCost(trip1) + inst.CalculateTripCost(trip2);
			moveBiCrossExchange(inst, trip1, trip2, best_i, best_k, best_m, best_o, best_j, best_l, best_n, best_p, best_sense1, best_sense2);
			double newCost = inst.CalculateTripCost(trip1) + inst.CalculateTripCost(trip2);
			//cout << trip1.printTrip() << "\t" << trip1.getCost() << endl << endl;  
			//cout << trip2.printTrip() << "\t" << trip2.getCost() << endl << endl;   
			if (abs(newCost-oldCost-bestIncrease) > 0.01)
			{
				cout << "Error on BiCrossExchanging trips"<< endl;
				cout << "Expected Increase\t" << bestIncrease << endl;
				cout << trip1.printTrip() << "\t" << trip1.getCost() << endl; 
				cout << trip2.printTrip() << "\t" << trip2.getCost() << endl; 
				cout << best_i  << "\t" << best_k  << "\t" << best_j  << "\t" << best_l << "\t" << best_sense1 << endl;
				cout << best_m  << "\t" << best_o  << "\t" << best_n  << "\t" << best_p << "\t" << best_sense1 << endl;
				cout << newCost << endl;				
				cout << oldCost << endl;
				cout << newCost - oldCost << "\t" << bestIncrease << endl;
				
				throw -1;
			}
		}
	}
	
	double finalCost = inst.CalculateTripCost(trip1) + inst.CalculateTripCost(trip2);
	msg << "searchCrossExchange Final cost:\t" << std::setprecision(17) << finalCost << endl;
	if (finalCost < initialCost)
	{
		if (showLog)
		{
			cout << msg.str();
		}
		account += initialCost-finalCost;		
		trip1.resetNeighborhoodStatuses();
		trip2.resetNeighborhoodStatuses();
		//cout << "{Crossexchange made\t" << index1 << "\t" << index2 << "\t" << initialCost - finalCost << endl;
	}
	else
	{		
		//cout << "InterMassCenterDistance\t" << InterMassCenterDistance(inst,trip1,trip2) << "\tNo CrossExchange Improve" << endl;
	}
}

void SearchMerge2Polar(Instance & inst, Trip & trip1, Trip & trip2, bool & improveFlag, double & account, bool & feasible, bool force, bool showLog)
{
	double initialCost = inst.CalculateTripCost(trip1) + inst.CalculateTripCost(trip2);

	stringstream msg;
	msg << "SearchMerge2Polar Initial  cost:\t" << std::setprecision(17) << initialCost << endl;

	//check feasibility
	double polarWeight =  trip1.getPolarWeight() +  trip2.getPolarWeight() + inst.sleighWeight; 
	double otherWeight = trip1.getWeight() + trip2.getWeight() - polarWeight; 

	if ( ( polarWeight > inst.weightLimit) || ( otherWeight > inst.weightLimit )  )
	{
		feasible = false;
		return;
	}
	else
	{
		feasible = true;
	}

	Trip auxTrip1 = trip1;
	Trip auxTrip2 = trip2;

	vector<Gift> bagOfPolarGifts;
	vector<Gift> bagOfOtherGifts;

	//remove all polar gifts from auxTrip1
	for (int i = auxTrip1.getSize()-1; i > -1; i--)
	{		
		Gift auxGift = auxTrip1.getGift(i);
		if (auxGift.isPolar())
		{
			bagOfPolarGifts.push_back(auxGift);
			auxTrip1.eraseGift(inst,i,false);
		}
	}
	//remove all normal gifts from auxTrip2
	for (int i = auxTrip2.getSize()-1; i > -1; i--)
	{		
		Gift auxGift = auxTrip2.getGift(i);
		if (!auxGift.isPolar())
		{
			bagOfOtherGifts.push_back(auxGift);
			auxTrip2.eraseGift(inst,i,false);
		}
	}	

	auxTrip1.UpdateAuxiliaryDataStructures(inst);
	auxTrip2.UpdateAuxiliaryDataStructures(inst);
	
	//send all normal gifts to auxTrip1
	for (int g = 0; g < (int)bagOfOtherGifts.size(); g++)
	{
		int best_i = 0;
		double bestIncrease = 1e15;
		for (int i = 0; i <= auxTrip1.getSize(); i++)
		{
			double increase = auxTrip1.evaluateInsertion(inst,i, bagOfOtherGifts[g]);
			if (increase < bestIncrease - 0.01)
			{
				best_i = i;
				bestIncrease = increase;
			}
		}
		auxTrip1.insertGift(inst,best_i,bagOfOtherGifts[g]);
	}

	//send all polar gifts to auxTrip2
	for (int g = 0; g < (int)bagOfPolarGifts.size(); g++)
	{
		int best_i = 0;
		double bestIncrease = 1e15;
		for (int i = 0; i <= auxTrip2.getSize(); i++)
		{
			double increase = auxTrip2.evaluateInsertion(inst,i, bagOfPolarGifts[g]);
			if (increase < bestIncrease - 0.01)
			{
				best_i = i;
				bestIncrease = increase;
			}
		}
		auxTrip2.insertGift(inst,best_i,bagOfPolarGifts[g]);
	}

	double account_search2opt = 0;
	double account_search3opt = 0;
	double account_searchIntraRelocation = 0;
	double account_searchIntraChainRelocation = 0;
	double account_searchIntraExchange = 0;

	//perform all single trip movements on newTrip
	bool improveFlag2 = true;
	while (improveFlag2)
	{
		improveFlag2 = false;
		//try search2Opt (2Opt)					
		auxTrip1.search2Opt(inst, improveFlag2, account_search2opt, false);

		//try  search3Opt (3Opt)		
		auxTrip1.search3Opt(inst, improveFlag2, account_search3opt, false);

		//try  searchIntraRelocation (relocates a single gift to a new place in the trip)
		auxTrip1.searchIntraRelocation(inst, improveFlag2, account_searchIntraRelocation, false);

		//try searchIntraChainRelocation (relocates a chain of gifts to a new place into the trip		
		auxTrip1.searchIntraChainRelocation(inst, improveFlag2, account_searchIntraChainRelocation, false);
							
		//try searchIntraExchange (intra trip 2 gift swap)
		auxTrip1.searchIntraExchange(inst, improveFlag2, account_searchIntraExchange, false);		
	}
	improveFlag2 = true;
	while (improveFlag2)
	{
		improveFlag2 = false;
		//try search2Opt (2Opt)					
		auxTrip2.search2Opt(inst, improveFlag2, account_search2opt, false);

		//try  search3Opt (3Opt)		
		auxTrip2.search3Opt(inst, improveFlag2, account_search3opt, false);

		//try  searchIntraRelocation (relocates a single gift to a new place in the trip)
		auxTrip2.searchIntraRelocation(inst, improveFlag2, account_searchIntraRelocation, false);

		//try searchIntraChainRelocation (relocates a chain of gifts to a new place into the trip		
		auxTrip2.searchIntraChainRelocation(inst, improveFlag2, account_searchIntraChainRelocation, false);
							
		//try searchIntraExchange (intra trip 2 gift swap)
		auxTrip2.searchIntraExchange(inst, improveFlag2, account_searchIntraExchange, false);		
	}

	double finalCost = inst.CalculateTripCost(auxTrip1) + inst.CalculateTripCost(auxTrip2);
	msg << "SearchMerge2Polar Final cost:\t" << std::setprecision(17) << finalCost << endl;
	if (finalCost < initialCost - RELEVANT_INCREASE)
	{
		if (showLog)
		{
			cout << msg.str();
		}
		account += initialCost-finalCost;
		improveFlag = true;

		trip1 = auxTrip1;
		trip2 = auxTrip2;
		trip1.resetNeighborhoodStatuses();
		trip2.resetNeighborhoodStatuses();
	}
	else
	{
		if (force)
		{
			trip1 = auxTrip1;
			trip2 = auxTrip2;
			auxTrip1.resetNeighborhoodStatuses();
			auxTrip2.resetNeighborhoodStatuses();
		}
	}
}

void PerturbRandomCrossExchange(Instance& inst, Solution& sol, int cxLevel, bool& hasPerturbation)
{
	int cxCount = 0;
	while (cxCount < cxLevel)
	{
		int t1 = rand() % (sol.Trips.size()+1);

		if (t1 < (int)sol.Trips.size())
		{
			bool hasCX = false;

			while (!hasCX)
			{
				int t2 = rand() % sol.Trips.size();	
				while (t2 == t1)
				{
					t2= rand() % sol.Trips.size();
				}
				int i = -1 + (rand() % sol.Trips[t1].getSize());
				int j = -1 + (rand() % sol.Trips[t2].getSize());

				int k = i + (rand() % (sol.Trips[t1].getSize() - i));										
				int l = j + (rand() % (sol.Trips[t2].getSize() - j));								

				double weightChain1 = sol.Trips[t1].getWeightAt(i) - sol.Trips[t1].getWeightAt(k);
				double weightChain2 = sol.Trips[t2].getWeightAt(j) - sol.Trips[t2].getWeightAt(l);

				if ( (sol.Trips[t1].getWeight() - weightChain1 + weightChain2 <= inst.weightLimit) &&
					(sol.Trips[t2].getWeight() - weightChain2 + weightChain1 <= inst.weightLimit) )
				{
					moveCrossExchange(inst, sol.Trips[t1], sol.Trips[t2], i, k, j, l, 0); 		
					sol.Trips[t1].resetNeighborhoodStatuses();
					sol.Trips[t2].resetNeighborhoodStatuses();
					hasCX = true;
					cxCount++;
				}				
			}								
		}
		else
		{
			Trip newTrip(inst);
								
			int t2 = rand() % sol.Trips.size();	
			int j = -1 + (rand() % sol.Trips[t2].getSize());								
			int l = j + (rand() % (sol.Trips[t2].getSize() - j));
						
									
			moveCrossExchange(inst, newTrip, sol.Trips[t2], -1, -1, j, l, 0); 		
			sol.Trips.push_back(newTrip);
			sol.Trips[t2].resetNeighborhoodStatuses();
			cxCount++;
		}
		sol.ClearEmptyTrips();	
	}		
	
	hasPerturbation = true;
}

void PerturbExplode(Instance& inst, Solution& sol, bool& hasPerturbation)
{
	int t = rand()%sol.Trips.size();
	for (int i = 0; i < sol.Trips[t].getSize(); i++)
	{
		sol.Trips.push_back(Trip(inst, sol.Trips[t].getGift(i)));
	}
	sol.Trips.erase(sol.Trips.begin()+t);
	hasPerturbation = true;
}

void PerturbMerge2Polar(Instance& inst, Solution& sol, bool& hasPerturbation)
{
	bool auxImproveFlag = false;
	double auxSearchAccount = 0;

	vector<int> index_i;
	vector<int> index_j;
	for (int i = 0; i < (int)sol.Trips.size(); i++)
	{
		index_i.push_back(i);
		index_j.push_back(i);
	}
	random_shuffle(index_i.begin(), index_i.end());
	random_shuffle(index_j.begin(), index_j.end());

	bool foundFeasible = false;
	for (int i = 0; i < (int)index_i.size(); i++)
	{										
		for (int j = 0; j < (int)index_j.size(); j++)
		{
			if (index_j[j] > index_i[i])
			{
				if ( (sol.Trips[index_i[i]].isPolar()) && (sol.Trips[index_j[j]].isPolar()) )
				{
					SearchMerge2Polar(inst, sol.Trips[index_i[i]], sol.Trips[index_j[j]], auxImproveFlag, auxSearchAccount, foundFeasible, true);
					if (foundFeasible)
					{
						break;
					}
				}
			}
		}
		if (foundFeasible)
		{
			break;
		}
	}

	hasPerturbation = foundFeasible;	
}

void PerturbMerge2(Instance& inst, Solution& sol, bool& hasPerturbation)
{
	sol.SortMCLongitude();
	double has2Merge = false;

	vector<int> merge_i;
	vector<int> merge_j;

	for (int i = 0; i < (int)sol.Trips.size()-2; i++)
	{
		for (int t1 = 1; t1 <= searchMerge2_reach; t1++)
		{
			int j = i + t1;
			if (j >= (int)sol.Trips.size()) //looped around
			{
				break;
			}									
			if (sol.Trips[i].getWeight() + sol.Trips[j].getWeight() 
				- inst.sleighWeight <= inst.weightLimit)
			{
				merge_i.push_back(i);	
				merge_j.push_back(j);	
				has2Merge = true;
				break;
			}	
		}
	}

	if (has2Merge)
	{
		int index = rand()% merge_i.size();
		Trip newTrip(inst);

		for (int l = 0; l < sol.Trips[ merge_i[index] ].getSize(); l++)
		{
			newTrip.appendGift(inst, sol.Trips[ merge_i[index] ].getGift(l));
		}
		for (int l = 0; l < sol.Trips[ merge_j[index] ].getSize(); l++)
		{
			newTrip.appendGift(inst, sol.Trips[ merge_j[index] ].getGift(l));
		}						

		vector<int> indexes;
		indexes.push_back(merge_i[index]);
		indexes.push_back(merge_j[index]);
		sort(indexes.begin(), indexes.end());

		sol.Trips.erase(sol.Trips.begin() + indexes[1]);	
		sol.Trips.erase(sol.Trips.begin() + indexes[0]);											

		sol.Trips.push_back(newTrip);	
		hasPerturbation = true;
	}		
	else
	{
		hasPerturbation = false;
	}	
}

void PerturbMerge3(Instance& inst, Solution& sol, bool& hasPerturbation)
{
	sol.SortMCLongitude();
	double has3Merge = false;

	vector<int> merge_i;
	vector<int> merge_j;
	vector<int> merge_k;

	for (int i = 0; i < (int)sol.Trips.size()-2; i++)
	{
		for (int t1 = 1; t1 <= searchMerge2_reach; t1++)
		{
			int j = i + t1;
			if (j >= (int)sol.Trips.size()) //looped around
			{
				break;
			}
			for (int t2 = 1; t2 <= searchMerge2_reach; t2++)
			{
				int k = j + t2;
				if (k >= (int)sol.Trips.size())  //looped around
				{
					break;
				}
				if (sol.Trips[i].getWeight() + sol.Trips[j].getWeight() 
					+ sol.Trips[k].getWeight() - 2*inst.sleighWeight <= inst.weightLimit)
				{
					merge_i.push_back(i);	
					merge_j.push_back(j);	
					merge_k.push_back(k);	
					has3Merge = true;
					break;
				}										
			}
		}
	}

	if (has3Merge)
	{
		int index = rand()% merge_i.size();
		Trip newTrip(inst);

		for (int l = 0; l < sol.Trips[ merge_i[index] ].getSize(); l++)
		{
			newTrip.appendGift(inst, sol.Trips[ merge_i[index] ].getGift(l));
		}
		for (int l = 0; l < sol.Trips[ merge_j[index] ].getSize(); l++)
		{
			newTrip.appendGift(inst, sol.Trips[ merge_j[index] ].getGift(l));
		}
		for (int l = 0; l < sol.Trips[ merge_k[index] ].getSize(); l++)
		{
			newTrip.appendGift(inst, sol.Trips[ merge_k[index] ].getGift(l));
		}						

		vector<int> indexes;
		indexes.push_back(merge_i[index]);
		indexes.push_back(merge_j[index]);
		indexes.push_back(merge_k[index]);
		sort(indexes.begin(), indexes.end());

		sol.Trips.erase(sol.Trips.begin() + indexes[2]);	
		sol.Trips.erase(sol.Trips.begin() + indexes[1]);	
		sol.Trips.erase(sol.Trips.begin() + indexes[0]);											

		sol.Trips.push_back(newTrip);	
		hasPerturbation = true;
	}		
	else
	{
		hasPerturbation = false;
	}
}

//
//void PerturbCondense3(Instance& inst, Solution& sol, bool& hasPerturbation)
//{
//	sol.SortMCLongitude();
//	double has3Merge = false;
//
//	vector<int> merge_i;
//	vector<int> merge_j;
//	vector<int> merge_k;
//
//	for (int i = 0; i < (int)sol.Trips.size()-2; i++)
//	{
//		for (int t1 = 1; t1 <= searchMerge2_reach; t1++)
//		{
//			int j = i + t1;
//			if (j >= (int)sol.Trips.size()) //looped around
//			{
//				break;
//			}
//			for (int t2 = 1; t2 <= searchMerge2_reach; t2++)
//			{
//				int k = j + t2;
//				if (k >= (int)sol.Trips.size())  //looped around
//				{
//					break;
//				}
//				if (sol.Trips[i].getWeight() + sol.Trips[j].getWeight() 
//					+ sol.Trips[k].getWeight() - 2*inst.sleighWeight <= inst.weightLimit)
//				{
//					merge_i.push_back(i);	
//					merge_j.push_back(j);	
//					merge_k.push_back(k);	
//					has3Merge = true;
//					break;
//				}										
//			}
//		}
//	}
//
//	if (has3Merge)
//	{
//		int index = rand()% merge_i.size();
//		Trip newTrip(inst);
//
//		for (int l = 0; l < sol.Trips[ merge_i[index] ].getSize(); l++)
//		{
//			newTrip.appendGift(inst, sol.Trips[ merge_i[index] ].getGift(l));
//		}
//		for (int l = 0; l < sol.Trips[ merge_j[index] ].getSize(); l++)
//		{
//			newTrip.appendGift(inst, sol.Trips[ merge_j[index] ].getGift(l));
//		}
//		for (int l = 0; l < sol.Trips[ merge_k[index] ].getSize(); l++)
//		{
//			newTrip.appendGift(inst, sol.Trips[ merge_k[index] ].getGift(l));
//		}						
//
//		vector<int> indexes;
//		indexes.push_back(merge_i[index]);
//		indexes.push_back(merge_j[index]);
//		indexes.push_back(merge_k[index]);
//		sort(indexes.begin(), indexes.end());
//
//		sol.Trips.erase(sol.Trips.begin() + indexes[2]);	
//		sol.Trips.erase(sol.Trips.begin() + indexes[1]);	
//		sol.Trips.erase(sol.Trips.begin() + indexes[0]);											
//
//		sol.Trips.push_back(newTrip);	
//		hasPerturbation = true;
//	}		
//	else
//	{
//		hasPerturbation = false;
//	}
//}


void PerturbHoldSP(Instance& inst, Solution& sol, bool& hasPerturbation)
{	
	double weightLimit = 1 + rand()%1000;

	bool hasGift = true;								
	Trip newTrip(inst);

	while (hasGift)
	{
		hasGift = false;
		int best_i = 0;
		int best_t = 0;
		double lowestLatitude = -58;
		for (int t = 0; t < (int)sol.Trips.size(); t++)
		{
			for (int i = 0; i < sol.Trips[t].getSize(); i++)
			{
				double auxLatitude = sol.Trips[t].getGiftLatitude(i);
				if ( (auxLatitude < lowestLatitude) && ( newTrip.getWeight() + sol.Trips[t].getWeight(i) <= weightLimit) )
				{
					lowestLatitude = auxLatitude;
					best_i = i;
					best_t = t;
					hasGift = true;
				}
			}
		}									

		if (hasGift)
		{
			Gift auxGift = sol.Trips[best_t].getGift(best_i);
			sol.Trips[best_t].eraseGift(inst, best_i);

			newTrip.appendGift(inst, auxGift);
			sol.Trips[best_t].resetNeighborhoodStatuses();
			hasPerturbation = true;
		}					
	}
				
	if (hasPerturbation)
	{
		sol.Trips.push_back(newTrip);	
	}
}

void PerturbRandomShift(Instance& inst, Solution& sol, bool& hasPerturbation)
{		
	int failCount = 0;

	while ( (!hasPerturbation) && (failCount < 1e4) )
	{
		failCount++;
		int t1 = rand()%sol.Trips.size();
		int t2 = rand()%sol.Trips.size();
		if (t1 != t2)
		{
			int i = rand()%sol.Trips[t1].getSize();
			int j = rand()%(sol.Trips[t2].getSize()+1);

			Gift auxGift = sol.Trips[t1].getGift(i);

			if (sol.Trips[t2].getWeight() + auxGift.weight <= inst.weightLimit)
			{
				sol.Trips[t1].eraseGift(inst,i);
				sol.Trips[t2].insertGift(inst, j, auxGift);
	
				sol.Trips[t1].resetNeighborhoodStatuses();
				sol.Trips[t2].resetNeighborhoodStatuses();
				hasPerturbation = true;
			}	
		}	
	}
}

void PerturbRandomExchange(Instance& inst, Solution& sol, bool& hasPerturbation)
{
	int failCount = 0;

	while ( (!hasPerturbation) && (failCount < 1e4) )
	{
		failCount++;
		int t1 = rand()%sol.Trips.size();
		int t2 = rand()%sol.Trips.size();

		if (t2 != t1)
		{
			int i = rand()%sol.Trips[t1].getSize();
			int j = rand()%sol.Trips[t2].getSize();

			Gift auxGift1 = sol.Trips[t1].getGift(i);
			Gift auxGift2 = sol.Trips[t2].getGift(j);

			if ( (sol.Trips[t1].getWeight() + auxGift2.weight <= inst.weightLimit) && 
				(sol.Trips[t2].getWeight() + auxGift1.weight <= inst.weightLimit) )
			{
				sol.Trips[t1].eraseGift(inst,i);
				sol.Trips[t1].insertGift(inst, i, auxGift2);

				sol.Trips[t2].eraseGift(inst,j);
				sol.Trips[t2].insertGift(inst, j, auxGift1);
	
				sol.Trips[t1].resetNeighborhoodStatuses();
				sol.Trips[t2].resetNeighborhoodStatuses();
				hasPerturbation = true;
			}	
		}	
	}	
}

void PerturbSplitLongest(Instance& inst, Solution& sol, bool& hasPerturbation)
{
	int t = rand()%sol.Trips.size();
	
	Trip newTrip(inst);
						
	double longestArc = 0;
	int best_i = 1;
	//find splitTrip longestArc
	for (int i = 1; i < sol.Trips[t].getSize()-1; i++)
	{
		double arcLenght = sol.Trips[t].getDistanceFrom(i);
		if (arcLenght > longestArc)
		{
			longestArc = arcLenght;
			best_i = i;
		}
	}		

	int tripSize = sol.Trips[t].getSize();
	for (int i = best_i+1; i < tripSize; i++)
	{
		newTrip.appendGift(inst, sol.Trips[t].getGift(best_i+1));
		sol.Trips[t].eraseGift(inst,best_i+1, false);
	}
	sol.Trips[t].UpdateAuxiliaryDataStructures(inst);
	sol.Trips[t].resetNeighborhoodStatuses();

	newTrip.setTabu();
	sol.Trips.push_back(newTrip);	

	hasPerturbation = true;
}

void PerturbSplitCostlier(Instance& inst, Solution& sol, bool& hasPerturbation)
{
	int t = rand()%sol.Trips.size();
	
	Trip newTrip(inst);
						
	double costlierArc = 0;
	int best_i = 1;
	//find splitTrip longestArc
	for (int i = 1; i < sol.Trips[t].getSize()-1; i++)
	{
		double arcCost = sol.Trips[t].getDistanceFrom(i)*sol.Trips[t].getWeight(i);
		if (arcCost > costlierArc)
		{
			costlierArc = arcCost;
			best_i = i;
		}
	}		

	int tripSize = sol.Trips[t].getSize();
	for (int i = best_i+1; i < tripSize; i++)
	{
		newTrip.appendGift(inst, sol.Trips[t].getGift(best_i+1));
		sol.Trips[t].eraseGift(inst,best_i+1, false);
	}
	sol.Trips[t].UpdateAuxiliaryDataStructures(inst);
	sol.Trips[t].resetNeighborhoodStatuses();

	newTrip.setTabu();
	sol.Trips.push_back(newTrip);	

	hasPerturbation = true;
}

void searchNetRelocation_Controller(Instance& inst, Solution& sol, bool& overallImproveFlag, vector<double>& searchAccount, ofstream& log)
{
	bool loopAgain = true;
	int reach = searchNetRelocation_reach;
	if (reach > sol.Trips.size()/2)
	{
		reach = sol.Trips.size()/2;
	}

	while (loopAgain)
	{
		loopAgain = false;
		bool improveFlag = false;

		sol.SortMCLongitude();
		vector<bool> optimized(sol.Trips.size());
		for (int i = 0; i < (int)optimized.size(); i++)
		{
			optimized[i] = sol.Trips[i].getNeighborhoodStatus(NEIGHBORHOOD_NetRelocation);
			sol.Trips[i].setNeighborhoodCleared(NEIGHBORHOOD_NetRelocation);
		}
		for (int i = 0; i < (int)sol.Trips.size(); i++)
		{
			log << "Inter SearchNetRelocation trip " << i+1 << "/" << sol.Trips.size() << endl;
			log << "Current Cost: " << std::setprecision(17) << inst.EvaluateSol(sol) << endl;					
								
			bool shoudSearch = false;

			for (int t = -reach; t <= reach; t++) //look at the k nearest trips
			{	
				int j = i+t;
				if ( ( j != i ) && ( j >= 0 ) && ( j < (int)sol.Trips.size() ) ) 
				{										
					if ( ( i >= (int)optimized.size() ) || ( !optimized[i] ) || ( j >= (int)optimized.size() ) || ( !optimized[j] ) )
					{
						shoudSearch = true;
						break;
					}
				}
			}
							
			if (shoudSearch)
			{
				vector<Trip*> ptr_kCloseTrips;
				for (int t = -searchNetRelocation_reach; t <= searchNetRelocation_reach; t++) //look at the k nearest trips
				{	
					int j = i+t;
					if ( ( j != i ) && ( j >= 0 ) && ( j < (int)sol.Trips.size() ) ) 
					{
						ptr_kCloseTrips.push_back(&sol.Trips[j]);
					}
				}		

				Trip emptyTrip(inst);			

				ptr_kCloseTrips.push_back(&emptyTrip);	
				SearchNetRelocation(inst, sol.Trips[i], ptr_kCloseTrips, improveFlag, searchAccount[NEIGHBORHOOD_NetRelocation]);

				if (emptyTrip.getSize() > 0)
				{
					sol.Trips.push_back(emptyTrip);
				}
			}
		}
	}
}

void searchCrossExchange_Controller(Instance& inst, Solution& sol, bool& overallImproveFlag, vector<double>& searchAccount, ofstream& log, bool permitsNewTrips, bool sloopy)
{
	bool loopAgain = true;
	while (loopAgain)
	{
		loopAgain = false;
		sol.SortMCLongitude();
		vector<bool> optimized(sol.Trips.size());
		for (int i = 0; i < (int)optimized.size(); i++)
		{
			optimized[i] = sol.Trips[i].getNeighborhoodStatus(NEIGHBORHOOD_CrossExchange);
			sol.Trips[i].setNeighborhoodCleared(NEIGHBORHOOD_CrossExchange);
		}
		for (int i = 0; i < (int)sol.Trips.size(); i++)
		{						
			bool tripImproveFlag = false;
			log << "Inter searchCrossExchange trip " << i+1 << "/" << sol.Trips.size() << endl;
			log << "Current Cost: " << std::setprecision(17) << inst.EvaluateSol(sol) << endl;					
						
			for (int j = i+1; j < (int)sol.Trips.size(); j++) //look at the searchCrossExchange_reach nearest trips
			{
				if ( ( i >= (int)optimized.size() ) || ( !optimized[i] ) || ( j >= (int)optimized.size() ) || ( !optimized[j] ) )
				{
				#ifdef CROSSEXCHANGE_ARTUR
					searchCrossExchangeArtur(inst, sol.Trips[i], sol.Trips[j], tripImproveFlag, searchAccount,i,j);
				#else
					searchCrossExchange(inst, sol.Trips[i], sol.Trips[j], tripImproveFlag, searchAccount[NEIGHBORHOOD_CrossExchange], i, j, abs(i-j) > searchCrossExchange_reach);
				#endif
				}						
			}
			if (permitsNewTrips)
			{
				if  ( ( i >= (int)optimized.size() ) || ( !optimized[i] ) )
				{
					Trip emptyTrip(inst);
					searchCrossExchange(inst, sol.Trips[i], emptyTrip, tripImproveFlag, searchAccount[NEIGHBORHOOD_CrossExchange], i, -1);
					if (emptyTrip.getSize() > 0)
					{
						sol.Trips.push_back(emptyTrip);
					}
				}
			}
			
			//if any improvement happened, the trip should be compared to all combinations again
			if (tripImproveFlag) 
			{
				if (!sloopy)
				{
					loopAgain = true;
				} 
				overallImproveFlag = true;
			}
		}	
	}
}

void searchIntra_Controller(Instance& inst, Solution& sol, vector<double>& candidateSearchAccount, ofstream& log)
{	
	int count = 0;

	#ifdef PARALLEL_INTRA_SEARCH
	#pragma omp parallel for	
	#endif
	for (int i = 0; i < (int)sol.Trips.size(); i++)
	{
		if (sol.Trips[i].getNeighborhoodStatus(NEIGHBORHOOD_2Opt))
		{
			bool intraImproveFlag = true;
			while (intraImproveFlag)
			{
				intraImproveFlag = false;
					
				sol.Trips[i].search2Opt(inst, intraImproveFlag, candidateSearchAccount[NEIGHBORHOOD_2Opt]);	
				sol.Trips[i].search3Opt(inst, intraImproveFlag, candidateSearchAccount[NEIGHBORHOOD_3Opt]);	
				sol.Trips[i].search4Opt(inst, intraImproveFlag, candidateSearchAccount[NEIGHBORHOOD_4Opt]);					
				sol.Trips[i].searchIntraRelocation(inst, intraImproveFlag, candidateSearchAccount[NEIGHBORHOOD_IntraRelocation]);					
				sol.Trips[i].searchIntraChainRelocation(inst, intraImproveFlag, candidateSearchAccount[NEIGHBORHOOD_IntraChainRelocation]);					
				sol.Trips[i].searchIntraExchange(inst, intraImproveFlag, candidateSearchAccount[NEIGHBORHOOD_IntraExchange]);
			}
		}
	}
}

void searchNetCrossExchange_Controller(Instance& inst, Solution& sol, bool& overallImproveFlag, vector<double>& searchAccount, ofstream& log)
{
	bool loopAgain = true;
	while (loopAgain)
	{
		loopAgain = false;
		sol.SortMCLongitude();
		vector<bool> optimized(sol.Trips.size());
		bool improveFlag = false;

		for (int i = 0; i < (int)optimized.size(); i++)
		{
			optimized[i] = sol.Trips[i].getNeighborhoodStatus(NEIGHBORHOOD_NetCrossExchange);
			sol.Trips[i].setNeighborhoodCleared(NEIGHBORHOOD_NetCrossExchange);
		}

		//interCount++;
		sol.SortMCLongitude();
		for (int i = 0; i < (int)sol.Trips.size(); i++)
		{
			log << "Inter SearchNetCrossExchange trip " << i+1 << "/" << sol.Trips.size() << endl;
			log << "Current Cost: " << std::setprecision(17) << inst.EvaluateSol(sol) << endl;					
								
			bool shoudSearch = false;
			for (int j = 0; j < (int)sol.Trips.size(); j++)
			{
				if ( j != i )
				{
					int interOffset = sol.getTripPairOffset(i,j);
					if ( (interOffset >= -searchNetCX_reach) && (interOffset <= searchNetCX_reach) )
					{										
						if ( ( i >= (int)optimized.size() ) || ( !optimized[i] ) || ( j >= (int)optimized.size() ) || ( !optimized[j] ) )
						{
							shoudSearch = true;
							break;
						}
					}
				}
			}							
			if (shoudSearch)
			{
				vector<Trip*> ptr_kCloseTrips;

				for (int j = 0; j < (int)sol.Trips.size(); j++)
				{
					if ( j != i )
					{
						int interOffset = sol.getTripPairOffset(i,j);
						if ( (interOffset >= -searchNetCX_reach) && (interOffset <= searchNetCX_reach) )
						{			
							ptr_kCloseTrips.push_back(&sol.Trips[j]);							
						}
					}
				}

				Trip emptyTrip(inst);			

				ptr_kCloseTrips.push_back(&emptyTrip);		
									
				SearchNetCrossExchange(inst, sol.Trips[i], ptr_kCloseTrips, improveFlag, searchAccount[NEIGHBORHOOD_NetCrossExchange]);

				if (emptyTrip.getSize() > 0)
				{
					sol.Trips.push_back(emptyTrip);
				}
			}
		}	
		sol.ValidateWeights(inst);
		sol.ClearEmptyTrips();		

		if (improveFlag) 
		{
		#ifndef SLOPPY_CROSS_EXCHANGE
			loopAgain = true;
		#endif
			overallImproveFlag = true;
		}
	} //end - while loopagain
}


void searchAllNetCrossExchange_Controller(Instance& inst, Solution& sol, bool& overallImproveFlag, vector<double>& searchAccount, ofstream& log)
{
	bool loopAgain = true;

	while (loopAgain)
	{
		loopAgain = false;
		sol.SortMCLongitude();
		vector<bool> optimized(sol.Trips.size());
		bool improveFlag = false;
		bool lastSearch = false;

		for (int i = 0; i < (int)optimized.size(); i++)
		{
			optimized[i] = sol.Trips[i].getNeighborhoodStatus(NEIGHBORHOOD_AllNetCrossExchange);
			sol.Trips[i].setNeighborhoodCleared(NEIGHBORHOOD_AllNetCrossExchange);
		}

		for (int i = 0; i < (int)sol.Trips.size(); i++)
		{
			log << "Inter SearchAllNetCrossExchange " << i+1 << "/" << sol.Trips.size() << endl;
			log << "Current Cost: " << std::setprecision(17) << inst.EvaluateSol(sol) << endl;					
								
			bool shoudSearch = false;
			for (int j = 0; j < (int)sol.Trips.size(); j++)
			{
				if ( j != i )
				{
					int interOffset = sol.getTripPairOffset(i,j);
					if ( (interOffset >= -searchAllNetCX_reach) && (interOffset <= searchAllNetCX_reach) )
					{										
						if ( ( i >= (int)optimized.size() ) || ( !optimized[i] ) || ( j >= (int)optimized.size() ) || ( !optimized[j] ) )
						{
							shoudSearch = true;
							break;
						}
					}
				}
			}
							
			if (shoudSearch)
			{
				vector<Trip*> ptr_kCloseTrips;

				for (int j = 0; j < (int)sol.Trips.size(); j++)
				{
					if ( j != i )
					{
						int interOffset = sol.getTripPairOffset(i,j);
						if ( (interOffset >= -searchNetCX_reach) && (interOffset <= searchNetCX_reach) )
						{	
							ptr_kCloseTrips.push_back(&sol.Trips[j]);							
						}
					}
				}

				Trip emptyTrip(inst);			

				ptr_kCloseTrips.push_back(&emptyTrip);		
									
				SearchAllNetCrossExchange(inst, ptr_kCloseTrips, improveFlag, searchAccount[NEIGHBORHOOD_AllNetCrossExchange]);	

				if (emptyTrip.getSize() > 0)
				{
					sol.Trips.push_back(emptyTrip);
				}							
			}

			if (lastSearch)
			{
				break;
			}
		}	

		if (improveFlag) 
		{
		#ifndef SLOPPY_CROSS_EXCHANGE
			loopAgain = true;
		#endif
			overallImproveFlag = true;
		}
	} //end - while loopagain
}

void searchCondense3_Controller(Instance& inst, Solution& sol, bool& overallImproveFlag, vector<double>& searchAccount, ofstream& log)
{
	bool loopAgain = true;
	while (loopAgain)
	{
		loopAgain = false;
		sol.SortMCLongitude();
		vector<bool> optimized(sol.Trips.size());
		bool improveFlag = false;
		bool lastSearch = false;

		for (int i = 0; i < (int)optimized.size(); i++)
		{
			optimized[i] = sol.Trips[i].getNeighborhoodStatus(NEIGHBORHOOD_Condense3);
			sol.Trips[i].setNeighborhoodCleared(NEIGHBORHOOD_Condense3);
		}
							
		for (int i = 0; i < (int)sol.Trips.size()-2; i++)
		{
			log << "Inter SeachCondense3 " << i+1 << "/" << sol.Trips.size() << endl;
			log << "Current Cost: " << std::setprecision(17) << inst.EvaluateSol(sol) << endl;					
								
			for (int j = i+1; ( (j < (int)sol.Trips.size()-1) && (j - i <= searchCondense_reach) ); j++)
			{
				for (int k = j+1; (k < (int)sol.Trips.size() && (k - j <= searchCondense_reach) ); k++)
				{				
					if ( (!optimized[i]) || (!optimized[j]) || (!optimized[k]) )
					{
						if (sol.Trips[i].getWeight() + sol.Trips[j].getWeight() + sol.Trips[k].getWeight() - inst.sleighWeight <= 2*inst.weightLimit)
						{
							Trip newTrip(inst);
							vector<Trip*> ptr_Trips;
							ptr_Trips.push_back(&sol.Trips[i]);
							ptr_Trips.push_back(&sol.Trips[j]);
							ptr_Trips.push_back(&sol.Trips[k]);
							
							vector<Trip*> ptr_newTrips;
														
							SearchCondense(inst, ptr_Trips,  ptr_newTrips, improveFlag, searchAccount[NEIGHBORHOOD_Condense3], true);

							if ((int)ptr_newTrips.size() > 0)
							{
								improveFlag = true;

								vector<int> indexes;
								indexes.push_back(i);
								indexes.push_back(j);
								indexes.push_back(k);
								sort(indexes.begin(), indexes.end());
								
								sol.Trips.erase(sol.Trips.begin() + indexes[2]);
								sol.Trips.erase(sol.Trips.begin() + indexes[1]);	
								sol.Trips.erase(sol.Trips.begin() + indexes[0]);											

								for (int i = 0; i < ptr_newTrips.size(); i++)
								{
									sol.Trips.push_back(*ptr_newTrips[i]);
								}
							}
						}					
					}
				}
			}
		}

		if (improveFlag) 
		{
		#ifndef SLOPPY_CROSS_EXCHANGE
			loopAgain = true;
		#endif
			overallImproveFlag = true;
		}
	}
}



void searchMerge3_Controller(Instance& inst, Solution& sol, bool& overallImproveFlag, vector<double>& searchAccount, ofstream& log)
{
	bool loopAgain = true;
	bool auxFeasible;
	while (loopAgain)
	{
		loopAgain = false;
		sol.SortMCLongitude();
		vector<bool> optimized(sol.Trips.size());
		bool improveFlag = false;
		bool lastSearch = false;

		for (int i = 0; i < (int)optimized.size(); i++)
		{
			optimized[i] = sol.Trips[i].getNeighborhoodStatus(NEIGHBORHOOD_Merge3);
			sol.Trips[i].setNeighborhoodCleared(NEIGHBORHOOD_Merge3);
		}
							
		for (int i = 0; i < (int)sol.Trips.size()-2; i++)
		{
			log << "Inter SearchMerge3 " << i+1 << "/" << sol.Trips.size() << endl;
			log << "Current Cost: " << std::setprecision(17) << inst.EvaluateSol(sol) << endl;					
								
			for (int j = i+1; j < (int)sol.Trips.size() - 1; j++)
			{
				for (int k = j+1; k < (int)sol.Trips.size(); k++)
				{
				
					if ( (!optimized[i]) || (!optimized[j]) || (!optimized[k]) )
					{
						if (sol.Trips[i].getWeight() + sol.Trips[j].getWeight() + sol.Trips[k].getWeight() - 2*inst.sleighWeight <= inst.weightLimit)
						{
							Trip newTrip(inst);
							vector<Trip*> ptr_Trips;
							ptr_Trips.push_back(&sol.Trips[i]);
							ptr_Trips.push_back(&sol.Trips[j]);
							ptr_Trips.push_back(&sol.Trips[k]);

							SearchMerge(inst, ptr_Trips, newTrip, improveFlag, searchAccount[NEIGHBORHOOD_Merge3], true);

							if (newTrip.getSize() > 0)
							{
								improveFlag = true;

								vector<int> indexes;
								indexes.push_back(i);
								indexes.push_back(j);
								indexes.push_back(k);
								sort(indexes.begin(), indexes.end());
								
								sol.Trips.erase(sol.Trips.begin() + indexes[2]);
								sol.Trips.erase(sol.Trips.begin() + indexes[1]);	
								sol.Trips.erase(sol.Trips.begin() + indexes[0]);											

								sol.Trips.push_back(newTrip);
							}
						}					
					}
				}
			}
		}

		if (improveFlag) 
		{
		#ifndef SLOPPY_CROSS_EXCHANGE
			loopAgain = true;
		#endif
			overallImproveFlag = true;
		}
	}
}

void searchMerge4_Controller(Instance& inst, Solution& sol, bool& overallImproveFlag, vector<double>& searchAccount, ofstream& log)
{
	bool loopAgain = true;
	bool auxFeasible;
	while (loopAgain)
	{
		loopAgain = false;
		sol.SortMCLongitude();
		vector<bool> optimized(sol.Trips.size());
		bool improveFlag = false;
		bool lastSearch = false;

		for (int i = 0; i < (int)optimized.size(); i++)
		{
			optimized[i] = sol.Trips[i].getNeighborhoodStatus(NEIGHBORHOOD_Merge4);
			sol.Trips[i].setNeighborhoodCleared(NEIGHBORHOOD_Merge4);
		}
							
		for (int i = 0; i < (int)sol.Trips.size()-3; i++)
		{
			log << "Inter SearchMerge4 " << i+1 << "/" << sol.Trips.size() << endl;
			log << "Current Cost: " << std::setprecision(17) << inst.EvaluateSol(sol) << endl;					
								
			for (int j = i+1; j < (int)sol.Trips.size()-2; j++)
			{
				for (int k = j+1; k < (int)sol.Trips.size()-1; k++)
				{
					for (int l = k+1; k < (int)sol.Trips.size(); k++)
					{
						if ( (!optimized[i]) || (!optimized[j]) || (!optimized[k]) || (!optimized[l]) )
						{
							if (sol.Trips[i].getWeight() + sol.Trips[j].getWeight() + sol.Trips[k].getWeight() + sol.Trips[l].getWeight() - 3*inst.sleighWeight <= inst.weightLimit)
							{
								Trip newTrip(inst);
								vector<Trip*> ptr_Trips;
								ptr_Trips.push_back(&sol.Trips[i]);
								ptr_Trips.push_back(&sol.Trips[j]);
								ptr_Trips.push_back(&sol.Trips[k]);
								ptr_Trips.push_back(&sol.Trips[l]);

								SearchMerge(inst, ptr_Trips, newTrip, improveFlag, searchAccount[NEIGHBORHOOD_Merge4], true);
								if (newTrip.getSize() > 0)
								{
									improveFlag = true;

									vector<int> indexes;
									indexes.push_back(i);
									indexes.push_back(j);
									indexes.push_back(k);
									indexes.push_back(l);
									sort(indexes.begin(), indexes.end());
								
									sol.Trips.erase(sol.Trips.begin() + indexes[3]);
									sol.Trips.erase(sol.Trips.begin() + indexes[2]);
									sol.Trips.erase(sol.Trips.begin() + indexes[1]);	
									sol.Trips.erase(sol.Trips.begin() + indexes[0]);											

									sol.Trips.push_back(newTrip);
								}
							}					
						}
					}
				}
			}
		}

		if (improveFlag) 
		{
		#ifndef SLOPPY_CROSS_EXCHANGE
			loopAgain = true;
		#endif
			overallImproveFlag = true;
		}
	}
}


void searchMerge5_Controller(Instance& inst, Solution& sol, bool& overallImproveFlag, vector<double>& searchAccount, ofstream& log)
{
	bool loopAgain = true;
	bool auxFeasible;
	while (loopAgain)
	{
		loopAgain = false;
		sol.SortMCLongitude();
		vector<bool> optimized(sol.Trips.size());
		bool improveFlag = false;
		bool lastSearch = false;

		for (int i = 0; i < (int)optimized.size(); i++)
		{
			optimized[i] = sol.Trips[i].getNeighborhoodStatus(NEIGHBORHOOD_Merge5);
			sol.Trips[i].setNeighborhoodCleared(NEIGHBORHOOD_Merge5);
		}
							
		for (int i = 0; i < (int)sol.Trips.size()-3; i++)
		{
			log << "Inter SearchMerge5 " << i+1 << "/" << sol.Trips.size() << endl;
			log << "Current Cost: " << std::setprecision(17) << inst.EvaluateSol(sol) << endl;					
								
			for (int j = i+1; j < (int)sol.Trips.size()-3; j++)
			{
				for (int k = j+1; k < (int)sol.Trips.size()-2; k++)
				{
					for (int l = k+1; l < (int)sol.Trips.size()-1; l++)
					{
						for (int m = l+1; m < (int)sol.Trips.size(); m++)
						{
							if ( (!optimized[i]) || (!optimized[j]) || (!optimized[k]) || (!optimized[l]) )
							{
								if (sol.Trips[i].getWeight() + sol.Trips[j].getWeight() + sol.Trips[k].getWeight() + sol.Trips[l].getWeight() + sol.Trips[m].getWeight() - 4*inst.sleighWeight <= inst.weightLimit)
								{
									Trip newTrip(inst);
									vector<Trip*> ptr_Trips;
									ptr_Trips.push_back(&sol.Trips[i]);
									ptr_Trips.push_back(&sol.Trips[j]);
									ptr_Trips.push_back(&sol.Trips[k]);
									ptr_Trips.push_back(&sol.Trips[l]);
									ptr_Trips.push_back(&sol.Trips[m]);

									SearchMerge(inst, ptr_Trips, newTrip, improveFlag, searchAccount[NEIGHBORHOOD_Merge5], true);
									if (newTrip.getSize() > 0)
									{
										improveFlag = true;

										vector<int> indexes;
										indexes.push_back(i);
										indexes.push_back(j);
										indexes.push_back(k);
										indexes.push_back(l);
										indexes.push_back(m);
										sort(indexes.begin(), indexes.end());
								
										sol.Trips.erase(sol.Trips.begin() + indexes[m]);
										sol.Trips.erase(sol.Trips.begin() + indexes[3]);
										sol.Trips.erase(sol.Trips.begin() + indexes[2]);
										sol.Trips.erase(sol.Trips.begin() + indexes[1]);	
										sol.Trips.erase(sol.Trips.begin() + indexes[0]);											

										sol.Trips.push_back(newTrip);
									}
								}					
							}
						}
					}
				}
			}
		}

		if (improveFlag) 
		{
		#ifndef SLOPPY_CROSS_EXCHANGE
			loopAgain = true;
		#endif
			overallImproveFlag = true;
		}
	}
}

void searchMerge2_Controller(Instance& inst, Solution& sol, bool& overallImproveFlag, vector<double>& searchAccount, ofstream& log)
{
	bool loopAgain = true;
	bool auxFeasible;
	while (loopAgain)
	{
		loopAgain = false;
		sol.SortMCLongitude();
		vector<bool> optimized(sol.Trips.size());
		bool improveFlag = false;
		bool lastSearch = false;

		for (int i = 0; i < (int)optimized.size(); i++)
		{
			optimized[i] = sol.Trips[i].getNeighborhoodStatus(NEIGHBORHOOD_Merge2);
			sol.Trips[i].setNeighborhoodCleared(NEIGHBORHOOD_Merge2);
		}
							
		for (int i = 0; i < (int)sol.Trips.size(); i++)
		{
			log << "Inter SearchMerge " << i+1 << "/" << sol.Trips.size() << endl;
			log << "Current Cost: " << std::setprecision(17) << inst.EvaluateSol(sol) << endl;					
								
			for (int t = -searchMerge2_reach; t <= searchMerge2_reach; t++)
			{
				int j = i + t;
				while (j >= (int)sol.Trips.size())
				{
					j -= sol.Trips.size();
				}
				if ( (i != j) && (j >= 0) )
				{
					if ( (!optimized[i]) || (!optimized[j]) )
					{
						if (sol.Trips[i].getWeight() + sol.Trips[j].getWeight() - inst.sleighWeight <= inst.weightLimit)
						{
							Trip newTrip(inst);
							vector<Trip*> ptr_Trips;
							ptr_Trips.push_back(&sol.Trips[i]);
							ptr_Trips.push_back(&sol.Trips[j]);

							SearchMerge(inst, ptr_Trips, newTrip, improveFlag, searchAccount[NEIGHBORHOOD_Merge2], true);
							if (newTrip.getSize() > 0)
							{
								improveFlag = true;

								vector<int> indexes;
								indexes.push_back(i);
								indexes.push_back(j);
								sort(indexes.begin(), indexes.end());

								sol.Trips.erase(sol.Trips.begin() + indexes[1]);	
								sol.Trips.erase(sol.Trips.begin() + indexes[0]);											

								sol.Trips.push_back(newTrip);
							}
						}					
					}
				}
			}
		}

		if (improveFlag) 
		{
		#ifndef SLOPPY_CROSS_EXCHANGE
			loopAgain = true;
		#endif
			overallImproveFlag = true;
		}
	}
}

void searchMerge2Polar_Controller(Instance& inst, Solution& sol, bool& overallImproveFlag, vector<double>& searchAccount, ofstream& log)
{
	bool loopAgain = true;
	bool auxFeasible;
	while (loopAgain)
	{
		loopAgain = false;
		sol.SortMCLongitude();
		vector<bool> optimized(sol.Trips.size());
		bool improveFlag = false;
		bool lastSearch = false;

		for (int i = 0; i < (int)optimized.size(); i++)
		{
			optimized[i] = sol.Trips[i].getNeighborhoodStatus(NEIGHBORHOOD_Merge2Polar);
			sol.Trips[i].setNeighborhoodCleared(NEIGHBORHOOD_Merge2Polar);
		}
							
		for (int i = 0; i < (int)sol.Trips.size()-1; i++)
		{
			log << "Inter SearchMerge2Polar " << i+1 << "/" << sol.Trips.size() << endl;
			log << "Current Cost: " << std::setprecision(17) << inst.EvaluateSol(sol) << endl;					
								
			for (int j = i+1; j < (int)sol.Trips.size(); j++)
			{
				if ( (sol.Trips[i].isPolar()) && (sol.Trips[j].isPolar()) )
				{
					if ( (!optimized[i]) || (!optimized[j]) )
					{
						SearchMerge2Polar(inst, sol.Trips[i], sol.Trips[j], improveFlag, searchAccount[NEIGHBORHOOD_Merge2Polar], auxFeasible);
					}
				}
			}
		}

		if (improveFlag) 
		{
		#ifndef SLOPPY_CROSS_EXCHANGE
			loopAgain = true;
		#endif
			overallImproveFlag = true;
		}
	}
}
