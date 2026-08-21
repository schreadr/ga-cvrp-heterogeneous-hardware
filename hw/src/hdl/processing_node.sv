///////////////////////////////////////////////////////////////////////////////////
//
// Project		: Capacitated Vehicle Routing Problem on FPGA erterte
// Function		: Processing node for crossover, mutation and 2-opt - Special version, where each node ID holds the city coordinates so that the distances can be calculated via cab driver geometrics
// File			: ProcessingNode_revisited_coordinates_2opt.sv
// Created		: 10/27/2021
// Author		: Maximilian Heer
// Notes		: Features the maximal preservative crossover, k-swap-mutation and local 2-opt algorithm for optimization
// Advisory 	: Number of cities must not be a prime number, since partition in equally sized subtours won't work in this case
//
///////////////////////////////////////////////////////////////////////////////////

`timescale 1ns/1ps
`default_nettype none

module processing_node #(
	parameter pNumCities = 16,
	parameter pCapBitWidth = 3,
	parameter pIDBitWidth = 12,
	parameter pCoordBitWidth = 6,
	parameter pDistMaxBitWidth = 13,
	parameter pVehicleCapacity = 10,
	parameter pRandomSubtourOffset = 5,
	parameter pMutationDistance = 6,
	parameter pInitMutationProcesses = 25,
	parameter pDepotXCoord = 16,
	parameter pDepotYCoord = 16,
	parameter pCityNumBitWidth = 5
)(
	// Connecting the processing node to common clock and reset
	input wire logic clk_i,
	input wire logic rst_i,

	// Input Signal for init mode - changes mutation prob. etc. for getting an init-population
	input wire logic init_mode_i,

	// Input Signal for starting the transmission of parent data of parent 1
	input wire logic start_parent_1_i,

	// Input Signal for starting the transmission of parent data of parent 2
	input wire logic start_parent_2_i,

	// Input Signal for finished transmission of parent data of parent 1
	input wire logic finished_parent_1_i,

	// Input Signal for finished transmission of parent data of parent 2
	input wire logic finished_parent_2_i,

	// Input connection for getting data from the pop matrix
	input wire logic[71:0] pop_data_i,

	// Output connection for sending data to the pop matrix
	output logic[71:0] pop_data_o,

	// Output to show that input is loaded
	output logic input_reading_done,

	// Output connection for signaling that transmission of Individual 1 is active
	output logic transmission_ind_1_o,

	// Output connection for signaling that transmission of Individual 2 is active
	output logic transmission_ind_2_o,

	// Output signal to show that output individual is still parent
	output logic unchanged_ind_o,

	// Input signal for random city number for crossover for child 1
	//input wire logic[pCityNumBitWidth-1:0] random_city_number_crossover_1_i,

	// Input signal for random city number for crossover for child 2
	//input wire logic[pCityNumBitWidth-1:0] random_city_number_crossover_2_i,

	// Input signal for random city number for mutation for child 1
	input wire logic [pCityNumBitWidth-1:0] random_city_number_mutation_1_i,

	// Input signal for random city number for mutation for child 2
	input wire logic [pCityNumBitWidth-1:0] random_city_number_mutation_2_i,

	// Output signal for showing that one crossover operation is done
	//output logic crossover_done,

	// Output signal wether slot 0 or 1 contains better fitness value
	output logic better_fitness_slot_o,

	// Output signal with the best fitness signal
    output logic[pIDBitWidth+pCapBitWidth:0] better_fitness_value_o,

    // Input signal that indicated whether mutation should be done or not in this cycle
    input wire logic do_mutation_i, 
    
    // Input signal that indicates whether wild child is applied or not = transmit children in any case, even if they are worse than their parents 
    input wire logic do_wild_child_i, 
    
    // Input signal that decides about the length of copied chunk from parent 1 to child 1
    input wire logic [pCityNumBitWidth-1:0] crossover_1_length_i, 
    
    // Input signal that decides about the length of copied chunk from parent 2 to child 2 
    input wire logic [pCityNumBitWidth-1:0] crossover_2_length_i, 
    
    input wire logic [pCityNumBitWidth-1:0] mutation_length_1_i, 
    
    input wire logic [pCityNumBitWidth-1:0] mutation_length_2_i
);

	// Definition of localparams
	localparam lpIndFitBitWidth = pDistMaxBitWidth + $clog2(pNumCities-1); // Bit Width of the Fitness value for an entire Individual
	localparam lpNodeIDBitWidth = pIDBitWidth; // Bit Width of the Node ID
	localparam lpPopFieldBitWidth = lpNodeIDBitWidth + pCapBitWidth + 1; // Bit Width of an entry in an Individual
	localparam lpVehicleCapBitWidth = $clog2(pVehicleCapacity-1); // Bit Width of the capacity of the vehicle
	localparam lpNumOfEntriesPerMemLine = (72/(lpNodeIDBitWidth+pCapBitWidth+1)); // Number of Entries per Memory Channel (72 Bit)
	localparam lpRestBitsPerMemLine = 72 - lpNumOfEntriesPerMemLine*(lpNodeIDBitWidth+pCapBitWidth+1); // Rest Bits per Memory Channel (72 Bit) due to misalignment of city entries and memory channel bit width
	localparam lpCityEntryBitWidth = lpNodeIDBitWidth+pCapBitWidth+1;
	localparam lpDepotXCoord = pDepotXCoord;
	localparam lpDepotYCoord = pDepotYCoord;
	localparam lpInitMutationProcessesBitWidth = $clog2(pInitMutationProcesses-1);


	/////////////////////////////////////////////////////////////////////////////////////////
	//
	// Definition of data types
	//
	/////////////////////////////////////////////////////////////////////////////////////////

	// Definition of the Individual Fitness word
	//typedef logic [lpIndFitBitWidth-1:0] IndividualFitness;
	typedef logic [lpCityEntryBitWidth-1:0] IndividualFitness;

	// Definition of a single coordinate
	typedef logic [pCoordBitWidth-1:0] NodeCoordinate;

	// Definition of a nodecounter datatype
	typedef logic [pIDBitWidth-1:0] NodeCounter;

	// Definition of the NodeID
	typedef struct packed{
		NodeCoordinate xCoordinate;
		NodeCoordinate yCoordinate;
	} NodeID;

	// Definition of the NodeCapacity
	typedef logic [pCapBitWidth-1:0] NodeCapacity;

	// Definition of the VehicleCapacity
    typedef logic [lpVehicleCapBitWidth:0] VehicleCapacity;

	// Definition of the Depot Bit
	typedef logic Depot;

	// Definition of a city-entry from an Individual
	typedef struct packed{
		NodeID nodeID;
		NodeCapacity nodeCapacity;
		Depot depot;
	} CityEntry;

	// Definition of states for the parents-FSM
	typedef enum logic[3:0] {
		IDLE_PARENT = 4'b0000,
		WAIT_PARENT_1 = 4'b0001,
		BEGIN_PARENT_2 = 4'b0010,
		WAIT_PARENT_2 = 4'b0011,
		PARENT_2_UNCHANGED = 4'b0100,
		TRANSMIT_CHILD_1_BEGIN = 4'b0101,
		TRANSMIT_CHILD_1_REST = 4'b0110,
		TRANSMIT_CHILD_2_BEGIN = 4'b0111,
		TRANSMIT_CHILD_2_REST = 4'b1000,
		START_NEW_INPUT_LINE_PARENT_1 = 4'b1001,
		READING_INPUT_VALUES_PARENT_1 = 4'b1010,
		START_NEW_INPUT_LINE_PARENT_2 = 4'b1011,
		READING_INPUT_VALUES_PARENT_2 = 4'b1100,
		WAIT_STATE_1 = 4'b1101,
		WAIT_STATE_2 = 4'b1110
	} ParentsFSMState;

	// Definition of states for the children-FSM
	typedef enum logic[3:0] {
		IDLE = 4'b0000,
		INSERT_REST_NODES = 4'b0001,
		PERFORM_MUTATION = 4'b0010,
		DEPOT_INSERTION = 4'b0011,
		TWO_OPT_EVALUATION = 4'b0100,
		TWO_OPT_CALCULATION = 4'b0101,
		DISTANCE_CALCULATION = 4'b0110,
		DISTANCE_CALCULATION_DEPOT_STOP = 4'b0111,
		DISTANCE_CALCULATION_FINAL = 4'b1000,
		DISTANCE_CALCULATION_INTERMEDIATE_FINAL = 4'b1001,
		LOAD_CHILD = 4'b1010, 
		TWO_OPT_SWAP = 4'b1100, 
		DISTANCE_CALCULATION_WAIT_STATE = 4'b1101, 
		DISTANCE_CALCULATION_DEPOT_STOP_WAIT_STATE = 4'b1110, 
		DISTANCE_CALCULATION_INTERMEDIATE_FINAL_WAIT_STATE = 4'b1111
	} ChildrenFSMState;


	/////////////////////////////////////////////////////////////////////////////////////////////
	//
	// Definition of the register files for parents and offsprings
	//
	/////////////////////////////////////////////////////////////////////////////////////////////

	// PARENT 1

	// Individual Fitness entry
	IndividualFitness fitness_value_parent_1;

	// Individual City Storage
	CityEntry parent_1_tour[pNumCities];


	// PARENT 2

	// Individual Fitness entry
	IndividualFitness fitness_value_parent_2;

	// Individual City Storage
	CityEntry parent_2_tour[pNumCities];


	// CHILD 1

	// Individual Fitness entry
	IndividualFitness fitness_value_child_1;

	// Individual City Storage
	CityEntry child_1_tour[pNumCities];


	// CHILD 2

	// Individual Fitness entry
	IndividualFitness fitness_value_child_2;

	// Individual City Storage
	CityEntry child_2_tour[pNumCities];



	////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//
	// Additional regs and signals for calculation purposes
	//
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	// State reg for the parents FSM
	ParentsFSMState state_parents;

	// State reg for the child 1 FSM
	ChildrenFSMState state_child_1;

	// State reg for the child 2 FSM
	ChildrenFSMState state_child_2;

	// Counter for storing data to the parents register banks
	logic[lpNodeIDBitWidth-1:0] cityCounterParents;

	// Signal that child 1 is calculated
	logic child_1_finished;

	// Signal that child 2 is calculated
	logic child_2_finished;

	// Common wire signal that shows that both children are calculated
	wire children_finished;

	// Signal that parent 1 dominates two out of three with fitness value
	wire parent_1_dominates_2;

	// Signal that parent 2 dominates two out of three with fitness value
	wire parent_2_dominates_2;

	// Signal that child 1 dominates two out of three with fitness value
	wire child_1_dominates_2;

	// Signal that child 2 dominates two out of three with fitness value
	wire child_2_dominates_2;

	// Wires for comparison of individual fitness values
	wire parent_1_fitter_parent_2;
	wire parent_1_fitter_child_1;
	wire parent_1_fitter_child_2;
	wire parent_2_fitter_child_1;
	wire parent_2_fitter_child_2;
	wire child_1_fitter_child_2;

	// Reg for showing that second child needs to be transmitted
	logic second_child_to_be_transmitted;

	// City Counter to iterate through child 1 for inserting nodes
	NodeCounter city_selector_child_1_counter_1;

	// City Counter for insertion of rest nodes
	NodeCounter child_1_iterator;
	NodeCounter child_2_iterator;

	// City Counter to iterate through child 2 for inserting nodes into child 1
	NodeCounter city_selector_child_1_counter_2;

	// City Counter to iterate through child 1 for inserting nodes
	NodeCounter city_selector_child_2_counter_1;

	// City Counter to iterate through child 2 for inserting nodes into child 2
	NodeCounter city_selector_child_2_counter_2;
	
	// Swap counter for the children FSM 
	NodeCounter child_1_swap_counter;
	NodeCounter child_2_swap_counter;
	
	// City Counter for jumping back after each way through to find starting point again
    NodeCounter child_1_return; 
    NodeCounter child_2_return; 
    
    // Indicator whether improvement was made or not  
    logic child_1_improvement_made; 
    logic child_2_improvement_made; 

	// Comparator of node to be inserted in child 1 with already transmitted cities
	logic [pNumCities-1:0] node_already_in_child_1;

	// Comparator of node to be inserted in child 2 with already transmitted cities
	logic [pNumCities-1:0] node_already_in_child_2;

	// City Counters for loading input values to the parent regfiles
	NodeCounter city_1_load_counter;
	NodeCounter city_2_load_counter;
	NodeCounter incoming_data_counter;

	// First mutation point child 1
	NodeCounter first_mutation_point_child_1;

	// First mutation point child 2
	NodeCounter first_mutation_point_child_2;
	
	// Mutation length for child 1 
	NodeCounter mutation_length_child_1; 
	
	// Mutation length for child 2 
	NodeCounter mutation_length_child_2; 

	// Crossover point child 1
	NodeCounter crossover_point_child_1;

	// Crossover point child 2
	NodeCounter crossover_point_child_2;

	// Capacity Counter for child 1
	VehicleCapacity capacity_counter_child_1;

	// Capacity Counter for child 2
	VehicleCapacity capacity_counter_child_2;

	// Distance Value for Offspring 1
	logic[pDistMaxBitWidth-1:0] dist_child_1;

	// Distance Value for Offspring 2
	logic[pDistMaxBitWidth-1:0] dist_child_2;

	// Distance Calculation child 1 - coordinates
	NodeCoordinate x1_1_child_1;
	NodeCoordinate x1_2_child_1;
	NodeCoordinate y1_1_child_1;
	NodeCoordinate y1_2_child_1;

	NodeCoordinate x2_1_child_1;
	NodeCoordinate x2_2_child_1;
	NodeCoordinate y2_1_child_1;
	NodeCoordinate y2_2_child_1;

	NodeCoordinate x3_1_child_1;
	NodeCoordinate x3_2_child_1;
	NodeCoordinate y3_1_child_1;
	NodeCoordinate y3_2_child_1;

	NodeCoordinate x4_1_child_1;
	NodeCoordinate x4_2_child_1;
	NodeCoordinate y4_1_child_1;
	NodeCoordinate y4_2_child_1;

	NodeCoordinate x5_1_child_1;
	NodeCoordinate x5_2_child_1;
	NodeCoordinate y5_1_child_1;
	NodeCoordinate y5_2_child_1;
	

	// Distance Calculation child 2 - coordinate
	NodeCoordinate x1_1_child_2;
	NodeCoordinate x1_2_child_2;
	NodeCoordinate y1_1_child_2;
	NodeCoordinate y1_2_child_2;

	NodeCoordinate x2_1_child_2;
	NodeCoordinate x2_2_child_2;
	NodeCoordinate y2_1_child_2;
	NodeCoordinate y2_2_child_2;

	NodeCoordinate x3_1_child_2;
	NodeCoordinate x3_2_child_2;
	NodeCoordinate y3_1_child_2;
	NodeCoordinate y3_2_child_2;

	NodeCoordinate x4_1_child_2;
	NodeCoordinate x4_2_child_2;
	NodeCoordinate y4_1_child_2;
	NodeCoordinate y4_2_child_2;

	NodeCoordinate x5_1_child_2;
	NodeCoordinate x5_2_child_2;
	NodeCoordinate y5_1_child_2;
	NodeCoordinate y5_2_child_2;

	// cab metrics child 1
	wire logic[pDistMaxBitWidth-1:0] y1_distance_child_1;
	wire logic[pDistMaxBitWidth-1:0] x1_distance_child_1;
	wire logic[pDistMaxBitWidth-1:0] y2_distance_child_1;
	wire logic[pDistMaxBitWidth-1:0] x2_distance_child_1;
	wire logic[pDistMaxBitWidth-1:0] y3_distance_child_1;
	wire logic[pDistMaxBitWidth-1:0] x3_distance_child_1;
	wire logic[pDistMaxBitWidth-1:0] y4_distance_child_1;
	wire logic[pDistMaxBitWidth-1:0] x4_distance_child_1;
	wire logic[pDistMaxBitWidth-1:0] y5_distance_child_1;
	wire logic[pDistMaxBitWidth-1:0] x5_distance_child_1;
	wire logic[pDistMaxBitWidth-1:0] y6_distance_child_1;
	wire logic[pDistMaxBitWidth-1:0] x6_distance_child_1;
	wire logic[pDistMaxBitWidth-1:0] y7_distance_child_1;
	wire logic[pDistMaxBitWidth-1:0] x7_distance_child_1;
	wire logic[pDistMaxBitWidth-1:0] y8_distance_child_1;
	wire logic[pDistMaxBitWidth-1:0] x8_distance_child_1;
	wire logic[pDistMaxBitWidth-1:0] part_dist_1_child_1;
	wire logic[pDistMaxBitWidth-1:0] part_dist_2_child_1;
	wire logic[pDistMaxBitWidth-1:0] part_dist_3_child_1;
	wire logic[pDistMaxBitWidth-1:0] part_dist_4_child_1;
	wire logic[pDistMaxBitWidth-1:0] part_dist_5_child_1;
	wire logic[pDistMaxBitWidth-1:0] part_dist_6_child_1;
	wire logic[pDistMaxBitWidth-1:0] part_dist_7_child_1;
	wire logic[pDistMaxBitWidth-1:0] part_dist_8_child_1;

	// cab metrics child 1
	wire logic[pDistMaxBitWidth-1:0] y1_distance_child_2;
	wire logic[pDistMaxBitWidth-1:0] x1_distance_child_2;
	wire logic[pDistMaxBitWidth-1:0] y2_distance_child_2;
	wire logic[pDistMaxBitWidth-1:0] x2_distance_child_2;
	wire logic[pDistMaxBitWidth-1:0] y3_distance_child_2;
	wire logic[pDistMaxBitWidth-1:0] x3_distance_child_2;
	wire logic[pDistMaxBitWidth-1:0] y4_distance_child_2;
	wire logic[pDistMaxBitWidth-1:0] x4_distance_child_2;
	wire logic[pDistMaxBitWidth-1:0] y5_distance_child_2;
	wire logic[pDistMaxBitWidth-1:0] x5_distance_child_2;
	wire logic[pDistMaxBitWidth-1:0] y6_distance_child_2;
	wire logic[pDistMaxBitWidth-1:0] x6_distance_child_2;
	wire logic[pDistMaxBitWidth-1:0] y7_distance_child_2;
	wire logic[pDistMaxBitWidth-1:0] x7_distance_child_2;
	wire logic[pDistMaxBitWidth-1:0] y8_distance_child_2;
	wire logic[pDistMaxBitWidth-1:0] x8_distance_child_2;
	wire logic[pDistMaxBitWidth-1:0] part_dist_1_child_2;
	wire logic[pDistMaxBitWidth-1:0] part_dist_2_child_2;
	wire logic[pDistMaxBitWidth-1:0] part_dist_3_child_2;
	wire logic[pDistMaxBitWidth-1:0] part_dist_4_child_2;
	wire logic[pDistMaxBitWidth-1:0] part_dist_5_child_2;
	wire logic[pDistMaxBitWidth-1:0] part_dist_6_child_2;
	wire logic[pDistMaxBitWidth-1:0] part_dist_7_child_2;
	wire logic[pDistMaxBitWidth-1:0] part_dist_8_child_2;

	// Special treatment for depot stops within
	logic child_1_depot_second;
	logic child_2_depot_second;

	// Additional registers for init mode
	logic init_mode_set_child_1;
	logic init_mode_set_child_2;

	logic [lpInitMutationProcessesBitWidth-1:0] mutation_process_counter_child_1;
	logic [lpInitMutationProcessesBitWidth-1:0] mutation_process_counter_child_2;

	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//
	// Wire Assignments for calculations
	//
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	// Calculation whether both children are calculated
	assign children_finished = child_1_finished & child_2_finished;

	// Comparison of individual fitnesses
	assign parent_1_fitter_parent_2 = (fitness_value_parent_1 <= fitness_value_parent_2) && !do_wild_child_i;
	assign parent_1_fitter_child_1 = (fitness_value_parent_1 <= fitness_value_child_1) && !do_wild_child_i;
	assign parent_1_fitter_child_2 = (fitness_value_parent_1 <= fitness_value_child_2) && !do_wild_child_i;
	assign parent_2_fitter_child_1 = (fitness_value_parent_2 <= fitness_value_child_1) && !do_wild_child_i;
	assign parent_2_fitter_child_2 = (fitness_value_parent_2 <= fitness_value_child_2) && !do_wild_child_i;
	assign child_1_fitter_child_2 = (fitness_value_child_1 <= fitness_value_child_2);

	// Calculation of the domination regarding fitness values
	assign parent_1_dominates_2 = parent_1_fitter_parent_2 ? (parent_1_fitter_child_1 || parent_1_fitter_child_2) : (parent_1_fitter_child_1 && parent_1_fitter_child_2);
	assign parent_2_dominates_2 = !parent_1_fitter_parent_2 ? (parent_2_fitter_child_1 || parent_2_fitter_child_2) : (parent_2_fitter_child_1 && parent_2_fitter_child_2);
	assign child_1_dominates_2 = !parent_1_fitter_child_1 ? (!parent_2_fitter_child_1 || child_1_fitter_child_2) : (!parent_2_fitter_child_1 && child_1_fitter_child_2);
	assign child_2_dominates_2 = !parent_1_fitter_child_2 ? (!parent_2_fitter_child_2 || !child_1_fitter_child_2) : (!parent_2_fitter_child_2 && !child_1_fitter_child_2);

	// Calculation of the comparator-signals for node insertion into the children 
	/*always @ (*) begin
		for(int cityCounter = 0; cityCounter < pRandomSubtourOffset; cityCounter++) begin
			node_already_in_child_1[cityCounter] <= (child_1_tour[cityCounter].nodeID == parent_2_tour[city_selector_child_1_counter_2].nodeID);
			node_already_in_child_2[cityCounter] <= (child_2_tour[cityCounter].nodeID == parent_1_tour[city_selector_child_2_counter_2].nodeID);
		end
	end*/
	
	// Calculation of the comparator-signals for node insertion into child 1 
	always @ (*) begin 
	   for(int cityCounter = 0; cityCounter < pNumCities; cityCounter++) begin 
	       if(cityCounter < crossover_1_length_i) begin
	           node_already_in_child_1[cityCounter] <= (child_1_tour[cityCounter].nodeID == parent_2_tour[city_selector_child_1_counter_2].nodeID);
	       end else begin 
	           node_already_in_child_1[cityCounter] <= 0; 
	       end 
	   end 
	end 
	
	// Calculation of the comparator-signals for node insertion into child 2 
	always @ (*) begin 
	   for(int cityCounter = 0; cityCounter < pNumCities; cityCounter++) begin 
	       if(cityCounter < crossover_2_length_i) begin 
	           node_already_in_child_2[cityCounter] <= (child_2_tour[cityCounter].nodeID == parent_1_tour[city_selector_child_2_counter_2].nodeID);
	       end else begin 
	           node_already_in_child_2[cityCounter] <= 0; 
	       end 
	   end 
	end

	// Modified euclidian distance calculation - 1 - child 1
	assign y1_distance_child_1 = (y1_1_child_1-y1_2_child_1)*(y1_1_child_1-y1_2_child_1);
	assign x1_distance_child_1 = (x1_1_child_1-x1_2_child_1)*(x1_1_child_1-x1_2_child_1);
	assign part_dist_1_child_1 = y1_distance_child_1 + x1_distance_child_1;

	// Modified euclidian distance calculation - 2 - child 1
	assign y2_distance_child_1 = (y2_1_child_1-y2_2_child_1)*(y2_1_child_1-y2_2_child_1);
	assign x2_distance_child_1 = (x2_1_child_1-x2_2_child_1)*(x2_1_child_1-x2_2_child_1);
	assign part_dist_2_child_1 = y2_distance_child_1 + x2_distance_child_1;

	// Modified euclidian distance calculation - 3 - child 1
	assign y3_distance_child_1 = (y3_1_child_1-y3_2_child_1)*(y3_1_child_1-y3_2_child_1);
	assign x3_distance_child_1 = (x3_1_child_1-x3_2_child_1)*(x3_1_child_1-x3_2_child_1);
	assign part_dist_3_child_1 = y3_distance_child_1 + x3_distance_child_1;

	// Modified euclidian distance calculation - 4 - child 1
	assign y4_distance_child_1 = (y4_1_child_1-y4_2_child_1)*(y4_1_child_1-y4_2_child_1);
	assign x4_distance_child_1 = (x4_1_child_1-x4_2_child_1)*(x4_1_child_1-x4_2_child_1);
	assign part_dist_4_child_1 = y4_distance_child_1 + x4_distance_child_1;

	// Modified euclidian distance calculation - 5 - child 1
	assign y5_distance_child_1 = (y5_1_child_1-y5_2_child_1)*(y5_1_child_1-y5_2_child_1);
	assign x5_distance_child_1 = (x5_1_child_1-x5_2_child_1)*(x5_1_child_1-x5_2_child_1);
	assign part_dist_5_child_1 = y5_distance_child_1 + x5_distance_child_1;
	

	// Modified euclidian distance calculation - 1 - child 2
	assign y1_distance_child_2 = (y1_1_child_2-y1_2_child_2)*(y1_1_child_2-y1_2_child_2);
	assign x1_distance_child_2 = (x1_1_child_2-x1_2_child_2)*(x1_1_child_2-x1_2_child_2);
	assign part_dist_1_child_2 = y1_distance_child_2 + x1_distance_child_2;

	// Modified euclidian distance calculation - 2 - child 2
	assign y2_distance_child_2 = (y2_1_child_2-y2_2_child_2)*(y2_1_child_2-y2_2_child_2);
	assign x2_distance_child_2 = (x2_1_child_2-x2_2_child_2)*(x2_1_child_2-x2_2_child_2);
	assign part_dist_2_child_2 = y2_distance_child_2 + x2_distance_child_2;

	// Modified euclidian distance calculation - 3 - child 2
	assign y3_distance_child_2 = (y3_1_child_2-y3_2_child_2)*(y3_1_child_2-y3_2_child_2);
	assign x3_distance_child_2 = (x3_1_child_2-x3_2_child_2)*(x3_1_child_2-x3_2_child_2);
	assign part_dist_3_child_2 = y3_distance_child_2 + x3_distance_child_2;

	// Modified euclidian distance calculation - 4 - child 2
	assign y4_distance_child_2 = (y4_1_child_2-y4_2_child_2)*(y4_1_child_2-y4_2_child_2);
	assign x4_distance_child_2 = (x4_1_child_2-x4_2_child_2)*(x4_1_child_2-x4_2_child_2);
	assign part_dist_4_child_2 = y4_distance_child_2 + x4_distance_child_2;

	// Modified euclidian distance calculation - 5 - child 2
	assign y5_distance_child_2 = (y5_1_child_2-y5_2_child_2)*(y5_1_child_2-y5_2_child_2);
	assign x5_distance_child_2 = (x5_1_child_2-x5_2_child_2)*(x5_1_child_2-x5_2_child_2);
	assign part_dist_5_child_2 = y5_distance_child_2 + x5_distance_child_2;
	
	
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//
	// Initializing two sqr_int-modules for the distance calculation within the two children 
	// 
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////
	
	// controlling signals for both child 1 and child 2 
	logic start_child_1; 
	logic start_child_2; 
	logic busy_child_1; 
	logic busy_child_2; 
	logic valid_child_1; 
	logic valid_child_2; 

    // Output values for the sqr_int-modules
    logic [pDistMaxBitWidth-1:0] root_child_1; 
    logic [pDistMaxBitWidth-1:0] root_child_2; 
    logic [pDistMaxBitWidth-1:0] rem_child_1; 
    logic [pDistMaxBitWidth-1:0] rem_child_2; 
	
	
	sqrt_int #(
	   .WIDTH(pDistMaxBitWidth)
	) sqr_int_child_1 (
	   .clk(clk_i), 
	   .start(start_child_1), 
	   .busy(busy_child_1), 
	   .valid(valid_child_1), 
	   .rad(part_dist_1_child_1), 
	   .root(root_child_1), 
	   .rem(rem_child_1)
	);
	
	sqrt_int #(
	   .WIDTH(pDistMaxBitWidth)
	) sqr_int_child_2 (
	   .clk(clk_i), 
	   .start(start_child_2),
	   .busy(busy_child_2), 
	   .valid(valid_child_2),
	   .rad(part_dist_1_child_2), 
	   .root(root_child_2), 
	   .rem(rem_child_2)
	); 


	//////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//
	// FSM for handling of the parents - writing to them from pop-memory, final comparison of fitness values
	//
	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	always @ (posedge clk_i) begin
		if(rst_i) begin

			// resetting the parents city counter
			cityCounterParents <= 0;

			// resetting the fitness values for both parents
			fitness_value_parent_1 <= 0;
			fitness_value_parent_2 <= 0;

			// resetting the reigster banks for both parents
			for(int cityCounter = 0; cityCounter < pNumCities; cityCounter++) begin
				// parent 1
				parent_1_tour[cityCounter].nodeID <= 0;
				parent_1_tour[cityCounter].nodeCapacity <= 0;
				parent_1_tour[cityCounter].depot <= 0;

				// parent 2
				parent_2_tour[cityCounter].nodeID <= 0;
				parent_2_tour[cityCounter].nodeCapacity <= 0;
				parent_2_tour[cityCounter].depot <= 0;
			end

			second_child_to_be_transmitted <= 0;

			// Reset the state-reg
			state_parents <= IDLE_PARENT;

			crossover_point_child_1 <= 0;
			crossover_point_child_2 <= 0;

			city_1_load_counter <= 0;
			city_2_load_counter <= 0;
			incoming_data_counter <= 0;

			input_reading_done <= 0;
			better_fitness_slot_o <= 0; 
			better_fitness_value_o <= 0; 

		end else begin
			case(state_parents)
				IDLE_PARENT: begin
					transmission_ind_1_o <= 0;
					transmission_ind_2_o <= 0;
					unchanged_ind_o <= 0;
					//crossover_done <= 0;
					//better_fitness_slot_o <= 0;
					//better_fitness_value_o <= 0;

					// if signal comes to load parent 1, begin to store
					if(start_parent_1_i) begin
						fitness_value_parent_1 <= pop_data_i[71:71-lpCityEntryBitWidth]/2;
						state_parents <= READING_INPUT_VALUES_PARENT_1;
						incoming_data_counter <= 1;
						city_1_load_counter <= 0;
						city_2_load_counter <= 0;
					end

					// If calculation of children is finished, begin to transfer them back into memory - or not, if parents are better than new children
					if(children_finished) begin
						if(parent_1_dominates_2 && parent_2_dominates_2) begin
							unchanged_ind_o <= 1;
							transmission_ind_1_o <= 1;
							state_parents <= PARENT_2_UNCHANGED;
							if(fitness_value_parent_1 <= fitness_value_parent_2) begin
								better_fitness_slot_o <= 0;
								better_fitness_value_o <= fitness_value_parent_1;
							end else begin
								better_fitness_slot_o <= 1;
								better_fitness_value_o <= fitness_value_parent_2;
							end

						end else if(parent_1_dominates_2 && child_1_dominates_2) begin
							unchanged_ind_o <= 1;
							transmission_ind_1_o <= 1;
							state_parents <= TRANSMIT_CHILD_1_BEGIN;
							if(fitness_value_parent_1 <= fitness_value_child_1) begin
								better_fitness_slot_o <= 0;
								better_fitness_value_o <= fitness_value_parent_1;
							end else begin
								better_fitness_slot_o <= 1;
								better_fitness_value_o <= fitness_value_child_1;
							end

						end else if(parent_1_dominates_2 && child_2_dominates_2) begin
							unchanged_ind_o <= 1;
							transmission_ind_1_o <= 1;
							state_parents <= TRANSMIT_CHILD_2_BEGIN;
							if(fitness_value_parent_1 <= fitness_value_child_2) begin
								better_fitness_slot_o <= 0;
								better_fitness_value_o <= fitness_value_parent_1;
							end else begin
								better_fitness_slot_o <= 1;
								better_fitness_value_o <= fitness_value_child_2;
							end

						end else if(parent_2_dominates_2 && child_1_dominates_2) begin
							unchanged_ind_o <= 1;
							transmission_ind_2_o <= 1;
							state_parents <= TRANSMIT_CHILD_1_BEGIN;
							if(fitness_value_parent_2<= fitness_value_child_1) begin
								better_fitness_slot_o <= 1;
								better_fitness_value_o <= fitness_value_parent_2;
							end else begin
								better_fitness_slot_o <= 0;
								better_fitness_value_o <= fitness_value_child_1;
							end

						end else if(parent_2_dominates_2 && child_2_dominates_2) begin
							unchanged_ind_o <= 1;
							transmission_ind_2_o <= 1;
							state_parents <= TRANSMIT_CHILD_2_BEGIN;
							if(fitness_value_parent_2<= fitness_value_child_2) begin
								better_fitness_slot_o <= 1;
								better_fitness_value_o <= fitness_value_parent_2;
							end else begin
								better_fitness_slot_o <= 0;
								better_fitness_value_o <= fitness_value_child_2;
							end

						end else if(child_1_dominates_2 && child_2_dominates_2) begin

							///////////////////// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! ////////////////////////////

							transmission_ind_1_o  <= 1;
							pop_data_o[71-:lpCityEntryBitWidth] <= fitness_value_child_1;
							for(int cityCounter = 1; cityCounter < lpNumOfEntriesPerMemLine; cityCounter++) begin
								pop_data_o[71-lpCityEntryBitWidth*cityCounter-:lpCityEntryBitWidth] <= child_1_tour[cityCounter-1];
							end
							cityCounterParents <= lpNumOfEntriesPerMemLine - 1;
							second_child_to_be_transmitted <= 1;
							state_parents <= TRANSMIT_CHILD_1_REST;
							if(fitness_value_child_1<= fitness_value_child_2) begin
								better_fitness_slot_o <= 0;
								better_fitness_value_o <= fitness_value_child_1;
							end else begin
								better_fitness_slot_o <= 1;
								better_fitness_value_o <= fitness_value_child_2;
							end
						end else begin
							transmission_ind_1_o <= 1;
							unchanged_ind_o <= 1;
							state_parents <= PARENT_2_UNCHANGED;
						end
					end
				end

				START_NEW_INPUT_LINE_PARENT_1: begin
					input_reading_done <= 0;
					if(start_parent_1_i) begin
						state_parents <= READING_INPUT_VALUES_PARENT_1;
					end else if (finished_parent_1_i) begin
						state_parents <= BEGIN_PARENT_2;
					end else begin
						state_parents <= START_NEW_INPUT_LINE_PARENT_1;
					end
				end

				WAIT_STATE_1: begin
					state_parents <= START_NEW_INPUT_LINE_PARENT_1;
					input_reading_done <= 0;
				end

				READING_INPUT_VALUES_PARENT_1: begin
					if(incoming_data_counter < lpNumOfEntriesPerMemLine) begin
						parent_1_tour[city_1_load_counter] <= pop_data_i[71-incoming_data_counter*lpCityEntryBitWidth-:lpCityEntryBitWidth];
						city_1_load_counter++;
						incoming_data_counter++;
						state_parents <= READING_INPUT_VALUES_PARENT_1;
					end else begin
						input_reading_done <= 1;
						incoming_data_counter <= 0;
						state_parents <= WAIT_STATE_1;
					end
				end

				START_NEW_INPUT_LINE_PARENT_2: begin
					input_reading_done <= 0;
					if(start_parent_2_i) begin
						state_parents <= READING_INPUT_VALUES_PARENT_2;
					end else if (finished_parent_1_i) begin
						state_parents <= IDLE_PARENT;
					end else begin
						state_parents <= START_NEW_INPUT_LINE_PARENT_2;
					end
				end

				WAIT_STATE_2: begin
					state_parents <= START_NEW_INPUT_LINE_PARENT_2;
					input_reading_done <= 0;
				end

				READING_INPUT_VALUES_PARENT_2: begin
					if(incoming_data_counter < lpNumOfEntriesPerMemLine) begin
						parent_2_tour[city_2_load_counter] <= pop_data_i[71-incoming_data_counter*lpCityEntryBitWidth-:lpCityEntryBitWidth];
						city_2_load_counter++;
						incoming_data_counter++;
						state_parents <= READING_INPUT_VALUES_PARENT_2;
					end else begin
						input_reading_done <= 1;
						incoming_data_counter <= 0;
						state_parents <= WAIT_STATE_2;
					end
				end

				BEGIN_PARENT_2: begin
					// if signal comes to load parent 2, begin to store
					if(start_parent_2_i) begin
						fitness_value_parent_2 <= pop_data_i[71:71-lpCityEntryBitWidth]/2;
						state_parents <= READING_INPUT_VALUES_PARENT_2;
						incoming_data_counter <= 1;
						city_2_load_counter <= 0;
					end
				end

				PARENT_2_UNCHANGED: begin
					transmission_ind_1_o <= 0;
					transmission_ind_2_o <= 1;
					unchanged_ind_o <= 1;
					//crossover_done <= 1;
					state_parents <= IDLE_PARENT;
				end

				TRANSMIT_CHILD_1_BEGIN: begin
					if(transmission_ind_1_o) begin
						transmission_ind_1_o <= 0;
						transmission_ind_2_o <= 1;
					end else begin
						transmission_ind_1_o <= 1;
						transmission_ind_2_o <= 0;
					end
					unchanged_ind_o <= 0;
					pop_data_o[71-:lpCityEntryBitWidth] <= fitness_value_child_1;
					for(int cityCounter = 1; cityCounter < lpNumOfEntriesPerMemLine; cityCounter++) begin
						pop_data_o[71-lpCityEntryBitWidth*cityCounter-:lpCityEntryBitWidth] <= child_1_tour[cityCounter-1];
					end
					cityCounterParents <= lpNumOfEntriesPerMemLine - 1;
					state_parents <= TRANSMIT_CHILD_1_REST;
				end

				TRANSMIT_CHILD_1_REST: begin
					for(int cityCounter = 0; cityCounter < lpNumOfEntriesPerMemLine; cityCounter++) begin
						pop_data_o[71-cityCounter*lpCityEntryBitWidth-:lpCityEntryBitWidth] <= child_1_tour[cityCounterParents+cityCounter];
					end
					if(cityCounterParents < pNumCities - lpNumOfEntriesPerMemLine) begin
						state_parents <= TRANSMIT_CHILD_1_REST;
						cityCounterParents <= cityCounterParents + lpNumOfEntriesPerMemLine;
					end else begin
						cityCounterParents <= 0;
						if(second_child_to_be_transmitted) begin
							state_parents <= TRANSMIT_CHILD_2_BEGIN;
						end else begin
							state_parents <= IDLE_PARENT;
							//crossover_done <= 1;
						end
					end
				end

				TRANSMIT_CHILD_2_BEGIN: begin
					if(transmission_ind_1_o) begin
						transmission_ind_1_o <= 0;
						transmission_ind_2_o <= 1;
					end else begin
						transmission_ind_1_o <= 1;
						transmission_ind_2_o <= 0;
					end
					unchanged_ind_o <= 0;
					pop_data_o[71-:lpCityEntryBitWidth] <= fitness_value_child_2;
					for(int cityCounter = 1; cityCounter < lpNumOfEntriesPerMemLine; cityCounter++) begin
						pop_data_o[71-lpCityEntryBitWidth*cityCounter-:lpCityEntryBitWidth] <= child_2_tour[cityCounter-1];
					end
					cityCounterParents <= lpNumOfEntriesPerMemLine - 1;
					state_parents <= TRANSMIT_CHILD_2_REST;
				end

				TRANSMIT_CHILD_2_REST: begin
					for(int cityCounter = 0; cityCounter < lpNumOfEntriesPerMemLine; cityCounter++) begin
						pop_data_o[71-cityCounter*lpCityEntryBitWidth-:lpCityEntryBitWidth] <= child_2_tour[cityCounterParents +cityCounter];
					end
					if(cityCounterParents < pNumCities - lpNumOfEntriesPerMemLine) begin
						state_parents <= TRANSMIT_CHILD_2_REST;
						cityCounterParents <= cityCounterParents + lpNumOfEntriesPerMemLine;
					end else begin
						//crossover_done <= 1;
						second_child_to_be_transmitted <= 0;
						cityCounterParents <= 0;
						state_parents <= IDLE_PARENT;
					end
				end
			endcase
		end
	end



	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//
	// FSM for calculation of the first offspring
	//
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	always @ (posedge clk_i) begin
		if(rst_i) begin
			fitness_value_child_1 <= 0;
			for(int cityCounter = 0; cityCounter < pNumCities; cityCounter++) begin
				child_1_tour[cityCounter] <= 0;
			end
			city_selector_child_1_counter_1 <= 0;
			city_selector_child_1_counter_2 <= 0;
			capacity_counter_child_1 <= 0;
			child_1_iterator <= 0;
			child_1_swap_counter <= 0; 
			child_1_return <= 0; 
			child_1_improvement_made <= 0; 
			
			first_mutation_point_child_1 <= 0; 
			mutation_length_child_1 <= 0; 

			// Reset the state reg
			state_child_1 <= IDLE;
			
			start_child_1 <= 0; 

			// Distance child 1 reset
			dist_child_1 <= 0;
			child_1_depot_second <= 0;

			// Resetting the finished signal
			child_1_finished <= 0;

			// Resetting the init mode reg
			init_mode_set_child_1 <= 0;

			// Resetting the mutation number counter for the init mode
			mutation_process_counter_child_1 <= 0;

			// Resetting the counters that are necessary for the 2-opt and distance calculation
			x1_1_child_1 <= 0;
			x1_2_child_1 <= 0;
			y1_1_child_1 <= 0;
			y1_2_child_1 <= 0;

			x2_1_child_1 <= 0;
			y2_1_child_1 <= 0;
			x2_2_child_1 <= 0;
			y2_2_child_1 <= 0;

			x3_1_child_1 <= 0;
			y3_1_child_1 <= 0;
			x3_2_child_1 <= 0;
			y3_2_child_1 <= 0;

			x4_1_child_1 <= 0;
			y4_1_child_1 <= 0;
			x4_2_child_1 <= 0;
			y4_2_child_1 <= 0;

			x5_1_child_1 <= 0;
			y5_1_child_1 <= 0;
			x5_2_child_1 <= 0;
			y5_2_child_1 <= 0;

		end else begin
			case(state_child_1)
				// Waiting until parents are loaded to begin with crossover - first insert random subtour into the first children
				IDLE: begin
					child_1_finished <= 0;
					if(finished_parent_2_i) begin
						state_child_1 <= LOAD_CHILD;
						city_selector_child_1_counter_1 <= 0;
						city_selector_child_1_counter_2 <= 0;
						child_1_iterator <= 0;
						capacity_counter_child_1 <= 0;
						init_mode_set_child_1 <= init_mode_i;
						mutation_process_counter_child_1 <= 0;
						fitness_value_child_1 <= 0;
						start_child_1 <= 0; 
					end
				end

				LOAD_CHILD: begin
					if(city_selector_child_1_counter_1 < crossover_1_length_i) begin
						child_1_tour[city_selector_child_1_counter_1] <= parent_1_tour[crossover_point_child_1 + city_selector_child_1_counter_1];
						city_selector_child_1_counter_1 <= city_selector_child_1_counter_1 + 1;
					end else begin
						state_child_1 <= INSERT_REST_NODES;
						city_selector_child_1_counter_1 <= crossover_1_length_i;
						city_selector_child_1_counter_2 <= 0;
						child_1_iterator <= 0;
						capacity_counter_child_1 <= 0;
						init_mode_set_child_1 <= init_mode_i;
						mutation_process_counter_child_1 <= 0;
						fitness_value_child_1 <= 0;
					end
				end

				INSERT_REST_NODES: begin
					child_1_tour[city_selector_child_1_counter_1] <= parent_2_tour[city_selector_child_1_counter_2];
					if(node_already_in_child_1 != 0) begin
						city_selector_child_1_counter_1 <= city_selector_child_1_counter_1;
					end else begin
						city_selector_child_1_counter_1 <= city_selector_child_1_counter_1 + 1;
					end
					city_selector_child_1_counter_2 <= city_selector_child_1_counter_2 + 1;
					if(city_selector_child_1_counter_2 == pNumCities) begin
                        if(do_mutation_i || init_mode_set_child_1) begin
						    state_child_1 <= PERFORM_MUTATION;
						    first_mutation_point_child_1 <= random_city_number_mutation_1_i;
						    mutation_length_child_1 <= mutation_length_1_i; 
                        end else begin
                            state_child_1 <= DEPOT_INSERTION;
                            city_selector_child_1_counter_1 <= 0;
                            capacity_counter_child_1 <= 0;
                        end
					end else begin
						state_child_1 <= INSERT_REST_NODES;
					end
				end

				PERFORM_MUTATION: begin
					if(city_selector_child_1_counter_1 < mutation_length_child_1/2) begin
						child_1_tour[first_mutation_point_child_1 + city_selector_child_1_counter_1] <= child_1_tour[first_mutation_point_child_1 + mutation_length_child_1 - city_selector_child_1_counter_1 - 1];
						child_1_tour[first_mutation_point_child_1 + mutation_length_child_1 - city_selector_child_1_counter_1 - 1] <= child_1_tour[first_mutation_point_child_1 + city_selector_child_1_counter_1];
						city_selector_child_1_counter_1 <= city_selector_child_1_counter_1 + 1;
					end else begin
						city_selector_child_1_counter_1 <= 0;
						if(init_mode_set_child_1 && mutation_process_counter_child_1 < pInitMutationProcesses) begin
							state_child_1 <= PERFORM_MUTATION;
							first_mutation_point_child_1 <= random_city_number_mutation_1_i;
							mutation_length_child_1 <= mutation_length_1_i; 
							mutation_process_counter_child_1 <= mutation_process_counter_child_1 + 1;
						end else begin
							state_child_1 <= DEPOT_INSERTION;
							city_selector_child_1_counter_1 <= 0;
							capacity_counter_child_1 <= 0;
						end
					end
				end

				DEPOT_INSERTION: begin
					if(capacity_counter_child_1 <= pVehicleCapacity) begin
						capacity_counter_child_1 <= capacity_counter_child_1 + child_1_tour[city_selector_child_1_counter_1].nodeCapacity;
						// Resetting the depot entry just to make sure that there are no unwanted depot stops in the tour
						child_1_tour[city_selector_child_1_counter_1].depot <= 0;
						city_selector_child_1_counter_1 <= city_selector_child_1_counter_1 + 1;
					end else begin
						capacity_counter_child_1 <= 0;
						city_selector_child_1_counter_1 <= city_selector_child_1_counter_1 - 1;
						child_1_tour[city_selector_child_1_counter_1 - 2].depot <= 1;
					end

					// Check whether all cities in the tour have been checked
					if(city_selector_child_1_counter_1 < pNumCities) begin
						state_child_1 <= DEPOT_INSERTION;
					end else begin
						capacity_counter_child_1 <= 0;
						child_1_tour[pNumCities-1].depot <= 1;
						dist_child_1 <= 0;

						if(!child_1_tour[0].depot) begin
							x1_1_child_1 <= lpDepotXCoord;
							x1_2_child_1 <= child_1_tour[0].nodeID.xCoordinate;
							y1_1_child_1 <= lpDepotYCoord;
							y1_2_child_1 <= child_1_tour[0].nodeID.yCoordinate;

							x2_1_child_1 <= child_1_tour[0].nodeID.xCoordinate;
							y2_1_child_1 <= child_1_tour[0].nodeID.yCoordinate;
							x2_2_child_1 <= child_1_tour[1].nodeID.xCoordinate;
							y2_2_child_1 <= child_1_tour[1].nodeID.yCoordinate;

							x5_1_child_1 <= lpDepotXCoord;
							y5_1_child_1 <= lpDepotYCoord;
							x5_2_child_1 <= child_1_tour[1].nodeID.xCoordinate;
							y5_2_child_1 <= child_1_tour[1].nodeID.yCoordinate;

							x3_1_child_1 <= child_1_tour[1].nodeID.xCoordinate;
							y3_1_child_1 <= child_1_tour[1].nodeID.yCoordinate;

							x4_1_child_1 <= child_1_tour[0].nodeID.xCoordinate;
							y4_1_child_1 <= child_1_tour[0].nodeID.yCoordinate;

							if(child_1_tour[1].depot) begin
								x3_2_child_1 <= lpDepotXCoord;
								y3_2_child_1 <= lpDepotYCoord;

								x4_2_child_1 <= lpDepotXCoord;
								y4_2_child_1 <= lpDepotYCoord;
							end else begin
								x3_2_child_1 <= child_1_tour[2].nodeID.xCoordinate;
								y3_2_child_1 <= child_1_tour[2].nodeID.yCoordinate;

								x4_2_child_1 <= child_1_tour[2].nodeID.xCoordinate;
								y4_2_child_1 <= child_1_tour[2].nodeID.yCoordinate;
							end
							city_selector_child_1_counter_1 <= 0;
						    child_1_return <= 0;
						    child_1_improvement_made <= 0; 
							city_selector_child_1_counter_2 <= 1;
							state_child_1 <= TWO_OPT_EVALUATION;
						end else begin
							city_selector_child_1_counter_1 <= 1;
							child_1_return <= 1; 
							child_1_improvement_made <= 0; 
							city_selector_child_1_counter_2 <= 2;
							state_child_1 <= TWO_OPT_CALCULATION;
						end
					end
				end

				TWO_OPT_EVALUATION: begin
					if(part_dist_1_child_1 + part_dist_3_child_1 > part_dist_5_child_1 + part_dist_4_child_1) begin
						if(!child_1_tour[city_selector_child_1_counter_2].depot) begin
							child_1_tour[city_selector_child_1_counter_1] <= child_1_tour[city_selector_child_1_counter_2];
							child_1_tour[city_selector_child_1_counter_2] <= child_1_tour[city_selector_child_1_counter_1];
						end else begin
							child_1_tour[city_selector_child_1_counter_1].nodeID <= child_1_tour[city_selector_child_1_counter_2].nodeID;
							child_1_tour[city_selector_child_1_counter_1].nodeCapacity <= child_1_tour[city_selector_child_1_counter_2].nodeCapacity;
							child_1_tour[city_selector_child_1_counter_1].depot <= 0;

							child_1_tour[city_selector_child_1_counter_2].nodeID <= child_1_tour[city_selector_child_1_counter_1].nodeID;
							child_1_tour[city_selector_child_1_counter_2].nodeCapacity <= child_1_tour[city_selector_child_1_counter_1].nodeCapacity;
							child_1_tour[city_selector_child_1_counter_2].depot <= 1;
						end
						state_child_1 <= TWO_OPT_SWAP;
						child_1_improvement_made <= 1; 
						child_1_swap_counter <= 1;
					end else begin 
                        if(child_1_tour[city_selector_child_1_counter_2].depot) begin
                            city_selector_child_1_counter_1 <= city_selector_child_1_counter_1 + 1;
                            city_selector_child_1_counter_2 <= city_selector_child_1_counter_1 + 2;
                        end else begin
                            city_selector_child_1_counter_2 <= city_selector_child_1_counter_2 + 1;
                        end
                        state_child_1 <= TWO_OPT_CALCULATION;
					end
				end
				
				TWO_OPT_SWAP: begin
				    if(city_selector_child_1_counter_1 + child_1_swap_counter < city_selector_child_1_counter_2 - child_1_swap_counter) begin
				        child_1_tour[city_selector_child_1_counter_1 + child_1_swap_counter] <= child_1_tour[city_selector_child_1_counter_2 - child_1_swap_counter];
				        child_1_tour[city_selector_child_1_counter_2 - child_1_swap_counter] <= child_1_tour[city_selector_child_1_counter_1 + child_1_swap_counter];
				        child_1_swap_counter <= child_1_swap_counter + 1;
				        state_child_1 <= TWO_OPT_SWAP; 
				    end else begin 
				        if(child_1_tour[city_selector_child_1_counter_2].depot) begin
						  city_selector_child_1_counter_1 <= city_selector_child_1_counter_1 + 1;
						  city_selector_child_1_counter_2 <= city_selector_child_1_counter_1 + 2;
					    end else begin
						  city_selector_child_1_counter_2 <= city_selector_child_1_counter_2 + 1;
					    end
					    state_child_1 <= TWO_OPT_CALCULATION;
				    end
				end 

				TWO_OPT_CALCULATION: begin
					if(city_selector_child_1_counter_1 == (pNumCities-1)) begin
                        if(child_1_improvement_made == 0) begin 
                            state_child_1 <= DISTANCE_CALCULATION_WAIT_STATE;
                            start_child_1 <= 1; 
						    x1_1_child_1 <= lpDepotXCoord;
						    y1_1_child_1 <= lpDepotYCoord;
						    x1_2_child_1 <= child_1_tour[0].nodeID.xCoordinate;
						    y1_2_child_1 <= child_1_tour[0].nodeID.yCoordinate;
						    city_selector_child_1_counter_1 <= 0;
                        end else begin 
                            city_selector_child_1_counter_1 <= child_1_return; 
                            city_selector_child_1_counter_2 <= child_1_return + 1; 
                            child_1_improvement_made <= 0; 
                            state_child_1 <= TWO_OPT_CALCULATION; 
                        end 
						
					end else begin
						if(child_1_tour[city_selector_child_1_counter_1].depot) begin
						    if(child_1_improvement_made == 0) begin 
							 city_selector_child_1_counter_1 <= city_selector_child_1_counter_1 + 1;
							 city_selector_child_1_counter_2 <= city_selector_child_1_counter_1 + 2;
							 child_1_return <= city_selector_child_1_counter_1 + 1; 
							 state_child_1 <= TWO_OPT_CALCULATION;
							end else begin 
							 city_selector_child_1_counter_1 <= child_1_return;
							 city_selector_child_1_counter_2 <= child_1_return + 1; 
							 child_1_improvement_made <= 0; 
							 state_child_1 <= TWO_OPT_CALCULATION; 
							end 
						end else begin
							state_child_1 <= TWO_OPT_EVALUATION;
							if(child_1_tour[city_selector_child_1_counter_1-1].depot) begin
								x1_1_child_1 <= lpDepotXCoord;
								y1_1_child_1 <= lpDepotYCoord;

								x5_1_child_1 <= lpDepotXCoord;
								y5_1_child_1 <= lpDepotYCoord;
							end else begin
								x1_1_child_1 <= child_1_tour[city_selector_child_1_counter_1-1].nodeID.xCoordinate;
								y1_1_child_1 <= child_1_tour[city_selector_child_1_counter_1-1].nodeID.yCoordinate;

								x5_1_child_1 <= child_1_tour[city_selector_child_1_counter_1-1].nodeID.xCoordinate;
								y5_1_child_1 <= child_1_tour[city_selector_child_1_counter_1-1].nodeID.yCoordinate;
							end

							x1_2_child_1 <= child_1_tour[city_selector_child_1_counter_1].nodeID.xCoordinate;
							y1_2_child_1 <= child_1_tour[city_selector_child_1_counter_1].nodeID.yCoordinate;

							x4_1_child_1 <= child_1_tour[city_selector_child_1_counter_1].nodeID.xCoordinate;
							y4_1_child_1 <= child_1_tour[city_selector_child_1_counter_1].nodeID.yCoordinate;

							x2_1_child_1 <= child_1_tour[city_selector_child_1_counter_1].nodeID.xCoordinate;
							y2_1_child_1 <= child_1_tour[city_selector_child_1_counter_1].nodeID.yCoordinate;

							x2_2_child_1 <= child_1_tour[city_selector_child_1_counter_1 + 1].nodeID.xCoordinate;
							y2_2_child_1 <= child_1_tour[city_selector_child_1_counter_1 + 1].nodeID.yCoordinate;

							x5_2_child_1 <= child_1_tour[city_selector_child_1_counter_2].nodeID.xCoordinate;
							y5_2_child_1 <= child_1_tour[city_selector_child_1_counter_2].nodeID.yCoordinate;

							x3_1_child_1 <= child_1_tour[city_selector_child_1_counter_2].nodeID.xCoordinate;
							y3_1_child_1 <= child_1_tour[city_selector_child_1_counter_2].nodeID.yCoordinate;

							if(child_1_tour[city_selector_child_1_counter_2].depot) begin
								x3_2_child_1 <= lpDepotXCoord;
								y3_2_child_1 <= lpDepotYCoord;

								x4_2_child_1 <= lpDepotXCoord;
								y4_2_child_1 <= lpDepotYCoord;
							end else begin
								x3_2_child_1 <= child_1_tour[city_selector_child_1_counter_2+1].nodeID.xCoordinate;
								y3_2_child_1 <= child_1_tour[city_selector_child_1_counter_2+1].nodeID.yCoordinate;

								x4_2_child_1 <= child_1_tour[city_selector_child_1_counter_2+1].nodeID.xCoordinate;
								y4_2_child_1 <= child_1_tour[city_selector_child_1_counter_2+1].nodeID.yCoordinate;
							end
						end
					end
				end

				DISTANCE_CALCULATION: begin
					if(city_selector_child_1_counter_1 == (pNumCities - 1)) begin
					    start_child_1 <= 1; 
						state_child_1 <= DISTANCE_CALCULATION_INTERMEDIATE_FINAL_WAIT_STATE;
						x1_1_child_1 <= lpDepotXCoord;
						y1_1_child_1 <= lpDepotYCoord;
						x1_2_child_1 <= child_1_tour[pNumCities-1].nodeID.xCoordinate;
						y1_2_child_1 <= child_1_tour[pNumCities-1].nodeID.yCoordinate;
					end else begin
						if(child_1_tour[city_selector_child_1_counter_1].depot) begin
							x1_1_child_1 <= child_1_tour[city_selector_child_1_counter_1].nodeID.xCoordinate;
							y1_1_child_1 <= child_1_tour[city_selector_child_1_counter_1].nodeID.yCoordinate;
							x1_2_child_1 <= lpDepotXCoord;
							y1_2_child_1 <= lpDepotYCoord;
							start_child_1 <= 1; 
							state_child_1 <= DISTANCE_CALCULATION_DEPOT_STOP_WAIT_STATE;
						end else begin
							x1_1_child_1 <= child_1_tour[city_selector_child_1_counter_1].nodeID.xCoordinate;
							y1_1_child_1 <= child_1_tour[city_selector_child_1_counter_1].nodeID.yCoordinate;
							x1_2_child_1 <= child_1_tour[city_selector_child_1_counter_1+1].nodeID.xCoordinate;
							y1_2_child_1 <= child_1_tour[city_selector_child_1_counter_1+1].nodeID.yCoordinate;
							city_selector_child_1_counter_1 <= city_selector_child_1_counter_1 + 1;
							start_child_1 <= 1; 
							state_child_1 <= DISTANCE_CALCULATION_WAIT_STATE;
						end
					end
					if(rem_child_1 < root_child_1) begin 
					   fitness_value_child_1 <= fitness_value_child_1 + root_child_1;
					end else begin 
					   fitness_value_child_1 <= fitness_value_child_1 + root_child_1 + 1; 
					end
				end
				
				DISTANCE_CALCULATION_WAIT_STATE: begin 
				    start_child_1 <= 0; 
				    if(valid_child_1) begin 
				        state_child_1 <= DISTANCE_CALCULATION; 
				    end 
				end 
				
				DISTANCE_CALCULATION_DEPOT_STOP_WAIT_STATE: begin 
				    start_child_1 <= 0; 
				    if(valid_child_1) begin 
				        state_child_1 <= DISTANCE_CALCULATION_DEPOT_STOP; 
				    end 
				end 
				
				DISTANCE_CALCULATION_INTERMEDIATE_FINAL_WAIT_STATE: begin 
				    start_child_1 <= 0; 
				    if(valid_child_1) begin 
				        state_child_1 <= DISTANCE_CALCULATION_INTERMEDIATE_FINAL; 
				    end 
				end

				DISTANCE_CALCULATION_DEPOT_STOP: begin
				    x1_1_child_1 <= lpDepotXCoord;
				    y1_1_child_1 <= lpDepotYCoord;
				    x1_2_child_1 <= child_1_tour[city_selector_child_1_counter_1+1].nodeID.xCoordinate;
				    y1_2_child_1 <= child_1_tour[city_selector_child_1_counter_1+1].nodeID.yCoordinate;
				    city_selector_child_1_counter_1 <= city_selector_child_1_counter_1 + 1;
				    if(rem_child_1 < root_child_1) begin 
					   fitness_value_child_1 <= fitness_value_child_1 + root_child_1;
					end else begin 
					   fitness_value_child_1 <= fitness_value_child_1 + root_child_1 + 1; 
					end
					start_child_1 <= 1; 
				    state_child_1 <= DISTANCE_CALCULATION_WAIT_STATE;
				end

				DISTANCE_CALCULATION_INTERMEDIATE_FINAL: begin
					if(rem_child_1 < root_child_1) begin 
					   fitness_value_child_1 <= fitness_value_child_1 + root_child_1;
					end else begin 
					   fitness_value_child_1 <= fitness_value_child_1 + root_child_1 + 1; 
					end
					state_child_1 <= DISTANCE_CALCULATION_FINAL;
				end

				DISTANCE_CALCULATION_FINAL: begin
					child_1_finished <= 1;
					if(state_parents != IDLE_PARENT) begin
						state_child_1 <= IDLE;
						city_selector_child_1_counter_1 <= 0;
					end
				end
			endcase
		end
	end


	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//
	// FSM for calculation of the second offspring
	//
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	always @ (posedge clk_i) begin
		if(rst_i) begin
			fitness_value_child_2 <= 0;
			for(int cityCounter = 0; cityCounter < pNumCities; cityCounter++) begin
				child_2_tour[cityCounter] <= 0;
			end
			city_selector_child_2_counter_1 <= 0;
			city_selector_child_2_counter_2 <= 0;
			child_2_swap_counter <= 0; 
			capacity_counter_child_2 <= 0;
			child_2_iterator <= 0;
			child_2_return <= 0; 
			child_2_improvement_made <= 0; 
			
			// Reset the state reg
			state_child_2 <= IDLE;
			
			start_child_2 <= 0; 
			
			first_mutation_point_child_2 <= 0; 
			mutation_length_child_2 <= 0; 

			// Distance child 1 reset
			dist_child_2 <= 0;
			child_2_depot_second <= 0;

			// Resetting the finished signal for child 2
			child_2_finished <= 0;

			// Resetting the init mode reg
			init_mode_set_child_2 <= 0;

			// Resetting the mutation number counter for the init mode
			mutation_process_counter_child_2 <= 0;

			// Resetting the counters that are necessary for the 2-opt and distance calculation
			x1_1_child_2 <= 0;
			x1_2_child_2 <= 0;
			y1_1_child_2 <= 0;
			y1_2_child_2 <= 0;

			x2_1_child_2 <= 0;
			y2_1_child_2 <= 0;
			x2_2_child_2 <= 0;
			y2_2_child_2 <= 0;

			x3_1_child_2 <= 0;
			y3_1_child_2 <= 0;
			x3_2_child_2 <= 0;
			y3_2_child_2 <= 0;

			x4_1_child_2 <= 0;
			y4_1_child_2 <= 0;
			x4_2_child_2 <= 0;
			y4_2_child_2 <= 0;

			x5_1_child_2 <= 0;
			y5_1_child_2 <= 0;
			x5_2_child_2 <= 0;
			y5_2_child_2 <= 0;

		end else begin
			case(state_child_2)
				// Waiting until parents are loaded to begin with crossover - first insert random subtour into the first children
				IDLE: begin
				    child_2_finished <= 0;
					if(finished_parent_2_i) begin
						state_child_2 <= LOAD_CHILD;
						city_selector_child_2_counter_1 <= 0;
						city_selector_child_2_counter_2 <= 0;
						child_2_iterator <= 0;
						init_mode_set_child_2 <= init_mode_i;
						mutation_process_counter_child_2 <= 0;
						capacity_counter_child_2 <= 0;
						fitness_value_child_2 <= 0;
						start_child_2 <= 0; 
					end
				end

				LOAD_CHILD: begin
					if(city_selector_child_2_counter_1 < crossover_2_length_i) begin
						child_2_tour[city_selector_child_2_counter_1] <= parent_2_tour[crossover_point_child_2 + city_selector_child_2_counter_1];
						city_selector_child_2_counter_1 <= city_selector_child_2_counter_1 + 1;
					end else begin
						state_child_2 <= INSERT_REST_NODES;
						city_selector_child_2_counter_1 <= crossover_2_length_i;
						city_selector_child_2_counter_2 <= 0;
						child_2_iterator <= 0;
						init_mode_set_child_2 <= init_mode_i;
						mutation_process_counter_child_2 <= 0;
						capacity_counter_child_2 <= 0;
						fitness_value_child_2 <= 0;
					end
				end

				INSERT_REST_NODES: begin
					child_2_tour[city_selector_child_2_counter_1] <= parent_1_tour[city_selector_child_2_counter_2];
					if(node_already_in_child_2 != 0) begin
						city_selector_child_2_counter_1 <= city_selector_child_2_counter_1;
					end else begin
						city_selector_child_2_counter_1 <= city_selector_child_2_counter_1 + 1;
					end
					city_selector_child_2_counter_2 <= city_selector_child_2_counter_2 + 1;
					if(city_selector_child_2_counter_2 == pNumCities) begin
                        if(do_mutation_i || init_mode_set_child_2) begin
						    state_child_2 <= PERFORM_MUTATION;
						    first_mutation_point_child_2 <= random_city_number_mutation_2_i;
						    mutation_length_child_2 <= mutation_length_2_i; 
                        end else begin
                            state_child_2 <= DEPOT_INSERTION;
                            city_selector_child_2_counter_1 <= 0;
                            capacity_counter_child_2 <= 0;
                        end
					end else begin
						state_child_2 <= INSERT_REST_NODES;
					end
				end

				PERFORM_MUTATION: begin
					if(city_selector_child_2_counter_1 < mutation_length_child_2/2) begin
						child_2_tour[first_mutation_point_child_2 + city_selector_child_2_counter_1] <= child_2_tour[first_mutation_point_child_2 + mutation_length_child_2 - city_selector_child_2_counter_1 - 1];
						child_2_tour[first_mutation_point_child_2 + mutation_length_child_2 - city_selector_child_2_counter_1 - 1] <= child_2_tour[first_mutation_point_child_2 + city_selector_child_2_counter_1];
						city_selector_child_2_counter_1 <= city_selector_child_2_counter_1 + 1;
					end else begin
						city_selector_child_2_counter_1 <= 0;
						if(init_mode_set_child_2 && mutation_process_counter_child_2 < pInitMutationProcesses) begin
							state_child_2 <= PERFORM_MUTATION;
							first_mutation_point_child_2 <= random_city_number_mutation_2_i;
							mutation_length_child_2 <= mutation_length_2_i; 
							mutation_process_counter_child_2 <= mutation_process_counter_child_2 + 1;
						end else begin
							state_child_2 <= DEPOT_INSERTION;
							city_selector_child_2_counter_1 <= 0;
                            capacity_counter_child_2 <= 0;
						end
					end
				end

				DEPOT_INSERTION: begin
					if(capacity_counter_child_2 <= pVehicleCapacity) begin
						capacity_counter_child_2 <= capacity_counter_child_2 + child_2_tour[city_selector_child_2_counter_1].nodeCapacity;
						// Resetting the depot entry just to make sure that there are no unwanted depot stops in the tour
						child_2_tour[city_selector_child_2_counter_1].depot <= 0;
						city_selector_child_2_counter_1 <= city_selector_child_2_counter_1 + 1;
					end else begin
						capacity_counter_child_2 <= 0;
						city_selector_child_2_counter_1 <= city_selector_child_2_counter_1 - 1;
						child_2_tour[city_selector_child_2_counter_1 - 2].depot <= 1;
					end

					// Check whether all cities in the tour have been checked
					if(city_selector_child_2_counter_1 < pNumCities) begin
						state_child_2 <= DEPOT_INSERTION;
					end else begin
						child_2_tour[pNumCities-1].depot <= 1;
						capacity_counter_child_2 <= 0;
						dist_child_2 <= 0;

						if(!child_2_tour[0].depot) begin
							x1_1_child_2 <= lpDepotXCoord;
							x1_2_child_2 <= child_2_tour[0].nodeID.xCoordinate;
							y1_1_child_2 <= lpDepotYCoord;
							y1_2_child_2 <= child_2_tour[0].nodeID.yCoordinate;

							x2_1_child_2 <= child_2_tour[0].nodeID.xCoordinate;
							y2_1_child_2 <= child_2_tour[0].nodeID.yCoordinate;
							x2_2_child_2 <= child_2_tour[1].nodeID.xCoordinate;
							y2_2_child_2 <= child_2_tour[1].nodeID.yCoordinate;

							x5_1_child_2 <= lpDepotXCoord;
							y5_1_child_2 <= lpDepotYCoord;
							x5_2_child_2 <= child_2_tour[1].nodeID.xCoordinate;
							y5_2_child_2 <= child_2_tour[1].nodeID.yCoordinate;

							x3_1_child_2 <= child_2_tour[1].nodeID.xCoordinate;
							y3_1_child_2 <= child_2_tour[1].nodeID.yCoordinate;

							x4_1_child_2 <= child_2_tour[0].nodeID.xCoordinate;
							y4_1_child_2 <= child_2_tour[0].nodeID.yCoordinate;

							if(child_2_tour[1].depot) begin
								x3_2_child_2 <= lpDepotXCoord;
								y3_2_child_2 <= lpDepotYCoord;

								x4_2_child_2 <= lpDepotXCoord;
								y4_2_child_2 <= lpDepotYCoord;
							end else begin
								x3_2_child_2 <= child_2_tour[2].nodeID.xCoordinate;
								y3_2_child_2 <= child_2_tour[2].nodeID.yCoordinate;

								x4_2_child_2 <= child_2_tour[2].nodeID.xCoordinate;
								y4_2_child_2 <= child_2_tour[2].nodeID.yCoordinate;
							end
							city_selector_child_2_counter_1 <= 0;
							city_selector_child_2_counter_2 <= 1;
							child_2_return <= 0; 
							child_2_improvement_made <= 0; 
							state_child_2 <= TWO_OPT_EVALUATION;
						end else begin
							city_selector_child_2_counter_1 <= 1;
							city_selector_child_2_counter_2 <= 2;
							child_2_return <= 1; 
							child_2_improvement_made <= 0; 
							state_child_2 <= TWO_OPT_CALCULATION;
						end
					end
				end

				TWO_OPT_EVALUATION: begin
					if(part_dist_1_child_2 + part_dist_3_child_2 > part_dist_5_child_2 + part_dist_4_child_2) begin
						if(!child_2_tour[city_selector_child_2_counter_2].depot) begin
							child_2_tour[city_selector_child_2_counter_1] <= child_2_tour[city_selector_child_2_counter_2];
							child_2_tour[city_selector_child_2_counter_2] <= child_2_tour[city_selector_child_2_counter_1];
						end else begin
							child_2_tour[city_selector_child_2_counter_1].nodeID <= child_2_tour[city_selector_child_2_counter_2].nodeID;
							child_2_tour[city_selector_child_2_counter_1].nodeCapacity <= child_2_tour[city_selector_child_2_counter_2].nodeCapacity;
							child_2_tour[city_selector_child_2_counter_1].depot <= 0;

							child_2_tour[city_selector_child_2_counter_2].nodeID <= child_2_tour[city_selector_child_2_counter_1].nodeID;
							child_2_tour[city_selector_child_2_counter_2].nodeCapacity <= child_2_tour[city_selector_child_2_counter_1].nodeCapacity;
							child_2_tour[city_selector_child_2_counter_2].depot <= 1;
						end
						state_child_2 <= TWO_OPT_SWAP;
						child_2_improvement_made <= 1; 
						child_2_swap_counter <= 1;
					end else begin 
                        if(child_2_tour[city_selector_child_2_counter_2].depot) begin
                            city_selector_child_2_counter_1 <= city_selector_child_2_counter_1 + 1;
                            city_selector_child_2_counter_2 <= city_selector_child_2_counter_1 + 2;
                        end else begin
                            city_selector_child_2_counter_2 <= city_selector_child_2_counter_2 + 1;
                        end
                        state_child_2 <= TWO_OPT_CALCULATION;
					end
				end
				
				TWO_OPT_SWAP: begin
				    if(city_selector_child_2_counter_1 + child_2_swap_counter < city_selector_child_2_counter_2 - child_2_swap_counter) begin
				        child_2_tour[city_selector_child_2_counter_1 + child_2_swap_counter] <= child_2_tour[city_selector_child_2_counter_2 - child_2_swap_counter];
				        child_2_tour[city_selector_child_2_counter_2 - child_2_swap_counter] <= child_2_tour[city_selector_child_2_counter_1 + child_2_swap_counter];
				        child_2_swap_counter <= child_2_swap_counter + 1;
				        state_child_2 <= TWO_OPT_SWAP; 
				    end else begin 
				        if(child_2_tour[city_selector_child_2_counter_2].depot) begin
						  city_selector_child_2_counter_1 <= city_selector_child_2_counter_1 + 1;
						  city_selector_child_2_counter_2 <= city_selector_child_2_counter_1 + 2;
					    end else begin
						  city_selector_child_2_counter_2 <= city_selector_child_2_counter_2 + 1;
					    end
					    state_child_2 <= TWO_OPT_CALCULATION;
				    end
				end 

				TWO_OPT_CALCULATION: begin
					if(city_selector_child_2_counter_1 == (pNumCities-1)) begin
					   if(child_2_improvement_made == 0) begin 
                            state_child_2 <= DISTANCE_CALCULATION_WAIT_STATE;
                            start_child_2 <= 1; 
                            x1_1_child_2 <= lpDepotXCoord;
                            y1_1_child_2 <= lpDepotYCoord;
                            x1_2_child_2 <= child_2_tour[0].nodeID.xCoordinate;
                            y1_2_child_2 <= child_2_tour[0].nodeID.yCoordinate;
                            city_selector_child_2_counter_1 <= 0;
                       end else begin 
                            city_selector_child_2_counter_1 <= child_2_return;
                            city_selector_child_2_counter_2 <= child_2_return + 1; 
                            child_2_improvement_made <= 0; 
                            state_child_2 <= TWO_OPT_CALCULATION; 
                       end 
					end else begin
						if(child_2_tour[city_selector_child_2_counter_1].depot) begin
						    if(child_2_improvement_made == 0) begin 
							 city_selector_child_2_counter_1 <= city_selector_child_2_counter_1 + 1;
							 city_selector_child_2_counter_2 <= city_selector_child_2_counter_1 + 2;
							 child_2_return <= city_selector_child_2_counter_1 + 1; 
							 state_child_2 <= TWO_OPT_CALCULATION;
							end else begin 
							 city_selector_child_2_counter_1 <= child_2_return;
							 city_selector_child_2_counter_2 <= child_2_return + 1; 
							 child_2_improvement_made <= 0; 
							 state_child_2 <= TWO_OPT_CALCULATION; 
							end 
						end else begin
							state_child_2 <= TWO_OPT_EVALUATION;
							if(child_2_tour[city_selector_child_2_counter_1-1].depot) begin
								x1_1_child_2 <= lpDepotXCoord;
								y1_1_child_2 <= lpDepotYCoord;

								x5_1_child_2 <= lpDepotXCoord;
								y5_1_child_2 <= lpDepotYCoord;
							end else begin
								x1_1_child_2 <= child_2_tour[city_selector_child_2_counter_1-1].nodeID.xCoordinate;
								y1_1_child_2 <= child_2_tour[city_selector_child_2_counter_1-1].nodeID.yCoordinate;

								x5_1_child_2 <= child_2_tour[city_selector_child_2_counter_1-1].nodeID.xCoordinate;
								y5_1_child_2 <= child_2_tour[city_selector_child_2_counter_1-1].nodeID.yCoordinate;
							end

							x1_2_child_2 <= child_2_tour[city_selector_child_2_counter_1].nodeID.xCoordinate;
							y1_2_child_2 <= child_2_tour[city_selector_child_2_counter_1].nodeID.yCoordinate;

							x4_1_child_2 <= child_2_tour[city_selector_child_2_counter_1].nodeID.xCoordinate;
							y4_1_child_2 <= child_2_tour[city_selector_child_2_counter_1].nodeID.yCoordinate;

							x2_1_child_2 <= child_2_tour[city_selector_child_2_counter_1].nodeID.xCoordinate;
							y2_1_child_2 <= child_2_tour[city_selector_child_2_counter_1].nodeID.yCoordinate;

							x2_2_child_2 <= child_2_tour[city_selector_child_2_counter_1 + 1].nodeID.xCoordinate;
							y2_2_child_2 <= child_2_tour[city_selector_child_2_counter_1 + 1].nodeID.yCoordinate;

							x5_2_child_2 <= child_2_tour[city_selector_child_2_counter_2].nodeID.xCoordinate;
							y5_2_child_2 <= child_2_tour[city_selector_child_2_counter_2].nodeID.yCoordinate;

							x3_1_child_2 <= child_2_tour[city_selector_child_2_counter_2].nodeID.xCoordinate;
							y3_1_child_2 <= child_2_tour[city_selector_child_2_counter_2].nodeID.yCoordinate;

							if(child_2_tour[city_selector_child_2_counter_2].depot) begin
								x3_2_child_2 <= lpDepotXCoord;
								y3_2_child_2 <= lpDepotYCoord;

								x4_2_child_2 <= lpDepotXCoord;
								y4_2_child_2 <= lpDepotYCoord;
							end else begin
								x3_2_child_2 <= child_2_tour[city_selector_child_2_counter_2+1].nodeID.xCoordinate;
								y3_2_child_2 <= child_2_tour[city_selector_child_2_counter_2+1].nodeID.yCoordinate;

								x4_2_child_2 <= child_2_tour[city_selector_child_2_counter_2+1].nodeID.xCoordinate;
								y4_2_child_2 <= child_2_tour[city_selector_child_2_counter_2+1].nodeID.yCoordinate;
							end
						end
					end
				end

				DISTANCE_CALCULATION: begin
					if(city_selector_child_2_counter_1 == (pNumCities - 1)) begin
						state_child_2 <= DISTANCE_CALCULATION_INTERMEDIATE_FINAL_WAIT_STATE;
						start_child_2 <= 1;
						x1_1_child_2 <= lpDepotXCoord;
						y1_1_child_2 <= lpDepotYCoord;
						x1_2_child_2 <= child_2_tour[pNumCities-1].nodeID.xCoordinate;
						y1_2_child_2 <= child_2_tour[pNumCities-1].nodeID.yCoordinate;
					end else begin
						if(child_2_tour[city_selector_child_2_counter_1].depot) begin
							x1_1_child_2 <= child_2_tour[city_selector_child_2_counter_1].nodeID.xCoordinate;
							y1_1_child_2 <= child_2_tour[city_selector_child_2_counter_1].nodeID.yCoordinate;
							x1_2_child_2 <= lpDepotXCoord;
							y1_2_child_2 <= lpDepotYCoord;
							start_child_2 <= 1; 
							state_child_2 <= DISTANCE_CALCULATION_DEPOT_STOP_WAIT_STATE;
						end else begin
							x1_1_child_2 <= child_2_tour[city_selector_child_2_counter_1].nodeID.xCoordinate;
							y1_1_child_2 <= child_2_tour[city_selector_child_2_counter_1].nodeID.yCoordinate;
							x1_2_child_2 <= child_2_tour[city_selector_child_2_counter_1+1].nodeID.xCoordinate;
							y1_2_child_2 <= child_2_tour[city_selector_child_2_counter_1+1].nodeID.yCoordinate;
							city_selector_child_2_counter_1 <= city_selector_child_2_counter_1 + 1;
							start_child_2 <= 1; 
							state_child_2 <= DISTANCE_CALCULATION_WAIT_STATE;
						end
					end
					
					if(rem_child_2 < root_child_2) begin 
					   fitness_value_child_2 <= fitness_value_child_2 + root_child_2;
					end else begin 
					   fitness_value_child_2 <= fitness_value_child_2 + root_child_2 + 1; 
					end 
				end
				
				DISTANCE_CALCULATION_WAIT_STATE: begin 
				    start_child_2 <= 0; 
				    if(valid_child_2) begin 
				        state_child_2 <= DISTANCE_CALCULATION; 
				    end 
				end 
				
				DISTANCE_CALCULATION_DEPOT_STOP_WAIT_STATE: begin 
				    start_child_2 <= 0; 
				    if(valid_child_2) begin 
				        state_child_2 <= DISTANCE_CALCULATION_DEPOT_STOP; 
				    end 
				end 
				
				DISTANCE_CALCULATION_INTERMEDIATE_FINAL_WAIT_STATE: begin 
				    start_child_2 <= 0; 
				    if(valid_child_2) begin 
				        state_child_2 <= DISTANCE_CALCULATION_INTERMEDIATE_FINAL; 
				    end 
				end 

				DISTANCE_CALCULATION_DEPOT_STOP: begin
				    x1_1_child_2 <= lpDepotXCoord;
				    y1_1_child_2 <= lpDepotYCoord;
				    x1_2_child_2 <= child_2_tour[city_selector_child_2_counter_1+1].nodeID.xCoordinate;
				    y1_2_child_2 <= child_2_tour[city_selector_child_2_counter_1+1].nodeID.yCoordinate;
				    city_selector_child_2_counter_1 <= city_selector_child_2_counter_1 + 1;
				    start_child_2 <= 1; 
				    state_child_2 <= DISTANCE_CALCULATION_WAIT_STATE;
				    if(rem_child_2 < root_child_2) begin 
					   fitness_value_child_2 <= fitness_value_child_2 + root_child_2;
					end else begin 
					   fitness_value_child_2 <= fitness_value_child_2 + root_child_2 + 1; 
					end 
				end

				DISTANCE_CALCULATION_INTERMEDIATE_FINAL: begin
					state_child_2 <= DISTANCE_CALCULATION_FINAL;
					if(rem_child_2 < root_child_2) begin 
					   fitness_value_child_2 <= fitness_value_child_2 + root_child_2;
					end else begin 
					   fitness_value_child_2 <= fitness_value_child_2 + root_child_2 + 1; 
					end 
				end

				DISTANCE_CALCULATION_FINAL: begin
					child_2_finished <= 1;
					if(state_parents != IDLE_PARENT) begin
						state_child_2 <= IDLE;
						city_selector_child_2_counter_1 <= 0;
					end
				end
			endcase
		end
	end
endmodule
