///////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Project		: Capacitated Vehicle Routing Problem on FPGA
// Function		: Controller for the complete algorithm, combines multiple processing nodes, memory ports and random number generators - test version for the whole algorithm, with a simulated reg file memory
// File			: controller.sv
// Created		: 12/04/2021
// Author 		: Maximilian Heer
// Notes 		: Features multiple processing nodes for parallel execution of multiple crossovers as well as memory ports and random number generators
// Advisory		: Try out various numbers of processing nodes and find out whether they fit the hardware on the FPGA
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

`timescale 1ns/1ps
`default_nettype none

module controller_v2_n16_k8_stepwise_loading_BRAM_2 #(
	parameter pNumCities = 100,
	parameter pVehicleCapacity = 206,
	parameter pDepotXCoord = 365,
	parameter pDepotYCoord = 689,
	parameter pNumInitRuns = 5000,
	parameter pNumRuns = 100000,
	parameter pNumProcessingNodes = 2,
    parameter pBestSolutionKnown = 10395194,
    parameter pRunBeyondBestKnown = 0,
    parameter pMutationProbabilityThreshold = 400, 
    parameter pWildChildProbabiltyThreshold = 25, 
    parameter pWildChildLimit = 50000, 
    parameter pSizeElitistList = 100, 
    parameter pElitistSelectionProbabilityThreshold = 200
)(
	// Generating module input for clk and rst to connect with hardware resources of the FPGA
	input wire logic clk_i,
	input wire logic rst_i,
	output logic done_o,
	output logic city_signal_1,
	output logic city_signal_2,
	output logic city_signal_3,
	output logic city_signal_4,
	output logic [pNumProcessingNodes-1:0] processingNodes_output,
	output logic TX_active_o,
	output logic TX_serial_o,
	output logic TX_done_o 
);

	///////////////////////////////////////////////////////////////////////////////////////////////////////////
	//
	// Definition of the localparams
	//
	///////////////////////////////////////////////////////////////////////////////////////////////////////////

	// Bit Width of the capacity requirement in each node
	localparam lpCapBitWidth = 7;

	// Bit Width of the IDs of the nodes
	localparam lpIDBitWidth = 20;

	// Bit Width of the coordinates of the nodes
	localparam lpCoordBitWidth = 10;

	// Bit Width of the maximum distance in the playfield
	localparam lpDistMaxBitWidth = 32;

	// Offset of cities for the selection of a random subtour from parent 1 to directly insert in the children
	localparam lpRandomSubtourOffset = 50;

	// Range of cities whose order is inverted for mutation
	localparam lpMutationDistance = 13;

	// Number of mutation processes that are executed for initialization of a new population in the beginning
	localparam lpInitMutationProcesses = 125;

	// Size of entire population
	localparam lpSizeOfPopulation = 20*pNumCities;

	// Bit Width of the number of a city within an individual
	localparam lpCityNumBitWidth = $clog2(pNumCities - 1);

	// Bit Width of the number of an individual within the population
	localparam lpIndividualNumBitWidth = $clog2(lpSizeOfPopulation - 1);

	// Bit Width of the number of processing nodes in the design
	localparam lpProcessingNumBitWidth = pNumProcessingNodes;

	// Bit Width of the number of crossover runs in the algorithm
	localparam lpNumRunsBitWidth = $clog2(pNumRuns - 1);
	
	// Bit width of the enumerator for the elitist list 
	localparam lpElitistListEnumeratorBitWidth = $clog2(pSizeElitistList-1); 

	// Bit Width for a summator for all priority values
	localparam lpBitWidthPrioSummator = pNumProcessingNodes + $clog2(pNumProcessingNodes);

	// Number of reading accesses required to completely read or write one individual from memory
	localparam lpNumOfMemAccesses = 51;

	// Bitwidth of a counter for reading / writing accesses to the memory
	localparam lpMemAccessesCounterBitWidth = $clog2(lpNumOfMemAccesses+1);

	// Number of memory lines in the population memory
	localparam lpNumOfMemLines = lpSizeOfPopulation * lpNumOfMemAccesses;
	
	// Number of bits used for addressing single memory lines 
	localparam lpNumOfMemLinesBitWidth = $clog2(lpNumOfMemLines-1); 

	// Bitwidth of the clock counter
	localparam lpClockCounterBitWidth = 256;

	localparam lpClockCounterTransmitUpperLimit = lpClockCounterBitWidth / 8;
	
	localparam lpInitFitness = 28'b1111111111111111111111111111;


	////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//
	// Definition of own datatypes
	//
	////////////////////////////////////////////////////////////////////////////////////////////////////////////

	// Definition of the number of a city within an individual
	typedef logic [lpCityNumBitWidth-1:0] CityNumber;
	
	// Definition of the enumerators for the elitist list 
	typedef logic [lpElitistListEnumeratorBitWidth-1:0] ElitistListEnumerator; 

	// Definition of the number of an individual within the population
	typedef logic [lpIndividualNumBitWidth-1:0] IndividualNumber;
	
	// Definition of the number of a memline within the population memory 
	typedef logic [lpNumOfMemLinesBitWidth-1:0] MemLineEnumerator; 

	// Definition of the two numbers of both parents in a processing node
	typedef struct packed{
		IndividualNumber parent_1;
		IndividualNumber parent_2;
	} DoubleIndividualNumbers;

	// Definition of a 256-bit memory word for data exchange between memory ports and processing nodes
	typedef logic [71:0] MemoryInterfaceWord;

	// Address in the BRAM
	typedef logic [$clog2(lpNumOfMemLines-1)-1:0] MemAddress;

	// Definition of word for fitness value
	typedef logic [lpIDBitWidth+lpCapBitWidth:0] FitnessValueWord;

	// Definition of a counter for the processing nodes
	typedef logic [lpProcessingNumBitWidth-1:0] ProcessingNodeCounter;

	// Definition of a value word for marking the priority of a calculation
	typedef logic [lpProcessingNumBitWidth:0] ProcessingPriority;

	// Definition of a counter for the number of runs
	typedef logic [lpNumRunsBitWidth-1:0] RunCounter;

	// Definition of the states for the main FSM
	typedef enum logic[3:0] {
		IDLE = 0,
		WAIT_INIT_RUNS = 1,
		WAIT_STANDARD_RUNS = 2,
		WAIT_FOR_END = 3,
		EVALUATE_BEST_SOLUTION = 4,
		FINISHED = 5
	} MainFSMState;

	// Definition of the states for the processing node FSM
	typedef enum logic[4:0] {
		IDLE_PN = 5'b00000,
		START = 5'b00001,
		LOAD_PARENT_1 = 5'b00010,
		START_LOADING_PARENT_2 = 5'b00011,
		LOAD_PARENT_2 = 5'b00100,
		WAIT_PROCESSING = 5'b00101,
		WB_OFFSPRING_1 = 5'b00110,
		START_WB_OFFSPRING_2 = 5'b00111,
		WB_OFFSPRING_2 = 5'b01000,
		FINAL = 5'b01001,
		TRANSMIT_CLOCK_CYCLES = 5'b01010,
		START_WB_OFFSPRING_1 = 5'b01011,
		READ_WAIT_STATE = 5'b01100,
		BEGIN_TRANSMITTING = 5'b01101,
		TRANSMIT_SOLUTION = 5'b01110,
		DONE = 5'b01111,
		TRANSMIT_CLOCK_CYCLES_INTERMEDIATE = 5'b10000,
		TRANSMIT_SOLUTION_INTERMEDIATE = 5'b10001,
		READ_WAIT_STATE_2 = 5'b10010,
		ELITIST_CHECKER_WAIT_STATE = 5'b10011, 
		REPLACE_ELITIST = 5'b10100
	} ProcessingNodeFSMState;

	// Definition of a summator word for the processing priorities
	typedef logic [lpBitWidthPrioSummator-1:0] PrioSummator;

	// Definition of a counter for memory accesses
	typedef logic [lpMemAccessesCounterBitWidth-1:0] MemAccessCounter;

	// Definition of a memory word
	typedef logic [71:0] MemoryWord;


	////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//
	// Definition of own signals and registers
	//
	////////////////////////////////////////////////////////////////////////////////////////////////////////////
	
	wire logic clk;
	wire logic clkfb;

	// Raw outputs of the two random number generators for crossover and mutation
	CityNumber raw_random_crossover_mutation_output_1;
	CityNumber raw_random_crossover_mutation_output_2;
	
	// Length of swapped route for mutation 
	CityNumber mutation_length_1; 
	CityNumber mutation_length_2; 
	
	// Raw values for the mutation starting point, since I want my own LFSRs for that 
	CityNumber raw_mutation_length_1; 
	CityNumber raw_mutation_length_2; 
	CityNumber raw_mutation_output_1; 
	CityNumber raw_mutation_output_2; 
	
	// City Number outputs for choosing the size of chunk that is crossed during crossover 
	CityNumber crossover_chunk_1; 
	CityNumber crossover_chunk_2; 
	
	// City Number regfile 
	CityNumber crossover_1_length_regfile[pNumProcessingNodes];
	CityNumber crossover_2_length_regfile[pNumProcessingNodes];

	// Adapted random number for the selection of subtours for crossover
	//CityNumber crossover_starting_point_1;
	//CityNumber crossover_starting_point_2;

	// Adapted random number for the selection of mutation parts
	CityNumber mutation_starting_point_1;
	CityNumber mutation_starting_point_2;

	// Raw output of the first random number generator for individual selection
	IndividualNumber raw_individual_output_1;

	// Raw output of the second random number generator for individual selection
	IndividualNumber raw_individual_output_2;

	// Adapted output of the random number generator for individual selection
	IndividualNumber adapted_individual_output;

	// Init mode signal for all processing nodes
	logic init_mode[pNumProcessingNodes];

	// Exchange register with ports for start_parent_1
	logic start_parent_1_regfile[pNumProcessingNodes];

	// Exchange register with ports for start_parent_2
	logic start_parent_2_regfile[pNumProcessingNodes];

	// Exchange register with ports for finished_parent_1_i
	logic finished_parent_1_regfile[pNumProcessingNodes];

    // Exchange register with ports for do_mutation
    logic do_mutation_regfile[pNumProcessingNodes];
    
    // Exchange register with ports for do_wildchild
    logic do_wild_child_regfile[pNumProcessingNodes];

	// Exchange register with ports for finished_parents_2_i
	logic finished_parent_2_regfile[pNumProcessingNodes];

	// Signal from main FSM to the node FSMs to start processing
	(* mark_debug = "true" *) logic start_processing[pNumProcessingNodes];

	// Signal from main FSM to the node FSMs to start the init mode for processing
	(* mark_debug = "true" *) logic processing_init[pNumProcessingNodes];

	// Transmission signals for child 1 from processing nodes
	wire logic transmission_ind_1_out[pNumProcessingNodes];

	// Transmission signals for child 2 from processing nodes
	wire logic transmission_ind_2_out[pNumProcessingNodes];

	// Loading signal for input storing process
	wire logic input_reading_done_out[pNumProcessingNodes];

	// Signals for unchanged transmission
	wire logic unchanged_transmission_out[pNumProcessingNodes];

	// Signals that crossover is done
	//wire logic crossover_done_out[pNumProcessingNodes];

	// Signal to show which is the better fitness value
	wire logic better_fitness_slot_out[pNumProcessingNodes];

	// Signals to show the better fitness value
	FitnessValueWord better_fitness_value_out[pNumProcessingNodes];

	// Array that holds all processing priorities for all processingNodes
	ProcessingPriority processing_priority[pNumProcessingNodes];

	// Summator signal for the single priorities of the processingNodes
	PrioSummator priority_sum;

	// Register file that counts all memory accesses for each processing node
	MemAccessCounter mem_access_counter[pNumProcessingNodes];

	// Input Wires from the processing nodes to then transfer to the BRAM pop memory
	MemoryInterfaceWord pop_data_pn_to_mem[pNumProcessingNodes];

	/* PLLE4_BASE #(
	    .CLKFBOUT_MULT(1),
		.CLKIN_PERIOD(10),
		.STARTUP_WAIT("TRUE")
	) clk_pll(
		.CLKIN(clk_i),
		.CLKOUT0(clk),
		.CLKFBIN(clkfb),
		.CLKFBOUT(clkfb)
	); */
	
	// Implement the clock downsampler to achieve lower clock frequency 
	/*ClockDownSampler #(
	       .pInputFreq(100),
	       .pOutputFreq(10)
	   )clock_downsampler(
	       .clk_i(clk_i), 
	       .rst_i(rst_i), 
	       .clk_o(clk)
	   );*/
	
	assign clk = clk_i;

	////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//
	// Initialisation of two random number generators for selecting subtours for crossover and mutation
	//
	/////////////////////////////////////////////////////////////////////////////////////////////////////////////

	// First random number generator
	LFSR #(
			.pNumBits(lpCityNumBitWidth)
		)random_city_selector_1(
			.clk(clk),
			.en_i(1'b1),
			.LFSR_Data_o(raw_random_crossover_mutation_output_1)
		);

	// Second random number generator
	LFSR #(
			.pNumBits(lpCityNumBitWidth),
			.pInitValue(6)
		)random_city_selector_2(
			.clk(clk),
			.en_i(1'b1),
			.LFSR_Data_o(raw_random_crossover_mutation_output_2)
		);

	assign city_signal_3 = raw_random_crossover_mutation_output_1[0];
	//assign city_signal_4 = raw_random_crossover_mutation_output_2[0];
	
	assign crossover_chunk_1 = (raw_random_crossover_mutation_output_1 < pNumCities) ? raw_random_crossover_mutation_output_1 : (pNumCities - 1); 
	assign crossover_chunk_2 = (raw_random_crossover_mutation_output_2 < pNumCities) ? raw_random_crossover_mutation_output_2 : (pNumCities - 1); 


    ////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //
    // Initialisation of a random number generator for choosing whether to do mutation or not
    //
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////

    logic [9:0] MutationDeciderOutput;
    logic [13:0] ElitistDeciderOutput1; 
    logic [13:0] ElitistDeciderOutput2; 
    

    LFSR #(
        .pNumBits(10)
    ) mutation_decided(
        .clk(clk),
        .en_i(1'b1),
        .LFSR_Data_o(MutationDeciderOutput)
    );
    
    LFSR #(
        .pNumBits(14), 
        .pInitValue(6)
    ) elitist_decided (
        .clk(clk), 
        .en_i(1'b1), 
        .LFSR_Data_o(ElitistDeciderOutput1)
    );
    
    LFSR #(
        .pNumBits(14)
    ) elitist_decided_2 (
        .clk(clk), 
        .en_i(1'b1), 
        .LFSR_Data_o(ElitistDeciderOutput2)
    ); 


	////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//
	// Assignments and calculations of the random number outputs for crossover and mutation
	//
	////////////////////////////////////////////////////////////////////////////////////////////////////////////

	// Assignment of the crossover starting points
	//assign crossover_starting_point_1 = (((raw_random_crossover_mutation_output_1+lpRandomSubtourOffset) < (pNumCities)) ? raw_random_crossover_mutation_output_1 : (pNumCities-lpRandomSubtourOffset));
	//assign crossover_starting_point_2 = (((raw_random_crossover_mutation_output_2+lpRandomSubtourOffset) < (pNumCities)) ? raw_random_crossover_mutation_output_2 : (pNumCities-lpRandomSubtourOffset));

    // LFSR for the selection of the mutation starting point 
    LFSR #(
        .pNumBits(lpCityNumBitWidth), 
        .pInitValue(8)
    ) mutation_start_1_random (
        .clk(clk),
		.en_i(1'b1),
		.LFSR_Data_o(raw_mutation_output_1)
    ); 
    
    LFSR #(
        .pNumBits(lpCityNumBitWidth), 
        .pInitValue(3)
    ) mutation_start_2_random (
        .clk(clk),
		.en_i(1'b1),
		.LFSR_Data_o(raw_mutation_output_2)
    ); 
    
    LFSR #(
        .pNumBits(lpCityNumBitWidth), 
        .pInitValue(1)
    ) mutation_length_1_random (
        .clk(clk),
		.en_i(1'b1),
		.LFSR_Data_o(raw_mutation_length_1)
    ); 
    
    LFSR #(
        .pNumBits(lpCityNumBitWidth), 
        .pInitValue(8)
    ) mutation_length_2_random (
        .clk(clk),
		.en_i(1'b1),
		.LFSR_Data_o(raw_mutation_length_2)
    ); 
    

	// Assignments of the mutation starting points
	assign mutation_starting_point_1 = (raw_mutation_output_1 < pNumCities) ? raw_mutation_output_1 : (pNumCities - 1);
	assign mutation_starting_point_2 = (raw_mutation_output_1 < pNumCities) ? raw_mutation_output_1 : (pNumCities - 1);
    
    assign mutation_length_1 = ((mutation_starting_point_1 + raw_mutation_length_1) < pNumCities) ? raw_mutation_length_1 : (pNumCities - 1 - mutation_starting_point_1); 
    assign mutation_length_2 = ((mutation_starting_point_2 + raw_mutation_length_2) < pNumCities) ? raw_mutation_length_2 : (pNumCities - 1 - mutation_starting_point_2); 
    

	////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//
	// Initialisation of two random number generators for selecting individuals from the population as parents
	//
	////////////////////////////////////////////////////////////////////////////////////////////////////////////

	// Selector for first parent
	LFSR #(
			.pNumBits(lpIndividualNumBitWidth)
		)random_individual_selector_1(
			.clk(clk),
			.en_i(!stop_LFSR_1),
			.LFSR_Data_o(raw_individual_output_1)
		);

	// Selector of second parent
	/*LFSR #(
			.pNumBits(lpIndividualNumBitWidth),
			.pInitValue(6)
		)random_individual_selector_2(
			.clk(clk),
			.en_i(1'b1),
			.LFSR_Data_o(raw_individual_output_2)
		);*/
		
	assign raw_individual_output_2 = clock_counter[lpIndividualNumBitWidth+3:4];

	assign city_signal_1 = raw_individual_output_1[0];
	assign city_signal_2 = raw_individual_output_2[0];
	
	
	////////////////////////////////////////////////////////////////////////////////////////
	//
	// Initialisation of all required LFSRs to choose individuals from the elitist list 
	//
	////////////////////////////////////////////////////////////////////////////////////////
	
	// Selector outputs for the elitist list 
	ElitistListEnumerator raw_elitist_selector_1; 
	ElitistListEnumerator raw_elitist_selector_2; 
	
	ElitistListEnumerator adapted_elitist_selector_1; 
	ElitistListEnumerator adapted_elitist_selector_2; 
	
	// Two random number generators for individuals from the elitist list 
	LFSR #(
	   .pNumBits(lpElitistListEnumeratorBitWidth), 
	   .pInitValue(0)
	)elitist_spot_selector_1(
	   .clk(clk), 
	   .en_i(1'b1), 
	   .LFSR_Data_o(raw_elitist_selector_1)
	);   
	
	LFSR #(
	   .pNumBits(lpElitistListEnumeratorBitWidth), 
	   .pInitValue(6)
	)elitist_spot_selector_2(
	   .clk(clk), 
	   .en_i(1'b1), 
	   .LFSR_Data_o(raw_elitist_selector_2)
	); 
	
	assign adapted_elitist_selector_1 = (raw_elitist_selector_1 < pSizeElitistList) ? raw_elitist_selector_1 : (pSizeElitistList-1);
	assign adapted_elitist_selector_2 = (raw_elitist_selector_2 < pSizeElitistList) ? raw_elitist_selector_2 : (pSizeElitistList-1); 


	/////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//
	// Initialisation of a UART transmitter for sending data from the FPGA to the PC
	//
	/////////////////////////////////////////////////////////////////////////////////////////////////////////////

	// Control signals for the UART transmitter
	logic tx_dv_i;
	logic [7:0] tx_byte_i;

	uart_tx #(
		//.CLKS_PER_BIT(10417)
		.CLKS_PER_BIT(868)
	) uart_communicator (
		.i_Clock(clk),
		.i_Tx_DV(tx_dv_i),
		.i_Tx_Byte(tx_byte_i),
		.o_Tx_Active(TX_active_o),
		.o_Tx_Serial(TX_serial_o),
		.o_Tx_Done(TX_done_o)
	);
	assign city_signal_4 = tx_dv_i;

	// Counters for transmitting the clock counter and the loaded data
	localparam lpSolutionAccessCounterBitWidth = $clog2(lpNumOfMemAccesses*9);
	localparam lpClockCounterTransmissionCounterBitWidth = $clog2(lpClockCounterBitWidth);

	logic [lpSolutionAccessCounterBitWidth : 0] SolutionAccessCounter;
	logic [lpClockCounterTransmissionCounterBitWidth : 0] ClockCounterTransmissionCounter;


	///////////////////////////////////////////////////////////////////////////////////////////////////////////
	//
	// Additional signals for controlling the memory access
	//
	///////////////////////////////////////////////////////////////////////////////////////////////////////////

	// Register file that holds all individuals currently in use
	IndividualNumber ind_in_use [4*pNumProcessingNodes];

	// Signal that shows that chosen 1 is currently not in use
	wire logic individual_1_not_in_use;

	// Signal that shows that chosen 2 is currently not in use
	wire logic individual_2_not_in_use;

	// Signal that shows that both selected signals are not identical
	wire logic individuals_not_identical;

	// Multiple wire signale for storing whether chosen individuals are already in use
	logic [4*pNumProcessingNodes-1 : 0] individual_1_comparison;
	logic [4*pNumProcessingNodes-1 : 0] individual_2_comparison;
	logic [4*pNumProcessingNodes-1 : 0] individual_1_follower_comparison; 
	logic [4*pNumProcessingNodes-1 : 0] individual_2_follower_comparison; 

	// Signals for calculated individual numbers (since random numbers could be higher than population size)
	IndividualNumber individual_proposed_1;
	IndividualNumber individual_proposed_2;
	
	IndividualNumber general_population_proposed_1; 
	IndividualNumber general_population_proposed_2; 
	
	IndividualNumber elite_population_proposed_1; 
	IndividualNumber elite_population_proposed_2; 
	
	MemLineEnumerator write_back_address_1[pNumProcessingNodes]; 
	MemLineEnumerator write_back_address_2[pNumProcessingNodes]; 

	// Signals for approving the proposed individuals
	wire logic ind_1_approved;
	wire logic ind_2_approved;

	// Signal for approving the current round of signals
	wire logic current_inds_approved;

	// Signal for stopping first LFSR from time to time
	wire logic stop_LFSR_1;

	//wire logic crossover_done;


	// counter for the clock cycles
	logic [lpClockCounterBitWidth-1:0] clock_counter;


	/////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//
	// Initialisation of the BRAMs
	//
	/////////////////////////////////////////////////////////////////////////////////////////////////////////////

	MemoryWord data_pn_to_mem[2];
	wire MemoryWord data_mem_to_pn[2];
	MemAddress mem_addr[2];
	logic mem_we[2];



	population_memory_BRAM_2 #(
		.pSize(lpSizeOfPopulation)
	) population_ram (
		.clk_i(clk),
		.rst_i(rst_i),
		.addr_1_i(mem_addr[0]),
		.we_1_i(mem_we[0]),
		.data_1_i(data_pn_to_mem[0]),
		.data_1_o(data_mem_to_pn[0]),
		.addr_2_i(mem_addr[1]),
		.we_2_i(mem_we[1]),
		.data_2_i(data_pn_to_mem[1]),
		.data_2_o(data_mem_to_pn[1])
	);



	////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//
	// Initialisation of 16 Processing Nodes
	//
	////////////////////////////////////////////////////////////////////////////////////////////////////////////

	// Parallel instantiation of up to 16 processing nodes (number can be chosen freely)
	genvar processingNodeCounter;
	generate
		for(processingNodeCounter = 0; processingNodeCounter < pNumProcessingNodes; processingNodeCounter++) begin: processing_node_generator
			ProcessingNode_stepwise_comparator_mix_up_72_Bit_2 #(
				.pNumCities(pNumCities),
				.pCapBitWidth(lpCapBitWidth),
				.pIDBitWidth(lpIDBitWidth),
				.pCoordBitWidth(lpCoordBitWidth),
				.pDistMaxBitWidth(lpDistMaxBitWidth),
				.pVehicleCapacity(pVehicleCapacity),
				.pRandomSubtourOffset(lpRandomSubtourOffset),
				.pMutationDistance(lpMutationDistance),
				.pInitMutationProcesses(lpInitMutationProcesses),
				.pDepotXCoord(pDepotXCoord),
				.pDepotYCoord(pDepotYCoord),
				.pCityNumBitWidth(lpCityNumBitWidth)
			) processing_node (
				.clk_i(clk),
				.rst_i(rst_i),
				.init_mode_i(init_mode[processingNodeCounter]),
				.start_parent_1_i(start_parent_1_regfile[processingNodeCounter]),
				.start_parent_2_i(start_parent_2_regfile[processingNodeCounter]),
				.finished_parent_1_i(finished_parent_1_regfile[processingNodeCounter]),
				.finished_parent_2_i(finished_parent_2_regfile[processingNodeCounter]),
				.pop_data_i(data_mem_to_pn[processingNodeCounter]),
				.pop_data_o(pop_data_pn_to_mem[processingNodeCounter]),
				.input_reading_done(input_reading_done_out[processingNodeCounter]),
				.transmission_ind_1_o(transmission_ind_1_out[processingNodeCounter]),
				.transmission_ind_2_o(transmission_ind_2_out[processingNodeCounter]),
				.unchanged_ind_o(unchanged_transmission_out[processingNodeCounter]),
				//.random_city_number_crossover_1_i(crossover_starting_point_1),
				//.random_city_number_crossover_2_i(crossover_starting_point_2),
				.random_city_number_mutation_1_i(mutation_starting_point_1),
				.random_city_number_mutation_2_i(mutation_starting_point_2),
				//.crossover_done(crossover_done),
				.better_fitness_slot_o(better_fitness_slot_out[processingNodeCounter]),
                .better_fitness_value_o(better_fitness_value_out[processingNodeCounter]),
                .do_mutation_i(do_mutation_regfile[processingNodeCounter]), 
                .do_wild_child_i(do_wild_child_regfile[processingNodeCounter]),
                .crossover_1_length_i(crossover_1_length_regfile[processingNodeCounter]),
                .crossover_2_length_i(crossover_2_length_regfile[processingNodeCounter]), 
                .mutation_length_1_i(mutation_length_1), 
                .mutation_length_2_i(mutation_length_2)
			);
		end
	endgenerate


	///////////////////////////////////////////////////////////////////////////////////////////////////////////
	//
	// Initialisation of independent register files for storing relevant information from each processing node
	//
	///////////////////////////////////////////////////////////////////////////////////////////////////////////

	// Best list - storing the current best value from each processing node
	FitnessValueWord best_known_solutions[pNumProcessingNodes];
	
	// Store the worst of the best solutions to compare if a new solutions should be included in the list or not 
	FitnessValueWord worst_of_the_best[pNumProcessingNodes];

	// Best list addresses - storing the addresses of the adresses of the best solutions
	IndividualNumber best_known_solutions_addresses[pNumProcessingNodes];
	
	// List of the elite solutions, respectively their fitness values 
	FitnessValueWord elite_solution_list[pNumProcessingNodes][pSizeElitistList]; 
	IndividualNumber elite_solution_addresses[pNumProcessingNodes][pSizeElitistList]; 
	
	// Counter for the elitist list 
	ElitistListEnumerator elitist_list_counter[pNumProcessingNodes]; 
	ElitistListEnumerator worst_of_the_best_address[pNumProcessingNodes]; 
	
	FitnessValueWord comparator_fitness[pNumProcessingNodes]; 
	ElitistListEnumerator comparator_fitness_address[pNumProcessingNodes]; 
	
	// logic signals that solution 1 / solution 2 was written back as real solution 
	logic solution_1_written_back[pNumProcessingNodes]; 
	logic solution_2_written_back[pNumProcessingNodes]; 
	
	

	// Initial starting signals for the processingNode-functionalities
	//logic start_proeccesing[pNumProcessingNodes];

	// Final stop signals for the processingNode-functionalities
	(* mark_debug = "true" *) logic stop_processing[pNumProcessingNodes];

	// Initial signal for running in init mode
	logic process_init[pNumProcessingNodes];

	// Central counter for all runs
	RunCounter run_counter;

	// Run Counter grid
	RunCounter run_counter_arr[pNumProcessingNodes];

	// Central counter for processing nodes
	ProcessingNodeCounter processing_node_counter;

	// register file for the FSMs of the processing nodes to store whether they have finished calculating
	logic [pNumProcessingNodes-1:0] all_finished_reg_file;

	// Combined signal to show that all processing nodes have finished calculation
	wire logic all_finished;

	// Signal to load the best solution into processing node 1
	logic load_best_solution_processing_node_1;

	// Search register for final best solution in the end
	FitnessValueWord best_final_solution;

	// Search register for the address for the final best solution in the end
	IndividualNumber best_final_solution_address;

	// State variable for the main FSM
	MainFSMState state_main;

	// State variable for the processing node FSM
	(* mark_debug = "true" *) ProcessingNodeFSMState state_pn[pNumProcessingNodes];

	// Extra reg for indicating which child was load first
	logic child_2_first[pNumProcessingNodes];


	////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//
	// Assignment of common signals
	//
	////////////////////////////////////////////////////////////////////////////////////////////////////////////

	// Common finished signals
	assign all_finished = (all_finished_reg_file == 0);


	////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//
	// Calculating whether the best known solution was already met or not
	//
	////////////////////////////////////////////////////////////////////////////////////////////////////////////

	logic [pNumProcessingNodes-1:0] best_known_solution_met_reg_file;
	logic best_known_solution_met;
	logic respective_best_solution_written_back[pNumProcessingNodes]; 

	always @ (*) begin
	   for(int nodeCounter = 0; nodeCounter < pNumProcessingNodes; nodeCounter++) begin
	       if(best_known_solutions[nodeCounter] <= pBestSolutionKnown) begin
	           best_known_solution_met_reg_file[nodeCounter] <= 1;
	       end else begin
	           best_known_solution_met_reg_file[nodeCounter] <= 0;
	       end
	       processingNodes_output[nodeCounter] = unchanged_transmission_out[nodeCounter];
	       
	       respective_best_solution_written_back[nodeCounter] = ((!better_fitness_slot_out[nodeCounter] && solution_1_written_back[nodeCounter]) || (better_fitness_slot_out[nodeCounter] && solution_2_written_back[nodeCounter]));  
	   end
	end

	assign best_known_solution_met = (best_known_solution_met_reg_file > 0);


	/////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//
	// Small Synchronous Process to just count clock cycles
	//
	////////////////////////////////////////////////////////////////////////////////////////////////////////////

	// Define a signal to stop the clock counter when calculation is done
	logic stop_clock_counter;

	always @ (posedge clk) begin
	   if(rst_i) begin
	       clock_counter <= 0;
	   end else begin
		   if(! stop_clock_counter) begin
	       	clock_counter <= clock_counter + 1;
		   end
	   end
	end


	////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//
	// Central FSM - starts the initialization phase, counts the number of crossover operations and stops everything at the end
	//
	////////////////////////////////////////////////////////////////////////////////////////////////////////////

	always @ (posedge clk) begin
		if(rst_i) begin
			for(int nodeCounter = 0; nodeCounter < pNumProcessingNodes; nodeCounter++) begin

				// resetting the registers that are used for communication with the processing nodes
				start_processing[nodeCounter] <= 0;
				stop_processing[nodeCounter] <= 0;
				processing_init[nodeCounter] <= 0;
			end

			state_main <= IDLE;
			processing_node_counter <= 0;
			load_best_solution_processing_node_1 <= 0;
			best_final_solution <= 0;
			best_final_solution_address <= 0;
			// done_o <= 0;
		end else begin
			case(state_main)
				IDLE: begin
					start_processing[processing_node_counter] <= 1;
					processing_init[processing_node_counter] <= 1;
					processing_node_counter <= processing_node_counter + 1;

					// Once all processing nodes are started, go to next state and wait for
					if(processing_node_counter == pNumProcessingNodes) begin
						state_main <= WAIT_INIT_RUNS;
						processing_node_counter <= 0;
					end
                    //done_o <= 0;
				end

				WAIT_INIT_RUNS: begin
					// If number of init runs is reached, leave the init mode and go to normal processing mode
					if(run_counter == pNumInitRuns) begin
						state_main <= WAIT_STANDARD_RUNS;
						for(int nodeCounter = 0; nodeCounter < pNumProcessingNodes; nodeCounter++) begin
							processing_init[nodeCounter] <= 0;
						end
					end
				end

				WAIT_STANDARD_RUNS: begin
					// If number of overall runs is reached, finish the crossover process and go to final wait state
					if(run_counter == pNumRuns || (!pRunBeyondBestKnown && best_known_solution_met)) begin
						state_main <= WAIT_FOR_END;
						for(int nodeCounter = 0; nodeCounter < pNumProcessingNodes; nodeCounter++) begin
							start_processing[nodeCounter] <= 0;
						end
					end
				end

				WAIT_FOR_END: begin
					if(all_finished) begin
						state_main <= EVALUATE_BEST_SOLUTION;
						processing_node_counter <= 1;
						best_final_solution <= best_known_solutions[0];
						best_final_solution_address <= best_known_solutions_addresses[0];
					end
				end

				EVALUATE_BEST_SOLUTION: begin
					if(best_known_solutions[processing_node_counter] < best_final_solution) begin
						best_final_solution <= best_known_solutions[processing_node_counter];
						best_final_solution_address <= best_known_solutions_addresses[processing_node_counter];
					end

					if(processing_node_counter < pNumProcessingNodes - 1) begin
						processing_node_counter <= processing_node_counter + 1;
						state_main <= EVALUATE_BEST_SOLUTION;
					end else begin
						state_main <= FINISHED;
						load_best_solution_processing_node_1 <= 1;
					end
				end

				FINISHED: begin
					state_main <= FINISHED;
					// done_o <= 1;
				end

			endcase
		end
	end

	//////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//
	// Checking whether randomly chosen memory addresses can be used or are already in use, which would cause a block
	//
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////

    // Get the possible selection from the standard population 
    assign general_population_proposed_1 = (raw_individual_output_1 > lpSizeOfPopulation - 1) ? (raw_individual_output_1 - lpSizeOfPopulation) : raw_individual_output_1; 
    assign general_population_proposed_2 = (raw_individual_output_2 > lpSizeOfPopulation - 1) ? (raw_individual_output_2 - lpSizeOfPopulation) : raw_individual_output_2;
    
    assign elite_population_proposed_1 = elite_solution_addresses[0][adapted_elitist_selector_1]; 
    assign elite_population_proposed_2 = elite_solution_addresses[0][adapted_elitist_selector_2]; 
    
	// First of all calculating regular individual numbers
	assign individual_proposed_1 = (ElitistDeciderOutput1 < pElitistSelectionProbabilityThreshold) ? elite_population_proposed_1 : general_population_proposed_1;
	assign individual_proposed_2 = (ElitistDeciderOutput2 < pElitistSelectionProbabilityThreshold) ? elite_population_proposed_2 : general_population_proposed_2;

	// Check wether the proposed individuals are identical
	assign individuals_not_identical = (individual_proposed_1 != individual_proposed_2) && (individual_proposed_1 + 1 != individual_proposed_2) && (individual_proposed_1 != individual_proposed_2 + 1);

    // Generate additional +1 proposals for checks with the inds_in_use 
    IndividualNumber individual_proposed_1_follower; 
    IndividualNumber individual_proposed_2_follower; 
    
    assign individual_proposed_1_follower = individual_proposed_1 + 1; 
    assign individual_proposed_2_follower = individual_proposed_2 + 1; 
    
	// Fill out the multi-bit signals for comparison of chosen individuals with those currently in use
	always @ (*) begin
		for(int individualCounter = 0; individualCounter < 8*pNumProcessingNodes; individualCounter++) begin
			if(individual_proposed_1 == ind_in_use[individualCounter]) begin
				individual_1_comparison[individualCounter] = 1;
			end else begin
				individual_1_comparison[individualCounter] = 0;
			end
			
			if(individual_proposed_2 == ind_in_use[individualCounter]) begin
				individual_2_comparison[individualCounter] = 1;
			end else begin
				individual_2_comparison[individualCounter] = 0;
			end
			
			if(individual_proposed_1_follower == ind_in_use[individualCounter]) begin
				individual_1_follower_comparison[individualCounter] = 1;
			end else begin
				individual_1_follower_comparison[individualCounter] = 0;
			end
			
			if(individual_proposed_2_follower == ind_in_use[individualCounter]) begin
				individual_2_follower_comparison[individualCounter] = 1;
			end else begin
				individual_2_follower_comparison[individualCounter] = 0;
			end
		end
	end
	
	// Check whether a general population individual is an elitist by chance 
	logic [pSizeElitistList-1:0] general_individual_1_is_elitist; 
	logic [pSizeElitistList-1:0] general_individual_2_is_elitist; 
	
	always @ (*) begin 
	   for(int elite_counter = 0; elite_counter < pSizeElitistList; elite_counter++) begin 
	       general_individual_1_is_elitist[elite_counter] <= (general_population_proposed_1 == elite_solution_addresses[1][elite_counter]);
	       general_individual_2_is_elitist[elite_counter] <= (general_population_proposed_2 == elite_solution_addresses[1][elite_counter]); 
	   end 
	end 
	
	logic proposed_individual_1_is_elitist; 
	logic proposed_individual_2_is_elitist; 
	
	assign proposed_individual_1_is_elitist = (ElitistDeciderOutput1 < pElitistSelectionProbabilityThreshold) ? 1'b1 : (general_individual_1_is_elitist != 0);
	assign proposed_individual_2_is_elitist = (ElitistDeciderOutput2 < pElitistSelectionProbabilityThreshold) ? 1'b1 : (general_individual_2_is_elitist != 0); 
    
    logic parent_1_is_elitist[pNumProcessingNodes]; 
    logic parent_2_is_elitist[pNumProcessingNodes];
    
    logic parent_1_shifted_address[pNumProcessingNodes]; 
    logic parent_2_shifted_address[pNumProcessingNodes]; 
    
	// Assign the signals for recognizing usage
	assign individual_1_not_in_use = (individual_1_comparison == 0);
	assign individual_2_not_in_use = (individual_2_comparison == 0);
	
	// Assign the signals for recognizing usage of the followers 
	logic individual_1_follower_not_in_use; 
	logic individual_2_follower_not_in_use; 
	
	assign individual_1_follower_not_in_use = (individual_1_follower_comparison == 0); 
	assign individual_2_follower_not_in_use = (individual_2_follower_comparison == 0); 
	

	// Final check for the proposed individuals
	assign ind_1_approved = individual_1_not_in_use && individuals_not_identical && individual_1_follower_not_in_use;
	assign ind_2_approved = individual_2_not_in_use && individuals_not_identical && individual_2_follower_not_in_use;
	assign current_inds_approved = ind_1_approved && ind_2_approved;

	assign stop_LFSR_1 = individual_1_not_in_use && !individuals_not_identical;


	/////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//
	// Granting access based on priority of the processing nodes
	//
	/////////////////////////////////////////////////////////////////////////////////////////////////////////////

	// Summing up the priority values in the priority register file for finding out who is allowed to start processing
	always @ (*) begin
		run_counter = 0;
		priority_sum = 0;
		for(int prioCounter = 0; prioCounter < pNumProcessingNodes; prioCounter++) begin
			priority_sum += processing_priority[prioCounter];
			run_counter += run_counter_arr[prioCounter];
		end
	end


	//////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//
	// Calculating the all_finished signal
	//
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////

	always @ (*) begin
	   for(int finishedCounter = 0; finishedCounter < pNumProcessingNodes; finishedCounter++) begin
		   all_finished_reg_file[finishedCounter] = (state_pn[finishedCounter] != IDLE_PN);
	   end
	end

	logic [3:0] serial_transmission_counter;

	/////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//
	// Local FSMs for the various processing nodes
	//
	/////////////////////////////////////////////////////////////////////////////////////////////////////////////

	always @ (posedge clk) begin
		for(int pn_counter = 0; pn_counter < pNumProcessingNodes; pn_counter++) begin
			if(rst_i) begin
				init_mode[pn_counter] <= 0;
				start_parent_1_regfile[pn_counter] <= 0;
				start_parent_2_regfile[pn_counter] <= 0;
				finished_parent_1_regfile[pn_counter] <= 0;
				finished_parent_2_regfile[pn_counter] <= 0;
				done_o <= 0; 
				solution_1_written_back[pn_counter] <= 0; 
				solution_2_written_back[pn_counter] <= 0; 
				
				best_known_solutions[pn_counter] <= 2**(lpIDBitWidth+lpCapBitWidth+1) - 1;
				best_known_solutions_addresses[pn_counter] <= 0;

				// resetting the actual memory registers
				for(int best_counter = 0; best_counter < pSizeElitistList; best_counter++) begin 
				    elite_solution_list[pn_counter][best_counter] <= 2**(lpIDBitWidth+lpCapBitWidth+1) - 1; 
				    elite_solution_addresses[pn_counter][best_counter] <= 0; 
				end
				
				worst_of_the_best[pn_counter] <= 2**(lpIDBitWidth+lpCapBitWidth+1) - 1; 

				processing_priority[pn_counter] <= 0;
				state_pn[pn_counter] <= IDLE_PN;
				run_counter_arr[pn_counter] <= 0;
				child_2_first[pn_counter] <= 0;
				mem_we[pn_counter] <= 0;
				mem_addr[pn_counter] <= 0;
                do_mutation_regfile[pn_counter] <= 0;
                do_wild_child_regfile[pn_counter] <= 0; 
                crossover_1_length_regfile[pn_counter] <= 0; 
                crossover_2_length_regfile[pn_counter] <= 0; 
                elitist_list_counter[pn_counter] <= 0; 
                worst_of_the_best_address[pn_counter] <= 0; 
                comparator_fitness_address[pn_counter] <= 0; 
                comparator_fitness[pn_counter] <= 0; 
                parent_1_is_elitist[pn_counter] <= 0; 
                parent_2_is_elitist[pn_counter] <= 0; 
                write_back_address_1[pn_counter] <= 0; 
                write_back_address_2[pn_counter] <= 0; 
                parent_1_shifted_address[pn_counter] <= 0; 
                parent_2_shifted_address[pn_counter] <= 0; 

				tx_dv_i <= 0;
				tx_byte_i <= 0;

				for(int indCounter = 0; indCounter < 4*pNumProcessingNodes - 1; indCounter++) begin
				    ind_in_use[indCounter] <= 0;
				end

				serial_transmission_counter <= 0;
				stop_clock_counter <= 0;

			end else begin
				case(state_pn[pn_counter])
					IDLE_PN: begin
						if(start_processing[pn_counter]) begin
							state_pn[pn_counter] <= START;
							processing_priority[pn_counter] <= 2**pn_counter;

							// Checking for init-mode
							if(processing_init[pn_counter]) begin
								init_mode[pn_counter] <= 1;
							end else begin
								init_mode[pn_counter] <= 0;
							end
						end else if (load_best_solution_processing_node_1 && pn_counter == 0) begin
							state_pn[pn_counter] <= TRANSMIT_CLOCK_CYCLES;
							stop_clock_counter <= 1;
							tx_dv_i <= 1;
							tx_byte_i <= clock_counter[lpClockCounterBitWidth-1 -: 8];
							ClockCounterTransmissionCounter <= 1;
						end
						child_2_first[pn_counter] <= 0;
					end

					START: begin
						if((2**pn_counter > priority_sum/2) && current_inds_approved) begin
							processing_priority[pn_counter] <= 0;
							parent_1_is_elitist[pn_counter] <= proposed_individual_1_is_elitist; 
							parent_2_is_elitist[pn_counter] <= proposed_individual_2_is_elitist; 
							ind_in_use[4*pn_counter] <= individual_proposed_1;
							ind_in_use[4*pn_counter + 2] <= individual_proposed_2;
							
							if(proposed_individual_1_is_elitist && (individual_proposed_1 != (lpSizeOfPopulation-1))) begin 
							     ind_in_use[4*pn_counter + 1] <= individual_proposed_1 + 1; 
							end else begin 
							     ind_in_use[4*pn_counter + 1] <= individual_proposed_1; 
							end 
							
							if(proposed_individual_2_is_elitist && (individual_proposed_2 != (lpSizeOfPopulation-1))) begin 
							     ind_in_use[4*pn_counter + 3] <= individual_proposed_2 + 1; 
							end else begin 
							     ind_in_use[4*pn_counter + 3] <= individual_proposed_2; 
							end 
							
							write_back_address_1[pn_counter] <= 0; 
							write_back_address_2[pn_counter] <= 0; 
							
							state_pn[pn_counter] <= LOAD_PARENT_1;
							solution_1_written_back[pn_counter] <= 0; 
							solution_2_written_back[pn_counter] <= 0; 
							parent_1_shifted_address[pn_counter] <= 0; 
							parent_2_shifted_address[pn_counter] <= 0; 

							mem_addr[pn_counter] <= lpNumOfMemAccesses*individual_proposed_1;

                            // Set the do_mutation_regfile to determine whether mutation is happening or not in the processing node in this cycle
                            do_mutation_regfile[pn_counter] <= (MutationDeciderOutput < pMutationProbabilityThreshold);
                            
                            // Set the do_wild_child_regfile to determine whether wild_child is happening or not in the processing node in this cycle 
                            do_wild_child_regfile[pn_counter] <= (MutationDeciderOutput < pWildChildProbabiltyThreshold) && (run_counter < pWildChildLimit); 

							// start_parent_1_regfile[pn_counter] <= 1;
							mem_access_counter[pn_counter] <= 1;
							
							// Assign the length of chunk to be crossed during crossover process 
							crossover_1_length_regfile[pn_counter] <= crossover_chunk_1; 
							crossover_2_length_regfile[pn_counter] <= crossover_chunk_2; 
						end
					end

					LOAD_PARENT_1: begin
                        start_parent_1_regfile[pn_counter] <= 1;
						if(input_reading_done_out[pn_counter]) begin
							if(mem_access_counter[pn_counter] < lpNumOfMemAccesses) begin
								mem_access_counter[pn_counter] <= mem_access_counter[pn_counter] + 1;

								mem_addr[pn_counter] <= lpNumOfMemAccesses*ind_in_use[2*pn_counter] + mem_access_counter[pn_counter];

							end else begin
								run_counter_arr[pn_counter] <= run_counter_arr[pn_counter] + 1;
								finished_parent_1_regfile[pn_counter] <= 1;
								state_pn[pn_counter] <= START_LOADING_PARENT_2;
								start_parent_1_regfile[pn_counter] <= 0;
							end
						end
					end

					START_LOADING_PARENT_2: begin
                        start_parent_1_regfile[pn_counter] <= 0;
						state_pn[pn_counter] <= LOAD_PARENT_2;
						finished_parent_1_regfile[pn_counter] <= 1;

						mem_addr[pn_counter] <= lpNumOfMemAccesses*ind_in_use[2*pn_counter+1];

						// start_parent_2_regfile[pn_counter] <= 1;
						mem_access_counter[pn_counter] <= 1;

					end

					LOAD_PARENT_2: begin
                        start_parent_2_regfile[pn_counter] <= 1;
						if(input_reading_done_out[pn_counter]) begin
							if(mem_access_counter[pn_counter] < lpNumOfMemAccesses) begin
								mem_access_counter[pn_counter] <= mem_access_counter[pn_counter] + 1;

								mem_addr[pn_counter] <= lpNumOfMemAccesses*ind_in_use[2*pn_counter+1] + mem_access_counter[pn_counter];

							end else begin
								finished_parent_2_regfile[pn_counter] <= 1;
								state_pn[pn_counter] <= WAIT_PROCESSING;
								start_parent_2_regfile[pn_counter] <= 0;
							end
						end
					end

					WAIT_PROCESSING: begin
                        start_parent_2_regfile[pn_counter] <= 0;
						finished_parent_2_regfile[pn_counter] <= 0;
						if(transmission_ind_1_out[pn_counter]) begin
							child_2_first[pn_counter] <= 0;
							if(unchanged_transmission_out[pn_counter]) begin
								mem_we[pn_counter] <= 0;
								state_pn[pn_counter] <= START_WB_OFFSPRING_2;
							end else begin
                                solution_1_written_back[pn_counter] <= 1; 
								mem_we[pn_counter] <= 1;
								// mem_addr[pn_counter] <= lpNumOfMemAccesses*ind_in_use[2*pn_counter];
								data_pn_to_mem[pn_counter] <= pop_data_pn_to_mem[pn_counter];

								state_pn[pn_counter] <= WB_OFFSPRING_1;
								mem_access_counter[pn_counter] <= 1;
								
								// Write-back mechanism depends on whether the overwritten parent is an elite parent or not 
								if(parent_1_is_elitist[pn_counter]) begin 
								    write_back_address_1[pn_counter] <= lpNumOfMemAccesses*ind_in_use[4*pn_counter + 1]; 
								    mem_addr[pn_counter] <= lpNumOfMemAccesses*ind_in_use[4*pn_counter + 1]; 
								    parent_1_shifted_address[pn_counter] <= 1; 
								end else begin 
								    write_back_address_1[pn_counter] <= lpNumOfMemAccesses*ind_in_use[4*pn_counter]; 
								    mem_addr[pn_counter] <= lpNumOfMemAccesses*ind_in_use[4*pn_counter]; 
								end 
								
							end
						end else if(transmission_ind_2_out[pn_counter]) begin
							child_2_first[pn_counter] <= 1;
							if(unchanged_transmission_out[pn_counter]) begin
								mem_we[pn_counter] <= 0;
								state_pn[pn_counter] <= START_WB_OFFSPRING_1;
							end else begin
                                solution_2_written_back[pn_counter] <= 1; 
								mem_we[pn_counter] <= 1;
								// mem_addr[pn_counter] <= lpNumOfMemAccesses*ind_in_use[2*pn_counter+1];
								data_pn_to_mem[pn_counter] <= pop_data_pn_to_mem[pn_counter];
								
								// Write-back mechanism depends on whether the overwritten parent is an elite parent or not 
								if(parent_2_is_elitist[pn_counter]) begin 
								    write_back_address_2[pn_counter] <= lpNumOfMemAccesses*ind_in_use[4*pn_counter + 3]; 
								    mem_addr[pn_counter] <= lpNumOfMemAccesses*ind_in_use[4*pn_counter + 3]; 
								    parent_2_shifted_address[pn_counter] <= 1; 
								end else begin 
								    write_back_address_2[pn_counter] <= lpNumOfMemAccesses*ind_in_use[4*pn_counter + 2]; 
								    mem_addr[pn_counter] <= lpNumOfMemAccesses*ind_in_use[4*pn_counter + 2]; 
								end 

								state_pn[pn_counter] <= WB_OFFSPRING_2;
								mem_access_counter[pn_counter] <= 1;
							end
						end
						 
					end

					START_WB_OFFSPRING_1: begin
						finished_parent_1_regfile[pn_counter] <= 0;
						if(transmission_ind_1_out[pn_counter]) begin
							if(unchanged_transmission_out[pn_counter]) begin
								mem_we[pn_counter] <= 0;
								state_pn[pn_counter] <= FINAL;
							end else begin
                                solution_1_written_back[pn_counter] <= 1; 
								mem_we[pn_counter] <= 1;
								// mem_addr[pn_counter] <= lpNumOfMemAccesses*ind_in_use[2*pn_counter];
								data_pn_to_mem[pn_counter] <= pop_data_pn_to_mem[pn_counter];
								
								// Write-back mechanism depends on whether the overwritten parent is an elite parent or not 
								if(parent_1_is_elitist[pn_counter]) begin 
								    write_back_address_1[pn_counter] <= lpNumOfMemAccesses*ind_in_use[4*pn_counter + 1]; 
								    mem_addr[pn_counter] <= lpNumOfMemAccesses*ind_in_use[4*pn_counter + 1]; 
								    parent_1_shifted_address[pn_counter] <= 1; 
								end else begin 
								    write_back_address_1[pn_counter] <= lpNumOfMemAccesses*ind_in_use[4*pn_counter]; 
								    mem_addr[pn_counter] <= lpNumOfMemAccesses*ind_in_use[4*pn_counter]; 
								end


								state_pn[pn_counter] <= WB_OFFSPRING_1;
								mem_access_counter[pn_counter] <= 1;
							end
						end
					end

					WB_OFFSPRING_1: begin
						if(mem_access_counter[pn_counter] < lpNumOfMemAccesses-1) begin
							mem_access_counter[pn_counter] <= mem_access_counter[pn_counter] + 1;

							mem_we[pn_counter] <= 1;
							mem_addr[pn_counter] <= write_back_address_1[pn_counter]+mem_access_counter[pn_counter];
							data_pn_to_mem[pn_counter] <= pop_data_pn_to_mem[pn_counter];


						end else begin

							mem_we[pn_counter] <= 1;
							mem_addr[pn_counter] <= write_back_address_1[pn_counter]+mem_access_counter[pn_counter];
							data_pn_to_mem[pn_counter] <= pop_data_pn_to_mem[pn_counter];

							state_pn[pn_counter] <= START_WB_OFFSPRING_2;
							start_parent_1_regfile[pn_counter] <= 0;
							if(!child_2_first[pn_counter]) begin
								state_pn[pn_counter] <= START_WB_OFFSPRING_2;
							end else begin
								state_pn[pn_counter] <= FINAL;
							end
						end
					end

					START_WB_OFFSPRING_2: begin
						finished_parent_1_regfile[pn_counter] <= 0;
						if(transmission_ind_2_out[pn_counter]) begin
							if(unchanged_transmission_out[pn_counter]) begin
								mem_we[pn_counter] <= 0;
								state_pn[pn_counter] <= FINAL;
							end else begin
                                solution_2_written_back[pn_counter] <= 1; 
								mem_we[pn_counter] <= 1;
								// mem_addr[pn_counter] <= lpNumOfMemAccesses*ind_in_use[2*pn_counter+1];
								data_pn_to_mem[pn_counter] <= pop_data_pn_to_mem[pn_counter];
								
								if(parent_2_is_elitist[pn_counter]) begin 
								    write_back_address_2[pn_counter] <= lpNumOfMemAccesses*ind_in_use[4*pn_counter + 3]; 
								    mem_addr[pn_counter] <= lpNumOfMemAccesses*ind_in_use[4*pn_counter + 3]; 
								    parent_2_shifted_address[pn_counter] <= 1; 
								end else begin 
								    write_back_address_2[pn_counter] <= lpNumOfMemAccesses*ind_in_use[4*pn_counter + 2]; 
								    mem_addr[pn_counter] <= lpNumOfMemAccesses*ind_in_use[4*pn_counter + 2]; 
								end 

								state_pn[pn_counter] <= WB_OFFSPRING_2;
								mem_access_counter[pn_counter] <= 1;
							end
						end
					end

					WB_OFFSPRING_2: begin
						if(mem_access_counter[pn_counter] < lpNumOfMemAccesses-1) begin
							mem_access_counter[pn_counter] <= mem_access_counter[pn_counter] + 1;

							mem_we[pn_counter] <= 1;
							mem_addr[pn_counter] <= write_back_address_2[pn_counter] + mem_access_counter[pn_counter];
							data_pn_to_mem[pn_counter] <= pop_data_pn_to_mem[pn_counter];

						end else begin

							mem_we[pn_counter] <= 1;
							mem_addr[pn_counter] <= write_back_address_2[pn_counter] + mem_access_counter[pn_counter];
							data_pn_to_mem[pn_counter] <= pop_data_pn_to_mem[pn_counter];

							state_pn[pn_counter] <= FINAL;
							start_parent_1_regfile[pn_counter] <= 0;
							if(child_2_first[pn_counter]) begin
								state_pn[pn_counter] <= START_WB_OFFSPRING_1;
							end else begin
								state_pn[pn_counter] <= FINAL;
							end
						end
					end

					FINAL: begin
						mem_we[pn_counter] <= 0;
						finished_parent_2_regfile[pn_counter] <= 0;
						if(pn_counter == 0 && load_best_solution_processing_node_1) begin
							state_pn[pn_counter] <= TRANSMIT_CLOCK_CYCLES;
							stop_clock_counter <= 1;
							tx_dv_i <= 1;
							tx_byte_i <= clock_counter[lpClockCounterBitWidth-1 -: 8];
							ClockCounterTransmissionCounter <= 0;
						end else begin
						    // Check whether the best found solution belongs into the elitist group - if yes, group it there. If not, just return to the IDLE-state 
						    if((better_fitness_value_out[pn_counter] < worst_of_the_best[pn_counter]) && respective_best_solution_written_back[pn_counter]) begin 
						          state_pn[pn_counter]  <= ELITIST_CHECKER_WAIT_STATE; 
						          elite_solution_list[pn_counter][worst_of_the_best_address[pn_counter]] <= better_fitness_value_out[pn_counter]; 
						          if(better_fitness_slot_out[pn_counter] == 1'b0) begin
						              if(parent_1_shifted_address[pn_counter]) begin 
						                  elite_solution_addresses[pn_counter][worst_of_the_best_address[pn_counter]] <= ind_in_use[4*pn_counter+1]; 
						              end else begin 
						                  elite_solution_addresses[pn_counter][worst_of_the_best_address[pn_counter]] <= ind_in_use[4*pn_counter]; 
						              end 
								  end else begin
								      if(parent_2_shifted_address[pn_counter]) begin 
								          elite_solution_addresses[pn_counter][worst_of_the_best_address[pn_counter]] <= ind_in_use[4*pn_counter + 3]; 
								      end else begin 
								          elite_solution_addresses[pn_counter][worst_of_the_best_address[pn_counter]] <= ind_in_use[4*pn_counter + 2]; 
								      end 
								  end
						    end else begin 
							     state_pn[pn_counter] <= IDLE_PN;
							end
							
							// Setting the best_known_solution 
							if(better_fitness_value_out[pn_counter] < best_known_solutions[pn_counter]) begin
								best_known_solutions[pn_counter] <= better_fitness_value_out[pn_counter];
								if(better_fitness_slot_out[pn_counter] == 1'b0) begin
								    if(parent_1_shifted_address[pn_counter]) begin 
						                  best_known_solutions_addresses[pn_counter] <= ind_in_use[4*pn_counter+1]; 
						            end else begin 
						                  best_known_solutions_addresses[pn_counter] <= ind_in_use[4*pn_counter]; 
						            end 
								end else begin
									if(parent_2_shifted_address[pn_counter]) begin 
								          best_known_solutions_addresses[pn_counter] <= ind_in_use[4*pn_counter + 3]; 
								    end else begin 
								          best_known_solutions_addresses[pn_counter] <= ind_in_use[4*pn_counter + 2]; 
								    end
								end
							end else if (do_wild_child_regfile[pn_counter]) begin 
							     if((best_known_solutions_addresses[pn_counter] == ind_in_use[4*pn_counter+1]) || (best_known_solutions_addresses[pn_counter] == ind_in_use[4*pn_counter + 3])) begin 
							         best_known_solutions[pn_counter] <= lpInitFitness;
							     end
							end
						end
					end
					
					ELITIST_CHECKER_WAIT_STATE: begin 
					   state_pn[pn_counter] <= REPLACE_ELITIST; 
					   comparator_fitness[pn_counter] <= elite_solution_list[pn_counter][0]; 
					   comparator_fitness_address[pn_counter] <= 0; 
					end 
					
					REPLACE_ELITIST: begin 
					   if(elitist_list_counter[pn_counter] < pSizeElitistList) begin 
                           if(elite_solution_list[pn_counter][elitist_list_counter[pn_counter]] > comparator_fitness[pn_counter]) begin 
                               comparator_fitness[pn_counter] <= elite_solution_list[pn_counter][elitist_list_counter[pn_counter]];
                               comparator_fitness_address[pn_counter] <= elitist_list_counter[pn_counter]; 
                           end 
                           elitist_list_counter[pn_counter] <= elitist_list_counter[pn_counter] + 1; 
                       end else begin 
                            state_pn[pn_counter] <= IDLE_PN; 
                            worst_of_the_best[pn_counter] <= comparator_fitness[pn_counter]; 
                            worst_of_the_best_address[pn_counter] <= comparator_fitness_address[pn_counter]; 
                       end  
					end 

					TRANSMIT_CLOCK_CYCLES_INTERMEDIATE: begin
						if(TX_done_o) begin
							state_pn[pn_counter] <= TRANSMIT_CLOCK_CYCLES;
						end else begin
							tx_dv_i <= 0;
						end
					end

					TRANSMIT_CLOCK_CYCLES: begin
							if(ClockCounterTransmissionCounter == lpClockCounterTransmitUpperLimit) begin
								tx_dv_i <= 0;
								state_pn[pn_counter] <= READ_WAIT_STATE;
								mem_we[pn_counter] <= 0;
								mem_addr[pn_counter] <= lpNumOfMemAccesses*best_final_solution_address;
								mem_access_counter[pn_counter] <= 1;
							end else begin
								tx_dv_i <= 1;
								tx_byte_i <= clock_counter[lpClockCounterBitWidth-1-ClockCounterTransmissionCounter*8 -: 8];
								ClockCounterTransmissionCounter <= ClockCounterTransmissionCounter + 1;
								state_pn[pn_counter] <= TRANSMIT_CLOCK_CYCLES_INTERMEDIATE;
							end
					end

					READ_WAIT_STATE: begin
						state_pn[pn_counter] <= READ_WAIT_STATE_2;
					end

					READ_WAIT_STATE_2: begin
						state_pn[pn_counter] <= BEGIN_TRANSMITTING;
					end

					BEGIN_TRANSMITTING: begin
					    tx_dv_i <= 1;
						tx_byte_i <= data_mem_to_pn[pn_counter][71:64];
					    serial_transmission_counter <= 1;
						state_pn[pn_counter] <= TRANSMIT_SOLUTION_INTERMEDIATE;
					end

					TRANSMIT_SOLUTION_INTERMEDIATE: begin
						if(TX_done_o) begin
							state_pn[pn_counter] <= TRANSMIT_SOLUTION;
						end else begin
							tx_dv_i <= 0;
						end
					end

					TRANSMIT_SOLUTION: begin
					       if(serial_transmission_counter == 9) begin
					           if(mem_access_counter[pn_counter] < lpNumOfMemAccesses) begin
                                   tx_dv_i <= 0;
                                   state_pn[pn_counter] <= READ_WAIT_STATE;
								   serial_transmission_counter <= 0;
                                   mem_we[pn_counter] <= 0;
                                   mem_addr[pn_counter] <= lpNumOfMemAccesses*best_final_solution_address + mem_access_counter[pn_counter];
                                   mem_access_counter[pn_counter] <= mem_access_counter[pn_counter] + 1;
                               end else begin
                                   state_pn[pn_counter] <= DONE;
                               end
					       end else begin
					           tx_dv_i <= 1;
							   tx_byte_i <= data_mem_to_pn[pn_counter][71-serial_transmission_counter*8-:8];
							   serial_transmission_counter <= serial_transmission_counter + 1;
							   state_pn[pn_counter] <= TRANSMIT_SOLUTION_INTERMEDIATE;
					       end
					end

					DONE: begin
                        done_o <= 1; 
					end
				endcase
			end
		end
	end
endmodule
