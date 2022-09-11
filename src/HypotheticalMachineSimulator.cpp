/** @file HypotheticalMachineSimulator.cpp
 * @brief HypotheticalMachineSimulator implementations
 *
 * @author Student Name
 * @note   cwid: 111 11 111
 * @date   Fall 2019
 * @note   ide:  g++ 8.2.0 / GNU Make 4.2.1
 *
 * Implementation file for our Hypothetical machine class and
 * supporting functions.
 */
#include "HypotheticalMachineSimulator.hpp"
#include "SimulatorException.hpp"
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>

/**
 * @brief OpcodeMnemonic overload output stream
 *
 * Overload the output stream operator to display the
 * mnemonic opcodes to an output stream in a more human
 * friendly fashion.
 *
 * @param out The output stream we should send the representation
 *   of the indicated list to.
 * @param opcode A enumerated type of type OPCODE_MNEMONICS
 *   current state of.
 *
 * @returns ostream& Returns a reference to the originaly provided
 *   output stream, but after we  have inserted the opcode
 *   representation onto the stream.
 */
ostream& operator<<(ostream& out, const OpcodeMnemonic& opcode)
{
  switch (opcode)
  {
  case NOOP_HALT:
    out << int(NOOP_HALT) << " (NOOP_HALT)";
    break;
  case LOAD:
    out << int(LOAD) << " (LOAD)";
    break;
  case STORE:
    out << int(STORE) << " (STORE)";
    break;
  case JMP:
    out << int(JMP) << " (JMP)";
    break;
  case SUB:
    out << int(SUB) << " (SUB)";
    break;
  case ADD:
    out << int(ADD) << " (ADD)";
    break;
  default:
    out << int(opcode) << " (Error: unknown/invalid opcode)";
  }

  return out;
}

/**
 * @brief HypotheticalMachineSimulator default constructor
 *
 * Default constructor for the Hypothetical Machine Simulator.
 * This constructor will initialize the hypothetical machine
 * state so that all registers and memory are initialized to
 * empty or NOOP/HALT states.
 */
HypotheticalMachineSimulator::HypotheticalMachineSimulator()
{
  memory = NULL; // no memory defined until we load program;
  this->reset();
}

/**
 * @brief HypotheticalMachineSimulator destructor
 *
 * Destructor for the hypothetical machine simulator.
 * We dynamically allocate the memory array to hold
 * the values of our simulated memory, which must be
 * released.
 */
HypotheticalMachineSimulator::~HypotheticalMachineSimulator()
{
  this->reset();
}

/**
 * @brief reset machine
 *
 * Reset the hypothetical machine back to clean state.
 * The actual work of deallocation and initialization
 * is done here so we can call on destruction or reload
 * of a program.
 */
void HypotheticalMachineSimulator::reset()
{
  // if we currently have any memory, deallocate it.
  if (memory)
  {
    delete[] memory;
  }

  // initialize all values to empty/beginning states
  memory = NULL;
  memoryBaseAddress = memoryBoundsAddress = memorySize = 0;
  pc = ac = ir = 0;
  irOpcode = NOOP_HALT;
  irAddress = 0;
  memoryAddressList.clear();
}

/**
 * @brief load program
 *
 * Load program from simulation file into memory and registers.
 * We load the description of the initial register contents,
 * and the contents of program instructions and data in memory
 * from a basic text file.  File must be in the expected
 * format or else this function will give an exception
 * and give up.
 *
 * @exception Will throw SimulatorException
 *   if file not found, or if file contents are badly formatted
 *   or missing.
 * @param programFile A string parameter specifying the
 *   file name we are to load into the hypothetical machine
 *   simulator.  Can be a relative or absolute path name, but
 *   of course the file must exist and be of the correct
 *   input format for this function to work.
 */
void HypotheticalMachineSimulator::loadProgram(string programFile)
{
  ifstream programStream;

  // open the file as a stream for reading,
  // error check that file loaded successfully
  programStream.open(programFile.c_str());
  if (!programStream.is_open())
  {
    // cerr << "Error: could not open program file: "
    //   << programFile << endl;
    string msg = "Error: could not open program file: " + programFile;
    throw SimulatorException(msg);
  }

  // parse the file, we expect file in an exact format
  string key;
  int value;

  // registers come first
  programStream >> key >> value;
  if (key != "PC")
  {
    throw SimulatorException("Error: expecting PC from program file");
  }
  this->pc = value;

  programStream >> key >> value;
  if (key != "AC")
  {
    throw SimulatorException("Error: expecting AC from program file");
  }
  this->ac = value;

  // Now we load memory, starting with specification of memory bounds
  int memoryBaseAddress;
  int memoryBoundsAddress;
  programStream >> key >> memoryBaseAddress >> memoryBoundsAddress;
  if (key != "MEM")
  {
    throw SimulatorException("Error: expecting MEM from program file");
  }

  // Task 3.2: uncomment this call to initializeMemory() once you have completed
  // task 3
  initializeMemory(memoryBaseAddress, memoryBoundsAddress);

  // Finally load memory contents.  Remaining lines of file from
  // current line to last line are key/value pairs of
  //    addr value
  // to give the address and contents of memory.
  int addr;
  while (programStream >> addr >> value)
  {
    // Task 3.2: uncomment this call to pokeAddress() once you have completed task 3
    pokeAddress(addr, value);

    // keep track of memory addresses for display purposes
    memoryAddressList.push_back(addr);
  }
  // sort the list of memory addresses
  memoryAddressList.sort();
}

/**
 * @brief initialize memory
 *
 * Initialize the contents of memory.  Allocate array larget enough to
 * hold memory contents for the program.  Record base and bounds
 * address for memory address translation.  This memory function
 * dynamically allocates enough memory to hold the addresses for the
 * indicated begin and end memory ranges.
 *
 * @param memoryBaseAddress The int value for the base or beginning
 *   address of the simulated memory address space for this
 *   simulation.
 * @param memoryBoundsAddress The int value for the bounding address,
 *   e.g. the maximum or upper valid address of the simulated memory
 *   address space for this simulation.
 *
 * @exception Throws SimulatorException if
 *   address space is invalid.  Currently we support only 4 digit
 *   opcodes XYYY, where the 3 digit YYY specifies a reference
 *   address.  Thus we can only address memory from 000 - 999
 *   given the limits of the expected opcode format.
 */
/**
 * @brief initialize memory
 *
 * Initialize the contents of memory.  Allocate array larget enough to
 * hold memory contents for the program.  Record base and bounds
 * address for memory address translation.  This memory function
 * dynamically allocates enough memory to hold the addresses for the
 * indicated begin and end memory ranges.
 *
 * @param memoryBaseAddress The int value for the base or beginning
 *   address of the simulated memory address space for this
 *   simulation.
 * @param memoryBoundsAddress The int value for the bounding address,
 *   e.g. the maximum or upper valid address of the simulated memory
 *   address space for this simulation.
 *
 * @exception Throws SimulatorException if
 *   address space is invalid.  Currently we support only 4 digit
 *   opcodes XYYY, where the 3 digit YYY specifies a reference
 *   address.  Thus we can only address memory from 000 - 999
 *   given the limits of the expected opcode format.
 */
void HypotheticalMachineSimulator::initializeMemory(int memoryBaseAddress, int memoryBoundsAddress)
{
  // In this simulateion, we use XYYY where X is an opcode and YYY is a memory
  // reference address.  This means memory is limited to addresses 000 - 999
  if ((memoryBaseAddress < 0) or (memoryBaseAddress > 999) or (memoryBoundsAddress < 0) or (memoryBoundsAddress > 999))
  {
    ostringstream msg;
    msg << "HypotheticalMachineSimulator::initializeMemory> Memory initialization error" << endl
        << "  memory can't be below 0 or exceed 1000, 3 decimal digit memory references only" << endl
        << "  memoryBaseAddress: " << memoryBaseAddress << endl
        << "  memoryBoundsAddress: " << memoryBoundsAddress;
    throw SimulatorException(msg.str());
  }

  // record memory properties needed for address translation operations
  this->memoryBaseAddress = memoryBaseAddress;
  this->memoryBoundsAddress = memoryBoundsAddress;

  // memory size is difference of end minus begin, and add 1 because of 0 based
  // indexing
  this->memorySize = memoryBoundsAddress - memoryBaseAddress + 1;

  // also memory should have at least 1 value to be meaningful, if memory size is 0 or negative
  // then throw an exception
  if (memorySize < 1)
  {
    ostringstream msg;
    msg << "HypotheticalMachineSimulator::initializeMemory> Illegal memory range" << endl
        << "  memory must be at least size 1 or larger to run simulations" << endl
        << "  memoryBaseAddress: " << memoryBaseAddress << endl
        << "  memoryBoundsAddress: " << memoryBoundsAddress << endl
        << "  memorySize: " << memorySize;
    throw SimulatorException(msg.str());
  }
  // if we have previous memory allocated, make sure it is free before allocating some more
  if (memory)
  {
    delete[] memory;
  }

  // dynamically allocate a block of integers to use to represent
  // our hypothetical machine memory
  this->memory = new int[memorySize];

  // initialize memory to all 0 values, we use 0 as noop/halt in this simulator
  for (int addr = 0; addr < memorySize; addr++)
  {
    memory[addr] = 0;
  }
}

/**
 * @brief memory address translation
 *
 * Given a memory address, translate the address to the real array index/memory
 * reference.  In addition we check the final bounds and throw an exception
 * for an illegal memory access attempt.
 *
 * @param virtualAddress The virtual or simulated address that we should translate to
 *   the real address/index.
 *
 * @exception Throws SimulatorException if the simAddress
 *   referenced is illegal, e.g. if it is below or above the memory bounds.
 * @returns int Returns the calculated real address translation of the simulated
 *   address.
 */
int HypotheticalMachineSimulator::translateAddress(int virtualAddress) const
{
  int realAddress;

  // subtract the base address from the virtual address to get the
  // real address/index in memory
  realAddress = virtualAddress - memoryBaseAddress;

  // memory bounds checking, throw exception if memory reference is out of bounds
  if ((realAddress < 0) or (realAddress >= memorySize))
  {
    ostringstream msg;
    msg << "translateAddress: Memory bounds access error, illegal reference to address: " << virtualAddress;
    throw SimulatorException(msg.str());
  }

  return realAddress;
}

/**
 * @brief poke memory
 *
 * Set (poke) the value of a given address in our memory.
 * The address given is in our simulator address space, so
 * remember it needs to be translated to the real address
 * before poke occurs.
 *
 * @param virtualAddress The virtual simulation address we are to write
 *   the value into.
 * @param value The value to be written into the indicated virtual
 *   address.
 */
void HypotheticalMachineSimulator::pokeAddress(int virtualAddress, int value)
{
  int realAddress = translateAddress(virtualAddress);
  memory[realAddress] = value;
}

/**
 * @brief peek memory
 *
 * Retrive (peek) the value of a given address in our simulator
 * memory.  The address given is in our simulator address space,
 * so remember it needs to be translated to the real address before
 * peek occurs
 *
 * @param virtualAddress The virtual address we should access and read
 *   a value from.
 *
 * @returns int The value that is currently in the indicated virtual
 *   memory address we are reading from and returning.
 */
int HypotheticalMachineSimulator::peekAddress(int virtualAddress) const
{
  int realAddress = translateAddress(virtualAddress);
  return memory[realAddress];
}

/**
 * @brief fetch phase
 *
 * Perform the fetch phase of a fetch/execute cycle. Update the
 * machine with results of the fetch.  We load the IR with indicated
 * instruction, which is basically all that occurs during the fetch
 * phase of a cycle.
 */
void HypotheticalMachineSimulator::fetch()
{
  // load the instruction register, using PC as index of
  // instruction to fetch.
  ir = peekAddress(pc);
}

/**
 * @brief execute phase
 *
 * Perform the execute phase of a fetch/execute cycle.  Execute the
 * current instruction residing in the instruction register.  We first
 * translate the instruction into an opcode/reference pair.  Then we
 * delegete work using a lookup table based on the indicated opcode to
 * a function to perform the actual work of the operation.  The
 * PC is also incremented during the execute phase in preparation
 * for the next fetch.
 */
void HypotheticalMachineSimulator::execute()
{
  // decode the instruction register, first decimal digit is
  // the opcode, last 3 digits are the reference address/value
  // Largest maximum instruction is 9999, throw exception immediately
  // if we see something nonsensical
  if ((ir < 0) or (ir > 9999))
  {
    ostringstream msg;
    msg << "Error: execute: invalid instruction, out of range, ir: " << ir;
    throw SimulatorException(msg.str());
  }

  // decode the opcode and address from the instruction register.
  // Use integer division and modulus division to get 4th digit and first 3 digits
  // respectively, to translate the opcode and address reference from the ir
  irOpcode = static_cast<OpcodeMnemonic>(ir / 1000);
  irAddress = ir % 1000;

  // if this is a noop/halt we don't increment pc, but otherwise
  // increment pc in preparation for next fetch cycle
  // NOTE: in future we should change the simulation so that
  // we don't need this special case for NOOPS maybe?
  if (irOpcode != NOOP_HALT)
  {
    incrementPC();
  }

  // use table lookup to translate opcode into function to call to perform
  // specific operation indicated
  switch (irOpcode)
  {
  case NOOP_HALT:
    // do nothing
    break;
  case LOAD:
    executeLoad();
    break;
  case STORE:
    executeStore();
    break;
  case JMP:
    executeJmp();
    break;
  case SUB:
    executeSub();
    break;
  case ADD:
    executeAdd();
    break;
  default:
    ostringstream msg;
    msg << "Error: execute: invalid opcode seen, ir=" << ir;
    throw SimulatorException(msg.str());
  }
}

/**
 * @brief execute load
 *
 * Execute a load instruction.
 * @pre current irOpcode is a LOAD when called.
 */
void HypotheticalMachineSimulator::executeLoad()
{
  // for load, the irAddress gives us the reference address to fetch value from into
  // the accumulator
  ac = peekAddress(irAddress);
}

/**
 * @brief execute store
 *
 * Execute a store instruction.
 * @pre current irOpcode is a STORE when called
 */
void HypotheticalMachineSimulator::executeStore()
{
  // for load, the irAddress gives us the reference address to fetch value from into
  // the accumulator
  pokeAddress(irAddress, ac);
}

/**
 * @brief execute add
 *
 * Execute an add instruction.
 * @pre current irOpcode is an ADD when called
 */
void HypotheticalMachineSimulator::executeAdd()
{
  // execute the indicated add
  ac = ac + peekAddress(irAddress);
}

/**
 * @brief execute sub
 *
 * Execute a subtract instruction.
 * @pre current irOpcode is a SUB when called
 */
void HypotheticalMachineSimulator::executeSub()
{
  // execute the indicated subtract
  ac = ac - peekAddress(irAddress);
}

/**
 * @brief execute jmp
 *
 * Execute a jump instruction.
 * @pre current irOpcode is a JMP when called
 */
void HypotheticalMachineSimulator::executeJmp()
{
  // execute the indicated jump instruction
  pc = irAddress;
}

/**
 * @brief run simulation
 *
 * Run simulation of hypothetical machine.  We run cycles until we hit
 * a NOOP_HALT instruction, or until we reach the specified maximum
 * number of cycles.  This function returns the number of cycles that
 * were executed.  If verbose is specified, the state of the system
 * registers and memory are displayed after each fetch/execute phase.
 *
 * @param maxCycles When running the simulation, we will onlyc
 *   simulate this maximum number of fetch/execute cycles,
 *   to avoid infinite loops.  Default maxCycles is 100 if
 *   not specified.
 * @param verbose If true we will display the full system
 *   state to standard output after each fetch and each
 *   execute cycle. Defaults to false if not specified.
 *
 * @returns int Returns the number of actual fetch/execute
 *   cycles that were performed.  This cycles shoule be
 *   cycles <= maxCycles given when this function is called.
 */
int HypotheticalMachineSimulator::runSimulation(int maxCycles, bool verbose)
{
  int cycle = 0;

  // Finally for step 7 I have given you an implementation of this
  // function.  It uses your fetch() and execute() functions (which in)
  // turn use the others you implemented) to run a full simulation of
  // our hypothetical machine.  Uncomment the while loop and then all of
  // your unit tests should be passing if the previous functions were fully
  // implemented correctly and as asked for.  If unit tests are not passing
  // after uncommenting the code below, you shold go back to the first failing
  // unit test and figure out and fix that issue, and proceed fixing issues 1
  // at a time.
  bool done = false;

  while (!done)
  {
    // perform fetch stage
    fetch();
    irOpcode = NOOP_HALT;
    irAddress = 0;
    if (verbose)
    {
      cout << "==================== cycle: " << cycle + 1 << endl;
      cout << "-------------------- fetch" << endl;
      cout << *this;
    }

    // perform execute stage
    execute();
    if (verbose)
    {
      cout << "-------------------- execute" << endl;
      cout << *this;
    }

    // increment cycle counter for next cycle
    cycle++;

    // we are done if we exceed the maximum number of
    // cycles to simulate
    if (cycle >= maxCycles)
    {
      done = true;
    }

    // or we are done when we hit a NOOP_HALT
    // instruction
    if (irOpcode == NOOP_HALT)
    {
      done = true;
    }
  }

  return cycle;
}

/**
 * @brief memory base address accessor
 *
 * @returns int Returns the current virtual memory base address.
 */
int HypotheticalMachineSimulator::getMemoryBaseAddress() const
{
  return memoryBaseAddress;
}

/**
 * @brief memory bounds address accessor
 *
 * @returns int Returns the current virtual upper bounds address.
 */
int HypotheticalMachineSimulator::getMemoryBoundsAddress() const
{
  return memoryBoundsAddress;
}

/**
 * @brief memory size accessor
 *
 * @returns int Returns the current total memory size for this
 *   simulation.
 */
int HypotheticalMachineSimulator::getMemorySize() const
{
  return memorySize;
}

/**
 * @brief program counter accessor
 *
 * @returns int Returns the current PC (program counter)
 *   register of the simulation.
 */
int HypotheticalMachineSimulator::getPC() const
{
  return pc;
}

/**
 * @brief program counter increment
 *
 * Mutator to increment pc to next location in memory.  a convenience
 * method in case we ever need to do something more complicated in
 * addition to pc increment at end of execute stage.
 */
void HypotheticalMachineSimulator::incrementPC()
{
  pc++;
}

/**
 * @brief accumulator accessor
 *
 * @returns int Returns the current value of the simulator
 *   AC (accumulator) register.
 */
int HypotheticalMachineSimulator::getAC() const
{
  return ac;
}

/**
 * @brief instruction register accessor
 *
 * @returns int Returns the current value of the simulator
 *   IR (instruction regiser).
 */
int HypotheticalMachineSimulator::getIR() const
{
  return ir;
}

/**
 * @brief instruction register opcode accessor
 *
 * @returns int Returns the translated opcode from the
 *   fetched instruction fetched into the instruction
 *   register.
 */
int HypotheticalMachineSimulator::getIROpcode() const
{
  return irOpcode;
}

/**
 * @brief instruction register address accessor
 *
 * @returns int Returns the referenced address of the current
 *   loaded instruction/opcode in the instruction register.
 */
int HypotheticalMachineSimulator::getIRAddress() const
{
  return irAddress;
}

/**
 *@brief overload output stream operator for simulation
 *
 * Overload the output stream operator so that we can display current
 * state of Hypothetical Machine simulation to a stream.
 *
 * @param out The output stream we should send the representation
 *   of the current machine state to.
 * @param sim The HypotheticalMachineSimulator object to display
 *   current state of.
 *
 * @returns ostream& Returns a reference to the originaly provided
 *   output stream, but after we  have inserted current hypothetical
 *   machine state onto the stream.
 */
ostream& operator<<(ostream& out, const HypotheticalMachineSimulator& sim)
{
  out << "CPU Registers" << endl
      << "-------------" << endl
      << "PC: " << sim.pc << endl
      << "AC: " << sim.ac << endl
      << "IR: " << sim.ir << endl;
  if (sim.irOpcode != NOOP_HALT)
  {
    out << "    opcode : " << sim.irOpcode << endl;
  }
  if (sim.irAddress != 0)
  {
    out << "    address: " << sim.irAddress << endl;
  }
  out << endl;

  out << "Memory" << endl << "------" << endl;
  // uncomment the following once you have completed task 7 so that system tests
  // and full simulations will work
  for (auto addr : sim.memoryAddressList)
  {
    cout << setw(3) << left << addr << ": " << sim.peekAddress(addr) << endl;
  }

  cout << endl;

  return out;
}
