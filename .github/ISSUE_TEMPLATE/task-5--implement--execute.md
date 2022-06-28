---
name: 'Task 5: Implement `execute()` simulation member method'
about: Task 5 for Students
title: 'Task 5: Implement `execute()` simulation member method'
labels: enhancement
assignees: ''

---

**Description**

Implement the `execute()` member method that performs the execution portion of the basic fetch-execute cycle.  The main work of executing an instruction is performed in several specific member function, that will be implemented in the next task.  In this method, you need to increment the PC by 1 to simulate the normal sequential flow-of-control of executing instructions.  Then you will translate the instruction, breaking it into its opcode (first digit of the instruction in decimal) and into the reference address (last 3 digits of the instruction).  Finally based on the opcode that is extracted, you will call one of the specific member methods to execute that opcode.

**Suggested Solution**

- Use a switch statement on the opcode to determine specific function to call in next task to execute the opcode/instruction.


**Additional Requirements**

- You are required to use the declared global constant mnemonics for the opcodes, e.g. LOAD, STORE, etc.  Do not sprinkle your code with magic numbers.

