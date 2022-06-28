---
name: 'Task 6: Implement `execute*()` instruction execution subfunctions'
about: Task 6 for Students
title: 'Task 6: Implement `execute*()` instruction execution subfunctions'
labels: enhancement
assignees: ''

---

**Description**

In this task there are several functiions that perform the actual execution of specific machine instructions in the hypothetical machine simulation.  You will implement all of the following here:

- `executeLoad()`: implements the LOAD instruction
- `executeStore()`: implements the STORE instruction
- `executeSub()`: implements the SUB instruction for arithmetic subtraction
- `executeAdd()`: implements the ADD instruction for arithmetic addition
- `executeJump()`: implements the JMP instruction for program flow control


**Suggested Solution**


**Additional Requirements**

- You must reuse `peekAddress()` and `pokeAddress()` function any time you are getting a value from `memory` or storing a value back to `memory`.  All of these function, except for a jump, need to do this. 
- You will store results of arithmetic operations back into the accumulator AC register.
- Load loads a value into the AC accumulator.  Store does the opposite and stores value in AC back to memory.
- The JMP instruction actually causes the PC program counter to be modified.


