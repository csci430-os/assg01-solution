---
name: 'Task 1: Implement `initializeMemory()` simulation member function'
about: Task 1 for Students
title: 'Task 1: Implement `initializeMemory()` simulation member function'
labels: enhancement, good first issue
assignees: ''

---

**Description**

Implement the missing `initializeMemory()` simulation memory method.  This method needs to initialize base, bounds and size member variables.  And it needs to allocate the `memory` array used by the simulation to simulate the contents of the hypothetical machine memory.  This memory should be initialized.  In addition, some error checks need to be performed to look out for bad base or bounds addresses given for the simulation.


**Suggested Solution**


**Additional Requirements**

- You are required to test base and bounds addresses for illegal addresses, and thow the indicated exception if bad initialization addresses are given.
- You are required to dynamically allocate the `memory` array of the correct size for the indicated simulation.
- You are required to make sure that all values in the newly allocated `memory` are initialized to 0.  The simulation expects any memory content that is not specified in the input simulation file will be initialized to 0.
