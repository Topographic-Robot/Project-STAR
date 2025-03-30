ASM and Inline ASM
==================

In this project, **Assembly code** and **Inline Assembly** should be used with extreme caution. While it can offer performance improvements or enable access to low-level hardware features, its use can make code harder to understand, maintain, and port. Always prefer writing in C unless absolutely necessary.

General Guidelines for ASM and Inline ASM
-----------------------------------------

- **Avoid ASM When Possible**: Always try to write in C before resorting to assembly language. C compilers are highly optimized and can often produce efficient machine code that rivals hand-written assembly.

- **Document ASM Code Thoroughly**: Assembly code is often harder to understand than C, so it should be accompanied by thorough comments that explain **why** ASM was used, as well as what the code does.

Example:

.. code-block:: asm

    /* Set register to zero using ASM */
    mov r0, #0      /* Initialize r0 to 0 */

- **Use Inline ASM Sparingly**: Inline assembly allows you to embed assembly instructions directly in C code. It should only be used when absolutely necessary, such as when interacting with hardware registers or performing CPU-specific optimizations.

- **Constrain the Scope of Inline ASM**: When using inline assembly, constrain its scope to as few lines as possible and avoid intermixing it with complex C logic. This helps isolate potential issues and reduces the risk of bugs.

Example:

.. code-block:: c

    void set_flag(void)
    {
      asm volatile("mov r0, #1" : : : "r0");  /* Set flag in register r0 */
    }

- **Always Use `volatile` for Inline ASM**: Inline assembly should be marked as `volatile` to prevent the compiler from optimizing it out.

- **Do Not Use Inline ASM for Optimizations**: Modern C compilers perform aggressive optimizations. Inline ASM should not be used to try and optimize code performance unless absolutely necessary, as it is often more error-prone than compiler optimizations.

- **Use Constraints Correctly**: When using inline ASM in C, use the appropriate constraints to inform the compiler how the assembly instructions interact with C variables.

Example:

.. code-block:: c

    int add(int a, int b)
    {
      int result;
      asm volatile(
        "add %[r], %[a], %[b]"
        : [r] "=r" (result)    /* Output */
        : [a] "r" (a), [b] "r" (b) /* Inputs */
      );
      return result;
    }

- **Prefer Named Registers**: Always use named registers or C variable references in inline ASM instead of hardcoding register names. This improves portability and makes it easier to understand how the assembly interacts with C.

- **Platform-Specific ASM**: If inline assembly or standalone ASM is used, ensure that it is documented as platform-specific and cannot be easily ported to other architectures.

Bad Example:
------------

.. code-block:: c

    asm("mov r0, #1");    /* INCORRECT: No constraints, platform-specific */

Good Example:
-------------

.. code-block:: c

    int set_register(int value)
    {
      asm volatile(
        "mov r0, %[val]"
        :
        : [val] "r" (value)
        : "r0"   /* CORRECT: Specifying the register and constraint */
      );
      return value;
    }

When to Use ASM and Inline ASM
------------------------------

- **Hardware Access**: Use ASM when direct hardware access is required and C alone cannot provide the necessary control (e.g., setting or clearing specific registers).

- **Performance Critical Code**: ASM may be justified in performance-critical sections where C compiler optimizations fall short, but this should be documented and used as a last resort.

- **CPU-Specific Instructions**: Use ASM for executing instructions that are unique to the target CPU (e.g., specific ARM or x86 instructions that are not exposed in C).

General Guidelines
------------------

- Avoid ASM unless absolutely necessary.

- Use inline ASM for simple tasks like setting hardware registers, not for optimizations.

- Always use `volatile` for inline ASM to prevent it from being optimized out.

- Ensure thorough documentation for any assembly code, explaining both what the code does and why ASM is required.

- Use constraints in inline ASM to inform the compiler about register usage and interactions with C variables.

- Isolate inline ASM code from complex C logic, and keep it as minimal as possible.

- Make sure all ASM or inline ASM code is portable or clearly marked as platform-specific if not.


Assertions
==========

In this project, **assertions** are used as a tool to catch programming errors and invalid states during development. They help ensure that assumptions made in the code are correct, and they provide valuable debugging information when things go wrong. However, assertions should never be relied on to handle runtime errors or expected conditions in production code.

Guidelines
----------

- **Use Assertions for Debugging**: Assertions are intended to catch conditions that should never occur during normal program execution. Use them to assert assumptions about the state of the program, such as parameter validation or invariants within algorithms.

Example of Using `assert`:

.. code-block:: c

    #include <assert.h>

    void process_data(int* data)
    {
      assert(data != NULL);  /* Assert that data is not NULL */
      /* Continue processing */
    }

- **Avoid Assertions in Production Code**: Assertions should not be used for handling expected runtime errors or conditions that might naturally occur in a program's lifecycle. Instead, use proper error handling techniques like `if` statements or return codes.

Bad Example:

.. code-block:: c

    void read_file(FILE* fp)
    {
      assert(fp != NULL);  /* INCORRECT: This could be a valid runtime error */
      /* Reading file logic */
    }

Good Example:

.. code-block:: c

    void read_file(FILE* fp)
    {
      if (fp == NULL) {
        /* Handle the error, perhaps by returning an error code */
        return;
      }
      /* Reading file logic */
    }

- **Avoid Side Effects in Assertions**: Assertions should not contain side effects, as the code inside the `assert` macro will be removed in non-debug builds (e.g., when `NDEBUG` is defined). Ensure that assertions only check conditions without altering the program state.

Bad Example:

.. code-block:: c

    assert(buffer = malloc(1024));  /* INCORRECT: Side effect inside assert */

Good Example:

.. code-block:: c

    buffer = malloc(1024);
    assert(buffer != NULL);  /* CORRECT: No side effects inside assert */

- **Document Assertions**: When using an assertion, provide a comment to explain what the assertion is checking and why it is necessary. This will help other developers (or future you) understand the assumption being made.

Example:

.. code-block:: c

    assert(index >= 0 && index < MAX_SIZE);  /* Index should always be within bounds */

- **Use `static_assert` When Possible**: For compile-time checks, prefer using `static_assert` (available in C11 and later) to catch errors early in the compilation process.

Example of Using `static_assert`:

.. code-block:: c

    #include <assert.h>

    static_assert(sizeof(int) == 4, "int must be 4 bytes");  /* Compile-time assertion */

ESP32-Specific Assertions
-------------------------

In ESP32 projects, you have two specific macros for error checking: **`ESP_ERROR_CHECK`** and **`ESP_ERROR_CHECK_WITHOUT_ABORT`**. These are commonly used to catch errors during development, especially when working with ESP-IDF functions.

- **`ESP_ERROR_CHECK`**: This macro checks the return value of an ESP-IDF function and aborts the program if the return code is not `ESP_OK`. It is useful for quickly catching errors during development, but it should not be used for handling recoverable errors in production.

Example:

.. code-block:: c

    esp_err_t err = some_esp_function();
    ESP_ERROR_CHECK(err);  /* Abort if an error occurred */

- **`ESP_ERROR_CHECK_WITHOUT_ABORT`**: This macro works similarly to `ESP_ERROR_CHECK`, but it will not abort the program when an error occurs. This can be useful when you want to log errors without stopping the entire program.

Example:

.. code-block:: c

    esp_err_t err = another_esp_function();
    ESP_ERROR_CHECK_WITHOUT_ABORT(err);  /* Log error but continue execution */

**When to Use**:

- Use `ESP_ERROR_CHECK` during development to catch and abort on critical errors that should never occur.

- Use `ESP_ERROR_CHECK_WITHOUT_ABORT` when you want to log errors but allow the program to continue running.

When to Use Assertions
----------------------

- **Parameter Validation**: Use assertions to validate parameters that are assumed to be valid in internal functions. Public API functions should use proper error handling, while internal functions can use assertions to enforce assumptions about input data.

- **Invariants and Assumptions**: Use assertions to enforce invariants (conditions that should always be true) within algorithms or data structures. For example, asserting that a pointer is not `NULL`, or that a buffer index is within valid bounds.

- **Critical Failures in Development**: Use assertions to catch critical failures during development that should never occur in a well-tested, stable system. These might include invalid state transitions or unhandled edge cases.

General Guidelines
------------------

- **Use assertions for catching programming errors** and invalid states during development.

- **Do not rely on assertions for runtime error handling** in production code.

- **Avoid side effects in assertions** since they are removed in non-debug builds.

- **Use `static_assert` for compile-time checks** when available.

- **Provide comments explaining assertions** to clarify their purpose.

- **Use `ESP_ERROR_CHECK` and `ESP_ERROR_CHECK_WITHOUT_ABORT`** in ESP32 projects to catch and log errors effectively.

Braces
======

Proper brace placement makes code cleaner and easier to read. Follow these guidelines for consistent use of braces in this project:

- **Always Use Braces for Control Statements**: Every `if`, `for`, `while`, or similar control structure must have braces, even if there is only one statement inside. Omitting braces for single statements is not allowed.

Example:

Bad Example:

.. code-block:: c

    if (foo) bar();  /* INCORRECT: Missing braces */

Good Example:

.. code-block:: c

    if (foo) {
      bar();       /* Correct: Braces added */
    }

- **Same Line for Control Structures and Blocks**: For control structures like `if`, `for`, `while`, and blocks like structs and enums, always place the opening brace `{` on the same line as the statement. This rule applies unless the line is excessively long, in which case you may split it, but still aim to keep the brace near the closing parenthesis.

Example 1:
----------

Bad Example:

.. code-block:: c

    if (condition)
    {
      /* INCORRECT */
    }

    for (int i = 0; i < c_const; ++i)
    {
      /* INCORRECT */
    }

Good Example:

.. code-block:: c

    if (condition) {
      /* ... */
    }

    for (int i = 0; i < c_const; ++i) {
      /* ... */
    }

    typedef struct {
      int member;
    } my_struct_t;

- **Functions Have Braces on a New Line**: The only exception to the same-line rule is for functions. For function definitions, place the opening brace `{` on a new line.

Example 2:
----------

Bad Example:

.. code-block:: c

    static void my_function(void) {
      /* INCORRECT */
    }

Good Example:

.. code-block:: c

    static void priv_my_function(void)
    {
      /* Function logic */
    }

General Guidelines
------------------

- Always use braces, even for single statements.

- Keep braces next to the ending parenthesis when splitting long statements.

- Functions are the only structures that have the brace on a new line. All other blocks, including structs and control structures, should keep the brace on the same line.

Switch Statements
=================

- **Same Line for Switch and Opening Brace**: The opening brace `{` for a `switch` statement should be on the same line as the `switch` keyword.

Example:

.. code-block:: c

    switch (x) {
      case 1: {
        /* Code for case 1 */
        break;
      }
      case 2: {
        /* Code for case 2 */
        break;
      }
      case 3: return 0;
      case 4: return 1;
      case 5: return 2;
      default: {
        /* Code for default case */
        break;
      }
    }

- **Braces for Case Blocks**: Each `case` should have its own block of code enclosed in braces `{}`. This ensures clarity and prevents errors when adding new statements to a case.

General Guidelines
------------------

- Keep the opening brace `{` on the same line as the `switch` keyword.
- Enclose each `case` block in braces `{}`.
* ** */* */case 3: return 0


Breaking Long Lines
===================

To maintain readability and prevent horizontal scrolling, it's important to break long lines of code when they exceed the maximum line length. For this project, the maximum line length is **80 characters**. Lines longer than this should be split in a consistent and readable way.

Guidelines
----------

- **Maximum Line Length**: Ensure that no line of code exceeds roughly 80 characters. This includes comments, code, and declarations.

- **Break at Natural Points**: Break lines at natural points, such as after operators (`+`, `-`, `*`, `=`, etc.), commas, or logical groupings to improve readability.

- **Indentation for Continuation Lines**: When breaking lines, ensure the continued line is indented properly for readability. Line up indentation after the line break to maintain visual alignment.

Correct Example:

.. code-block:: c

  int sum = first_variable + second_variable + third_variable +
            fourth_variable;

Incorrect Example:

.. code-block:: c

  int sum = first_variable + second_variable + third_variable
  + fourth_variable;  /* Operator at the start of the next line */

- **Avoid Breaking Inside Function Calls**: Break long function call arguments at commas or logical points but avoid splitting a single argument across multiple lines.

Correct Example:

.. code-block:: c

  my_function(first_argument, second_argument, third_argument,
              fourth_argument);

Incorrect Example:

.. code-block:: c

  my_function(first_argument, second_argument, third_argument, fourth_
  argument);  /* Avoid breaking a single argument across lines */

- **Aligning with Open Parentheses**: If a long statement is broken, the subsequent lines should align with the opening parenthesis or after operators where possible for clarity.

Correct Example:

.. code-block:: c

  if (condition_one && condition_two &&
      condition_three) {  /* Alignment with opening parenthesis */
    /* Code here */
  }

Incorrect Example:

.. code-block:: c

  if (condition_one && condition_two
    && condition_three) {  /* Misaligned continuation */
    /* Code here */
  }

General Guidelines
------------------

- Always keep lines under 80 characters.

- Break at logical points like operators, commas, or parentheses.

- Indent continuation lines by two spaces.

- Avoid breaking single arguments across multiple lines.

- Align continuation lines with parentheses or logical grouping for clarity.

Comments
========

Comments are an essential part of writing readable code, but they should be used effectively. Follow these guidelines to ensure that comments add clarity without being excessive. Since we use Doxygen for generating documentation, **all functions must include Doxygen comments**, and they should follow specific conventions regarding where to place them.

General Commenting Guidelines
-----------------------------

- **Use Block Comments (`/* */`)**: Always prefer `/* */` comments over `//`. Block comments are more versatile and consistent for C projects. Avoid starting block comments with an empty first line.

  - For single-line comments, keep them on one line:

    Correct:

    .. code-block:: c

      /* This is a single-line comment */

    Incorrect:

    .. code-block:: c

      /* This is a single-line comment
       */

  - For comments following variables or statements, align them for consistency:

    Correct:

    .. code-block:: c

      int x;   /* This is x */
      int foo; /* This is foo */

    Incorrect:

    .. code-block:: c

      int x; /* This is x */
      int foo; /* This is foo */

Doxygen Documentation
----------------------

Since we use Doxygen to generate documentation, **every function** must include a Doxygen-style comment block. These comments provide valuable information about what each function does, its parameters, return values, and any important details about usage.

**Where to place Doxygen comments:**

- **Header files (`.h` files)**: Doxygen comments should be placed in header files for public functions and any functions that are shared across multiple source files. This allows the comments to be used for API documentation and ensures that users of your code understand the function's purpose and usage.

- **Source files (`.c` files)**: You may include Doxygen comments in source files for private or static functions that are not exposed through header files. If the function is defined in both a header and source file, place the Doxygen comment in the header file only.

**Doxygen Comment Structure:**

- **@brief**: A short summary of what the function does.

- **@param**: A description of each parameter, including type and usage.

- **@return**: What the function returns, or `void` if nothing is returned.

- **@note**: (Optional) Any additional notes about the function, such as important edge cases or limitations.

Doxygen Example
---------------

For public functions (in header file):

.. code-block:: c

  /**
   * @brief  Adds two numbers.
   * @param  a First number to be added.
   * @param  b Second number to be added.
   * @return The sum of a and b.
   */
  int add(int a, int b);

For private/static functions (in source file):

.. code-block:: c

  /**
   * @brief Initializes the internal counter.
   * @param value Initial value for the counter.
   * @note  This function is only used internally and is static.
   */
  static void init_counter(int value);

Key Doxygen Tags
----------------

- **@brief**: Provides a concise summary of the function.

- **@param**: Describes the function parameters. List each parameter separately.

- **@return**: Describes the return value. If the function does not return a value, you can omit this tag or specify `void`.

- **@note**: Use this for any important additional information, such as side effects, limitations, or edge cases.

- **@warning**: Used for warning the user about potential risks in the function's use.

- **@deprecated**: Marks a function as deprecated, with a note on what to use instead.

Example with Additional Tags:

.. code-block:: c

  /**
   * @brief      Opens a connection to the server.
   * @param      server_address The address of the server.
   * @param      port Port number to use for the connection.
   * @return     0 on success, -1 on error.
   * @warning    Ensure that the server address is valid before calling this function.
   * @deprecated Use `open_server_connection_v2()` instead.
   */
  int open_connection(const char* server_address, int port);

Block Diagrams at the Start of Files
------------------------------------

At the start of each file, when relevant, include a block diagram to represent the current part of the project or module. This provides context for what the file or module does at a glance.

Example:

.. code-block:: c

  /* Block Diagram: This module handles user input
   *  +---------+      +---------+
   *  | Input   | ---> | Handler |
   *  +---------+      +---------+
   */

Purpose of Comments
-------------------

Comments should describe **what** the code does, not **how** it works. Avoid over-commenting or explaining how code works, especially if it's complex. Instead, focus on writing clear and self-explanatory code.

- **Avoid Explaining How Code Works**: Well-written code should be clear enough to convey how it works. Comments should not be used to explain poorly written code.

- **Explain What the Code Does**: Use comments to describe the intent or purpose of code, particularly for more complex functions or logic.

Bad Example
-----------

.. code-block:: c

    int x = 5; /* Set x to 5
                */
    /* This loop increments x by 1 ten times
     */
    for (int i = 0; i < 10; i++) {
      x++;
    }

Good Example
------------

.. code-block:: c

    /* Increments x ten times */
    for (int i = 0; i < 10; i++) {
      x++;
    }

Placing Comments Outside Functions
----------------------------------

- **Head of the Function**: Place comments at the start of functions to describe **what** the function does and, if necessary, **why** it does it. Avoid putting comments inside the function body unless there's something specific that needs a note or a warning.

Example:

.. code-block:: c

  /* Processes user input and updates state
   * Handles edge cases for invalid input
   */
  static void process_input(int input_value)
  {
    /* ... */
  }

General Guidelines
------------------

- Use `/* */` for comments.

- Do not start block comments with an empty first line.

- Use Doxygen formatting for documentation generation (e.g., `@param`, `@return`, `@brief`).

- Use keywords like `TODO:`, `FIXME:`, `BUG:`, and `XXX:` to mark important areas.

- For single-line comments, keep them on one line (e.g., `/* This is a comment */`).

- Align comments after variable declarations or statements.

- Include block diagrams at the start of files when relevant.

- Focus on explaining **what** the code does, not **how** it works.

- Place comments at the head of functions, not inside function bodies.

Concurrency and Parallelism
===========================

Concurrency and parallelism allow a program to perform multiple tasks simultaneously or overlap I/O-bound tasks with computation. In embedded systems like ESP32, efficient use of concurrency is crucial to manage tasks such as networking, sensors, or interrupt handling. This section outlines the best practices for ensuring thread-safe and efficient concurrent programming, along with an explanation of when to use **RTOS types**, **mutexes**, **semaphores**, and **atomic operations**.

General Guidelines
------------------

- **Avoid Global State**: Global variables should be avoided, especially in multi-threaded environments, to prevent race conditions. Use thread-local storage or pass variables explicitly to functions.

- **Thread Safety**: Ensure that shared resources (e.g., data structures, hardware peripherals) are properly synchronized when accessed by multiple threads or tasks. This can be done using RTOS types, mutexes, semaphores, or atomic operations.

- **Minimize Lock Contention**: When using synchronization primitives like mutexes, hold locks for the shortest time possible to avoid contention between threads. Design critical sections carefully to reduce lock duration.

RTOS Types, Mutexes, Semaphores, and Atomic Operations
------------------------------------------------------

When dealing with concurrency, choosing the right synchronization mechanism is key. In embedded systems like ESP32, using RTOS types is often more efficient and suitable than traditional methods. Each has its strengths, limitations, and specific use cases.

**RTOS Types**
RTOS types, such as FreeRTOS tasks, queues, and semaphores, are specifically designed for embedded systems, providing efficient task management and synchronization.

- **What They Do**: RTOS types manage task scheduling, communication, and synchronization in a way that is optimized for embedded environments.

- **How They Work**: RTOS types leverage the real-time operating system's capabilities to ensure efficient and predictable task execution and resource management.

- **When to Use**: Use RTOS types when developing for embedded systems like ESP32, where efficient task management and low overhead are crucial.

  **Example**:

  .. code-block:: c

    void task1(void *pvParameters)
    {
      while (1) {
        /* Task code */
        vTaskDelay(1000 / portTICK_PERIOD_MS); /* Delay for 1 second */
      }
    }

    void app_main(void)
    {
      xTaskCreate(task1, "Task 1", 2048, NULL, 5, NULL);
    }

**Mutexes**
A **mutex** (mutual exclusion) is a synchronization primitive used to protect shared resources by ensuring that only one thread or task can access the resource at any given time. Mutexes are ideal for ensuring **exclusive access** to a critical section of code.

- **What It Does**: A mutex locks access to a resource. When one thread locks a mutex, other threads attempting to lock it must wait until the mutex is unlocked by the original thread.

- **How It Works**: When a thread or task locks a mutex, other threads attempting to lock the same mutex are blocked until the mutex is unlocked. Only the thread that locked the mutex can unlock it.

- **When to Use**: Use a mutex when you need **exclusive access** to a shared resource, such as when modifying shared data structures (e.g., linked lists, buffers) or hardware registers.

  **Example**:

  .. code-block:: c

    static pthread_mutex_t lock;

    void critical_function(void)
    {
      pthread_mutex_lock(&lock);
      /* Critical section */
      pthread_mutex_unlock(&lock);
    }

**Semaphores**
A **semaphore** is a signaling mechanism and can be used to synchronize threads or tasks. Unlike mutexes, semaphores allow multiple threads or tasks to access a resource concurrently, depending on the semaphore count.

- **What It Does**: A semaphore has a counter that allows it to manage access to a pool of resources. Threads can increment or decrement the counter to signal that a resource is available or has been used.

- **How It Works**: Semaphores can either signal threads to wait or proceed depending on the counter value. A binary semaphore (with a counter of 1) behaves similarly to a mutex, while a counting semaphore allows multiple threads to access a shared resource up to the limit set by the counter.

- **When to Use**: Use semaphores when you need to manage **shared resources** that multiple threads can access simultaneously (e.g., a pool of connections, worker threads). Use them in **producer-consumer** models, where one task produces data and another consumes it.

  **Example**:

  .. code-block:: c

    static sem_t semaphore;

    void producer(void)
    {
      /* Produce data */
      sem_post(&semaphore); /* Signal consumer */
    }

    void consumer(void)
    {
      sem_wait(&semaphore); /* Wait for producer */
      /* Consume data */
    }

**Atomic Operations**
**Atomic operations** are a low-level synchronization mechanism that allows certain operations (such as incrementing a counter) to be performed without interference from other threads, without the overhead of using mutexes or semaphores. Atomic operations work directly on the memory in a thread-safe manner.

- **What It Does**: An atomic operation ensures that a specific operation (such as incrementing or comparing) happens **atomically**, meaning without interruption.

- **How It Works**: Atomic operations leverage hardware support to ensure that the operation completes in a single step, without the risk of being interrupted by another thread.

- **When to Use**: Use atomic operations when working with **simple data types** such as counters, flags, or booleans that multiple threads modify. They are highly efficient and should be preferred when the operation is simple and does not require more complex synchronization.

  **Example**:

  .. code-block:: c

    static atomic_int counter;

    void increment(void)
    {
      atomic_fetch_add(&counter, 1);
    }

When to Use RTOS Types, Mutexes, Semaphores, or Atomic Operations
-----------------------------------------------------------------

- **Use RTOS Types** when:

  - Developing for embedded systems like ESP32, where efficient task management and low overhead are crucial.

  - You need to leverage the real-time operating system's capabilities for task scheduling and synchronization.

- **Use Mutexes** when:

  - You need **exclusive access** to a shared resource (e.g., modifying a shared data structure).

  - The resource must only be accessed by one thread at a time.

  - The operation involves multiple steps that must all be done without interruption.

- **Use Semaphores** when:

  - You need to manage **multiple shared resources** that several threads can access concurrently (e.g., a pool of database connections).

  - You are implementing **producer-consumer** scenarios where tasks signal each other to proceed.

- **Use Atomic Operations** when:

  - You need to perform simple operations (e.g., incrementing a counter, flipping a flag) that need to be **thread-safe** without using locks.

  - The performance overhead of mutexes or semaphores is unnecessary due to the simplicity of the operation.

Task Management in ESP32
------------------------

In ESP32, FreeRTOS is the operating system that handles task scheduling, making it easy to create and manage multiple tasks.

- **FreeRTOS Tasks**: Tasks in FreeRTOS are lightweight threads that run concurrently. Each task should have its own stack, and the task's priority should be set based on its importance.

  Example:

  .. code-block:: c

    void task1(void *pvParameters)
    {
      while (1) {
        /* Task code */
        vTaskDelay(1000 / portTICK_PERIOD_MS); /* Delay for 1 second */
      }
    }

    void app_main(void)
    {
      xTaskCreate(task1, "Task 1", 2048, NULL, 5, NULL);
    }

- **Task Prioritization**: Set task priorities according to their criticality. Higher priority tasks will preempt lower priority ones, so avoid setting all tasks to high priority unless necessary.

- **Task Notifications and Queues**: Use task notifications or queues to communicate between tasks. This ensures that tasks can safely pass data to each other without race conditions.

  Example:

  .. code-block:: c

    static xQueueHandle queue;

    void producer_task(void *pvParameters)
    {
      int data = 42;
      while (1) {
        xQueueSend(queue, &data, portMAX_DELAY);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
      }
    }

    void consumer_task(void *pvParameters)
    {
      int received_data;
      while (1) {
        xQueueReceive(queue, &received_data, portMAX_DELAY);
        /* Process data */
      }
    }

    void app_main(void)
    {
      queue = xQueueCreate(10, sizeof(int));
      xTaskCreate(producer_task, "Producer Task", 2048, NULL, 5, NULL);
      xTaskCreate(consumer_task, "Consumer Task", 2048, NULL, 5, NULL);
    }

Concurrency Best Practices
--------------------------

- **Use RTOS Types for Embedded Systems**: Leverage RTOS types like tasks, queues, and semaphores for efficient task management and synchronization in embedded systems.

- **Use Mutexes for Shared Resources**: Protect access to shared resources with mutexes or semaphores to avoid race conditions.

- **Minimize Critical Section Length**: Keep the duration of critical sections short to reduce lock contention between threads.

- **Use Atomic Operations for Simple Data**: When possible, use atomic operations for simple shared variables instead of mutexes to avoid the overhead of locking.

- **Task Prioritization**: Assign proper task priorities in multi-tasking environments. Avoid giving all tasks the same priority, as this can lead to task starvation or inefficient scheduling.

- **Avoid Deadlocks**: Be mindful of potential deadlocks when using multiple mutexes. Ensure that locks are acquired in a consistent order across different parts of the program.

- **Handle Interrupts Carefully**: Interrupt Service Routines (ISRs) should avoid performing complex tasks. Instead, use task notifications or queues to signal a task to handle the actual work after the ISR completes.

- **Concurrency Debugging**: Use FreeRTOS tracing or logging features to monitor task switching and detect issues like priority inversion, starvation, or excessive blocking.

General Guidelines
------------------

- **Minimize Use of Global State**: Avoid global variables to prevent race conditions. Instead, pass data through function arguments or use thread-local storage.

- **Prioritize Task Management**: Use task prioritization wisely to avoid starvation or inefficient scheduling. Ensure that critical tasks run at appropriate priority levels.

- **Protect Shared Resources**: Always use mutexes or semaphores when multiple tasks or threads access shared resources.

- **Use Atomic Operations for Simple Data**: For simple operations like counter increments, use atomic operations instead of mutexes for better performance.

- **Keep ISRs Simple**: Avoid complex logic in interrupt service routines. Use ISRs only to signal tasks, and let tasks handle the logic.

Enums
=====

Enums (enumerations) should be used to define a set of named integer constants, improving code readability and maintainability. Enums group related constants and make the code more self-explanatory. Follow these guidelines for using enums in this project.

General Guidelines for Enums
----------------------------

- **Enum Names Should End in `_t`**: All enum types should end with `_t` to indicate that they are typedefs, ensuring consistency across the project.

- **Use Snake Case for Enum Values**: All enum values should follow snake_case naming conventions, just like variables and function names.

- **Use Descriptive Names for Enum Members**: Enum members should have descriptive names that convey their meaning. Avoid abbreviations or ambiguous terms.

- **Enum Values Should Start with `k_`**: To distinguish enum members from other constants or variables, enum values should start with `k_`. This helps with consistency and clarity in the code.

- **Assign Specific Values Only When Necessary**: Enum members are automatically assigned incremental values starting from `0`. Only assign specific values if they are necessary, such as for protocol definitions or when compatibility is required.

- **Use `typedef enum : uint8_t` for Embedded Systems**: When working with embedded systems, use `typedef enum : uint8_t` to ensure efficient memory usage.

  Example:

  .. code-block:: c

    typedef enum : uint8_t {
      k_error_none  = 0,
      k_error_minor = 1,
      k_error_major = 2
    } error_level_t;

Example 1:
----------

Bad Example:

.. code-block:: c

    typedef enum {
      SUCCESS,
      FAILURE
    } STATUS; /* INCORRECT: Not using snake_case, enum name not ending with _t */

Example 2:
----------

Bad Example:

.. code-block:: c

    typedef enum {
      SUCCESS,
      FAILURE
    } result_t; /* INCORRECT: Enum members not following snake_case */

General Guidelines
------------------

- Always use snake_case for both enum names and members.

- Enum types should end with `_t` and values should start with `k_` for clarity and consistency.

- Use descriptive names for enum members to improve readability.

- Only assign specific values to enum members when necessary.

Error Handling
==============

Effective error handling is critical for building robust and maintainable software. This section outlines the best practices for handling errors in this project, ensuring that all modules use a consistent approach. The general principles of error handling should be applied throughout the codebase, focusing on clear communication of errors and minimizing crashes.

General Guidelines for Error Handling
-------------------------------------

- **Return Error Codes**: Functions should return error codes whenever an error occurs. Use negative values for failure and zero for success. Avoid returning `-1` or `1` generically—use specific error codes to indicate the nature of the failure.

- **Use `errno.h` Error Codes**: Where appropriate, use standard POSIX error codes from `errno.h` (e.g., `EINVAL`, `ENOMEM`, `EIO`). These standardized codes help communicate common error conditions clearly.

  Example:

  .. code-block:: c

    #include <errno.h>

    int process_data(void* data)
    {
      if (!data) {
        return -EINVAL; /* Invalid argument */
      }
      /* Process the data */
      return 0;
    }

- **Handle Errors at the Point of Failure**: Always handle errors as close as possible to where they occur. If a function returns an error code, the caller should check that value and handle the error appropriately.

  Example:

  .. code-block:: c

    int ret = process_data(data);
    if (ret < 0) {
      /* Handle error, log message or clean up */
      return ret;
    }

- **Never Ignore Return Values**: Always check the return value of a function, particularly when dealing with I/O, memory allocation, or any function that could fail.

  Bad Example:

  .. code-block:: c

    malloc(100); /* No check if memory allocation succeeded */

  Good Example:

  .. code-block:: c

    void *ptr = malloc(100);
    if (!ptr) {
      /* Handle memory allocation failure */
      return -ENOMEM;
    }

- **Use `NULL` for Pointer Return Failures**: When a function returns a pointer, return `NULL` to indicate failure. Ensure the caller checks for `NULL` before dereferencing the pointer.

  Example:

  .. code-block:: c

    void *allocate_buffer(size_t size)
    {
      void *buffer = malloc(size);
      if (!buffer) {
        return NULL; /* Return NULL on allocation failure */
      }
      return buffer;
    }

- **Error Propagation**: If a function cannot resolve an error condition, propagate the error code back to the caller rather than masking it.

- **Clean Up on Failure**: Ensure that any allocated resources (e.g., memory, file handles, network sockets) are cleaned up appropriately when an error occurs. Use a consistent strategy, such as `goto` for clean-up sections at the end of functions, to handle error exits efficiently.

  Example:

  .. code-block:: c

    int do_something(void)
    {
      int *ptr = malloc(sizeof(int));
      if (!ptr) {
        return -ENOMEM;
      }

      FILE *file = fopen("data.txt", "r");
      if (!file) {
        free(ptr);
        return -EIO;
      }

      /* Do some work */

      fclose(file);
      free(ptr);
      return 0;
    }

Using `goto` for Clean-up
-------------------------

In C, the `goto` statement can be useful for simplifying error handling when dealing with resource cleanup. Use `goto` to jump to a clean-up section at the end of the function where all allocated resources are safely released.

  Example:

  .. code-block:: c

    int read_file(const char* path)
    {
      FILE* file   = NULL;
      char* buffer = NULL;

      file = fopen(path, "r");
      if (!file) {
        return -EIO;
      }

      buffer = malloc(1024);
      if (!buffer) {
        fclose(file);
        return -ENOMEM;
      }

        /* Do work with file and buffer */

    cleanup:
      if (file) {
        fclose(file);
      }
      if (buffer) {
        free(buffer);
      }

      return 0;
    }

ESP32-Specific Error Handling
-----------------------------

When working with ESP32 and **ESP-IDF**, you should use ESP32's built-in macros and functions for error handling:

- **ESP_ERROR_CHECK()**: Use `ESP_ERROR_CHECK()` to check return values from ESP-IDF functions that return `esp_err_t`. This macro will terminate the program if an error occurs and log the error message.

  Example:

  .. code-block:: c

    esp_err_t ret = esp_wifi_init(&cfg);
    ESP_ERROR_CHECK(ret);

- **ESP_ERROR_CHECK_WITHOUT_ABORT()**: This macro works like `ESP_ERROR_CHECK()` but logs the error without terminating the program. Use this when you want to handle the error gracefully without stopping the system.

  Example:

  .. code-block:: c

    esp_err_t ret = esp_wifi_init(&cfg);
    if (ESP_ERROR_CHECK_WITHOUT_ABORT(ret) != ESP_OK) {
      /* Handle the error */
    }

- **Error Return Codes in ESP-IDF**: The ESP-IDF uses its own set of return codes, usually starting with `ESP_ERR_`. Always return these codes directly when writing ESP-IDF functions, and check them using the macros above.

Best Practices
--------------

- **Consistent Error Handling**: Always return specific error codes and handle them consistently in your code.

- **Use Standard Error Codes**: Where possible, use standard POSIX error codes from `errno.h` to avoid ambiguity.

- **Never Ignore Return Values**: Always check the return values of functions, especially those that can fail (e.g., `malloc`, `fopen`).

- **Clear and Actionable Error Messages**: Log clear and actionable error messages to help diagnose issues quickly.

- **Clean Up on Failure**: Always free allocated memory or close file handles when an error occurs.

General Guidelines
------------------

- **Return Error Codes**: Functions should return error codes where applicable.

- **Use `NULL` for Pointer Failures**: Return `NULL` for pointer failures to indicate allocation issues.

- **ESP32 Macros**: Use `ESP_ERROR_CHECK()` for ESP-IDF error handling to log errors and terminate or handle gracefully.

- **Resource Cleanup**: Use `goto` for clean-up at the end of functions when handling errors.

Functions
=========

In this project, functions should be designed with clear and consistent return values and names to improve readability and prevent bugs. Return values should clearly indicate whether a function succeeded or failed, and function names should reflect the type of return value expected. Braces for function definitions must be placed on the next line, while control structures like `if`, `for`, `while`, etc., should have braces on the same line.

Use `const` and Pass-by-Reference
_________________________________
Whenever possible, parameters should be declared as `const` to indicate that they are read-only. Additionally, prefer passing large structures or data by reference (using pointers) to improve performance. However, prioritize readability if passing by reference makes the function unclear.

Example of `const` and Pass-by-Reference:

.. code-block:: c

    void process_data(const int* data)
    {
      /* Function logic */
    }

Return Value Conventions
------------------------

Functions can return values of many different kinds, but the two most common types are:

- **Error-code integer**: (-Exxx = failure, 0 = success)

- **Succeeded boolean**: (0 = failure, non-zero = success)

Mixing these two types of return values is a common source of bugs. Always follow this convention to avoid confusion:

**Action vs. Predicate Functions**

- If the name of a function is an action (or an imperative command), it should return an **error-code integer**.

- If the name is a predicate (a function that returns true/false), it should return a **succeeded boolean**.

**Example**:

If the function name is an action, return an error-code integer:

.. code-block:: c

    int add_work(void)
    {
      return 0;      /* Success */
      return -EBUSY; /* Failure */
    }

If the function name is a predicate, return a succeeded boolean:

.. code-block:: c

    int pci_dev_present(void)
    {
      return 1; /* Success */
      return 0; /* Failure */
    }

All exported functions must respect this convention, and all public functions should as well. Private (static) functions are not required to follow this convention, but it is recommended.

Private and Exposed Private Functions
--------------------------------------

- **Private Functions**: When possible, private functions should be declared `static` to restrict their scope to the source file.

Example:

.. code-block:: c

    static int calculate_internal_value(int input)
    {
      return input * 2;
    }

- **Exposed Private Functions**: If a function cannot be `static` and must be exposed, prefix its name with `priv_` to clearly indicate that it is meant for internal use and should not be accessed by users of the API.

Example:

.. code-block:: c

    int priv_process_data(int data)
    {
      return data + 10;
    }

General Function Guidelines
----------------------------

- **Use `const` for Parameters**: If a parameter is not going to be modified, always declare it as `const` to ensure clarity and prevent accidental changes.

- **Pass by Reference for Large Data**: When dealing with large data structures, prefer passing by reference to avoid unnecessary copying. However, prioritize readability if passing by reference makes the function less clear.

Example of `const` and Passing by Reference:

.. code-block:: c

    void update_values(const int* values)
    {
      /* Function logic */
    }

- **Clear Names**: Function names should describe what the function does, and the return type should match the behavior of the function name (e.g., use a boolean return type for predicate functions).

- **Static Functions**: Use `static` for private functions that are only used within a single source file. Public functions should be declared in header files and follow the naming conventions.

- **Exposed Private Functions**: Use the `priv_` prefix for functions that cannot be static but are intended for internal use only.

- **Return NULL for Failed Pointer Functions**: For functions that return pointers, use `NULL` to indicate failure.

- **Prefer Early Exits**: Use early `return` statements to handle errors and avoid deeply nested control structures.

- **Always Use Braces for Functions on New Line**: For all function definitions, the opening brace must be on the next line.

- **When you have multiple parameters, put each parameter on a new line and align parameter names**: This makes the function more readable and easier to understand.

Example:

.. code-block:: c

    int my_function(int*              param1,
                    const int** const param2,
                    int               param3)
    {
      /* Function logic */
    }

**Exported Functions**
----------------------

Functions that are exposed publicly in headers should follow the return value conventions described above. They should also be clearly documented using Doxygen, and should avoid returning raw error codes directly to the user when possible.

General Guidelines
------------------

- Use clear and descriptive function names.

- **Always declare parameters as `const` when possible**.

- **Pass by reference for large data**, unless it makes the function unreadable.

- Follow return conventions: action functions return error codes, predicate functions return boolean values.

- Always use `static` for functions not exposed in the header files.

- Use the `priv_` prefix for private functions that cannot be made `static`.

- Use `NULL` for failed pointer returns.

- Avoid deeply nested control structures by using early exits.

- Always include braces around conditional blocks, even for single lines.

- **Function braces must always be placed on the next line**, but control structure braces should stay on the same line.

Globals
=======

In this project, **global variables should be avoided as much as possible**. Global variables can introduce unintended side effects, increase complexity, and make the code harder to debug and maintain. Instead, prefer using local variables, passing arguments to functions, or encapsulating data within structures or objects.

Guidelines
----------

- **Avoid Global Variables**: Global variables can create dependencies between different parts of the code, leading to bugs that are hard to trace. Instead, use function arguments or encapsulation methods like structs to pass data.

- **Use `static` for File Scope**: If a variable needs to persist across function calls but is only used within a single source file, declare it `static` to limit its scope to that file. This helps prevent accidental changes from other parts of the project.

  Example:

  .. code-block:: c

    static int s_counter = 0; /* File-scope variable, limited to this file */

- **Minimize Use of Global Constants**: If you need a constant across multiple files, prefer using `const` or `enum` values in a header file rather than using global variables.

  Example of Using `const` in a Header File:

  .. code-block:: c

    /* In a header file, declare the constant */
    extern const int max_connections;

  .. code-block:: c

    /* In the source file, define the constant */
    const int max_connections = 100;

- **Document Global Variables Clearly**: If a global variable is absolutely necessary, document its purpose clearly and describe how it interacts with other parts of the code. This includes comments explaining why the global is used and any important constraints.

When to Use Global Variables
----------------------------

Global variables should be used **only in the rarest cases** where there is a legitimate reason to have a single shared state across the entire program or module. Even in such cases, you should ensure:

- The variable is `const` to prevent unintended modifications, wherever possible.

- The variable is used across multiple files and cannot be scoped to a single file or function.

- The variable's behavior is well-documented, with clear usage patterns to prevent side effects.

Correct Example (Avoiding Globals)
----------------------------------

.. code-block:: c

    /* Prefer passing arguments or using static variables instead of global variables */

    void process_data(int input) {
      static int s_data_count = 0; /* Limited to this file */
      s_data_count           += input;
      /* ... */
    }

Incorrect Example (Using Globals)
----------------------------------

.. code-block:: c

    int g_data_count = 0; /* Global variable, avoid this */

    void process_data(int input) {
      g_data_count += input; /* Global access, can cause side effects */
    }

General Guidelines
------------------

- **Minimize global variable usage**: Avoid using global variables unless absolutely necessary.

- **Use `static` for file-scoped variables**: To prevent other files from accessing variables that don't need global visibility.

- **Use `const` globals** when constants need to be shared across files, as they are safer and provide better control than mutable globals.

- **Document global variables**: Clearly comment on any global variables, explaining their purpose and the reason they are needed in the context of the project.

Header Files
============

Header files play a crucial role in organizing and structuring a C project. They contain declarations of functions, constants, macros, and types that can be shared across multiple source files. This section provides guidelines for creating clear, maintainable, and reusable header files, along with the proper use of header guards.

General Guidelines for Header Files
-----------------------------------

- **Header Files Should Only Contain Declarations**: Header files should declare, but not define, functions, variables, and types. Function implementations, global variable definitions, and other logic should remain in the corresponding source files (`.c`).

  Example:

  .. code-block:: c

    /* header file: math_utils.h */
    #ifndef MATH_UTILS_H
    #define MATH_UTILS_H

    int add(int a, int b); /* Declaration only */

    #endif /* MATH_UTILS_H */

  The implementation will go in the source file:

  .. code-block:: c

    /* source file: math_utils.c */
    #include "math_utils.h"

    int add(int a, int b) {
      return a + b; }

- **Minimal Includes in Headers**: Header files should include only the headers that are strictly necessary for their declarations. Avoid excessive includes to reduce dependency chains and compilation time. Use forward declarations where possible.

  Example:

  .. code-block:: c

    /* Avoid including unnecessary headers */
    struct device_t; /* Forward declaration instead of including device.h */

    void init_device(struct device_t* device);

- **Self-Contained Headers**: Ensure that each header file is self-contained, meaning it can be included independently without relying on the inclusion of other headers beforehand. Always include the necessary headers for types and constants used within the file.

  Example:

  .. code-block:: c

    #ifndef MY_HEADER_H
    #define MY_HEADER_H

    #include <stdint.h> /* Ensure the necessary headers are included */

    typedef struct {
      uint32_t id;
      uint8_t  status;
    } device_t;

    #endif /* MY_HEADER_H */

- **Use `extern` for Global Variables**: If global variables need to be declared in a header file, they should be declared using `extern` to avoid multiple definition errors.

  Example:

  .. code-block:: c

    /* Declaration in header file: */
    extern int global_counter;

    /* Definition in source file: */
    int global_counter = 0;

- **Documentation in Headers**: Use Doxygen-style comments for documenting functions, constants, and types in header files. This allows the automatic generation of documentation and makes the code more understandable to other developers.

  Example:

  .. code-block:: c

    /**
     * @brief  Adds two integers.
     * @param  a First integer.
     * @param  b Second integer.
     * @return The sum of a and b.
     */
    int add(int a, int b);

- **Consistency in Naming Conventions**: Use consistent naming conventions for types, functions, and constants in headers. Follow the naming conventions specified in this project's guidelines (e.g., `snake_case` for variables, `PascalCase` for structs, and `_t` for typedefs).

  Example:

  .. code-block:: c

    typedef struct {
      int x;
      int y;
    } point_t;

Header Guards
-------------

Header guards prevent multiple inclusions of the same header file and avoid issues like redefinition of types, constants, or functions. Proper use of header guards ensures that each header file is only included once during compilation.

- **Use Include Guards in Every Header File**: All header files must use include guards to prevent multiple inclusions.

Example of a Header Guard:

.. code-block:: c

    #ifndef MY_HEADER_H
    #define MY_HEADER_H

    /* Declarations and definitions */

    #endif /* MY_HEADER_H */

- **Naming Convention for Header Guards**: The macro used for header guards should follow a consistent naming convention to avoid conflicts with other projects or libraries. It should be based on the file name, converted to uppercase, with words separated by underscores (`_`), and ending with `_H`.

Example:

For a file named `device_manager.h`:

.. code-block:: c

    #ifndef DEVICE_MANAGER_H
    #define DEVICE_MANAGER_H

    /* Declarations for device manager module */

    #endif /* DEVICE_MANAGER_H */

- **Avoid Underscore Prefixes**: Do not use leading underscores in the header guard macro name, as these are reserved for use by the C standard library.

Bad Example:

.. code-block:: c

    #ifndef _DEVICE_MANAGER_H
    #define _DEVICE_MANAGER_H
    /* INCORRECT: Leading underscores are reserved by the standard */

    #endif /* _DEVICE_MANAGER_H */

- **Use Unique Names**: Ensure that header guard names are unique to the project. In larger projects with many files, it is a good practice to prefix the header guard with the project or module name to avoid potential conflicts with other projects or libraries.

Example:

.. code-block:: c

    #ifndef PROJECT_NAME_DEVICE_MANAGER_H
    #define PROJECT_NAME_DEVICE_MANAGER_H

    /* Declarations for device manager module */

    #endif /* PROJECT_NAME_DEVICE_MANAGER_H */

- **Place Header Guards at the Very Beginning**: The `#ifndef`, `#define`, and `#endif` should be the very first and last lines in the header file, ensuring that the entire file is protected from multiple inclusions.

Bad Example:

.. code-block:: c

    /* Some comment or code */

    #ifndef DEVICE_MANAGER_H
    #define DEVICE_MANAGER_H

    /* Declarations for device manager module */

    #endif /* DEVICE_MANAGER_H */
    /* INCORRECT: Code placed before the include guard */

Good Example:

.. code-block:: c

    #ifndef DEVICE_MANAGER_H
    #define DEVICE_MANAGER_H

    /* Declarations for device manager module */

    #endif /* DEVICE_MANAGER_H */
    /* CORRECT: Include guard is the very first and last thing in the file */

ESP32-Specific Header Guards
----------------------------

When working with the ESP32 platform, follow the same conventions for header guards. However, to avoid conflicts with the ESP-IDF or other external libraries, it is recommended to include the project's name or a module-specific prefix in the guard names.

Example:

For a file named `wifi_manager.h` in a project called `my_project`:

.. code-block:: c

    #ifndef MY_PROJECT_WIFI_MANAGER_H
    #define MY_PROJECT_WIFI_MANAGER_H

    /* Declarations for Wi-Fi manager */

    #endif /* MY_PROJECT_WIFI_MANAGER_H */

When to Use Header Guards
-------------------------

- **Every Header File**: All header files must use header guards to prevent multiple inclusion issues.

- **Use Project/Module Prefixes**: In larger projects, always use a unique prefix to avoid conflicts with third-party libraries.

General Guidelines
------------------

- Always use `#ifndef`, `#define`, and `#endif` for header guards.

- The header guard should be based on the file name, converted to uppercase with underscores separating words, and end with `_H`.

- Avoid leading underscores in header guard names.

- Use project or module-specific prefixes for large projects to avoid name conflicts.

- Place header guards at the very top and bottom of the file.

- Keep header files focused on declarations, not definitions.

- Include only necessary headers to avoid dependency chains and reduce compilation time.

- Use forward declarations where possible to minimize includes.

- Document all functions, constants, and types in header files using Doxygen-style comments.

Horizontal Space
================

Proper use of horizontal space helps make the code more readable and maintainable. Follow these guidelines for horizontal spacing:

- **Space After Conditional and Loop Keywords**: Always add a single space after keywords like `if`, `switch`, `for`, and `while`. The opening brace `{` should be on the same line as the keyword. This improves readability by keeping the code concise.

Example 1:
----------

Bad Example:

.. code-block:: c

    if(condition) {  /* INCORRECT */
      /* ... */
    }

    switch (n){
    case 0:
      /* ... */
    }

Good Example:

.. code-block:: c

    if (condition) { /* correct */
      /* ... */
    }

    switch (n) {
      case 0:
        /* ... */
    }

- **Binary Operators**: Add a single space around binary operators like `+`, `-`, `=`, and `&&`. For multiplication (`*`) and division (`/`) operators, spaces should always be used for clarity.

Example 2:
----------

Bad Example:

.. code-block:: c

    const int y = y0+(x-x0)*(y1-y0)/(x1-x0);    /* INCORRECT */

Good Example:

.. code-block:: c

    const int y     = y0 + (x - x0) * (y1 - y0) / (x1 - x0); /* correct */
    int       y_cur = -y;                                    /* correct */

- **Alignment of Variables, Assignments, and Comments**: Align the `=` signs when declaring multiple variables, and line up the comments for consistency.

Example 3:
----------

Bad Example:

.. code-block:: c

    int foo = 12; /* This is foo */
    int large_foo = 32;    /* This is large foo */

Good Example:

.. code-block:: c

    int foo       = 12; /* This is foo */
    int large_foo = 32; /* This is large foo */

- **Optional Alignment for Function Arguments**: Horizontal space can sometimes be used within a line to align function arguments, improving readability, but use this sparingly. Excessive alignment can cause issues if future lines need to be added or removed.

Example 4:
----------

.. code-block:: c

    esp_rom_gpio_connect_in_signal(PIN_CAM_D6,   I2S0I_DATA_IN14_IDX, false);
    esp_rom_gpio_connect_in_signal(PIN_CAM_D7,   I2S0I_DATA_IN15_IDX, false);
    esp_rom_gpio_connect_in_signal(PIN_CAM_HREF, I2S0I_H_ENABLE_IDX,  false);
    esp_rom_gpio_connect_in_signal(PIN_CAM_PCLK, I2S0I_DATA_IN15_IDX, false);

**General Guidelines**:

- Avoid using TAB characters for horizontal alignment.

- Never add trailing whitespace at the end of the line.

Include Statements
==================

In C projects, **include statements** are used to bring in the necessary header files for a module to function correctly. Proper management of include statements helps reduce compilation time, avoid dependency issues, and maintain clear project structure. This section outlines the best practices for managing include statements in header and source files.

General Guidelines for Include Statements
-----------------------------------------

- **Use Angle Brackets for System/Library Headers**: When including standard library headers or external library headers, always use angle brackets (`< >`). This ensures that the compiler searches the system directories first.

  Example:

  .. code-block:: c

    #include <stdio.h>
    #include <stdint.h>

- **Use Quotation Marks for Project Headers**: When including project-specific headers, use double quotation marks (`" "`). This tells the compiler to first search the project's directories for the file.

  Example:

  .. code-block:: c

    #include "my_project.h"
    #include "device_manager.h"

- **Minimal Includes in Header Files**: Include only the necessary headers in your header files to reduce unnecessary dependencies. Use forward declarations instead of including entire header files whenever possible.

  Example of Forward Declaration:

  .. code-block:: c

    /* Forward declare struct device_t instead of including its full header */
    struct device_t;

    void init_device(struct device_t *device); /* Declaration without including "device.h" */

  By using forward declarations, you reduce the number of headers included in header files, improving compilation times and avoiding circular dependencies.

Forward Declaration Explained
-----------------------------

Forward declarations allow you to declare a type or function without providing its full definition. This is particularly useful when only a pointer to a type is required, or when you need to avoid circular dependencies between header files.

When to Use Forward Declarations
--------------------------------

- **For Structs**: Use forward declarations when you only need a pointer to a struct, and you do not need to access its members.

- **For Functions**: Use forward declarations in header files to declare the existence of a function without defining its logic. The full implementation can be placed in the corresponding `.c` file.

Example:

.. code-block:: c

    /* Forward declaration of struct device_t */
    struct device_t;

    /* Function that only needs a pointer to device_t */
    void init_device(struct device_t *device);

Full Definition of Forward Declared Struct
------------------------------------------

In the source file (or another header file), the full definition of the struct can be provided:

.. code-block:: c

    struct device_t {
      int id;
      int status;
    };

Benefits of Forward Declarations
--------------------------------

- **Reduced Dependencies**: By using forward declarations, you can minimize the number of headers that need to be included in a file. This reduces dependency chains and improves compilation time.

- **Avoid Circular Dependencies**: Forward declarations help break circular dependencies, which can occur when two headers include each other.

- **Simpler and Cleaner Code**: By declaring only what is necessary, your headers remain simpler and easier to maintain.

When Not to Use Forward Declarations
------------------------------------

- **When accessing struct members**: If you need to access or modify the members of a struct, you must include the full definition of the struct.

- **For complex types**: If the type is used heavily throughout a file, it may be clearer to include the full definition to avoid confusion.

Include Complete Headers in Source Files
----------------------------------------

While header files should include minimal dependencies, source files should include the full set of headers they require. This ensures that all dependencies are fully met during compilation.

Example:

.. code-block:: c

    #include <stdio.h>
    #include "device.h"

    void print_device_info(device_t *device)
    {
      printf("Device ID: %d\n", device->id);
    }

Order of Includes
-----------------

Include statements should be organized in a specific order to improve readability and minimize conflicts:

1. First include the corresponding header file for the source file.

2. Then include any external or system headers.

3. Lastly, include any project-specific headers.

Example:

.. code-block:: c

    #include "my_source.h" /* Corresponding header file */
    #include <stdio.h>     /* Standard library headers */
    #include "device.h"    /* Project-specific headers */

Guard Against Redundant Includes
--------------------------------

Avoid including the same header multiple times in a file, especially within a single source file. Header guards ensure that headers are only included once during the compilation process, but it's still important to include only what's necessary.

Use `#include` in the Correct Location
--------------------------------------

Avoid placing `#include` statements in the middle of functions or blocks. Include all necessary headers at the top of the file for clarity.

When to Include Headers
-----------------------

- **Always Include What You Use**: Each source file should include the necessary headers for the functions and types it uses. Do not rely on indirect includes (headers included by other headers). This makes dependencies explicit and easier to track.

- **Forward Declarations Instead of Full Includes**: In header files, use forward declarations whenever possible to avoid pulling in unnecessary dependencies.

- **Place Includes at the Top**: Always place `#include` statements at the top of the file, before any other code.

Indentation
===========

Consistent indentation is crucial for maintaining code readability and structure. This project follows the rule of **2 spaces** for indentation, without using tabs. Using spaces ensures uniformity across different environments and editors.

- **2-Space Rule**: Each level of indentation should use 2 spaces, ensuring the code is easy to read and properly nested. Avoid using tabs, as different editors may interpret them inconsistently, leading to misaligned code.


Bad Example
-----------

.. code-block:: c

  /* Incorrect example with 4-space indentation */

  static void process_input(int input_value)
  {
    if (input_value > 0) {
      s_threshold = 10;
    } else {
      s_threshold = -1;
    }
  }

---

Good Example
------------

.. code-block:: c

  /* Example of good indentation with 2-space rule */

  static void process_input(int input_value)
  {
    if (input_value > 0) {
      s_threshold = 10;
    } else {
      s_threshold = -1;
    }
  }

  static void calculate_output(void)
  {
    int result = 0;
    result     = s_threshold + 1;
  }

Notes:

- Make sure each indentation level is exactly 2 spaces.

- Mixing tabs and spaces should be avoided at all costs to prevent misalignment in different editors.

- Consistent indentation makes the structure of the code clear, improving maintainability and readability for all contributors.

Coding Style
===============

This section covers the coding style for building the topographical robot.

.. toctree::
   :maxdepth: 2
   :caption: Coding Style

   reasoning
   naming
   indentation
   horizontal_space
   vertical_space
   line_endings
   breaking_long_lines
   braces
   static_and_const
   comments
   pointers
   type_definitions
   types
   structures
   enums
   functions
   assertions
   macros
   globals
   register_transfer_level
   logging
   asm_inline_asm
   header_files
   include_statements
   error_handler
   concurrency_and_parllelism
   memory_management
   project_structure

Line Endings
============

In this project, **Unix-style line endings** (`LF`, represented as ``\n``) are mandatory across all files. This ensures consistency, particularly when working on cross-platform projects where differences in line endings can cause unnecessary diffs or issues in version control.

Guidelines
----------

- **Always use Unix-style (`LF`) endings**: Ensure that every file in the project, whether it's source code, header files, scripts, or configuration files, uses Unix-style line endings.

- **Avoid Windows-style (`CRLF`) endings**: Windows uses `CRLF` (``\r\n``) line endings, which should be avoided to prevent inconsistencies. Many version control systems and code editors can introduce unwanted `CRLF` line endings, so make sure to configure your tools appropriately.

- **Editor Configuration**: Set your text editor or IDE to automatically use Unix-style line endings for all files in the project. In most editors, this can be done by adjusting the settings:

  - **VS Code**: Ensure `"files.eol": "\n"` is set in your settings.

  - **Vim/Neovim**: Use `set fileformat=unix`.

  - **Sublime Text**: Enable ``\n`` in the line endings preferences.

  - **Git**: Set the global config with `git config --global core.autocrlf input` to ensure Git handles line endings correctly.

Checking for Line Endings
-------------------------

- **Git**: Git can automatically detect and handle line endings. Use the following command to prevent Git from converting line endings:

  .. code-block:: bash

    git config --global core.autocrlf input

- **Editor Plugins**: Many text editors support plugins or extensions that can detect and fix line ending issues. Consider using one to enforce consistent Unix-style line endings.

Why Unix Line Endings?
----------------------

- **Cross-Platform Compatibility**: Unix-style line endings are standard on most Unix-like systems (Linux, macOS), and maintaining this consistency helps ensure compatibility across all platforms.

- **Version Control**: Using a consistent line ending format prevents unnecessary diffs in version control. Mixing `LF` and `CRLF` line endings can result in changes being flagged that aren't actual code changes.

General Guidelines
------------------

- Ensure all files use Unix-style (`LF`) line endings.

- Configure your text editor to enforce Unix-style line endings.

- Use Git settings to automatically handle line endings during commits.

- Avoid mixing line endings in the same file to maintain consistency.

Logging
=======

In this project, logging is handled using the ESP-IDF logging macros for different verbosity levels. These macros provide structured and meaningful logging to ensure efficient debugging and runtime analysis.

Several macros are available for different verbosity levels:

- **ESP_LOGE** - Error (lowest)

- **ESP_LOGW** - Warning

- **ESP_LOGI** - Info

- **ESP_LOGD** - Debug

- **ESP_LOGV** - Verbose (highest)

Use these macros in your code to provide relevant logging based on the severity of the message.

Guidelines
----------

- **Use Descriptive Log Messages**: Log messages should include relevant context (e.g., variable values, function names). Avoid vague log messages that lack useful information.

  Correct Example:

  .. code-block:: c

    ESP_LOGI("network", "Connection established to server at %s:%d", server_address, port);

  Incorrect Example:

  .. code-block:: c

    ESP_LOGI("network", "Connected");  /* Lacks essential context */

- **Log at Appropriate Levels**: Choose the correct logging level based on the importance of the message. The ESP logging macros support the following levels:

  - **ESP_LOGE**: Log errors that require immediate attention but do not stop the system.

  - **ESP_LOGW**: Log warnings indicating potential problems that might not immediately affect execution.

  - **ESP_LOGI**: Log informational messages about successful operations or system state.

  - **ESP_LOGD**: Log detailed debug information for development and debugging purposes.

  - **ESP_LOGV**: Log very detailed verbose messages, mainly for in-depth debugging.

  Example of Logging Levels:

  .. code-block:: c

    ESP_LOGD("input", "Processing input data: %d", input_data);
    ESP_LOGI("system", "Data processed successfully.");
    ESP_LOGW("input", "Input value close to maximum limit: %d", input_value);
    ESP_LOGE("memory", "Failed to allocate memory for buffer");
    ESP_LOGV("system", "Detailed system log for verbose debugging");

- **Avoid Excessive Logging**: Too many logs, especially at the `DEBUG` or `VERBOSE` level, can flood the log output and make important messages harder to find. Only log what's necessary, and ensure that verbose logging can be disabled in production environments.

  Correct Example:

  .. code-block:: c

    if (input_value > THRESHOLD) {
      ESP_LOGW("input", "Input value exceeded threshold: %d", input_value);
    }

  Incorrect Example:

  .. code-block:: c

    ESP_LOGD("input", "Checking input value...");
    ESP_LOGD("input", "Comparing input value with threshold...");
    ESP_LOGD("input", "Input value comparison done.");

- **Log Error Codes and Conditions**: Always log error codes or relevant conditions when logging errors, so the source of the issue is clear.

  Example:

  .. code-block:: c

    if (result == ERROR_CODE) {
      ESP_LOGE("operation", "Operation failed with error code %d", result);
    }

- **Include Timestamps in Logs**: ESP logging macros automatically include timestamps, which helps track when events occurred and aids in understanding the sequence of events.

- **Use Conditional Logging for Performance**: Avoid excessive logging in performance-critical sections of code. If logging is necessary, use conditional logging to reduce performance impact.

  Example:

  .. code-block:: c

    if (LOG_LOCAL_LEVEL >= ESP_LOG_DEBUG) {
      ESP_LOGD("performance", "Processing data: %d", data);
    }

Logging Format
--------------

In ESP-IDF, the log format includes:

1. **Timestamp**: The time the log message was generated.

2. **Log Level**: The severity of the message (e.g., ERROR, WARN, INFO).

3. **Tag**: The module or component generating the log (e.g., "network", "system").

4. **Message**: The log message itself, which should be clear and descriptive.

5. **Context**: Any relevant context, such as variable values or error codes.

Example:

.. code-block:: text

    [I][network:123] Connection established to server at 192.168.0.1:8080
    [E][memory:456] Operation failed with error code -2

When to Log
-----------

- **Initialization**: Log important initialization steps, such as starting modules or establishing connections.

- **State Changes**: Log state transitions, such as when a system starts or stops a service.

- **Errors and Exceptions**: Always log errors and exceptions, along with relevant context to aid debugging.

- **Warnings**: Log conditions that may cause problems, even if they don't immediately impact the system (e.g., near-limit conditions).

- **Critical Operations**: Log significant operations like database transactions or external API calls.

- **Shutdown**: Log when the system is shutting down or cleaning up resources.

General Guidelines
------------------

- Use descriptive log messages that include relevant context (e.g., variable values, error codes).

- Log at appropriate levels using ESP_LOGE, ESP_LOGW, ESP_LOGI, ESP_LOGD, and ESP_LOGV.

- Avoid excessive logging, especially at the `DEBUG` and `VERBOSE` levels.

- Include error codes and conditions in error logs.

- Use conditional logging in performance-critical code.

- Leverage ESP logging macros, which automatically include timestamps and tags for easier log tracking.

Macros
======

In this project, **macros** should be used sparingly and only when absolutely necessary. While macros can provide some flexibility, they can also make code harder to read, debug, and maintain due to their lack of type safety and visibility. Prefer `const` variables, `inline` functions, or enums wherever possible to avoid the downsides of macros.

Guidelines
----------

- **Avoid Unnecessary Macros**: Always consider using `const`, `inline` functions, or enums before defining a macro. These alternatives offer type safety, debugging support, and better readability.

Example of Using `const` Instead of a Macro
-------------------------------------------

.. code-block:: c

    /* Macro for a constant */
    #define MAX_BUFFER_SIZE 1024

    /* Preferred: Using const instead of macro */
    const int max_buffer_size = 1024;

- **Use All Uppercase for Macro Names**: Macro names should be written in **all uppercase** to distinguish them from variables and functions. Use underscores (`_`) to separate words.

  Correct Example:

  .. code-block:: c

    #define MAX_BUFFER_SIZE 1024

  Incorrect Example:

  .. code-block:: c

    #define maxBufferSize 1024

- **Always Use Parentheses Around Macro Arguments**: When defining macros that take arguments, always use parentheses around both the arguments and the macro body to avoid unexpected behavior due to operator precedence.

  Correct Example:

  .. code-block:: c

    #define SQUARE(x) ((x) * (x))

  Incorrect Example:

  .. code-block:: c

    #define SQUARE(x) x * x

Example of Using `inline` Instead of a Macro
--------------------------------------------

While **inline** functions can be useful in replacing macros and improving type safety, excessive use of `inline` can lead to performance issues. As described in the Linux Kernel guidelines, an overuse of inlining increases the memory footprint and cache usage, which can result in slowdowns due to cache misses and increased memory usage. It's important to inline only small functions that are called frequently.

Guidelines for Using `inline`:

- **Inline small functions**: Only functions with fewer than three lines should generally be inlined.

- **Avoid inlining large functions**: Functions with complex logic should not be inlined as they increase the instruction cache footprint and cause performance degradation.

- **Let the compiler decide**: The compiler is usually smart enough to automatically inline functions where it makes sense, especially for static and frequently used functions. Avoid explicitly forcing `inline` for functions unless absolutely necessary.

Correct Example:

.. code-block:: c

    /* Good inline example: Small and simple function */
    inline int square(int x)
    {
      return x * x;
    }

Incorrect Example:

.. code-block:: c

    /* Bad inline example: Larger function, should not be inlined */
    inline int process_data(int data[], int length)
    {
      int result = 0;
      for (int i = 0; i < length; ++i) {
        result += data[i];
      }
      return result;
    }

- **Enclose Macro Definitions in Blocks**: If the macro body contains more than one statement, enclose the macro body in a `do { ... } while(0)` block to ensure it behaves like a single statement and can be used safely in conditional logic.

  Correct Example:

  .. code-block:: c

    #define SWAP(a, b) do {                \
                         int temp  = (a);  \
                         (a)       = (b);  \
                         (b)       = temp; \
                       } while (0)

  Incorrect Example:

  .. code-block:: c

    #define SWAP(a, b) int temp = a; a = b; b = temp;

- **Minimize Macro Scope**: Macros can easily introduce global visibility, making them difficult to track and debug. To avoid issues, keep the scope of macros as limited as possible and prefer function-like macros only where necessary.

- **Document Macros Clearly**: Always provide a clear comment describing the purpose of the macro. Since macros can be harder to trace during debugging, ensure that their usage and intention are well documented.

Correct Example:

.. code-block:: c

    /* Macro to swap two integers */
    #define SWAP(a, b) do {                 \
                          int temp  = (a);  \
                          (a)       = (b);  \
                          (b)       = temp; \
                        } while (0)

    /* Macro to calculate the square of a number */
    #define SQUARE(x) ((x) * (x))

Incorrect Example:

.. code-block:: c

    #define SWAP(a, b) int temp = a; a = b; b = temp;
    #define SQUARE(x) x * x

When to Use Macros
------------------

Use macros when other options such as `const` or `inline` functions are not viable, such as:

- **Conditional Compilation**: Macros can be useful for including or excluding code based on certain conditions, especially in cross-platform projects or for debugging.

  Example:

  .. code-block:: c

    #ifdef DEBUG
    #define LOG(msg) printf("DEBUG: %s\n", msg)
    #else
    #define LOG(msg) ((void)0)
    #endif

- **Header Guards**: Macros are essential for preventing multiple inclusions of header files.

  Example:

  .. code-block:: c

    #ifndef MY_HEADER_H
    #define MY_HEADER_H

    /* Header content */

    #endif /* MY_HEADER_H */

General Guidelines
------------------

- Prefer `const`, `inline` functions, and enums over macros when possible.

- Use all uppercase for macro names with underscores to separate words.

- Always enclose macro arguments in parentheses and use blocks for multi-statement macros.

- Keep macros' scope limited and document their purpose clearly.

- Use macros for conditional compilation and header guards where necessary.

Style Note
----------

When writing multi-line macros, always **align the backslashes (`\\`) vertically** for consistency and readability, similar to how variables and comments are aligned. This improves readability and maintains a uniform coding style across the project.

Memory Management
=================

Proper memory management is essential in any software project, but it becomes even more critical in embedded systems like the ESP32, where memory is a limited resource. This section outlines the best practices for managing memory efficiently and avoiding common pitfalls such as memory leaks, heap fragmentation, and stack overflows.

General Guidelines
------------------

- **Prefer Static Allocation**: Whenever possible, use static memory allocation instead of dynamic allocation (e.g., `malloc`/`free`) to prevent heap fragmentation and improve performance.

  - **Static Memory**: This memory is allocated at compile time and persists throughout the program's lifetime. It's ideal for variables that need to exist for the entire duration of the program.

  Example:

  .. code-block:: c

    static int buffer[1024];

  - **Dynamic Memory**: This memory is allocated at runtime using functions like `malloc` and `free`. While flexible, dynamic memory can lead to fragmentation over time, particularly in long-running applications.

  Example:

  .. code-block:: c

    int* buffer = malloc(1024 * sizeof(int));
    if (!buffer) {
      /* Handle allocation failure */
    }
    free(buffer);

- **Use Dynamic Memory Sparingly**: In memory-constrained environments, minimize the use of dynamic memory. If dynamic memory must be used, ensure that all allocations are matched with corresponding deallocations to prevent memory leaks.

- **Avoid Heap Fragmentation**: Fragmentation occurs when free memory is divided into small, non-contiguous blocks, making it difficult to allocate larger blocks of memory. Use dynamic memory only for objects with well-defined lifetimes, and consider static allocation for long-lived data.

- **Check for Memory Allocation Failures**: Always check the return value of `malloc`, `calloc`, or `realloc` to ensure that memory was successfully allocated. If memory allocation fails, handle the failure gracefully.

  Example:

  .. code-block:: c

    int* buffer = malloc(1024 * sizeof(int));
    if (!buffer) {
      /* Handle allocation failure */
      return -ENOMEM;
    }

- **Free Dynamically Allocated Memory**: Ensure that all dynamically allocated memory is properly deallocated using `free`. Failing to do so will lead to memory leaks.

  Example:

  .. code-block:: c

    free(buffer);

Stack vs. Heap
--------------

In embedded systems, memory is divided into the **stack** and the **heap**. Understanding the differences and when to use each is critical for efficient memory management.

- **Stack**: The stack is a region of memory used for local variables and function calls. It is fast, but limited in size. Exceeding the stack size can lead to stack overflows, which are difficult to debug.

  - Use the stack for small, short-lived variables (e.g., local variables in a function).

  - Be mindful of recursive functions or deep call chains, which can consume a lot of stack space.

  Example:

  .. code-block:: c

    void function(void) {
      int local_array[100];  /* Stored on the stack */
    }

- **Heap**: The heap is used for dynamically allocated memory (e.g., `malloc`). The heap offers more flexibility but is prone to fragmentation over time.

  - Use the heap for large or variable-sized data that needs to persist across function calls.

  Example:

  .. code-block:: c

    int* buffer = malloc(100 * sizeof(int));  /* Stored on the heap */
    if (buffer) {
      free(buffer);  /* Free the memory when no longer needed */
    }

- **Stack Size**: Pay attention to the stack size of each task in an embedded system. Underestimating the required stack size can lead to stack overflows, while overestimating it wastes valuable memory.

Memory Management in FreeRTOS (ESP32)
-------------------------------------

FreeRTOS provides flexible memory management for tasks, queues, and semaphores. Follow these best practices when using FreeRTOS:

- **Static vs. Dynamic Task Allocation**: FreeRTOS allows tasks to be created either with dynamic or static memory allocation. Use **static allocation** (`xTaskCreateStatic()`) for long-lived tasks to avoid heap fragmentation.

  Example:

  .. code-block:: c

    static StackType_t  task_stack[1024];
    static StaticTask_t task_buffer;

    void task1(void* pvParameters)
    {
      while (1) {
        /* Task code */
      }
    }

    void app_main(void)
    {
      xTaskCreateStatic(task1, "Task 1", 1024, NULL, 5, task_stack, &task_buffer);
    }

- **Use Heap Memory Efficiently**: FreeRTOS has multiple heap allocation schemes (e.g., heap_1, heap_2, heap_3, heap_4). Choose the heap memory management scheme that best fits your project's needs. For projects with long runtimes, consider using `heap_4` for its ability to handle fragmentation better.

Memory Leaks and Debugging
--------------------------

Memory leaks occur when dynamically allocated memory is not properly freed. Over time, memory leaks can deplete available memory, causing the system to crash or behave unpredictably. Follow these guidelines to prevent and detect memory leaks:

- **Ensure Balanced Allocation and Deallocation**: Every call to `malloc`, `calloc`, or `realloc` should have a corresponding call to `free`. Keep track of dynamically allocated memory, especially in complex systems.

- **Use Debugging Tools**: Use memory debugging tools like Valgrind, AddressSanitizer, or built-in logging mechanisms in ESP-IDF to detect memory leaks and improper memory usage.

  - In ESP-IDF, you can use `heap_caps_print_heap_info()` to get information on the state of the heap and detect potential memory issues.

- **Memory Leak Debugging in FreeRTOS**: FreeRTOS provides built-in mechanisms to help track memory usage, such as the `vTaskGetRunTimeStats()` function and FreeRTOS trace capabilities, which can help track down memory leaks and stack overflows.

Best Practices
--------------

- **Prefer Static Allocation**: Use static allocation for global or long-lived data to prevent heap fragmentation.

- **Use Dynamic Allocation Sparingly**: If dynamic memory must be used, ensure proper deallocation and avoid using dynamic memory for long-lived objects.

- **Check for Memory Allocation Failures**: Always verify that memory allocation was successful, and handle allocation failures gracefully.

- **Avoid Memory Leaks**: Track all memory allocations and ensure that every allocation has a corresponding deallocation.

- **Manage Stack Size**: Ensure that each task or function has enough stack space to avoid stack overflows.

- **Use FreeRTOS Static Allocation**: For FreeRTOS tasks, use static allocation where possible to avoid heap fragmentation in long-running tasks.

- **Monitor and Debug Memory Usage**: Use tools like Valgrind or ESP-IDF's heap monitoring functions to track memory usage and detect leaks or stack overflows.

Naming
======

Choosing clear and meaningful names for variables, functions, classes, and other identifiers is crucial in any project, especially one with many contributors:

- **Clarity**: Good names provide clarity about the purpose and functionality of code elements. Descriptive names make it easier for others (and your future self) to understand the code without needing to dig through the implementation.

- **Consistency**: Using a consistent naming convention across the project ensures that all developers follow the same rules, making the codebase uniform. The following conventions should be followed:

  - **Static Declarations**: Any variable or function that is only used in a single source file should be declared `static`. Static variables should be prefixed with `s_` for easy identification (e.g., `static bool s_invert`).

  - **Namespacing**: Public names (non-static variables and functions) should be namespaced with a per-component or per-unit prefix to avoid naming collisions. For example, use `mycomp_vfs_register()` or `mycomp_console_run()` to clearly indicate the source of the function or variable. If a prefix like `mycomp_` is chosen, it should be consistent across all names within that component.

  - **Enum and Struct Typedefs**: All enums and structs should use snake_case and end with `_t` to indicate they are typedefs (e.g., `error_code_t`, `my_struct_t`).

  - **Enum Values**: Enum values should start with `k_` to distinguish them from other identifiers (e.g., `k_success`, `k_failure`).

  - **Variable and Function Naming**: Use snake_case for variable and function names (e.g., `my_function`, `my_variable`).

  - **Constants**: Constants should always be used where applicable (e.g., `max_length`).

  - **Avoid Abbreviations**: Avoid unnecessary abbreviations (e.g., shortening `data` to `dat`) unless the resulting name would be excessively long. This ensures that names remain clear and understandable.

- **Avoiding Conflicts**: Clear and distinct names reduce the likelihood of naming conflicts, which can lead to bugs or confusion. Names should avoid being too generic (e.g., `temp` or `data`) and instead reflect the specific role of the entity.

- **Contextual Meaning**: Names should reflect the context in which they are used. For instance, a variable holding a user's first name should be called `first_name` instead of something vague like `str1`. This helps other developers quickly grasp the intent of the code.

- **Searchability**: Meaningful names make the codebase easier to search through. Contributors can quickly locate relevant code by searching for terms related to the functionality they are working on.

- **Scalability**: As the project grows, a thoughtful naming convention ensures the code remains understandable. Properly named classes, methods, and variables contribute to a well-structured, scalable codebase that can accommodate new features and improvements without confusion.

Bad Example
-----------

.. code-block:: c

    /* Bad code with inconsistent naming and unclear purpose */

    int  temp;  /* Unclear variable name */
    bool invert;  /* Should be static since it is only used in this file */
    enum Status { OK, ERROR };  /* Enum without suffix and unclear names */

    void doSomething(int a)
    {
      if (a > 0) {
        temp = 10;
      } else {
        temp = -1;
      }
    }

    int foo;  /* Global variable without a clear prefix or name */

    void function(void)
    {
      foo = temp + 1;  /* Vague variable usage */
    }


Issues in the bad example:

- The variable `temp` is too vague, giving no context about what it represents.

- `invert` should be declared as `static` because it's only used in this file.

- Enum values `OK` and `ERROR` lack proper suffixes and are too generic.

- Function name `doSomething` doesn't describe its action.

- The global variable `foo` has no clear purpose or prefix to indicate its scope or function.

---

Good Example
------------

.. code-block:: c

    /* Good code with consistent naming and clarity */

    static bool s_invert;  /* Static variable with s_ prefix to indicate file scope */

    typedef enum status_t {  /* Enum with snake_case and _t suffix */
      k_success,
      k_failure
    } status_t;

    static int s_threshold = 0;  /* Declare static threshold at file scope */

    static void process_input(int input_value)
    {
      /* Clear variable name and static scope for file use */
      if (input_value > 0) {
        s_threshold = 10;
      } else {
        s_threshold = -1;
      }
    }

    static void calculate_output(void)
    {
      int result = 0;  /* Clear, descriptive variable usage */
      result     = s_threshold + 1;
    }


In the good example:

- `s_invert` is declared static with a clear prefix.

- The `status_t` enum uses snake_case and ends with `_t`, with enum values starting with `k_`.

- The function name `process_input` clearly describes its purpose.

- Variables like `s_threshold` and `result` have meaningful names, indicating their roles in the logic.

- No unnecessary global variables are used, and static variables are properly prefixed for clarity.

Pointers
========

Pointers are a powerful tool in C programming, but they can lead to bugs or undefined behavior if used incorrectly. Follow these guidelines for consistent, safe, and efficient use of pointers in this project.

Guidelines
----------

- **Pointer Declaration and Placement**: Always place the asterisk (`*`) adjacent to the type, not the variable name. This improves readability and consistency in the code.

Example 1:
----------

Bad Example:

.. code-block:: c

    int *ptr;  /* INCORRECT */
    char *str;

Good Example:

.. code-block:: c

    int*  ptr;  /* CORRECT */
    char* str;

- **Initialize Pointers**: Always initialize pointers, either to `NULL` or a valid memory location, to avoid using uninitialized pointers, which can lead to undefined behavior.

Example 2:
----------

Bad Example:

.. code-block:: c

    int *ptr;  /* INCORRECT: Uninitialized pointer */
    *ptr = 10;

Good Example:

.. code-block:: c

    int* ptr = NULL;  /* Correct initialization */
    ptr      = malloc(sizeof(int));  /* Or allocate memory */

- **Avoid Dangling Pointers**: A dangling pointer refers to a pointer that continues to reference a memory location after that memory has been freed. Always set a pointer to `NULL` after freeing the allocated memory.

Example 3:
----------

Bad Example:

.. code-block:: c

    int *ptr = malloc(sizeof(int));
    free(ptr);  /* INCORRECT: Dangling pointer */
    /* ptr still points to the now freed memory */

Good Example:

.. code-block:: c

    int* ptr = malloc(sizeof(int));
    free(ptr);
    ptr = NULL;  /* CORRECT: Pointer set to NULL after free */

- **Use `const` When Appropriate**: Use `const` with pointers when the data being pointed to should not be modified. This provides an extra layer of safety and makes the code easier to understand.

Example 4:
----------

.. code-block:: c

    const char* message = "Hello";  /* CORRECT: The pointer data can't be changed */

    char* const ptr = some_buffer;  /* The pointer can't be reassigned */

- **Pointer Arithmetic**: Be cautious with pointer arithmetic. It's only valid for pointers to elements in an array. Misuse of pointer arithmetic can lead to out-of-bounds access and undefined behavior.

Example 5:
----------

Bad Example:

.. code-block:: c

    int  arr[5];
    int *ptr = arr;
    ptr     += 10;  /* INCORRECT: Pointer out of bounds */

Good Example:

.. code-block:: c

    int  arr[5];
    int* ptr = arr;
    ptr     += 2;  /* CORRECT: Pointer remains within bounds */

- **Dereferencing Pointers**: Always ensure a pointer is non-NULL before dereferencing it to avoid segmentation faults or undefined behavior.

Example 6:
----------

Bad Example:

.. code-block:: c

    int *ptr = NULL;
    *ptr     = 42;  /* INCORRECT: Dereferencing NULL pointer */

Good Example:

.. code-block:: c

    if (ptr != NULL) {
      *ptr = 42;  /* CORRECT: Safe dereferencing */
    }

General Guidelines
------------------

- Use `*` next to the variable name when declaring pointers.

- Always initialize pointers to `NULL` or a valid memory location.

- Set pointers to `NULL` after freeing the memory they point to.

- Use `const` to specify when pointer data should not be modified.

- Be cautious with pointer arithmetic and avoid going out of bounds.

- Always check that a pointer is non-NULL before dereferencing it.

Project Structure
=================

In this project, each module should reside in its own directory, containing source files and a separate `include/` folder for header files. The build system will be managed using **CMake** with `CMakeLists.txt` files for defining how each module is built. This structure makes the project modular, scalable, and easy to maintain.

General Guidelines for Project Structure
----------------------------------------

- **Module-Based Structure**: Each module should have its own directory with source files and a dedicated `include/` folder for header files.

- **CMake for Build Management**: The project should use **ESP-IDF CMake** for the build process, with each module having its own `CMakeLists.txt` file to manage its build instructions.

- **Consistent Naming Conventions**: Use consistent naming conventions across the project for directories, files, and functions.

Recommended Directory Structure
-------------------------------

The following is an example of a modular ESP32 project structure:

.. code-block::

    project_root/
    ├── components/
    │   └── pstar_hal/
    │       ├── bh1750/
    │       │   ├── include/          # Header files for bh1750 module
    │       │   │   └── bh1750.h
    │       │   ├── bh1750.c          # Source file for bh1750
    │       │   └── CMakeLists.txt    # CMake file for bh1750 module
    │       └── CMakeLists.txt        # CMake file for pstar_hal module
    ├── main/
    │   ├── main.c                    # Main entry point of the application
    │   └── CMakeLists.txt            # CMake file for main module
    ├── CMakeLists.txt                # Top-level CMake file for the entire project
    └── sdkconfig                     # ESP-IDF SDK configuration file

CMake Build System (ESP-IDF)
----------------------------

The project uses **ESP-IDF CMake** for managing the build process. Each module should have its own `CMakeLists.txt` file that specifies how that module should be built. The top-level `CMakeLists.txt` file in the project root includes all modules and manages the overall build process.

Top-Level `CMakeLists.txt` Example
----------------------------------

In your project, the top-level `CMakeLists.txt` includes the ESP-IDF's project definitions.

.. code-block:: cmake

    cmake_minimum_required(VERSION 3.16)

    include($ENV{IDF_PATH}/tools/cmake/project.cmake)
    project(Topographic-Robot)

`pstar_hal` Module `CMakeLists.txt`
--------------------------------------

The `pstar_hal` module's `CMakeLists.txt` file registers the `bh1750` component within the `components/pstar_hal/` directory. It includes the necessary source files and header directories for the build.

.. code-block:: cmake

    idf_component_register(SRCS "bh1750/bh1750.c"
                           INCLUDE_DIRS "bh1750/include")

`bh1750` Module `CMakeLists.txt`
--------------------------------

The `bh1750` module's `CMakeLists.txt` file directly registers the component using the `idf_component_register()` function.

.. code-block:: cmake

    idf_component_register(SRCS "bh1750.c"
                           INCLUDE_DIRS "include")

Main Module `CMakeLists.txt` Example
------------------------------------

The `main/` module should include the `pstar_hal` module using the `REQUIRES` keyword to declare its dependency on it.

.. code-block:: cmake

    idf_component_register(SRCS "main.c"
                           INCLUDE_DIRS ""
                           REQUIRES pstar_hal)

Naming Conventions
------------------

- **Source and Header Files**: Use `snake_case` for all source (`.c`) and header (`.h`) file names to keep consistent with the project's naming conventions.

- **Directory Names**: Use `snake_case` for directories and make sure that directory names describe the purpose of the module.

Version Control
---------------

- **Use `.gitignore` (or equivalent)**: Ensure that unnecessary files like object files (`.o`), binaries, and the `build/` directory are excluded from version control.

  Example:

  .. code-block::

    # Ignore build files
    build/

    # Ignore object files
    *.o

    # Ignore binaries
    *.exe
    *.out

Why Having a Coding Style is Important
=======================================

A consistent coding style is essential in projects where multiple people contribute for several reasons:

- **Readability**: When everyone follows the same coding style, the code becomes easier to read and understand for everyone involved. This reduces the cognitive load for new contributors and allows developers to focus on functionality rather than deciphering different styles.

- **Maintainability**: A uniform coding style makes the codebase more maintainable over time. As developers come and go, adhering to a consistent style ensures that future contributors can easily pick up and maintain the project without confusion.

- **Collaboration**: With a shared style guide, teams can collaborate more efficiently. Code reviews, pair programming, and debugging become smoother when everyone is on the same page regarding style conventions.

- **Error Prevention**: A well-defined coding style can help reduce errors and bugs. It encourages best practices, such as proper indentation, naming conventions, and clear formatting, which minimizes the risk of mistakes going unnoticed.

- **Automation and Tooling**: Adopting a consistent style allows for better integration with automated tools, such as linters and formatters. These tools can enforce the style automatically, saving time and ensuring that every contribution meets the project's standards.

- **Professionalism**: A consistent coding style demonstrates professionalism and discipline within a development team, which can lead to better project management and clearer communication with stakeholders.

Register Transfer Level (RTL)
=============================

Register Transfer Level (RTL) design focuses on the flow of data between registers and the logic operations performed on that data. It is critical in hardware description languages (HDL) such as Verilog and VHDL, where the code describes the behavior of digital circuits at the register level. In this project, we follow strict RTL guidelines to ensure clarity, reliability, and maintainability of the hardware design.

Guidelines
----------

- **Always Use Non-blocking Assignments for Sequential Logic**: For sequential logic (inside `always` blocks sensitive to a clock edge), use non-blocking assignments (`<=`) to ensure the correct behavior of registers and avoid race conditions.

  Correct Example:

  .. code-block:: verilog

    always @(posedge clk) begin
      if (reset)
        counter <= 0;
      else
        counter <= counter + 1;
    end

  Incorrect Example:

  .. code-block:: verilog

    always @(posedge clk) begin
      if (reset)
        counter = 0;  /* Blocking assignment, incorrect for sequential logic */
      else
        counter = counter + 1;
    end

- **Use Blocking Assignments for Combinational Logic**: For combinational logic, use blocking assignments (`=`) within `always` blocks. This ensures that the logic is evaluated sequentially, and the output reflects the latest input conditions.

  Correct Example:

  .. code-block:: verilog

    always @(*) begin
      result = a + b;  /* Blocking assignment for combinational logic */
    end

  Incorrect Example:

  .. code-block:: verilog

    always @(*) begin
      result <= a + b;  /* Non-blocking assignment in combinational logic */
    end

- **Keep the Clock Domain Clean**: Ensure that logic within one clock domain does not inadvertently interact with another clock domain. Use appropriate synchronizers and clock domain crossing techniques to prevent metastability.

  Example of Clock Domain Crossing:

  .. code-block:: verilog

    always @(posedge clk1 or posedge reset) begin
      if (reset)
        sync_reg <= 0;
      else
        sync_reg <= async_signal;
    end

    always @(posedge clk2) begin
      sync_output <= sync_reg;
    end

- **Use Descriptive Signal Names**: Avoid generic names like `temp`, `signal1`, etc. Choose meaningful signal names that clearly indicate the function and purpose of the signal.

  Correct Example:

  .. code-block:: verilog

    reg [7:0] data_in;   /* Clear and descriptive */
    reg [7:0] data_out;  /* Output of the data */

  Incorrect Example:

  .. code-block:: verilog

    reg [7:0] temp;   /* Ambiguous and unclear */
    reg [7:0] sig1;   /* Non-descriptive */

- **Follow a Consistent Reset Strategy**: Always ensure that reset behavior is consistent across the design. Use synchronous or asynchronous resets depending on the design requirements, and clearly document the reset behavior.

  Correct Example (Synchronous Reset):

  .. code-block:: verilog

    always @(posedge clk) begin
      if (reset)
        data_reg <= 0;
      else
        data_reg <= next_data;
    end

  Incorrect Example:

  .. code-block:: verilog

    always @(posedge clk or posedge reset) begin
      if (reset)
        data_reg <= 0;
      else
        data_reg = next_data;  /* Incorrect use of blocking assignment */
    end

- **Document RTL Code Clearly**: Each module should include a comment block explaining its functionality, inputs, and outputs. Key design choices, such as pipeline stages or timing constraints, should be documented as well.

RTL Design Examples
-------------------

Correct Example (Sequential Logic with Non-blocking Assignments):

.. code-block:: verilog

  module counter (
      input  wire clk,
      input  wire reset,
      output reg  [3:0] count
  );

  always @(posedge clk or posedge reset) begin
    if (reset)
      count <= 0;
    else
      count <= count + 1;
  end

  endmodule

Incorrect Example (Blocking Assignment in Sequential Logic):

.. code-block:: verilog

  module counter (
      input  wire clk,
      input  wire reset,
      output reg  [3:0] count
  );

  always @(posedge clk or posedge reset) begin
    if (reset)
      count = 0;  /* Incorrect use of blocking assignment */
    else
      count = count + 1;
  end

  endmodule

General Guidelines
------------------

- **Use non-blocking assignments (`<=`) for sequential logic** inside clocked processes.

- **Use blocking assignments (`=`) for combinational logic**.

- **Ensure proper clock domain crossings** using synchronizers.

- **Name signals descriptively** to improve code readability and maintainability.

- **Keep reset strategy consistent** across the design.

- **Document modules and key decisions** within the RTL code.

Static and Const
================

Using `static` and `const` effectively can improve code safety, performance, and readability. As a general rule, always use `static` and `const` when possible.

Using `const` over Macros
-------------------------

Using `const` globals is generally better than using macros for several reasons:

1. **Type Safety**: `const` variables have a defined type, so the compiler can catch type-related errors, whereas macros are simple text replacements without type checking.

2. **Scoping**: `const` globals respect C scoping rules, whereas macros are globally visible after being defined and can cause naming conflicts or unexpected behavior.

3. **Debugging**: `const` variables appear in debugging information, while macros do not, making it harder to trace issues with macros.

4. **Better readability**: `const` variables make the code clearer and more maintainable, as their intent is explicit compared to macros, which can be obscure.

For most situations, it's preferable to use `const` globals, unless you need text substitution or preprocessor directives, in which case macros might be more appropriate.

Example 1:
----------

Bad Example:

.. code-block:: c

    #define MAX_SIZE 100 /* INCORRECT, using a macro for a constant */

Good Example:

.. code-block:: c

    const int max_size = 100; /* Correct, using a const global */

Using `static` for Internal Linkage
-----------------------------------

- **`static` Variables**: Variables and functions that are only used within a single source file should be declared `static`. This ensures internal linkage and avoids accidental linkage in other parts of the project.

Example 2:
----------

Bad Example:

.. code-block:: c

    int internal_counter = 0; /* INCORRECT, lacks static for internal use */

    void increment_counter(void) {
      internal_counter++;
    }

Good Example:

.. code-block:: c

    static int s_internal_counter = 0; /* Correct, using static for internal use */

    static void increment_counter(void) {
      s_internal_counter++;
    }

**General Guidelines**:

- Always prefer `const` to macros when declaring constants.

- Always use `static` for variables and functions that are only used within a single source file to limit scope and avoid name collisions.

Structures
==========

Structures (structs) in this project should be used to group related data together in a clean and organized manner. Proper use of structures enhances code readability, maintainability, and provides an efficient way to represent complex data. Follow these guidelines for struct usage and naming conventions.

General Guidelines for Structures
----------------------------------

- **Use Structs to Group Related Data**: Structs should be used to encapsulate related variables into a single entity, rather than passing around or managing multiple separate variables.

  Example:

  .. code-block:: c

    typedef struct {
      int x;
      int y;
    } point_t;

- **Struct Names Should End in `_t`**: All struct types should end with `_t` to indicate that they are typedefs, ensuring consistency across the project.

  Example:

  .. code-block:: c

    typedef struct {
      int width;
      int height;
    } rectangle_t;

- **Use Snake Case for Struct Members**: All member variables inside a struct should follow snake_case naming conventions. This ensures uniformity across the project and improves readability.

  Example:

  .. code-block:: c

    typedef struct {
      int x_position;
      int y_position;
    } position_t;

- **Always Document Each Member of the Struct**: Every struct member should be clearly documented with a comment, providing a short description of its purpose or role within the structure.

  Example:

  .. code-block:: c

    /**
     * @brief Struct representing a 2D point.
     */
    typedef struct {
      int x_position; /**< X-coordinate of the point */
      int y_position; /**< Y-coordinate of the point */
    } point_t;

- **Use Pointers for Large Structs**: When passing large structs around, use pointers instead of passing by value. This avoids the overhead of copying the entire struct and improves performance.

  Example:

  .. code-block:: c

    void move_point(point_t* p) {
      p->x_position += 1;
      p->y_position += 1;
    }

- **Initialize Structs to Zero or Default Values**: Always ensure that structs are initialized properly, either with zero or appropriate default values. This avoids potential bugs caused by uninitialized data.

  Example:

  .. code-block:: c

    point_t p = {0}; /* Initialize struct to zero */

  Or:

  .. code-block:: c

    point_t p = { .x_position = 0, .y_position = 0 }; /* Initialize with default values */

- **Use `const` with Struct Pointers When Data Shouldn't Change**: If a function should not modify the data inside a struct, mark the pointer as `const` to enforce this behavior.

  Example:

  .. code-block:: c

    void print_point(const point_t* p) {
      printf("X: %d, Y: %d\n", p->x_position, p->y_position);
    }

Struct Alignment and Padding
----------------------------

- **Minimize Padding and Align Struct Members**: Struct members should be ordered to minimize padding and ensure proper alignment. This improves performance and reduces memory overhead.

  Example:

  .. code-block:: c

    typedef struct {
      int  a; /* Aligns with the int size */
      char b; /* Padding added here */
    } aligned_struct_t;

  Rearranged for better alignment:

  .. code-block:: c

    typedef struct {
      char b; /* Now placed before the int */
      int  a; /* Padding removed */
    } aligned_struct_t;

- **Consider Memory Layout for Large Structs**: For larger structs, consider the memory layout and how the data will be accessed. Group similar data types together to minimize cache misses and improve access speed.

Encapsulation and Struct Access
-------------------------------

- **Use Accessor Functions to Modify Struct Members**: When appropriate, use getter and setter functions to modify struct members. This allows for better encapsulation and flexibility, especially if the struct implementation changes over time.

  Example:

  .. code-block:: c

    void set_point_x(point_t* p, int x) {
      p->x_position = x;
    }

    int get_point_x(const point_t* p) {
      return p->x_position;
    }

- **Avoid Exposing Internal Structs in Public APIs**: In public APIs, avoid directly exposing the internal structure of a struct. Instead, use opaque types or provide accessor functions to interact with the struct, ensuring that internal implementation details can change without breaking API contracts.

  Example:

  .. code-block:: c

    /* Private definition of struct */
    typedef struct point_t point_t;

    /* Accessor functions */
    point_t* create_point(int x, int y);
    void destroy_point(point_t* p);
    int get_point_x(const point_t* p);

Naming Conventions
------------------

- **Use Meaningful Names**: Struct names should be descriptive and indicate the type of data they represent.

- **Follow Snake Case for Struct Members**: All struct members should use snake_case to maintain consistency.

- **Prefix with Module Name for Shared Structs**: When a struct is shared between multiple modules, prefix its name with the module's name to avoid naming collisions.

Example 1:
----------

Bad Example:

.. code-block:: c

    typedef struct {
      int width;
      int height;
    } rect; /* INCORRECT: Non-descriptive name, does not end with _t */

Good Example:

.. code-block:: c

    typedef struct {
      int width;
      int height;
    } rectangle_t; /* CORRECT: Descriptive name and follows naming convention */

Example 2:
----------

Bad Example:

.. code-block:: c

    typedef struct {
      int x;
      int y;
    } position_t; /* INCORRECT: Poor documentation and lacks meaningful member names */

Good Example:

.. code-block:: c

    /**
     * @brief 2D position in space.
     */
    typedef struct {
      int x_position; /**< X-coordinate */
      int y_position; /**< Y-coordinate */
    } position_t;     /* CORRECT: Proper documentation and meaningful member names */

General Guidelines
------------------

- Always end struct names with `_t`.

- Use snake_case for struct members.

- Document each struct member for clarity and future maintenance.

- Minimize struct padding by ordering members based on their size.

- Use pointers for large structs when passing them as function arguments.

- Consider using accessor functions for encapsulation.

Type Definitions
================

Using `typedef` allows for more readable and maintainable code by creating meaningful aliases for types. Follow these guidelines when creating type definitions in this project.

Guidelines
----------

- **Use `_t` Suffix for `typedef` Types**: All user-defined types created with `typedef` must end with `_t`. This is a common convention in C to distinguish type names from variables and functions.

Example 1:
----------

Bad Example:

.. code-block:: c

    typedef struct {
      int x;
      int y;
    } Point; /* INCORRECT: Missing '_t' suffix */

Good Example:

.. code-block:: c

    typedef struct {
      int x;
      int y;
    } point_t; /* CORRECT: '_t' suffix added */

- **Use Meaningful Type Names**: Always use descriptive and meaningful names for `typedef` types to make the code more readable and easier to understand.

Example 2:
----------

Bad Example:

.. code-block:: c

    typedef struct {
      int width;
      int height;
    } W_H; /* INCORRECT: Non-descriptive type name */

Good Example:

.. code-block:: c

    typedef struct {
      int width;
      int height;
    } dimensions_t; /* CORRECT: Descriptive type name */

- **Use `typedef` to Simplify Complex Types**: Use `typedef` to simplify the declaration of complex types, such as function pointers, or to make the code more consistent and readable.

Example 3:
----------

Bad Example:

.. code-block:: c

    int (*compare)(const void *, const void *); /* INCORRECT: Hard to read */

Good Example:

.. code-block:: c

    typedef int (*compare_fn_t)(const void *, const void *); /* CORRECT: Simplified with typedef */

- **Do Not Overuse `typedef`**: Avoid using `typedef` for basic types like `int`, `char`, or `float`, as this can make the code harder to follow and debug. Use `typedef` only when it improves the clarity of the code.

Example 4:
----------

Bad Example:

.. code-block:: c

    typedef int my_int; /* INCORRECT: Overuse of typedef */

Good Example:

.. code-block:: c

    /* No typedef needed for basic types like int */

- **Struct and Enum Typedefs**: When defining structs or enums, always use `typedef` and the `_t` suffix to create a new type for ease of use. This ensures consistency across the codebase and simplifies the declaration of variables.

Example 5:
----------

Bad Example:

.. code-block:: c

    struct person {
      char* name;
      int   age;
    };

    struct person john; /* INCORRECT: Repeated 'struct' keyword */

Good Example:

.. code-block:: c

    typedef struct {
      char* name;
      int   age;
    } person_t;

    person_t john; /* CORRECT: Typedef used to avoid repeated 'struct' keyword */

- **Enum Type Definitions**: Always use `typedef` with enums, and follow the same convention of appending `_t` to the type name. Enum values should be in `snake_case` and starting with `k_`.

Example 6:
----------

Bad Example:

.. code-block:: c

    enum color {
      RED,
      BLUE,
      GREEN
    };

Good Example:

.. code-block:: c

    typedef enum {
      k_red,
      k_blue,
      k_green
    } color_t;

General Guidelines
------------------

- Always use the `_t` suffix for all `typedef` types to distinguish them from variables and functions.

- Use `typedef` to simplify complex types, such as structs, enums, and function pointers.

- Avoid using `typedef` for basic types like `int`, `float`, or `char`.

- Always create meaningful and descriptive names for `typedef` types.

- Follow consistent naming conventions for `struct`, `enum`, and function pointer type definitions.

Types
=====

In C programming, it's essential to choose the correct data types for variables and function parameters to ensure clarity, efficiency, and portability. Whenever possible, use fixed-width types from `stdint.h`, such as `uint8_t`, `uint32_t`, `int16_t`, and `int_fast8_t`. These types guarantee the size of the data and improve cross-platform compatibility.

General Guidelines for Types
----------------------------

- **Use Fixed-Width Integer Types**: Always use specific integer types like `uint8_t`, `int16_t`, `uint32_t`, etc., rather than using `int`, `long`, or other platform-dependent types. Fixed-width types provide clarity and ensure that the size of the variable is consistent across platforms.

  Example:

  .. code-block:: c

    uint8_t counter = 0;  /* Preferred over "int" */

- **Use `int_fastN_t` for Performance**: When performance is critical and you don't need a specific width, use `int_fast8_t`, `int_fast16_t`, etc. These types are at least as large as the number specified, but may be faster on certain platforms.

  Example:

  .. code-block:: c

    int_fast16_t sum = 0;

- **Use `size_t` for Sizes and Memory-Related Variables**: Use `size_t` for variables that hold sizes, memory-related values, or counts of objects. This type ensures portability and correctness when dealing with memory and sizes.

  Example:

  .. code-block:: c

    size_t buffer_size = 1024;

- **Avoid Platform-Dependent Types**: Avoid using platform-dependent types such as `long`, `short`, `int`, or `char` unless absolutely necessary. Instead, use `int8_t`, `int16_t`, `uint32_t`, and similar types to ensure cross-platform behavior.

  Example:

  .. code-block:: c

    int16_t temperature = 25;  /* Preferred over "short" */

- **Use `bool` for Boolean Values**: When representing boolean values, use `bool` (from `stdbool.h`). Avoid using integers for this purpose, as `bool` makes the code more readable and self-explanatory.

  Example:

  .. code-block:: c

    bool is_valid = true;  /* Preferred over "int is_valid = 1" */

- **Use Typedefs for Complex Data Structures**: When dealing with structures, unions, or other complex data types, use `typedef` to give them meaningful names, making the code cleaner and easier to read.

  Example:

  .. code-block:: c

    typedef struct {
      uint8_t  id;
      uint16_t age;
      bool     is_active;
    } user_info_t;

Example 1:
----------

Bad Example:

.. code-block:: c

    int count = 0; /* INCORRECT: Use of "int" instead of fixed-width type */

Good Example:

.. code-block:: c

    uint8_t count = 0; /* CORRECT: Fixed-width type used */

Example 2:
----------

Bad Example:

.. code-block:: c

    long buffer_size = 1024; /* INCORRECT: "long" type is platform-dependent */

Good Example:

.. code-block:: c

    size_t buffer_size = 1024; /* CORRECT: Using "size_t" for memory size */

Example 3:
----------

Bad Example:

.. code-block:: c

    int is_active = 1; /* INCORRECT: Using "int" for a boolean value */

Good Example:

.. code-block:: c

    bool is_active = true; /* CORRECT: Using "bool" for boolean values */

General Guidelines
------------------

- Always use fixed-width integer types (`uint8_t`, `int32_t`, etc.) for clarity and portability.

- Use `int_fastN_t` for performance when a specific size isn't required but speed is essential.

- Use `size_t` for variables related to memory sizes or object counts.

- Avoid using platform-dependent types like `long`, `short`, or `int`.

- Use `bool` for boolean values instead of integers.

- Use `typedef` to simplify complex data structures and make them easier to work with.

Vertical Space
==============

Proper vertical space usage makes the code easier to read by clearly separating functions and logical blocks of code. Follow these guidelines:

- **One Empty Line Between Functions**: Always place one empty line between function definitions to improve readability. This helps visually distinguish where one function ends and the next begins.

- **No Empty Lines at the Beginning or End of Functions**: Do not place empty lines at the start or end of a function body. The opening and closing braces should be immediately adjacent to the function code.

Bad Example
-----------

.. code-block:: c

  void function1(void)
  {

    do_one_thing();
    do_another_thing();

  } /* INCORRECT, do not place an empty line here */


  void function2(void)
  {

    int var = 0;
    while (var < c_some_constant) { /* Correct constant naming */
      do_stuff(&var);
    }

  } /* INCORRECT, do not use an empty line here */

---

Good Example
------------

.. code-block:: c

  void function1(void)
  {
    do_one_thing();
    do_another_thing();
  }

  /* Place one empty line between functions */

  void function2(void)
  {
    int var = 0;
    while (var < c_some_constant) { /* Correct constant naming */
      do_stuff(&var);
    }
  }

**Line Length**: The maximum line length is **80** characters, as long as it does not seriously affect the readability of the code.

