#include <stdio.h>
#include <string.h>

/* Global variables for observing memory and variable values */
int global_counter = 0;
const char* global_message = "Debug Test";

/* Function declarations */
void print_message(const char* msg, int count);
int calculate_factorial(int n);
int fibonacci(int n);
void modify_global_variables(void);
void string_operations(void);

int main(void)
{
    printf("Hello World! %s\n", CONFIG_BOARD_TARGET);
    
    /* Test various functions for setting breakpoints */
    printf("\n=== Starting Debug Tests ===\n");
    
    /* 1. Test basic function calls - good for breakpoints */
    print_message("First function call", 1);
    
    /* 2. Test recursive function - observe call stack */
    int fact_result = calculate_factorial(5);
    printf("Factorial of 5 = %d\n", fact_result);
    
    /* 3. Test another recursive function */
    int fib_result = fibonacci(6);
    printf("6th Fibonacci number = %d\n", fib_result);
    
    /* 4. Test global variable modification */
    modify_global_variables();
    printf("Global counter: %d\n", global_counter);
    
    /* 5. Test string operations */
    string_operations();
    
    printf("\n=== Debug Tests Completed ===\n");
    
    return 0;
}

/* Simple print function for testing function calls and parameter passing */
void print_message(const char* msg, int count)
{
    printf("Message[%d]: %s\n", count, msg);
    global_counter += count;
}

/* Recursive factorial function for testing call stack */
int calculate_factorial(int n)
{
    if (n <= 1) {
        return 1;  /* Set breakpoint here to observe recursion termination */
    }
    return n * calculate_factorial(n - 1);
}

/* Fibonacci sequence calculation - different recursion pattern */
int fibonacci(int n)
{
    if (n <= 1) {
        return n;
    }
    return fibonacci(n - 1) + fibonacci(n - 2);
}

/* Function to modify global variables for observing variable changes */
void modify_global_variables(void)
{
    static int static_counter = 0;  /* Static variable to observe persistence */
    
    global_counter += 10;
    static_counter += 5;
    
    printf("Modified globals - Global counter: %d, Static counter: %d\n", 
           global_counter, static_counter);
}

/* String operations function for testing memory examination */
void string_operations(void)
{
    char buffer[50];
    int numbers[] = {1, 2, 3, 4, 5};
    
    /* String operations */
    strcpy(buffer, "Test string: ");
    strcat(buffer, global_message);
    
    printf("String result: %s\n", buffer);
    printf("Array contents: ");
    
    /* Array operations */
    for (int i = 0; i < 5; i++) {
        printf("%d ", numbers[i]);
    }
    printf("\n");
    
    /* Pointer operations */
    char* ptr = buffer + 5;
    printf("Value at pointer position: '%c'\n", *ptr);
}
