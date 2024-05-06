#include <stdio.h>
#include <stdlib.h>

#define ROWS 4
#define COLS 4

typedef struct {
    int *values;
    int *colIndices;
    int *rowLengths;
    int nonZeros;
} CISRMatrix;

// Function to convert dense matrix to CISR format
CISRMatrix denseToCISR(int denseMatrix[ROWS][COLS], int rows, int cols) {
    int count = 0;
    int rownum[4] = {0, 0, 0, 0};
    // First pass to count non-zero elements
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            if (denseMatrix[i][j] != 0) {
                count++;
                rownum[i]++;
            }
        }
    }

    // Allocate memory for CISR structures based on non-zero count
    CISRMatrix cisr;
    cisr.values = (int *)malloc(count * sizeof(int));
    cisr.colIndices = (int *)malloc(count * sizeof(int));
    cisr.rowLengths = (int *)malloc(count * sizeof(int));
    cisr.nonZeros = count;

    // Second pass to fill the structures
    int index = 0;
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            if (denseMatrix[i][j] != 0) {
                cisr.values[index] = denseMatrix[i][j];
                cisr.colIndices[index] = j;
                cisr.rowLengths[index] = rownum[i];
                index++;
            }
        }
    }

    return cisr;
}

// Function to print the CISR matrix
void printCISR(CISRMatrix cisr) {
    printf("Values: ");
    for (int i = 0; i < cisr.nonZeros; i++) {
        printf("%d ", cisr.values[i]);
    }
    printf("\nColumn Indices: ");
    for (int i = 0; i < cisr.nonZeros; i++) {
        printf("%d ", cisr.colIndices[i]);
    }
    printf("\nRow Lengths: ");
    for (int i = 0; i < cisr.nonZeros; i++) {
        printf("%d ", cisr.rowLengths[i]);
    }    
    printf("\n");
}

int main() {
    int denseMatrix[ROWS][COLS] = {
        {1, 0, 0, 0},
        {0, 2, 0, 3},
        {4, 0, 5, 0},
        {0, 0, 0, 6}
    };

    CISRMatrix cisr = denseToCISR(denseMatrix, ROWS, COLS);
    printCISR(cisr);

    // Free the allocated memory
    free(cisr.values);
    free(cisr.colIndices);
    free(cisr.rowLengths);
    return 0;
}
