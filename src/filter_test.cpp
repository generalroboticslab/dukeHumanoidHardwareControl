#include <iostream>
#include "Iir.h"  // Include the iir1 library header

int main() {
    const int order = 4; // 4th order (=2 biquads)
    // Create an array of 5 lowpass filters
    Iir::Butterworth::LowPass<order> lowpassFilters[5]; 

    // Set different cutoff frequencies for each filter
    double cutoffFrequencies[] = {10.0, 50.0, 100.0, 200.0, 250.0};
    for (int i = 0; i < 5; ++i) {
        lowpassFilters[i].setup(1000, cutoffFrequencies[i]); // Sample rate 44.1 kHz
    }

    // Process audio samples (example)
    double inputSample = 0.5; 
    for (int k = 0; k < 1000; ++k) {
        for (int i = 0; i < 5; ++i) {
            double filteredSample = lowpassFilters[i].filter(inputSample);
            printf("%3.5f\t", filteredSample);
        }
        std::cout << std::endl;
    }
    return 0;
}
